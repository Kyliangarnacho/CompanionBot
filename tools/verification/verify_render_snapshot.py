"""Finite official MuJoCo state snapshot / private-renderer equivalence check.

No controller/estimator/actuator consumer receives this state. A frozen model
and explicit integration snapshots feed only private MjData, mj_forward and RGB.
The source physics owner never shares mutable MjData with the render worker.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import platform
import sys
import threading
import time
import traceback

import cv2
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from companionbot_contracts.paths import repository_root
from companionbot_contracts.spatial_geometry import load_spatial_geometry
from companionbot_simulation.core.retail_scene import prepare_retail_model
from companionbot_simulation.core.visual_companion_scene import (
    prepare_visual_companion_model,VisualCompanionActor,OnboardRGBRenderer)

ROOT=repository_root()
STATE_SPEC=mujoco.mjtState.mjSTATE_INTEGRATION
CAMERA=ROOT/'models/minisegway/native_runtime/config/visual_camera_640_fast.json'
ASSET=ROOT/'.venv/assets/visual_companion_gazebo/derived'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def capture_state(model,data):
    state=np.empty(mujoco.mj_stateSize(model,STATE_SPEC),dtype=np.float64)
    mujoco.mj_getState(model,data,state,STATE_SPEC)
    return state


def maximum_delta(left,right):
    return float(np.max(np.abs(left-right))) if np.asarray(left).size else 0.


def _render_worker(model,camera_path,jobs):
    """One worker owns its data and all GL creation/use/close operations."""
    renderer=OnboardRGBRenderer(model,camera_config=camera_path)
    private=mujoco.MjData(model);rows=[]
    try:
        for job in jobs:
            snapshot=job['snapshot'];original=snapshot.copy()
            started=time.perf_counter()
            # 3.2's Eigen binding requires writeable memory; this is a worker-
            # private buffer, never a revived shared producer-owned buffer.
            local=np.array(snapshot,dtype=np.float64,copy=True,order='C')
            mujoco.mj_setState(model,private,local,STATE_SPEC)
            exact_before_forward=np.array_equal(capture_state(model,private),snapshot)
            mujoco.mj_forward(model,private)
            restored=time.perf_counter()
            image=renderer.render_bgr(private)
            rows.append(dict(index=job['index'],thread_native_id=threading.get_native_id(),
                input_snapshot_unchanged=bool(np.array_equal(snapshot,original)),
                exact_integration_state_before_forward=bool(exact_before_forward),
                restore_and_forward_wall_s=restored-started,render_wall_s=time.perf_counter()-restored,
                time_s=float(private.time),qpos=private.qpos.copy(),qvel=private.qvel.copy(),
                mocap_pos=private.mocap_pos.copy(),mocap_quat=private.mocap_quat.copy(),
                xpos=private.xpos.copy(),xmat=private.xmat.copy(),cam_xpos=private.cam_xpos.copy(),
                cam_xmat=private.cam_xmat.copy(),image=image))
    finally:renderer.close()
    return rows


def run(output):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    if (output/'summary.json').exists():raise FileExistsError('preserve_snapshot_evidence_choose_new_output')
    protocol=dict(schema_version=1,state_spec=int(STATE_SPEC),
        body_position_max_abs_error_m=1e-12,rotation_matrix_max_abs_error=1e-12,
        integration_fields_exact=True,time_advanced_by_render=False,rgb_max_absolute_difference=0,
        scope='same fixed model and integration snapshot; same EGL profile/context backend; no model inference',
        no_physics_in_renderer=True,no_snapshot_to_controller=True,hardware_execution_ready=False)
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    report=dict(passed=False,python=sys.executable,version=sys.version,platform=platform.platform(),
        mujoco_version=mujoco.__version__,source_sha256=sha(__file__),protocol=protocol,
        host_started_s=time.perf_counter(),hardware_execution_ready=False)
    renderer=None
    try:
        base=prepare_retail_model(ROOT/'models/minisegway/mini_segway.xml',output/'bookstore.xml')
        path=prepare_visual_companion_model(base,output/'visual.xml',asset_dir=ASSET,camera_config=CAMERA)
        model=mujoco.MjModel.from_xml_path(str(path));producer=mujoco.MjData(model)
        actor=VisualCompanionActor(model,asset_dir=ASSET);geometry=load_spatial_geometry()
        renderer=OnboardRGBRenderer(model,camera_config=CAMERA)
        report.update(model_sha256=sha(path),camera_sha256=sha(CAMERA),state_size=mujoco.mj_stateSize(model,STATE_SPEC),
            state_bytes=8*mujoco.mj_stateSize(model,STATE_SPEC),producer_thread_native_id=threading.get_native_id(),
            mj_copyData_python_exported=hasattr(mujoco,'mj_copyData'))
        cases=[dict(pitch=0.,yaw=0.,time=0.,actor=[1.8,0.,0.],integrate=False),
               dict(pitch=.12,yaw=.2,time=.5,actor=[1.8,.2,0.],integrate=False),
               dict(pitch=-.10,yaw=-.3,time=1.,actor=[1.5,-.3,0.],integrate=False),
               dict(pitch=.02,yaw=.1,time=1.5,actor=[2.1,0.,0.],integrate=True)]
        jobs=[];references=[];metadata=[]
        for index,case in enumerate(cases):
            mujoco.mj_resetDataKeyframe(model,producer,0)
            rotation=geometry.mj_to_ros.T@Rotation.from_euler('zy',[case['yaw'],case['pitch']]).as_matrix()@geometry.mj_to_ros
            quat=Rotation.from_matrix(rotation).as_quat();producer.qpos[3:7]=[quat[3],*quat[:3]]
            producer.time=case['time'];producer.qvel[:]=np.linspace(-.01,.02,model.nv)
            producer.ctrl[:]=[.02,-.01]
            actor.set_pose(producer,case['time']+.25,case['actor'],np.pi)
            mujoco.mj_forward(model,producer)
            if case['integrate']:
                for _ in range(20):mujoco.mj_step(model,producer)
            capture_started=time.perf_counter();snapshot=capture_state(model,producer)
            capture_wall=time.perf_counter()-capture_started
            # Record the old direct-data image separately: after mj_step its
            # derived xpos can lag the integrated qpos. It is not the oracle.
            direct_xpos=producer.xpos.copy()
            direct_image=renderer.render_bgr(producer) if case['integrate'] else None
            # Offline reference only: make all derived poses canonical for this
            # exact snapshot. Production must NOT call this on physics MjData.
            mujoco.mj_forward(model,producer)
            references.append(dict(time_s=float(producer.time),qpos=producer.qpos.copy(),qvel=producer.qvel.copy(),
                mocap_pos=producer.mocap_pos.copy(),mocap_quat=producer.mocap_quat.copy(),
                xpos=producer.xpos.copy(),xmat=producer.xmat.copy(),cam_xpos=producer.cam_xpos.copy(),
                cam_xmat=producer.cam_xmat.copy(),image=renderer.render_bgr(producer)))
            meta=dict(index=index,case=case,snapshot_sha256=hashlib.sha256(snapshot.tobytes()).hexdigest(),
                source_time_s=float(snapshot[0]),capture_state_wall_s=capture_wall,
                direct_data_vs_canonical_xpos_delta_m=maximum_delta(direct_xpos,producer.xpos))
            if direct_image is not None:
                meta['direct_data_vs_canonical_rgb_max_delta']=int(np.max(np.abs(direct_image.astype(int)-references[-1]['image'].astype(int))))
                cv2.imwrite(str(output/f'{index}_direct_after_step.png'),direct_image)
            snapshot.setflags(write=False);jobs.append(dict(index=index,snapshot=snapshot));metadata.append(meta)
            np.savez_compressed(output/f'{index}_state.npz',integration_state=snapshot)
            cv2.imwrite(str(output/f'{index}_reference.png'),references[-1]['image'])
        # The producer can now mutate freely. A queued snapshot must not alias it.
        immutable_copy=[j['snapshot'].copy() for j in jobs]
        producer.qpos[0]+=.1;producer.time+=.5
        report['snapshot_independent_of_mutated_source']=all(np.array_equal(j['snapshot'],before)for j,before in zip(jobs,immutable_copy))
        # Preserve the actual binding rejection; don't claim mj_setState accepts
        # readonly buffers simply because its C signature is const.
        try:mujoco.mj_setState(model,mujoco.MjData(model),jobs[0]['snapshot'],STATE_SPEC)
        except TypeError as error:report['readonly_binding']=dict(accepted=False,error=str(error).split('\n')[0])
        else:report['readonly_binding']=dict(accepted=True)
        with ThreadPoolExecutor(max_workers=1,thread_name_prefix='private-render-validation') as pool:
            restored=pool.submit(_render_worker,model,CAMERA,jobs).result(timeout=30.)
        checks=[]
        for reference,actual,meta in zip(references,restored,metadata):
            fields={name:maximum_delta(reference[name],actual[name]) for name in
                ('qpos','qvel','mocap_pos','mocap_quat','xpos','xmat','cam_xpos','cam_xmat')}
            rgb_delta=int(np.max(np.abs(reference['image'].astype(int)-actual['image'].astype(int))))
            passed=(actual['exact_integration_state_before_forward'] and actual['input_snapshot_unchanged']
                and actual['time_s']==reference['time_s']
                and all(fields[name]==0 for name in ('qpos','qvel','mocap_pos','mocap_quat'))
                and fields['xpos']<=1e-12 and fields['cam_xpos']<=1e-12
                and fields['xmat']<=1e-12 and fields['cam_xmat']<=1e-12 and rgb_delta==0)
            meta.update(passed=passed,maximum_field_errors=fields,rgb_max_absolute_difference=rgb_delta,
                worker_thread_native_id=actual['thread_native_id'],render_time_not_advanced=actual['time_s']==reference['time_s'],
                restore_and_forward_wall_s=actual['restore_and_forward_wall_s'],render_wall_s=actual['render_wall_s'])
            checks.append(meta);cv2.imwrite(str(output/f"{meta['index']}_private_worker.png"),actual['image'])
        report.update(cases=checks,passed=all(c['passed'] for c in checks) and report['snapshot_independent_of_mutated_source'])
    except BaseException as error:
        report.update(error=repr(error),traceback=traceback.format_exc());raise
    finally:
        if renderer is not None:renderer.close()
        report['host_ended_s']=time.perf_counter();(output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']:raise SystemExit(1)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output-dir',type=Path,required=True)
    run(parser.parse_args().output_dir)


if __name__=='__main__':main()
