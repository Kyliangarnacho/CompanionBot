"""Finite actual two-camera rendering and source-contract diagnostic, no control.

Both BGR images come from distinct body-attached MuJoCo cameras applied to one
private snapshot. Geometric truth below is a post-hoc camera construction check,
never a matcher input. Optional fixed-grid AI/stereo evaluation keeps renderer
depth in a separate one-way scoring pass; it has no control interface.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import resource
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
from companionbot_simulation.core.visual_companion_scene import VisualCompanionActor
from companionbot_simulation.core.stereo_companion_scene import (
    STEREO_CAMERA_CONFIG,load_stereo_camera,prepare_stereo_companion_model,StereoRGBRenderer)

ROOT=repository_root()
PROTOCOL=ROOT/'models/minisegway/native_runtime/config/stereo_depth_protocol.json'
SPEC=int(mujoco.mjtState.mjSTATE_INTEGRATION)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def state(model,data):
    output=np.empty(mujoco.mj_stateSize(model,SPEC));mujoco.mj_getState(model,data,output,SPEC);return output


def render_worker(model,jobs,config_path,output):
    data=mujoco.MjData(model);renderer=StereoRGBRenderer(model,camera_config=config_path)
    config=renderer.config;rows=[];negative={}
    try:
        from OpenGL import GL
        driver={name:GL.glGetString(code).decode()for name,code in
            [('vendor',GL.GL_VENDOR),('renderer',GL.GL_RENDERER),('version',GL.GL_VERSION)]}
        for index,snapshot in enumerate(jobs):
            mujoco.mj_setState(model,data,np.array(snapshot,copy=True),SPEC);mujoco.mj_forward(model,data)
            before=state(model,data);callback=[]
            left,right=renderer.render_bgr(data,on_left=lambda image,ready:callback.append(
                dict(read_complete_host_s=ready,called_host_s=time.perf_counter(),shape=list(image.shape))))
            after=state(model,data)
            lid,rid=model.camera(config['name']).id,model.camera(config['right_camera']['name']).id
            lrot=data.cam_xmat[lid].reshape(3,3)@np.diag([1.,-1.,-1.])
            rrot=data.cam_xmat[rid].reshape(3,3)@np.diag([1.,-1.,-1.])
            actual_r=rrot.T@lrot;actual_t=rrot.T@(data.cam_xpos[lid]-data.cam_xpos[rid])
            fx,fy,cx,cy=config['K'][0][0],config['K'][1][1],config['K'][0][2],config['K'][1][2]
            errors=[]
            for z in (1.1,1.5,2.1,2.7,3.):
                point=np.array([.1,.2,z]);other=actual_r@point+actual_t
                uv=np.array([fx*point[0]/point[2]+cx,fy*point[1]/point[2]+cy])
                uv_right=np.array([fx*other[0]/other[2]+cx,fy*other[1]/other[2]+cy])
                errors.append(dict(z_m=z,vertical_error_px=float(abs(uv[1]-uv_right[1])),
                    disparity_error_px=float(abs((uv[0]-uv_right[0])-fx*config['baseline_m']/z))))
            timing=dict(renderer.last_timing)
            passed=(np.array_equal(before,after) and left.shape==right.shape==(config['height'],config['width'],3)
                and left.dtype==right.dtype==np.uint8 and bool(np.any(left!=right))
                and np.allclose(actual_r,np.eye(3),rtol=0,atol=1e-12)
                and np.allclose(actual_t,[-config['baseline_m'],0,0],rtol=0,atol=1e-12)
                and callback[0]['read_complete_host_s']<=callback[0]['called_host_s']<=timing['right_render_begin_host_s']
                and all(e['vertical_error_px']<=1e-10 and e['disparity_error_px']<=1e-10 for e in errors))
            cv2.imwrite(str(output/f'{index}_left.png'),left);cv2.imwrite(str(output/f'{index}_right.png'),right)
            rows.append(dict(index=index,passed=passed,worker_native_tid=threading.get_native_id(),
                source_snapshot_sha256=hashlib.sha256(snapshot.tobytes()).hexdigest(),
                integration_state_unchanged=True,images_are_independent=bool(np.any(left!=right)),
                actual_right_from_left_rotation=actual_r.tolist(),actual_right_from_left_translation_m=actual_t.tolist(),
                geometry_projection_checks=errors,on_left=callback[0],timing=timing))
        try:renderer.render_right_bgr(data)
        except RuntimeError as error:negative['right_without_left']=str(error)
        renderer.render_left_bgr(data);data.qpos[0]+=.001
        try:renderer.render_right_bgr(data)
        except RuntimeError as error:negative['changed_snapshot']=str(error)
        with ThreadPoolExecutor(max_workers=1)as wrong:
            try:wrong.submit(renderer.render_left_bgr,data).result()
            except RuntimeError as error:negative['wrong_gl_owner_thread']=str(error)
    finally:renderer.close()
    negative_passed=(negative.get('right_without_left')=='stereo_right_without_left_snapshot'
        and negative.get('changed_snapshot')=='stereo_left_right_snapshot_changed'
        and negative.get('wrong_gl_owner_thread')=='stereo_gl_owner_thread_mismatch')
    return dict(passed=all(r['passed']for r in rows)and negative_passed,cases=rows,
        negative_cases=negative,negative_cases_passed=negative_passed,opengl=driver)


def run(output):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    if(output/'summary.json').exists():raise FileExistsError('preserve_stereo_evidence_choose_new_output')
    protocol=json.loads(PROTOCOL.read_text());config=load_stereo_camera()
    if protocol['camera_config_sha256']!=sha(STEREO_CAMERA_CONFIG):raise ValueError('stereo_protocol_camera_hash_mismatch')
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    (output/'camera.json').write_text(json.dumps(config,indent=2)+'\n')
    report=dict(passed=False,scope='WSL MuJoCo actual stereo rendering/source-contract only; not metric accuracy/motion acceptance',
        protocol_sha256=sha(PROTOCOL),camera_sha256=sha(STEREO_CAMERA_CONFIG),source_sha256=sha(__file__),
        started_host_s=time.perf_counter(),hardware_execution_ready=False)
    try:
        base=prepare_retail_model(ROOT/'models/minisegway/mini_segway.xml',output/'bookstore.xml')
        path=prepare_stereo_companion_model(base,output/'stereo.xml',asset_dir=ROOT/'.venv/assets/visual_companion_gazebo/derived')
        model=mujoco.MjModel.from_xml_path(str(path));data=mujoco.MjData(model)
        actor=VisualCompanionActor(model,asset_dir=ROOT/'.venv/assets/visual_companion_gazebo/derived')
        geometry=load_spatial_geometry();jobs=[]
        for index,(pitch,yaw)in enumerate(((0.,0.),(.12,.2),(-.1,-.3))):
            mujoco.mj_resetDataKeyframe(model,data,0)
            rotation=geometry.mj_to_ros.T@Rotation.from_euler('zy',[yaw,pitch]).as_matrix()@geometry.mj_to_ros
            q=Rotation.from_matrix(rotation).as_quat();data.qpos[3:7]=[q[3],*q[:3]]
            actor.set_pose(data,.25+index*.1,[1.8,0.,0.],np.pi);mujoco.mj_forward(model,data)
            snapshot=state(model,data);snapshot.setflags(write=False);jobs.append(snapshot)
            np.savez_compressed(output/f'{index}_snapshot.npz',integration_state=snapshot)
        with ThreadPoolExecutor(max_workers=1,thread_name_prefix='stereo-render-test')as worker:
            report.update(worker.submit(render_worker,model,jobs,STEREO_CAMERA_CONFIG,output).result(timeout=30.))
    except BaseException as error:
        report.update(error=repr(error),traceback=traceback.format_exc());raise
    finally:
        report['ended_host_s']=time.perf_counter();(output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']:raise SystemExit(1)


def make_stereo_processor(camera,protocol,calibration_id):
    """Use the same explicit declared-calibration adapter as runtime."""
    from companionbot_perception import _native_stereo as native
    from companionbot_perception.runtime.stereo import make_native_config
    return native,native.StereoDepthProcessor(make_native_config(camera,protocol,calibration_id=calibration_id))


def evaluate_depth(args):
    """Fixed48pose real RGB/AI/stereo diagnostic; GT is one-way scoring only."""
    # Reuse diagnostic-only scene/ROI utilities, never its log-affine fitter.
    from verify_visual_companion_render import samples,posthoc_surface,audit_scene,stats,quantiles
    from companionbot_perception.core.camera import ColorFrame
    from companionbot_perception.core.camera_calibration import CameraCalibration
    from companionbot_perception.core.depth import DepthFrame,YoloDepthAdapter,YoloDepthConfig
    from companionbot_perception.core.detection_stream import process_person_frame
    from companionbot_perception.core.master_depth import representative_master_depth,torso_roi_xyxy
    from companionbot_perception.core.master_reid_session import MasterReIDSession
    from companionbot_perception.core.person_detector import PersonDetector,PersonDetectorConfig
    from companionbot_perception.core.person_reid import (
        MasterReIDEvidence,PersonReIdentifier,TorchReIDOSNetBackend,default_osnet_weights_path)
    from companionbot_perception.core.person_tracker import PersonTracker
    from companionbot_perception.core.ultralytics_bytetrack import UltralyticsByteTrackBackend
    from companionbot_perception.runtime.session import _SameFramePairer
    from companionbot_simulation.core.visual_companion_scene import load_character_assets
    import torch

    output=args.output_dir.resolve();output.mkdir(parents=True,exist_ok=True)
    if(output/'summary.json').exists():raise FileExistsError('preserve_stereo_evidence_choose_new_output')
    protocol=json.loads(PROTOCOL.read_text());camera=load_stereo_camera()
    _,manifest=load_character_assets(ROOT/'.venv/assets/visual_companion_gazebo/derived')
    if(protocol['camera_config_sha256']!=sha(STEREO_CAMERA_CONFIG)
            or protocol['character_asset_lock_sha256']!=manifest['lock_sha256']):
        raise ValueError('stereo_protocol_asset_camera_identity_mismatch')
    if args.torch_threads!=protocol['runtime_budget']['torch_intraop_threads']:
        raise ValueError('stereo_frozen_inference_thread_budget_mismatch')
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    (output/'camera.json').write_text(json.dumps(camera,indent=2)+'\n')
    rows=[];events=[];renderer=None;tracker=None
    report=dict(passed=False,completed=False,scope='WSL true MuJoCo stereo RGB and original AI fixed-pose diagnostic only',
        hardware_execution_ready=False,full_visual_motion_acceptance=False,python=sys.executable,
        protocol_sha256=sha(PROTOCOL),camera_config_sha256=sha(STEREO_CAMERA_CONFIG),
        source_sha256=sha(__file__),started_host_s=time.perf_counter(),mode=args.mode,
        gt_controller_path=False,posthoc_fit=False,torch_threads=args.torch_threads)
    try:
        native,processor=make_stereo_processor(camera,protocol,sha(STEREO_CAMERA_CONFIG))
        from companionbot_perception.runtime.stereo import algorithm_identity
        report['algorithm_identity']=algorithm_identity()
        report['native_stereo']=dict(path=native.__file__,sha256=sha(native.__file__),
            compiled_opencv=native.OPENCV_COMPILED_VERSION,runtime_opencv=native.OPENCV_RUNTIME_VERSION,
            calibration_audit=processor.calibration_audit())
        base=prepare_retail_model(ROOT/'models/minisegway/mini_segway.xml',output/'bookstore.xml')
        scene=prepare_stereo_companion_model(base,output/'stereo.xml',asset_dir=ROOT/'.venv/assets/visual_companion_gazebo/derived')
        model=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(model)
        actor=VisualCompanionActor(model,asset_dir=ROOT/'.venv/assets/visual_companion_gazebo/derived')
        report['plant_overlay_audit']=audit_scene(base,model)
        renderer=StereoRGBRenderer(model)
        from OpenGL import GL
        report['opengl']={name:GL.glGetString(code).decode()for name,code in
            [('vendor',GL.GL_VENDOR),('renderer',GL.GL_RENDERER),('version',GL.GL_VERSION)]}
        calibration=CameraCalibration(camera['K'],camera['distortion'],camera['width'],camera['height'],
            str(output/'camera.json'),'declared_sim_stereo_pinhole')
        models=[ROOT/protocol['detector']['model'],ROOT/protocol['depth']['model'],default_osnet_weights_path()]
        if not all(p.is_file()for p in models):raise FileNotFoundError('explicit_model_weights_missing_no_download')
        report['models_sha256']={str(p.relative_to(ROOT)):sha(p)for p in models}
        detector=PersonDetector(PersonDetectorConfig(models[0],protocol['detector']['imgsz'],protocol['detector']['confidence']))
        depth=YoloDepthAdapter(YoloDepthConfig(models[1],protocol['depth']['imgsz']))
        policy=json.loads((ROOT/'models/minisegway/stage7/stage7_6/config/stage7_6_reid_auto_reacquire_config.json').read_text())['auto_reacquire']
        tracker=PersonTracker(UltralyticsByteTrackBackend())
        master=MasterReIDSession(MasterReIDEvidence(PersonReIdentifier(TorchReIDOSNetBackend(models[2]))),
            initial_track_id=1,accept_threshold=policy['reid_accept_threshold'],retry_threshold=policy['reid_retry_threshold'],
            retry_interval_s=policy['retry_interval_s'],stable_updates=policy['minimum_stable_tracker_updates'])
        pairer=_SameFramePairer();sequence=0;previous_capture=None
        warmup=protocol['source_contract']['fixed_per_pose_warmup']
        offsets=[('warmup',x)for x in warmup['animation_offsets_s']]+[
            ('measurement',x)for x in protocol['source_contract']['original_three_frame_offsets_s']]
        # Each channel's native model setup completes before applying the public
        # PyTorch budget, since Ultralytics setup_device otherwise resets it.
        initialized=set();setup_lock=threading.Lock()
        def prepare_channel(name,adapter,frame):
            if name not in initialized:
                with setup_lock:
                    adapter(frame)
                    torch.set_num_threads(args.torch_threads)
                    initialized.add(name)
            if torch.get_num_threads()!=args.torch_threads:
                torch.set_num_threads(args.torch_threads)
        def detect(frame):
            started=time.perf_counter()
            prepare_channel('detector',lambda f:process_person_frame(f,detector),frame)
            detections=process_person_frame(frame,detector);tracking=tracker.update(detections)
            result=master.process(frame,tracking)
            return detections,result,time.perf_counter()-started,time.perf_counter(),torch.get_num_threads()
        def estimate(frame):
            started=time.perf_counter();prepare_channel('depth',depth.process,frame)
            result=depth.process(frame)
            return result,time.perf_counter()-started,time.perf_counter(),torch.get_num_threads()
        with ThreadPoolExecutor(max_workers=2,thread_name_prefix='stereo-ai-diagnostic')as workers:
            for index,(split,pose)in enumerate(samples(protocol,'smoke' if args.mode=='depth-smoke' else 'evaluate-depth')):
                sample_dir=output/f'{index:03d}_{split}';sample_dir.mkdir()
                row=dict(index=index,split=split,scene_pose_posthoc=pose,source_frames=[],
                    stereo_depth_m=None,model_depth_m=None,posthoc_axial_z_m=None)
                for phase,offset in offsets:
                    if previous_capture is not None:
                        remaining=warmup['minimum_actual_capture_interval_s']-(time.perf_counter()-previous_capture)
                        if remaining>0:time.sleep(remaining)
                    actor.set_pose(data,pose['animation_time_s']+offset,
                        [pose['distance_m'],pose['lateral_m'],0.],pose['actor_yaw_rad'])
                    acquired=time.perf_counter();previous_capture=acquired
                    mujoco.mj_forward(model,data)
                    def left_ready(image,ready):
                        image.setflags(write=False)
                        frame=ColorFrame(image,sequence,'mujoco:stereo-diagnostic',camera['width'],camera['height'],ready)
                        captured.update(frame=frame,detector=workers.submit(detect,frame),depth=workers.submit(estimate,frame))
                    captured={};left,right=renderer.render_bgr(data,on_left=left_ready)
                    right.setflags(write=False);frame=captured['frame']
                    stamp=native.StereoFrameStamp('stereo-fixed-grid',sequence,acquired,sha(STEREO_CAMERA_CONFIG),'host_perf_counter')
                    stereo=processor.process(left,right,stamp,stamp);stereo_ready=time.perf_counter()
                    detections,perceived,detect_wall,detect_ready,detect_threads=captured['detector'].result()
                    metric=DepthFrame(frame.source_id,frame.sequence_id,frame.host_receive_time_s,
                        frame.width,frame.height,stereo.depth_m,'opencv4.6.StereoSGBM','declared_rectified_stereo')
                    representative=representative_master_depth(perceived.master,metric)
                    metric_ready=time.perf_counter()
                    depth_result,depth_wall,depth_ready,depth_threads=captured['depth'].result()
                    pairer.add_tracking(frame,perceived.tracking,perceived.master)
                    legacy_pair=pairer.add_depth(frame,depth_result,dict(ready_time_s=depth_ready))
                    if legacy_pair is None:raise RuntimeError('original_yolo_depth_exact_frame_pair_failed')
                    legacy_rep=representative_master_depth(legacy_pair.master,legacy_pair.depth)
                    result=dict(sequence=sequence,phase=phase,animation_offset_s=offset,
                        capture_host_s=acquired,capture_sim_time_s=float(data.time),source_clock_domain='host_perf_counter',
                        source_host_s=frame.host_receive_time_s,render_parts=dict(renderer.last_timing),
                        detector_wall_s=detect_wall,detector_ready_host_s=detect_ready,
                        depth_wall_s=depth_wall,depth_ready_host_s=depth_ready,
                        stereo_ready_host_s=stereo_ready,stereo_diagnostics=dict(stereo.diagnostics),
                        result_ready_host_s=metric_ready,capture_age_s=metric_ready-acquired,
                        result_age_s=metric_ready-frame.host_receive_time_s,
                        fresh=metric_ready-acquired<=protocol['freshness']['max_result_source_age_s'],
                        detection_count=len(detections.detections),detections=[asdict(d)for d in detections.detections],
                        tracking=[asdict(t)for t in perceived.tracking.tracks],master_state=perceived.master.state.value,
                        master_track_id=perceived.master.bound_track_id,
                        master_visible=perceived.master.master_track is not None,
                        same_frame_paired=True,legacy_same_frame_paired=True,
                        representative_stereo_depth=asdict(representative)if representative else None,
                        representative_yolo_depth=asdict(legacy_rep)if legacy_rep else None,
                        torch_threads={'detector':detect_threads,'depth':depth_threads})
                    row['source_frames'].append(result);sequence+=1
                    if phase=='measurement'and offset==protocol['source_contract']['original_three_frame_offsets_s'][-1]:
                        cv2.imwrite(str(sample_dir/'left.png'),left);cv2.imwrite(str(sample_dir/'right.png'),right)
                        gt_depth,gt_mask=posthoc_surface(renderer,data)
                        np.savez_compressed(sample_dir/'depth_and_posthoc.npz',stereo_depth_m=stereo.depth_m,
                            disparity_px=stereo.disparity_px,stereo_valid=stereo.valid_mask,
                            stereo_rejection=stereo.rejection_mask,lr_error_px=stereo.lr_error_px,
                            model_depth=depth_result.depth_m,posthoc_axial_depth=gt_depth,posthoc_visible_skin=gt_mask)
                        row['final_measurement_fresh']=result['fresh'];row['final_master_state']=result['master_state']
                        if representative is not None:row['stereo_depth_m']=representative.depth_m
                        if legacy_rep is not None:row['model_depth_m']=legacy_rep.depth_m
                        # GT ROI is defined solely by the original visible Master
                        # bbox and original torso function, never matcher validity.
                        track=perceived.master.master_track
                        if track is not None:
                            l,t,r,b=torso_roi_xyxy(track.bbox_xyxy_px,frame.width,frame.height)
                            mask=gt_mask[t:b,l:r];truth=gt_depth[t:b,l:r][mask]
                            row['roi_xyxy_px']=[l,t,r,b];row['posthoc_visible_actor_pixels_in_roi']=int(truth.size)
                            row['posthoc_actor_roi_fraction']=float(mask.mean())if mask.size else 0.
                            if truth.size>=10:row['posthoc_axial_z_m']=float(np.median(truth))
                        if row['stereo_depth_m']is not None and row['posthoc_axial_z_m']is not None:
                            row['stereo_error_m']=row['stereo_depth_m']-row['posthoc_axial_z_m']
                rows.append(row);(sample_dir/'record.json').write_text(json.dumps(row,indent=2)+'\n')
                print(json.dumps(dict(index=index,split=split,stereo_depth_m=row['stereo_depth_m'],
                    truth_m=row['posthoc_axial_z_m'],error_m=row.get('stereo_error_m'),
                    master_state=row['final_master_state'],fresh=row['final_measurement_fresh'])),flush=True)
        frames=[f for r in rows for f in r['source_frames']];measured=[f for f in frames if f['phase']=='measurement']
        held=[r for r in rows if r['split']=='heldout_split'];valid=[r for r in held if r.get('stereo_error_m')is not None]
        limits=protocol['measurement_acceptance'];metrics=stats([r['stereo_error_m']for r in valid])
        coverage=len(valid)==len(held)==32
        accuracy=bool(metrics['count']and metrics['rmse_m']<=limits['heldout_rmse_m']
            and metrics['p95_absolute_error_m']<=limits['heldout_p95_absolute_error_m'])
        report.update(completed=True,samples=len(rows),source_frames=len(frames),measurement_frames=len(measured),
            heldout=dict(expected=32,available=len(valid),coverage_passed=coverage,metrics=metrics,accuracy_passed=accuracy,
                unavailable_indices=[r['index']for r in held if r.get('stereo_error_m')is None]),
            measurement_capture_age=quantiles([f['capture_age_s']for f in measured]),
            measured_fresh=sum(f['fresh']for f in measured),measurement_master_visible=sum(f['master_visible']for f in measured),
            native_stereo_wall=quantiles([f['stereo_diagnostics']['binding_total_wall_s']for f in measured]),
            stereo_processor_snapshot=processor.snapshot(),passed=coverage and accuracy,
            accuracy_does_not_certify_online_freshness=True)
        events=master.events
        names=('distance_m','lateral_m','actor_yaw_rad','animation_time_s')
        pose_keys=sorted(json.dumps({k:float(r['scene_pose_posthoc'][k])for k in names},sort_keys=True,
            separators=(',',':'),allow_nan=False)for r in valid)
        if len(pose_keys)!=len(set(pose_keys)):raise RuntimeError('duplicate_heldout_pose_keys')
        validation=dict(schema_version=1,depth_source='native_opencv_stereo',depth_convention='axial_z',
            camera_config_sha256=report['camera_config_sha256'],protocol_sha256=report['protocol_sha256'],
            character_asset_lock_sha256=manifest['lock_sha256'],algorithm_identity=report['algorithm_identity'],
            models_sha256=report['models_sha256'],
            heldout=dict(passed=coverage and accuracy,available_samples=len(valid),expected_samples=32,
                distinct_pose_keys=pose_keys,
                rmse_m=metrics['rmse_m'],p95_absolute_error_m=metrics['p95_absolute_error_m']),
            online_ground_truth_calibration=False,depth_scale_fit=False,
            scope='fixed simulated camera/asset/geometry/software metric-accuracy only; online source-age/identity/history gates remain mandatory',
            evidence_records='records.json',evidence_summary='summary.json',hardware_execution_ready=False)
        (output/'stereo_validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    except BaseException as error:
        report.update(error=repr(error),traceback=traceback.format_exc());raise
    finally:
        if tracker is not None:tracker.close()
        if renderer is not None:renderer.close()
        report['ended_host_s']=time.perf_counter();report['peak_rss_mib']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
        (output/'records.json').write_text(json.dumps(rows,indent=2)+'\n')
        (output/'master_events.json').write_text(json.dumps(events,indent=2)+'\n')
        (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    if args.mode=='evaluate-depth'and not report['passed']:raise SystemExit(1)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--mode',choices=('render','depth-smoke','evaluate-depth'),default='render')
    parser.add_argument('--torch-threads',type=int,default=4)
    args=parser.parse_args()
    if args.mode=='render':run(args.output_dir)
    else:evaluate_depth(args)


if __name__=='__main__':main()
