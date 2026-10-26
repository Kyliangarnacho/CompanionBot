"""Headless rendered-human perception diagnostic; no motion/actuator interface.

RGB alone enters the existing detector, ByteTrack, Master/ReID and independent
YOLO depth. Renderer depth/skin masks are produced by a separate post-hoc pass.
The fixed calibration/held-out split is loaded before any model is run. Failed
detection/depth/age gates are retained; this program never substitutes GT depth.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import hashlib
import itertools
import json
from pathlib import Path
import platform
import resource
import sys
import time
import traceback

import cv2
import mujoco
import numpy as np

from companionbot_contracts.paths import repository_root
from companionbot_perception.core.camera import ColorFrame
from companionbot_perception.core.camera_calibration import CameraCalibration
from companionbot_perception.core.depth import YoloDepthAdapter, YoloDepthConfig
from companionbot_perception.core.detection_stream import process_person_frame
from companionbot_perception.core.master_depth import representative_master_depth, back_project_camera_point
from companionbot_perception.core.master_reid_session import MasterReIDSession
from companionbot_perception.core.person_detector import PersonDetector, PersonDetectorConfig
from companionbot_perception.core.person_reid import (MasterReIDEvidence, PersonReIdentifier,
    TorchReIDOSNetBackend, default_osnet_weights_path)
from companionbot_perception.core.person_tracker import PersonTracker
from companionbot_perception.core.ultralytics_bytetrack import UltralyticsByteTrackBackend
from companionbot_perception.runtime.session import _SameFramePairer
from companionbot_simulation.core.retail_scene import prepare_retail_model
from companionbot_simulation.core.visual_companion_scene import (
    CAMERA_CONFIG, load_character_assets, load_visual_camera, prepare_visual_companion_model, VisualCompanionActor, OnboardRGBRenderer)

ROOT = repository_root()
PROTOCOL = ROOT/'models/minisegway/native_runtime/config/visual_depth_protocol.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return dict(count=0, rmse_m=None, p95_absolute_error_m=None)
    return dict(count=len(a), rmse_m=float(np.sqrt(np.mean(a*a))),
        p95_absolute_error_m=float(np.percentile(np.abs(a),95)), max_absolute_error_m=float(np.max(np.abs(a))))


def quantiles(values):
    a=np.asarray(values,dtype=float)
    return None if not len(a) else dict(count=len(a),p50_s=float(np.percentile(a,50)),
        p95_s=float(np.percentile(a,95)),p99_s=float(np.percentile(a,99)),max_s=float(np.max(a)))


def samples(protocol, mode):
    if mode == 'smoke':
        return [('smoke',dict(distance_m=1.8,lateral_m=0.,actor_yaw_rad=np.pi,animation_time_s=.25))]
    rows=[]
    for split in ('calibration_split','heldout_split'):
        fields=('distance_m','lateral_m','actor_yaw_rad','animation_time_s')
        rows.extend((split,dict(zip(fields,values))) for values in itertools.product(*(protocol[split][k] for k in fields)))
    return rows


def posthoc_surface(renderer,data):
    """Independent GT pass. This return never enters DepthFrame or perception."""
    renderer.renderer.enable_depth_rendering()
    try:
        renderer.renderer.update_scene(data,camera=renderer.config['name'],scene_option=renderer.option)
        with_skin=renderer.renderer.render().copy()
        renderer.option.skingroup[2]=0
        renderer.renderer.update_scene(data,camera=renderer.config['name'],scene_option=renderer.option)
        without_skin=renderer.renderer.render().copy()
    finally:
        renderer.option.skingroup[2]=1
        renderer.renderer.disable_depth_rendering()
    visible=np.isfinite(with_skin)&(with_skin>0)&(with_skin < without_skin-1e-4)
    return with_skin,visible


def audit_scene(base,model):
    """Verify the visual overlay kept every pre-existing plant parameter."""
    original=mujoco.MjModel.from_xml_path(str(base))
    checks={name:int(getattr(original,name))==int(getattr(model,name)) for name in ('nq','nv','nu','nsensor')}
    for name,count in (('body_mass',original.nbody),('body_inertia',original.nbody),
            ('body_ipos',original.nbody),('body_iquat',original.nbody),
            ('geom_friction',original.ngeom),('geom_solref',original.ngeom),
            ('geom_solimp',original.ngeom),('geom_contype',original.ngeom),
            ('geom_conaffinity',original.ngeom),('geom_pos',original.ngeom),
            ('geom_quat',original.ngeom),('geom_size',original.ngeom),
            ('dof_damping',original.nv),('dof_frictionloss',original.nv),
            ('actuator_ctrlrange',original.nu),('actuator_forcerange',original.nu),
            ('actuator_gear',original.nu)):
        checks[name]=bool(np.array_equal(getattr(original,name),getattr(model,name)[:count]))
    checks['physics_timestep']=original.opt.timestep==model.opt.timestep==.001
    checks['physics_gravity']=bool(np.array_equal(original.opt.gravity,model.opt.gravity))
    checks['physics_integrator']=original.opt.integrator==model.opt.integrator
    result=dict(passed=all(checks.values()),checks=checks,
        added_bodies=int(model.nbody-original.nbody),added_geoms=int(model.ngeom-original.ngeom),
        native_skins=int(model.nskin),new_actuators=0,actor_state_controller_input=False)
    if not result['passed']:raise RuntimeError('visual_overlay_changed_original_plant_parameters')
    return result


def evaluate(rows,protocol,calibration):
    """Fit once on calibration rows and assess untouched held-out rows."""
    result={}
    limits=protocol['measurement_acceptance']
    for convention in protocol['depth']['conventions_evaluated']:
        fit_rows=[r for r in rows if r['split']=='calibration_split' and r.get('posthoc_'+convention+'_m') is not None and r.get('model_depth_m') is not None]
        held=[r for r in rows if r['split']=='heldout_split' and r.get('posthoc_'+convention+'_m') is not None and r.get('model_depth_m') is not None]
        mapping=None
        if len(fit_rows)>=2 and np.ptp([r['model_depth_m'] for r in fit_rows])>1e-9:
            raw=np.log([r['model_depth_m'] for r in fit_rows]);truth=np.log([r['posthoc_'+convention+'_m'] for r in fit_rows])
            a,b=np.linalg.lstsq(np.column_stack((raw,np.ones(len(raw)))),truth,rcond=None)[0]
            mapping=dict(log_slope=float(a),log_offset=float(b),form='exp(b) * raw**a',fit_samples=len(fit_rows),fit_split='calibration_split_only')
        raw_errors=[r['model_depth_m']-r['posthoc_'+convention+'_m'] for r in held]
        calibrated_errors=[]
        for row in held:
            if mapping is not None:
                fitted=float(np.exp(mapping['log_offset'])*row['model_depth_m']**mapping['log_slope'])
                row.setdefault('calibrated_model_depth_m',{})[convention]=fitted
                calibrated_errors.append(fitted-row['posthoc_'+convention+'_m'])
        raw_metrics,cal_metrics=stats(raw_errors),stats(calibrated_errors)
        def passes(metrics):
            return bool(metrics['count'] and metrics['rmse_m']<=limits['heldout_rmse_m'] and metrics['p95_absolute_error_m']<=limits['heldout_p95_absolute_error_m'])
        result[convention]=dict(frozen_mapping=mapping,raw_heldout=raw_metrics,
            calibrated_heldout=cal_metrics,raw_measurement_gate=passes(raw_metrics),
            calibrated_measurement_gate=passes(cal_metrics),
            complete_heldout_coverage=len(held)==sum(r['split']=='heldout_split' for r in rows),
            source_convention_certified=False,
            note='heldout engineering check only; fitting does not certify C920 or physical model geometry')
    return result


def run(args):
    output=args.output_dir.resolve();output.mkdir(parents=True,exist_ok=True)
    if (output/'summary.json').exists():
        raise FileExistsError('preserve_existing_visual_evidence_choose_new_output_dir')
    protocol=json.loads(args.protocol.read_text());camera=load_visual_camera(args.camera_config)
    camera_path=Path(args.camera_config or CAMERA_CONFIG)
    _,asset_manifest=load_character_assets(args.asset_dir)
    if (protocol['camera_config_sha256']!=sha(camera_path)
            or protocol['character_asset_lock_sha256']!=asset_manifest['lock_sha256']):
        raise ValueError('visual_protocol_asset_or_camera_identity_mismatch')
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    (output/'camera.json').write_text(json.dumps(camera,indent=2)+'\n')
    report=dict(scope='WSL MuJoCo EGL/headless render and real model inference diagnostic; no control or hardware',
        hardware_execution_ready=False,started_host_s=time.perf_counter(),python=sys.executable,
        python_version=sys.version,platform=platform.platform(),protocol_sha256=sha(args.protocol),
        source_sha256=sha(__file__),mode=args.mode,explicit_gt_controller_path=False,
        gt_policy='RGB enters models; separate render depth only posthoc assessment; actor pose only image/physics generation')
    rows=[];events=[];renderer=None;tracker=None
    try:
        base=prepare_retail_model(ROOT/'models/minisegway/mini_segway.xml',output/'bookstore.xml')
        scene=prepare_visual_companion_model(base,output/'visual.xml',asset_dir=args.asset_dir,camera_config=args.camera_config)
        model=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(model)
        report['plant_overlay_audit']=audit_scene(base,model)
        actor=VisualCompanionActor(model,asset_dir=args.asset_dir);renderer=OnboardRGBRenderer(model,camera_config=args.camera_config)
        from OpenGL import GL
        report['opengl']={name:GL.glGetString(code).decode('utf-8') for name,code in
            [('vendor',GL.GL_VENDOR),('renderer',GL.GL_RENDERER),('version',GL.GL_VERSION)]}
        calibration=CameraCalibration(camera['K'],camera['distortion'],camera['width'],camera['height'],str(output/'camera.json'),'declared_sim_pinhole')
        model_files=[ROOT/protocol['detector']['model'],ROOT/protocol['depth']['model'],default_osnet_weights_path()]
        if not all(path.is_file() for path in model_files):
            raise FileNotFoundError('explicit_model_weights_missing_no_runtime_download')
        report['models_sha256']={str(p.relative_to(ROOT)):sha(p) for p in model_files}
        detector=PersonDetector(PersonDetectorConfig(model_files[0],protocol['detector']['imgsz'],protocol['detector']['confidence']))
        depth=YoloDepthAdapter(YoloDepthConfig(model_files[1],protocol['depth']['imgsz']))
        reid_policy=json.loads((ROOT/'models/minisegway/stage7/stage7_6/config/stage7_6_reid_auto_reacquire_config.json').read_text())['auto_reacquire']
        tracker=PersonTracker(UltralyticsByteTrackBackend())
        master=MasterReIDSession(MasterReIDEvidence(PersonReIdentifier(TorchReIDOSNetBackend(model_files[2]))),
            initial_track_id=args.select_track_id,accept_threshold=reid_policy['reid_accept_threshold'],
            retry_threshold=reid_policy['reid_retry_threshold'],retry_interval_s=reid_policy['retry_interval_s'],
            stable_updates=reid_policy['minimum_stable_tracker_updates'])
        pairer=_SameFramePairer();sequence=0
        # Independent channels consume the exact same immutable source frame.
        # This is a finite diagnostic using the standard-library executor only.
        with ThreadPoolExecutor(max_workers=2,thread_name_prefix='visual-diagnostic') as workers:
            for sample_index,(split,pose) in enumerate(samples(protocol,args.mode)):
                sample_dir=output/f'{sample_index:03d}_{split}';sample_dir.mkdir()
                row=dict(index=sample_index,split=split,scene_pose_posthoc=pose,model_depth_m=None,
                    posthoc_axial_z_m=None,posthoc_euclidean_range_m=None,source_frames=[])
                # Three real source frames allow the unchanged explicit Master
                # reference confirmation. The source times are actual host read
                # completion, never fabricated animation or simulation times.
                for repeat in range(args.frames_per_pose):
                    actor.set_pose(data,pose['animation_time_s']+repeat*.05,
                        [pose['distance_m'],pose['lateral_m'],0.],pose['actor_yaw_rad'])
                    data.time=sample_index+repeat*.05 # scene time only, not a control clock
                    mujoco.mj_forward(model,data)
                    started=time.perf_counter();bgr=renderer.render_bgr(data);received=time.perf_counter()
                    bgr.setflags(write=False)
                    frame=ColorFrame(bgr,sequence,'mujoco:visual-diagnostic',camera['width'],camera['height'],received)
                    def detect():
                        before=time.perf_counter();detections=process_person_frame(frame,detector)
                        tracking=tracker.update(detections);result=master.process(frame,tracking)
                        return detections,result,time.perf_counter()-before
                    def estimate_depth():
                        before=time.perf_counter();result=depth.process(frame)
                        return result,time.perf_counter()-before,time.perf_counter()
                    detection_future=workers.submit(detect);depth_future=workers.submit(estimate_depth)
                    detections,perceived,detector_wall=detection_future.result()
                    depth_result,depth_wall,ready=depth_future.result()
                    depth_ready=ready
                    ready=time.perf_counter()  # pair is ready only after BOTH channels
                    # Feed the existing strict pairer, never latest-bbox/latest-depth.
                    pairer.add_tracking(frame,perceived.tracking,perceived.master)
                    paired=pairer.add_depth(frame,depth_result,dict(ready_time_s=depth_ready))
                    if paired is None:raise RuntimeError('same_frame_pairing_failed')
                    representative=representative_master_depth(paired.master,paired.depth)
                    frame_row=dict(sequence=sequence,source_host_s=received,render_started_host_s=started,
                        capture_host_s=started,capture_sim_time_s=float(data.time),
                        render_wall_s=received-started,detector_wall_s=detector_wall,depth_wall_s=depth_wall,
                        render_parts=dict(renderer.last_timing),
                        result_ready_host_s=ready,result_age_s=ready-received,
                        depth_ready_host_s=depth_ready,
                        capture_age_s=ready-started,
                        fresh=ready-started<=protocol['freshness']['max_result_source_age_s'],
                        detection_count=len(detections.detections),detections=[asdict(d) for d in detections.detections],
                        tracking=[asdict(t) for t in perceived.tracking.tracks],
                        master_state=perceived.master.state.value,master_track_id=perceived.master.bound_track_id,
                        same_frame_paired=True,representative_depth=asdict(representative) if representative else None)
                    row['source_frames'].append(frame_row);sequence+=1
                    if repeat==args.frames_per_pose-1:
                        cv2.imwrite(str(sample_dir/'rgb.png'),frame.bgr)
                        gt_depth,mask=posthoc_surface(renderer,data)
                        np.savez_compressed(sample_dir/'raw_depth_and_posthoc.npz',
                            model_depth=depth_result.depth_m,posthoc_axial_depth=gt_depth,posthoc_visible_skin=mask)
                        if representative is not None:
                            row['model_depth_m']=representative.depth_m
                            row['anchor_uv_px']=representative.anchor_uv_px
                            l,t,r,b=representative.roi_xyxy_px
                            roi_mask=mask[t:b,l:r];valid=gt_depth[t:b,l:r][roi_mask]
                            row['posthoc_visible_actor_pixels_in_roi']=int(valid.size)
                            row['posthoc_actor_roi_fraction']=float(roi_mask.mean()) if roi_mask.size else 0.
                            if valid.size>=10:
                                row['posthoc_axial_z_m']=float(np.median(valid))
                                ys,xs=np.indices((b-t,r-l));fx,fy=calibration.K[0,0],calibration.K[1,1]
                                scale=np.sqrt(1+((xs+l-calibration.K[0,2])/fx)**2+((ys+t-calibration.K[1,2])/fy)**2)
                                row['posthoc_euclidean_range_m']=float(np.median((gt_depth[t:b,l:r]*scale)[roi_mask]))
                rows.append(row)
                (sample_dir/'record.json').write_text(json.dumps(row,indent=2)+'\n')
                print(json.dumps(dict(index=sample_index,split=split,last=row['source_frames'][-1],model_depth_m=row['model_depth_m'])),flush=True)
        report['depth_evaluation']=evaluate(rows,protocol,calibration)
        all_frames=[f for r in rows for f in r['source_frames']]
        report.update(completed=True,samples=len(rows),source_frames=len(all_frames),
            frames_with_detections=sum(f['detection_count']>0 for f in all_frames),
            frames_master_locked=sum(f['master_state']=='LOCKED' for f in all_frames),
            same_frame_pairs=sum(f['same_frame_paired'] for f in all_frames),
            fresh_results=sum(f['fresh'] for f in all_frames),
            render_wall=quantiles([f['render_wall_s'] for f in all_frames]),
            detector_wall=quantiles([f['detector_wall_s'] for f in all_frames]),
            depth_wall=quantiles([f['depth_wall_s'] for f in all_frames]),
            result_age=quantiles([f['result_age_s'] for f in all_frames]),
            capture_age=quantiles([f['capture_age_s'] for f in all_frames]),
            camera_geometry_checked=True,rendered_character_native_skin=True,
            perception_observations_available=any(r['model_depth_m'] is not None for r in rows),
            full_visual_motion_acceptance=False,execution='diagnostic_only_no_control_interface')
        events=master.events
    except BaseException as error:
        report.update(completed=False,error=repr(error),traceback=traceback.format_exc())
        raise
    finally:
        if tracker is not None:tracker.close()
        if renderer is not None:renderer.close()
        report['ended_host_s']=time.perf_counter();report['peak_rss_mib']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
        (output/'records.json').write_text(json.dumps(rows,indent=2)+'\n')
        (output/'master_events.json').write_text(json.dumps(events,indent=2)+'\n')
        (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--mode',choices=('smoke','evaluate-depth'),default='smoke')
    parser.add_argument('--protocol',type=Path,default=PROTOCOL)
    parser.add_argument('--camera-config',type=Path,default=None)
    parser.add_argument('--asset-dir',type=Path,default=None,help='explicit ignored derived character cache')
    parser.add_argument('--select-track-id',type=int,default=1,help='explicit diagnostic Master selection, no GT identity assignment')
    parser.add_argument('--frames-per-pose',type=int,default=3)
    args=parser.parse_args()
    if args.frames_per_pose<3:parser.error('at least three real frames required for frozen Master confirmation')
    run(args)


if __name__=='__main__':main()
