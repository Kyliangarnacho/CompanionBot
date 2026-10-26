"""RGB sensing composition inside the existing spatial simulation owner.

Actor route is private scene-generation input. Only rendered ColorFrame pixels
and public sensorized capture pitch feed perception; the native FollowController
receives source-mapped model observations. No actor pose/depth/GT crosses over.
"""
import json
import math
from pathlib import Path
import threading
import time

import numpy as np

from companionbot_contracts.paths import repository_root
from companionbot_perception.runtime.rendered import RenderCapture, RenderedPerceptionRuntime
from companionbot_simulation.core.visual_companion_scene import (
    load_visual_camera, prepare_visual_companion_model, VisualCompanionActor, OnboardRGBRenderer,
)


class VisualCompanionSession:
    def __init__(self, config_path, output_dir, *, epoch, adapter, source_now):
        self.path = Path(config_path).resolve()
        self.config = json.loads(self.path.read_text())
        if self.config['schema_version'] != 1 or self.config['hardware_execution_ready']:
            raise ValueError('invalid_visual_simulation_configuration')
        self.root, self.output = repository_root(), Path(output_dir)
        self.output.mkdir(parents=True, exist_ok=True)
        self.epoch, self.adapter, self.source_now = epoch, adapter, source_now
        self.controller = adapter.follow_controller
        self.camera_path = self.root/self.config['camera_config']
        self.stereo_enabled = self.config.get('metric_depth_source','yolo_depth') == 'native_stereo'
        if self.stereo_enabled:
            from companionbot_simulation.core.stereo_companion_scene import load_stereo_camera
            self.camera = load_stereo_camera(self.camera_path)
        else:
            self.camera = load_visual_camera(self.camera_path)
        self.route = self.config['actor_route']
        if (len(self.route) < 2 or self.route[0]['time_s'] != 0
                or any(b['time_s'] <= a['time_s'] for a,b in zip(self.route,self.route[1:]))):
            raise ValueError('invalid_simulation_actor_route')
        for waypoint in self.route:
            if len(waypoint['position_ros_m']) != 3 or not np.isfinite([
                    *waypoint['position_ros_m'], waypoint['time_s'], waypoint['yaw_rad']]).all():
                raise ValueError('nonfinite_simulation_actor_route')
        self.actor = self.renderer = self.perception = None
        self.sequence, self._closed = 0, False
        self._log_lock = threading.Lock()
        self._callback_lock = threading.Lock()
        self._render_condition = threading.Condition()
        self._render_worker = None
        self._render_stop = False
        self._render_error = None
        self._render_cleanup_errors = []
        self._pending_capture = None
        self.snapshot_overwritten = self.rendered_count = 0
        self._observations = None
        self.capture_rows = []
        self.period_ticks = round(1/self.camera['frame_rate_hz']/.002)
        if not math.isclose(self.period_ticks*.002,1/self.camera['frame_rate_hz'],abs_tol=1e-12):
            raise ValueError('visual_capture_period_not_on_control_horizon')

    def prepare_model(self, source, destination):
        if self.stereo_enabled:
            from companionbot_simulation.core.stereo_companion_scene import prepare_stereo_companion_model
            return prepare_stereo_companion_model(source,destination,
                asset_dir=self.root/self.config['character_asset_dir'],camera_config=self.camera_path)
        return prepare_visual_companion_model(source,destination,
            asset_dir=self.root/self.config['character_asset_dir'],camera_config=self.camera_path)

    def setup(self, sim):
        import mujoco
        self._observations = (self.output/'native_observation_results.jsonl').open('w')
        self.actor = VisualCompanionActor(sim.model,asset_dir=self.root/self.config['character_asset_dir'])
        calibration = self.config.get('depth_calibration')
        validation = self.config.get('stereo_validation')
        self.perception = RenderedPerceptionRuntime(camera_path=self.camera_path,
            protocol_path=self.root/self.config['depth_protocol'], source_epoch=self.epoch,
            output_dir=self.output/'perception', on_update=self.on_update,
            select_track_id=self.config.get('select_track_id'),
            character_asset_lock_sha256=self.actor.manifest['lock_sha256'],
            depth_calibration_path=self.root/calibration if calibration else None,
            inference_threads=self.config.get('inference_threads'),
            metric_depth_source=self.config.get('metric_depth_source','yolo_depth'),
            stereo_validation_path=self.root/validation if validation else None)
        # MuJoCo's documented integration-state API copies only a bounded
        # snapshot. Physics never waits for OpenGL or inference. Two reused
        # buffers exchange ownership under this source's short condition lock.
        self._state_spec = int(mujoco.mjtState.mjSTATE_INTEGRATION)
        count = mujoco.mj_stateSize(sim.model,self._state_spec)
        self._pending_state, self._render_state = np.empty(count), np.empty(count)
        self._render_worker = threading.Thread(target=self._render_loop,args=(sim.model,),
            name='onboard-rgb-renderer')
        self._render_worker.start()
        self.physics_step(sim)

    def physics_step(self, sim):
        # Original upstream walking poses are sampled at 60Hz. Physics still
        # advances at 1ms; the explicit kinematic actor is a scene participant.
        if sim.step_index % 16: return
        t = float(sim.data.time)
        route = self.route
        times = [w['time_s'] for w in route]
        position = [float(np.interp(t,times,[w['position_ros_m'][i] for w in route])) for i in range(3)]
        yaw = float(np.interp(t,times,np.unwrap([w['yaw_rad'] for w in route])))
        index = max(0,min(len(route)-2,int(np.searchsorted(times,t,side='right')-1)))
        moving = t < times[-1] and np.linalg.norm(np.asarray(route[index+1]['position_ros_m'])-
                                                       route[index]['position_ros_m']) > 1e-9
        self.actor.set_pose(sim.data,t,position,yaw,animate=moving)

    def observe(self, context, *, sensors_valid, control_sequence):
        if self._render_error is not None:
            raise RuntimeError('visual_renderer_failed:'+self._render_error)
        if control_sequence % self.period_ticks: return
        import mujoco
        self.perception.check_health()
        sim, estimate, source = context['sim'],context['estimate_after'],context['time_s']
        before_cpu, before = time.thread_time(), time.perf_counter()
        with self._render_condition:
            if self._render_stop: raise RuntimeError('visual_renderer_closed')
            if not math.isclose(float(sim.data.time),source,rel_tol=0.,abs_tol=1e-9):
                raise ValueError('visual_snapshot_and_sensor_horizon_mismatch')
            if self._pending_capture is not None: self.snapshot_overwritten += 1
            mujoco.mj_getState(sim.model,sim.data,self._pending_state,self._state_spec)
            self._pending_capture = (self.sequence,source,before,float(estimate.theta_hat_rad),
                bool(sensors_valid),time.perf_counter()-before,time.thread_time()-before_cpu)
            self._render_condition.notify()
        self.sequence += 1

    def _render_loop(self, model):
        import mujoco
        renderer = None
        try:
            data = mujoco.MjData(model)
            if self.stereo_enabled:
                from companionbot_simulation.core.stereo_companion_scene import StereoRGBRenderer
                renderer = StereoRGBRenderer(model,camera_config=self.camera_path)
            else:
                renderer = OnboardRGBRenderer(model,camera_config=self.camera_path)
            while True:
                with self._render_condition:
                    self._render_condition.wait_for(lambda:self._render_stop or self._pending_capture is not None)
                    if self._render_stop: break
                    metadata = self._pending_capture
                    self._pending_capture = None
                    self._pending_state,self._render_state = self._render_state,self._pending_state
                sequence,source,before,pitch,valid,snapshot_wall,snapshot_cpu = metadata
                render_started = time.perf_counter()
                mujoco.mj_setState(model,data,self._render_state,self._state_spec)
                if not math.isclose(float(data.time),source,rel_tol=0.,abs_tol=1e-9):
                    raise ValueError('visual_restored_snapshot_time_mismatch')
                # Reconstruct derived camera/skin transforms at the snapshot's
                # actual state. This private data never advances physics and
                # never returns pose/depth to the controller.
                mujoco.mj_forward(model,data)
                if self.stereo_enabled:
                    image = renderer.render_left_bgr(data)
                    received = renderer.last_timing['left_read_complete_host_s']
                    capture = RenderCapture(self.epoch,sequence,source,before,received,pitch,source,valid)
                    with self._callback_lock:
                        if self._closed: break
                        frame = self.perception.submit_bgr(image,capture)
                    # Original detector/YOLODepth run while the distinct right
                    # camera renders this SAME private snapshot. No authority
                    # lock is held during either render or native matching.
                    right = renderer.render_right_bgr(data)
                    right_ready = renderer.last_timing['right_read_complete_host_s']
                    self.perception.submit_stereo_pair(frame,right,capture,
                        right_read_complete_host_s=right_ready)
                    metric_ready = time.perf_counter()
                else:
                    image = renderer.render_bgr(data)
                    received = time.perf_counter()
                    capture = RenderCapture(self.epoch,sequence,source,before,received,pitch,source,valid)
                    with self._callback_lock:
                        if self._closed: break
                        self.perception.submit_bgr(image,capture)
                    right_ready = metric_ready = None
                self.rendered_count += 1
                self.capture_rows.append(dict(sequence=sequence,source_time_s=source,
                    acquisition_begin_host_s=before,read_complete_host_s=received,
                    snapshot_wall_s=snapshot_wall,snapshot_thread_cpu_s=snapshot_cpu,
                    snapshot_to_render_start_s=render_started-before,
                    render_wall_s=(right_ready if right_ready is not None else received)-render_started,
                    right_read_complete_host_s=right_ready,stereo_metric_ready_host_s=metric_ready,
                    acquisition_to_read_complete_s=received-before,
                    sensorized_pitch_rad=pitch,pitch_horizon_s=source,sensors_valid=valid))
        except BaseException as error:
            self._render_error = repr(error)
            self.adapter.native_authority.invalidate_follow('follow_renderer_failed',self.adapter.clock())
        finally:
            if renderer is not None:
                try: renderer.close()
                except Exception as error: self._render_cleanup_errors.append(repr(error))

    def on_update(self, update):
        with self._callback_lock:
            if self._closed:
                return
            self._on_update(update)

    def _on_update(self, update):
        capture, observation = update.capture, update.observation
        if capture is None:
            # No provable frame/task provenance: never revoke another task
            # using this late result. The native observation watchdog remains
            # authoritative when no fresh, covered result arrives.
            with self._log_lock:
                self._observations.write(json.dumps(dict(accepted=False,command_accepted=False,
                    reason='follow_capture_history_unavailable',received_host_s=time.perf_counter()))+'\n')
            return
        received_host = time.perf_counter()
        capture_age = received_host-capture.acquisition_begin_host_s
        valid = bool(update.motion_valid and observation is not None and
            0 <= capture_age <= self.perception.max_age_s)
        reason = ('visual_source_stale_at_authority' if update.motion_valid and not valid else update.reason)
        before_cpu, before_wall = time.thread_time(), time.perf_counter()
        result = self.controller.accept_measurement(capture.source_epoch,capture.sequence,
            capture.simulation_time_s,self.source_now(),self.adapter.clock(),
            observation.x_forward_m if observation else 0.,observation.y_left_m if observation else 0.,
            valid,reason)
        native_wall, native_cpu = time.perf_counter()-before_wall, time.thread_time()-before_cpu
        row = dict(sequence=capture.sequence,capture_source_s=capture.simulation_time_s,
            ready_host_s=update.ready_host_s,
            acquisition_to_ready_s=update.ready_host_s-capture.acquisition_begin_host_s,
            read_complete_to_ready_s=update.ready_host_s-capture.read_complete_host_s,
            model_motion_valid=valid,reason=result.reason,accepted=result.accepted,
            command_accepted=result.command_accepted,authority_epoch=result.authority_epoch)
        row.update(native_follow_wall_s=native_wall,native_follow_thread_cpu_s=native_cpu,
            source_now_s=self.source_now(),authority_received_host_s=received_host,
            acquisition_to_authority_s=capture_age)
        if result.radial is not None: row['radial']=dict(result.radial.row)
        if result.governor is not None: row['governor']=result.governor.row
        with self._log_lock: self._observations.write(json.dumps(row)+'\n')

    def close(self):
        # Callback admission closes atomically before joining workers or closing
        # their result sink. A timed-out inference cannot write a closed handle.
        with self._callback_lock:
            if self._closed: return
            self._closed = True
        self.adapter.native_authority.invalidate_follow('follow_visual_runtime_shutdown',self.adapter.clock())
        errors = []
        with self._render_condition:
            self._render_stop = True
            self._render_condition.notify_all()
        if self._render_worker is not None:
            self._render_worker.join(timeout=10.)
            if self._render_worker.is_alive(): errors.append('visual_renderer_failed_to_stop')
        if self._render_error is not None: errors.append(self._render_error)
        errors.extend(self._render_cleanup_errors)
        if self.perception is not None:
            try: self.perception.close()
            except Exception as error: errors.append(repr(error))
        if self._observations is not None: self._observations.close()
        (self.output/'captures.json').write_text(json.dumps(self.capture_rows,indent=2)+'\n')
        (self.output/'visual_runtime.json').write_text(json.dumps(dict(
            configuration=str(self.path),capture_count=self.sequence,rendered_count=self.rendered_count,
            metric_depth_source=self.config.get('metric_depth_source','yolo_depth'),
            snapshots_overwritten=self.snapshot_overwritten,renderer_worker_stopped=
                self._render_worker is None or not self._render_worker.is_alive(),
            snapshot_api='MuJoCo mjSTATE_INTEGRATION getState/setState + private mj_forward',
            cleanup_errors=errors,
            native_follow=self.controller.snapshot(),hardware_execution_ready=False),indent=2)+'\n')
        if errors: raise RuntimeError('visual_cleanup_failed:'+','.join(errors))
