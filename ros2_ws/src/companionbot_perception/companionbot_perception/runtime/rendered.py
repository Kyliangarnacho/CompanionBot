"""Rendered RGB adapter for the existing Stage 7 perception algorithms.

Only RGB and sensorized capture alignment enter this adapter. Actor pose and
renderer depth have no input argument. Independent detector/depth latest slots
feed the original exact-frame pairer and Master/ReID implementation.
"""
from collections import OrderedDict, deque
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import threading
import time

from companionbot_contracts.paths import repository_root
from companionbot_perception.core.camera import ColorFrame
from companionbot_perception.core.camera_calibration import CameraCalibration
from companionbot_perception.core.depth import YoloDepthAdapter, YoloDepthConfig
from companionbot_perception.core.detection_stream import LatestFrameSlot
from companionbot_perception.core.master_depth import representative_master_depth, back_project_camera_point
from companionbot_perception.core.master_reid_session import MasterReIDSession
from companionbot_perception.core.person_detector import PersonDetector, PersonDetectorConfig
from companionbot_perception.core.person_reid import MasterReIDEvidence, PersonReIdentifier, TorchReIDOSNetBackend, default_osnet_weights_path
from companionbot_perception.core.person_tracker import PersonTracker
from companionbot_perception.core.robot_geometry import CameraExtrinsic, PitchSample, camera_to_robot_observation
from companionbot_perception.core.ultralytics_bytetrack import UltralyticsByteTrackBackend
from companionbot_perception.runtime.session import _SameFramePairer, _depth_operation, _detector_operation


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class _MonitoredSameFramePairer(_SameFramePairer):
    """Observe the original bounded cache without changing matching policy."""
    def __init__(self, capacity=16):
        super().__init__(capacity)
        self.trimmed = dict(depth=0,tracking=0)

    def _trim(self, pending):
        before = len(pending)
        super()._trim(pending)
        self.trimmed['depth' if pending is self._depth else 'tracking'] += before-len(pending)

    def diagnostics(self):
        return dict(capacity=self.capacity,pending_depth=len(self._depth),pending_tracking=len(self._tracking),
            trimmed=dict(self.trimmed))


class _InferenceThreadBudget:
    """Explicit process policy applied after both original lazy model setups.

    Ultralytics CPU setup changes PyTorch's intra-op setting on first predict.
    Two ordinary barriers keep all subsequent inference outside the public
    setter phase. This controls the two AI workers, not ROS or rendering, and
    does not change inter-op/OpenCV settings. None preserves library behavior.
    """
    def __init__(self, threads, *, backend=None, timeout_s=10.):
        available = len(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else os.cpu_count() or 1
        if threads is not None and (isinstance(threads, bool) or not isinstance(threads, int)
                or not 1 <= threads <= available):
            raise ValueError('invalid_inference_threads_for_available_cpus')
        self.threads, self._backend = threads, backend
        self._timeout_s = timeout_s
        self._barrier = threading.Barrier(2) if threads is not None else None
        self._setter_lock = threading.Lock()
        self.channels = {}

    def _torch(self):
        if self._backend is None:
            import torch
            self._backend = torch
        return self._backend

    def actual(self):
        return self._torch().get_num_threads() if self.threads is not None else None

    def check(self):
        actual = self.actual()
        if self.threads is not None and actual != self.threads:
            raise RuntimeError('visual_inference_thread_budget_not_honored')
        return actual

    def after_first_inference(self, name, stop):
        if self._barrier is None:
            return True
        before = time.perf_counter()
        original = self.actual()
        try:
            self._barrier.wait(timeout=self._timeout_s)
            if stop.is_set():
                return False
            # set_num_threads may clear backend caches. No worker may compute
            # while either original model setup or either setter is running.
            with self._setter_lock:
                self._torch().set_num_threads(self.threads)
                actual = self.check()
                self.channels[name] = dict(native_tid=threading.get_native_id(),
                    after_original_first_predict_threads=original,
                    after_public_setter_threads=actual)
            self._barrier.wait(timeout=self._timeout_s)
        except threading.BrokenBarrierError as error:
            if stop.is_set():
                return False
            raise RuntimeError('visual_inference_thread_budget_setup_incomplete') from error
        self.channels[name]['setup_wait_wall_s'] = time.perf_counter()-before
        return not stop.is_set()

    def abort(self):
        if self._barrier is not None:
            self._barrier.abort()

    def report(self):
        return dict(requested_threads=self.threads, channels=dict(self.channels),
            scope='explicit PyTorch intra-op policy for both model workers in this process',
            mechanism='both real first-predict setups, public setters, then per-call verification',
            first_real_frame_retained=True, interop_or_opencv_changed=False)


@dataclass(frozen=True)
class RenderCapture:
    source_epoch: str
    sequence: int
    simulation_time_s: float
    acquisition_begin_host_s: float
    read_complete_host_s: float
    pitch_rad: float
    pitch_sample_simulation_time_s: float
    sensors_valid: bool

    def __post_init__(self):
        if (not self.source_epoch or len(self.source_epoch) > 128
                or isinstance(self.sequence, bool) or not isinstance(self.sequence, int) or self.sequence < 0
                or not all(math.isfinite(x) for x in (self.simulation_time_s,
                    self.acquisition_begin_host_s, self.read_complete_host_s, self.pitch_rad,
                    self.pitch_sample_simulation_time_s))
                or self.simulation_time_s < 0
                or self.acquisition_begin_host_s > self.read_complete_host_s
                or not isinstance(self.sensors_valid, bool)):
            raise ValueError('invalid_render_capture_alignment')


class CapturePitchHistory:
    """Exact acquisition records; never substitute a latest pitch on a miss."""
    def __init__(self, capacity=128):
        if capacity < 1: raise ValueError('invalid_capture_history_capacity')
        self.capacity = capacity
        self._samples = OrderedDict()
        self._lock = threading.Lock()
        self.overwritten = 0

    def add(self, capture):
        with self._lock:
            if capture.read_complete_host_s in self._samples:
                raise ValueError('duplicate_render_receipt_time')
            self._samples[capture.read_complete_host_s] = capture
            if len(self._samples) > self.capacity:
                self._samples.popitem(last=False); self.overwritten += 1

    def capture(self, source_time_s):
        with self._lock: return self._samples.get(source_time_s)

    def at_source_time(self, source_time_s):
        value = self.capture(source_time_s)
        if (value is None or not value.sensors_valid
                or abs(value.pitch_sample_simulation_time_s-value.simulation_time_s) > 1e-9):
            return None
        return PitchSample(source_time_s, value.pitch_rad, 'sensorized_sim_capture_history', False,
            pitch_time_s=value.acquisition_begin_host_s, alignment_basis='capture_time_mapped_to_host')


class VerifiedDepthCalibration:
    """Offline calibration artifact, accepted only against fixed heldout gates.

    Runtime reads coefficients and measured validation, never assessment images,
    actor states or perfect depth. An absent/failed artifact leaves motion invalid.
    """
    def __init__(self, path, *, camera_path, protocol_path, model_path, character_asset_lock_sha256):
        value = json.loads(Path(path).read_text())
        protocol = json.loads(Path(protocol_path).read_text())
        gate = protocol['measurement_acceptance']
        heldout = value['heldout']
        split = protocol['heldout_split']
        required_positions = math.prod(len(split[k]) for k in
            ('distance_m','lateral_m','actor_yaw_rad','animation_time_s'))
        if (value['camera_config_sha256'] != _sha(camera_path)
                or value['protocol_sha256'] != _sha(protocol_path)
                or value['model_sha256'] != _sha(model_path)
                or heldout['passed'] is not True
                or protocol['character_asset_lock_sha256'] != character_asset_lock_sha256
                or value['character_asset_lock_sha256'] != character_asset_lock_sha256
                or heldout.get('available_samples', 0) != required_positions
                or not 0 <= heldout['rmse_m'] <= gate['heldout_rmse_m']
                or not 0 <= heldout['p95_absolute_error_m'] <= gate['heldout_p95_absolute_error_m']
                or value['depth_convention'] not in ('axial_z', 'euclidean_range')):
            raise ValueError('depth_calibration_not_validated_for_this_runtime')
        self.scale = float(value['coefficients']['log_scale'])
        self.bias = float(value['coefficients']['log_bias'])
        if not math.isfinite(self.scale) or not math.isfinite(self.bias) or self.scale <= 0:
            raise ValueError('invalid_depth_calibration_coefficients')
        self.convention = value['depth_convention']
        self.identity = _sha(path)

    def apply(self, depth):
        try:
            value = math.exp(self.bias + self.scale*math.log(depth))
        except (OverflowError, ValueError):
            return None
        return value if math.isfinite(value) and value > 0 else None


@dataclass(frozen=True)
class PerceptionUpdate:
    capture: RenderCapture | None
    master_state: str
    reason: str
    ready_host_s: float
    observation: object | None = None
    raw_model_depth: float | None = None
    calibrated_depth_m: float | None = None
    motion_valid: bool = False
    metric_depth_source: str = 'yolo_depth'


class RenderedPerceptionRuntime:
    """One AI owner with independent existing model channels and bounded inputs."""
    def __init__(self, *, camera_path, protocol_path, source_epoch, output_dir,
                 on_update, character_asset_lock_sha256, select_track_id=None, depth_calibration_path=None,
                 inference_threads=None, metric_depth_source='yolo_depth', stereo_validation_path=None):
        if metric_depth_source not in ('yolo_depth', 'native_stereo'):
            raise ValueError('unknown_explicit_metric_depth_source')
        if metric_depth_source == 'native_stereo' and depth_calibration_path:
            raise ValueError('stereo_has_no_monocular_calibration_fallback')
        self.metric_depth_source = metric_depth_source
        self.inference_thread_budget = _InferenceThreadBudget(inference_threads)
        self.camera_path, self.protocol_path = Path(camera_path), Path(protocol_path)
        camera = json.loads(self.camera_path.read_text())
        protocol = json.loads(self.protocol_path.read_text())
        if protocol['camera_config_sha256'] != _sha(self.camera_path):
            raise ValueError('visual_protocol_camera_identity_mismatch')
        if protocol['character_asset_lock_sha256'] != character_asset_lock_sha256:
            raise ValueError('visual_protocol_character_identity_mismatch')
        self.source_epoch, self.on_update = source_epoch, on_update
        self.output_dir = Path(output_dir); self.output_dir.mkdir(parents=True, exist_ok=True)
        self.calibration = CameraCalibration(camera['K'], camera['distortion'], camera['width'],
            camera['height'], str(self.camera_path.resolve()), 'declared_mujoco_pinhole')
        mount = camera['camera_extrinsic']
        self.extrinsic = CameraExtrinsic(mount['rotation_body_camera'], mount['translation_body_camera_m'], mount['provisional'])
        if self.extrinsic.provisional: raise ValueError('render_camera_extrinsic_unverified')
        root = repository_root()
        model_path = root/protocol['depth']['model']
        detector_path = root/protocol['detector']['model']
        reid_path = default_osnet_weights_path()
        models = {'detector':detector_path,'depth':model_path,'reid':reid_path}
        for name, path in models.items():
            if not Path(path).is_file():
                raise FileNotFoundError('visual_model_unavailable:'+name+':'+str(path))
        self.model_identities = {name:dict(path=str(Path(path).resolve()),sha256=_sha(path))
                                 for name,path in models.items()}
        self.depth_calibration = (VerifiedDepthCalibration(depth_calibration_path,
            camera_path=self.camera_path, protocol_path=self.protocol_path, model_path=model_path,
            character_asset_lock_sha256=character_asset_lock_sha256)
            if depth_calibration_path else None)
        self.stereo = None
        if metric_depth_source == 'native_stereo':
            from .stereo import StereoDepthAdapter
            if inference_threads != protocol['runtime_budget']['torch_intraop_threads']:
                raise ValueError('stereo_protocol_thread_budget_mismatch')
            self.stereo = StereoDepthAdapter(camera_path=self.camera_path, protocol_path=self.protocol_path,
                character_asset_lock_sha256=character_asset_lock_sha256, validation_path=stereo_validation_path)
        self.max_age_s = float(protocol['freshness']['max_result_source_age_s'])
        self.depth = YoloDepthAdapter(YoloDepthConfig(model_path, protocol['depth']['imgsz']))
        d = protocol['detector']
        self.detector = PersonDetector(PersonDetectorConfig(detector_path, d['imgsz'], d['confidence']))
        self.tracker = PersonTracker(UltralyticsByteTrackBackend())
        reid = json.loads((root/'models/minisegway/stage7/stage7_6/config/stage7_6_reid_auto_reacquire_config.json').read_text())['auto_reacquire']
        self.master = MasterReIDSession(MasterReIDEvidence(PersonReIdentifier(
            TorchReIDOSNetBackend(reid_path))), initial_track_id=select_track_id,
            accept_threshold=reid['reid_accept_threshold'], retry_threshold=reid['reid_retry_threshold'],
            retry_interval_s=reid['retry_interval_s'], stable_updates=reid['minimum_stable_tracker_updates'])
        self.master.events = deque(maxlen=4096)
        self.pitch = CapturePitchHistory()
        self.pairer = _MonitoredSameFramePairer()
        self.stereo_pairer = _MonitoredSameFramePairer() if self.stereo is not None else None
        self.slots = {'depth': LatestFrameSlot(), 'tracking': LatestFrameSlot()}
        self.completed = queue.Queue(maxsize=64)
        self.stop = threading.Event()
        self.errors = []
        self.events = deque(maxlen=4096)
        self.channel_timings = {name:deque(maxlen=4096) for name in self.slots}
        if self.stereo is not None: self.channel_timings['stereo'] = deque(maxlen=4096)
        self.monocular_diagnostics = deque(maxlen=4096)
        self.counts = dict(submitted=0, paired=0, valid=0, unavailable=0,
            completed_capacity_faults=0, obsolete_updates=0, obsolete_pairs=0, stereo_processed=0,
            diagnostic_monocular_pairs=0)
        self._last_sequence, self._last_host = -1, -math.inf
        self._log_lock, self._state_lock = threading.Lock(), threading.Lock()
        self._publication_lock = threading.Lock()
        self._last_published_sequence = self._last_pair_sequence = -1
        self._log = (self.output_dir/'perception_updates.jsonl').open('w')
        self._closed = False
        self.workers = [threading.Thread(target=self._worker, args=(name,), name='visual-'+name)
                        for name in self.slots]
        self.workers.append(threading.Thread(target=self._pairs, name='visual-pairing'))
        for worker in self.workers: worker.start()

    def submit_bgr(self, bgr, capture):
        self.check_health()
        if (capture.source_epoch != self.source_epoch or capture.sequence <= self._last_sequence
                or capture.read_complete_host_s <= self._last_host):
            raise ValueError('render_source_restart_or_replay_requires_new_runtime')
        if capture.read_complete_host_s > time.perf_counter():
            raise ValueError('render_receipt_is_in_future_host_domain')
        self._last_sequence, self._last_host = capture.sequence, capture.read_complete_host_s
        frame = ColorFrame(bgr, capture.sequence, 'mujoco:camera:'+capture.source_epoch,
            bgr.shape[1], bgr.shape[0], capture.read_complete_host_s)
        if (frame.width, frame.height) != (self.calibration.frame_width, self.calibration.frame_height):
            raise ValueError('render_resolution_differs_from_calibration')
        # Renderer transfers ownership of this array; no second RGB serialization.
        frame.bgr.setflags(write=False)
        self.pitch.add(capture)
        for slot in self.slots.values(): slot.put(frame)
        self.counts['submitted'] += 1
        return frame

    def submit_stereo_pair(self, frame, right_bgr, capture, *, right_read_complete_host_s):
        """Called by the same render worker after the left image starts AI.

        This native call releases the GIL. No publication/authority lock is held
        during matching, and the original independent YOLO depth keeps running.
        """
        self.check_health()
        if self.stereo is None: raise ValueError('stereo_channel_not_enabled')
        metric = self.stereo.process(frame,right_bgr,capture,
            right_read_complete_host_s=right_read_complete_host_s)
        self.counts['stereo_processed'] += 1
        self.channel_timings['stereo'].append(dict(sequence=frame.sequence_id,**metric.diagnostics))
        row = dict(ready_time_s=metric.ready_host_s,**metric.diagnostics)
        try: self.completed.put_nowait(('stereo',frame,metric.depth,row))
        except queue.Full:
            self.counts['completed_capacity_faults'] += 1
            error = RuntimeError('visual_completed_queue_capacity_exhausted')
            self._fail(error)
            raise error

    def _emit(self, update):
        # Both channels may finish out of order. Serialize publication and its
        # consumer callback so an old LOST/unavailable result cannot overtake a
        # newer confirmed observation. Native authority also checks task/source
        # generations; this is the frame-result ordering boundary only.
        with self._publication_lock:
            if update.capture is not None:
                if update.capture.sequence <= self._last_published_sequence:
                    self.counts['obsolete_updates'] += 1
                    return
                self._last_published_sequence = update.capture.sequence
            record = asdict(update)
            with self._log_lock:
                self._log.write(json.dumps(record)+'\n')
            with self._state_lock:
                self.events.append(record)
                self.counts['valid' if update.motion_valid else 'unavailable'] += 1
            self.on_update(update)

    def _fail(self, error):
        with self._state_lock: self.errors.append(repr(error))
        self.stop.set()
        self.inference_thread_budget.abort()
        for slot in self.slots.values(): slot.close()

    def _worker(self, name):
        try:
            first_inference = True
            while not self.stop.is_set():
                frame = self.slots[name].get(.05)
                if frame is None:
                    if self.slots[name].closed: break
                    continue
                started_cpu, started_wall = time.thread_time(), time.perf_counter()
                actual_threads_before = (self.inference_thread_budget.actual() if first_inference
                    else self.inference_thread_budget.check())
                if name == 'depth': result, row = _depth_operation(self.depth, frame)
                else:
                    result, row = _detector_operation(self.detector, self.tracker, frame, self.master)
                    if result.master.master_track is None:
                        self._emit(PerceptionUpdate(self.pitch.capture(frame.host_receive_time_s),
                            result.master.state.value, 'master_not_visible_or_locked', row['ready_time_s'],
                            metric_depth_source=self.metric_depth_source))
                wall_s, cpu_s = time.perf_counter()-started_wall, time.thread_time()-started_cpu
                actual_threads_after = (self.inference_thread_budget.actual() if first_inference
                    else self.inference_thread_budget.check())
                self.channel_timings[name].append(dict(sequence=frame.sequence_id,
                    wall_s=wall_s,thread_cpu_s=cpu_s,
                    source_to_ready_s=row['ready_time_s']-frame.host_receive_time_s,
                    inference_threads_before=actual_threads_before,
                    inference_threads_after=actual_threads_after,
                    thread_budget_first_real_inference=first_inference))
                if first_inference:
                    first_inference = False
                    if not self.inference_thread_budget.after_first_inference(name, self.stop):
                        break
                try: self.completed.put_nowait((name, frame, result, row))
                except queue.Full:
                    self.counts['completed_capacity_faults'] += 1
                    raise RuntimeError('visual_completed_queue_capacity_exhausted')
        except BaseException as error: self._fail(error)

    def _pairs(self):
        try:
            while not self.stop.is_set():
                try: name, frame, result, row = self.completed.get(timeout=.05)
                except queue.Empty: continue
                if name == 'stereo':
                    pair = self.stereo_pairer.add_depth(frame,result,row)
                else:
                    diagnostic_pair = (self.pairer.add_depth(frame,result,row) if name == 'depth'
                        else self.pairer.add_tracking(frame,result.tracking,result.master))
                    if self.stereo_pairer is None:
                        pair = diagnostic_pair
                    else:
                        # Original YOLODepth still has exact same-source Master
                        # pairing and logs. It is not a metric authorization
                        # dependency or fallback for this explicit stereo mode.
                        if diagnostic_pair is not None:
                            sample = representative_master_depth(diagnostic_pair.master,diagnostic_pair.depth)
                            self.counts['diagnostic_monocular_pairs'] += 1
                            self.monocular_diagnostics.append(dict(sequence=diagnostic_pair.color.sequence_id,
                                source_time_s=diagnostic_pair.color.host_receive_time_s,
                                master_state=diagnostic_pair.master.state.value,
                                raw_model_depth=sample.depth_m if sample else None,
                                ready_time_s=time.perf_counter(),motion_authority=False))
                        pair = (self.stereo_pairer.add_tracking(frame,result.tracking,result.master)
                            if name == 'tracking' else None)
                if pair is not None: self._publish_pair(pair)
        except BaseException as error: self._fail(error)

    def _publish_pair(self, pair):
        if pair.color.sequence_id <= self._last_pair_sequence:
            self.counts['obsolete_pairs'] += 1
            return
        self._last_pair_sequence = pair.color.sequence_id
        self.counts['paired'] += 1
        frame = pair.color
        capture = self.pitch.capture(frame.host_receive_time_s)
        sample = representative_master_depth(pair.master,pair.depth)
        reason, observation, depth_m = 'master_or_model_depth_unavailable', None, None
        verified = (self.stereo.validation is not None if self.stereo is not None
            else self.depth_calibration is not None)
        if sample is not None:
            depth_m = (self.depth_calibration.apply(sample.depth_m) if self.depth_calibration else sample.depth_m)
            if depth_m is not None:
                convention = self.depth_calibration.convention if self.depth_calibration else 'axial_z'
                point = back_project_camera_point(sample.anchor_uv_px, depth_m,
                    self.calibration, depth_convention=convention)
                observation = camera_to_robot_observation(point, frame, pair.master.bound_track_id,
                    self.extrinsic,self.pitch,depth_convention=convention)
                if observation is not None:
                    observation = replace(observation, depth_convention_provisional=not verified)
                reason = ('capture_alignment_unavailable' if observation is None else
                    'depth_semantics_unverified' if not verified else
                    'visual_observation_ready')
        ready = time.perf_counter()
        if reason == 'visual_observation_ready' and (capture is None or
                not 0 <= ready-capture.acquisition_begin_host_s <= self.max_age_s):
            reason = 'visual_source_stale'
        valid = reason == 'visual_observation_ready'
        self._emit(PerceptionUpdate(capture, pair.master.state.value, reason, ready,
            observation, sample.depth_m if sample and self.stereo is None else None,
            depth_m if verified else None, valid,self.metric_depth_source))

    def check_health(self):
        if self.errors: raise RuntimeError('rendered_perception_failed:'+self.errors[0])
        if self._closed: raise RuntimeError('rendered_perception_closed')

    def close(self):
        if self._closed: return
        self._closed = True
        self.stop.set()
        self.inference_thread_budget.abort()
        for slot in self.slots.values(): slot.close()
        for worker in self.workers: worker.join(timeout=10.)
        alive = [w.name for w in self.workers if w.is_alive()]
        if alive:
            # Do not destroy models/log handles underneath a running inference.
            raise RuntimeError('visual_workers_failed_to_stop:'+','.join(alive))
        self.tracker.close()
        with self._log_lock: self._log.close()
        report = dict(counts=self.counts, errors=self.errors, pending_results=self.completed.qsize(),
            slots={name:dict(submitted=s.submitted_count,taken=s.taken_count,overwritten=s.overwritten_count,
                pending=s.pending_count) for name,s in self.slots.items()},
            capture_history_overwritten=self.pitch.overwritten, events_retained=len(self.events),
            master_events=list(self.master.events),channel_timings={k:list(v) for k,v in self.channel_timings.items()},
            timing_capacity_per_channel=4096, hardware_execution_ready=False,
            model_identities=self.model_identities,
            metric_depth_source=self.metric_depth_source,
            original_yolo_pairing=self.pairer.diagnostics(),
            stereo_pairing=self.stereo_pairer.diagnostics() if self.stereo_pairer is not None else None,
            diagnostic_monocular_pairs=list(self.monocular_diagnostics),
            stereo_validation_identity=(self.stereo.validation.identity if self.stereo is not None
                and self.stereo.validation is not None else None),
            inference_thread_budget=self.inference_thread_budget.report(),
            depth_calibration_identity=self.depth_calibration.identity if self.depth_calibration else None)
        (self.output_dir/'perception_runtime.json').write_text(json.dumps(report,indent=2)+'\n')
        if self.errors: raise RuntimeError('rendered_perception_failed:'+self.errors[0])
