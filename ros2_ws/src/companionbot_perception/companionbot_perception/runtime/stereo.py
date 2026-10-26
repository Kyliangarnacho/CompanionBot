"""Explicit image-only stereo metric channel, separate from YOLO depth.

The C++ processor accepts two RGB arrays and declared camera geometry. Python
only composes low-rate frame contracts and checks an offline validation artifact.
No online fit, previous-depth substitution or monocular fallback is provided.
"""
from dataclasses import dataclass
import hashlib
import itertools
import json
import math
from pathlib import Path
import time

import numpy as np

from companionbot_contracts.paths import repository_root
from companionbot_perception import _native_stereo as native
from companionbot_perception.core.depth import DepthFrame


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def algorithm_identity():
    """Bind certificates to this actual native implementation and OpenCV ABI."""
    root = repository_root()
    base = Path('ros2_ws/src/companionbot_perception/native')
    files = (base/'include/companionbot_perception/stereo_depth.hpp',
             base/'src/stereo_depth.cpp', base/'src/bindings.cpp')
    # Inspect only this process's loaded objects. Source hashes alone cannot
    # prove that a stale overlay ELF implements the current source tree.
    mapped = set()
    for line in Path('/proc/self/maps').read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) == 6 and fields[5].startswith('/'):
            mapped.add(fields[5])
    binaries = dict(module=str(Path(native.__file__).resolve()))
    for role, prefix in (('stereo_library','libcompanionbot_stereo_depth.so'),
            ('opencv_core','libopencv_core.so.'),('opencv_imgproc','libopencv_imgproc.so.'),
            ('opencv_calib3d','libopencv_calib3d.so.')):
        matches = {str(Path(p).resolve()) for p in mapped if Path(p).name.startswith(prefix)}
        if len(matches) != 1 or any(p.endswith(' (deleted)') for p in matches):
            raise RuntimeError('stereo_loaded_library_identity_unavailable:'+role)
        binaries[role] = next(iter(matches))
    return dict(abi_version=native.ABI_VERSION,
        opencv_compiled_version=native.OPENCV_COMPILED_VERSION,
        opencv_runtime_version=native.OPENCV_RUNTIME_VERSION,
        source_sha256={str(path):sha256(root/path) for path in files},
        loaded_binaries={role:dict(path=path,sha256=sha256(path)) for role,path in binaries.items()})


def heldout_pose_keys(protocol):
    """Canonical complete Cartesian positions, independent of result count."""
    names = ('distance_m','lateral_m','actor_yaw_rad','animation_time_s')
    return sorted(json.dumps(dict(zip(names, map(float, values))),sort_keys=True,separators=(',',':'),allow_nan=False)
        for values in itertools.product(*(protocol['heldout_split'][name] for name in names)))


def inference_model_identities(protocol):
    # Matching accuracy is assessed inside the original AI-selected torso ROI;
    # detector/ReID checkpoint changes must invalidate that validation too.
    from companionbot_perception.core.person_reid import default_osnet_weights_path
    root = repository_root()
    paths = (root/protocol['detector']['model'],root/protocol['depth']['model'],default_osnet_weights_path())
    return {str(Path(p).resolve().relative_to(root)):sha256(p) for p in paths}


def make_native_config(camera, protocol, *, calibration_id):
    """Map one frozen camera/matcher document into the ordinary C++ library."""
    if camera.get('sensor_kind') != 'stereo_rgb_simulation':
        raise ValueError('explicit_stereo_camera_required')
    values = protocol['stereo']
    if values['opencv_native_version'] != native.OPENCV_COMPILED_VERSION:
        raise ValueError('stereo_opencv_version_mismatch')
    if values['mode'] != 'MODE_SGBM':
        raise ValueError('unsupported_stereo_matcher_mode')
    config = native.StereoConfig()
    config.width, config.height = camera['width'], camera['height']
    k = np.asarray(camera['K'], dtype=float)
    if (k.shape != (3,3) or not np.isfinite(k).all()
            or k[0,1] != 0 or k[1,0] != 0 or not np.array_equal(k[2],[0.,0.,1.])):
        raise ValueError('invalid_stereo_camera_intrinsics')
    config.fx, config.fy, config.cx, config.cy = k[0,0], k[1,1], k[0,2], k[1,2]
    config.baseline_m, config.calibration_id = camera['baseline_m'], calibration_id
    config.right_rotation_from_left = np.asarray(camera['right_from_left']['rotation']).reshape(9).tolist()
    config.right_translation_from_left = camera['right_from_left']['translation_m']
    for name, key in (('min_disparity','min_disparity'),('num_disparities','num_disparities'),
            ('block_size','block_size'),('p1','P1'),('p2','P2'),('disp12_max_diff','disp12_max_diff_px'),
            ('pre_filter_cap','pre_filter_cap'),('uniqueness_ratio','uniqueness_ratio'),
            ('speckle_window_size','speckle_window_size'),('speckle_range','speckle_range'),
            ('lr_max_error_px','left_right_consistency_max_error_px')):
        setattr(config, name, values[key])
    config.texture_min_gradient_rms = values['texture_gate']['minimum_rms_gray_code_per_pixel']
    # Native constructor independently rejects non-parallel R/T, invalid K,
    # unsupported dimensions and parameters. Distortion is explicitly zero.
    distortion = np.asarray(camera['distortion'], dtype=float)
    if distortion.shape != (5,) or np.any(distortion != 0):
        raise ValueError('stereo_profile_requires_explicit_rectification')
    mounts = (camera['camera_extrinsic'], camera['right_camera']['camera_extrinsic'])
    transforms = []
    for mount in mounts:
        rotation = np.asarray(mount['rotation_body_camera'],dtype=float)
        translation = np.asarray(mount['translation_body_camera_m'],dtype=float)
        if (mount['provisional'] is not False or rotation.shape != (3,3) or translation.shape != (3,)
                or not np.isfinite(rotation).all() or not np.isfinite(translation).all()
                or not np.allclose(rotation.T@rotation,np.eye(3),rtol=0,atol=1e-12)
                or not math.isclose(np.linalg.det(rotation),1.,rel_tol=0,abs_tol=1e-12)):
            raise ValueError('invalid_stereo_body_extrinsic')
        transforms.append((rotation,translation))
    (rl,tl),(rr,tr) = transforms
    if (camera['name'] == camera['right_camera']['name']
            or camera['frame_id'] == camera['right_camera']['frame_id']
            or not np.allclose(rr.T@rl,np.asarray(config.right_rotation_from_left).reshape(3,3),rtol=0,atol=1e-12)
            or not np.allclose(rr.T@(tl-tr),config.right_translation_from_left,rtol=0,atol=1e-12)):
        raise ValueError('stereo_relative_and_body_extrinsics_differ')
    return config


class VerifiedStereoValidation:
    """A fixed-grid result is an authorization prerequisite, never a scale fit."""
    def __init__(self, path, *, camera_path, protocol_path, character_asset_lock_sha256):
        value = json.loads(Path(path).read_text())
        protocol = json.loads(Path(protocol_path).read_text())
        split = protocol['heldout_split']
        required = math.prod(len(split[k]) for k in
            ('distance_m','lateral_m','actor_yaw_rad','animation_time_s'))
        heldout, gate = value['heldout'], protocol['measurement_acceptance']
        if (value.get('schema_version') != 1 or value.get('depth_source') != 'native_opencv_stereo'
                or value.get('depth_convention') != 'axial_z'
                or value.get('camera_config_sha256') != sha256(camera_path)
                or protocol['camera_config_sha256'] != sha256(camera_path)
                or value.get('protocol_sha256') != sha256(protocol_path)
                or value.get('algorithm_identity') != algorithm_identity()
                or value.get('models_sha256') != inference_model_identities(protocol)
                or protocol['character_asset_lock_sha256'] != character_asset_lock_sha256
                or value.get('character_asset_lock_sha256') != character_asset_lock_sha256
                or heldout.get('passed') is not True
                or heldout.get('available_samples') != required
                or sorted(heldout.get('distinct_pose_keys',[])) != heldout_pose_keys(protocol)
                or not 0 <= heldout.get('rmse_m', math.nan) <= gate['heldout_rmse_m']
                or not 0 <= heldout.get('p95_absolute_error_m', math.nan) <= gate['heldout_p95_absolute_error_m']
                or value.get('online_ground_truth_calibration') is not False
                or value.get('depth_scale_fit') is not False):
            raise ValueError('stereo_validation_not_validated_for_this_runtime')
        self.identity = sha256(path)


@dataclass(frozen=True)
class StereoMetricResult:
    depth: DepthFrame
    ready_host_s: float
    diagnostics: dict


class StereoDepthAdapter:
    """One low-rate native call per physical image pair; no control tick API."""
    def __init__(self, *, camera_path, protocol_path, character_asset_lock_sha256,
                 validation_path=None):
        self.camera_path, self.protocol_path = Path(camera_path), Path(protocol_path)
        camera, protocol = (json.loads(p.read_text()) for p in (self.camera_path,self.protocol_path))
        if (protocol['camera_config_sha256'] != sha256(self.camera_path)
                or protocol['character_asset_lock_sha256'] != character_asset_lock_sha256):
            raise ValueError('stereo_protocol_source_identity_mismatch')
        self.calibration_id = sha256(self.camera_path)
        self.processor = native.StereoDepthProcessor(make_native_config(camera,protocol,
            calibration_id=self.calibration_id))
        self.validation = (VerifiedStereoValidation(validation_path,
            camera_path=self.camera_path, protocol_path=self.protocol_path,
            character_asset_lock_sha256=character_asset_lock_sha256) if validation_path else None)

    def process(self, frame, right_bgr, capture, *, right_read_complete_host_s):
        if ((frame.source_id, frame.sequence_id, frame.host_receive_time_s) != (
                'mujoco:camera:'+capture.source_epoch, capture.sequence,capture.read_complete_host_s)
                or not math.isfinite(right_read_complete_host_s)
                or not capture.read_complete_host_s <= right_read_complete_host_s <= time.perf_counter()):
            raise ValueError('stereo_left_frame_capture_mismatch')
        stamp = native.StereoFrameStamp(capture.source_epoch,capture.sequence,
            capture.simulation_time_s,self.calibration_id,'mujoco_sim')
        result = self.processor.process(frame.bgr,right_bgr,stamp,stamp)
        ready = time.perf_counter()
        depth = DepthFrame(frame.source_id, frame.sequence_id, frame.host_receive_time_s,
            frame.width, frame.height, result.depth_m, 'native.opencv_stereo',
            'OpenCV'+native.OPENCV_COMPILED_VERSION+':StereoSGBM:'+self.calibration_id)
        diagnostics = dict(result.diagnostics)
        diagnostics.update(right_read_complete_host_s=right_read_complete_host_s,
            ready_host_s=ready,source_simulation_time_s=capture.simulation_time_s,
            acquisition_to_metric_ready_s=ready-capture.acquisition_begin_host_s,
            validation_identity=self.validation.identity if self.validation else None)
        return StereoMetricResult(depth,ready,diagnostics)
