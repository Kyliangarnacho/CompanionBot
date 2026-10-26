"""Metric authorization boundaries; fixtures do not certify rendered accuracy."""
from collections import deque
from dataclasses import replace
import copy
import json
from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from companionbot_perception.core.camera import ColorFrame
from companionbot_perception.core.camera_calibration import CameraCalibration
from companionbot_perception.core.depth import DepthFrame
from companionbot_perception.core.master_selection import MasterState, MasterTrackingFrame
from companionbot_perception.core.person_tracker import TrackedPerson
from companionbot_perception.core.robot_geometry import CameraExtrinsic
from companionbot_perception.runtime.rendered import CapturePitchHistory, RenderCapture, RenderedPerceptionRuntime
from companionbot_perception.runtime.session import _PairedFrame, _SameFramePairer
from companionbot_perception.runtime.stereo import (
    VerifiedStereoValidation, algorithm_identity, heldout_pose_keys, inference_model_identities,
    make_native_config, sha256,
)

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'models/minisegway/native_runtime/config'


def documents():
    return [json.loads((CONFIG/name).read_text()) for name in ('stereo_camera.json','stereo_depth_protocol.json')]


@pytest.fixture
def certificate(tmp_path):
    camera, protocol = documents()
    camera_path, protocol_path = tmp_path/'camera.json', tmp_path/'protocol.json'
    camera_path.write_text(json.dumps(camera))
    protocol['camera_config_sha256'] = sha256(camera_path)
    protocol_path.write_text(json.dumps(protocol))
    # Explicit synthetic artifact for validator rejection tests only; it is
    # never written to the production config or used to authorize a robot.
    value = dict(schema_version=1,depth_source='native_opencv_stereo',depth_convention='axial_z',
        camera_config_sha256=sha256(camera_path),protocol_sha256=sha256(protocol_path),
        character_asset_lock_sha256=protocol['character_asset_lock_sha256'],
        algorithm_identity=algorithm_identity(),online_ground_truth_calibration=False,depth_scale_fit=False,
        models_sha256=inference_model_identities(protocol),
        heldout=dict(passed=True,available_samples=32,distinct_pose_keys=heldout_pose_keys(protocol),
            rmse_m=.02,p95_absolute_error_m=.07))
    path = tmp_path/'unit_validation.json'
    return path,value,dict(camera_path=camera_path,protocol_path=protocol_path,
        character_asset_lock_sha256=protocol['character_asset_lock_sha256'])


@pytest.mark.parametrize('fault', ['duplicate_position','missing_position','partial_coverage','stale_elf',
    'failed','too_large_rmse','too_large_p95','nonfinite','online_gt','depth_fit','wrong_camera','changed_detector'])
def test_stereo_certificate_cannot_count_duplicates_or_accept_changed_binary(certificate,fault):
    path,value,kwargs = certificate
    if fault == 'duplicate_position':
        value['heldout']['distinct_pose_keys'][-1] = value['heldout']['distinct_pose_keys'][0]
    elif fault == 'missing_position': value['heldout']['distinct_pose_keys'].pop()
    elif fault == 'partial_coverage': value['heldout']['available_samples'] = 31
    elif fault == 'stale_elf': value['algorithm_identity']['loaded_binaries']['stereo_library']['sha256'] = 'stale'
    elif fault == 'failed': value['heldout']['passed'] = False
    elif fault == 'too_large_rmse': value['heldout']['rmse_m'] = .0300001
    elif fault == 'too_large_p95': value['heldout']['p95_absolute_error_m'] = .0900001
    elif fault == 'nonfinite': value['heldout']['rmse_m'] = float('nan')
    elif fault == 'online_gt': value['online_ground_truth_calibration'] = True
    elif fault == 'depth_fit': value['depth_scale_fit'] = True
    elif fault == 'wrong_camera': value['camera_config_sha256'] = 'changed_camera'
    elif fault == 'changed_detector': value['models_sha256'][next(iter(value['models_sha256']))] = 'changed_checkpoint'
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError,match='not_validated'):
        VerifiedStereoValidation(path,**kwargs)


def test_stereo_certificate_identity_is_actual_loaded_elf_and_complete_position_set(certificate):
    path,value,kwargs = certificate
    path.write_text(json.dumps(value))
    assert VerifiedStereoValidation(path,**kwargs).identity == sha256(path)
    identity = algorithm_identity()
    assert len(identity['loaded_binaries']) == 5
    for binary in identity['loaded_binaries'].values():
        assert Path(binary['path']).is_file()
        assert binary['sha256'] == sha256(binary['path'])
    assert len(set(value['heldout']['distinct_pose_keys'])) == 32


@pytest.mark.parametrize('fault',['skew','bottom_row','distortion','right_mount','provisional','same_id'])
def test_stereo_geometry_rejects_unrepresented_intrinsics_or_body_mount(fault):
    camera,protocol = documents()
    if fault == 'skew': camera['K'][0][1] = .001
    elif fault == 'bottom_row': camera['K'][2][0] = .001
    elif fault == 'distortion': camera['distortion'] = []
    elif fault == 'right_mount': camera['right_camera']['camera_extrinsic']['translation_body_camera_m'][1] += .001
    elif fault == 'provisional': camera['camera_extrinsic']['provisional'] = True
    elif fault == 'same_id': camera['right_camera']['frame_id'] = camera['frame_id']
    with pytest.raises(ValueError): make_native_config(camera,protocol,calibration_id='unit')


def pair_fixture():
    now = time.perf_counter()
    capture = RenderCapture('unit',1,10.,now-.05,now-.03,0.,10.,True)
    color = ColorFrame(np.zeros((100,100,3),np.uint8),1,'mujoco:camera:unit',100,100,capture.read_complete_host_s)
    track = TrackedPerson(1,(20,0,80,100),.9,0,'person')
    master = MasterTrackingFrame(color.source_id,1,color.host_receive_time_s,100,100,MasterState.LOCKED,1,track)
    depth = DepthFrame(color.source_id,1,color.host_receive_time_s,100,100,
        np.full((100,100),2.,np.float32),'unit_stereo','synthetic_fixture')
    meta = SimpleNamespace(source_id=color.source_id,source_sequence_id=1,source_time_s=color.host_receive_time_s,
        frame_width=100,frame_height=100)
    return capture,_PairedFrame(color,depth,{},meta,master)


@pytest.mark.parametrize('fault',[None,'uncertified','stale','no_pitch_history','lost','insufficient_depth'])
def test_stereo_geometry_uses_original_pitch_identity_and_age_gates(fault):
    capture,pair = pair_fixture()
    runtime = RenderedPerceptionRuntime.__new__(RenderedPerceptionRuntime)
    runtime._last_pair_sequence = -1
    runtime.counts = dict(paired=0,obsolete_pairs=0)
    runtime.stereo = SimpleNamespace(validation=None if fault == 'uncertified' else object())
    runtime.metric_depth_source, runtime.depth_calibration = 'native_stereo',None
    runtime.max_age_s = .2
    runtime.calibration = CameraCalibration([[100,0,50],[0,100,50],[0,0,1]],np.zeros(5),100,100,'unit','pinhole')
    runtime.extrinsic = CameraExtrinsic([[0,0,1],[-1,0,0],[0,-1,0]],[.08,.075,.35],False)
    runtime.pitch = CapturePitchHistory()
    if fault == 'stale': capture = replace(capture,acquisition_begin_host_s=time.perf_counter()-1.)
    if fault != 'no_pitch_history': runtime.pitch.add(capture)
    if fault == 'lost':
        pair = replace(pair,master=replace(pair.master,state=MasterState.LOST,master_track=None))
    if fault == 'insufficient_depth':
        invalid = np.full((100,100),np.nan,np.float32);invalid[49,46:55] = 2.
        pair = replace(pair,depth=replace(pair.depth,depth_m=invalid))
    updates = [];runtime._emit = updates.append
    runtime._publish_pair(pair)
    assert len(updates) == 1
    update = updates[0]
    assert update.motion_valid is (fault is None)
    assert update.metric_depth_source == 'native_stereo'
    assert update.raw_model_depth is None
    if fault is None:
        assert update.observation.x_forward_m == pytest.approx(2.08)
        assert update.observation.y_left_m == pytest.approx(.075)
        assert update.observation.depth_convention_provisional is False


@pytest.mark.parametrize('order',[('depth','tracking','stereo'),('stereo','tracking','depth'),('depth','tracking')])
def test_stereo_channel_neither_waits_for_nor_falls_back_to_monocular_depth(order):
    capture,pair = pair_fixture()
    runtime = RenderedPerceptionRuntime.__new__(RenderedPerceptionRuntime)
    runtime.pairer,runtime.stereo_pairer = _SameFramePairer(),_SameFramePairer()
    runtime.stop = threading.Event()
    runtime.counts = dict(diagnostic_monocular_pairs=0)
    runtime.monocular_diagnostics = deque(maxlen=16)
    metric_pairs,errors = [],[]
    runtime._publish_pair = metric_pairs.append
    runtime._fail = errors.append
    mono = replace(pair.depth,depth_m=np.full((100,100),100.,np.float32),backend_name='original_yolo_diagnostic')
    tracking = SimpleNamespace(tracking=pair.tracking,master=pair.master)
    results = iter((name,pair.color,{'depth':mono,'tracking':tracking,'stereo':pair.depth}[name],{}) for name in order)
    def get(timeout):
        try: return next(results)
        except StopIteration:
            runtime.stop.set()
            raise queue.Empty
    runtime.completed = SimpleNamespace(get=get)
    runtime._pairs()
    assert not errors
    assert runtime.counts['diagnostic_monocular_pairs'] == 1
    assert runtime.monocular_diagnostics[0]['raw_model_depth'] == 100.
    assert len(metric_pairs) == int('stereo' in order)
    if metric_pairs: assert np.all(metric_pairs[0].depth.depth_m == 2.)
