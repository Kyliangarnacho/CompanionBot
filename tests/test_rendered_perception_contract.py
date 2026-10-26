"""Pure source-time/calibration fault checks; no pretend vision/hardware output."""
import hashlib
import json

import pytest

from companionbot_perception.runtime.rendered import (
    CapturePitchHistory, RenderCapture, VerifiedDepthCalibration,
)


def capture(sequence, *, simulation=1., pitch_time=1., valid=True):
    return RenderCapture('camera_epoch',sequence,simulation,100.+sequence,100.01+sequence,
                         .12,pitch_time,valid)


def test_pitch_uses_exact_sensorized_acquisition_record_and_bounded_coverage():
    history=CapturePitchHistory(2)
    first=capture(0); history.add(first)
    aligned=history.at_source_time(first.read_complete_host_s)
    assert aligned.theta_rad==.12 and aligned.pitch_time_s==100.
    assert aligned.alignment_basis=='capture_time_mapped_to_host'
    assert aligned.source_time_s==first.read_complete_host_s
    assert aligned.provisional is False
    assert history.at_source_time(first.read_complete_host_s+1e-7) is None
    history.add(capture(1,pitch_time=.998))
    assert history.at_source_time(101.01) is None
    history.add(capture(2,valid=False))
    assert history.at_source_time(102.01) is None
    assert history.at_source_time(first.read_complete_host_s) is None
    assert history.overwritten==1
    with pytest.raises(ValueError,match='duplicate'): history.add(capture(2))


@pytest.mark.parametrize('changes',[dict(simulation=-1),dict(pitch_time=float('nan')),
                                   dict(sequence=True),dict(valid=1)])
def test_malformed_capture_rejected(changes):
    values=dict(sequence=0); values.update(changes)
    with pytest.raises(ValueError): capture(**values)


def artifacts(tmp_path):
    camera,protocol,model=[tmp_path/name for name in ('camera.json','protocol.json','model.pt')]
    camera.write_text('{}'); model.write_bytes(b'unit_test_model_identity')
    protocol.write_text(json.dumps({'measurement_acceptance':{
        'heldout_rmse_m':.03,'heldout_p95_absolute_error_m':.09},
        'character_asset_lock_sha256':'unit_asset_hash',
        'heldout_split':dict(distance_m=[1.,2.,3.,4.],lateral_m=[-.3,.3],
            actor_yaw_rad=[-.2,.2],animation_time_s=[.5,1.])}))
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    value=dict(camera_config_sha256=sha(camera),protocol_sha256=sha(protocol),model_sha256=sha(model),
        character_asset_lock_sha256='unit_asset_hash',
        heldout=dict(passed=True,available_samples=32,rmse_m=.02,p95_absolute_error_m=.07),
        coefficients=dict(log_scale=1.,log_bias=0.),depth_convention='axial_z')
    certificate=tmp_path/'certificate.json';certificate.write_text(json.dumps(value))
    return certificate,value,dict(camera_path=camera,protocol_path=protocol,model_path=model,
        character_asset_lock_sha256='unit_asset_hash')


def test_only_verified_depth_identity_can_supply_metric_mapping(tmp_path):
    certificate,value,paths=artifacts(tmp_path)
    mapping=VerifiedDepthCalibration(certificate,**paths)
    assert mapping.apply(1.5)==pytest.approx(1.5)
    assert mapping.apply(0.) is None
    assert mapping.convention=='axial_z'
    paths['model_path'].write_bytes(b'changed_checkpoint')
    with pytest.raises(ValueError,match='not_validated'): VerifiedDepthCalibration(certificate,**paths)


@pytest.mark.parametrize('field,value',[('passed',False),('available_samples',0),
    ('available_samples',10),
    ('rmse_m',.030001),('p95_absolute_error_m',.090001),('rmse_m',float('nan'))])
def test_failed_or_empty_heldout_never_authorizes_motion(tmp_path,field,value):
    certificate,data,paths=artifacts(tmp_path);data['heldout'][field]=value
    certificate.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='not_validated'): VerifiedDepthCalibration(certificate,**paths)


def test_old_lost_result_cannot_overtake_newer_confirmed_publication():
    """Exercise the actual shared publication boundary without model inference."""
    from collections import deque
    import io
    import threading
    from companionbot_perception.runtime.rendered import RenderedPerceptionRuntime, PerceptionUpdate
    runtime=RenderedPerceptionRuntime.__new__(RenderedPerceptionRuntime)
    runtime._publication_lock=threading.Lock()
    runtime._log_lock=threading.Lock();runtime._state_lock=threading.Lock()
    runtime._last_published_sequence=-1
    runtime.counts=dict(obsolete_updates=0,valid=0,unavailable=0)
    runtime.events=deque(maxlen=16);runtime._log=io.StringIO()
    published=[];runtime.on_update=published.append
    runtime._emit(PerceptionUpdate(capture(12),'LOCKED','ready',113.,motion_valid=True))
    runtime._emit(PerceptionUpdate(capture(11),'LOST','old_lost',114.))
    runtime._emit(PerceptionUpdate(capture(12),'LOST','duplicate_lost',114.))
    assert [u.capture.sequence for u in published]==[12]
    assert runtime.counts['obsolete_updates']==2
    runtime._emit(PerceptionUpdate(capture(13),'LOST','fresh_lost',115.))
    assert [u.motion_valid for u in published]==[True,False]
