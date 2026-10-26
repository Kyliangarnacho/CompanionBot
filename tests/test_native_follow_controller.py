"""Native whole-follow composition tests using declared sensor/software inputs.

These fixtures never claim visual accuracy: real RGB/model acceptance is tested
separately. Their purpose is to prove the new production algorithm owner uses
consumed RobotState history and the frozen algorithms without a Python loop.
"""
import json
import math
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

import pytest

from companionbot_contracts.packets import PROTOCOL_VERSION, RobotStatePacket, TargetObservationPacket
from companionbot_follow import _native_runtime as native
from companionbot_follow.core.radial_velocity_kf import PiRadialObservationPipeline, PiRobotStateHistory, RadialVelocityKFConfig, RadialVelocityKalmanFilter
from companionbot_follow.core.follow_governor import FollowGovernor, FollowGovernorConfig
from companionbot_follow.core.bearing_yaw import BearingYawServo

ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT/'models/minisegway/stage6/config/stage6_4_2d_follow_config.json').read_text())
RADIAL = json.loads((ROOT/'models/minisegway/stage6/config/stage6_5_radial_kf_config.json').read_text())


def composition():
    config = native.FollowAuthorityConfig(CONFIG['governor']['max_abs_velocity_m_s'],
        CONFIG['yaw']['max_rate_rad_s'], .3, CONFIG['yaw']['freshness_timeout_s'], .35, .15)
    authority = native.MotionAuthority('bookstore', 'v1', follow_config=config)
    authority.update_health(True, '', 10.)
    authority.update_mcu_status('NORMAL', '', 10.)
    authority.update_follow_obstacle(10., 5.1, 5.1, 10.)
    authority.update_follow_braking(.5, 10.)
    yaw = CONFIG['yaw']
    controller = native.FollowController(authority, native.RadialKFConfig(**RADIAL['kf']),
        native.FollowGovernorConfig(**CONFIG['governor']), RADIAL['state_history_retention_s'],
        yaw['gain_s_inv'], yaw['max_rate_rad_s'], yaw['deadband_rad'])
    return authority, controller


def control(authority, command, sequence, source=5.1, receipt=10., task='task'):
    return authority.control(task, 'agent', sequence, command, 1.2,
        'bookstore', 'v1', source, receipt)


def add_history(controller, begin=4.8, end=5.3):
    for sequence in range(round((end-begin)/.05)+1):
        assert controller.accept_state(native.FollowRobotState(sequence, begin+sequence*.05, .1))


def ready(authority, controller):
    add_history(controller)
    result = controller.accept_measurement('camera', 1, 5.02, 5.1, 10., 1.8, 0., True, '')
    assert result.accepted and not result.command_accepted
    assert result.reason == 'follow_readiness_only'
    assert control(authority, 'CHECK_FOLLOW', 1)['accepted']
    assert control(authority, 'ACQUIRE_FOLLOW', 2)['accepted']
    return result


def test_readiness_requires_consumed_sensor_history_and_never_grants_task():
    authority, controller = composition()
    result = controller.accept_measurement('camera', 1, 5.02, 5.1, 10., 1.8, 0., True, '')
    assert not result.accepted and result.reason == 'follow_state_history_unavailable'
    assert not control(authority, 'CHECK_FOLLOW', 1)['accepted']
    add_history(controller)
    result = controller.accept_measurement('camera', 2, 5.07, 5.15, 10.05, 1.8, 0., True, '')
    assert result.accepted and not result.command_accepted
    assert not authority.snapshot(False)['authorized']
    assert authority.sample(5.15, 10.05).linear_velocity_target_m_s == 0.


def test_new_task_starts_original_kf_and_governor_with_postgrant_image_only():
    authority, controller = composition(); ready(authority, controller)
    result = controller.accept_measurement('camera', 1, 5.02, 5.1, 10., 9., 0., True, '')
    assert not result.accepted and result.reason == 'follow_pregrant_observation'
    result = controller.accept_measurement('camera', 2, 5.12, 5.18, 10.08, 1.8, 0., True, '')
    assert result.command_accepted
    assert result.radial.estimate.master_radial_velocity_hat_m_s == 0.
    assert result.governor.intent.linear_velocity_target_m_s == pytest.approx((1.8-1.1)/3.)
    assert authority.sample(5.18, 10.08).linear_velocity_target_m_s == pytest.approx((1.8-1.1)/3.)
    snapshot = controller.snapshot()
    assert snapshot['algorithm_resets'] == 2
    assert snapshot['state_history']['retained_samples'] > 2


def test_invalid_measurement_revokes_before_latched_governor_can_be_used():
    authority, controller = composition(); ready(authority, controller)
    assert controller.accept_measurement('camera', 2, 5.12, 5.18, 10.08, 1.8, 0., True, '').command_accepted
    assert authority.sample(5.18, 10.08).linear_velocity_target_m_s > 0.
    result = controller.accept_measurement('camera', 3, 5.17, 5.23, 10.13, 0., 0., False, 'master_lost')
    assert not result.command_accepted and result.reason == 'master_lost'
    assert authority.take_stop()[1] == 'master_lost'
    assert not authority.snapshot(False)['authorized']
    assert authority.sample(5.23, 10.13).linear_velocity_target_m_s == 0.
    result = controller.accept_measurement('camera', 4, 5.22, 5.28, 10.18, 1.8, 0., True, '')
    assert result.accepted and not result.command_accepted  # readiness only
    assert not control(authority, 'RENEW_FOLLOW', 3, source=5.28, receipt=10.18)['accepted']


def test_camera_epoch_change_revokes_and_retired_frames_do_not_erase_current_filter():
    authority, controller = composition(); ready(authority, controller)
    assert controller.accept_measurement('camera', 2, 5.12, 5.18, 10.08, 1.8, 0., True, '').command_accepted
    result = controller.accept_measurement('new_camera', 0, 5.17, 5.23, 10.13, 1.8, 0., True, '')
    assert result.accepted and not result.command_accepted
    assert authority.take_stop()[1] == 'follow_source_epoch_changed'
    assert not authority.snapshot(False)['authorized']
    before = controller.snapshot()
    result = controller.accept_measurement('camera', 999, 5.22, 5.28, 10.18, 7., 0., True, '')
    assert not result.accepted and result.reason == 'follow_observation_identity_rejected'
    assert controller.snapshot()['camera_epoch'] == 'new_camera'
    assert controller.snapshot()['governor'] == before['governor']


def test_complete_follow_composition_matches_python_frozen_algorithms_500_measurements():
    authority, controller = composition(); ready(authority, controller)
    history = PiRobotStateHistory()
    pipeline = PiRadialObservationPipeline(RadialVelocityKalmanFilter(RadialVelocityKFConfig(**RADIAL['kf'])), history)
    estimates = {}
    governor = FollowGovernor(FollowGovernorConfig(**CONFIG['governor']), radial_estimate_provider=lambda obs: estimates[obs.sequence_id])
    yaw = BearingYawServo(**{k: CONFIG['yaw'][k] for k in ('gain_s_inv', 'max_rate_rad_s', 'deadband_rad')})
    # The controller already consumed these public packets before grant.
    for sequence in range(11):
        t = 4.8 + sequence*.05
        history.accept(RobotStatePacket(PROTOCOL_VERSION, sequence, t, .1, 0., 0., 'NORMAL'))
    state_sequence, next_state = 10, 5.35
    control_sequence = 2
    for index in range(500):
        sequence, capture = index+2, 5.12+index*.05
        source, receipt = capture+.06, 10.08+index*.05
        authority.update_health(True, '', receipt)
        authority.update_follow_obstacle(10., source, source, receipt)
        authority.update_follow_braking(.5, receipt)
        while next_state <= source+1e-9:
            state_sequence += 1
            velocity = .1+.03*math.sin(next_state*.7)
            assert controller.accept_state(native.FollowRobotState(state_sequence, next_state, velocity))
            history.accept(RobotStatePacket(PROTOCOL_VERSION, state_sequence, next_state, velocity, 0., 0., 'NORMAL'))
            next_state += .05
        if index%4 == 0:
            control_sequence += 1
            assert control(authority, 'RENEW_FOLLOW', control_sequence, source, receipt)['accepted']
        distance, beta = 1.4+.5*math.sin(index*.021), .3*math.sin(index*.017)
        x, y = distance*math.cos(beta), distance*math.sin(beta)
        packet = TargetObservationPacket(PROTOCOL_VERSION, sequence, capture, x, y, True, 1.)
        observation = pipeline.accept_observation(packet, source)
        estimates[sequence] = pipeline.estimate_for(observation)
        expected = governor(observation)
        result = controller.accept_measurement('camera', sequence, capture, source, receipt, x, y, True, '')
        assert result.command_accepted
        assert result.radial.estimate.distance_hat_m == pytest.approx(estimates[sequence].distance_hat_m, abs=2e-12, rel=2e-12)
        assert result.radial.estimate.master_radial_velocity_hat_m_s == pytest.approx(estimates[sequence].master_radial_velocity_hat_m_s, abs=2e-12, rel=2e-12)
        assert result.governor.intent.linear_velocity_target_m_s == pytest.approx(expected.linear_velocity_target_m_s, abs=2e-12, rel=2e-12)
        assert result.yaw_rate_rad_s == pytest.approx(yaw.command(x, y)[1], abs=2e-12, rel=2e-12)
        actual = authority.sample(source, receipt)
        assert actual.linear_velocity_target_m_s == pytest.approx(expected.linear_velocity_target_m_s, abs=2e-12, rel=2e-12)
        assert actual.yaw_rate_target_rad_s == pytest.approx(result.yaw_rate_rad_s)
    assert controller.snapshot()['commands_accepted'] == 500


def test_shared_controller_handles_sensor_and_measurement_threads_without_lock_inversion():
    authority, controller = composition()
    def states():
        for sequence in range(1000):
            controller.accept_state(native.FollowRobotState(sequence, sequence*.002, .1))
    def measurements():
        for sequence in range(500):
            # Invalid depth must stay unavailable, independently of interleaving.
            controller.accept_measurement('camera', sequence, sequence*.002,
                sequence*.002, 10.+sequence*.002, 0., 0., False, 'depth_unavailable')
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = [pool.submit(states), pool.submit(measurements)]
        for future in futures: future.result(timeout=10.)
    assert not authority.snapshot(False)['authorized']
    assert controller.snapshot()['measurement_calls'] == 500
    assert controller.snapshot()['state_history']['accepted_count'] == 1000


def test_old_invalid_results_do_not_cross_watermarks_or_new_task_grant():
    authority, controller = composition(); ready(authority, controller)
    assert controller.accept_measurement('camera', 2, 5.12, 5.18, 10.08, 1.8, 0., True, '').command_accepted
    before = controller.snapshot()
    old_epoch = authority.snapshot(False)['authority_epoch']
    for sequence, capture in ((1, 5.02), (2, 5.12), (999, 5.11)):
        result = controller.accept_measurement('camera', sequence, capture, 5.18, 10.08,
            0., 0., False, 'late_master_lost')
        assert not result.accepted and not result.command_accepted
        assert authority.snapshot(False)['authorized']
        assert controller.snapshot()['governor'] == before['governor']
    assert control(authority, 'STOP', 3, 5.18, 10.08)['accepted']
    authority.take_stop()
    assert control(authority, 'ACQUIRE_FOLLOW', 4, 5.18, 10.08, 'new')['accepted']
    assert authority.snapshot(False)['authority_epoch'] > old_epoch
    result = controller.accept_measurement('camera', 1000, 5.17, 5.23, 10.13,
        0., 0., False, 'old_task_pitch_missing')
    assert result.reason == 'follow_pregrant_observation'
    assert authority.snapshot(False)['authorized']
    assert controller.snapshot()['algorithm_resets'] == before['algorithm_resets']
    # A genuinely new postgrant invalid frame still revokes immediately.
    result = controller.accept_measurement('camera', 1001, 5.22, 5.28, 10.18,
        0., 0., False, 'fresh_master_lost')
    assert not result.command_accepted
    assert not authority.snapshot(False)['authorized']
    assert authority.take_stop()[1] == 'fresh_master_lost'
