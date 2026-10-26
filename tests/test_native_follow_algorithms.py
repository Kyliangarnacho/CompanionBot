"""Step parity of native Stage 6 algorithms against the current Python oracle."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from companionbot_contracts.packets import PROTOCOL_VERSION, RobotStatePacket, TargetObservationPacket
from companionbot_follow import _native_runtime as native
from companionbot_follow.core.bearing_yaw import BearingYawServo
from companionbot_follow.core.follow_governor import FollowGovernor, FollowGovernorConfig
from companionbot_follow.core.radial_velocity_kf import (
    PiRadialObservationPipeline, PiRobotStateHistory, RadialEstimate,
    RadialVelocityKFConfig, RadialVelocityKalmanFilter)
from companionbot_follow.core.rolling_reference import TargetObservation

ROOT = Path(__file__).resolve().parents[1]
KF_CONFIG = json.loads((ROOT / "models/minisegway/stage6/config/stage6_5_radial_kf_config.json").read_text())["kf"]
GOV_CONFIG = json.loads((ROOT / "models/minisegway/stage6/config/stage6_4_2d_follow_config.json").read_text())["governor"]


def assert_tree(actual, expected, tolerance=2e-12):
    if isinstance(expected, dict):
        assert actual.keys() == expected.keys()
        for key in expected:
            assert_tree(actual[key], expected[key], tolerance)
    elif isinstance(expected, (float, np.floating)):
        assert actual == pytest.approx(expected, abs=tolerance, rel=tolerance)
    elif isinstance(expected, (tuple, list)):
        assert len(actual) == len(expected)
        for lhs, rhs in zip(actual, expected):
            assert_tree(lhs, rhs, tolerance)
    else:
        assert actual == expected


def nobs(observation):
    return native.FollowObservation(observation.capture_time_s, observation.x_forward_m, observation.y_left_m,
        observation.version, observation.sequence_id, observation.valid, observation.confidence)


def nestimate(value):
    return native.RadialEstimate(value.sequence_id, value.capture_time_s, value.distance_hat_m,
        value.master_radial_velocity_hat_m_s, value.aligned_robot_velocity_m_s, value.beta_rad, value.innovation_m)


def check_governor(native_governor, oracle, observation, estimate=None):
    event_count, row_count = len(oracle.events), len(oracle.observation_history)
    expected = oracle(observation)
    actual = native_governor.update(nobs(observation), None if estimate is None else nestimate(estimate))
    assert actual.intent.source_time_s == expected.source_time_s
    assert actual.intent.linear_velocity_target_m_s == pytest.approx(expected.linear_velocity_target_m_s, abs=2e-12, rel=2e-12)
    assert actual.intent.yaw_rate_target_rad_s == expected.yaw_rate_target_rad_s
    assert_tree(actual.event, oracle.events[-1] if len(oracle.events) > event_count else None)
    assert_tree(actual.row, oracle.observation_history[-1] if len(oracle.observation_history) > row_count else None)
    for key, value in native_governor.snapshot().items():
        assert_tree(value, getattr(oracle, key))
    return actual


def test_kf_joseph_update_variable_capture_dt_five_thousand_steps():
    actual = native.RadialVelocityKalmanFilter(native.RadialKFConfig(**KF_CONFIG))
    oracle = RadialVelocityKalmanFilter(RadialVelocityKFConfig(**KF_CONFIG))
    rng = np.random.default_rng(640524)
    capture = 0.
    for index in range(5000):
        capture += float(rng.uniform(.015, .085))
        distance = float(1.2 + .2 * math.sin(index * .013) + rng.normal(0., .03))
        velocity = float(.2 + .1 * math.cos(index * .017))
        expected = oracle.update(capture_time_s=capture, distance_meas_m=distance, robot_radial_velocity_m_s=velocity)
        result = actual.update(capture, distance, velocity)
        assert_tree((result.distance_hat_m, result.master_radial_velocity_hat_m_s, result.innovation_m), expected)
        np.testing.assert_allclose(actual.x, oracle.x, atol=2e-12, rtol=2e-12)
        np.testing.assert_allclose(np.asarray(actual.P).reshape(2, 2), oracle.P, atol=2e-12, rtol=2e-12)
        assert actual.P[1] == actual.P[2]
        assert actual.last_capture_time_s == oracle.last_capture_time_s


def test_kf_duplicate_time_and_invalid_inputs_do_not_mutate():
    actual = native.RadialVelocityKalmanFilter(native.RadialKFConfig(**KF_CONFIG))
    actual.update(0., 1., .5)
    before = (actual.x, actual.P, actual.last_capture_time_s)
    for values, reason in [((0., 1., .5), "strictly increase"), ((.1, -1., .5), "nonnegative"),
                           ((float("nan"), 1., .5), "finite")]:
        with pytest.raises(ValueError, match=reason):
            actual.update(*values)
        assert (actual.x, actual.P, actual.last_capture_time_s) == before


def test_robot_state_history_retention_interpolation_and_rejection_parity():
    actual, oracle = native.PiRobotStateHistory(), PiRobotStateHistory()
    for sequence in range(2000):
        time_s, velocity = sequence * .05, .3 * math.sin(sequence * .01)
        assert actual.accept(native.FollowRobotState(sequence, time_s, velocity))
        oracle.accept(RobotStatePacket(PROTOCOL_VERSION, sequence, time_s, velocity, 0., 0., "NORMAL"))
        assert actual.snapshot()["retained_samples"] == len(oracle.samples)
        for query in (time_s - 2.5, time_s - 1.999, time_s - .013, time_s, time_s + .001):
            assert_tree(actual.velocity_at(query), oracle.velocity_at(query))
    assert not actual.accept(native.FollowRobotState(1, 0., 0.))
    assert actual.snapshot()["out_of_order_rejections"] == 1


def test_history_capacity_rejects_in_window_overflow_without_eviction():
    actual = native.PiRobotStateHistory()
    for sequence in range(1024):
        assert actual.accept(native.FollowRobotState(sequence, sequence * .0001, .1))
    assert not actual.accept(native.FollowRobotState(1024, .1024, .1))
    assert actual.snapshot()["retained_samples"] == 1024
    assert actual.snapshot()["capacity_rejections"] == 1
    assert actual.velocity_at(0.) is not None
    assert actual.accept(native.FollowRobotState(1025, 3., .1))
    assert actual.snapshot()["retained_samples"] == 2


def test_radial_pipeline_valid_missing_invalid_out_of_order_step_parity():
    actual = native.RadialObservationPipeline(native.RadialKFConfig(**KF_CONFIG))
    oracle = PiRadialObservationPipeline(RadialVelocityKalmanFilter(RadialVelocityKFConfig(**KF_CONFIG)), PiRobotStateHistory())
    for sequence in range(20):
        time_s, velocity = sequence * .05, .1 + sequence * .01
        actual.accept_state(native.FollowRobotState(sequence, time_s, velocity))
        oracle.state_history.accept(RobotStatePacket(PROTOCOL_VERSION, sequence, time_s, velocity, 0., 0., "NORMAL"))
    observations = [(0, .025, True), (1, .125, True), (2, .175, False), (1, .225, True),
                    (2, .325, True), (3, 1.125, True), (4, .425, True)]
    for sequence, capture, valid in observations:
        obs = TargetObservation(capture, 1.1 + .2 * capture, .1, sequence_id=sequence, valid=valid)
        packet = TargetObservationPacket(PROTOCOL_VERSION, sequence, capture, obs.x_forward_m, obs.y_left_m, valid, 1.)
        expected = oracle.accept_observation(packet, capture + .08)
        result = actual.accept_observation(nobs(obs), capture + .08)
        assert result.accepted == (expected is not None)
        assert_tree(result.row, oracle.rows[-1])
        if result.estimate:
            estimate = oracle.estimate_for(expected)
            for field in estimate.__dataclass_fields__:
                assert_tree(getattr(result.estimate, field), getattr(estimate, field))


def test_governor_raw_wait_two_slowdowns_then_stable_stop_matches_python():
    config = GOV_CONFIG | dict(initial_velocity_m_s=.25)
    actual, oracle = native.FollowGovernor(native.FollowGovernorConfig(**config)), FollowGovernor(FollowGovernorConfig(**config))
    distance = 1.1
    for sequence in range(107):
        if 1 <= sequence <= 3:
            distance -= .15 * .05
        elif 4 <= sequence <= 6:
            distance -= .25 * .05
        velocity = .25 if sequence < 7 else 0.
        actual.set_robot_velocity_hat(velocity); oracle.set_robot_velocity_hat(velocity)
        check_governor(actual, oracle, TargetObservation(sequence * .05, distance, 0., sequence_id=sequence))
    assert [value["event"] for value in actual.events] == ["slowdown_wait", "slowdown_wait"]
    assert actual.snapshot()["latched_velocity_m_s"] == 0.


def test_governor_latest_five_twenty_window_false_dip_and_fast_stop():
    velocities = {sequence: (.4 if sequence < 15 else .37) for sequence in range(20)}
    velocities.update({20: .3, 21: .3, 22: .3, 23: 0., 24: 0., 25: 0.})
    estimates = {}
    oracle = FollowGovernor(FollowGovernorConfig(**GOV_CONFIG), radial_estimate_provider=lambda obs: estimates[obs.sequence_id])
    actual = native.FollowGovernor(native.FollowGovernorConfig(**GOV_CONFIG), True)
    for sequence in range(26):
        distance = 1.8 if sequence == 0 else 1.1 if sequence >= 19 else 1.6
        observation = TargetObservation(sequence * .05, 1.8, 0., sequence_id=sequence)
        estimate = estimates[sequence] = RadialEstimate(sequence, observation.capture_time_s, distance, velocities[sequence], .4, 0., 0.)
        result = check_governor(actual, oracle, observation, estimate)
        if sequence == 19:
            assert result.event["cruise_velocity_median_m_s"] == .37
            assert result.event["cruise_velocity_long_median_m_s"] == .4
            assert result.intent.linear_velocity_target_m_s == .4
        elif sequence == 22:
            assert result.row["slowdown_rejection_reason"] == "short_kf_dip_without_long_window_slowdown"
    assert actual.events[-1]["event"] == "slowdown_wait"
    assert actual.snapshot()["latched_velocity_m_s"] == 0.


def test_governor_invalid_duplicate_and_short_median_parity():
    samples = {0: (1.8, .4), 1: (1.6, .42), 3: (1.6, .39), 4: (1.6, .41), 5: (1.1, .1), 6: (1.2, .55)}
    estimates = {}
    oracle = FollowGovernor(FollowGovernorConfig(**GOV_CONFIG), radial_estimate_provider=lambda obs: estimates[obs.sequence_id])
    actual = native.FollowGovernor(native.FollowGovernorConfig(**GOV_CONFIG), True)
    for sequence in (0, 1, 2, 3, 4, 4, 5, 6):
        observation = TargetObservation(sequence * .05, 1.8, 0., sequence_id=sequence, valid=sequence != 2)
        estimate = None
        if observation.valid:
            estimate = estimates[sequence] = RadialEstimate(sequence, observation.capture_time_s, *samples[sequence], 0., 0., 0.)
        result = check_governor(actual, oracle, observation, estimate)
        if sequence == 2:
            # Algorithm parity intentionally retains latch. Runtime safety gates
            # must revoke invalid/LOST motion before consuming this intent.
            assert result.intent.linear_velocity_target_m_s > 0.
    assert actual.events[-1]["cruise_velocity_window_samples"] == 5


@pytest.mark.parametrize("with_kf", [False, True])
def test_governor_random_variable_capture_trace_three_thousand_steps(with_kf):
    estimates = {}
    oracle = FollowGovernor(FollowGovernorConfig(**GOV_CONFIG),
        radial_estimate_provider=(lambda obs: estimates[obs.sequence_id]) if with_kf else None)
    actual = native.FollowGovernor(native.FollowGovernorConfig(**GOV_CONFIG), with_kf)
    rng = np.random.default_rng(6311)
    capture = 0.
    for sequence in range(3000):
        capture += float(rng.uniform(.03, .08))
        distance = float(1.3 + .6 * math.sin(sequence * .013))
        beta = float(.5 * math.sin(sequence * .008))
        observation = TargetObservation(capture, distance * math.cos(beta), distance * math.sin(beta),
                                        sequence_id=sequence, valid=sequence % 97 != 5)
        robot_velocity = float(.2 + .12 * math.sin(sequence * .01))
        estimate = None
        if with_kf and observation.valid:
            estimate = estimates[sequence] = RadialEstimate(sequence, capture, distance + .02,
                float(.18 + .2 * math.sin(sequence * .009)), robot_velocity, beta, 0.)
        actual.set_robot_velocity_hat(robot_velocity); oracle.set_robot_velocity_hat(robot_velocity)
        check_governor(actual, oracle, observation, estimate)
        if sequence % 113 == 1:
            check_governor(actual, oracle, observation, estimate)


def test_matching_radial_estimate_required_without_raw_fallback():
    actual = native.FollowGovernor(native.FollowGovernorConfig(**GOV_CONFIG), True)
    observation = native.FollowObservation(1., 2., 0., sequence_id=8)
    with pytest.raises(ValueError, match="matching finite radial estimate"):
        actual.update(observation)
    with pytest.raises(ValueError, match="matching finite radial estimate"):
        actual.update(observation, native.RadialEstimate(7, 1., 2., .2, .1, 0., 0.))
    assert actual.snapshot()["last_sequence_id"] is None


def test_governor_event_retention_is_bounded_and_observable():
    actual = native.FollowGovernor(native.FollowGovernorConfig(**GOV_CONFIG), True)
    for sequence in range(2200):
        distance = 1.8 if sequence % 2 == 0 else 1.
        observation = native.FollowObservation(sequence * .05, distance, 0., sequence_id=sequence)
        estimate = native.RadialEstimate(sequence, sequence * .05, distance, 0., 0., 0., 0.)
        assert actual.update(observation, estimate).event is not None
    assert len(actual.events) == 2048
    assert actual.events_overwritten == 152


def test_bearing_yaw_full_angle_envelope_matches_python():
    actual, oracle = native.BearingYawServo(1., .5, .01), BearingYawServo(gain_s_inv=1., max_rate_rad_s=.5, deadband_rad=.01)
    for angle in np.linspace(-math.pi, math.pi, 1001):
        x, y = math.cos(angle), math.sin(angle)
        assert_tree(actual.command(x, y), oracle.command(x, y))
    for values in ((float("nan"), 0.), (0., float("inf"))):
        with pytest.raises(ValueError, match="finite"):
            actual.command(*values)
