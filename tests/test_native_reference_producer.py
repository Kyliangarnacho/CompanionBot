"""Parity of native complete navigation LIGHTWEIGHT ownership against frozen math.

The Python source is an explicit test oracle, never a runtime fallback. Physics,
limits, DT, candidate policy and float32 wire semantics stay the same.
"""
import math

import numpy as np
import pytest

from companionbot_follow import _native_runtime as native
from companionbot_follow.core.rolling_reference import (
    MotionIntent, RuckigLightweightVelocityPlanner, SparseTwoPathReferenceSource,
    SparseVelocityCommandScheduler, TargetObservation,
)
from companionbot_follow.runtime.reference_producer import PiReferenceBlockProducer


def producers():
    class Reader:
        time = velocity = yaw = 0.
        def read(self):
            return TargetObservation(self.time, self.velocity, self.yaw)
    reader = Reader()
    # LIGHTWEIGHT doesn't read A/B, projection scales, or the FULL planner.
    planner = RuckigLightweightVelocityPlanner(controller_dt_s=.002,
        max_velocity_m_s=.6, max_acceleration_m_s2=.6, max_jerk_m_s3=1.,
        A=np.eye(4), B=np.zeros((4, 1)), state_scales=np.ones(4), input_scale_nm=1.)
    scheduler = SparseVelocityCommandScheduler(accept_delta_v_m_s=.1,
        full_delta_v_m_s=.35, stable_window_s=.15, stable_range_m_s=.02,
        require_stable_candidate=False, minimum_delta_enabled=False)
    source = SparseTwoPathReferenceSource(lightweight_planner=planner,
        full_planner=planner, scheduler=scheduler, block_samples=25,
        feedforward_fade_s=.2, observation_reader=reader.read,
        follower=lambda o: MotionIntent(o.capture_time_s, o.x_forward_m, o.y_left_m),
        quiet_theta_rad=.01, quiet_theta_dot_rad_s=.01)
    return reader, source, PiReferenceBlockProducer(source), native.LightweightProducer(native.ProducerConfig())


@pytest.mark.parametrize("seed", [7, 62, 113])
def test_lightweight_retarget_and_wire_every_sample(seed):
    reader, source, python, cpp = producers()
    rng = np.random.default_rng(seed)
    for block in range(800):
        reader.time = block * .05
        if block % 7 == 0:
            reader.velocity = float(rng.uniform(0, .25))
            reader.yaw = float(rng.uniform(-.5, .5))
        start_tick = block * 25
        expected, _ = python.produce(start_tick, reader.time)
        actual = cpp.produce(start_tick, reader.time, reader.time, reader.velocity, reader.yaw)
        # Ruckig is exactly the same installed library. Float64 transient and
        # repeated-addition hold agree before the existing float32 encoding.
        assert actual.bytes == expected.to_bytes(), (seed, block)
        assert np.allclose(cpp.next_reference[:2], source.lifecycle.reference_state[:2], atol=2e-13, rtol=0)
        assert abs(cpp.next_reference[2] - source.lifecycle.reference_acceleration_m_s2) <= 2e-13
        assert math.isfinite(actual.compute_wall) and actual.compute_wall >= 0


def test_prefill_reuses_original_observation_and_reset_epoch():
    reader, source, python, cpp = producers()
    reader.velocity, reader.yaw = .2, .3
    expected, _ = python.produce(0, 0.)
    assert cpp.produce(0, 0., 0., .2, .3).bytes == expected.to_bytes()
    reader.velocity, reader.yaw = .1, -.3
    expected, _ = python.produce(25, 0., observe_command=False)
    assert cpp.produce(25, 0., 0., .1, -.3, False).bytes == expected.to_bytes()
    cpp.reset(2.5, 0., 4)
    assert cpp.reference_epoch == 4 and cpp.next_sequence == 0
    assert cpp.next_reference == [2.5, 0., 0.]


def test_full_required_and_invalid_input_reject_without_consuming_sequence():
    cpp = native.LightweightProducer(native.ProducerConfig())
    for v in (float("nan"), .7, .4):
        with pytest.raises((ValueError, RuntimeError)):
            cpp.produce(0, 0., 0., v, 0.)
        assert cpp.next_sequence == 0
    with pytest.raises(RuntimeError, match="prefill_without"):
        cpp.produce(0, 0., 0., 0., 0., False)
