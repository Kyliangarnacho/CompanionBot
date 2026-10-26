"""Full Pi scheduling/C99 authority parity on identical sensor/intent traces.

The Python packet link is an explicit test oracle. There is no Python runtime
fallback and these fixtures certify neither a real robot nor plant dynamics.
"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from companionbot_follow import _native_runtime as follow_native
from companionbot_follow.runtime.navigation_motion import NavigationMotionAdapter
from companionbot_simulation import _native_simulation as native


def oracle_module():
    path = Path(__file__).parent / "oracles/navigation_packet_link_python.py"
    spec = importlib.util.spec_from_file_location("navigation_packet_link_test_oracle", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def configuration():
    from companionbot_simulation.scenarios import run_stage5_v1_5_pi_mcu_stream as stage5
    config = stage5.load_json(stage5.CONFIG_PATH)
    common, _, _, frozen = stage5.load_common(config)
    assert all(frozen.values())
    return stage5, config, common


def runtime_pair(configuration, latency):
    stage5, config, common = configuration
    clock = SimpleNamespace(now=0.)
    adapters = [NavigationMotionAdapter(map_id="parity_map", map_version="fixture_v1",
                                       clock=lambda: clock.now) for _ in range(2)]
    arbiter, link, _ = oracle_module().build_navigation_reference(common, stage5, config, adapters[0])
    link.transport.latency_s = latency  # Explicit original 4ms transport ablation.
    producer = follow_native.ProducerConfig()
    limits = common[3]["reference_limits"]
    producer.dt = common[3]["controller_dt_s"]
    producer.block_samples = config["reference_block"]["sample_count"]
    producer.max_velocity = limits["max_velocity_m_s"]
    producer.max_acceleration = limits["max_acceleration_m_s2"]
    producer.max_jerk = common[2]["max_jerk_m_s3"]
    producer.full_delta_velocity = config["scheduler"]["T_full_delta_v_m_s"]
    brake = arbiter.brake_config
    mcu = native.McuConfig(native.BrakeConfig(brake.dt_s, brake.safe_max_deceleration_m_s2,
        brake.safe_max_jerk_m_s3, brake.hidden_reference_handoff_time_s),
        arbiter.heartbeat_timeout_s, arbiter.starvation_margin_ticks * brake.dt_s,
        arbiter.nominal_pitch_rad, arbiter.yaw_source.timeout_ticks * brake.dt_s,
        arbiter.yaw_source.decay_step / brake.dt_s)
    runtime = native.NavigationRuntime(adapters[1].native_authority, producer, mcu, latency)
    estimate = SimpleNamespace(velocity_hat_m_s=0., theta_hat_rad=arbiter.nominal_pitch_rad,
                              theta_dot_hat_rad_s=0.)
    return clock, adapters, arbiter, link, runtime, estimate


@pytest.mark.parametrize("latency", [0., .004])
@pytest.mark.parametrize("mode", ["normal", "stop_resume", "cancel_pending", "lease_expiry",
                                  "command_expiry", "health_loss"])
def test_complete_native_scheduler_every_tick(configuration, latency, mode):
    clock, adapters, arbiter, link, runtime, estimate = runtime_pair(configuration, latency)
    task, sequence, maximum_delta = "first", 0, 0.
    resumed = False
    previous_epoch = 0
    states = set()
    for tick in range(7500):
        now = tick * .002
        clock.now = now
        for adapter in adapters:
            healthy = not (mode == "health_loss" and 2000 <= tick < 4500)
            adapter.update_health(healthy, "healthy" if healthy else "fixture_health_loss")

        def control(command):
            nonlocal sequence
            sequence += 1
            answers = [adapter.control(task_id=task, source_epoch="parity_fixture",
                sequence=sequence, command=command, lease_duration_s=.8,
                map_id=adapter.map_id, map_version=adapter.map_version, now_s=now)
                for adapter in adapters]
            assert answers[0] == answers[1], (tick, answers)
            return answers[0]["accepted"]

        if tick == 500:
            assert control("ACQUIRE")
        if mode in ("stop_resume", "cancel_pending") and tick == 3000:
            assert control("STOP")
        if mode != "normal" and tick == 5000:
            task, sequence = "explicit_second", 0
            assert control("ACQUIRE")
        if mode == "cancel_pending" and tick == 5010:
            previous_epoch = runtime.epoch
            assert control("STOP")
        active_window = tick >= 500 and (mode == "normal" or tick < 3000 or tick >= 5000)
        if active_window and tick % 100 == 0 and not (mode == "lease_expiry" and 2000 <= tick < 5000):
            control("RENEW")
        if active_window and tick % 50 == 0 and not (mode == "command_expiry" and 2000 <= tick < 5000):
            target = (.08, .21, .12, .25)[(tick // 700) % 4]
            answers = [adapter.receive_cmd(target, .2 if tick < 2500 else -.15,
                source_time_s=now, now_s=now, source_id="fixed_intent_trace") for adapter in adapters]
            assert answers[0] == answers[1]
        link.mcu_local_state(now, estimate, SimpleNamespace(psi_control_rad=0., r_hat_rad_s=0.))
        link.before_control_tick(now)
        expected = arbiter.command(now)
        actual = runtime.step(tick, now, 0., estimate.theta_hat_rad, 0.)
        e = np.array([expected.reference_state[0], expected.reference_state[1],
            expected.reference_acceleration_m_s2, expected.reference_state[2],
            expected.reference_state[3], expected.u_ff_after_lifecycle_nm,
            expected.yaw_rate_target_rad_s])
        reference = actual.reference
        a = np.array([reference.p, reference.v, reference.a, reference.theta,
                      reference.theta_dot, reference.u_ff, actual.yaw_rate])
        maximum_delta = max(maximum_delta, float(np.max(np.abs(a - e))))
        np.testing.assert_allclose(a, e, atol=2e-12, rtol=0., err_msg=f"tick={tick}, mode={mode}")
        assert runtime.state_name(actual.safety_state) == arbiter.state, (tick, mode)
        assert runtime.reason_name(actual.reason) == arbiter.reason, (tick, mode)
        assert actual.epoch == arbiter.receiver.reference_epoch, (tick, mode)
        assert actual.block_sequence == expected.block_id
        if actual.safety_state == 0:
            assert actual.sample_index == expected.block_sample_index
        expected_next = arbiter.next_reference_state
        next_ref = actual.next_reference
        np.testing.assert_allclose([next_ref.p, next_ref.v, next_ref.theta, next_ref.theta_dot],
                                   expected_next, atol=2e-12, rtol=0.)
        states.add(arbiter.state)
        if mode != "normal" and tick > 5050 and arbiter.state == "NORMAL":
            resumed = True
        if mode == "cancel_pending" and tick > 5012:
            assert actual.safety_state == 3 and actual.epoch == previous_epoch + 1
    report = runtime.report()
    metrics = report["metrics"]
    assert metrics["step_count"] == 7500 and metrics["step_samples_overwritten"] == 0
    assert metrics["queue_capacity_rejections"] == 0 and metrics["reference_underruns"] == 0
    assert metrics["logs"]["producer"]["overwritten"] == 0
    assert metrics["logs"]["transport"]["overwritten"] == 0
    assert maximum_delta < 2e-12
    if mode == "normal":
        assert states == {"NORMAL"}
    else:
        assert {"BRAKING", "SAFE_HOLD"} <= states
        assert resumed == (mode != "cancel_pending")


def test_invalid_tick_and_clock_rejected_without_advancing(configuration):
    _, _, _, _, runtime, estimate = runtime_pair(configuration, 0.)
    runtime.step(0, 100., 0., estimate.theta_hat_rad, 0.)
    for tick, receipt in [(0, 100.), (2, 100.004), (1, 99.9), (1, float("nan"))]:
        with pytest.raises((ValueError, RuntimeError)):
            runtime.step(tick, receipt, 0., estimate.theta_hat_rad, 0.)
    runtime.step(1, 100.002, 0., estimate.theta_hat_rad, 0.)
    assert runtime.report()["metrics"]["step_count"] == 2


def test_bounded_logs_count_overwrite_without_hiding_producer_window(configuration):
    _, _, _, _, runtime, estimate = runtime_pair(configuration, 0.)
    for tick in range(55000):
        runtime.step(tick, tick * .002, 0., estimate.theta_hat_rad, 0.)
    report = runtime.report()
    metrics = report["metrics"]
    assert metrics["step_count"] == 55000 and metrics["step_samples_overwritten"] == 0
    assert metrics["logs"]["yaw"] == dict(total=2200, retained=2048, capacity=2048, overwritten=152)
    assert len(report["yaw_events"]) == 2048
    assert metrics["logs"]["producer"]["overwritten"] == 0
    assert metrics["logs"]["producer"]["retained"] == 2201
    assert metrics["references_dropped"] == 0 and metrics["reference_underruns"] == 0


def test_reason_capacity_cannot_discard_accepted_stop_or_rearm(configuration):
    clock, adapters, _, _, runtime, estimate = runtime_pair(configuration, 0.)
    authority = adapters[1].native_authority
    runtime.step(0, 0., 0., estimate.theta_hat_rad, 0.)
    for tick in range(1, 801):
        clock.now = tick * .002
        if tick < 322:
            authority.stop(f"explicit_fixture_stop_{tick}", clock.now, True)
        output = runtime.step(tick, clock.now, 0., estimate.theta_hat_rad, 0.)
        assert output.safety_state != 0
    report = runtime.report()
    assert report["metrics"]["reason_capacity_faulted"] is True
    assert runtime.epoch >= 321
    assert report["arbiter_state"] == "SAFE_HOLD"
    assert any(e["event"] == "stop_reason_capacity_exhausted" for e in report["packet_events"])
    adapters[1].update_health(True)
    result = adapters[1].control(task_id="new_after_capacity", source_epoch="fixture",
        sequence=1, command="ACQUIRE", lease_duration_s=.8, map_id=adapters[1].map_id,
        map_version=adapters[1].map_version, now_s=clock.now)
    assert not result["accepted"]


def test_transport_capacity_revokes_mcu_before_fatal_error(configuration):
    _, _, _, _, runtime, estimate = runtime_pair(configuration, 100.)
    failed = False
    for tick in range(1000):
        try:
            runtime.step(tick, tick * .002, 0., estimate.theta_hat_rad, 0.)
        except RuntimeError as error:
            assert "transport_capacity_exhausted" in str(error)
            failed = True
            break
    assert failed
    report = runtime.report()
    assert report["metrics"]["queue_capacity_rejections"] == 1
    assert runtime.safety_state != 0 and runtime.epoch >= 2
