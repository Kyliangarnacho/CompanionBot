"""Complete native two-path scheduler/lifecycle vs frozen Python algorithm.

The full mathematical planner is the original Ruckig/SciPy solver. Its accepted
plan crosses the binding once; no oracle scheduler/lifecycle runs in production.
"""
import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from companionbot_contracts.packets import ReferenceBlockPacket
from companionbot_follow import _native_runtime as native
from companionbot_follow.core.rolling_reference import MotionIntent, TargetObservation
from companionbot_simulation.scenarios import run_stage5_v1_5_pi_mcu_stream as stage5


class Reader:
    time = velocity = yaw = 0.

    def read(self):
        return TargetObservation(self.time, self.velocity, self.yaw)


def make_pair(policy, backend="lsqr"):
    config = stage5.load_json(stage5.CONFIG_PATH)
    common, _, _, frozen = stage5.load_common(config)
    assert all(frozen.values())
    reader = Reader()
    def planner():
        return stage5.make_full_planner(config, common,
            planning_dt_s=float(config["full_dynamic"]["planning_dt_s"]), backend=backend)
    source = stage5.make_reference_source(config, common, reader,
        follower=lambda o: MotionIntent(o.capture_time_s, o.x_forward_m, o.y_left_m),
        scheduler_gate_overrides=policy, full_planner_override=planner())
    native_full_solver = planner()
    cfg = native.TwoPathConfig()
    producer = native.ProducerConfig()
    producer.dt = source.dt_s
    producer.block_samples = source.block_samples
    producer.max_velocity = source.lightweight_planner.max_velocity_m_s
    producer.max_acceleration = source.lightweight_planner.max_acceleration_m_s2
    producer.max_jerk = source.lightweight_planner.max_jerk_m_s3
    producer.full_delta_velocity = source.scheduler.full_delta_v_m_s
    cfg.producer = producer
    scheduler = native.SchedulerConfig()
    for native_name, oracle_name in (
        ("accept_delta_velocity", "accept_delta_v_m_s"),
        ("full_delta_velocity", "full_delta_v_m_s"),
        ("stable_window_s", "stable_window_s"),
        ("stable_range_velocity", "stable_range_m_s"),
        ("initial_accepted_velocity", "accepted_velocity_m_s"),
        ("require_stable_candidate", "require_stable_candidate"),
        ("minimum_delta_enabled", "minimum_delta_enabled"),
        ("force_full_dynamic", "force_full_dynamic"),
    ):
        setattr(scheduler, native_name, getattr(source.scheduler, oracle_name))
    cfg.scheduler = scheduler
    cfg.feedforward_fade_s = source.lifecycle.feedforward_fade_s
    cfg.quiet_theta_rad = source.quiet_theta_rad
    cfg.quiet_theta_dot_rad_s = source.quiet_theta_dot_rad_s
    cfg.defer_full_accept_until_observed_exit = source.defer_full_accept_until_observed_exit
    solver_calls = []
    def solver(request):
        solver_calls.append((request.execution_start_time, request.target_velocity))
        rolling = native_full_solver.plan(np.asarray(request.start_reference),
            request.start_acceleration, request.target_velocity)
        return native.FullPlan.from_rolling(rolling)
    return reader, source, native.TwoPathProducer(cfg, solver), solver_calls


def check_block(reader, source, cpp, index, *, observe=True, generated=None):
    generated = reader.time if generated is None else generated
    start_tick = index * 25
    commands = [source.command(start_tick * .002 + j * .002,
        observe_command=observe if j == 0 else True,
        planning_request_time_s=generated if j == 0 else None) for j in range(25)]
    rows = [(c.reference_state[0], c.reference_state[1], c.reference_acceleration_m_s2,
        c.reference_state[2], c.reference_state[3], c.u_ff_after_lifecycle_nm) for c in commands]
    expected = ReferenceBlockPacket.from_rows(reference_epoch=0, sequence_id=index,
        start_control_tick=start_tick, generated_time_s=generated, dt_s=.002,
        yaw_rate_target_rad_s=commands[0].yaw_rate_target_rad_s, rows=rows)
    actual = cpp.produce(start_tick, generated, reader.time, reader.velocity, reader.yaw, observe)
    assert actual.bytes == expected.to_bytes(), (index, source.summary()["replan_events"][-1:] or None)
    np.testing.assert_allclose(actual.samples, rows, atol=2e-12, rtol=0)
    for i, (old, new) in enumerate(zip(commands, actual.details)):
        for key, old_value, new_value in (
            ("reference_state", old.reference_state, new.reference_state),
            ("acceleration", old.reference_acceleration_m_s2, new.acceleration),
            ("raw_feedforward", old.u_ff_raw_nm, new.raw_feedforward),
            ("lifecycle_feedforward", old.u_ff_after_lifecycle_nm, new.lifecycle_feedforward),
            ("fade_alpha", old.fade_alpha, new.fade_alpha),
        ):
            np.testing.assert_allclose(old_value, new_value, atol=2e-12, rtol=0, err_msg=f"block {index} sample {i} {key}")
        assert (new.feedforward_phase, new.velocity_phase, new.scheduler_mode,
                new.replanned, new.accepted_changed) == (
            old.feedforward_phase, old.velocity_phase, old.scheduler_mode,
            old.replanned, old.accepted_velocity_changed), (index, i)
        assert actual.raw_velocity == old.raw_linear_velocity_target_m_s
        assert actual.target_velocity == old.linear_velocity_target_m_s
        assert actual.candidate_delta == old.candidate_delta_v_m_s
        assert actual.candidate_target == old.candidate_target_m_s
        assert actual.candidate_stable == old.candidate_stable
        assert actual.pending_velocity == old.latest_pending_raw_v_cmd_m_s
        assert actual.pending_active == old.pending_command_active
    np.testing.assert_allclose(cpp.next_reference, source.next_reference_state, atol=2e-12, rtol=0)
    assert abs(cpp.next_acceleration - source.next_reference_acceleration_m_s2) < 2e-12
    assert actual.compute_wall >= 0 and actual.compute_cpu >= 0
    return actual


def compare_events(source, cpp):
    old, new = source.summary(), cpp.summary()
    assert old["block_count"] == new["block_count"]
    # Timings are measured separately, not expected to be numerically equal.
    ignored = {"planning_wall_time_s", "planning_complete_time_s", "matrix_build_time_s",
        "sparse_assembly_time_s", "lsqr_time_s", "rhs_build_time_s", "factorization_time_s", "solve_time_s",
        "matrix_and_rhs_build_time_s", "lsqr_solve_time_s", "projection_total_time_s"}
    for name in ("scheduler_events", "replan_events", "full_exit_events", "fade_events", "quiet_events"):
        assert len(old[name]) == len(new[name]), name
        for left, right in zip(old[name], new[name]):
            assert left.keys() == right.keys(), (name, left.keys() - right.keys(), right.keys() - left.keys())
            for key in left:
                if key in ignored or key.endswith("_time_ms"): continue
                a, b = left[key], right[key]
                if isinstance(a, (int, float)) and not isinstance(a, bool):
                    assert abs(a - b) <= 2e-12, (name, key, a, b)
                else:
                    assert a == b, (name, key, a, b)


@pytest.mark.parametrize("policy", [
    {},
    dict(require_stable_candidate=False, minimum_delta_enabled=False),
    dict(require_stable_candidate=False, minimum_delta_enabled=False,
         follow_force_full_dynamic=True, follow_defer_until_full_exit=True),
])
@pytest.mark.parametrize("backend", ["lsqr", "cached_kkt"])
def test_complete_full_lightweight_quiet_fade_pending_parity(policy, backend):
    reader, source, cpp, solver_calls = make_pair(policy, backend)
    # Frozen finite inputs include reversals, small retargets, changed commands
    # while FULL is locked, repeated stop, and ample quiet/fade completion time.
    schedule = [(0., 0.), (.2, .2), (.8, .22), (1., .48), (1.4, -.12),
        (1.8, .35), (2.2, .55), (3., 0.), (4.8, -.25), (6., -.3),
        (8., .45), (10., 0.), (12., .12), (14., 0.)]
    for index in range(380):
        reader.time = index * .05
        reader.velocity = next(v for t, v in reversed(schedule) if reader.time + 1e-12 >= t)
        reader.yaw = .4 * math.sin(reader.time)
        check_block(reader, source, cpp, index)
    assert solver_calls
    assert len(solver_calls) == sum(e["planning_path"] == "FULL_DYNAMIC" for e in source.replan_events)
    assert source.full_exit_events and source.quiet_events and source.fade_events
    compare_events(source, cpp)


def test_prefill_and_projected_full_exit_gating():
    policy = dict(require_stable_candidate=False, minimum_delta_enabled=False,
        follow_force_full_dynamic=True, follow_defer_until_full_exit=True)
    reader, source, cpp, calls = make_pair(policy)
    reader.velocity, reader.yaw = .3, .1
    check_block(reader, source, cpp, 0)
    reader.velocity, reader.yaw = -.4, -.2
    check_block(reader, source, cpp, 1, observe=False, generated=0.)
    # The producer executes 50ms ahead of the source command, as in the Pi link.
    for index in range(2, 180):
        reader.time = (index - 1) * .05
        reader.velocity = .1 if index < 90 else 0.
        check_block(reader, source, cpp, index)
    assert len(calls) >= 2
    compare_events(source, cpp)


def test_full_callback_failure_is_explicit_and_no_block_is_issued():
    cfg = native.TwoPathConfig()
    cfg.scheduler.require_stable_candidate = False
    cfg.scheduler.minimum_delta_enabled = False
    producer = native.TwoPathProducer(cfg)
    with pytest.raises(ValueError, match="solver_required"):
        producer.produce(0, 0., 0., .5, 0.)
    assert producer.next_sequence == 0


def companion_runtime_fixture(solver_wrapper=lambda solver: solver, *, maximum_projection_steps=None):
    """Software contract fixture; declared health/commands, no physical/visual claim."""
    from companionbot_follow.runtime import native_follow
    from companionbot_simulation import _native_simulation as simulation
    stream = stage5.load_json(stage5.CONFIG_PATH)
    common, _, _, frozen = stage5.load_common(stream)
    assert all(frozen.values())
    authority = native.MotionAuthority('map', 'v1', follow_config=native_follow.authority_configuration())
    controller = native_follow.controller(authority)
    producer = native.ProducerConfig()
    producer.max_velocity = common[3]['reference_limits']['max_velocity_m_s']
    producer.max_acceleration = common[3]['reference_limits']['max_acceleration_m_s2']
    producer.max_jerk = common[2]['max_jerk_m_s3']
    producer.full_delta_velocity = stream['scheduler']['T_full_delta_v_m_s']
    follow, _ = native_follow.configuration()
    safety, yaw = follow['safety'], follow['yaw']
    pitch = common[7]['parameters']['theta_eq_rad']
    mcu = simulation.McuConfig(simulation.BrakeConfig(.002, safety['safe_max_deceleration_m_s2'],
        safety['safe_max_jerk_m_s3'], safety['hidden_reference_handoff_time_s']),
        safety['heartbeat_timeout_s'], safety['reference_starvation_margin_s'], pitch,
        yaw['freshness_timeout_s'], yaw['decay_rate_rad_s2'])
    clearance = native_follow.clearance_configuration(common)
    runtime = simulation.NavigationRuntime(authority,
        native_follow.producer_configuration(common, stream, producer), mcu, 0.,
        solver_wrapper(native_follow.full_solver(common, stream)), controller,
        simulation.FollowClearanceConfig(clearance['radius_m'], clearance['reaction_time_s'],
            clearance['maximum_projection_steps'] if maximum_projection_steps is None else maximum_projection_steps))
    authority.update_health(True, 'declared_software_fixture', 0.)
    authority.update_follow_obstacle(10., 0., 0., 0.)
    runtime.step(0, 0., 0., pitch, 0.)
    return authority, controller, runtime, pitch


def grant_fixture_follow(authority, runtime, pitch):
    assert authority.update_follow_health(True, '', 'camera', 1, .001, .001, .001)
    result = authority.control('task', 'agent', 1, 'ACQUIRE_FOLLOW', 1.2, 'map', 'v1', .001, .001)
    assert result['accepted'], result
    assert authority.update_follow_health(True, '', 'camera', 2, .002, .002, .002)
    assert authority.receive_follow_cmd('task', 'agent', result['authority_epoch'],
        'camera', 2, .002, .002, .002, .4, 0.)
    for tick in range(1, 25):
        runtime.step(tick, tick*.002, 0., pitch, 0.)


def test_native_solver_exception_revokes_c99_before_propagation():
    def broken(_):
        raise RuntimeError('declared_solver_failure')
    authority, controller, runtime, pitch = companion_runtime_fixture(lambda _: broken)
    grant_fixture_follow(authority, runtime, pitch)
    old_epoch = runtime.epoch
    with pytest.raises(RuntimeError, match='declared_solver_failure'):
        runtime.step(25, .05, 0., pitch, 0.)
    report = runtime.report()
    assert runtime.epoch == old_epoch + 1
    assert report['arbiter_state'] == 'BRAKING'
    assert report['arbiter_reason'] == 'REFERENCE_PRODUCTION_FAILED'
    assert not authority.snapshot(False)['authorized']
    assert report['safety_events'][-1]['reason'] == 'REFERENCE_PRODUCTION_FAILED'


def test_brake_projection_keeps_tail_and_fails_closed_at_resource_bound():
    authority, controller, runtime, pitch = companion_runtime_fixture()
    rows = runtime.report()['follow_braking_events']
    assert rows and rows[0]['valid'] and rows[0]['accepted']
    assert rows[0]['reference_projection_steps'] >= 500  # Original one-second handoff.
    # A positive moving sensor estimate requires an additional conservative
    # stop projection; no plant truth or v^2/(2a) shortcut is used.
    for tick in range(1, 26):
        runtime.step(tick, tick*.002, .3, pitch, 0.)
    row = runtime.report()['follow_braking_events'][-1]
    assert row['valid'] and row['measured_stop_distance_m'] > row['reference_stop_distance_m']
    assert row['measured_tail_margin_m'] > 0.
    assert row['required_clearance_m'] > row['reaction_distance_m'] + row['measured_stop_distance_m']
    # An explicitly too-small computation allowance cannot manufacture ready.
    authority, controller, short, pitch = companion_runtime_fixture(maximum_projection_steps=1)
    assert not short.report()['follow_braking_events'][0]['valid']
    assert not authority.control('task', 'agent', 1, 'CHECK_FOLLOW', 1.2,
        'map', 'v1', 0., 0.)['accepted']


def concurrency_fixture():
    import threading
    import time
    entered, release = threading.Event(), threading.Event()
    def wrap(solver):
        def callback(request):
            entered.set()
            if not release.wait(5.): raise RuntimeError('fixture_solver_release_timeout')
            return solver(request)
        return callback
    authority, controller, runtime, pitch = companion_runtime_fixture(wrap)
    grant_fixture_follow(authority, runtime, pitch)
    errors, outputs = [], []
    def step():
        try: outputs.append(runtime.step(25, .05, 0., pitch, 0.))
        except BaseException as error: errors.append(repr(error))
    def report():
        try: outputs.append(runtime.report())
        except BaseException as error: errors.append(repr(error))
    worker = threading.Thread(target=step)
    worker.start()
    assert entered.wait(3.)
    reporter = threading.Thread(target=report)
    reporter.start()
    # The report waits on the runtime mutex without retaining the GIL, allowing
    # the Python low-frequency solver to finish and release that mutex.
    time.sleep(.03)
    release.set()
    worker.join(5.); reporter.join(5.)
    assert not worker.is_alive() and not reporter.is_alive(), 'GIL/runtime-mutex deadlock'
    assert not errors, errors
    assert len(outputs) == 2
    print('native FULL solver + concurrent report completed', flush=True)


def test_full_solver_report_gil_lock_order_in_bounded_subprocess():
    import subprocess
    import sys
    completed = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--gil-fixture'],
        capture_output=True, text=True, timeout=20.)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert 'concurrent report completed' in completed.stdout


if __name__ == '__main__':
    import sys
    if '--gil-fixture' in sys.argv:
        concurrency_fixture()


def brake_oracle_absolute_travel(velocity, acceleration, maximum_steps):
    """Independent Python oracle for the unchanged frozen C99 brake law."""
    from companionbot_control.core.safety_brake import SafetyBrakeConfig, SafetyBrakePrimitive, SafetyReference
    follow = json.loads((Path(__file__).resolve().parents[1] /
        'models/minisegway/stage6/config/stage6_4_2d_follow_config.json').read_text())
    safety = follow['safety']
    cfg = SafetyBrakeConfig(safety['safe_max_deceleration_m_s2'], safety['safe_max_jerk_m_s3'],
        safety['hidden_reference_handoff_time_s'], .002)
    brake = SafetyBrakePrimitive(cfg, SafetyReference(0., velocity, acceleration, 0., 0., 0.))
    absolute_travel, previous_velocity = 0., velocity
    for index in range(maximum_steps):
        sample = brake.step()
        if index:
            absolute_travel += .5*(abs(previous_velocity)+abs(sample.v_m_s))*cfg.dt_s
        previous_velocity = sample.v_m_s
        if (brake.velocity_settled and brake.hidden_handoff_done
                and abs(-2*sample.v_m_s) <= cfg.safe_max_deceleration_m_s2
                and abs(-2*sample.v_m_s-sample.a_m_s2) <= cfg.safe_max_jerk_m_s3*cfg.dt_s):
            return absolute_travel+abs(sample.v_m_s)/2, abs(sample.v_m_s)/2, index+1
    raise AssertionError('Python brake oracle did not converge within explicit resource allowance')


def test_reaction_continued_acceleration_includes_later_braking_and_tail():
    authority, controller, runtime, pitch = companion_runtime_fixture()
    row = runtime.report()['follow_braking_events'][0]
    assert row['reference_velocity_m_s'] == row['local_velocity_m_s'] == 0.
    assert row['reference_stop_distance_m'] == row['measured_stop_distance_m'] == 0.
    assert row['reaction_velocity_bound_m_s'] > 0.
    assert row['reaction_stop_distance_m'] > 0.
    assert row['required_clearance_m'] > row['reaction_distance_m'] + .369
    assert len(row['reaction_projections']) == 4
    for prediction in row['reaction_projections']:
        expected = brake_oracle_absolute_travel(prediction['initial_velocity_m_s'],
            prediction['initial_acceleration_m_s2'], 14000)
        assert prediction['valid']
        assert prediction['absolute_travel_m'] == pytest.approx(expected[0], abs=2e-12)
        assert prediction['tail_margin_m'] == pytest.approx(expected[1], abs=2e-12)
        assert prediction['steps'] == expected[2]
    assert row['reaction_stop_distance_m'] == max(p['absolute_travel_m'] for p in row['reaction_projections'])


def test_dynamic_reference_acceleration_is_logged_in_braking_projection():
    authority, controller, runtime, pitch = companion_runtime_fixture()
    grant_fixture_follow(authority, runtime, pitch)
    sequence = 2
    for tick in range(25, 251):
        now = tick*.002
        if tick % 25 == 0:
            sequence += 1
            authority.update_health(True, 'declared_software_fixture', now)
            assert authority.update_follow_obstacle(10., now, now, now)
            assert authority.update_follow_health(True, '', 'camera', sequence, now, now, now)
            snapshot = authority.snapshot(False)
            assert authority.receive_follow_cmd('task', 'agent', snapshot['authority_epoch'],
                'camera', sequence, now, now, now, .4, 0.)
        runtime.step(tick, now, 0., pitch, 0.)
    rows = runtime.report()['follow_braking_events']
    moving = [row for row in rows if row['reference_velocity_m_s'] > 0. and row['reference_acceleration_m_s2'] > .01]
    assert moving
    assert all(row['valid'] for row in moving)
    assert all(row['reaction_stop_distance_m'] > 0. for row in moving)
    assert all(row['within_profile_envelope'] for row in moving)


def test_observed_overspeed_invalidates_guard_and_revokes_active_follow():
    authority, controller, runtime, pitch = companion_runtime_fixture()
    grant_fixture_follow(authority, runtime, pitch)
    previous_epoch = runtime.epoch
    runtime.step(25, .05, .7, pitch, 0.)
    report = runtime.report()
    row = report['follow_braking_events'][-1]
    assert not row['within_profile_envelope'] and not row['valid']
    assert row['required_clearance_m'] is None
    assert not authority.snapshot(False)['authorized']
    assert runtime.epoch == previous_epoch+1
    assert report['arbiter_reason'] == 'follow_braking_invalid'
