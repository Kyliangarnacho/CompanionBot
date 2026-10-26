"""C++ owner parity against the frozen test-only Python oracle.

These are software authority tests, not MCU/physical execution certification.
"""
import importlib.util
from pathlib import Path
import random

import pytest

from companionbot_follow.runtime.navigation_motion import NavigationMotionAdapter

_SPEC = importlib.util.spec_from_file_location(
    "navigation_motion_python_oracle", Path(__file__).parent / "oracles/navigation_motion_python.py")
_ORACLE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_ORACLE)


class Clock:
    now = 10.
    def __call__(self):
        return self.now


def pair():
    clock = Clock()
    owners = [implementation(map_id="bookstore", map_version="v1", clock=clock)
              for implementation in (NavigationMotionAdapter, _ORACLE.NavigationMotionAdapter)]
    for owner in owners:
        owner.update_health(True)
        owner.update_mcu_status("NORMAL", "")
    return owners, clock


def control(owner, *, task="task", sequence=1, command="ACQUIRE", now_s=100., **values):
    return owner.control(**(dict(task_id=task, source_epoch="epoch", sequence=sequence,
        command=command, lease_duration_s=1.2, map_id="bookstore", map_version="v1", now_s=now_s) | values))


def snapshots_equal(owners):
    actual, oracle = [owner.snapshot() for owner in owners]
    assert {key: actual[key] for key in oracle} == oracle


def same_call(owners, operation, *args, **kwargs):
    result = [getattr(owner, operation)(*args, **kwargs) for owner in owners]
    assert result[0] == result[1]
    return result[0]


def test_normal_motion_five_thousand_ticks_matches_python_authority():
    owners, clock = pair()
    rng = random.Random(1429)
    for epoch in range(10):
        task = f"task_{epoch}"
        source_start = 100. + epoch * 2.
        assert control(owners[0], task=task, now_s=source_start) == control(
            owners[1], task=task, now_s=source_start)
        sequence = 1
        for tick in range(500):
            clock.now += .002
            source = source_start + tick * .002
            if tick % 10 == 0:
                same_call(owners, "update_health", True)
            if tick % 100 == 0:
                sequence += 1
                assert control(owners[0], task=task, sequence=sequence, command="RENEW", now_s=source) == control(
                    owners[1], task=task, sequence=sequence, command="RENEW", now_s=source)
            if tick % 50 == 0:
                v, yaw = rng.uniform(0., .4), rng.uniform(-.8, .8)
                same_call(owners, "receive_cmd", v, yaw, source_time_s=source, now_s=source)
            same_call(owners, "sample", source)
            if tick % 100 == 0:
                snapshots_equal(owners)
        sequence += 1
        assert control(owners[0], task=task, sequence=sequence, command="STOP", now_s=source) == control(
            owners[1], task=task, sequence=sequence, command="STOP", now_s=source)
        same_call(owners, "take_stop")
        same_call(owners, "update_mcu_status", "SAFE_HOLD", "navigation_stop")
        snapshots_equal(owners)


@pytest.mark.parametrize("fault", ["source_expiry", "receipt_expiry", "source_jump", "health_loss", "mcu_fault"])
def test_source_receipt_and_independent_fault_parity(fault):
    owners, clock = pair()
    for owner in owners:
        assert control(owner)["accepted"]
    same_call(owners, "receive_cmd", .2, .1, source_time_s=100.01, now_s=100.02)
    source = 100.02
    if fault == "source_expiry":
        source += .31
    elif fault == "receipt_expiry":
        clock.now += .31
        same_call(owners, "update_health", True)
    elif fault == "source_jump":
        source -= .1
    elif fault == "health_loss":
        same_call(owners, "update_health", False, "scan_lost")
    else:
        same_call(owners, "update_mcu_status", "SAFE_HOLD", "heartbeat_timeout")
    same_call(owners, "sample", source)
    assert not owners[0].snapshot()["authorized"]
    same_call(owners, "take_stop")
    snapshots_equal(owners)


def test_invalid_and_stale_inputs_preserve_legacy_rejection_order():
    owners, _ = pair()
    operations = [dict(command="BOGUS"), dict(map_version="wrong"), dict(), dict(sequence=2),
                  dict(sequence=3, command="RENEW", lease_duration_s=.01),
                  dict(sequence=4, task="other"), dict(sequence=True), dict(sequence=-1)]
    for values in operations:
        assert control(owners[0], **values) == control(owners[1], **values)
        snapshots_equal(owners)
    for source, now, v in [(99.,100.,.2),(100.01,100.,-.1),(100.02,100.,float("nan"))]:
        same_call(owners, "receive_cmd", v, 0., source_time_s=source, now_s=now)
        snapshots_equal(owners)


def test_expired_renew_requires_fresh_explicit_acquire():
    """Intentional fix: old owner snapshot allowed expired RENEW to re-acquire."""
    owners, clock = pair()
    for owner in owners:
        assert control(owner)["accepted"]
    clock.now += 1.3
    same_call(owners, "update_health", True)
    actual = control(owners[0], sequence=2, command="RENEW", now_s=101.3)
    legacy = control(owners[1], sequence=2, command="RENEW", now_s=101.3)
    assert not actual["accepted"] and actual["reason"] == "navigation_owner_mismatch"
    assert legacy["accepted"]
    assert not owners[0].snapshot()["authorized"]
    assert control(owners[0], task="new", now_s=101.3)["accepted"]


def test_bounded_identity_capacity_does_not_forget_replay_or_block_stop():
    owners, _ = pair()
    owner = owners[0]
    for index in range(256):
        assert control(owner, task=f"check_{index}", command="CHECK")["accepted"]
    result = control(owner, task="new")
    assert not result["accepted"] and result["reason"] == "navigation_identity_capacity_exhausted"
    assert owner.snapshot()["retained_watermarks"] == 256
    assert not control(owner, task="check_0", command="CHECK")["accepted"]
    assert control(owner, task="new", command="STOP")["accepted"]
    assert owner.take_stop()[1] == "navigation_stop"
    assert owner.snapshot()["recovery_locked"]
    assert owner.snapshot()["safety_capacity_bypasses"] == 1


def test_stop_reason_capacity_locks_rearm_but_safety_request_survives():
    owner = pair()[0][0]
    for index in range(257):
        assert control(owner, sequence=index + 1)["accepted"]
        owner.stop(f"test_stop_{index}")
        owner.update_mcu_status("NORMAL", "")
    snapshot = owner.snapshot()
    assert snapshot["recovery_locked"] and snapshot["retained_stop_reasons"] == 256
    assert not control(owner, sequence=258)["accepted"]
    assert control(owner, command="STOP", sequence=259)["accepted"]


def test_events_are_bounded_with_overwrite_counter():
    owner, oracle = pair()[0]
    for value in (owner, oracle):
        assert control(value)["accepted"]
        for index in range(2100):
            stamp = 100. + (index + 1) * .00001
            assert value.receive_cmd(.2, 0., source_time_s=stamp, now_s=stamp)
    snapshots_equal([owner, oracle])
    assert owner.snapshot()["retained_events"] == 2048
    assert owner.snapshot()["events_overwritten"] == 53


def test_binding_shares_a_single_native_owner():
    owner = pair()[0][0]
    assert control(owner)["accepted"]
    assert owner.native_authority.snapshot(False)["authorized"]
    owner.native_authority.stop("native_direct_stop", owner.clock())
    assert not owner.snapshot()["authorized"]
    assert owner.take_stop()[1] == "native_direct_stop"
