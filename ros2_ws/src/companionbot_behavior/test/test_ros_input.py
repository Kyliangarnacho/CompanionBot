"""Existing Stage 9 ROS contract regression, relocated to its owner."""
from types import SimpleNamespace
import pytest
from companionbot_contracts.stamps import source_stamp
from companionbot_behavior.ros.input_guard import InputGuard
from companionbot_behavior.ros.interaction_bridge import InteractionBridge
from companionbot_interfaces.srv import UserInput

def input_stamp(seq=1, t=10.0):
    return source_stamp("test:text", "epoch-a", seq, t, time_semantics="text_submitted")


def test_freshness_order_and_epoch_are_not_ros_time():
    guard = InputGuard(clock=lambda: 10.1)
    stamp = input_stamp()
    assert guard.accept(stamp) is None
    assert guard.accept(stamp) == "duplicate_or_out_of_order"
    stamp = input_stamp(2, 10.05)
    stamp.clock_domain = "ros_time"
    assert guard.accept(stamp) == "unmapped_clock_or_time_semantics"
    stamp = input_stamp(2, 10.05)
    stamp.source_epoch = "restart"
    assert guard.accept(stamp) == "source_restart_requires_rebind"
    assert guard.accept(input_stamp(2, 10.05)) is None


@pytest.mark.parametrize("mutate,reason", [
    (lambda m: setattr(m, "clock_id", "another-machine"), "unmapped_clock_or_time_semantics"),
    (lambda m: setattr(m, "source_time_s", 12.0), "stale_or_future_input"),
    (lambda m: setattr(m, "source_time_s", 8.0), "stale_or_future_input"),
    (lambda m: setattr(m, "source_time_s", float("nan")), "stale_or_future_input"),
    (lambda m: setattr(m, "has_sequence", False), "invalid_source_contract")])
def test_rejected_input_does_not_poison_watermark(mutate, reason):
    guard = InputGuard(clock=lambda: 10.1)
    stamp = input_stamp()
    mutate(stamp)
    assert guard.accept(stamp) == reason
    assert guard.accept(input_stamp()) is None


def test_ros_input_only_queues_and_cannot_bypass_session():
    queued = []
    bridge = SimpleNamespace(guard=InputGuard(clock=lambda: 10.1), enqueue=queued.append)
    request = UserInput.Request(meta=input_stamp(), text="你好小柒")
    response = InteractionBridge._input(bridge, request, UserInput.Response())
    assert response.queued and "not_execution_ack" in response.reason and queued == ["你好小柒"]
    assert not InteractionBridge._input(bridge, request, UserInput.Response()).queued


