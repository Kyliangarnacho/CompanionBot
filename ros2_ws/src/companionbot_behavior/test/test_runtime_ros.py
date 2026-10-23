"""Real executor and DDS regression; no models, devices, GUI or actuator transport."""
import threading
import time

import numpy as np
import pytest
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from companionbot_interfaces.msg import ColorFrame
from companionbot_interfaces.srv import UserInput
from companionbot_contracts.stamps import source_stamp
from companionbot_perception.core.camera import ColorFrame as CoreFrame
from companionbot_behavior.ros.runtime import InteractionROS
from companionbot_behavior.runtime.inbox import SessionInbox


def until(executor, condition, timeout=4):
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        executor.spin_once(timeout_sec=.01)
        if condition():
            return
    raise TimeoutError("runtime contract test deadline")


@pytest.fixture
def composed(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "94")
    inbox = SessionInbox(lambda item: item == "/stop")
    runtime = InteractionROS(inbox.put, with_perception=True,
          ros_args=["test", "--ros-args", "-r", "__ns:=/runtime_contract_test"])
    client = Node("probe", namespace="runtime_contract_test", context=runtime.context)
    executor = SingleThreadedExecutor(context=runtime.context)
    executor.add_node(client)
    try:
        yield runtime, inbox, client, executor
    finally:
        executor.shutdown()
        client.destroy_node()
        runtime.close()
        assert all(not t.is_alive() for t in runtime.threads)


def test_slow_perception_publisher_cannot_block_input_and_slots_remain_bounded(composed, monkeypatch):
    runtime, inbox, client, executor = composed
    runtime.interaction.update_status("RUNNING")
    node = runtime.perception
    entered, release = threading.Event(), threading.Event()
    def slow_publish(frame, **metadata):
        entered.set()
        assert release.wait(timeout=4)
    monkeypatch.setattr(node.outputs, "publish_color", slow_publish)
    client.create_subscription(ColorFrame, "camera/color", lambda _: None, qos_profile_sensor_data)
    service = client.create_client(UserInput, "interaction/input")
    until(executor, lambda: service.service_is_ready() and node.outputs.color.get_subscription_count() > 0)
    pixels = np.zeros((6, 8, 3), np.uint8)
    try:
        node.publish_color(CoreFrame(pixels, 0, "fixture", 8, 6, time.perf_counter()))
        until(executor, entered.is_set)
        for seq in range(1, 501):
            node.publish_color(CoreFrame(pixels, seq, "fixture", 8, 6, time.perf_counter()))
        future = service.call_async(UserInput.Request(text="/status", meta=source_stamp(
            "fixture:text", "epoch", 1, time.perf_counter(), time_semantics="text_submitted")))
        until(executor, future.done, timeout=2)
        assert future.result().queued
        assert inbox.get_nowait().text == "/status"
        assert node.snapshot()["pending"] == 1
        assert node.snapshot()["superseded"]["color"] == 499
    finally:
        release.set()


def test_executor_failure_is_visible_to_application_and_cleanup_stops_threads(composed):
    runtime, _, _, executor = composed
    def broken():
        raise RuntimeError("injected_executor_failure")
    runtime.interaction.create_timer(.01, broken)
    until(executor, lambda: bool(runtime.errors))
    with pytest.raises(RuntimeError, match="ROS executor failed"):
        runtime.check_health()


def test_host_clock_cannot_be_silently_switched_to_sim_time(composed):
    runtime, _, _, _ = composed
    for node in runtime.nodes:
        result, = node.set_parameters([Parameter("use_sim_time", value=True)])
        assert not result.successful and result.reason == "host_clock_required"


def test_unconsumed_rgb_is_not_serialized(composed, monkeypatch):
    runtime, _, _, executor = composed
    node = runtime.perception
    def forbidden(*args, **kwargs):
        raise AssertionError("RGB without a diagnostic consumer must not be serialized")
    monkeypatch.setattr(node.outputs, "publish_color", forbidden)
    node.publish_color(CoreFrame(np.zeros((6, 8, 3), np.uint8), 1, "fixture", 8, 6, time.perf_counter()))
    until(executor, lambda: node.snapshot()["no_subscriber"]["color"] > 0)
    runtime.check_health()


def test_invalid_ros_configuration_rolls_back_before_executor_start(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "94")
    before = {t.ident for t in threading.enumerate()}
    with pytest.raises(ValueError, match="finite and positive"):
        InteractionROS(lambda _: True, ros_args=["test", "--ros-args", "-p", "diagnostics_period_s:=0.0"])
    assert not [t for t in threading.enumerate() if t.ident not in before and t.name.startswith("ros-")]
