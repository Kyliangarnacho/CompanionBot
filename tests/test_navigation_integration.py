import hashlib
import json

import pytest

from companionbot_follow.runtime.navigation_motion import NavigationMotionAdapter
from companionbot_behavior.runtime.nav2_backend import load_bindings, Nav2NavigationBackend
from companionbot_simulation.runtime.spatial_session import SpatialSession


class Clock:
    now = 10.
    def __call__(self):
        return self.now


def ready():
    clock = Clock()
    adapter = NavigationMotionAdapter(map_id="bookstore", map_version="v1", clock=clock)
    adapter.update_health(True)
    adapter.update_mcu_status("NORMAL", "")
    return adapter, clock


def control(adapter, command="ACQUIRE", **overrides):
    values = dict(task_id="task", source_epoch="agent", sequence=1, command=command,
                  lease_duration_s=1., map_id="bookstore", map_version="v1", now_s=5.)
    return adapter.control(**(values | overrides))


def test_navigation_requires_map_health_mcu_and_single_owner():
    adapter, _ = ready()
    assert not control(adapter, map_version="wrong")["accepted"]
    assert control(adapter, sequence=2)["accepted"]
    assert not control(adapter, task_id="other")["accepted"]
    assert not control(adapter, sequence=2)["accepted"]
    assert adapter.snapshot()["task_id"] == "task"


def test_navigation_cancel_clears_queued_intent_and_requires_new_grant():
    adapter, _ = ready()
    assert control(adapter)["accepted"]
    assert adapter.receive_cmd(.2, .3, source_time_s=5.01, now_s=5.02)
    assert adapter.sample(5.02).linear_velocity_target_m_s == .2
    assert adapter.sample(5.02).source_time_s == 5.01
    assert control(adapter, "STOP", sequence=2)["accepted"]
    assert adapter.sample(5.02).linear_velocity_target_m_s == 0
    assert adapter.take_stop()[1] == "navigation_stop"
    assert not adapter.receive_cmd(.2, .3, source_time_s=5.02, now_s=5.03)
    adapter.update_mcu_status("SAFE_HOLD", "navigation_stop")
    assert control(adapter, task_id="new", now_s=6.)["accepted"]
    assert not adapter.receive_cmd(.2, 0., source_time_s=5.9, now_s=6.01)


def test_navigation_command_expiry_sensor_loss_and_mcu_fault_fail_closed():
    adapter, clock = ready()
    assert control(adapter)["accepted"]
    assert adapter.receive_cmd(.2, 0., source_time_s=5.01, now_s=5.02)
    clock.now += .31
    adapter.update_health(True)
    assert adapter.sample(5.34).linear_velocity_target_m_s == 0
    assert adapter.snapshot()["reason"] == "navigation_command_expired"
    adapter.update_mcu_status("SAFE_HOLD", "heartbeat_timeout")
    assert not control(adapter, task_id="retry", now_s=6.)["accepted"]
    adapter.update_mcu_status("NORMAL", "")
    assert control(adapter, task_id="retry", sequence=2, now_s=6.)["accepted"]
    adapter.update_health(False, "scan_lost")
    assert not adapter.snapshot()["authorized"]
    assert adapter.take_stop()[1] == "scan_lost"


def test_explicit_stop_without_active_navigation_still_requests_mcu_hold():
    adapter, _ = ready()
    assert control(adapter, "STOP")["accepted"]
    assert adapter.take_stop()[1] == "navigation_stop"
    assert not control(adapter, "CHECK_STOPPED", sequence=2)["accepted"]
    adapter.update_mcu_status("SAFE_HOLD", "navigation_stop")
    assert control(adapter, "CHECK_STOPPED", sequence=3)["accepted"]


def test_map_binding_detects_modified_image_and_unverified_destination(tmp_path):
    image, map_yaml = tmp_path/"map.pgm", tmp_path/"map.yaml"
    image.write_bytes(b"P5\n1 1\n255\n\xff")
    map_yaml.write_text("image: map.pgm\nresolution: 0.05\n")
    binding = dict(schema_version=1, map_id="bookstore", map_version="v1", map_yaml="map.yaml",
        map_sha256=hashlib.sha256(map_yaml.read_bytes()).hexdigest(),
        image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
        destinations={"service_desk": dict(x_m=0., y_m=2.8, yaw_rad=1.57, reachability_verified=True)})
    path = tmp_path/"binding.json"; path.write_text(json.dumps(binding))
    assert load_bindings(path)["map_id"] == "bookstore"
    image.write_bytes(b"changed")
    with pytest.raises(ValueError, match="hash_or_version"):
        load_bindings(path)


def test_navigation_session_cannot_silently_run_default_reference(tmp_path):
    with pytest.raises(ValueError, match="explicit_motion_adapter"):
        SpatialSession(tmp_path, scenario="navigation")
    with pytest.raises(ValueError, match="route_file_requires_mapping"):
        SpatialSession(tmp_path, scenario="balance", route_file="unconsumed.json")


def test_initial_zero_nav2_feedback_is_unknown_until_a_real_path_distance():
    backend = Nav2NavigationBackend.__new__(Nav2NavigationBackend)
    backend.initial_distance = {}
    assert backend.progress_from_distance("task", 0.) is None
    assert backend.progress_from_distance("task", 5.) == 0.
    assert backend.progress_from_distance("task", 2.5) == .5
    assert backend.progress_from_distance("task", 0.) == .99
    assert backend.progress_from_distance("task", float("nan")) is None


def test_action_discovery_does_not_imply_lifecycle_execution_readiness():
    import asyncio
    from concurrent.futures import Future
    from types import SimpleNamespace
    from lifecycle_msgs.msg import State

    class LifecycleClient:
        state = State.PRIMARY_STATE_INACTIVE
        def service_is_ready(self):
            return True
        def call_async(self, request):
            future = Future()
            future.set_result(SimpleNamespace(current_state=SimpleNamespace(id=self.state)))
            return future

    backend = Nav2NavigationBackend.__new__(Nav2NavigationBackend)
    backend.error = None
    backend.action = SimpleNamespace(server_is_ready=lambda: True)
    backend.lifecycle_clients = [LifecycleClient() for _ in range(5)]
    assert not asyncio.run(backend.navigation_active())
    for client in backend.lifecycle_clients:
        client.state = State.PRIMARY_STATE_ACTIVE
    assert asyncio.run(backend.navigation_active())
    backend.lifecycle_clients[-1].state = State.PRIMARY_STATE_INACTIVE
    assert not asyncio.run(backend.navigation_active())
