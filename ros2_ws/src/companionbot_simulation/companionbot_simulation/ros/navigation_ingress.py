"""Composition-only adapter for the native rclcpp navigation ingress.

The native executor owns all ROS command/sensor/service callbacks. It shares
one C++ MotionAuthority with the reference runtime through an in-process
binding. Source time remains ROS time; lease and receive age are monotonic.
"""
from companionbot_follow import _native_runtime
from companionbot_follow.runtime.navigation_motion import NavigationMotionAdapter


class NavigationIngress:
    def __init__(self, node, *, map_id, map_version, output_dir, controller="/controller_server", enable_follow=False):
        follow_config = None
        if enable_follow:
            from companionbot_follow.runtime.native_follow import authority_configuration
            follow_config = authority_configuration()
        self.adapter = NavigationMotionAdapter(map_id=map_id, map_version=map_version,
            follow_config=follow_config)
        self.adapter.follow_controller = None
        if enable_follow:
            from companionbot_follow.runtime.native_follow import controller as build_controller
            self.adapter.follow_controller = build_controller(self.adapter.native_authority)
        self.runtime = _native_runtime.NavigationIngressRuntime(
            self.adapter.native_authority, str(output_dir), controller,
            bool(node.get_parameter("use_sim_time").value))

    def start(self):
        self.runtime.start()

    def check_health(self):
        self.runtime.check_health()

    def snapshot(self):
        return self.runtime.snapshot()

    def close(self):
        self.runtime.close()
