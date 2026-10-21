"""Opt-in provisional camera mount only; no odom/map or simulated localization."""
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from companionbot_perception.core.robot_geometry import CameraExtrinsic


def main():
    rclpy.init()
    node = Node("provisional_camera_tf", namespace="companionbot")
    from companionbot_contracts.paths import repository_root
    node.declare_parameter("config", str(repository_root() / "models/minisegway/stage7/config/final_demo.json"))
    mount = json.loads(Path(node.get_parameter("config").value).read_text())["camera_extrinsic"]
    extrinsic = CameraExtrinsic(mount["rotation_body_camera"], mount["translation_body_camera_m"], mount["provisional"])
    if extrinsic.provisional:
        node.get_logger().warning("Publishing UNMEASURED camera mount by explicit opt-in; no localization TF")
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id, transform.child_frame_id = "base_link", "camera_optical_frame"
    transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = extrinsic.translation_body_camera_m.tolist()
    q = Rotation.from_matrix(extrinsic.rotation_body_camera.copy()).as_quat()
    transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = q.tolist()
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(transform)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
