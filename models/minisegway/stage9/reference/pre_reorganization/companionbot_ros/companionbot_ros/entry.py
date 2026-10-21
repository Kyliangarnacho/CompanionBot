"""ROS executable delegates to the sole formal interaction entry."""
import sys
from rclpy.utilities import remove_ros_args


def main():
    from scripts.demo_stage8_interaction import main as demo
    # Rclpy will separately consume remapping/namespace parameters when initialized.
    demo(["--ros", *remove_ros_args(args=sys.argv)[1:]])
