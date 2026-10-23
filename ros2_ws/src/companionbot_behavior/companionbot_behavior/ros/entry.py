"""ROS executable delegates to the sole formal interaction entry."""
import sys


def main():
    from companionbot_behavior.app import main as demo
    # The app preserves ROS arguments for every node in the composition.
    demo(["--ros", *sys.argv[1:]])
