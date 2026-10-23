"""Static Python composition for the sole application (not rclcpp_components).

Each communication boundary has its own executor thread. Neither executor runs
Agent/inference/control algorithms. Lifecycle is owned by the application.
"""
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

from companionbot_perception.ros.node import PerceptionNode
from .interaction_bridge import InteractionBridge


class InteractionROS:
    def __init__(self, enqueue, *, with_perception=False, ros_args=None):
        self.context = Context()
        self.nodes, self.executors, self.threads = [], [], []
        self.errors = []
        self._closing = False
        self.perception = None
        try:
            rclpy.init(args=ros_args, context=self.context, signal_handler_options=SignalHandlerOptions.NO)
            self.interaction = InteractionBridge(enqueue, context=self.context)
            self.nodes.append(self.interaction)
            if with_perception:
                self.perception = PerceptionNode(context=self.context)
                self.nodes.append(self.perception)
            names = [node.get_fully_qualified_name() for node in self.nodes]
            if len(set(names)) != len(names):
                raise ValueError("Composition needs node-scoped __node remaps; global remap aliases both nodes")
            for node in self.nodes:
                executor = SingleThreadedExecutor(context=self.context)
                executor.add_node(node)
                self.executors.append(executor)
                thread = threading.Thread(target=self._spin, args=(executor,), name="ros-" + node.get_name())
                self.threads.append(thread)
                thread.start()
        except BaseException:
            self.close()
            raise

    def _spin(self, executor):
        try:
            executor.spin()
        except ExternalShutdownException:
            if not self._closing:
                self.errors.append("ExternalShutdownException")
        except Exception as error:
            self.errors.append(type(error).__name__)

    def check_health(self):
        if self.errors:
            raise RuntimeError("ROS executor failed: " + self.errors[0])
        if self.perception and self.perception.snapshot()["error_type"]:
            raise RuntimeError("Perception publication failed: " + self.perception.snapshot()["error_type"])
        if not self.context.ok() and not self._closing:
            raise RuntimeError("ROS context stopped unexpectedly")

    def begin_stop(self, error_type=""):
        state = "ERROR" if error_type else "STOPPING"
        if hasattr(self, "interaction"):
            self.interaction.update_status(state, error_type=error_type)
        if self.perception:
            self.perception.set_state(state, error_type)

    def snapshot(self):
        return {"nodes": [node.get_fully_qualified_name() for node in self.nodes],
                "executors": [thread.name for thread in self.threads],
                "executor_errors": self.errors.copy(),
                "interaction": self.interaction.snapshot(),
                "perception": self.perception.snapshot() if self.perception else None}

    def close(self):
        if self._closing:
            return
        self._closing = True
        self.begin_stop()
        failures = []
        for executor in reversed(self.executors):
            try:
                if not executor.shutdown(timeout_sec=5):
                    failures.append("executor_shutdown_timeout")
            except Exception as error:
                failures.append("executor_shutdown:" + type(error).__name__)
        for thread in self.threads:
            thread.join(timeout=5)
            if thread.is_alive():
                failures.append(thread.name + "_join_timeout")
        # Never destroy ROS handles under a still-running callback.
        if not any(thread.is_alive() for thread in self.threads):
            for node in reversed(self.nodes):
                try:
                    node.destroy_node()
                except Exception as error:
                    failures.append("node_destroy:" + type(error).__name__)
            if self.context.ok():
                try:
                    self.context.try_shutdown()
                except Exception as error:
                    failures.append("context_shutdown:" + type(error).__name__)
        self.errors.extend(failures)
        if failures:
            raise RuntimeError(",".join(failures))
