"""Perception's communication boundary, statically composed by the formal app.

The existing session owns capture/inference. Observer calls only replace bounded
slots; serialization and DDS run on this node's separate executor thread.
"""
import threading
import time
from uuid import uuid4

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rclpy.clock import Clock, ClockType

from companionbot_contracts.ros_diagnostics import positive_period, publish_status, require_host_time
from .publishers import PerceptionPublishers


class PerceptionNode(Node):
    def __init__(self, *, context, namespace="companionbot"):
        super().__init__("perception", namespace=namespace, context=context)
        try:
            require_host_time(self)
            self.period = positive_period(self, "publish_period_s", 0.02)
            self.diagnostics_period = positive_period(self, "diagnostics_period_s", 1.0)
            self.epoch = uuid4().hex
            self.outputs = PerceptionPublishers(self, self.epoch)
            self.diagnostics = self.create_publisher(DiagnosticArray, "diagnostics", 10)
            self._lock = threading.Lock()
            self._pending = {}
            self.state, self.error_type = "STARTING", ""
            self.accepted = dict.fromkeys(("color", "master", "observation"), 0)
            self.published = self.accepted.copy()
            self.superseded = self.accepted.copy()
            self.no_subscriber = self.accepted.copy()
            self.last_received = {}
            self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(self.period, self.flush, clock=self._steady_clock)
            self.create_timer(self.diagnostics_period, self.report, clock=self._steady_clock)
        except BaseException:
            self.destroy_node()
            raise

    def _offer(self, channel, value):
        with self._lock:
            if self.state in ("STOPPING", "CLOSED", "ERROR"):
                return
            self.accepted[channel] += 1
            if channel in self._pending:
                self.superseded[channel] += 1
            received = time.perf_counter()
            self._pending[channel] = (value, received)
            self.last_received[channel] = received

    def publish_color(self, frame):
        self._offer("color", frame)

    def publish_master(self, result):
        self._offer("master", result)

    def publish_observation(self, result):
        self._offer("observation", result)

    def set_state(self, state, error_type=""):
        with self._lock:
            if self.state in ("STOPPING", "CLOSED", "ERROR") and state in ("STARTING", "RUNNING"):
                return
            self.state, self.error_type = state, error_type
            if state in ("STOPPING", "CLOSED", "ERROR"):
                self._pending.clear()

    def flush(self):
        with self._lock:
            if self.state in ("STOPPING", "CLOSED", "ERROR"):
                return
            pending, self._pending = self._pending, {}
        try:
            for channel, (value, received) in pending.items():
                publisher = getattr(self.outputs, {"color": "color", "master": "master",
                                                      "observation": "observation"}[channel])
                # Avoid serializing multi-MB RGB without a diagnostic consumer.
                # These are observational topics; local consumers use the buffers.
                if publisher.get_subscription_count() == 0:
                    with self._lock:
                        self.no_subscriber[channel] += 1
                    continue
                getattr(self.outputs, "publish_" + channel)(value, received_at_s=received)
                with self._lock:
                    self.published[channel] += 1
        except Exception as error:
            self.set_state("ERROR", type(error).__name__)
            self.report()

    def snapshot(self):
        with self._lock:
            return {"epoch": self.epoch, "state": self.state, "error_type": self.error_type,
                    "pending": len(self._pending), "accepted": self.accepted.copy(),
                    "published": self.published.copy(), "superseded": self.superseded.copy(),
                    "no_subscriber": self.no_subscriber.copy(),
                    "last_received": self.last_received.copy(), "publish_period_s": self.period,
                    "diagnostics_period_s": self.diagnostics_period}

    def report(self):
        value = self.snapshot()
        age = time.perf_counter() - value["last_received"].get("color", time.perf_counter())
        level = (DiagnosticStatus.ERROR if value["error_type"] else
                 DiagnosticStatus.STALE if self.state == "RUNNING" and age > 1.0 else
                 DiagnosticStatus.WARN if self.state == "STARTING" else DiagnosticStatus.OK)
        publish_status(self, self.diagnostics, value["state"], value | {"color_age_s": age,
                       "stage6_connected": False}, level=level)
