"""Interaction I/O only. The composition owner supplies its ROS context."""
import json
import threading
import time
from uuid import uuid4

from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger
from companionbot_interfaces.msg import RobotSnapshot, BehaviorFeedback
from companionbot_interfaces.srv import UserInput
from companionbot_contracts.stamps import source_stamp
from companionbot_contracts.ros_diagnostics import positive_period, publish_status, require_host_time
from .input_guard import InputGuard
from companionbot_behavior.runtime.inbox import TextInput


class InteractionBridge(Node):
    def __init__(self, enqueue, *, context, namespace="companionbot"):
        super().__init__("interaction", namespace=namespace, context=context)
        try:
            require_host_time(self)
            self.diagnostics_period = positive_period(self, "diagnostics_period_s", 1.0)
            self._initialize(enqueue)
        except BaseException:
            self.destroy_node()
            raise

    def _initialize(self, enqueue):
        self.robot = self.create_publisher(RobotSnapshot, "robot/state", 10)
        self.feedback = self.create_publisher(BehaviorFeedback, "behavior/feedback", 32)
        self.events = self.create_publisher(String, "interaction/events", 32)
        self.diagnostics = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self._lock = threading.Lock()
        self.state = "STARTING"
        self._status = {"session_tick_s": time.perf_counter(), "error_type": ""}
        self.guard = InputGuard()
        self.enqueue = enqueue
        self.last_input_reason = "none"
        self.create_service(UserInput, "interaction/input", self._input)
        self.create_service(Trigger, "interaction/cancel", self._cancel)
        self.create_service(Trigger, "interaction/stop", self._stop)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.diagnostics_period, self.report, clock=self._steady_clock)

    def _input(self, request, response):
        if self.state != "RUNNING":
            response.reason = "runtime_not_running"
            return response
        if not request.text.strip() or len(request.text) > 4096:
            response.reason = "empty_or_oversized_text"
            return response
        input_id = uuid4().hex
        item = TextInput(request.text, input_id,
                         {key: getattr(request.meta, key) for key in request.meta.get_fields_and_field_types()} |
                         {"adapter_received_at_s": time.perf_counter()},
                         request.meta.source_time_s + self.guard.max_age_s)
        reason = self.guard.accept(request.meta, enqueue=lambda: self.enqueue(item))
        response.queued = reason is None
        response.reason = reason or "queued_for_session_gate_not_execution_ack"
        response.input_id = input_id if response.queued else ""
        self.last_input_reason = response.reason
        return response

    def _cancel(self, _request, response):
        response.success = self.state in ("STARTING", "RUNNING") and self.enqueue("/cancel") is not False
        response.message = "queued_cancel_not_stopped_confirmation" if response.success else "runtime_not_accepting"
        return response

    def _stop(self, _request, response):
        response.success = self.state in ("STARTING", "RUNNING") and self.enqueue("/stop") is not False
        response.message = "queued_stop_request_not_stopped_confirmation" if response.success else "runtime_not_accepting"
        return response

    def _header(self, header, frame=""):
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame

    def publish_robot(self, state):
        message = RobotSnapshot()
        self._header(message.header, "base_link")
        message.meta = source_stamp(state.source_id, state.source_epoch, state.sequence, state.observed_at_s,
                                   clock_domain=state.clock_domain, time_semantics=state.time_semantics,
                                   received_at_s=state.received_at_s, version=state.contract_version,
                                   produced_at_s=state.produced_at_s,
                                   produced_clock_domain=state.produced_clock_domain or "")
        for field in ("connected", "execution_ready", "safety_ok", "master_locked", "backend"):
            setattr(message, field, getattr(state, field))
        message.hardware_execution_ready = False
        self.robot.publish(message)

    def publish_feedback(self, result):
        message = BehaviorFeedback()
        self._header(message.header)
        message.meta = source_stamp(result.source_id, result.source_epoch, result.sequence, result.observed_at_s,
                                   clock_domain=result.clock_domain, time_semantics=result.time_semantics,
                                   received_at_s=result.received_at_s, version=result.contract_version)
        for field in ("status", "reason", "task_id", "backend", "destination_id", "input_id", "turn_id", "request_id"):
            setattr(message, field, getattr(result, field) or "")
        message.intent = result.intent.value if result.intent else ""
        message.has_progress, message.progress = result.progress is not None, result.progress or 0.0
        message.hardware_execution_ready = False
        self.feedback.publish(message)

    def publish_event(self, kind, data):
        # These are UI/diagnostic events. Typed execution feedback is separate.
        message = String()
        message.data = json.dumps({"kind": kind, "data": data, "received_at_s": time.perf_counter(),
                                   "backend": "fake", "hardware_execution_ready": False}, ensure_ascii=False)
        self.events.publish(message)

    def update_status(self, state=None, **values):
        with self._lock:
            if state is not None:
                self.state = state
            self._status.update(values)

    def snapshot(self):
        with self._lock:
            return self._status.copy() | {"state": self.state, "last_input_reason": self.last_input_reason,
                                          "diagnostics_period_s": self.diagnostics_period}

    def report(self):
        value = self.snapshot()
        age = time.perf_counter() - value["session_tick_s"]
        level = (DiagnosticStatus.ERROR if value["error_type"] else
                 DiagnosticStatus.STALE if self.state == "RUNNING" and age > 1.0 else
                 DiagnosticStatus.WARN if self.state == "STARTING" else DiagnosticStatus.OK)
        publish_status(self, self.diagnostics, value["state"], value | {"session_tick_age_s": age,
                       "execution_backend": "fake"}, level=level)
