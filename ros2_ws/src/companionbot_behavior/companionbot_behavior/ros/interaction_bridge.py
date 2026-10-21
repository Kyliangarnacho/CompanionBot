"""One ROS node around the existing Session/Supervisor/Stage 7 owners."""
import json
import time
from uuid import uuid4

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from companionbot_interfaces.msg import RobotSnapshot, BehaviorFeedback
from companionbot_interfaces.srv import UserInput
from companionbot_contracts.stamps import source_stamp
from companionbot_perception.ros.publishers import PerceptionPublishers
from .input_guard import InputGuard


class InteractionBridge:
    def __init__(self, enqueue):
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("interaction", namespace="companionbot", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.epoch = uuid4().hex
        self.perception = PerceptionPublishers(self.node, self.epoch)
        self.robot = self.node.create_publisher(RobotSnapshot, "robot/state", 10)
        self.feedback = self.node.create_publisher(BehaviorFeedback, "behavior/feedback", 32)
        self.events = self.node.create_publisher(String, "interaction/events", 32)
        self.guard = InputGuard()
        self.enqueue = enqueue
        self.node.create_service(UserInput, "interaction/input", self._input)
        self.node.create_service(Trigger, "interaction/cancel", self._cancel)
        self.node.create_service(Trigger, "interaction/stop", self._stop)

    def _input(self, request, response):
        if not request.text.strip() or len(request.text) > 4096:
            response.reason = "empty_or_oversized_text"
            return response
        reason = self.guard.accept(request.meta)
        response.queued = reason is None
        response.reason = reason or "queued_for_session_gate_not_execution_ack"
        if response.queued:
            self.enqueue(request.text)
        return response

    def _cancel(self, _request, response):
        self.enqueue("/cancel")
        response.success, response.message = True, "queued_cancel_not_stopped_confirmation"
        return response

    def _stop(self, _request, response):
        self.enqueue("/stop")
        response.success, response.message = True, "queued_stop_request_execution_backend_is_fake"
        return response

    def _header(self, header, frame=""):
        header.stamp = self.node.get_clock().now().to_msg()
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

    def spin_once(self):
        self.executor.spin_once(timeout_sec=0)

    def ok(self):
        return self.context.ok()

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        if self.context.ok():
            self.context.shutdown()
