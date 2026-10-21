"""One ROS node around the existing Session/Supervisor/Stage 7 owners."""
import json
import time
from uuid import uuid4

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from companionbot_interfaces.msg import (ColorFrame, MasterStatus, RelativeObservation,
                                        RobotSnapshot, BehaviorFeedback)
from companionbot_interfaces.srv import UserInput
from .contracts import InputGuard, source_stamp


class InteractionBridge:
    def __init__(self, enqueue):
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("interaction", namespace="companionbot", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.epoch = uuid4().hex
        self.color = self.node.create_publisher(ColorFrame, "camera/color", qos_profile_sensor_data)
        self.master = self.node.create_publisher(MasterStatus, "perception/master", qos_profile_sensor_data)
        self.observation = self.node.create_publisher(RelativeObservation, "perception/relative", qos_profile_sensor_data)
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

    def publish_color(self, frame):
        message = ColorFrame()
        message.meta = source_stamp(frame.source_id, self.epoch, frame.sequence_id, frame.host_receive_time_s)
        self._header(message.image.header, "camera_optical_frame")
        message.image.width, message.image.height = frame.width, frame.height
        message.image.encoding, message.image.is_bigendian = "bgr8", 0
        message.image.step = frame.width * 3
        message.image.data = np.ascontiguousarray(frame.bgr).tobytes()
        self.color.publish(message)

    def publish_master(self, result):
        message = MasterStatus()
        self._header(message.header, "camera_optical_frame")
        message.meta = source_stamp(result.source_id, self.epoch, result.source_sequence_id, result.source_time_s)
        message.state, message.visible = result.state.value, result.master_track is not None
        message.has_track_id, message.track_id = result.bound_track_id is not None, result.bound_track_id or 0
        message.width, message.height = result.frame_width, result.frame_height
        message.hardware_execution_ready = False
        self.master.publish(message)

    def publish_observation(self, result):
        message = RelativeObservation()
        self._header(message.header, "robot_leveled_axle")
        message.meta = source_stamp(result.source_id, self.epoch, result.source_sequence_id, result.source_time_s,
                                   time_semantics=result.time_semantics, clock_domain=result.clock_domain,
                                   produced_at_s=result.result_ready_time_s,
                                   produced_clock_domain="host_perf_counter")
        message.master_track_id = result.master_track_id
        message.width, message.height = result.width, result.height
        message.point.x, message.point.y, message.point.z = result.x_forward_m, result.y_left_m, result.z_up_m
        for field in ("pitch_rad", "pitch_time_s", "pitch_alignment_basis", "pitch_provider",
                      "extrinsic_provisional", "pitch_provisional", "depth_convention",
                      "depth_convention_provisional"):
            setattr(message, field, getattr(result, field))
        message.stage6_connected = False
        self.observation.publish(message)

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
