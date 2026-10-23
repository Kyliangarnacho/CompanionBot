"""Read-only taps of the one existing Stage 7 owner; no controller consumer."""
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from companionbot_interfaces.msg import ColorFrame, MasterStatus, RelativeObservation
from companionbot_contracts.stamps import source_stamp


class PerceptionPublishers:
    def __init__(self, node, epoch):
        self.node, self.epoch = node, epoch
        self.color = node.create_publisher(ColorFrame, "camera/color", qos_profile_sensor_data)
        self.master = node.create_publisher(MasterStatus, "perception/master", qos_profile_sensor_data)
        self.observation = node.create_publisher(RelativeObservation, "perception/relative", qos_profile_sensor_data)

    def _header(self, header, frame=""):
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = frame

    def publish_color(self, frame, *, received_at_s=None):
        message = ColorFrame()
        message.meta = source_stamp(frame.source_id, self.epoch, frame.sequence_id, frame.host_receive_time_s,
                                   received_at_s=received_at_s)
        self._header(message.image.header, "camera_optical_frame")
        message.image.width, message.image.height = frame.width, frame.height
        message.image.encoding, message.image.is_bigendian = "bgr8", 0
        message.image.step = frame.width * 3
        message.image.data = np.ascontiguousarray(frame.bgr).tobytes()
        self.color.publish(message)

    def publish_master(self, result, *, received_at_s=None):
        message = MasterStatus()
        self._header(message.header, "camera_optical_frame")
        message.meta = source_stamp(result.source_id, self.epoch, result.source_sequence_id, result.source_time_s,
                                   received_at_s=received_at_s)
        message.state, message.visible = result.state.value, result.master_track is not None
        message.has_track_id, message.track_id = result.bound_track_id is not None, result.bound_track_id or 0
        message.width, message.height = result.frame_width, result.frame_height
        message.hardware_execution_ready = False
        self.master.publish(message)

    def publish_observation(self, result, *, received_at_s=None):
        message = RelativeObservation()
        self._header(message.header, "robot_leveled_axle")
        message.meta = source_stamp(result.source_id, self.epoch, result.source_sequence_id, result.source_time_s,
                                   time_semantics=result.time_semantics, clock_domain=result.clock_domain,
                                   received_at_s=received_at_s,
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
