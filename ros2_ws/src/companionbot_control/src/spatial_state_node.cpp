// The unique production BalanceState -> odom / joints / physical-body TF owner.
// ROS does not schedule the MCU loop. There is no motion command or GT input.
#include "companionbot_control/spatial_state.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <companionbot_interfaces/msg/balance_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace companionbot::spatial {
namespace {
using Steady = std::chrono::steady_clock;
using Message = companionbot_interfaces::msg::BalanceState;
double stamp_seconds(const builtin_interfaces::msg::Time & stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {throw std::invalid_argument("invalid_ros_stamp");}
  return stamp.sec + stamp.nanosec * 1e-9;
}
geometry_msgs::msg::Quaternion quaternion(double roll, double pitch, double yaw) {
  const double sr = std::sin(roll/2), cr = std::cos(roll/2);
  const double sp = std::sin(pitch/2), cp = std::cos(pitch/2);
  const double sy = std::sin(yaw/2), cy = std::cos(yaw/2);
  geometry_msgs::msg::Quaternion result;
  result.x = sr*cp*cy - cr*sp*sy;
  result.y = cr*sp*cy + sr*cp*sy;
  result.z = cr*cp*sy - sr*sp*cy;
  result.w = cr*cp*cy + sr*sp*sy;
  return result;
}
template<typename T> std::string text(const T & value) {
  std::ostringstream stream;
  stream.precision(17);
  stream << value;
  return stream.str();
}
}  // namespace

class SpatialStateNode final : public rclcpp::Node {
 public:
  SpatialStateNode() : Node("spatial_state", "/companionbot") {
    auto geometry_path = parameter<std::string>("geometry_config", "");
    if (geometry_path.empty()) {
      geometry_path = ament_index_cpp::get_package_share_directory("companionbot_description") + "/config/minisegway.yaml";
    }
    geometry_ = load_geometry(geometry_path);
    source_config_ = SourceConfig{geometry_.geometry_id,
      parameter<std::string>("expected_source_id", "mujoco:spatial"),
      parameter<std::string>("expected_clock_domain", "mujoco_sim"),
      parameter<std::string>("expected_device_mapping_id", "")};
    gate_ = std::make_unique<SourceGate>(source_config_);
    if (get_parameter("use_sim_time").as_bool() != gate_->simulated()) {
      throw std::invalid_argument("spatial ROS time must match the configured source clock");
    }
    parameter_callback_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & values) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & value : values) {
        if (value.get_name() == "use_sim_time" && value.as_bool() != gate_->simulated()) {
          result.successful = false;
          result.reason = "source_clock_requires_restart";
        }
      }
      return result;
    });
    odom_ = std::make_unique<PlanarOdometry>(OdometryConfig{
      parameter("velocity_std", .03), parameter("yaw_rate_std", .02),
      parameter("position_rw", .005), parameter("yaw_rw", .002), parameter("max_gap_s", .12)});
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<Message>("state/balance", rclcpp::SensorDataQoS(),
      [this](const Message::ConstSharedPtr message) {receive(*message);});
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {watchdog();});
    started_ = Steady::now();
    diagnostics();
  }

  bool failed() const noexcept {return !error_.empty();}
  const std::string & error() const noexcept {return error_;}
  void close() {
    state_ = failed() ? "ERROR" : "CLOSED";
    if (rclcpp::ok(get_node_base_interface()->get_context())) {diagnostics();}
  }

 private:
  template<typename T> T parameter(const std::string & name, const T & value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  void receive(const Message & message) {
    if (failed()) {return;}
    try {
      const auto & meta = message.meta;
      SpatialSample sample{meta.source_epoch, meta.sequence, stamp_seconds(message.header.stamp),
        message.traveled_distance_m, message.forward_velocity_m_s, message.roll_rad,
        message.pitch_rad, message.yaw_rad, message.roll_rate_rad_s, message.pitch_rate_rad_s,
        message.yaw_rate_rad_s, message.wheel_relative_angle_rad, message.wheel_relative_velocity_rad_s,
        message.imu_sample_time_s, message.imu_available_time_s, message.imu_measurement_age_s,
        message.imu_valid, message.imu_stale, message.encoder_valid};
      SourceMetadata source{meta.contract_version, message.spatial_contract_version,
        message.header.frame_id, message.geometry_id, meta.source_id, meta.clock_domain, meta.clock_id,
        meta.time_semantics, message.ros_time_mapping_id, meta.has_sequence, meta.has_source_time,
        meta.source_time_s, message.ros_time_uncertainty_s};
      gate_->validate(source, sample, now().seconds());
      const auto estimate = odom_->update(sample);
      gate_->commit(source, sample);
      last_source_time_ = sample.time_s;
      publish(message, sample, estimate);
      ++count_;
      state_ = "RUNNING";
    } catch (const std::exception & exception) {
      ++rejected_;
      fail(exception.what());
    }
  }

  void publish(const Message & message, const SpatialSample & sample, const PlanarEstimate & estimate) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = message.header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = estimate.x_m;
    odom.pose.pose.position.y = estimate.y_m;
    odom.pose.pose.orientation = quaternion(0., 0., estimate.yaw_rad);
    for (const std::size_t dimension : {2, 3, 4}) {odom.pose.covariance[6*dimension + dimension] = 1e6;}
    const std::array<std::size_t, 3> dimensions{0, 1, 5};
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        odom.pose.covariance[6*dimensions[i] + dimensions[j]] = estimate.covariance[3*i + j];
      }
    }
    odom.twist.twist.linear.x = sample.velocity_m_s;
    odom.twist.twist.angular.z = sample.yaw_rate_rad_s;
    odom.twist.covariance[0] = std::pow(odom_->config().velocity_std, 2);
    odom.twist.covariance[35] = std::pow(odom_->config().yaw_rate_std, 2);
    for (const std::size_t dimension : {1, 2, 3, 4}) {odom.twist.covariance[6*dimension + dimension] = 1e6;}
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped planar, body;
    planar.header = odom.header;
    planar.child_frame_id = "base_footprint";
    planar.transform.translation.x = estimate.x_m;
    planar.transform.translation.y = estimate.y_m;
    planar.transform.rotation = odom.pose.pose.orientation;
    body.header.stamp = message.header.stamp;
    body.header.frame_id = "base_footprint";
    body.child_frame_id = "base_link";
    body.transform.translation.z = geometry_.wheel_radius_m;
    body.transform.rotation = quaternion(sample.roll_rad, sample.pitch_rad, 0.);
    tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{planar, body});

    sensor_msgs::msg::JointState joints;
    joints.header = message.header;
    joints.name = {"left_wheel_joint", "right_wheel_joint"};
    joints.position = {sample.wheel_angles_rad[geometry_.ros_left_encoder_index],
      sample.wheel_angles_rad[geometry_.ros_right_encoder_index]};
    joints.velocity = {sample.wheel_velocities_rad_s[geometry_.ros_left_encoder_index],
      sample.wheel_velocities_rad_s[geometry_.ros_right_encoder_index]};
    // No actual effort is provided; leave effort unavailable.
    joint_pub_->publish(joints);
  }

  void fail(const std::string & reason) {
    if (failed()) {return;}
    error_ = reason;
    state_ = "ERROR";
    RCLCPP_ERROR(get_logger(), "spatial output inhibited: %s", reason.c_str());
    diagnostics();
  }
  void watchdog() {
    if (!failed()) {
      if (get_publishers_info_by_topic(subscription_->get_topic_name()).size() > 1) {
        fail("multiple_balance_state_publishers");
      }
      if (!last_source_time_) {
        if (std::chrono::duration<double>(Steady::now() - started_).count() > 10.) {fail("spatial_source_startup_timeout");}
      } else if (now().seconds() - *last_source_time_ > odom_->config().max_gap_s) {
        fail("spatial_source_timeout_while_clock_advances");
      }
    }
    diagnostics();
  }
  void diagnostics() {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = failed() ? diagnostic_msgs::msg::DiagnosticStatus::ERROR : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = get_fully_qualified_name();
    status.hardware_id = "host-process:" + std::to_string(getpid());
    status.message = state_;
    const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    };
    add("runtime_state", state_);
    add("hardware_execution_ready", "False");
    add("samples", text(count_));
    add("rejected", text(rejected_));
    add("fault", error_);
    add("geometry_id", geometry_.geometry_id);
    add("source", source_config_.expected_source_id);
    add("source_clock", source_config_.expected_clock_domain);
    add("last_source_time_s", last_source_time_ ? text(*last_source_time_) : "None");
    add("odom_model", "planar_support_pitch_corrected_wheel_path");
    add("covariance_basis", "declared_uncalibrated_process_assumptions");
    add("map_localization_available", "False");
    add("runtime_language", "C++17/rclcpp");
    diagnostic_msgs::msg::DiagnosticArray result;
    result.header.stamp = now();
    result.status.push_back(std::move(status));
    diagnostic_pub_->publish(result);
  }

  Geometry geometry_;
  SourceConfig source_config_;
  std::unique_ptr<SourceGate> gate_;
  std::unique_ptr<PlanarOdometry> odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Subscription<Message>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  Steady::time_point started_;
  std::optional<double> last_source_time_;
  std::uint64_t count_{}, rejected_{};
  std::string state_{"STARTING"}, error_;
};
}  // namespace companionbot::spatial

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  std::shared_ptr<companionbot::spatial::SpatialStateNode> node;
  try {
    node = std::make_shared<companionbot::spatial::SpatialStateNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !node->failed()) {executor.spin_once(std::chrono::milliseconds(100));}
    result = node->failed() ? 1 : 0;
    node->close();
    executor.remove_node(node);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("spatial_state"), "%s", exception.what());
    result = 1;
  }
  node.reset();
  rclcpp::shutdown();
  return result;
}
