#include "companionbot_follow/navigation_ingress.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <companionbot_interfaces/srv/navigation_control.hpp>

namespace companionbot::follow {
namespace {
using Json = nlohmann::json;
using Gid = std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>;
using Twist = geometry_msgs::msg::TwistStamped;
using Control = companionbot_interfaces::srv::NavigationControl;
double receipt_now() {
  timespec timestamp{};
  if (clock_gettime(CLOCK_MONOTONIC, &timestamp) != 0) {
    throw std::runtime_error("navigation_monotonic_clock_unavailable");
  }
  return static_cast<double>(timestamp.tv_sec) + static_cast<double>(timestamp.tv_nsec) * 1e-9;
}
double thread_cpu_now() {
  timespec timestamp{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &timestamp) != 0) {
    throw std::runtime_error("navigation_thread_cpu_clock_unavailable");
  }
  return static_cast<double>(timestamp.tv_sec) + static_cast<double>(timestamp.tv_nsec) * 1e-9;
}
struct TimingSamples {
  static constexpr std::size_t kCapacity = 32768;
  std::array<double, kCapacity> wall{}, cpu{};
  std::uint64_t count{};
  double wall_total{}, cpu_total{}, wall_max{}, cpu_max{};
  void add(double wall_s, double cpu_s) {
    const auto index = count % kCapacity;
    wall[index] = wall_s; cpu[index] = cpu_s; ++count;
    wall_total += wall_s; cpu_total += cpu_s;
    wall_max = std::max(wall_max, wall_s); cpu_max = std::max(cpu_max, cpu_s);
  }
  IngressTimingSnapshot snapshot(const std::string& name) const {
    IngressTimingSnapshot result;
    result.name = name; result.samples = count; result.retained_samples = std::min<std::uint64_t>(count, kCapacity);
    result.wall_total_s = wall_total; result.cpu_total_s = cpu_total;
    result.wall_max_s = wall_max; result.cpu_max_s = cpu_max;
    if (count == 0) return result;
    std::vector<double> wall_sorted(wall.begin(), wall.begin() + result.retained_samples);
    std::vector<double> cpu_sorted(cpu.begin(), cpu.begin() + result.retained_samples);
    std::sort(wall_sorted.begin(), wall_sorted.end()); std::sort(cpu_sorted.begin(), cpu_sorted.end());
    const auto quantile = [](const std::vector<double>& values, double q) {
      const double position = q * static_cast<double>(values.size() - 1);
      const auto lower = static_cast<std::size_t>(std::floor(position));
      const auto upper = static_cast<std::size_t>(std::ceil(position));
      return values[lower] + (values[upper] - values[lower]) * (position - static_cast<double>(lower));
    };
    result.wall_p50_s = quantile(wall_sorted, .5); result.wall_p95_s = quantile(wall_sorted, .95);
    result.wall_p99_s = quantile(wall_sorted, .99); result.cpu_p50_s = quantile(cpu_sorted, .5);
    result.cpu_p95_s = quantile(cpu_sorted, .95); result.cpu_p99_s = quantile(cpu_sorted, .99);
    return result;
  }
};
double seconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}
Json nullable(const std::optional<std::string>& value) {
  return value ? Json(*value) : Json(nullptr);
}
}

class NavigationIngressRuntime::Impl {
 public:
  explicit Impl(std::shared_ptr<MotionAuthority> authority, const std::string& output_dir,
                std::string controller, bool use_sim_time)
      : authority_(std::move(authority)), controller_(std::move(controller)), use_sim_time_(use_sim_time) {
    if (!authority_ || output_dir.empty() || controller_.empty() || controller_.front() != '/') {
      throw std::invalid_argument("invalid_native_navigation_ingress_configuration");
    }
    std::filesystem::create_directories(output_dir);
    commands_log_.open(std::filesystem::path(output_dir) / "navigation_commands.jsonl");
    controls_log_.open(std::filesystem::path(output_dir) / "navigation_controls.jsonl");
    if (!commands_log_ || !controls_log_) throw std::runtime_error("navigation_log_open_failed");
    commands_log_.exceptions(std::ios::badbit | std::ios::failbit);
    controls_log_.exceptions(std::ios::badbit | std::ios::failbit);
  }

  void start() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    if (closed_) throw std::runtime_error("navigation_ingress_already_closed");
    if (started_) return;
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context_).append_parameter_override("use_sim_time", use_sim_time_);
    node_ = std::make_shared<rclcpp::Node>("navigation_ingress", "/companionbot", options);
    clock_callback_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
      rcl_interfaces::msg::SetParametersResult answer;
      answer.successful = std::all_of(parameters.begin(), parameters.end(), [this](const rclcpp::Parameter& value) {
        return value.get_name() != "use_sim_time" ||
            (value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL && value.as_bool() == use_sim_time_);
      });
      answer.reason = "navigation_clock_change_requires_runtime_restart";
      return answer;
    });
    command_sub_ = node_->create_subscription<Twist>(command_topic_, rclcpp::QoS(1),
        [this](Twist::ConstSharedPtr message, const rclcpp::MessageInfo& info) {
          measured(0, [&] { command(*message, info); });
        });
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/companionbot/scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
          measured(1, [&] { scan(*message); });
        });
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/companionbot/odom", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          measured(2, [&] { odom(*message); });
        });
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", rclcpp::QoS(10), [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) {
          measured(3, [&] { pose(*message); });
        });
    tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(100), [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
          measured(4, [&] { transforms(*message); });
        });
    control_service_ = node_->create_service<Control>("/companionbot/navigation/control",
        [this](const Control::Request::SharedPtr request, Control::Response::SharedPtr response) {
          measured(5, [&] { control(*request, *response); });
        });
    health_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
        [this] { measured(6, [&] { health(); }); });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);
    thread_stopped_ = false;
    started_ = true;
    worker_ = std::thread([this] {
      try {
        executor_->spin();
        if (!closed_) throw std::runtime_error("navigation_executor_stopped_unexpectedly");
      } catch (const std::exception& error) {
        authority_->stop("navigation_executor_failed", receipt_now());
        std::lock_guard<std::mutex> lock(error_mutex_);
        error_ = std::current_exception();
        error_text_ = error.what();
      } catch (...) {
        authority_->stop("navigation_executor_failed", receipt_now());
        std::lock_guard<std::mutex> lock(error_mutex_);
        error_ = std::current_exception();
        error_text_ = "navigation_executor_unknown_error";
      }
      thread_stopped_ = true;
    });
  }

  void check_health() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    if (error_) std::rethrow_exception(error_);
  }

  void close() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    if (closed_.exchange(true)) return;
    // Revoke before canceling transport. Native/Microcontroller consumers still
    // own their independently enforced stale/heartbeat and physical gates.
    authority_->stop("navigation_runtime_shutdown", receipt_now());
    if (executor_) executor_->cancel();
    if (worker_.joinable()) worker_.join();
    // A callback already executing when close began may have observed the old
    // lifecycle flag. Revoke again after joining, before releasing any resource.
    authority_->stop("navigation_runtime_shutdown", receipt_now());
    if (executor_ && node_) executor_->remove_node(node_);
    health_timer_.reset(); control_service_.reset(); command_sub_.reset();
    scan_sub_.reset(); odom_sub_.reset(); pose_sub_.reset(); tf_sub_.reset();
    clock_callback_.reset(); node_.reset(); executor_.reset();
    if (context_ && context_->is_valid()) context_->shutdown("navigation_runtime_shutdown");
    context_.reset();
    commands_log_.close(); controls_log_.close();
  }

  NavigationIngressSnapshot snapshot(bool include_timings) const {
    NavigationIngressSnapshot result;
    result.started = started_; result.closed = closed_; result.thread_stopped = thread_stopped_;
    result.node_name = "/companionbot/navigation_ingress";
    {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      result.reason = last_reason_; result.commands_received = commands_received_;
      result.commands_accepted = commands_accepted_; result.commands_rejected = commands_rejected_;
      result.controls_received = controls_received_; result.health_checks = health_checks_;
      result.gid_rejections = gid_rejections_; result.last_command_source_age_s = last_command_source_age_s_;
      result.last_command_receipt_s = last_command_receipt_s_; result.max_callback_wall_s = max_callback_wall_s_;
      result.invalid_scans = invalid_scans_; result.invalid_odometry = invalid_odometry_; result.invalid_poses = invalid_poses_;
      if (include_timings || closed_) {
        constexpr std::array<const char*, 7> names{"command", "scan", "odom", "pose", "tf", "control", "health"};
        for (std::size_t index = 0; index < names.size(); ++index) {
          result.callback_timings.push_back(timings_[index].snapshot(names[index]));
        }
        result.command_source_age = source_ages_.snapshot("command_source_age");
      }
    }
    { std::lock_guard<std::mutex> lock(error_mutex_); result.error = error_text_; }
    return result;
  }

 private:
  template <typename Function> void measured(std::size_t kind, Function&& function) {
    const auto started = receipt_now(), cpu_started = thread_cpu_now();
    try { function(); }
    catch (...) {
      record_timing(kind, receipt_now() - started, thread_cpu_now() - cpu_started);
      throw;
    }
    const double wall = receipt_now() - started;
    record_timing(kind, wall, thread_cpu_now() - cpu_started);
  }
  void record_timing(std::size_t kind, double wall, double cpu) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    max_callback_wall_s_ = std::max(max_callback_wall_s_, wall);
    timings_[kind].add(wall, cpu);
  }
  double source_now() const { return static_cast<double>(node_->get_clock()->now().nanoseconds()) * 1e-9; }
  void record(const std::string& name, const builtin_interfaces::msg::Time& stamp) {
    sensor_times_[name] = {receipt_now(), seconds(stamp)};
  }
  void scan(const sensor_msgs::msg::LaserScan& message) {
    const bool valid = message.header.frame_id == "lidar_link" && message.ranges.size() >= 180 &&
        message.range_min > 0. && message.range_max > message.range_min &&
        std::any_of(message.ranges.begin(), message.ranges.end(), [&](float range) {
          return std::isfinite(range) && range >= message.range_min && range <= message.range_max;
        });
    if (valid) record("scan", message.header.stamp);
    else {
      { std::lock_guard<std::mutex> lock(metrics_mutex_); ++invalid_scans_; }
      authority_->update_health(false, "navigation_invalid_scan", receipt_now());
    }
    // FOLLOW requires observed clearance over the complete surrounding plane.
    // REP117 +infinity is an observed no-return within range_max. NaN,
    // -infinity (too close) or missing coverage cannot mean free space. The
    // internal lower bound caps +infinity at range_max; source data is intact.
    const double step = std::abs(static_cast<double>(message.angle_increment));
    const double coverage = step * static_cast<double>(message.ranges.size());
    const double angular_tolerance = 4. * std::numeric_limits<float>::epsilon() *
        std::max(1., coverage);
    const bool full_scan = valid && std::isfinite(message.range_max) &&
        std::isfinite(message.angle_min) && std::isfinite(message.angle_max) &&
        std::isfinite(step) && step > 0. &&
        std::abs(static_cast<double>(message.angle_max-message.angle_min) -
            message.angle_increment * static_cast<double>(message.ranges.size()-1)) <= angular_tolerance &&
        step <= 2. * std::acos(-1.) / 180. + angular_tolerance &&
        coverage >= 2. * std::acos(-1.) - angular_tolerance &&
        std::all_of(message.ranges.begin(), message.ranges.end(), [&](float range) {
          return range == std::numeric_limits<float>::infinity() ||
            (std::isfinite(range) && range >= message.range_min && range <= message.range_max);
        });
    const double clearance = full_scan ? std::min(static_cast<double>(message.range_max),
        static_cast<double>(*std::min_element(message.ranges.begin(), message.ranges.end()))) : 0.;
    authority_->update_follow_obstacle(clearance, seconds(message.header.stamp),
        source_now(), receipt_now());
  }
  void odom(const nav_msgs::msg::Odometry& message) {
    const auto& position = message.pose.pose.position;
    const auto& twist = message.twist.twist;
    if (message.header.frame_id == "odom" && message.child_frame_id == "base_footprint" &&
        std::isfinite(position.x) && std::isfinite(position.y) &&
        std::isfinite(twist.linear.x) && std::isfinite(twist.angular.z)) {
      record("odom", message.header.stamp);
    } else {
      std::lock_guard<std::mutex> lock(metrics_mutex_); ++invalid_odometry_;
    }
  }
  void pose(const geometry_msgs::msg::PoseWithCovarianceStamped& message) {
    const auto& covariance = message.pose.covariance;
    covariance_valid_ = message.header.frame_id == "map" &&
        std::all_of(covariance.begin(), covariance.end(), [](double v) { return std::isfinite(v); });
    for (const auto index : {0, 7, 35}) {
      covariance_valid_ = covariance_valid_ && covariance[index] >= 0. && covariance[index] <= .25;
    }
    if (covariance_valid_) record("localization_pose", message.header.stamp);
    else {
      { std::lock_guard<std::mutex> lock(metrics_mutex_); ++invalid_poses_; }
      authority_->update_health(false, "navigation_localization_covariance", receipt_now());
    }
  }
  void transforms(const tf2_msgs::msg::TFMessage& message) {
    for (const auto& transform : message.transforms) {
      if (transform.header.frame_id == "map" && transform.child_frame_id == "odom") {
        record("localization_tf", transform.header.stamp);
      }
    }
  }
  bool endpoint_valid() {
    const auto endpoints = node_->get_publishers_info_by_topic(command_topic_);
    if (endpoints.size() != 1) {
      if (endpoints.size() > 1) endpoint_fault_ = "navigation_multiple_command_publishers";
      return false;
    }
    const auto& endpoint = endpoints.front();
    if (endpoint.node_name() == "_NODE_NAME_UNKNOWN_" || endpoint.node_namespace() == "_NODE_NAMESPACE_UNKNOWN_") {
      return false;  // Unknown Graph identity is never an authorization.
    }
    std::string name = endpoint.node_namespace();
    if (!name.empty() && name.back() == '/') name.pop_back();
    name += "/" + endpoint.node_name();
    if (name != controller_ || endpoint.topic_type() != "geometry_msgs/msg/TwistStamped") {
      endpoint_fault_ = "navigation_unapproved_command_publisher";
      return false;
    }
    const Gid gid = endpoint.endpoint_gid();
    if (!controller_gid_) controller_gid_ = gid;
    else if (gid != *controller_gid_) endpoint_fault_ = "navigation_controller_restarted_requires_runtime_restart";
    return endpoint_fault_.empty();
  }
  bool health() {
    const double now = source_now(), receipt = receipt_now();
    std::string reason = closed_ ? "navigation_runtime_shutdown" : endpoint_fault_;
    if (reason.empty() && !endpoint_valid()) {
      reason = endpoint_fault_.empty() ? "navigation_controller_endpoint_unavailable" : endpoint_fault_;
    }
    if (reason.empty() && !covariance_valid_) reason = "navigation_localization_unavailable";
    for (const auto& [name, max_age] : std::array<std::pair<const char*, double>, 3>{
             {{"scan", .35}, {"odom", .12}, {"localization_tf", .35}}}) {
      const auto value = sensor_times_.find(name);
      if (reason.empty() && (value == sensor_times_.end() || receipt - value->second.first > max_age ||
          now - value->second.second < -.3 || now - value->second.second > max_age)) {
        reason = std::string("navigation_") + name + "_unavailable";
      }
    }
    {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      ++health_checks_;
      last_reason_ = reason.empty() ? "navigation_health_valid" : reason;
    }
    authority_->update_health(reason.empty(), reason.empty() ? "navigation_health_valid" : reason, receipt);
    return reason.empty();
  }
  void log_command(const Twist& message, bool accepted, const std::string& reason) {
    const auto authority = authority_->snapshot(false);
    const double now = source_now(), receipt = receipt_now();
    const auto limited = accepted ? authority_->sample(now, receipt) : MotionIntent{};
    commands_log_ << Json{{"received_at_monotonic_s", receipt}, {"ros_observation_time_s", now},
        {"source_ros_time_s", seconds(message.header.stamp)}, {"source_frame", message.header.frame_id},
        {"leased_task_id", nullable(authority.task_id)}, {"authority_epoch", authority.authority_epoch},
        {"requested_linear_m_s", message.twist.linear.x}, {"requested_yaw_rad_s", message.twist.angular.z},
        {"limited_linear_m_s", accepted ? Json(limited.linear_velocity_target_m_s) : Json(nullptr)},
        {"limited_yaw_rad_s", accepted ? Json(limited.yaw_rate_target_rad_s) : Json(nullptr)},
        {"accepted", accepted}, {"reason", reason},
        {"actual_actuator_feedback", "separate_frozen_control_history"}}.dump() << '\n';
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    if (accepted) ++commands_accepted_; else ++commands_rejected_;
  }
  void command(const Twist& message, const rclcpp::MessageInfo& info) {
    {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      ++commands_received_; last_command_receipt_s_ = receipt_now();
      last_command_source_age_s_ = source_now() - seconds(message.header.stamp);
      source_ages_.add(last_command_source_age_s_, 0.);
    }
    if (!health()) {
      std::string reason;
      { std::lock_guard<std::mutex> lock(metrics_mutex_); reason = last_reason_; }
      log_command(message, false, reason); return;
    }
    Gid actual_gid{};
    const auto& raw_gid = info.get_rmw_message_info().publisher_gid;
    std::copy(std::begin(raw_gid.data), std::end(raw_gid.data), actual_gid.begin());
    if (!controller_gid_ || actual_gid != *controller_gid_) {
      endpoint_fault_ = "navigation_command_publisher_gid_mismatch";
      authority_->stop(endpoint_fault_, receipt_now());
      { std::lock_guard<std::mutex> lock(metrics_mutex_); ++gid_rejections_; }
      log_command(message, false, endpoint_fault_); return;
    }
    if (authority_->execution_state().follow) {
      log_command(message, false, "navigation_command_inactive_during_follow");
      return;
    }
    const auto& twist = message.twist;
    const std::array<double, 4> unsupported{twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y};
    if (message.header.frame_id != "base_footprint" ||
        std::any_of(unsupported.begin(), unsupported.end(), [](double value) {
          return !std::isfinite(value) || std::abs(value) > 1e-9;
        })) {
      authority_->invalidate_navigation("navigation_twist_frame_or_nonplanar_component", receipt_now());
      log_command(message, false, "navigation_twist_frame_or_nonplanar_component"); return;
    }
    const bool accepted = authority_->receive_cmd(twist.linear.x, twist.angular.z,
        seconds(message.header.stamp), source_now(), receipt_now());
    log_command(message, accepted, accepted ? "accepted" : authority_->snapshot(false).reason);
  }
  void control(const Control::Request& request, Control::Response& response) {
    if (closed_) {
      response.accepted = false;
      response.reason = "navigation_runtime_shutdown";
      response.authority_epoch = authority_->snapshot(false).authority_epoch;
      return;
    }
    health();
    const double source = source_now(), receipt = receipt_now();
    const auto answer = authority_->control({request.task_id, request.source_epoch, request.sequence,
        request.command, request.lease_duration_s, request.map_id, request.map_version, source, receipt});
    response.accepted = answer.accepted; response.reason = answer.reason;
    response.authority_epoch = answer.authority_epoch;
    controls_log_ << Json{{"received_at_monotonic_s", receipt}, {"ros_observation_time_s", source},
        {"task_id", request.task_id}, {"source_epoch", request.source_epoch}, {"sequence", request.sequence},
        {"command", request.command}, {"lease_duration_s", request.lease_duration_s}, {"map_id", request.map_id},
        {"map_version", request.map_version}, {"accepted", answer.accepted}, {"reason", answer.reason},
        {"authority_epoch", answer.authority_epoch}}.dump() << '\n';
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    ++controls_received_;
  }

  std::shared_ptr<MotionAuthority> authority_;
  const std::string controller_;
  const bool use_sim_time_;
  const std::string command_topic_{"/companionbot/navigation/cmd_vel"};
  mutable std::mutex lifecycle_mutex_, metrics_mutex_, error_mutex_;
  std::atomic<bool> started_{false}, closed_{false}, thread_stopped_{true};
  std::exception_ptr error_;
  std::string error_text_, last_reason_{"navigation_starting"}, endpoint_fault_;
  bool covariance_valid_{};
  std::optional<Gid> controller_gid_;
  std::map<std::string, std::pair<double, double>> sensor_times_;
  std::ofstream commands_log_, controls_log_;
  std::thread worker_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Subscription<Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Service<Control>::SharedPtr control_service_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr clock_callback_;
  std::uint64_t commands_received_{}, commands_accepted_{}, commands_rejected_{},
      controls_received_{}, health_checks_{}, gid_rejections_{}, invalid_scans_{}, invalid_odometry_{}, invalid_poses_{};
  double last_command_source_age_s_{}, last_command_receipt_s_{}, max_callback_wall_s_{};
  std::array<TimingSamples, 7> timings_;
  TimingSamples source_ages_;
};

NavigationIngressRuntime::NavigationIngressRuntime(std::shared_ptr<MotionAuthority> authority,
    const std::string& output_dir, const std::string& controller, bool use_sim_time)
    : impl_(std::make_unique<Impl>(std::move(authority), output_dir, controller, use_sim_time)) {}
NavigationIngressRuntime::~NavigationIngressRuntime() {
  try { impl_->close(); } catch (...) { /* explicit close reports errors to the host */ }
}
void NavigationIngressRuntime::start() { impl_->start(); }
void NavigationIngressRuntime::check_health() const { impl_->check_health(); }
void NavigationIngressRuntime::close() { impl_->close(); }
NavigationIngressSnapshot NavigationIngressRuntime::snapshot(bool include_timings) const { return impl_->snapshot(include_timings); }
}  // namespace companionbot::follow
