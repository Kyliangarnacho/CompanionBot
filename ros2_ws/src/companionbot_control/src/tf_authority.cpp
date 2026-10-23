// Jazzy rclpy does not expose publisher_gid. Keep actual transport authority
// inspection in rclcpp; this node never publishes transforms or robot commands.
#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class TfAuthority final : public rclcpp::Node {
 public:
  TfAuthority() : Node("tf_authority", "/companionbot") {
    map_authority_ = declare_parameter<std::string>("map_authority", "");
    if (!map_authority_.empty()) {
      if (map_authority_.front() != '/' || map_authority_.back() == '/')
        throw std::runtime_error("map_authority_must_be_fully_qualified");
      parents_.emplace("odom", "map");
    }
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    dynamic_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m, const rclcpp::MessageInfo &info) {
        inspect(*m, info, "/tf");
      });
    fixed_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(100).transient_local(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m, const rclcpp::MessageInfo &info) {
        inspect(*m, info, "/tf_static");
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {resolve_pending(); report("");});
  }

 private:
  using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
  struct Owner {Gid gid; std::string parent, name, topic;};
  struct Pending {Gid gid; std::string parent, topic; std::chrono::steady_clock::time_point first;};
  std::string map_authority_;
  std::map<std::string, std::string> parents_{
    {"base_footprint", "odom"}, {"base_link", "base_footprint"},
    {"left_wheel_link", "base_link"}, {"right_wheel_link", "base_link"},
    {"imu_link", "base_link"}, {"lidar_link", "base_link"},
    {"camera_link", "base_link"}, {"camera_optical_frame", "camera_link"}};
  std::map<std::string, Owner> owners_;
  std::map<std::string, Pending> pending_;  // At most the eight known child frames.
  uint64_t graph_pending_observations_ = 0;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dynamic_, fixed_;
  rclcpp::TimerBase::SharedPtr timer_;

  [[noreturn]] void fail(const std::string &error) {
    report(error);
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    throw std::runtime_error(error);
  }

  void inspect(const tf2_msgs::msg::TFMessage &message, const rclcpp::MessageInfo &info,
               const std::string &topic) {
    Gid gid{};
    const auto &raw = info.get_rmw_message_info().publisher_gid;
    std::copy(std::begin(raw.data), std::end(raw.data), gid.begin());
    for (const auto &tf : message.transforms) {
      if (parents_.count(tf.child_frame_id) == 0) continue;
      if (inspect_one(tf.child_frame_id, tf.header.frame_id, gid, topic)) {
        pending_.erase(tf.child_frame_id);
      } else {
        const auto old = pending_.find(tf.child_frame_id);
        if (old != pending_.end() && old->second.gid != gid)
          fail("tf_multiple_authorities:" + tf.child_frame_id);
        const bool added = pending_.emplace(tf.child_frame_id,
          Pending{gid, tf.header.frame_id, topic, std::chrono::steady_clock::now()}).second;
        if (added) ++graph_pending_observations_;
      }
    }
  }

  bool inspect_one(const std::string &child, const std::string &parent, const Gid &gid,
                   const std::string &topic) {
      if (parent != parents_.at(child)) fail("tf_parent_conflict:" + child);
      const auto previous = owners_.find(child);
      if (previous != owners_.end()) {
        if (previous->second.gid != gid) fail("tf_multiple_authorities:" + child);
        if (previous->second.topic != topic) fail("tf_static_dynamic_conflict:" + child);
        return true;
      }
      const bool spatial = child == "base_link" || child == "base_footprint";
      const bool map = child == "odom";
      const auto slash = map_authority_.find_last_of('/');
      const std::string wanted = map ? map_authority_.substr(slash + 1) :
        spatial ? "spatial_state" : "robot_state_publisher";
      const std::string wanted_namespace = map ? (slash == 0 ? "/" : map_authority_.substr(0, slash)) : get_namespace();
      const bool wheel = child == "left_wheel_link" || child == "right_wheel_link";
      if (topic != ((spatial || wheel || map) ? "/tf" : "/tf_static"))
        fail("tf_static_dynamic_conflict:" + child);
      bool matched = false;
      for (const auto &endpoint : get_publishers_info_by_topic(topic)) {
        if (endpoint.endpoint_gid() != gid) continue;
        if (matched) fail("tf_authority_endpoint_ambiguous");
        matched = true;
        // DDS endpoint discovery can precede node-name Graph discovery.
        // Keep the real GID pending; never approve an unknown name or guess it.
        if (endpoint.node_name() == "_NODE_NAME_UNKNOWN_" ||
            endpoint.node_namespace() == "_NODE_NAMESPACE_UNKNOWN_") return false;
        if (endpoint.node_name() != wanted || endpoint.node_namespace() != wanted_namespace)
          fail("tf_unexpected_authority:" + child + ":observed=" + endpoint.node_namespace() + "/" +
               endpoint.node_name() + ":expected=" + wanted_namespace + "/" + wanted);
      }
      if (!matched) return false;
      owners_.emplace(child, Owner{gid, parent, wanted, topic});
      return true;
  }

  void resolve_pending() {
    for (auto item = pending_.begin(); item != pending_.end();) {
      const auto &p = item->second;
      if (inspect_one(item->first, p.parent, p.gid, p.topic)) item = pending_.erase(item);
      else {
        if (std::chrono::steady_clock::now() - p.first > std::chrono::seconds(1))
          fail("tf_authority_endpoint_unresolved:" + item->first);
        ++item;
      }
    }
  }

  void report(const std::string &error) {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(get_namespace()) + "/tf_authority";
    status.hardware_id = "transport_authority_only";
    status.level = error.empty() ? (pending_.empty() ? status.OK : status.WARN) : status.ERROR;
    status.message = error.empty() ? (pending_.empty() ? "RUNNING" : "AWAITING_GRAPH_IDENTITY") : error;
    diagnostic_msgs::msg::KeyValue pending_count, observations;
    pending_count.key = "meta.pending_count";
    pending_count.value = std::to_string(pending_.size());
    observations.key = "meta.graph_pending_observations";
    observations.value = std::to_string(graph_pending_observations_);
    status.values = {pending_count, observations};
    for (const auto &[child, owner] : owners_) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = child;
      std::ostringstream text;
      text << owner.parent << "|" << owner.name << "|" << owner.topic << "|";
      for (uint8_t byte : owner.gid) text << std::hex << std::setfill('0') << std::setw(2) << unsigned(byte);
      value.value = text.str();
      status.values.push_back(value);
    }
    message.status.push_back(status);
    diagnostics_->publish(message);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<TfAuthority>());}
  catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("tf_authority"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
