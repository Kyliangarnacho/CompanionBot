#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "companionbot_follow/motion_authority.hpp"

namespace companionbot::follow {
struct IngressTimingSnapshot {
  std::string name;
  std::uint64_t samples{}, retained_samples{};
  double wall_total_s{}, cpu_total_s{}, wall_max_s{}, cpu_max_s{};
  double wall_p50_s{}, wall_p95_s{}, wall_p99_s{}, cpu_p50_s{}, cpu_p95_s{}, cpu_p99_s{};
};
struct NavigationIngressSnapshot {
  bool started{}, closed{}, thread_stopped{};
  std::string reason, error, node_name;
  std::uint64_t commands_received{}, commands_accepted{}, commands_rejected{},
      controls_received{}, health_checks{}, gid_rejections{}, invalid_scans{}, invalid_odometry{}, invalid_poses{};
  double last_command_source_age_s{}, last_command_receipt_s{}, max_callback_wall_s{};
  std::vector<IngressTimingSnapshot> callback_timings;
  IngressTimingSnapshot command_source_age;
};

// Owns only the low-frequency ROS ingress and its standard rclcpp executor.
// Motion authority is the SAME shared object consumed by the native planner.
// No ROS callback runs MCU control, clears a physical gate or writes actuator.
class NavigationIngressRuntime {
 public:
  NavigationIngressRuntime(std::shared_ptr<MotionAuthority> authority,
                           const std::string& output_dir,
                           const std::string& controller = "/controller_server",
                           bool use_sim_time = true);
  ~NavigationIngressRuntime();
  NavigationIngressRuntime(const NavigationIngressRuntime&) = delete;
  NavigationIngressRuntime& operator=(const NavigationIngressRuntime&) = delete;
  void start();
  void check_health() const;
  void close();
  NavigationIngressSnapshot snapshot(bool include_timings = false) const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace companionbot::follow
