#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace companionbot::follow {

struct MotionIntent {
  double source_time_s{};
  double linear_velocity_target_m_s{};
  double yaw_rate_target_rad_s{};
};

struct AuthorityAnswer {
  bool accepted{};
  std::string reason;
  std::uint64_t authority_epoch{};
};

// Explicitly supplied from the frozen follow configuration by composition.
// Absence disables FOLLOW; navigation's envelope is never reused for follow.
struct FollowAuthorityConfig {
  double max_velocity_m_s{}, max_yaw_rate_rad_s{}, command_timeout_s{}, health_timeout_s{};
  double obstacle_timeout_s{}, braking_timeout_s{};
};

struct AuthorityExecutionState {
  bool authorized{}, follow{};
  std::uint64_t authority_epoch{};
};

// All timestamps are passed explicitly. source_now is the mapped source clock;
// receipt_now is a single local monotonic clock, never a remote device tick.
struct AuthorityControl {
  std::string task_id;
  std::string source_epoch;
  std::uint64_t sequence{};
  std::string command;
  double lease_duration_s{};
  std::string map_id;
  std::string map_version;
  double source_now_s{};
  double receipt_now_s{};
};

struct AuthorityEvent {
  AuthorityEvent(std::string name, double receipt)
      : event(std::move(name)), received_at_s(receipt) {}
  std::string event;
  double received_at_s{};
  std::optional<std::string> reason, task_id, source_epoch, source_id, mode;
  std::optional<std::uint64_t> authority_epoch;
  std::optional<double> source_time_s, requested_v_m_s, requested_yaw_rad_s,
      limited_v_m_s, limited_yaw_rad_s;
  std::optional<bool> saturated;
};

struct AuthoritySnapshot {
  std::optional<std::string> task_id, source_epoch, mcu_state;
  std::string reason, mcu_reason, map_id, map_version;
  std::uint64_t authority_epoch{};
  bool authorized{}, recovery_locked{}, mcu_recovery_owned{};
  std::size_t retained_events{}, retained_watermarks{}, retained_stop_reasons{};
  std::uint64_t events_overwritten{}, capacity_rejections{}, safety_capacity_bypasses{};
  std::vector<AuthorityEvent> events;
  std::string mode{"NONE"}, follow_reason{"follow_disabled"};
  bool follow_enabled{}, follow_health{}, master_locked{};
  std::optional<std::string> follow_source_epoch;
  std::optional<std::uint64_t> follow_sequence;
  std::optional<double> follow_source_time_s, follow_received_at_s,
      follow_source_age_s, follow_receipt_age_s, granted_source_time_s;
  std::uint64_t follow_health_rejections{}, follow_command_rejections{};
  bool follow_obstacle_clear{};
  std::string follow_safety_reason{"follow_obstacle_unavailable"};
  std::optional<double> follow_obstacle_clearance_m, follow_required_clearance_m,
      follow_obstacle_source_time_s, follow_obstacle_received_at_s,
      follow_obstacle_source_age_s, follow_obstacle_receipt_age_s,
      follow_braking_received_at_s, follow_braking_age_s;
  std::uint64_t follow_guard_rejections{};
};

// The sole task-scoped motion authority owner. It never clears MCU safety,
// physical arm or independent faults, and owns neither estimator nor actuator.
class MotionAuthority {
 public:
  static constexpr std::size_t kWatermarkCapacity = 256;
  static constexpr std::size_t kStopReasonCapacity = 256;
  static constexpr std::size_t kEventCapacity = 2048;

  MotionAuthority(std::string map_id, std::string map_version,
                  double max_velocity_m_s = .25, double max_yaw_rate_rad_s = .5,
                  double command_timeout_s = .3, double health_timeout_s = .35,
                  std::optional<FollowAuthorityConfig> follow_config = std::nullopt);
  AuthorityAnswer control(const AuthorityControl& request);
  void update_health(bool available, const std::string& reason, double receipt_now_s);
  void update_mcu_status(const std::string& state, const std::string& reason,
                         double receipt_now_s);
  // Production transports must carry C99 reason provenance, not infer it from
  // human-readable text. The legacy three-argument API is a test/oracle shim.
  void update_mcu_status_with_provenance(const std::string& state, const std::string& reason,
      bool task_owned_stop, double receipt_now_s);
  bool receive_cmd(double linear_m_s, double yaw_rad_s, double source_time_s,
                   double source_now_s, double receipt_now_s,
                   const std::string& source_id = "nav2:controller_server");
  // The perception adapter reports one exact frame's Master/depth/geometry/
  // clock validity. These calls authorize no task and cannot clear MCU faults.
  bool update_follow_health(bool available, const std::string& reason,
      const std::string& observation_epoch, std::uint64_t sequence,
      double source_time_s, double source_now_s, double receipt_now_s);
  bool follow_observation_identity_current(const std::string& observation_epoch,
      std::uint64_t sequence, double source_time_s) const;
  bool receive_follow_cmd(const std::string& task_id, const std::string& task_source_epoch,
      std::uint64_t authority_epoch, const std::string& observation_epoch,
      std::uint64_t observation_sequence, double source_time_s, double source_now_s,
      double receipt_now_s, double linear_m_s, double yaw_rad_s);
  bool update_follow_obstacle(double clearance_m, double source_time_s,
      double source_now_s, double receipt_now_s);
  bool update_follow_braking(double required_clearance_m, double receipt_now_s);
  // For the current perception owner's actual worker failure/closure. This
  // atomically invalidates FOLLOW only; stale frame results use identity gates.
  bool invalidate_follow(const std::string& reason, double receipt_now_s);
  // A malformed Nav2-only input cannot revoke a concurrently acquired FOLLOW.
  bool invalidate_navigation(const std::string& reason, double receipt_now_s);
  MotionIntent sample(double source_now_s, double receipt_now_s);
  void stop(const std::string& reason, double receipt_now_s, bool force = false);
  std::optional<std::pair<std::uint64_t, std::string>> take_stop();
  AuthoritySnapshot snapshot(bool include_events = true) const;
  AuthorityExecutionState execution_state() const;
  std::optional<FollowAuthorityConfig> follow_config() const { return follow_config_; }

  const std::string map_id, map_version;
  const double max_velocity_m_s, max_yaw_rate_rad_s, command_timeout_s, health_timeout_s;

 private:
  struct Latest {
    double source_time_s, received_at_s, linear_m_s, yaw_rad_s;
  };
  void event_locked(AuthorityEvent event);
  void stop_locked(const std::string& reason, double receipt_now_s, bool force = false);
  void check_locked(double source_now_s, double receipt_now_s);
  bool mcu_ready_locked() const;
  void update_mcu_status_locked(const std::string& state, const std::string& reason,
      bool task_owned_stop, double receipt_now_s);
  bool follow_ready_locked(double source_now_s, double receipt_now_s) const;
  bool follow_observation_ready_locked(double source_now_s, double receipt_now_s) const;
  bool follow_observation_identity_locked(const std::string& observation_epoch,
      std::uint64_t sequence, double source_time_s) const;
  std::string follow_safety_reason_locked(double source_now_s, double receipt_now_s) const;
  static std::string bounded_reason(const std::string& reason);

  mutable std::mutex mutex_;
  std::uint64_t authority_epoch_{}, stop_serial_{};
  std::optional<std::string> task_id_, source_epoch_, mcu_state_;
  std::map<std::pair<std::string, std::string>, std::uint64_t> watermarks_;
  double lease_until_{}, granted_source_time_{}, health_at_{};
  std::optional<Latest> latest_;
  enum class Mode { None, GuideTo, Follow };
  Mode mode_{Mode::None};
  const std::optional<FollowAuthorityConfig> follow_config_;
  struct FollowHealth {
    std::string epoch;
    std::uint64_t sequence{};
    double source_time_s{}, received_at_s{};
  };
  std::optional<FollowHealth> follow_health_sample_;
  std::set<std::string> follow_retired_epochs_;
  bool follow_healthy_{};
  std::string follow_reason_{"follow_disabled"};
  double follow_checked_source_s_{}, follow_checked_receipt_s_{};
  std::uint64_t follow_health_rejections_{}, follow_command_rejections_{};
  struct ObstacleSample { double clearance_m, source_time_s, received_at_s; };
  struct BrakingSample { double required_clearance_m, received_at_s; };
  std::optional<ObstacleSample> follow_obstacle_;
  std::optional<BrakingSample> follow_braking_;
  std::uint64_t follow_guard_rejections_{};
  bool healthy_{}, recovery_locked_{}, mcu_recovery_owned_{};
  std::string health_reason_{"navigation_health_unavailable"};
  std::string reason_{"no_navigation_authority"}, mcu_reason_{"mcu_status_unavailable"};
  std::optional<std::pair<std::uint64_t, std::string>> pending_stop_;
  std::set<std::string> owned_stop_reasons_;
  std::deque<AuthorityEvent> events_;
  std::uint64_t events_overwritten_{}, capacity_rejections_{}, safety_capacity_bypasses_{};
};

}  // namespace companionbot::follow
