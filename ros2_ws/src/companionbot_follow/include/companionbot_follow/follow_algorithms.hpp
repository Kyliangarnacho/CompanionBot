#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "companionbot_follow/motion_authority.hpp"

namespace companionbot::follow {

// Algorithm objects have one Pi runtime caller. Cross-thread task authority is
// separately synchronized by MotionAuthority; these are not ROS callbacks.
struct FollowObservation {
  double capture_time_s{}, x_forward_m{}, y_left_m{};
  std::uint32_t version{1};
  std::uint64_t sequence_id{};
  bool valid{true};
  double confidence{1.};
};
struct FollowRobotState {
  std::uint64_t sequence_id{};
  double sample_time_s{}, v_hat_m_s{};
};
struct RadialKFConfig {
  double initial_distance_std{}, initial_velocity_std{}, measurement_distance_std{}, master_accel_std{};
};
struct RadialKFResult {
  double distance_hat_m{}, master_radial_velocity_hat_m_s{}, innovation_m{};
};
struct RadialEstimate {
  std::uint64_t sequence_id{};
  double capture_time_s{}, distance_hat_m{}, master_radial_velocity_hat_m_s{},
      aligned_robot_velocity_m_s{}, beta_rad{}, innovation_m{};
};

class RadialVelocityKalmanFilter {
 public:
  explicit RadialVelocityKalmanFilter(RadialKFConfig config);
  RadialKFResult update(double capture_time_s, double distance_meas_m, double robot_radial_velocity_m_s);
  const std::optional<std::array<double, 2>>& x() const { return x_; }
  const std::optional<std::array<double, 4>>& covariance() const { return covariance_; }
  std::optional<double> last_capture_time_s() const { return last_capture_time_s_; }
  double innovation_m() const { return innovation_m_; }
 private:
  RadialKFConfig config_;
  std::optional<std::array<double, 2>> x_;
  std::optional<std::array<double, 4>> covariance_;
  std::optional<double> last_capture_time_s_;
  double innovation_m_{};
};

struct StateAlignment {
  double velocity_m_s{}, aligned_time_s{};
  std::uint64_t left_sequence{}, right_sequence{};
  double left_time_s{}, right_time_s{}, alpha{};
};
struct StateHistorySnapshot {
  std::size_t retained_samples{};
  std::uint64_t accepted_count{}, out_of_order_rejections{}, capacity_rejections{};
};
class PiRobotStateHistory {
 public:
  static constexpr std::size_t kCapacity = 1024;
  explicit PiRobotStateHistory(double retention_s = 2.);
  bool accept(const FollowRobotState& state);
  std::optional<StateAlignment> velocity_at(double capture_time_s) const;
  StateHistorySnapshot snapshot() const;
  std::optional<FollowRobotState> latest() const;
 private:
  double retention_s_;
  std::deque<FollowRobotState> samples_;
  std::uint64_t accepted_count_{}, out_of_order_rejections_{}, capacity_rejections_{};
};

struct RadialObservationResult {
  FollowObservation observation;
  double arrival_time_s{}, measured_distance_m{}, beta_rad{};
  std::string alignment;
  std::optional<StateAlignment> state_alignment;
  std::optional<RadialEstimate> estimate;
  std::optional<double> raw_radial_velocity_diagnostic_m_s, latest_robot_velocity_diagnostic_m_s;
  bool accepted{};
};
class RadialObservationPipeline {
 public:
  explicit RadialObservationPipeline(RadialKFConfig config, double retention_s = 2.);
  bool accept_state(const FollowRobotState& state) { return history_.accept(state); }
  RadialObservationResult accept_observation(const FollowObservation& observation, double arrival_time_s);
  std::optional<FollowObservation> latest_observation() const { return latest_observation_; }
  std::optional<RadialEstimate> latest_estimate() const { return latest_estimate_; }
  const RadialVelocityKalmanFilter& kf() const { return kf_; }
  const PiRobotStateHistory& state_history() const { return history_; }
  void reset_observations();  // Retain actual, same-clock state interpolation coverage.
 private:
  RadialKFConfig config_;
  RadialVelocityKalmanFilter kf_;
  PiRobotStateHistory history_;
  std::optional<FollowObservation> latest_observation_;
  std::optional<RadialEstimate> latest_estimate_;
  std::optional<double> last_raw_distance_, last_raw_time_;
};

struct FollowGovernorConfig {
  double desired_distance_m{}, d1_m{}, d2_m{}, catch_time_s{}, max_abs_velocity_m_s{};
  double initial_velocity_m_s{}, a_brake_effective_m_s2{.25}, catch_retrigger_margin_m_s{.05};
  std::uint32_t catch_retrigger_persistence_samples{3};
  double slowdown_velocity_deadband_m_s{.05};
};
struct GovernorState {
  std::string state{"CRUISE"};
  double latched_velocity_m_s{}, robot_velocity_hat_m_s{}, master_velocity_hat_m_s{},
      closing_speed_m_s{}, transition_distance_m{}, switch_distance_m{}, distance_m{}, beta_rad{},
      distance_rate_hat_m_s{}, robot_radial_velocity_hat_m_s{};
  std::uint64_t duplicate_observation_count{}, increasing_distance_samples{}, unable_to_close_sample_count{}, slowdown_samples{};
  std::optional<std::uint64_t> last_sequence_id;
};
struct GovernorEvent {
  double time_s{};
  std::uint64_t sequence_id{};
  std::string event, reason, previous_state;
  GovernorState values;
  double generated_velocity_m_s{};
  bool command_saturated{};
  std::optional<double> single_sample_kf_master_radial_velocity_m_s,
      single_sample_shadow_latched_velocity_m_s, cruise_velocity_median_m_s, cruise_velocity_long_median_m_s;
  std::optional<std::size_t> cruise_velocity_window_samples, cruise_velocity_long_window_samples;
};
struct GovernorObservationRow {
  FollowObservation observation;
  GovernorState values;
  double d2_m{};
  std::optional<double> derivative_m_s;
  bool catch_command_saturated{}, unable_to_close{}, command_changed{};
  std::optional<std::string> slowdown_rejection_reason;
};
struct GovernorResult {
  MotionIntent intent;
  std::optional<GovernorEvent> event;
  std::optional<GovernorObservationRow> row;
  bool duplicate{};
};
class FollowGovernor {
 public:
  static constexpr std::size_t kEventCapacity = 2048;
  explicit FollowGovernor(FollowGovernorConfig config, bool use_radial_estimates = false);
  void set_robot_velocity_hat(double velocity_m_s);
  GovernorResult update(const FollowObservation& observation,
                        const std::optional<RadialEstimate>& estimate = std::nullopt);
  GovernorState snapshot() const { return state_; }
  std::vector<GovernorEvent> events() const { return {events_.begin(), events_.end()}; }
  std::uint64_t events_overwritten() const { return events_overwritten_; }
 private:
  GovernorEvent transition(const FollowObservation& observation, const std::string& state,
                           double generated_velocity_m_s, const std::string& reason);
  GovernorObservationRow observation_row(const FollowObservation&, std::optional<double> derivative, bool accepted) const;
  MotionIntent intent(const FollowObservation& observation) const;
  std::pair<double, double> medians() const;
  FollowGovernorConfig config_;
  bool use_radial_estimates_;
  GovernorState state_;
  std::optional<FollowObservation> last_observation_;
  std::deque<double> recent_master_velocity_;
  std::deque<GovernorEvent> events_;
  std::uint64_t events_overwritten_{};
};

class BearingYawServo {
 public:
  BearingYawServo(double gain_s_inv, double max_rate_rad_s, double deadband_rad = 0.);
  std::pair<double, double> command(double x_forward_m, double y_left_m) const;
 private:
  double gain_, max_rate_, deadband_;
};

struct FollowControllerResult {
  bool accepted{}, command_accepted{};
  std::string reason;
  std::uint64_t authority_epoch{};
  std::optional<RadialObservationResult> radial;
  std::optional<GovernorResult> governor;
  std::optional<double> bearing_rad, yaw_rate_rad_s;
};
struct FollowControllerSnapshot {
  std::uint64_t measurement_calls{}, measurements_accepted{}, commands_accepted{},
      invalid_measurements{}, out_of_order_measurements{}, algorithm_resets{};
  std::optional<std::uint64_t> authority_epoch;
  std::optional<std::string> camera_epoch;
  GovernorState governor;
  StateHistorySnapshot state_history;
};

// One complete follow algorithm owner. The external adapter provides only
// source-mapped model measurements and consumed MCU RobotState packets. It
// cannot inject actor truth or grant a task. Lock order: controller->authority;
// MotionAuthority never calls back into this object or the transport runtime.
class FollowController {
 public:
  FollowController(std::shared_ptr<MotionAuthority> authority, RadialKFConfig kf_config,
      FollowGovernorConfig governor_config, double history_retention_s,
      double yaw_gain_s_inv, double yaw_max_rate_rad_s, double yaw_deadband_rad);
  bool accept_state(const FollowRobotState& state);
  FollowControllerResult accept_measurement(const std::string& epoch, std::uint64_t sequence,
      double capture_source_s, double source_now_s, double receipt_now_s,
      double x_forward_m, double y_left_m, bool valid, const std::string& reason);
  FollowControllerSnapshot snapshot() const;
 private:
  void reset_algorithms();
  mutable std::mutex mutex_;
  std::shared_ptr<MotionAuthority> authority_;
  FollowGovernorConfig governor_config_;
  RadialObservationPipeline pipeline_;
  FollowGovernor governor_;
  BearingYawServo yaw_;
  FollowControllerSnapshot counters_;
};
}  // namespace companionbot::follow
