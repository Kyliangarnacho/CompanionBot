#pragma once

#include "companionbot_follow/lightweight_producer.hpp"
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace companionbot::follow {

struct SchedulerConfig {
  double accept_delta_velocity{.12}, full_delta_velocity{.35};
  double stable_window_s{.20}, stable_range_velocity{.03};
  double initial_accepted_velocity{};
  bool require_stable_candidate{true}, minimum_delta_enabled{true}, force_full_dynamic{};
};
struct TwoPathConfig {
  ProducerConfig producer;
  SchedulerConfig scheduler;
  double feedforward_fade_s{.2}, quiet_theta_rad{}, quiet_theta_dot_rad_s{};
  bool defer_full_accept_until_observed_exit{};
};
struct PlanRequest {
  std::array<double, 4> start_reference{};
  double start_acceleration{}, target_velocity{}, execution_start_time{}, request_time{};
};
// Pure mathematical solver result. No callback owns scheduling/lifecycle state.
struct FullPlan {
  double target_velocity{}, transient_duration{}, ruckig_duration{}, horizon{}, residual_rms{};
  bool feedforward_rearmed{true};
  std::vector<std::array<double, 4>> states;
  std::vector<double> accelerations, inputs;
  std::vector<std::uint8_t> shaped_finished;
  nlohmann::json planning_diagnostics = nlohmann::json::object();
  nlohmann::json projection_diagnostics = nlohmann::json::array();
};
using FullPlanSolver = std::function<FullPlan(const PlanRequest &)>;
struct TwoPathSample {
  std::array<double, 4> reference_state{};
  double acceleration{}, raw_feedforward{}, lifecycle_feedforward{}, fade_alpha{};
  const char *feedforward_phase{}, *velocity_phase{}, *scheduler_mode{};
  bool replanned{}, accepted_changed{};
};
struct TwoPathBlock : ProducedBlock {
  std::array<TwoPathSample, 25> details;
  double raw_velocity{}, candidate_delta{}, pending_velocity{};
  std::optional<double> candidate_target;
  bool candidate_stable{}, pending_active{};
  std::uint64_t block_id{};
};

// Complete Stage 5/6 scheduler, lifecycle and block production owner. Ordinary
// C++ callers supply the low-frequency mathematical FULL solver. pybind11's
// standard functional adapter acquires the GIL only when that solver is used.
// One producer caller owns this object; it is not a ROS executor or MCU gate.
class TwoPathProducer {
 public:
  static constexpr std::size_t kCandidateCapacity = 1024, kLogCapacity = 16384;
  explicit TwoPathProducer(TwoPathConfig config, FullPlanSolver solver = {}, std::uint32_t epoch = 0);
  // Exact navigation policy: immediate LIGHTWEIGHT changes, no FULL downgrade.
  explicit TwoPathProducer(ProducerConfig config);
  void reset(double position, double velocity, std::uint32_t epoch);
  void set_policy(SchedulerConfig scheduler, bool defer_full_exit);
  TwoPathBlock produce(std::uint32_t start_tick, double generated_time,
                       double source_time, double velocity, double yaw,
                       bool observe_command = true);
  std::array<double, 4> next_reference() const;
  double next_acceleration() const;
  std::uint32_t epoch() const { return epoch_; }
  std::uint32_t next_sequence() const { return sequence_; }
  const ProducerConfig &config() const { return config_.producer; }
  std::string summary_json() const;

 private:
  struct Log {
    void push(nlohmann::json value);
    nlohmann::json export_rows() const;
    std::deque<nlohmann::json> rows;
    std::uint64_t total{}, overwritten{};
  };
  struct Scheduled {
    bool accepted{};
    std::string mode{"HOLD"};
    double raw{}, accepted_velocity{}, delta{};
    bool stable{};
    std::optional<double> target;
  };
  struct LifecycleCommand {
    std::array<double, 4> reference{};
    double acceleration{}, feedforward{}, fade_alpha{};
    bool fade_started{}, fade_finished{};
    const char *feedforward_phase{"OFF"}, *velocity_phase{};
  };
  static TwoPathConfig navigation_config(ProducerConfig);
  void clear_candidate();
  std::optional<double> candidate_target() const;
  double candidate_delta() const;
  Scheduled schedule(double now, double raw, bool allow_accept);
  FullPlan lightweight_plan(const PlanRequest &);
  void validate_plan(const FullPlan &, const PlanRequest &) const;
  void begin_plan(double time, const Scheduled &, double request_time);
  LifecycleCommand lifecycle_command(double time);
  LifecycleCommand enter_quiet_hold(double time);
  double raw_feedforward() const;
  bool advance();
  void encode(TwoPathBlock &) const;

  TwoPathConfig config_;
  FullPlanSolver solver_;
  ruckig::Ruckig<1> ruckig_;
  ruckig::Trajectory<1> trajectory_;
  std::size_t plan_capacity_{};
  std::uint32_t epoch_{}, sequence_{};
  std::uint64_t block_count_{};
  double accepted_{}, latest_pending_{}, hold_position_{}, hold_velocity_{};
  std::optional<double> candidate_start_, candidate_baseline_;
  std::deque<std::pair<double, double>> candidate_window_;
  bool candidate_stable_{}, transient_{}, fade_completed_{};
  std::optional<double> fade_start_;
  double feedforward_exit_{};
  std::optional<FullPlan> plan_;
  std::size_t interval_index_{};
  std::string active_mode_{"HOLD"};
  bool full_locked_{}, full_quiet_latched_{}, pending_from_full_exit_{}, observed_{};
  double last_source_{}, last_raw_{}, last_yaw_{};
  std::optional<double> last_full_exit_time_;
  std::optional<std::uint64_t> active_full_event_id_;
  Log scheduler_events_, intents_, replans_, full_exits_, fades_, quiets_;
};
}  // namespace companionbot::follow
