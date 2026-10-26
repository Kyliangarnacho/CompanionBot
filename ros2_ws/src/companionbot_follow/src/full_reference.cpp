#include "companionbot_follow/full_reference.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <time.h>

namespace companionbot::follow {
namespace {
using json = nlohmann::json;
double thread_seconds() {
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value)) throw std::runtime_error("thread_cpu_clock_unavailable");
  return static_cast<double>(value.tv_sec) + static_cast<double>(value.tv_nsec) * 1e-9;
}
void u32(std::uint8_t *out, std::uint32_t value) {
  for (unsigned i = 0; i < 4; ++i) out[i] = static_cast<std::uint8_t>(value >> (8 * i));
}
void f32(std::uint8_t *out, double value) {
  const float converted = static_cast<float>(value);
  if (!std::isfinite(converted)) throw std::overflow_error("reference_float32_overflow");
  std::uint32_t bits{}; std::memcpy(&bits, &converted, sizeof(bits)); u32(out, bits);
}
void f64(std::uint8_t *out, double value) {
  std::uint64_t bits{}; std::memcpy(&bits, &value, sizeof(bits));
  for (unsigned i = 0; i < 8; ++i) out[i] = static_cast<std::uint8_t>(bits >> (8 * i));
}
json nullable(const std::optional<double> &value) { return value ? json(*value) : json(nullptr); }
bool finite_positive(double value) { return std::isfinite(value) && value > 0.; }
}

void TwoPathProducer::Log::push(json value) {
  if (rows.size() == kLogCapacity) { rows.pop_front(); ++overwritten; }
  rows.push_back(std::move(value)); ++total;
}
json TwoPathProducer::Log::export_rows() const { return json(rows); }
TwoPathConfig TwoPathProducer::navigation_config(ProducerConfig config) {
  TwoPathConfig result;
  result.producer = config;
  result.scheduler.full_delta_velocity = config.full_delta_velocity;
  result.scheduler.require_stable_candidate = false;
  result.scheduler.minimum_delta_enabled = false;
  return result;
}
TwoPathProducer::TwoPathProducer(ProducerConfig config)
  : TwoPathProducer(navigation_config(config)) {}
TwoPathProducer::TwoPathProducer(TwoPathConfig config, FullPlanSolver solver, std::uint32_t epoch)
  : config_(config), solver_(std::move(solver)), ruckig_(config.producer.dt), epoch_(epoch) {
  const auto &p = config_.producer;
  const auto &s = config_.scheduler;
  if (!std::isfinite(p.dt) || std::abs(p.dt - .002) > 1e-12 || p.block_samples != 25 ||
      !finite_positive(p.max_velocity) || !finite_positive(p.max_acceleration) || !finite_positive(p.max_jerk) ||
      !finite_positive(s.accept_delta_velocity) || !finite_positive(s.full_delta_velocity) ||
      !finite_positive(s.stable_window_s) || !finite_positive(s.stable_range_velocity) ||
      !std::isfinite(s.initial_accepted_velocity) || std::abs(s.initial_accepted_velocity) > p.max_velocity ||
      !finite_positive(config_.feedforward_fade_s) || !std::isfinite(config_.quiet_theta_rad) ||
      !std::isfinite(config_.quiet_theta_dot_rad_s) || config_.quiet_theta_rad < 0. || config_.quiet_theta_dot_rad_s < 0.)
    throw std::invalid_argument("invalid_two_path_configuration");
  const double bound = (2 * p.max_velocity / p.max_acceleration +
      4 * p.max_acceleration / p.max_jerk + 8 * p.dt) / p.dt;
  if (!std::isfinite(bound) || bound > 1000000.) throw std::invalid_argument("two_path_profile_capacity_excessive");
  // Operational memory bound, not a changed planning horizon. Frozen FULL
  // plans are much shorter; untrusted solver outputs cannot allocate forever.
  plan_capacity_ = std::max<std::size_t>(static_cast<std::size_t>(std::ceil(bound)) + 1, 16384);
  reset(0., s.initial_accepted_velocity, epoch);
}
void TwoPathProducer::reset(double position, double velocity, std::uint32_t epoch) {
  if (!std::isfinite(position) || !std::isfinite(velocity) || std::abs(velocity) > config_.producer.max_velocity)
    throw std::invalid_argument("invalid_two_path_reset");
  epoch_ = epoch; sequence_ = 0; block_count_ = 0;
  accepted_ = latest_pending_ = velocity; hold_position_ = position; hold_velocity_ = velocity;
  clear_candidate(); transient_ = fade_completed_ = false; fade_start_.reset(); feedforward_exit_ = 0.;
  plan_.reset(); interval_index_ = 0; active_mode_ = "HOLD";
  full_locked_ = full_quiet_latched_ = pending_from_full_exit_ = observed_ = false;
  last_source_ = last_raw_ = last_yaw_ = 0.; last_full_exit_time_.reset(); active_full_event_id_.reset();
  scheduler_events_ = {}; intents_ = {}; replans_ = {}; full_exits_ = {}; fades_ = {}; quiets_ = {};
}
void TwoPathProducer::set_policy(SchedulerConfig scheduler, bool defer_full_exit) {
  if (transient_ || full_locked_ || fade_start_ || candidate_start_ ||
      std::abs(hold_velocity_) > 1e-12 || !finite_positive(scheduler.accept_delta_velocity) ||
      !finite_positive(scheduler.full_delta_velocity) || !finite_positive(scheduler.stable_window_s) ||
      !finite_positive(scheduler.stable_range_velocity))
    throw std::logic_error("two_path_policy_change_requires_stationary_reset");
  config_.scheduler = scheduler;
  config_.defer_full_accept_until_observed_exit = defer_full_exit;
}
void TwoPathProducer::clear_candidate() {
  candidate_start_.reset(); candidate_baseline_.reset(); candidate_window_.clear(); candidate_stable_ = false;
}
std::optional<double> TwoPathProducer::candidate_target() const {
  return candidate_window_.empty() ? std::nullopt : std::optional<double>(candidate_window_.back().second);
}
double TwoPathProducer::candidate_delta() const {
  return candidate_baseline_ && !candidate_window_.empty() ? candidate_window_.back().second - *candidate_baseline_ : 0.;
}
TwoPathProducer::Scheduled TwoPathProducer::schedule(double now, double raw, bool allow_accept) {
  const auto &cfg = config_.scheduler;
  latest_pending_ = raw;
  const double delta = raw - accepted_;
  if (std::abs(delta) <= 1e-12 || (cfg.minimum_delta_enabled && std::abs(delta) < cfg.accept_delta_velocity)) {
    if (candidate_start_) scheduler_events_.push({{"time_s", now}, {"event", "candidate_cancelled"},
      {"raw_velocity_m_s", raw}, {"frozen_accepted_baseline_m_s", nullable(candidate_baseline_)},
      {"reason", std::abs(delta) <= 1e-12 ? "returned_to_accepted_command" : "returned_within_accept_threshold"}});
    clear_candidate();
    return {false, "HOLD", raw, accepted_, 0., false, std::nullopt};
  }
  if (!candidate_start_) {
    candidate_start_ = now; candidate_baseline_ = accepted_; candidate_window_.clear(); candidate_stable_ = false;
    scheduler_events_.push({{"time_s", now}, {"event", "candidate_started"}, {"raw_velocity_m_s", raw},
      {"frozen_accepted_baseline_m_s", *candidate_baseline_}});
  }
  if (candidate_window_.size() == kCandidateCapacity) throw std::runtime_error("two_path_candidate_capacity_exceeded");
  candidate_window_.emplace_back(now, raw);
  const double window_start = now - cfg.stable_window_s;
  while (candidate_window_.size() > 1 && candidate_window_.front().first < window_start - 1e-12)
    candidate_window_.pop_front();
  double low = raw, high = raw;
  for (const auto &entry : candidate_window_) { low = std::min(low, entry.second); high = std::max(high, entry.second); }
  const double spread = high - low;
  const bool stable = !cfg.require_stable_candidate ||
    (now - *candidate_start_ + 1e-12 >= cfg.stable_window_s &&
     candidate_window_.back().first - candidate_window_.front().first + 1e-12 >= cfg.stable_window_s &&
     spread <= cfg.stable_range_velocity + 1e-12);
  if (cfg.require_stable_candidate && stable && !candidate_stable_)
    scheduler_events_.push({{"time_s", now}, {"event", "candidate_stable"}, {"candidate_v_cmd_m_s", raw},
      {"candidate_delta_v_m_s", delta}, {"window_range_m_s", spread},
      {"window_start_time_s", candidate_window_.front().first}, {"window_end_time_s", candidate_window_.back().first}});
  if (cfg.require_stable_candidate && !stable && candidate_stable_)
    scheduler_events_.push({{"time_s", now}, {"event", "candidate_unstable"}, {"raw_velocity_m_s", raw}, {"window_range_m_s", spread}});
  candidate_stable_ = stable && cfg.require_stable_candidate;
  const auto target = stable ? std::optional<double>(raw) : std::nullopt;
  if (!stable || !allow_accept) return {false, "CANDIDATE", raw, accepted_, delta, cfg.require_stable_candidate && stable, target};
  const double new_delta = *target - accepted_;
  if (cfg.minimum_delta_enabled && std::abs(new_delta) < cfg.accept_delta_velocity) {
    scheduler_events_.push({{"time_s", now}, {"event", "candidate_cancelled"}, {"candidate_v_cmd_m_s", *target},
      {"candidate_delta_v_m_s", new_delta}, {"reason", "stable_target_below_accept_threshold"}});
    clear_candidate(); return {false, "HOLD", raw, accepted_, new_delta, false, std::nullopt};
  }
  const std::string mode = cfg.force_full_dynamic || std::abs(new_delta) >= cfg.full_delta_velocity ? "FULL_DYNAMIC" : "LIGHTWEIGHT";
  const double previous = accepted_; accepted_ = *target;
  scheduler_events_.push({{"time_s", now}, {"event", "accepted"}, {"mode", mode}, {"candidate_v_cmd_m_s", *target},
    {"previous_accepted_velocity_m_s", previous}, {"accepted_velocity_m_s", accepted_}, {"candidate_delta_v_m_s", new_delta},
    {"candidate_stable_window_s", candidate_window_.back().first - candidate_window_.front().first},
    {"candidate_window_range_m_s", spread}, {"stable_candidate_gate_bypassed", !cfg.require_stable_candidate},
    {"minimum_delta_gate_bypassed", !cfg.minimum_delta_enabled}, {"force_full_dynamic", cfg.force_full_dynamic}});
  clear_candidate(); return {true, mode, raw, accepted_, new_delta, cfg.require_stable_candidate && stable, target};
}
std::array<double, 4> TwoPathProducer::next_reference() const {
  if (transient_) return plan_->states.at(interval_index_);
  return {hold_position_, hold_velocity_, 0., 0.};
}
double TwoPathProducer::next_acceleration() const { return transient_ ? plan_->accelerations.at(interval_index_) : 0.; }
double TwoPathProducer::raw_feedforward() const {
  return plan_ ? plan_->inputs.at(std::min(interval_index_, plan_->inputs.size() - 1)) : 0.;
}
FullPlan TwoPathProducer::lightweight_plan(const PlanRequest &request) {
  const auto &cfg = config_.producer;
  if (std::abs(request.start_reference[2]) > 1e-12 || std::abs(request.start_reference[3]) > 1e-12)
    throw std::invalid_argument("lightweight_path_requires_exact_quiet_start");
  ruckig::InputParameter<1> input;
  input.control_interface = ruckig::ControlInterface::Velocity;
  input.duration_discretization = ruckig::DurationDiscretization::Discrete;
  input.current_position = {request.start_reference[0]}; input.current_velocity = {request.start_reference[1]};
  input.current_acceleration = {request.start_acceleration}; input.target_velocity = {request.target_velocity};
  input.target_acceleration = {0.}; input.max_velocity = {cfg.max_velocity};
  input.max_acceleration = {cfg.max_acceleration}; input.max_jerk = {cfg.max_jerk};
  const auto status = ruckig_.calculate(input, trajectory_);
  if (status != ruckig::Result::Working && status != ruckig::Result::Finished)
    throw std::runtime_error("ruckig_two_path_calculation_failed");
  FullPlan result;
  result.target_velocity = request.target_velocity; result.transient_duration = result.ruckig_duration = result.horizon = trajectory_.get_duration();
  result.feedforward_rearmed = false;
  const auto intervals = static_cast<std::size_t>(std::nearbyint(result.transient_duration / cfg.dt));
  if (!intervals || intervals + 1 > plan_capacity_ || std::abs(intervals * cfg.dt - result.transient_duration) > 1e-10)
    throw std::runtime_error("ruckig_two_path_profile_geometry");
  result.states.resize(intervals + 1); result.accelerations.resize(intervals + 1);
  result.inputs.resize(intervals); result.shaped_finished.resize(intervals + 1);
  std::array<double, 1> p{}, v{}, a{};
  trajectory_.at_time(result.transient_duration, p, v, a);
  const double end_position = p[0];
  for (std::size_t i = 0; i <= intervals; ++i) {
    const double t = i * cfg.dt;
    if (t >= result.transient_duration - 1e-12) {
      p[0] = end_position + (t - result.transient_duration) * request.target_velocity; v[0] = request.target_velocity; a[0] = 0.;
    } else trajectory_.at_time(t, p, v, a);
    result.states[i] = {p[0], v[0], 0., 0.}; result.accelerations[i] = a[0];
    result.shaped_finished[i] = t >= result.transient_duration - 1e-12;
  }
  result.states.front() = request.start_reference; result.accelerations.front() = request.start_acceleration;
  result.states.back()[1] = request.target_velocity; result.accelerations.back() = 0.;
  return result;
}
void TwoPathProducer::validate_plan(const FullPlan &plan, const PlanRequest &request) const {
  const auto n = plan.states.size();
  if (n < 2 || n > plan_capacity_ || plan.accelerations.size() != n || plan.shaped_finished.size() != n || plan.inputs.size() != n - 1 ||
      !std::isfinite(plan.target_velocity) || !std::isfinite(plan.transient_duration) || plan.transient_duration < 0. ||
      std::abs(plan.target_velocity - request.target_velocity) > 1e-12 || !std::isfinite(plan.ruckig_duration) ||
      !std::isfinite(plan.horizon) || !std::isfinite(plan.residual_rms)) throw std::invalid_argument("invalid_full_plan_geometry");
  for (const auto &state : plan.states) for (double value : state)
    if (!std::isfinite(value)) throw std::invalid_argument("nonfinite_full_plan_state");
  for (double value : plan.accelerations) if (!std::isfinite(value)) throw std::invalid_argument("nonfinite_full_plan_acceleration");
  for (double value : plan.inputs) if (!std::isfinite(value)) throw std::invalid_argument("nonfinite_full_plan_input");
  for (std::size_t i = 0; i < 4; ++i)
    if (std::abs(plan.states.front()[i] - request.start_reference[i]) > 1e-10) throw std::invalid_argument("full_plan_start_discontinuity");
  if (std::abs(plan.states.back()[1] - plan.target_velocity) > 1e-9 || std::abs(plan.states.back()[2]) > 1e-9 ||
      std::abs(plan.states.back()[3]) > 1e-9) throw std::invalid_argument("full_plan_terminal_not_cruise_manifold");
}
void TwoPathProducer::begin_plan(double time, const Scheduled &scheduled, double request_time) {
  PlanRequest request{next_reference(), next_acceleration(), scheduled.accepted_velocity, time, request_time};
  const double old_raw_ff = raw_feedforward();
  const bool mid_replan = scheduled.mode == "LIGHTWEIGHT" && transient_ && active_mode_ == "LIGHTWEIGHT";
  if (scheduled.mode == "FULL_DYNAMIC" && !solver_) throw std::invalid_argument("full_dynamic_solver_required");
  const auto before = std::chrono::steady_clock::now();
  FullPlan rolling = scheduled.mode == "LIGHTWEIGHT" ? lightweight_plan(request) : solver_(request);
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - before).count();
  validate_plan(rolling, request);
  const auto first = rolling.states.front(), terminal = rolling.states.back();
  json event = {{"time_s", time}, {"planning_request_time_s", request_time}, {"execution_start_time_s", time},
    {"planning_complete_time_s", request_time + wall}, {"planning_path", scheduled.mode},
    {"target_velocity_m_s", scheduled.accepted_velocity}, {"candidate_delta_v_m_s", scheduled.delta},
    {"candidate_target_m_s", nullable(scheduled.target)}, {"mid_trajectory_replan", mid_replan}, {"planning_wall_time_s", wall},
    {"ruckig_duration_s", rolling.ruckig_duration}, {"horizon_s", rolling.horizon}, {"normalized_residual_rms", rolling.residual_rms},
    {"p_start_delta_m", first[0] - request.start_reference[0]}, {"v_start_delta_m_s", first[1] - request.start_reference[1]},
    {"a_start_delta_m_s2", rolling.accelerations[0] - request.start_acceleration},
    {"theta_start_delta_rad", first[2] - request.start_reference[2]}, {"theta_dot_start_delta_rad_s", first[3] - request.start_reference[3]},
    {"u_ff_raw_start_delta_nm", rolling.inputs[0] - old_raw_ff}, {"terminal_theta_rad", terminal[2]}, {"terminal_theta_dot_rad_s", terminal[3]}};
  event.update(rolling.planning_diagnostics);
  if (!rolling.projection_diagnostics.empty()) {
    const auto &projection = rolling.projection_diagnostics.at(0);
    event["projection_solver"] = projection.value("solver", json(nullptr));
    const auto stop = projection.value("lsqr_stop_code", json(nullptr));
    event["projection_solver_status"] = stop;
    event["projection_solver_converged"] = stop == 1 || stop == 2;
  } else { event["projection_solver"] = nullptr; event["projection_solver_status"] = nullptr; event["projection_solver_converged"] = nullptr; }
  plan_ = std::move(rolling); transient_ = true; interval_index_ = 0;
  fade_start_.reset(); fade_completed_ = false; feedforward_exit_ = 0.; active_mode_ = scheduled.mode;
  if (scheduled.mode == "FULL_DYNAMIC") {
    full_locked_ = true; full_quiet_latched_ = false;
    event["full_dynamic_horizon_s"] = plan_->horizon; event["quiet_time_s"] = nullptr;
    event["fade_completion_time_s"] = nullptr; event["full_exit_time_s"] = nullptr;
    active_full_event_id_ = replans_.total;
  }
  replans_.push(std::move(event));
}
TwoPathProducer::LifecycleCommand TwoPathProducer::lifecycle_command(double time) {
  LifecycleCommand out;
  out.reference = next_reference(); out.acceleration = next_acceleration();
  out.velocity_phase = transient_ ? "VELOCITY_TRANSIENT" : "VELOCITY_HOLD";
  if (fade_start_) {
    const double fraction = std::clamp((time - *fade_start_) / config_.feedforward_fade_s, 0., 1.);
    if (fraction >= 1.) { out.fade_finished = !fade_completed_; fade_completed_ = true; return out; }
    out.fade_alpha = 1. - 3. * std::pow(fraction, 2) + 2. * std::pow(fraction, 3);
    out.feedforward = out.fade_alpha * feedforward_exit_; out.feedforward_phase = "FADING"; return out;
  }
  if (!plan_ || !plan_->feedforward_rearmed) return out;
  if (interval_index_ == plan_->states.size() - 1) {
    fade_start_ = time; feedforward_exit_ = raw_feedforward(); out.feedforward = feedforward_exit_;
    out.fade_started = true; out.fade_alpha = 1.; out.feedforward_phase = "FADING"; return out;
  }
  if (interval_index_ >= plan_->inputs.size()) throw std::runtime_error("velocity_transition_exceeded_segment");
  out.feedforward = plan_->inputs[interval_index_]; out.feedforward_phase = "TRANSIENT"; return out;
}
TwoPathProducer::LifecycleCommand TwoPathProducer::enter_quiet_hold(double time) {
  const auto reference = next_reference();
  feedforward_exit_ = raw_feedforward(); hold_position_ = reference[0]; hold_velocity_ = reference[1];
  transient_ = false; fade_start_ = time; fade_completed_ = false;
  LifecycleCommand out;
  out.reference = next_reference(); out.velocity_phase = "VELOCITY_HOLD";
  out.feedforward = feedforward_exit_; out.fade_alpha = 1.; out.fade_started = true; out.feedforward_phase = "FADING";
  return out;
}
bool TwoPathProducer::advance() {
  if (transient_) {
    ++interval_index_;
    if (interval_index_ == plan_->states.size() - 1) {
      hold_position_ = plan_->states.back()[0]; hold_velocity_ = plan_->target_velocity; transient_ = false; return true;
    }
  } else hold_position_ += hold_velocity_ * config_.producer.dt;
  return false;
}
void TwoPathProducer::encode(TwoPathBlock &block) const {
  u32(block.bytes.data(), 1); u32(block.bytes.data() + 4, block.epoch);
  u32(block.bytes.data() + 8, block.sequence); u32(block.bytes.data() + 12, block.start_tick);
  f64(block.bytes.data() + 16, block.generated_time); f64(block.bytes.data() + 24, config_.producer.dt);
  u32(block.bytes.data() + 32, config_.producer.block_samples); f32(block.bytes.data() + 36, block.yaw);
  for (std::size_t i = 0; i < 25; ++i) for (std::size_t j = 0; j < 6; ++j)
    f32(block.bytes.data() + 40 + (i * 6 + j) * 4, block.samples[i][j]);
}
TwoPathBlock TwoPathProducer::produce(std::uint32_t start_tick, double generated_time,
    double source_time, double velocity, double yaw, bool observe) {
  const auto before = std::chrono::steady_clock::now(); const double cpu_before = thread_seconds();
  if (!std::isfinite(generated_time) || !std::isfinite(source_time) || !std::isfinite(velocity) || !std::isfinite(yaw) ||
      std::abs(velocity) > config_.producer.max_velocity + 1e-12 || sequence_ == std::numeric_limits<std::uint32_t>::max() ||
      start_tick > std::numeric_limits<std::uint32_t>::max() - 25) throw std::invalid_argument("invalid_two_path_block_input");
  if (!observe && !observed_) throw std::logic_error("two_path_prefill_without_observation");
  if (observe) { last_source_ = source_time; last_raw_ = velocity; last_yaw_ = yaw; observed_ = true; }
  const double start_time = start_tick * config_.producer.dt;
  Scheduled scheduled;
  if (observe) {
    const bool exit_observed = !config_.defer_full_accept_until_observed_exit || !last_full_exit_time_ || last_source_ + 1e-12 >= *last_full_exit_time_;
    scheduled = schedule(last_source_, last_raw_, !full_locked_ && exit_observed);
  } else scheduled = {false, candidate_start_ ? "CANDIDATE" : "HOLD", last_raw_, accepted_, candidate_delta(), candidate_stable_, candidate_target()};
  if (scheduled.accepted) {
    if (pending_from_full_exit_) { scheduler_events_.rows.back()["processed_after_full_exit"] = true; pending_from_full_exit_ = false; }
    begin_plan(start_time, scheduled, generated_time);
  }
  intents_.push({{"source_time_s", last_source_}, {"raw_linear_velocity_target_m_s", last_raw_},
    {"accepted_linear_velocity_target_m_s", accepted_}, {"candidate_target_m_s", nullable(candidate_target())},
    {"candidate_delta_v_m_s", candidate_delta()}, {"candidate_stable", candidate_stable_},
    {"latest_pending_raw_v_cmd_m_s", latest_pending_}, {"full_locked", full_locked_},
    {"observation_reused_for_prefill", !observe}, {"yaw_rate_target_rad_s", last_yaw_}});
  TwoPathBlock block;
  block.epoch = epoch_; block.sequence = sequence_; block.start_tick = start_tick; block.generated_time = generated_time;
  block.source_time = last_source_; block.target_velocity = accepted_; block.yaw = last_yaw_; block.raw_velocity = last_raw_;
  block.replanned = scheduled.accepted; block.duration = plan_ ? plan_->transient_duration : 0.;
  block.candidate_delta = scheduled.delta; block.candidate_target = scheduled.target ? scheduled.target : candidate_target();
  block.candidate_stable = scheduled.stable; block.block_id = block_count_;
  for (std::size_t i = 0; i < 25; ++i) {
    const double time = start_time + i * config_.producer.dt;
    auto command = lifecycle_command(time);
    if (full_locked_ && !full_quiet_latched_ && transient_ && plan_->shaped_finished.at(interval_index_) &&
        std::abs(command.acceleration) <= 1e-12 && std::abs(command.reference[2]) <= config_.quiet_theta_rad &&
        std::abs(command.reference[3]) <= config_.quiet_theta_dot_rad_s) {
      const auto previous = command.reference;
      command = enter_quiet_hold(time); full_quiet_latched_ = true;
      quiets_.push({{"time_s", time}, {"event", "quiet_snap"}, {"theta_before_rad", previous[2]}, {"theta_dot_before_rad_s", previous[3]}});
      if (active_full_event_id_) {
        const auto first = replans_.total - replans_.rows.size();
        if (*active_full_event_id_ >= first) replans_.rows.at(*active_full_event_id_ - first)["quiet_time_s"] = time;
      }
    }
    const char *mode = full_locked_ ? "FULL_DYNAMIC" : (candidate_start_ ? "CANDIDATE" :
      (active_mode_ == "LIGHTWEIGHT" && transient_ ? "LIGHTWEIGHT" : "NORMAL"));
    const auto &r = command.reference;
    block.samples[i] = {r[0], r[1], command.acceleration, r[2], r[3], command.feedforward};
    block.details[i] = {r, command.acceleration, raw_feedforward(), command.feedforward, command.fade_alpha,
      command.feedforward_phase, command.velocity_phase, mode, i == 0 && scheduled.accepted, i == 0 && scheduled.accepted};
    block.pending_velocity = latest_pending_;
    block.pending_active = full_locked_ && (candidate_start_ || std::abs(latest_pending_ - accepted_) >= config_.scheduler.accept_delta_velocity);
    if (command.fade_started) fades_.push({{"time_s", time}, {"event", "fade_started"}});
    if (command.fade_finished) {
      fades_.push({{"time_s", time}, {"event", "fade_finished"}});
      if (full_locked_ && full_quiet_latched_) {
        full_locked_ = false; active_mode_ = "HOLD"; pending_from_full_exit_ = candidate_start_.has_value();
        full_exits_.push({{"time_s", time}, {"event", "full_exit"}, {"accepted_velocity_m_s", accepted_},
          {"latest_pending_raw_v_cmd_m_s", latest_pending_}, {"pending_candidate_active", candidate_start_.has_value()},
          {"pending_candidate_stable", candidate_stable_}});
        last_full_exit_time_ = time;
        if (active_full_event_id_) {
          const auto first = replans_.total - replans_.rows.size();
          if (*active_full_event_id_ >= first) {
            auto &event = replans_.rows.at(*active_full_event_id_ - first);
            event["fade_completion_time_s"] = time; event["full_exit_time_s"] = time;
          }
        }
        active_full_event_id_.reset();
      }
    }
    const bool reached = advance();
    if (reached && active_mode_ == "LIGHTWEIGHT" && !transient_) active_mode_ = "HOLD";
  }
  encode(block); ++sequence_; ++block_count_;
  block.compute_cpu = thread_seconds() - cpu_before;
  block.compute_wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - before).count();
  return block;
}
std::string TwoPathProducer::summary_json() const {
  json report = {{"block_count", block_count_}, {"block_samples", config_.producer.block_samples},
    {"block_duration_s", config_.producer.dt * config_.producer.block_samples}, {"intent_count", intents_.total},
    {"intents", intents_.export_rows()}, {"scheduler_events", scheduler_events_.export_rows()},
    {"replan_count", replans_.total}, {"replan_events", replans_.export_rows()}, {"full_exit_events", full_exits_.export_rows()},
    {"fade_events", fades_.export_rows()}, {"quiet_events", quiets_.export_rows()}, {"latest_pending_velocity_m_s", latest_pending_},
    {"full_locked", full_locked_}, {"accepted_velocity_m_s", accepted_}, {"candidate_target_m_s", nullable(candidate_target())},
    {"candidate_delta_v_m_s", candidate_delta()}, {"candidate_stable", candidate_stable_},
    {"candidate_retained", candidate_window_.size()}, {"candidate_capacity", kCandidateCapacity}, {"plan_capacity", plan_capacity_}};
  report["logs"] = json::object();
  for (const auto &entry : {std::pair<const char *, const Log *>{"scheduler", &scheduler_events_}, {"intents", &intents_},
      {"replans", &replans_}, {"full_exits", &full_exits_}, {"fades", &fades_}, {"quiet", &quiets_}})
    report["logs"][entry.first] = {{"total", entry.second->total}, {"retained", entry.second->rows.size()}, {"overwritten", entry.second->overwritten}, {"capacity", kLogCapacity}};
  return report.dump();
}
}  // namespace companionbot::follow
