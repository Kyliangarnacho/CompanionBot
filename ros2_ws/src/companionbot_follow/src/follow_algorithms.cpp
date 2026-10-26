#include "companionbot_follow/follow_algorithms.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace companionbot::follow {
namespace {
using Matrix2 = std::array<double, 4>;
Matrix2 multiply(const Matrix2& a, const Matrix2& b) {
  Matrix2 result{};
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t col = 0; col < 2; ++col) {
      result[row * 2 + col] = a[row * 2] * b[col] + a[row * 2 + 1] * b[2 + col];
    }
  }
  return result;
}
Matrix2 transpose(const Matrix2& value) { return {value[0], value[2], value[1], value[3]}; }
bool all_finite(std::initializer_list<double> values) {
  return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}
void validate_observation(const FollowObservation& value) {
  if (!all_finite({value.capture_time_s, value.x_forward_m, value.y_left_m, value.confidence}) ||
      value.version < 1 || value.confidence < 0. || value.confidence > 1.) {
    throw std::invalid_argument("target observation values/version/confidence invalid");
  }
}
double median(std::array<double, 20> values, std::size_t count) {
  if (count == 0) throw std::logic_error("KF median requires a valid observation");
  std::sort(values.begin(), values.begin() + count);
  const auto mid = count / 2;
  return count % 2 ? values[mid] : (values[mid - 1] + values[mid]) / 2.;
}
}

RadialVelocityKalmanFilter::RadialVelocityKalmanFilter(RadialKFConfig config) : config_(config) {
  for (const auto value : {config.initial_distance_std, config.initial_velocity_std,
                          config.measurement_distance_std, config.master_accel_std}) {
    if (!std::isfinite(value) || value <= 0. || !std::isfinite(value * value)) {
      throw std::invalid_argument("KF standard deviations must be finite and positive");
    }
  }
}
RadialKFResult RadialVelocityKalmanFilter::update(double t, double z, double u) {
  if (!all_finite({t, z, u}) || z < 0.) {
    throw std::invalid_argument("KF input must be finite and distance nonnegative");
  }
  if (!x_) {
    x_ = std::array<double, 2>{z, 0.};
    covariance_ = Matrix2{config_.initial_distance_std * config_.initial_distance_std, 0., 0.,
                          config_.initial_velocity_std * config_.initial_velocity_std};
    last_capture_time_s_ = t;
    innovation_m_ = 0.;
    return {z, 0., 0.};
  }
  const double dt = t - *last_capture_time_s_;
  if (dt <= 0.) throw std::invalid_argument("KF capture timestamps must strictly increase");
  const Matrix2 A{1., dt, 0., 1.};
  const std::array<double, 2> G{.5 * dt * dt, dt};
  const std::array<double, 2> prior_x{
      (A[0] * (*x_)[0] + A[1] * (*x_)[1]) + (-dt * u),
      A[2] * (*x_)[0] + A[3] * (*x_)[1]};
  Matrix2 prior_P = multiply(multiply(A, *covariance_), transpose(A));
  const double sigma2 = config_.master_accel_std * config_.master_accel_std;
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t col = 0; col < 2; ++col) prior_P[row * 2 + col] += sigma2 * (G[row] * G[col]);
  }
  const double R = config_.measurement_distance_std * config_.measurement_distance_std;
  const double innovation = z - prior_x[0];
  const std::array<double, 2> gain{prior_P[0] / (prior_P[0] + R), prior_P[2] / (prior_P[0] + R)};
  const std::array<double, 2> next_x{prior_x[0] + gain[0] * innovation, prior_x[1] + gain[1] * innovation};
  const Matrix2 I_KH{1. - gain[0], 0., -gain[1], 1.};
  Matrix2 next_P = multiply(multiply(I_KH, prior_P), transpose(I_KH));
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t col = 0; col < 2; ++col) next_P[row * 2 + col] += R * (gain[row] * gain[col]);
  }
  const auto unsymmetric = next_P;
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t col = 0; col < 2; ++col) {
      next_P[row * 2 + col] = .5 * (unsymmetric[row * 2 + col] + unsymmetric[col * 2 + row]);
    }
  }
  if (!all_finite({next_x[0], next_x[1], next_P[0], next_P[1], next_P[2], next_P[3], innovation})) {
    throw std::invalid_argument("KF derived state nonfinite");
  }
  x_ = next_x; covariance_ = next_P; last_capture_time_s_ = t; innovation_m_ = innovation;
  return {next_x[0], next_x[1], innovation};
}

PiRobotStateHistory::PiRobotStateHistory(double retention_s) : retention_s_(retention_s) {
  if (!std::isfinite(retention_s) || retention_s <= 0.) throw std::invalid_argument("state retention must be positive");
}
bool PiRobotStateHistory::accept(const FollowRobotState& state) {
  if (!all_finite({state.sample_time_s, state.v_hat_m_s})) throw std::invalid_argument("state packet values must be finite");
  if (!samples_.empty() && (state.sequence_id <= samples_.back().sequence_id ||
                           state.sample_time_s <= samples_.back().sample_time_s)) {
    ++out_of_order_rejections_; return false;
  }
  const double cutoff = state.sample_time_s - retention_s_;
  // Preserve the left interpolation bracket, exactly as the Python retention
  // rule does. Capacity never silently removes an in-window measurement.
  while (samples_.size() > 1 && samples_[1].sample_time_s < cutoff) samples_.pop_front();
  if (samples_.size() == kCapacity) { ++capacity_rejections_; return false; }
  samples_.push_back(state); ++accepted_count_;
  return true;
}
std::optional<StateAlignment> PiRobotStateHistory::velocity_at(double t) const {
  if (!std::isfinite(t)) throw std::invalid_argument("alignment time must be finite");
  if (samples_.empty() || t < samples_.front().sample_time_s - 1e-9) return std::nullopt;
  for (std::size_t index = 0; index + 1 < samples_.size(); ++index) {
    const auto& left = samples_[index]; const auto& right = samples_[index + 1];
    if (left.sample_time_s - 1e-9 <= t && t <= right.sample_time_s + 1e-9) {
      const double alpha = std::max(0., std::min(1., (t - left.sample_time_s) / (right.sample_time_s - left.sample_time_s)));
      return StateAlignment{(1. - alpha) * left.v_hat_m_s + alpha * right.v_hat_m_s,
          t, left.sequence_id, right.sequence_id, left.sample_time_s, right.sample_time_s, alpha};
    }
  }
  const auto& last = samples_.back();
  if (std::abs(t - last.sample_time_s) <= 1e-9) {
    return StateAlignment{last.v_hat_m_s, t, last.sequence_id, last.sequence_id, t, t, 0.};
  }
  return std::nullopt;
}
StateHistorySnapshot PiRobotStateHistory::snapshot() const {
  return {samples_.size(), accepted_count_, out_of_order_rejections_, capacity_rejections_};
}
std::optional<FollowRobotState> PiRobotStateHistory::latest() const {
  return samples_.empty() ? std::optional<FollowRobotState>{} : samples_.back();
}

RadialObservationPipeline::RadialObservationPipeline(RadialKFConfig config, double retention_s)
    : config_(config), kf_(config), history_(retention_s) {}
void RadialObservationPipeline::reset_observations() {
  kf_ = RadialVelocityKalmanFilter(config_);
  latest_observation_.reset(); latest_estimate_.reset();
  last_raw_distance_.reset(); last_raw_time_.reset();
}
RadialObservationResult RadialObservationPipeline::accept_observation(const FollowObservation& observation,
                                                                     double arrival_time_s) {
  validate_observation(observation);
  if (!std::isfinite(arrival_time_s)) throw std::invalid_argument("observation arrival time must be finite");
  RadialObservationResult result;
  result.observation = observation; result.arrival_time_s = arrival_time_s;
  result.measured_distance_m = std::hypot(observation.x_forward_m, observation.y_left_m);
  result.beta_rad = std::atan2(observation.y_left_m, observation.x_forward_m);
  if (!observation.valid) { result.alignment = "INVALID_OBSERVATION"; return result; }
  if ((latest_estimate_ && observation.sequence_id <= latest_estimate_->sequence_id) ||
      (kf_.last_capture_time_s() && observation.capture_time_s <= *kf_.last_capture_time_s())) {
    result.alignment = "OUT_OF_ORDER"; return result;
  }
  result.state_alignment = history_.velocity_at(observation.capture_time_s);
  if (!result.state_alignment) { result.alignment = "STATE_HISTORY_MISS"; return result; }
  const auto& aligned = *result.state_alignment;
  const double u = aligned.velocity_m_s * std::cos(result.beta_rad);
  if (last_raw_time_) result.raw_radial_velocity_diagnostic_m_s =
      (result.measured_distance_m - *last_raw_distance_) / (observation.capture_time_s - *last_raw_time_) + u;
  const auto filtered = kf_.update(observation.capture_time_s, result.measured_distance_m, u);
  result.estimate = RadialEstimate{observation.sequence_id, observation.capture_time_s,
      filtered.distance_hat_m, filtered.master_radial_velocity_hat_m_s,
      aligned.velocity_m_s, result.beta_rad, filtered.innovation_m};
  result.latest_robot_velocity_diagnostic_m_s = history_.latest()->v_hat_m_s;
  latest_estimate_ = result.estimate; latest_observation_ = observation;
  last_raw_distance_ = result.measured_distance_m; last_raw_time_ = observation.capture_time_s;
  result.alignment = aligned.left_sequence == aligned.right_sequence ? "EXACT" : "INTERPOLATED";
  result.accepted = true;
  return result;
}

FollowGovernor::FollowGovernor(FollowGovernorConfig config, bool use_radial_estimates)
    : config_(config), use_radial_estimates_(use_radial_estimates) {
  if (!all_finite({config.desired_distance_m, config.d1_m, config.d2_m, config.catch_time_s,
      config.max_abs_velocity_m_s, config.initial_velocity_m_s, config.a_brake_effective_m_s2,
      config.catch_retrigger_margin_m_s, config.slowdown_velocity_deadband_m_s}) ||
      config.d2_m >= config.d1_m || config.desired_distance_m < 0. || config.d2_m < config.desired_distance_m ||
      config.catch_time_s <= 0. || config.max_abs_velocity_m_s <= 0. || config.a_brake_effective_m_s2 <= 0. ||
      config.catch_retrigger_margin_m_s <= 0. || config.slowdown_velocity_deadband_m_s <= 0. ||
      config.catch_retrigger_persistence_samples < 1 || std::abs(config.initial_velocity_m_s) > config.max_abs_velocity_m_s) {
    throw std::invalid_argument("follow governor configuration invalid");
  }
  state_.latched_velocity_m_s = config.initial_velocity_m_s;
  state_.switch_distance_m = config.d2_m; state_.distance_m = config.desired_distance_m;
}
void FollowGovernor::set_robot_velocity_hat(double velocity_m_s) {
  if (!std::isfinite(velocity_m_s)) throw std::invalid_argument("robot velocity estimate must be finite");
  state_.robot_velocity_hat_m_s = velocity_m_s;
}
MotionIntent FollowGovernor::intent(const FollowObservation& observation) const {
  return {observation.capture_time_s, state_.latched_velocity_m_s, 0.};
}
std::pair<double, double> FollowGovernor::medians() const {
  std::array<double, 20> recent{}, short_recent{};
  std::copy(recent_master_velocity_.begin(), recent_master_velocity_.end(), recent.begin());
  const auto count = recent_master_velocity_.size(), short_count = std::min<std::size_t>(count, 5);
  std::copy(recent.begin() + count - short_count, recent.begin() + count, short_recent.begin());
  return {median(short_recent, short_count), median(recent, count)};
}
GovernorObservationRow FollowGovernor::observation_row(const FollowObservation& observation,
    std::optional<double> derivative, bool accepted) const {
  GovernorObservationRow row;
  row.observation = observation; row.values = state_; row.d2_m = config_.d2_m;
  row.derivative_m_s = derivative; row.command_changed = accepted;
  row.catch_command_saturated = state_.state == "CATCH_UP" &&
      state_.latched_velocity_m_s >= config_.max_abs_velocity_m_s - 1e-12;
  row.unable_to_close = row.catch_command_saturated && state_.master_velocity_hat_m_s >
      state_.latched_velocity_m_s + config_.catch_retrigger_margin_m_s;
  return row;
}
GovernorEvent FollowGovernor::transition(const FollowObservation& observation, const std::string& new_state,
                                         double generated, const std::string& reason) {
  const auto previous_state = state_.state;
  double bounded = std::max(0., std::min(config_.max_abs_velocity_m_s, generated));
  if (new_state == "CRUISE" && bounded <= config_.slowdown_velocity_deadband_m_s) bounded = 0.;
  state_.state = new_state;
  if (new_state != "CATCH_UP") state_.increasing_distance_samples = 0;
  state_.latched_velocity_m_s = bounded;
  GovernorEvent event;
  event.time_s = observation.capture_time_s; event.sequence_id = observation.sequence_id;
  event.event = new_state == "CATCH_UP" ?
      (previous_state == "CATCH_UP" ? "catch_up_retriggered" : "catch_up_started") :
      (previous_state == "CRUISE" ? "slowdown_wait" : "cruise_resumed");
  event.reason = reason; event.previous_state = previous_state; event.values = state_;
  event.generated_velocity_m_s = generated; event.command_saturated = bounded != generated;
  if (previous_state == "CATCH_UP" && new_state == "CRUISE" && use_radial_estimates_) {
    const auto [short_median, long_median] = medians();
    double shadow = std::max(0., std::min(config_.max_abs_velocity_m_s, state_.master_velocity_hat_m_s));
    if (shadow <= config_.slowdown_velocity_deadband_m_s) shadow = 0.;
    event.single_sample_kf_master_radial_velocity_m_s = state_.master_velocity_hat_m_s;
    event.single_sample_shadow_latched_velocity_m_s = shadow;
    event.cruise_velocity_median_m_s = short_median;
    event.cruise_velocity_window_samples = std::min<std::size_t>(recent_master_velocity_.size(), 5);
    event.cruise_velocity_long_median_m_s = long_median;
    event.cruise_velocity_long_window_samples = recent_master_velocity_.size();
  }
  if (events_.size() == kEventCapacity) { events_.pop_front(); ++events_overwritten_; }
  events_.push_back(event);
  return event;
}

GovernorResult FollowGovernor::update(const FollowObservation& observation,
                                      const std::optional<RadialEstimate>& estimate) {
  validate_observation(observation);
  if (state_.last_sequence_id && observation.sequence_id < *state_.last_sequence_id) {
    throw std::invalid_argument("target observation sequence_id must increase");
  }
  GovernorResult result;
  if (state_.last_sequence_id && observation.sequence_id == *state_.last_sequence_id) {
    ++state_.duplicate_observation_count;
    result.intent = intent(last_observation_.value_or(observation)); result.duplicate = true;
    return result;
  }
  // Validate the estimate association before changing algorithm state. This
  // replaces the old Python callback assumption with an explicit data contract.
  if (observation.valid && use_radial_estimates_ && (!estimate ||
      estimate->sequence_id != observation.sequence_id || estimate->capture_time_s != observation.capture_time_s ||
      !all_finite({estimate->distance_hat_m, estimate->master_radial_velocity_hat_m_s,
                   estimate->aligned_robot_velocity_m_s, estimate->beta_rad, estimate->innovation_m}))) {
    throw std::invalid_argument("matching finite radial estimate required");
  }
  state_.last_sequence_id = observation.sequence_id;
  if (!observation.valid) {
    state_.increasing_distance_samples = 0; state_.slowdown_samples = 0;
    result.row = observation_row(observation, std::nullopt, false);
    result.intent = intent(observation); return result;
  }
  double distance = std::hypot(observation.x_forward_m, observation.y_left_m);
  const double beta = std::atan2(observation.y_left_m, observation.x_forward_m);
  std::optional<double> derivative;
  if (last_observation_) {
    const double dt = observation.capture_time_s - last_observation_->capture_time_s;
    if (dt <= 0.) throw std::invalid_argument("target observation timestamps must increase");
    derivative = (distance - std::hypot(last_observation_->x_forward_m, last_observation_->y_left_m)) / dt;
  }
  double radial_robot{};
  std::optional<double> decision_derivative;
  if (use_radial_estimates_) {
    distance = estimate->distance_hat_m;
    state_.robot_velocity_hat_m_s = estimate->aligned_robot_velocity_m_s;
    state_.master_velocity_hat_m_s = estimate->master_radial_velocity_hat_m_s;
    if (recent_master_velocity_.size() == 20) recent_master_velocity_.pop_front();
    recent_master_velocity_.push_back(state_.master_velocity_hat_m_s);
    radial_robot = state_.robot_velocity_hat_m_s * std::cos(beta);
    decision_derivative = state_.master_velocity_hat_m_s - radial_robot;
  } else {
    radial_robot = state_.robot_velocity_hat_m_s * std::cos(beta);
    decision_derivative = derivative;
    state_.master_velocity_hat_m_s = derivative ? radial_robot + *derivative : radial_robot;
  }
  state_.distance_m = distance; state_.beta_rad = beta;
  state_.distance_rate_hat_m_s = decision_derivative.value_or(0.);
  state_.robot_radial_velocity_hat_m_s = radial_robot;
  last_observation_ = observation;
  state_.closing_speed_m_s = std::max(-state_.distance_rate_hat_m_s, 0.);
  state_.transition_distance_m = (state_.closing_speed_m_s * state_.closing_speed_m_s) /
      (2. * config_.a_brake_effective_m_s2);
  state_.switch_distance_m = config_.d2_m + state_.transition_distance_m;
  const bool insufficient_catch = state_.master_velocity_hat_m_s >
      state_.latched_velocity_m_s + config_.catch_retrigger_margin_m_s;
  state_.increasing_distance_samples = state_.state == "CATCH_UP" && distance > config_.d1_m &&
      decision_derivative && *decision_derivative > 0. && insufficient_catch ? state_.increasing_distance_samples + 1 : 0;
  const bool unable = state_.state == "CATCH_UP" && insufficient_catch &&
      state_.latched_velocity_m_s >= config_.max_abs_velocity_m_s - 1e-12;
  state_.unable_to_close_sample_count += unable ? 1 : 0;
  const bool slowing = state_.state == "CRUISE" &&
      state_.latched_velocity_m_s - state_.master_velocity_hat_m_s > config_.slowdown_velocity_deadband_m_s &&
      state_.closing_speed_m_s > 0. && distance <= state_.switch_distance_m;
  state_.slowdown_samples = slowing ? state_.slowdown_samples + 1 : 0;
  bool accepted = false;
  if (state_.state == "CRUISE" && distance > config_.d1_m) {
    const double velocity = state_.master_velocity_hat_m_s + (distance - config_.d2_m) / config_.catch_time_s;
    result.event = transition(observation, "CATCH_UP", velocity, "distance_above_d1"); accepted = true;
  } else if (state_.state == "CATCH_UP" && distance <= state_.switch_distance_m) {
    double cruise = state_.master_velocity_hat_m_s;
    if (use_radial_estimates_) {
      const auto [short_median, long_median] = medians();
      cruise = long_median - short_median > config_.slowdown_velocity_deadband_m_s ? short_median : long_median;
    }
    result.event = transition(observation, "CRUISE", cruise, "distance_at_or_below_dynamic_switch"); accepted = true;
  } else if (state_.state == "CRUISE" && state_.slowdown_samples >= config_.catch_retrigger_persistence_samples) {
    double slowdown = state_.master_velocity_hat_m_s;
    bool rejected = false;
    if (use_radial_estimates_) {
      const auto [short_median, long_median] = medians();
      if (short_median <= config_.slowdown_velocity_deadband_m_s) slowdown = short_median;
      else if (state_.latched_velocity_m_s - long_median <= config_.slowdown_velocity_deadband_m_s) rejected = true;
      else slowdown = long_median;
    }
    if (!rejected) {
      result.event = transition(observation, "CRUISE", slowdown, "master_slowed_at_dynamic_switch"); accepted = true;
    } else {
      result.row = observation_row(observation, derivative, false);
      result.row->slowdown_rejection_reason = "short_kf_dip_without_long_window_slowdown";
      state_.slowdown_samples = 0; result.intent = intent(observation); return result;
    }
    state_.slowdown_samples = 0;
  } else if (state_.state == "CATCH_UP" && state_.increasing_distance_samples >= config_.catch_retrigger_persistence_samples) {
    const double velocity = state_.master_velocity_hat_m_s + (distance - config_.d2_m) / config_.catch_time_s;
    const double bounded = std::min(velocity, config_.max_abs_velocity_m_s);
    if (bounded > state_.latched_velocity_m_s + 1e-12) {
      result.event = transition(observation, "CATCH_UP", velocity,
          "distance_increasing_and_catch_command_insufficient"); accepted = true;
    }
    state_.increasing_distance_samples = 0;
  }
  result.row = observation_row(observation, derivative, accepted);
  result.intent = intent(observation); return result;
}

BearingYawServo::BearingYawServo(double gain, double max_rate, double deadband)
    : gain_(gain), max_rate_(max_rate), deadband_(deadband) {
  if (!all_finite({gain, max_rate, deadband}) || gain <= 0. || max_rate <= 0. || deadband < 0.) {
    throw std::invalid_argument("invalid bearing servo parameters");
  }
}
std::pair<double, double> BearingYawServo::command(double x, double y) const {
  if (!all_finite({x, y})) throw std::invalid_argument("bearing observation must be finite");
  const double beta = std::atan2(y, x);
  const double target = std::abs(beta) <= deadband_ ? 0. : gain_ * beta;
  return {beta, std::max(-max_rate_, std::min(target, max_rate_))};
}

FollowController::FollowController(std::shared_ptr<MotionAuthority> authority,
    RadialKFConfig kf_config, FollowGovernorConfig governor_config, double retention,
    double yaw_gain, double yaw_max_rate, double yaw_deadband)
    : authority_(std::move(authority)), governor_config_(governor_config),
      pipeline_(kf_config, retention), governor_(governor_config, true),
      yaw_(yaw_gain, yaw_max_rate, yaw_deadband) {
  if (!authority_ || !authority_->follow_config()) throw std::invalid_argument("follow authority must be explicitly configured");
  const auto limits = *authority_->follow_config();
  if (limits.max_velocity_m_s != governor_config.max_abs_velocity_m_s ||
      limits.max_yaw_rate_rad_s != yaw_max_rate)
    throw std::invalid_argument("canonical follow algorithm/authority limits mismatch");
}

void FollowController::reset_algorithms() {
  pipeline_.reset_observations();
  governor_ = FollowGovernor(governor_config_, true);
  ++counters_.algorithm_resets;
}

bool FollowController::accept_state(const FollowRobotState& state) {
  std::lock_guard<std::mutex> guard(mutex_);
  return pipeline_.accept_state(state);
}

FollowControllerResult FollowController::accept_measurement(const std::string& epoch,
    std::uint64_t sequence, double capture, double source_now, double receipt_now,
    double x, double y, bool valid, const std::string& reason) {
  std::lock_guard<std::mutex> guard(mutex_);
  ++counters_.measurement_calls;
  FollowControllerResult result;
  auto owner = authority_->snapshot(false);
  result.authority_epoch = owner.authority_epoch;
  if (owner.mode == "FOLLOW" && owner.granted_source_time_s && capture <= *owner.granted_source_time_s) {
    ++counters_.out_of_order_measurements;
    result.reason = "follow_pregrant_observation";
    return result;
  }
  if (!authority_->follow_observation_identity_current(epoch, sequence, capture)) {
    ++counters_.out_of_order_measurements;
    result.reason = "follow_observation_identity_rejected";
    return result;
  }
  if (!counters_.authority_epoch || *counters_.authority_epoch != owner.authority_epoch) {
    reset_algorithms();
    counters_.authority_epoch = owner.authority_epoch;
  }
  const auto invalid = [&](const std::string& why) {
    ++counters_.invalid_measurements;
    authority_->update_follow_health(false, why, epoch, sequence, capture, source_now, receipt_now);
    result.reason = why;
    result.authority_epoch = authority_->snapshot(false).authority_epoch;
    return result;
  };
  if (!valid) return invalid(reason.empty() ? "follow_observation_invalid" : reason);
  if (!all_finite({capture, source_now, receipt_now, x, y}) || epoch.empty() || epoch.size() > 128)
    return invalid("follow_measurement_identity_or_values_invalid");
  const bool changed_camera = counters_.camera_epoch && *counters_.camera_epoch != epoch;
  const FollowObservation observation{capture, x, y, 1, sequence, true, 1.};
  std::optional<RadialObservationPipeline> candidate;
  RadialObservationPipeline* pipeline = &pipeline_;
  if (changed_camera) {
    // A rejected retired epoch must not erase the current task's algorithm
    // history. Copy only on an epoch transition, never on normal frame updates.
    candidate = pipeline_;
    candidate->reset_observations();
    pipeline = &*candidate;
  }
  try {
    result.radial = pipeline->accept_observation(observation, source_now);
  } catch (const std::invalid_argument&) {
    return invalid("follow_algorithm_measurement_invalid");
  }
  if (!result.radial->accepted) {
    if (result.radial->alignment == "OUT_OF_ORDER") {
      ++counters_.out_of_order_measurements;
      result.reason = "follow_observation_out_of_order";
      return result;
    }
    return invalid("follow_state_history_unavailable");
  }
  if (!authority_->update_follow_health(true, "", epoch, sequence, capture, source_now, receipt_now)) {
    result.reason = "follow_health_rejected";
    return result;
  }
  if (candidate) pipeline_ = std::move(*candidate);
  if (changed_camera) {
    governor_ = FollowGovernor(governor_config_, true);
    ++counters_.algorithm_resets;
  }
  counters_.camera_epoch = epoch;
  result.accepted = true;
  ++counters_.measurements_accepted;
  owner = authority_->snapshot(false);
  result.authority_epoch = owner.authority_epoch;
  if (owner.mode != "FOLLOW" || !owner.task_id || !owner.source_epoch ||
      !owner.granted_source_time_s || capture <= *owner.granted_source_time_s) {
    result.reason = "follow_readiness_only";
    return result;
  }
  // If health changed the authority generation (camera restart), the old task
  // has already been revoked. A service grant racing this point is a new task
  // and requires another postgrant source image before any algorithm output.
  if (counters_.authority_epoch != owner.authority_epoch) {
    reset_algorithms(); counters_.authority_epoch = owner.authority_epoch;
    result.reason = "follow_new_task_requires_new_observation";
    return result;
  }
  try {
    result.governor = governor_.update(observation, result.radial->estimate);
    const auto [bearing, yaw_rate] = yaw_.command(x, y);
    result.bearing_rad = bearing; result.yaw_rate_rad_s = yaw_rate;
    result.command_accepted = authority_->receive_follow_cmd(*owner.task_id, *owner.source_epoch,
        owner.authority_epoch, epoch, sequence, capture, source_now, receipt_now,
        result.governor->intent.linear_velocity_target_m_s, yaw_rate);
  } catch (const std::invalid_argument&) {
    return invalid("follow_algorithm_intent_invalid");
  }
  if (result.command_accepted) ++counters_.commands_accepted;
  result.reason = result.command_accepted ? "follow_command_accepted" : "follow_command_rejected";
  return result;
}

FollowControllerSnapshot FollowController::snapshot() const {
  std::lock_guard<std::mutex> guard(mutex_);
  auto result = counters_;
  result.governor = governor_.snapshot();
  result.state_history = pipeline_.state_history().snapshot();
  return result;
}
}  // namespace companionbot::follow
