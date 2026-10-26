#include "companionbot_simulation/navigation_runtime.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <time.h>

namespace companionbot::simulation {
using json = nlohmann::json;
namespace {
constexpr std::uint32_t kCapacityFaultReason = 512;
constexpr std::uint32_t kReferenceFaultReason = 513;
double cpu_seconds() {
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value))
    throw std::runtime_error("runtime_thread_cpu_clock_unavailable");
  return static_cast<double>(value.tv_sec) + static_cast<double>(value.tv_nsec) * 1e-9;
}
bool moving(const follow::MotionIntent &intent) {
  return std::abs(intent.linear_velocity_target_m_s) > 1e-9 ||
         std::abs(intent.yaw_rate_target_rad_s) > 1e-9;
}
follow::TwoPathConfig navigation_policy(follow::TwoPathConfig config) {
  config.scheduler.require_stable_candidate = false;
  config.scheduler.minimum_delta_enabled = false;
  config.scheduler.force_full_dynamic = false;
  config.defer_full_accept_until_observed_exit = false;
  return config;
}
}  // namespace

void NavigationRuntime::Log::push(json row) {
  const auto index = static_cast<std::size_t>(total % capacity);
  if (rows.size() < capacity) rows.push_back(std::move(row));
  else { rows[index] = std::move(row); ++overwritten; }
  ++total;
}
json NavigationRuntime::Log::export_rows() const {
  json result = json::array();
  const std::size_t start = total >= capacity ? static_cast<std::size_t>(total % capacity) : 0;
  for (std::size_t i = 0; i < rows.size(); ++i)
    result.push_back(rows[(start + i) % rows.size()]);
  return result;
}

NavigationRuntime::NavigationRuntime(std::shared_ptr<follow::MotionAuthority> authority,
    follow::ProducerConfig producer, cb_mcu_config mcu, double latency)
    : authority_(std::move(authority)), producer_(producer), latency_(latency) {
  if (!authority_ || !std::isfinite(latency) || latency < 0. ||
      authority_->max_velocity_m_s >= producer.full_delta_velocity ||
      authority_->max_velocity_m_s > producer.max_velocity ||
      cb_mcu_init(&mcu_, &mcu, 0) != CB_OK)
    throw std::invalid_argument("invalid_native_navigation_configuration");
}

NavigationRuntime::NavigationRuntime(std::shared_ptr<follow::MotionAuthority> authority,
    follow::TwoPathConfig producer, cb_mcu_config mcu, double latency,
    follow::FullPlanSolver solver, std::shared_ptr<follow::FollowController> controller,
    std::optional<FollowClearanceConfig> clearance)
    : authority_(std::move(authority)), producer_(navigation_policy(producer), std::move(solver)),
      follow_config_(producer), follow_controller_(std::move(controller)), clearance_config_(clearance), latency_(latency) {
  if (!authority_ || !std::isfinite(latency) || latency < 0. ||
      authority_->max_velocity_m_s >= producer.scheduler.full_delta_velocity ||
      authority_->max_velocity_m_s > producer.producer.max_velocity || cb_mcu_init(&mcu_, &mcu, 0) != CB_OK ||
      (clearance && (!std::isfinite(clearance->radius_m) || clearance->radius_m <= 0. ||
        !std::isfinite(clearance->reaction_time_s) || clearance->reaction_time_s < 0. ||
        clearance->maximum_projection_steps == 0 || clearance->maximum_projection_steps > 1000000)))
    throw std::invalid_argument("invalid_native_companion_configuration");
}

void NavigationRuntime::update_follow_braking(double source_now) {
  if (!clearance_config_) return;
  const auto &cfg = *clearance_config_;
  const cb_reference initial = mcu_.has_last ? mcu_.output.reference : cb_reference{};
  struct Prediction { bool valid{}; double distance{}, tail{}; std::uint32_t steps{}; };
  const auto project = [&](double velocity, double acceleration) {
    Prediction result;
    cb_reference start = initial;
    start.p = 0.; start.v = velocity; start.a = acceleration;
    cb_brake brake{};
    if (cb_brake_init(&brake, &mcu_.config.brake, &start) != CB_OK) return result;
    double previous_velocity = start.v;
    for (std::uint32_t i = 0; i < cfg.maximum_projection_steps; ++i) {
      cb_reference sample{};
      if (cb_brake_step(&brake, &sample) != CB_OK || !std::isfinite(sample.p) ||
          !std::isfinite(sample.v) || !std::isfinite(sample.a)) return result;
      result.steps = i + 1;
      // A minimum all-direction range needs a swept-path bound, including a
      // possible velocity sign change. Absolute trapezoids bound the path of
      // the C99 piecewise-velocity reference more conservatively than net p.
      if (i > 0) result.distance += .5 * (std::abs(previous_velocity) + std::abs(sample.v)) * mcu_.config.brake.dt;
      previous_velocity = sample.v;
      // Once the original proportional target is unsaturated and reachable in
      // one jerk step, each future v is (1-2*dt)*v. The remaining trapezoidal
      // displacement is at most |v|/2; do not discard the settled tail.
      if (brake.settled && cb_brake_handoff_done(&brake) && mcu_.config.brake.dt < .5 &&
          std::abs(-2. * sample.v) <= mcu_.config.brake.deceleration &&
          std::abs(-2. * sample.v - sample.a) <= mcu_.config.brake.jerk * mcu_.config.brake.dt) {
        result.tail = std::abs(sample.v) / 2.;
        result.distance += result.tail; result.valid = true; return result;
      }
    }
    return result;
  };
  const auto reference = project(initial.v, initial.a), measured = project(local_velocity_, initial.a);
  const double speed = std::max(std::abs(initial.v), std::abs(local_velocity_));
  // Include the existing wire's one float32 rounding of canonical limits.
  // Beyond this declared profile envelope, no motion-ready claim is made.
  const double cap = std::max(producer_.config().max_velocity,
      static_cast<double>(static_cast<float>(producer_.config().max_velocity)));
  const double acceleration_cap = std::max(producer_.config().max_acceleration,
      static_cast<double>(static_cast<float>(producer_.config().max_acceleration)));
  const bool in_profile = speed <= cap && std::abs(initial.a) <= acceleration_cap;
  const double reaction = std::min(cap * cfg.reaction_time_s,
      speed * cfg.reaction_time_s + .5 * acceleration_cap * cfg.reaction_time_s * cfg.reaction_time_s);
  // A time-optimal Ruckig trajectory ending at zero target acceleration is not
  // automatically a proof of every allowed command sequence's reachable set.
  // Use the more conservative analytic v/a envelope instead. In particular,
  // continuing to accelerate during reaction also increases subsequent braking
  // distance; accounting only for the reaction displacement is insufficient.
  const double reaction_speed = std::min(cap, speed + acceleration_cap * cfg.reaction_time_s);
  json reaction_projections = json::array();
  double reaction_stop = 0., reaction_tail = 0.;
  bool reaction_valid = true;
  for (double v : {-reaction_speed, reaction_speed}) for (double a : {-acceleration_cap, acceleration_cap}) {
    const auto prediction = project(v, a);
    reaction_projections.push_back({{"initial_velocity_m_s", v}, {"initial_acceleration_m_s2", a},
        {"absolute_travel_m", prediction.distance}, {"tail_margin_m", prediction.tail},
        {"steps", prediction.steps}, {"valid", prediction.valid}});
    reaction_stop = std::max(reaction_stop, prediction.distance);
    reaction_tail = std::max(reaction_tail, prediction.tail);
    reaction_valid = reaction_valid && prediction.valid;
  }
  const double required = cfg.radius_m + reaction + std::max({reference.distance, measured.distance, reaction_stop});
  const bool valid = in_profile && reference.valid && measured.valid && reaction_valid && std::isfinite(required);
  const bool accepted = authority_->update_follow_braking(
      valid ? required : std::numeric_limits<double>::quiet_NaN(), current_receipt_);
  braking_events_.push({{"source_time_s", source_now}, {"receipt_time_s", current_receipt_},
      {"reference_velocity_m_s", initial.v}, {"reference_acceleration_m_s2", initial.a},
      {"local_velocity_m_s", local_velocity_}, {"reference_stop_distance_m", reference.distance},
      {"measured_stop_distance_m", measured.distance}, {"reference_tail_margin_m", reference.tail},
      {"measured_tail_margin_m", measured.tail}, {"reference_projection_steps", reference.steps},
      {"measured_projection_steps", measured.steps}, {"reaction_distance_m", reaction},
      {"reaction_velocity_bound_m_s", reaction_speed}, {"reaction_acceleration_bound_m_s2", acceleration_cap},
      {"reaction_stop_distance_m", reaction_stop}, {"reaction_tail_margin_m", reaction_tail},
      {"reaction_projections", std::move(reaction_projections)},
      {"profile_velocity_limit_m_s", cap}, {"profile_acceleration_limit_m_s2", acceleration_cap},
      {"within_profile_envelope", in_profile},
      {"reaction_braking_method", "analytic_velocity_acceleration_envelope_four_c99_brake_corners"},
      {"distance_semantics", "absolute_reference_travel_plus_unsaturated_tail_bound"},
      {"required_clearance_m", valid ? json(required) : json(nullptr)}, {"valid", valid}, {"accepted", accepted}});
}

void NavigationRuntime::select_producer_policy(const std::string &mode) {
  if (mode == "NONE" || mode == producer_mode_) return;
  if (mcu_.state != CB_SAFE_HOLD && (mcu_.state != CB_NORMAL ||
      std::abs(local_velocity_) > .03 || std::abs(local_pitch_ - mcu_.config.nominal_pitch) > .08 ||
      std::abs(local_pitch_rate_) > .20))
    throw std::logic_error("motion_policy_change_requires_mcu_stopped_state");
  if (mode == "FOLLOW") {
    if (!follow_config_ || !follow_controller_)
      throw std::logic_error("follow_runtime_not_configured");
    producer_.set_policy(follow_config_->scheduler, follow_config_->defer_full_accept_until_observed_exit);
  } else if (mode == "GUIDE_TO") {
    follow::SchedulerConfig policy;
    policy.full_delta_velocity = producer_.config().full_delta_velocity;
    policy.require_stable_candidate = false; policy.minimum_delta_enabled = false;
    producer_.set_policy(policy, false);
  } else throw std::logic_error("unknown_native_motion_mode");
  producer_mode_ = mode;
}

const char *NavigationRuntime::channel_name(Kind kind) {
  switch (kind) {
    case Kind::Reference: return "reference";
    case Kind::Heartbeat: return "heartbeat";
    case Kind::Yaw: return "yaw";
    case Kind::Safety: return "safety";
    case Kind::Resume: return "resume";
    case Kind::RobotState: return "robot_state";
    case Kind::SafetyStatus: return "safety_status";
  }
  throw std::logic_error("unknown_native_navigation_channel");
}

std::string NavigationRuntime::state_name(std::uint32_t code) {
  switch (code) {
    case CB_NORMAL: return "NORMAL";
    case CB_BRAKING: return "BRAKING";
    case CB_LONGITUDINAL_STOP: return "LONGITUDINAL_STOP";
    case CB_SAFE_HOLD: return "SAFE_HOLD";
  }
  throw std::out_of_range("invalid_mcu_state");
}

std::string NavigationRuntime::reason_name_unlocked(std::uint32_t code) const {
  switch (code) {
    case CB_REASON_NONE: return "NONE";
    case CB_REASON_PI: return "PI_SAFETY_COMMAND";
    case CB_REASON_HEARTBEAT: return "HEARTBEAT_TIMEOUT";
    case CB_REASON_STARVATION: return "REFERENCE_STARVATION_IMMINENT";
    case CB_REASON_UNDERRUN: return "REFERENCE_UNDERRUN";
    case kCapacityFaultReason: return "NATIVE_RUNTIME_CAPACITY_FAULT";
    case kReferenceFaultReason: return "REFERENCE_PRODUCTION_FAILED";
  }
  if (code >= 1024 && code - 1024 < reason_count_) return external_reasons_[code - 1024];
  throw std::out_of_range("unknown_native_mcu_reason");
}
std::string NavigationRuntime::reason_name(std::uint32_t code) const {
  std::lock_guard<std::mutex> guard(mutex_);
  return reason_name_unlocked(code);
}
std::uint32_t NavigationRuntime::external_reason(const std::string &reason) {
  if (reason.empty() || reason.size() > 256)
    throw std::invalid_argument("invalid_native_stop_reason");
  for (std::size_t i = 0; i < reason_count_; ++i)
    if (external_reasons_[i] == reason) return static_cast<std::uint32_t>(1024 + i);
  if (reason_count_ == kReasonCapacity) {
    // A bookkeeping limit must never prevent a stop. This dedicated reason is
    // outside the owner-recoverable external range and needs a full restart.
    if (!reason_capacity_faulted_) {
      reason_capacity_faulted_ = true;
      authority_->stop("native_runtime_reason_capacity_exhausted", current_receipt_, true);
      packet_events_.push({{"event", "stop_reason_capacity_exhausted"},
          {"time_s", last_tick_ * CB_DT}, {"recovery_locked", true}});
    }
    return kCapacityFaultReason;
  }
  external_reasons_[reason_count_] = reason;
  return static_cast<std::uint32_t>(1024 + reason_count_++);
}

void NavigationRuntime::enqueue(Packet packet, double source_now, bool startup, double compute) {
  if (packet.kind == Kind::Reference) {
    std::size_t references = 0;
    for (std::size_t i = 0; i < queue_size_; ++i)
      if (queue_[i].kind == Kind::Reference && queue_[i].to_mcu == packet.to_mcu) ++references;
    if (references >= 2) {
      ++references_dropped_;
      transport_events_.push({{"event", "packet_dropped"}, {"direction", "mcu"},
          {"channel", "reference"}, {"time_s", source_now}, {"reason", "reference_transport_capacity"}});
      return;
    }
  }
  if (queue_size_ == kQueueCapacity) {
    ++queue_capacity_rejections_;
    // Queue exhaustion is a visible fatal runtime error. No implicit replay,
    // command coalescing or manufactured health can conceal this failure.
    capacity_fault(source_now);
    throw std::runtime_error("native_navigation_transport_capacity_exhausted");
  }
  packet.ready = source_now + (startup ? 0. : latency_ + compute);
  queue_[queue_size_++] = std::move(packet);
  queue_high_water_ = std::max(queue_high_water_, queue_size_);
  const auto &queued = queue_[queue_size_ - 1];
  transport_events_.push({{"event", "packet_sent"}, {"direction", queued.to_mcu ? "mcu" : "pi"},
      {"channel", channel_name(queued.kind)}, {"time_s", source_now}, {"available_time_s", queued.ready}});
}

void NavigationRuntime::send_status(double source_now) {
  Packet packet;
  packet.to_mcu = false; packet.kind = Kind::SafetyStatus;
  packet.sequence = status_sequence_++; packet.source_time = source_now;
  packet.state = static_cast<std::uint32_t>(mcu_.state); packet.epoch = mcu_.epoch;
  packet.reason = static_cast<std::uint32_t>(mcu_.reason); packet.pi_lost = static_cast<std::uint32_t>(mcu_.pi_lost);
  published_state_ = packet.state; published_epoch_ = packet.epoch;
  published_reason_ = packet.reason; published_pi_lost_ = packet.pi_lost;
  enqueue(packet, source_now);
}

void NavigationRuntime::log_takeover(double source_now, std::uint32_t epoch,
    bool explicit_command, const cb_reference &previous, double issued_time) {
  safety_events_.push({{"event", "reference_revoked"}, {"time_s", source_now},
      {"invalidated_reference_epoch", epoch}, {"new_reference_epoch", mcu_.epoch}});
  safety_events_.push({{"event", "safety_takeover"}, {"time_s", source_now},
      {"reason", reason_name_unlocked(static_cast<std::uint32_t>(mcu_.reason))},
      {"action", mcu_.authority == CB_HARD ? "HARD_STOP" : "LONGITUDINAL_STOP"},
      {"invalidated_reference_epoch", epoch}, {"new_reference_epoch", mcu_.epoch},
      {"issued_time_s", explicit_command ? json(issued_time) : json(nullptr)},
      {"before_p_ref_m", previous.p}, {"before_v_ref_m_s", previous.v}});
}

void NavigationRuntime::capacity_fault(double source_now) {
  const auto previous = mcu_.output.reference;
  const auto epoch = mcu_.epoch;
  const auto tick = static_cast<std::uint32_t>(std::llround(source_now / CB_DT));
  // This is a local runtime fault, not simulated packet delivery: revoke the
  // C99 receiver before propagating the fatal capacity error to its owner.
  if (cb_mcu_safety(&mcu_, safety_sequence_++, CB_HARD, kCapacityFaultReason, tick) != CB_OK)
    throw std::runtime_error("native_capacity_fault_stop_rejected");
  production_stopped_ = true; resume_followup_.reset();
  authority_->stop("native_runtime_transport_capacity_exhausted", current_receipt_, true);
  log_takeover(source_now, epoch, false, previous);
}

void NavigationRuntime::reference_fault(double source_now) {
  const auto previous = mcu_.output.reference;
  const auto epoch = mcu_.epoch;
  const auto tick = static_cast<std::uint32_t>(std::llround(source_now / CB_DT));
  // A failed mathematical solver/policy/serialization cannot leave previously
  // queued motion authorized while its exception propagates to the process.
  if (cb_mcu_safety(&mcu_, safety_sequence_++, CB_HARD, kReferenceFaultReason, tick) != CB_OK)
    throw std::runtime_error("native_reference_fault_stop_rejected");
  production_stopped_ = true; resume_followup_.reset();
  authority_->stop("native_reference_production_failed", current_receipt_, true);
  log_takeover(source_now, epoch, false, previous);
}

void NavigationRuntime::receive_mcu(const Packet &packet, double source_now) {
  const auto tick = static_cast<std::uint32_t>(std::llround(source_now / CB_DT));
  const auto old_epoch = mcu_.epoch;
  const auto old_state = mcu_.state;
  const auto previous = mcu_.output.reference;
  int result = CB_INVALID;
  switch (packet.kind) {
    case Kind::Reference:
      result = cb_mcu_reference(&mcu_, packet.block.data(), packet.block.size(), tick);
      receiver_events_.push({{"event", result == CB_OK ? "block_received" : "reference_rejected"},
          {"time_s", source_now}, {"sequence_id", packet.sequence}, {"reference_epoch", packet.epoch},
          {"start_control_tick", packet.start_tick}, {"result", result}});
      break;
    case Kind::Heartbeat: result = cb_mcu_heartbeat(&mcu_, packet.sequence, tick); break;
    case Kind::Yaw:
      result = cb_mcu_yaw(&mcu_, packet.sequence, packet.yaw, tick);
      yaw_events_.push({{"event", result == CB_OK ? "yaw_received" : "yaw_rejected"},
          {"time_s", source_now}, {"source_time_s", packet.source_time},
          {"sequence_id", packet.sequence}, {"receive_tick", tick}});
      break;
    case Kind::Safety:
      result = cb_mcu_safety(&mcu_, packet.sequence, CB_HARD, static_cast<int>(packet.reason), tick);
      if (result == CB_OK) {
        if (old_state == CB_NORMAL) log_takeover(source_now, old_epoch, true, previous, packet.source_time);
        else safety_events_.push({{"event", "hard_stop_revoked"}, {"time_s", source_now},
            {"reason", reason_name_unlocked(static_cast<std::uint32_t>(mcu_.reason))},
            {"invalidated_reference_epoch", old_epoch}, {"new_reference_epoch", mcu_.epoch}});
        send_status(source_now);
      }
      break;
    case Kind::Resume:
      result = cb_mcu_resume(&mcu_, packet.sequence, packet.epoch, CB_HARD, tick);
      safety_events_.push({{"event", result == CB_OK ? "resume_requested" : "resume_rejected"},
          {"time_s", source_now}, {"reference_epoch", packet.epoch}, {"result", result}});
      break;
    default: throw std::logic_error("invalid_native_mcu_packet_direction");
  }
  if (result != CB_OK && packet.kind != Kind::Reference)
    packet_events_.push({{"event", "mcu_packet_rejected"}, {"time_s", source_now},
        {"channel", channel_name(packet.kind)}, {"sequence_id", packet.sequence}, {"result", result}});
}

void NavigationRuntime::deliver(bool to_mcu, double source_now) {
  // Snapshot arrivals before dispatch: a status produced by one received
  // command is not recursively delivered in the same receive pass.
  std::array<Packet, kQueueCapacity> arrivals{};
  std::size_t count = 0, waiting = 0;
  for (std::size_t i = 0; i < queue_size_; ++i) {
    if (queue_[i].to_mcu == to_mcu && queue_[i].ready <= source_now + 1e-12)
      arrivals[count++] = std::move(queue_[i]);
    else queue_[waiting++] = std::move(queue_[i]);
  }
  queue_size_ = waiting;
  for (std::size_t i = 0; i < count; ++i) {
    const auto &packet = arrivals[i];
    transport_events_.push({{"event", "packet_received"}, {"direction", to_mcu ? "mcu" : "pi"},
        {"channel", channel_name(packet.kind)}, {"time_s", source_now}});
    if (to_mcu) receive_mcu(packet, source_now);
    else if (packet.kind == Kind::RobotState) {
      latest_robot_ = packet;
      if (follow_controller_)
        follow_controller_->accept_state({packet.sequence, packet.source_time, packet.velocity});
    }
    else if (packet.kind == Kind::SafetyStatus) {
      latest_status_ = packet;
      if (packet.state != CB_NORMAL) production_stopped_ = true;
      else if (resume_sequence_ > 0) production_stopped_ = false;
    } else throw std::logic_error("invalid_native_pi_packet_direction");
  }
}

void NavigationRuntime::produce(std::uint32_t start, double source_now,
    const follow::MotionIntent &intent, bool observe, bool startup, bool resume_first) {
  follow::TwoPathBlock block;
  try {
    const auto owner = authority_->execution_state();
    select_producer_policy(!owner.authorized ? "NONE" : owner.follow ? "FOLLOW" : "GUIDE_TO");
    block = producer_.produce(start, source_now, intent.source_time_s,
        intent.linear_velocity_target_m_s, intent.yaw_rate_target_rad_s, observe);
  } catch (...) {
    reference_fault(source_now);
    throw;
  }
  Packet packet;
  packet.kind = Kind::Reference; packet.block = block.bytes;
  packet.sequence = block.sequence; packet.epoch = block.epoch; packet.start_tick = block.start_tick;
  const double available = source_now + (startup ? 0. : latency_ + (resume_first ? 0. : block.compute_wall));
  producer_events_.push({{"event", "block_generated"}, {"sequence_id", block.sequence},
      {"reference_epoch", block.epoch}, {"start_control_tick", start},
      {"generated_time_s", source_now}, {"compute_time_s", block.compute_wall},
      {"compute_cpu_time_s", block.compute_cpu}, {"replanned", block.replanned},
      {"source_time_s", block.source_time}, {"source_age_s", source_now - block.source_time},
      {"raw_velocity_target_m_s", block.raw_velocity}, {"accepted_velocity_target_m_s", block.target_velocity},
      {"candidate_delta_velocity_m_s", block.candidate_delta},
      {"candidate_target_m_s", block.candidate_target ? json(*block.candidate_target) : json(nullptr)},
      {"candidate_stable", block.candidate_stable}, {"pending_velocity_m_s", block.pending_velocity},
      {"pending_active", block.pending_active}, {"mode", producer_mode_},
      {"available_time_s", available}, {"deadline_headroom_s", start * CB_DT - available}});
  enqueue(std::move(packet), source_now, startup, resume_first ? 0. : block.compute_wall);
}

follow::MotionIntent NavigationRuntime::after_pi(std::uint32_t tick, double source_now, double receipt) {
  if (latest_status_ && latest_status_->state != CB_NORMAL &&
      (!latest_robot_ || latest_status_->source_time >= latest_robot_->source_time)) {
    authority_->update_mcu_status_with_provenance(state_name(latest_status_->state),
        reason_name_unlocked(latest_status_->reason), latest_status_->reason >= 1024 &&
        latest_status_->reason - 1024 < reason_count_, receipt);
  } else if (latest_robot_ && source_now - latest_robot_->source_time <= .1 &&
             latest_robot_->state == CB_NORMAL) {
    authority_->update_mcu_status_with_provenance("NORMAL", "normal_from_robot_state_packet", false, receipt);
  } else if (latest_status_) {
    authority_->update_mcu_status_with_provenance(state_name(latest_status_->state),
        reason_name_unlocked(latest_status_->reason), latest_status_->reason >= 1024 &&
        latest_status_->reason - 1024 < reason_count_, receipt);
  }
  const auto intent = authority_->sample(source_now, receipt);
  if (const auto stop = authority_->take_stop()) {
    production_stopped_ = true; resume_followup_.reset();
    Packet packet;
    packet.kind = Kind::Safety; packet.sequence = safety_sequence_++;
    packet.reason = external_reason(stop->second); packet.source_time = source_now;
    enqueue(packet, source_now);
    packet_events_.push({{"event", "pi_safety_command"}, {"time_s", source_now},
        {"reason", stop->second}, {"action", "HARD_STOP"}});
  }
  if (tick % 25 == 0) {
    Packet packet;
    packet.kind = Kind::Yaw; packet.sequence = yaw_sequence_++;
    packet.yaw = intent.yaw_rate_target_rad_s; packet.source_time = source_now;
    enqueue(packet, source_now);
  }
  const auto snapshot = authority_->execution_state();
  if (reason_capacity_faulted_ || !snapshot.authorized || !production_stopped_ || !moving(intent) ||
      !latest_status_ || !latest_robot_ || latest_status_->state != CB_SAFE_HOLD ||
      latest_robot_->state != latest_status_->state ||
      latest_robot_->source_time < latest_status_->source_time ||
      source_now - latest_robot_->source_time > .1 ||
      (last_resume_generation_ && snapshot.authority_epoch <= *last_resume_generation_)) return intent;
  // Only this task authority's own stop reason can be rearmed. A restored
  // heartbeat or a new STOP cannot turn an independent local fault into one.
  if (latest_status_->reason < 1024) return intent;
  auto lifecycle = json::parse(producer_.summary_json());
  // Per-block source/candidate values are retained in producer_events. Avoid
  // duplicating their larger intent rows across each retired reference epoch.
  lifecycle.erase("intents");
  lifecycle_epochs_.push({{"reference_epoch", producer_.epoch()}, {"mode", producer_mode_},
      {"summary", std::move(lifecycle)}});
  producer_.reset(latest_robot_->position, 0., latest_status_->epoch);
  const auto start = (tick / 25 + 2) * 25;
  // Original request_resume enqueues resume before the first future block.
  Packet resume;
  resume.kind = Kind::Resume; resume.sequence = resume_sequence_++;
  resume.epoch = latest_status_->epoch;
  enqueue(resume, source_now);
  produce(start, source_now, intent, true, false, true);
  resume_followup_ = start + 25;
  last_resume_generation_ = snapshot.authority_epoch;
  packet_events_.push({{"event", "pi_resume_requested"}, {"time_s", source_now},
      {"reference_epoch", resume.epoch}, {"authority", "HARD"}, {"first_block_sequence_id", 0}});
  return intent;
}

RuntimeOutput NavigationRuntime::step(std::uint32_t tick, double receipt,
    double velocity, double pitch, double rate) {
  const auto wall_before = std::chrono::steady_clock::now();
  const auto cpu_before = cpu_seconds();
  std::lock_guard<std::mutex> guard(mutex_);
  if (!std::isfinite(receipt) || !std::isfinite(velocity) || !std::isfinite(pitch) || !std::isfinite(rate) ||
      (started_ && (tick != last_tick_ + 1 || receipt < last_receipt_)) ||
      (!started_ && tick != 0) || tick >= std::numeric_limits<std::uint32_t>::max() - 1000)
    throw std::invalid_argument("invalid_native_navigation_step_time_or_estimate");
  const double now = tick * CB_DT;
  current_receipt_ = receipt;
  local_velocity_ = velocity; local_pitch_ = pitch; local_pitch_rate_ = rate;
  if (tick % 25 == 0) {
    update_follow_braking(now);
    Packet robot;
    robot.to_mcu = false; robot.kind = Kind::RobotState; robot.sequence = robot_sequence_++;
    robot.source_time = now; robot.velocity = velocity;
    robot.position = mcu_.has_last ? mcu_.output.reference.p : 0.;
    robot.state = static_cast<std::uint32_t>(mcu_.state);
    enqueue(robot, now);
  }
  deliver(false, now);
  // after_pi samples only after applying the arrived, source-timed MCU status.
  const auto intent = after_pi(tick, now, receipt);
  bool followup = false;
  if (!production_stopped_ && (!latest_status_ || latest_status_->state == CB_NORMAL) && resume_followup_) {
    produce(*resume_followup_, now, intent); resume_followup_.reset(); followup = true;
  }
  if (tick % 25 == 0) {
    Packet heartbeat;
    heartbeat.kind = Kind::Heartbeat; heartbeat.sequence = heartbeat_sequence_++;
    enqueue(heartbeat, now);
  }
  if (!followup && !production_stopped_ && (!latest_status_ || latest_status_->state == CB_NORMAL)) {
    if (tick == 0) { produce(0, now, intent, true, true); produce(25, now, intent, false, true); }
    else if (tick % 25 == 0) produce(tick + 25, now, intent);
  }
  deliver(true, now);
  const auto previous_epoch = mcu_.epoch;
  const auto previous_state = mcu_.state;
  const auto previous_reference = mcu_.output.reference;
  const bool was_armed = mcu_.resume_armed;
  RuntimeOutput output;
  const int result = cb_mcu_step(&mcu_, tick, velocity, pitch, rate, &output.command);
  if (result != CB_OK) throw std::runtime_error("native_mcu_control_step_failed");
  if (previous_state == CB_NORMAL && mcu_.state != CB_NORMAL)
    log_takeover(now, previous_epoch, false, previous_reference);
  if (!was_armed && mcu_.resume_armed)
    safety_events_.push({{"event", "resume_armed"}, {"time_s", now}, {"start_control_tick", mcu_.resume_tick}});
  if (previous_state != CB_NORMAL && mcu_.state == CB_NORMAL) {
    safety_events_.push({{"event", "normal_resumed"}, {"time_s", now}, {"reference_epoch", mcu_.epoch}});
    receiver_events_.push({{"event", "resume_block_activated"}, {"time_s", now},
        {"reference_epoch", mcu_.epoch}, {"sequence_id", mcu_.output.block_sequence}});
  } else if (previous_state == CB_BRAKING && (mcu_.state == CB_SAFE_HOLD || mcu_.state == CB_LONGITUDINAL_STOP))
    safety_events_.push({{"event", "safe_hold"}, {"time_s", now},
        {"reason", reason_name_unlocked(static_cast<std::uint32_t>(mcu_.reason))},
        {"state", state_name(static_cast<std::uint32_t>(mcu_.state))}});
  if (published_state_ != static_cast<std::uint32_t>(mcu_.state) || published_epoch_ != mcu_.epoch ||
      published_reason_ != static_cast<std::uint32_t>(mcu_.reason) || published_pi_lost_ != static_cast<std::uint32_t>(mcu_.pi_lost))
    send_status(now);
  output.block_start_time_s = mcu_.state == CB_NORMAL ? mcu_.active.start_tick * CB_DT : now;
  output.intent_source_time_s = mcu_.state == CB_NORMAL ? mcu_.active.generated_time : now;
  if (cb_mcu_peek(&mcu_, &output.next_reference) != CB_OK)
    throw std::runtime_error("native_reference_lookahead_unavailable");
  started_ = true; last_tick_ = tick; last_receipt_ = receipt;
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_before).count();
  const double cpu = cpu_seconds() - cpu_before;
  step_wall_[step_count_ % kStepCapacity] = wall; step_cpu_[step_count_ % kStepCapacity] = cpu;
  ++step_count_; step_wall_total_ += wall; step_cpu_total_ += cpu;
  step_wall_max_ = std::max(step_wall_max_, wall); step_cpu_max_ = std::max(step_cpu_max_, cpu);
  return output;
}

cb_reference NavigationRuntime::peek() const {
  std::lock_guard<std::mutex> guard(mutex_);
  cb_reference result;
  if (cb_mcu_peek(&mcu_, &result) != CB_OK) throw std::runtime_error("native_reference_lookahead_unavailable");
  return result;
}

std::uint32_t NavigationRuntime::epoch() const {
  std::lock_guard<std::mutex> guard(mutex_); return mcu_.epoch;
}
std::uint32_t NavigationRuntime::safety_state() const {
  std::lock_guard<std::mutex> guard(mutex_); return static_cast<std::uint32_t>(mcu_.state);
}
std::uint32_t NavigationRuntime::reason() const {
  std::lock_guard<std::mutex> guard(mutex_); return static_cast<std::uint32_t>(mcu_.reason);
}

std::string NavigationRuntime::report_json() const {
  std::lock_guard<std::mutex> guard(mutex_);
  json epochs = json::array(), current = json::array();
  const auto producers = producer_events_.export_rows();
  for (const auto &event : producers) {
    const auto epoch = event.at("reference_epoch");
    if (epochs.empty() || epochs.back().at("reference_epoch") != epoch)
      epochs.push_back({{"reference_epoch", epoch}, {"events", json::array()}});
    epochs.back()["events"].push_back(event);
    if (epoch == producer_.epoch()) current.push_back(event);
  }
  json metrics;
  metrics["step_count"] = step_count_;
  metrics["step_wall_total_s"] = step_wall_total_; metrics["step_cpu_total_s"] = step_cpu_total_;
  metrics["step_wall_max_s"] = step_wall_max_; metrics["step_cpu_max_s"] = step_cpu_max_;
  const auto retained = static_cast<std::size_t>(std::min<std::uint64_t>(step_count_, kStepCapacity));
  const auto first = step_count_ >= kStepCapacity ? static_cast<std::size_t>(step_count_ % kStepCapacity) : 0;
  metrics["step_wall_samples_s"] = json::array(); metrics["step_cpu_samples_s"] = json::array();
  for (std::size_t i = 0; i < retained; ++i) {
    metrics["step_wall_samples_s"].push_back(step_wall_[(first + i) % kStepCapacity]);
    metrics["step_cpu_samples_s"].push_back(step_cpu_[(first + i) % kStepCapacity]);
  }
  metrics["step_samples_overwritten"] = step_count_ > kStepCapacity ? step_count_ - kStepCapacity : 0;
  metrics["queue_high_water"] = queue_high_water_; metrics["queue_capacity"] = kQueueCapacity;
  metrics["queue_capacity_rejections"] = queue_capacity_rejections_; metrics["references_dropped"] = references_dropped_;
  metrics["reference_underruns"] = mcu_.underruns;
  metrics["reason_capacity_faulted"] = reason_capacity_faulted_;
  metrics["rejected_packets_by_code"] = mcu_.rejected;
  metrics["logs"] = json::object();
  for (const auto &pair : {std::pair<const char *, const Log *>{"safety", &safety_events_},
      {"receiver", &receiver_events_}, {"producer", &producer_events_}, {"packet", &packet_events_},
      {"yaw", &yaw_events_}, {"transport", &transport_events_}, {"lifecycle_epochs", &lifecycle_epochs_},
      {"braking_projection", &braking_events_}})
    metrics["logs"][pair.first] = {{"total", pair.second->total}, {"retained", pair.second->rows.size()},
        {"capacity", pair.second->capacity}, {"overwritten", pair.second->overwritten}};
  json report = {{"arbiter_state", state_name(static_cast<std::uint32_t>(mcu_.state))},
      {"arbiter_reason", reason_name_unlocked(static_cast<std::uint32_t>(mcu_.reason))},
      {"safety_events", safety_events_.export_rows()}, {"receiver_events", receiver_events_.export_rows()},
      {"producer_events", current}, {"producer_epochs", epochs},
      {"packet_events", packet_events_.export_rows()}, {"yaw_events", yaw_events_.export_rows()},
      {"transport_events", transport_events_.export_rows()}, {"metrics", metrics},
      {"reference_lifecycle_epochs", lifecycle_epochs_.export_rows()},
      {"reference_lifecycle_current", json::parse(producer_.summary_json())},
      {"follow_braking_events", braking_events_.export_rows()},
      {"owner", "native_cpp_pi_scheduler_and_shared_c99_mcu"},
      {"hardware_execution_ready", false}, {"actual_actuator_feedback", "owned_by_simulator_or_board_io"}};
  return report.dump();
}
}  // namespace companionbot::simulation
