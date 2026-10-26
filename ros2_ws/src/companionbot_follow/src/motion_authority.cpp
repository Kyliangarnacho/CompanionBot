#include "companionbot_follow/motion_authority.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace companionbot::follow {
namespace {
bool positive_finite(double value) { return std::isfinite(value) && value > 0.; }
}

MotionAuthority::MotionAuthority(std::string id, std::string version, double velocity,
                                 double yaw, double cmd_timeout, double health_timeout,
                                 std::optional<FollowAuthorityConfig> follow_config)
    : map_id(std::move(id)), map_version(std::move(version)),
      max_velocity_m_s(velocity), max_yaw_rate_rad_s(yaw),
      command_timeout_s(cmd_timeout), health_timeout_s(health_timeout), follow_config_(follow_config) {
  if (map_id.empty() || map_version.empty() || map_id.size() > 256 || map_version.size() > 256 ||
      !positive_finite(velocity) || !positive_finite(yaw) || !positive_finite(cmd_timeout) ||
      !positive_finite(health_timeout) || velocity > .6 || yaw > .5) {
    throw std::invalid_argument("invalid_navigation_envelope");
  }
  if (follow_config_) {
    const auto& config = *follow_config_;
    if (!positive_finite(config.max_velocity_m_s) || !positive_finite(config.max_yaw_rate_rad_s) ||
        !positive_finite(config.command_timeout_s) || !positive_finite(config.health_timeout_s) ||
        !positive_finite(config.obstacle_timeout_s) || !positive_finite(config.braking_timeout_s) ||
        config.max_velocity_m_s > .6 || config.max_yaw_rate_rad_s > .5)
      throw std::invalid_argument("invalid_follow_authority_envelope");
    follow_reason_ = "follow_observation_unavailable";
  }
}

std::string MotionAuthority::bounded_reason(const std::string& reason) {
  return reason.size() <= 256 ? reason : "navigation_invalid_reason_length";
}

void MotionAuthority::event_locked(AuthorityEvent event) {
  if (events_.size() == kEventCapacity) {
    events_.pop_front();
    ++events_overwritten_;
  }
  events_.push_back(std::move(event));
}

void MotionAuthority::stop_locked(const std::string& raw_reason, double receipt_now_s, bool force) {
  const bool had_authority = task_id_.has_value();
  task_id_.reset();
  source_epoch_.reset();
  mode_ = Mode::None;
  latest_.reset();
  lease_until_ = 0.;
  reason_ = bounded_reason(raw_reason);
  if (had_authority || force) {
    ++authority_epoch_;
    ++stop_serial_;
    pending_stop_ = std::make_pair(stop_serial_, reason_);
    if (owned_stop_reasons_.count(reason_) == 0) {
      if (owned_stop_reasons_.size() == kStopReasonCapacity) {
        recovery_locked_ = true;
        ++capacity_rejections_;
      } else {
        owned_stop_reasons_.insert(reason_);
      }
    }
    AuthorityEvent event{"navigation_authority_revoked", receipt_now_s};
    event.reason = reason_;
    event.authority_epoch = authority_epoch_;
    event_locked(std::move(event));
  }
}

void MotionAuthority::stop(const std::string& reason, double receipt_now_s, bool force) {
  std::lock_guard<std::mutex> guard(mutex_);
  stop_locked(reason, receipt_now_s, force);
}

void MotionAuthority::update_health(bool available, const std::string& reason, double receipt_now_s) {
  std::lock_guard<std::mutex> guard(mutex_);
  healthy_ = available && std::isfinite(receipt_now_s);
  health_reason_ = reason.empty() ? "navigation_health_unavailable" : bounded_reason(reason);
  health_at_ = receipt_now_s;
  if (!healthy_) stop_locked(health_reason_, receipt_now_s);
}

void MotionAuthority::update_mcu_status(const std::string& state, const std::string& reason,
                                      double receipt_now_s) {
  std::lock_guard<std::mutex> guard(mutex_);
  // Preserved only for the frozen Python oracle/software fixture boundary.
  update_mcu_status_locked(state, reason, owned_stop_reasons_.count(reason) != 0, receipt_now_s);
}

void MotionAuthority::update_mcu_status_with_provenance(const std::string& state, const std::string& reason,
    bool task_owned_stop, double receipt_now_s) {
  std::lock_guard<std::mutex> guard(mutex_);
  update_mcu_status_locked(state, reason, task_owned_stop, receipt_now_s);
}

void MotionAuthority::update_mcu_status_locked(const std::string& state, const std::string& reason,
    bool task_owned_stop, double receipt_now_s) {
  mcu_state_ = state.size() <= 32 ? state : "INVALID";
  mcu_reason_ = bounded_reason(reason);
  mcu_recovery_owned_ = task_owned_stop && owned_stop_reasons_.count(mcu_reason_) != 0 &&
      (state == "BRAKING" || state == "SAFE_HOLD" || state == "LONGITUDINAL_STOP");
  if (state != "NORMAL" && !mcu_recovery_owned_) {
    stop_locked("navigation_mcu_safety:" + mcu_reason_, receipt_now_s);
  }
}

bool MotionAuthority::mcu_ready_locked() const {
  return !recovery_locked_ && (mcu_state_ == "NORMAL" ||
      (mcu_state_ == "SAFE_HOLD" && mcu_recovery_owned_));
}

bool MotionAuthority::follow_observation_ready_locked(double source_now_s, double receipt_now_s) const {
  if (!follow_config_ || !follow_healthy_ || !follow_health_sample_ ||
      !std::isfinite(source_now_s) || !std::isfinite(receipt_now_s)) return false;
  const auto& sample = *follow_health_sample_;
  const double source_age = source_now_s - sample.source_time_s;
  const double receipt_age = receipt_now_s - sample.received_at_s;
  return source_age >= -.02 && source_age <= follow_config_->health_timeout_s &&
      receipt_age >= 0. && receipt_age <= follow_config_->health_timeout_s;
}

std::string MotionAuthority::follow_safety_reason_locked(double source_now, double receipt) const {
  if (!follow_config_) return "follow_disabled";
  if (!std::isfinite(source_now) || !std::isfinite(receipt)) return "follow_guard_clock_invalid";
  if (!follow_obstacle_) return "follow_obstacle_unavailable";
  if (source_now - follow_obstacle_->source_time_s < -.02 ||
      source_now - follow_obstacle_->source_time_s > follow_config_->obstacle_timeout_s ||
      receipt < follow_obstacle_->received_at_s ||
      receipt - follow_obstacle_->received_at_s > follow_config_->obstacle_timeout_s)
    return "follow_obstacle_expired";
  if (!follow_braking_) return "follow_braking_unavailable";
  if (receipt < follow_braking_->received_at_s ||
      receipt - follow_braking_->received_at_s > follow_config_->braking_timeout_s)
    return "follow_braking_expired";
  if (follow_obstacle_->clearance_m <= follow_braking_->required_clearance_m)
    return "follow_obstacle_clearance";
  return "";
}

bool MotionAuthority::follow_ready_locked(double source_now, double receipt) const {
  return follow_observation_ready_locked(source_now, receipt) && follow_safety_reason_locked(source_now, receipt).empty();
}

void MotionAuthority::check_locked(double source_now_s, double receipt_now_s) {
  follow_checked_source_s_ = source_now_s;
  follow_checked_receipt_s_ = receipt_now_s;
  if (!task_id_) return;
  if (!std::isfinite(source_now_s) || !std::isfinite(receipt_now_s)) {
    stop_locked("navigation_clock_invalid", receipt_now_s);
  } else if (receipt_now_s >= lease_until_) {
    stop_locked("navigation_lease_expired", receipt_now_s);
  } else if (!healthy_ || receipt_now_s - health_at_ > health_timeout_s) {
    stop_locked(!healthy_ ? health_reason_ : "navigation_health_timeout", receipt_now_s);
  } else if (mode_ == Mode::Follow && !follow_ready_locked(source_now_s, receipt_now_s)) {
    stop_locked(!follow_observation_ready_locked(source_now_s, receipt_now_s) ?
        (follow_healthy_ ? "follow_observation_expired" : follow_reason_) :
        follow_safety_reason_locked(source_now_s, receipt_now_s), receipt_now_s);
  } else if (latest_) {
    const double timeout = mode_ == Mode::Follow ? follow_config_->command_timeout_s : command_timeout_s;
    if (source_now_s < latest_->source_time_s - .02) {
      stop_locked("navigation_clock_discontinuity", receipt_now_s);
    } else if (source_now_s - latest_->source_time_s > timeout ||
               receipt_now_s - latest_->received_at_s > timeout) {
      stop_locked(mode_ == Mode::Follow ? "follow_command_expired" : "navigation_command_expired", receipt_now_s);
    }
  }
}

AuthorityAnswer MotionAuthority::control(const AuthorityControl& request) {
  std::lock_guard<std::mutex> guard(mutex_);
  const auto answer = [this](bool ok, const std::string& reason) {
    return AuthorityAnswer{ok, reason, authority_epoch_};
  };
  if (request.task_id.empty() || request.task_id.size() > 80 || request.source_epoch.empty() ||
      request.source_epoch.size() > 128 || !std::isfinite(request.source_now_s) ||
      !std::isfinite(request.receipt_now_s)) {
    return answer(false, "invalid_navigation_identity");
  }
  const auto key = std::make_pair(request.task_id, request.source_epoch);
  const auto watermark = watermarks_.find(key);
  if (watermark != watermarks_.end() && request.sequence <= watermark->second) {
    return answer(false, "stale_navigation_control");
  }
  const auto& command = request.command;
  if (command != "CHECK" && command != "CHECK_STOPPED" && command != "ACQUIRE" &&
      command != "RENEW" && command != "RELEASE" && command != "STOP" && command != "WAIT" &&
      command != "CHECK_FOLLOW" && command != "ACQUIRE_FOLLOW" && command != "RENEW_FOLLOW") {
    return answer(false, "unknown_navigation_control");
  }
  // Never evict replay protection. Capacity cannot block a safety revocation;
  // such a STOP/WAIT is accepted but locks future rearm until a fresh runtime.
  if (watermark == watermarks_.end() && watermarks_.size() == kWatermarkCapacity) {
    ++capacity_rejections_;
    recovery_locked_ = true;
    if (command != "STOP" && command != "WAIT") {
      return answer(false, "navigation_identity_capacity_exhausted");
    }
    ++safety_capacity_bypasses_;
  } else {
    watermarks_[key] = request.sequence;
  }
  if (command == "STOP" || command == "WAIT") {
    stop_locked(command == "STOP" ? "navigation_stop" : "navigation_wait",
                request.receipt_now_s, true);
    return answer(true, "navigation_authority_stopped");
  }
  if (request.map_id != map_id || request.map_version != map_version) {
    return answer(false, "navigation_map_version_mismatch");
  }
  if (command == "CHECK_STOPPED") {
    return answer(mcu_state_ == "SAFE_HOLD" && !task_id_,
                  mcu_state_ == "SAFE_HOLD" ? "mcu_safe_hold" : "mcu_stop_pending");
  }
  if (command == "CHECK" || command == "CHECK_FOLLOW") {
    check_locked(request.source_now_s, request.receipt_now_s);
    const bool healthy = healthy_ && request.receipt_now_s - health_at_ <= health_timeout_s;
    if (command == "CHECK_FOLLOW") {
      const bool follow_ready = follow_ready_locked(request.source_now_s, request.receipt_now_s);
      return answer(healthy && mcu_ready_locked() && follow_ready,
          !follow_config_ ? "follow_disabled" : recovery_locked_ ? "navigation_recovery_capacity_locked" :
          !healthy ? health_reason_ : !mcu_ready_locked() ? mcu_reason_ :
          !follow_ready ? (!follow_observation_ready_locked(request.source_now_s, request.receipt_now_s) ?
              (follow_healthy_ ? "follow_observation_expired" : follow_reason_) :
              follow_safety_reason_locked(request.source_now_s, request.receipt_now_s)) : "follow_ready");
    }
    return answer(healthy && mcu_ready_locked(), recovery_locked_ ? "navigation_recovery_capacity_locked" :
        (healthy && mcu_ready_locked() ? "navigation_ready" : !healthy ? health_reason_ : mcu_reason_));
  }
  const bool same_owner = task_id_ == request.task_id && source_epoch_ == request.source_epoch;
  if (command == "RELEASE") {
    if (!same_owner) return answer(false, "navigation_owner_mismatch");
    stop_locked("navigation_task_released", request.receipt_now_s);
    return answer(true, "navigation_authority_released");
  }
  if (!std::isfinite(request.lease_duration_s) || request.lease_duration_s < .1 ||
      request.lease_duration_s > 2.) return answer(false, "invalid_navigation_lease");
  check_locked(request.source_now_s, request.receipt_now_s);
  if (!healthy_ || request.receipt_now_s - health_at_ > health_timeout_s) {
    return answer(false, health_reason_);
  }
  if (recovery_locked_) return answer(false, "navigation_recovery_capacity_locked");
  if (!mcu_ready_locked()) return answer(false, mcu_reason_);
  const bool follow = command == "ACQUIRE_FOLLOW" || command == "RENEW_FOLLOW";
  const Mode requested_mode = follow ? Mode::Follow : Mode::GuideTo;
  if (follow && !follow_ready_locked(request.source_now_s, request.receipt_now_s))
    return answer(false, !follow_config_ ? "follow_disabled" :
        !follow_observation_ready_locked(request.source_now_s, request.receipt_now_s) ?
            (follow_healthy_ ? "follow_observation_expired" : follow_reason_) :
            follow_safety_reason_locked(request.source_now_s, request.receipt_now_s));
  // check_locked may have revoked the previously observed owner.
  const bool current_owner = task_id_ == request.task_id && source_epoch_ == request.source_epoch;
  if (task_id_ && !current_owner) return answer(false, "motion_authority_busy");
  if (task_id_ && mode_ != requested_mode) return answer(false, "motion_authority_busy");
  if ((command == "RENEW" || command == "RENEW_FOLLOW") && !current_owner)
    return answer(false, "navigation_owner_mismatch");
  if (!task_id_) {
    ++authority_epoch_;
    task_id_ = request.task_id;
    source_epoch_ = request.source_epoch;
    mode_ = requested_mode;
    granted_source_time_ = request.source_now_s;
    latest_.reset();
    AuthorityEvent event{"navigation_authority_acquired", request.receipt_now_s};
    event.task_id = task_id_;
    event.source_epoch = source_epoch_;
    event.authority_epoch = authority_epoch_;
    if (follow) event.mode = "FOLLOW";
    event_locked(std::move(event));
  }
  lease_until_ = request.receipt_now_s + request.lease_duration_s;
  reason_ = follow ? "follow_authorized" : "navigation_authorized";
  return answer(true, reason_);
}

bool MotionAuthority::receive_cmd(double linear_m_s, double yaw_rad_s, double source_time_s,
                                  double source_now_s, double receipt_now_s,
                                  const std::string& source_id) {
  std::lock_guard<std::mutex> guard(mutex_);
  // A foreign Nav2 callback cannot mutate the active follow task, including
  // by sending malformed or stale twist data from its independent lifecycle.
  if (mode_ == Mode::Follow) return false;
  if (!std::isfinite(linear_m_s) || !std::isfinite(yaw_rad_s) || !std::isfinite(source_time_s) ||
      !std::isfinite(source_now_s) || !std::isfinite(receipt_now_s) || source_id.size() > 256) {
    stop_locked("invalid_navigation_twist", receipt_now_s);
    return false;
  }
  check_locked(source_now_s, receipt_now_s);
  if (!task_id_) return false;
  if (source_time_s < granted_source_time_ - 1e-9 || source_now_s - source_time_s < -.02 ||
      source_now_s - source_time_s > command_timeout_s ||
      (latest_ && source_time_s <= latest_->source_time_s)) {
    AuthorityEvent event{"navigation_command_rejected", receipt_now_s};
    event.reason = "stale_or_unmapped_command";
    event.source_time_s = source_time_s;
    event.task_id = task_id_;
    event_locked(std::move(event));
    return false;
  }
  if (linear_m_s < -1e-9) {
    stop_locked("navigation_reverse_not_authorized", receipt_now_s);
    return false;
  }
  const double linear = std::max(0., std::min(max_velocity_m_s, linear_m_s));
  const double yaw = std::max(-max_yaw_rate_rad_s, std::min(max_yaw_rate_rad_s, yaw_rad_s));
  latest_ = Latest{source_time_s, receipt_now_s, linear, yaw};
  AuthorityEvent event{"navigation_command", receipt_now_s};
  event.task_id = task_id_;
  event.source_id = source_id;
  event.source_time_s = source_time_s;
  event.requested_v_m_s = linear_m_s;
  event.requested_yaw_rad_s = yaw_rad_s;
  event.limited_v_m_s = linear;
  event.limited_yaw_rad_s = yaw;
  event.saturated = linear != linear_m_s || yaw != yaw_rad_s;
  event_locked(std::move(event));
  return true;
}

bool MotionAuthority::update_follow_health(bool available, const std::string& reason,
    const std::string& epoch, std::uint64_t sequence, double source_time,
    double source_now, double receipt_now) {
  std::lock_guard<std::mutex> guard(mutex_);
  const auto unavailable = [&](const std::string& value) {
    follow_healthy_ = false;
    follow_reason_ = bounded_reason(value);
    ++follow_health_rejections_;
    if (mode_ == Mode::Follow) stop_locked(follow_reason_, receipt_now);
    return false;
  };
  if (!follow_config_) return unavailable("follow_disabled");
  // Identity/watermark gates precede validity. A delayed LOST/depth failure
  // from an old frame is no more authoritative than its delayed successful
  // counterpart, and cannot revoke a new task or a newer camera observation.
  if (!follow_observation_identity_locked(epoch, sequence, source_time) ||
      (follow_health_sample_ && epoch == follow_health_sample_->epoch &&
       receipt_now < follow_health_sample_->received_at_s)) {
    ++follow_health_rejections_;
    return false;
  }
  follow_checked_source_s_ = source_now;
  follow_checked_receipt_s_ = receipt_now;
  if (!std::isfinite(source_now) || !std::isfinite(receipt_now))
    return unavailable("follow_observation_identity_or_clock_invalid");
  if (source_now - source_time < -.02 || source_now - source_time > follow_config_->health_timeout_s)
    return unavailable("follow_observation_stale_or_unmapped");
  if (follow_health_sample_ && epoch != follow_health_sample_->epoch) {
    if (follow_retired_epochs_.size() == kWatermarkCapacity) {
      recovery_locked_ = true;
      ++capacity_rejections_;
      return unavailable("follow_source_epoch_capacity_exhausted");
    }
    follow_retired_epochs_.insert(follow_health_sample_->epoch);
    if (mode_ == Mode::Follow) stop_locked("follow_source_epoch_changed", receipt_now);
  }
  follow_health_sample_ = FollowHealth{epoch, sequence, source_time, receipt_now};
  follow_healthy_ = available;
  follow_reason_ = available ? "follow_observation_valid" :
      bounded_reason(reason.empty() ? "follow_observation_invalid" : reason);
  if (!available && mode_ == Mode::Follow) stop_locked(follow_reason_, receipt_now);
  return available;
}

bool MotionAuthority::follow_observation_identity_locked(const std::string& epoch,
    std::uint64_t sequence, double source_time) const {
  if (!follow_config_ || epoch.empty() || epoch.size() > 128 || !std::isfinite(source_time) ||
      follow_retired_epochs_.count(epoch) || (mode_ == Mode::Follow && source_time <= granted_source_time_)) return false;
  return !follow_health_sample_ || epoch != follow_health_sample_->epoch ||
      (sequence > follow_health_sample_->sequence && source_time > follow_health_sample_->source_time_s);
}

bool MotionAuthority::follow_observation_identity_current(const std::string& epoch,
    std::uint64_t sequence, double source_time) const {
  std::lock_guard<std::mutex> guard(mutex_);
  return follow_observation_identity_locked(epoch, sequence, source_time);
}

bool MotionAuthority::receive_follow_cmd(const std::string& task, const std::string& task_epoch,
    std::uint64_t authority_epoch, const std::string& observation_epoch, std::uint64_t observation_sequence,
    double source_time, double source_now, double receipt_now, double linear_m_s, double yaw_rad_s) {
  std::lock_guard<std::mutex> guard(mutex_);
  const auto reject = [&] { ++follow_command_rejections_; return false; };
  // Delayed work from another task/generation is powerless over current motion.
  if (mode_ != Mode::Follow || task_id_ != task || source_epoch_ != task_epoch ||
      authority_epoch_ != authority_epoch) return reject();
  check_locked(source_now, receipt_now);
  if (mode_ != Mode::Follow) return reject();
  if (!follow_ready_locked(source_now, receipt_now) || !follow_health_sample_ ||
      follow_health_sample_->epoch != observation_epoch || follow_health_sample_->sequence != observation_sequence ||
      follow_health_sample_->source_time_s != source_time || source_time <= granted_source_time_ ||
      source_now - source_time > follow_config_->command_timeout_s ||
      (latest_ && source_time <= latest_->source_time_s)) return reject();
  if (!std::isfinite(linear_m_s) || !std::isfinite(yaw_rad_s) || linear_m_s < -1e-9) {
    stop_locked("invalid_follow_intent", receipt_now);
    return reject();
  }
  const double linear = std::max(0., std::min(follow_config_->max_velocity_m_s, linear_m_s));
  const double yaw = std::max(-follow_config_->max_yaw_rate_rad_s,
                            std::min(follow_config_->max_yaw_rate_rad_s, yaw_rad_s));
  latest_ = Latest{source_time, receipt_now, linear, yaw};
  AuthorityEvent event{"follow_command", receipt_now};
  event.task_id = task_id_; event.source_id = observation_epoch; event.mode = "FOLLOW";
  event.authority_epoch = authority_epoch_; event.source_time_s = source_time;
  event.requested_v_m_s = linear_m_s; event.requested_yaw_rad_s = yaw_rad_s;
  event.limited_v_m_s = linear; event.limited_yaw_rad_s = yaw;
  event.saturated = linear != linear_m_s || yaw != yaw_rad_s;
  event_locked(std::move(event));
  return true;
}

bool MotionAuthority::update_follow_obstacle(double clearance, double source_time, double source_now, double receipt) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (!follow_config_) return false;
  if (!std::isfinite(clearance) || clearance < 0. || !std::isfinite(source_time) ||
      !std::isfinite(source_now) || !std::isfinite(receipt) ||
      source_now - source_time < -.02 || source_now - source_time > follow_config_->obstacle_timeout_s) {
    ++follow_guard_rejections_;
    follow_obstacle_.reset();
    if (mode_ == Mode::Follow) stop_locked("follow_obstacle_invalid", receipt);
    return false;
  }
  if (follow_obstacle_ && (source_time <= follow_obstacle_->source_time_s || receipt < follow_obstacle_->received_at_s)) {
    ++follow_guard_rejections_; return false;
  }
  follow_obstacle_ = ObstacleSample{clearance, source_time, receipt};
  if (mode_ == Mode::Follow && follow_braking_ && clearance <= follow_braking_->required_clearance_m)
    stop_locked("follow_obstacle_clearance", receipt);
  return true;
}

bool MotionAuthority::update_follow_braking(double required, double receipt) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (!follow_config_) return false;
  if (!std::isfinite(required) || required < 0. || !std::isfinite(receipt)) {
    ++follow_guard_rejections_;
    follow_braking_.reset();
    if (mode_ == Mode::Follow) stop_locked("follow_braking_invalid", receipt);
    return false;
  }
  if (follow_braking_ && receipt < follow_braking_->received_at_s) {
    ++follow_guard_rejections_; return false;
  }
  follow_braking_ = BrakingSample{required, receipt};
  if (mode_ == Mode::Follow && follow_obstacle_ && follow_obstacle_->clearance_m <= required)
    stop_locked("follow_obstacle_clearance", receipt);
  return true;
}

bool MotionAuthority::invalidate_follow(const std::string& reason, double receipt_now) {
  std::lock_guard<std::mutex> guard(mutex_);
  follow_healthy_ = false;
  follow_reason_ = bounded_reason(reason.empty() ? "follow_source_invalid" : reason);
  const bool revoked = mode_ == Mode::Follow;
  if (revoked) stop_locked(follow_reason_, receipt_now);
  return revoked;
}

bool MotionAuthority::invalidate_navigation(const std::string& reason, double receipt_now) {
  std::lock_guard<std::mutex> guard(mutex_);
  const bool revoked = mode_ == Mode::GuideTo;
  if (revoked) stop_locked(reason.empty() ? "navigation_input_invalid" : reason, receipt_now);
  return revoked;
}

MotionIntent MotionAuthority::sample(double source_now_s, double receipt_now_s) {
  std::lock_guard<std::mutex> guard(mutex_);
  check_locked(source_now_s, receipt_now_s);
  if (task_id_ && latest_) return {latest_->source_time_s, latest_->linear_m_s, latest_->yaw_rad_s};
  return {source_now_s, 0., 0.};
}

std::optional<std::pair<std::uint64_t, std::string>> MotionAuthority::take_stop() {
  std::lock_guard<std::mutex> guard(mutex_);
  auto result = std::move(pending_stop_);
  pending_stop_.reset();
  return result;
}

AuthoritySnapshot MotionAuthority::snapshot(bool include_events) const {
  std::lock_guard<std::mutex> guard(mutex_);
  AuthoritySnapshot result;
  result.task_id = task_id_; result.source_epoch = source_epoch_; result.authority_epoch = authority_epoch_;
  result.reason = reason_; result.mcu_state = mcu_state_; result.mcu_reason = mcu_reason_;
  result.map_id = map_id; result.map_version = map_version; result.authorized = task_id_.has_value();
  result.recovery_locked = recovery_locked_; result.retained_events = events_.size();
  result.mcu_recovery_owned = mcu_recovery_owned_;
  result.retained_watermarks = watermarks_.size(); result.retained_stop_reasons = owned_stop_reasons_.size();
  result.events_overwritten = events_overwritten_; result.capacity_rejections = capacity_rejections_;
  result.safety_capacity_bypasses = safety_capacity_bypasses_;
  result.mode = mode_ == Mode::Follow ? "FOLLOW" : mode_ == Mode::GuideTo ? "GUIDE_TO" : "NONE";
  result.follow_enabled = follow_config_.has_value();
  result.follow_health = follow_healthy_;
  result.follow_reason = follow_reason_;
  result.master_locked = follow_observation_ready_locked(follow_checked_source_s_, follow_checked_receipt_s_);
  result.follow_health_rejections = follow_health_rejections_;
  result.follow_command_rejections = follow_command_rejections_;
  if (task_id_) result.granted_source_time_s = granted_source_time_;
  result.follow_safety_reason = follow_safety_reason_locked(follow_checked_source_s_, follow_checked_receipt_s_);
  result.follow_obstacle_clear = result.follow_safety_reason.empty();
  result.follow_guard_rejections = follow_guard_rejections_;
  if (follow_obstacle_) {
    result.follow_obstacle_clearance_m = follow_obstacle_->clearance_m;
    result.follow_obstacle_source_time_s = follow_obstacle_->source_time_s;
    result.follow_obstacle_received_at_s = follow_obstacle_->received_at_s;
    result.follow_obstacle_source_age_s = follow_checked_source_s_ - follow_obstacle_->source_time_s;
    result.follow_obstacle_receipt_age_s = follow_checked_receipt_s_ - follow_obstacle_->received_at_s;
  }
  if (follow_braking_) {
    result.follow_required_clearance_m = follow_braking_->required_clearance_m;
    result.follow_braking_received_at_s = follow_braking_->received_at_s;
    result.follow_braking_age_s = follow_checked_receipt_s_ - follow_braking_->received_at_s;
  }
  if (follow_health_sample_) {
    result.follow_source_epoch = follow_health_sample_->epoch;
    result.follow_sequence = follow_health_sample_->sequence;
    result.follow_source_time_s = follow_health_sample_->source_time_s;
    result.follow_received_at_s = follow_health_sample_->received_at_s;
    result.follow_source_age_s = follow_checked_source_s_ - follow_health_sample_->source_time_s;
    result.follow_receipt_age_s = follow_checked_receipt_s_ - follow_health_sample_->received_at_s;
  }
  if (include_events) result.events.assign(events_.begin(), events_.end());
  return result;
}

AuthorityExecutionState MotionAuthority::execution_state() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return {task_id_.has_value(), mode_ == Mode::Follow, authority_epoch_};
}

}  // namespace companionbot::follow
