#include "companionbot_follow/lightweight_producer.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <time.h>

namespace companionbot::follow {
namespace {
double thread_seconds() {
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0)
    throw std::runtime_error("thread_cpu_clock_unavailable");
  return static_cast<double>(value.tv_sec) + static_cast<double>(value.tv_nsec) * 1e-9;
}
void u32(std::uint8_t *out, std::uint32_t value) {
  for (unsigned i = 0; i < 4; ++i) out[i] = static_cast<std::uint8_t>(value >> (8 * i));
}
void f32(std::uint8_t *out, double value) {
  const float converted = static_cast<float>(value);
  if (!std::isfinite(converted)) throw std::overflow_error("reference_float32_overflow");
  std::uint32_t bits{};
  static_assert(sizeof(converted) == sizeof(bits));
  std::memcpy(&bits, &converted, sizeof(bits));
  u32(out, bits);
}
void f64(std::uint8_t *out, double value) {
  std::uint64_t bits{};
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&bits, &value, sizeof(bits));
  for (unsigned i = 0; i < 8; ++i) out[i] = static_cast<std::uint8_t>(bits >> (8 * i));
}
}  // namespace

LightweightProducer::LightweightProducer(ProducerConfig config, std::uint32_t epoch)
    : config_(config), ruckig_(config.dt), epoch_(epoch) {
  if (!std::isfinite(config.dt) || std::abs(config.dt - .002) > 1e-12 ||
      config.block_samples != 25 || !std::isfinite(config.max_velocity) ||
      config.max_velocity <= 0 || !std::isfinite(config.max_acceleration) ||
      config.max_acceleration <= 0 || !std::isfinite(config.max_jerk) ||
      config.max_jerk <= 0 || !std::isfinite(config.full_delta_velocity) ||
      config.full_delta_velocity <= 0)
    throw std::invalid_argument("invalid_lightweight_producer_configuration");
  // Reserve a bounded, configuration-derived working buffer once. The bound
  // includes reversal at the velocity limits and acceleration ramp margins.
  const double bound = (2 * config.max_velocity / config.max_acceleration +
                       4 * config.max_acceleration / config.max_jerk + 8 * config.dt) / config.dt;
  if (!std::isfinite(bound) || bound > 1000000)
    throw std::invalid_argument("lightweight_profile_capacity_excessive");
  profile_.reserve(static_cast<std::size_t>(std::ceil(bound)) + 1);
}

void LightweightProducer::reset(double position, double velocity, std::uint32_t epoch) {
  if (!std::isfinite(position) || !std::isfinite(velocity) ||
      std::abs(velocity) > config_.max_velocity)
    throw std::invalid_argument("invalid_lightweight_reset");
  current_ = {position, velocity, 0.};
  accepted_ = velocity;
  epoch_ = epoch;
  sequence_ = 0;
  transient_ = observed_ = false;
  profile_.clear();
  index_ = 0;
}

void LightweightProducer::plan(double target) {
  ruckig::InputParameter<1> input;
  input.control_interface = ruckig::ControlInterface::Velocity;
  input.duration_discretization = ruckig::DurationDiscretization::Discrete;
  input.current_position = {current_[0]};
  input.current_velocity = {current_[1]};
  input.current_acceleration = {current_[2]};
  input.target_velocity = {target};
  input.target_acceleration = {0.};
  input.max_velocity = {config_.max_velocity};
  input.max_acceleration = {config_.max_acceleration};
  input.max_jerk = {config_.max_jerk};
  const auto result = ruckig_.calculate(input, trajectory_);
  if (result != ruckig::Result::Working && result != ruckig::Result::Finished)
    throw std::runtime_error("ruckig_lightweight_calculation_failed");
  duration_ = trajectory_.get_duration();
  const auto intervals = static_cast<std::size_t>(std::nearbyint(duration_ / config_.dt));
  if (intervals < 1 || intervals + 1 > profile_.capacity() ||
      std::abs(static_cast<double>(intervals) * config_.dt - duration_) > 1e-10)
    throw std::runtime_error("ruckig_lightweight_profile_geometry");
  profile_.resize(intervals + 1);
  std::array<double, 1> position{}, velocity{}, acceleration{};
  trajectory_.at_time(duration_, position, velocity, acceleration);
  const double terminal_position = position[0];
  for (std::size_t i = 0; i <= intervals; ++i) {
    const double t = static_cast<double>(i) * config_.dt;
    if (t >= duration_ - 1e-12) {
      profile_[i] = {terminal_position + (t - duration_) * target, target, 0.};
    } else {
      trajectory_.at_time(t, position, velocity, acceleration);
      profile_[i] = {position[0], velocity[0], acceleration[0]};
    }
  }
  // Original _kinematic_preview overwrites both endpoint constraints.
  profile_.front() = current_;
  profile_.back()[1] = target;
  profile_.back()[2] = 0.;
  accepted_ = target;
  index_ = 0;
  transient_ = true;
}

void LightweightProducer::advance() {
  if (transient_) {
    ++index_;
    current_ = profile_.at(index_);
    if (index_ == profile_.size() - 1) transient_ = false;
  } else {
    // Preserve repeated-addition hold arithmetic from VelocityReferenceLifecycle.
    current_[0] += current_[1] * config_.dt;
  }
}

ProducedBlock LightweightProducer::produce(std::uint32_t start_tick, double generated_time,
    double source_time, double velocity, double yaw, bool observe_command) {
  const auto before = std::chrono::steady_clock::now();
  const auto cpu_before = thread_seconds();
  if (!std::isfinite(generated_time) || !std::isfinite(source_time) ||
      !std::isfinite(velocity) || !std::isfinite(yaw) ||
      std::abs(velocity) > config_.max_velocity + 1e-12 ||
      sequence_ == std::numeric_limits<std::uint32_t>::max() ||
      start_tick > std::numeric_limits<std::uint32_t>::max() - 25)
    throw std::invalid_argument("invalid_lightweight_block_input");
  if (!observe_command && !observed_)
    throw std::logic_error("lightweight_prefill_without_observation");
  ProducedBlock block;
  if (observe_command) {
    const double delta = velocity - accepted_;
    if (std::abs(delta) > 1e-12) {
      if (std::abs(delta) >= config_.full_delta_velocity)
        throw std::invalid_argument("full_dynamic_plan_required");
      plan(velocity);
      block.replanned = true;
    }
    observed_ = true;
    last_source_ = source_time;
    last_yaw_ = yaw;
  }
  block.epoch = epoch_;
  block.sequence = sequence_;
  block.start_tick = start_tick;
  block.generated_time = generated_time;
  block.source_time = last_source_;
  block.target_velocity = accepted_;
  block.yaw = last_yaw_;
  block.duration = duration_;
  u32(block.bytes.data(), 1);
  u32(block.bytes.data() + 4, epoch_);
  u32(block.bytes.data() + 8, sequence_);
  u32(block.bytes.data() + 12, start_tick);
  f64(block.bytes.data() + 16, generated_time);
  f64(block.bytes.data() + 24, config_.dt);
  u32(block.bytes.data() + 32, config_.block_samples);
  f32(block.bytes.data() + 36, last_yaw_);
  for (std::size_t i = 0; i < 25; ++i) {
    block.samples[i] = {current_[0], current_[1], current_[2], 0., 0., 0.};
    for (std::size_t j = 0; j < 6; ++j)
      f32(block.bytes.data() + 40 + (i * 6 + j) * 4, block.samples[i][j]);
    advance();
  }
  ++sequence_;
  block.compute_cpu = thread_seconds() - cpu_before;
  block.compute_wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - before).count();
  return block;
}
}  // namespace companionbot::follow
