#pragma once
#include <array>
#include <cstdint>
#include <vector>
#include <ruckig/ruckig.hpp>

namespace companionbot::follow {
struct ProducerConfig {
  double dt{0.002}, max_velocity{0.6}, max_acceleration{0.6}, max_jerk{1.0};
  double full_delta_velocity{0.35};
  std::uint32_t block_samples{25};
};
struct ProducedBlock {
  std::array<std::uint8_t, 640> bytes{};
  std::array<std::array<double, 6>, 25> samples{};
  std::uint32_t epoch{}, sequence{}, start_tick{};
  double generated_time{}, compute_wall{}, compute_cpu{}, duration{};
  double source_time{}, target_velocity{}, yaw{};
  bool replanned{};
};

// Owns the existing navigation LIGHTWEIGHT policy: immediate interruptible
// retargets, zero hidden reference/feedforward, original velocity lifecycle.
// FULL policies are deliberately outside this class and rejected explicitly.
// The physics/MCU clock and the caller's receipt clock remain separate.
class LightweightProducer {
 public:
  explicit LightweightProducer(ProducerConfig config, std::uint32_t epoch = 0);
  void reset(double position, double velocity, std::uint32_t epoch);
  ProducedBlock produce(std::uint32_t start_tick, double generated_time,
                        double source_time, double velocity, double yaw,
                        bool observe_command = true);
  std::array<double, 3> next_reference() const { return current_; }
  std::uint32_t epoch() const { return epoch_; }
  std::uint32_t next_sequence() const { return sequence_; }
  const ProducerConfig &config() const { return config_; }
 private:
  void plan(double target);
  void advance();
  ProducerConfig config_;
  ruckig::Ruckig<1> ruckig_;
  ruckig::Trajectory<1> trajectory_;
  std::vector<std::array<double, 3>> profile_;
  std::array<double, 3> current_{};
  std::size_t index_{};
  std::uint32_t epoch_{}, sequence_{};
  double accepted_{}, last_source_{}, last_yaw_{}, duration_{};
  bool transient_{}, observed_{};
};
}  // namespace companionbot::follow
