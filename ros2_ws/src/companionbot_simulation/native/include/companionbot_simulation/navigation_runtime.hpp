#pragma once

#include "companionbot/control.h"
#include "companionbot_follow/full_reference.hpp"
#include "companionbot_follow/follow_algorithms.hpp"
#include "companionbot_follow/motion_authority.hpp"
#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace companionbot::simulation {

struct FollowClearanceConfig {
  double radius_m{}, reaction_time_s{};
  std::uint32_t maximum_projection_steps{};
};

struct RuntimeOutput {
  cb_mcu_output command{};
  cb_reference next_reference{};
  double block_start_time_s{}, intent_source_time_s{};
};

// One in-process Pi scheduler and software MCU endpoint. Simulation transport
// uses source ticks; task freshness uses the caller's local monotonic receipt
// clock. The same heap-free C99 MCU core owns all control safety transitions.
// This class neither estimates plant truth nor owns actual actuator feedback.
class NavigationRuntime {
 public:
  NavigationRuntime(std::shared_ptr<follow::MotionAuthority>, follow::ProducerConfig,
                    cb_mcu_config, double transport_latency_s = 0.);
  NavigationRuntime(std::shared_ptr<follow::MotionAuthority>, follow::TwoPathConfig,
                    cb_mcu_config, double transport_latency_s = 0.,
                    follow::FullPlanSolver = {}, std::shared_ptr<follow::FollowController> = {},
                    std::optional<FollowClearanceConfig> = std::nullopt);
  RuntimeOutput step(std::uint32_t tick, double receipt_now_s, double local_velocity,
                     double local_pitch, double local_pitch_rate);
  cb_reference peek() const;
  std::string report_json() const;
  std::string reason_name(std::uint32_t code) const;
  static std::string state_name(std::uint32_t code);
  std::uint32_t epoch() const;
  std::uint32_t safety_state() const;
  std::uint32_t reason() const;

 private:
  enum class Kind { Reference, Heartbeat, Yaw, Safety, Resume, RobotState, SafetyStatus };
  struct Packet {
    Kind kind{Kind::Heartbeat};
    bool to_mcu{true};
    double ready{}, source_time{}, position{}, velocity{}, yaw{};
    std::uint32_t sequence{}, epoch{}, reason{}, state{}, pi_lost{}, start_tick{};
    std::array<std::uint8_t, 640> block{};
  };
  struct Log {
    explicit Log(std::size_t capacity) : capacity(capacity) { rows.reserve(capacity); }
    void push(nlohmann::json row);
    nlohmann::json export_rows() const;
    const std::size_t capacity;
    std::vector<nlohmann::json> rows;
    std::uint64_t total{}, overwritten{};
  };
  static constexpr std::size_t kQueueCapacity = 32, kReasonCapacity = 256;
  static constexpr std::size_t kStepCapacity = 131072;
  static const char *channel_name(Kind);
  std::string reason_name_unlocked(std::uint32_t) const;
  std::uint32_t external_reason(const std::string &);
  void enqueue(Packet, double sent_time, bool startup = false, double compute_time = 0.);
  void deliver(bool to_mcu, double source_now);
  void receive_mcu(const Packet &, double source_now);
  void send_status(double source_now);
  void produce(std::uint32_t start_tick, double source_now, const follow::MotionIntent &,
               bool observe = true, bool startup = false, bool resume_first = false);
  follow::MotionIntent after_pi(std::uint32_t tick, double source_now, double receipt_now);
  void log_takeover(double source_now, std::uint32_t previous_epoch,
                    bool explicit_command, const cb_reference &previous,
                    double issued_time = 0.);
  void capacity_fault(double source_now);
  void reference_fault(double source_now);
  void select_producer_policy(const std::string &mode);
  void update_follow_braking(double source_now);

  mutable std::mutex mutex_;
  std::shared_ptr<follow::MotionAuthority> authority_;
  follow::TwoPathProducer producer_;
  std::optional<follow::TwoPathConfig> follow_config_;
  std::shared_ptr<follow::FollowController> follow_controller_;
  std::optional<FollowClearanceConfig> clearance_config_;
  std::string producer_mode_{"GUIDE_TO"};
  cb_mcu mcu_{};
  double latency_{};
  std::array<Packet, kQueueCapacity> queue_{};
  std::size_t queue_size_{}, queue_high_water_{};
  std::array<std::string, kReasonCapacity> external_reasons_{};
  std::size_t reason_count_{};
  bool reason_capacity_faulted_{};
  std::optional<Packet> latest_robot_, latest_status_;
  bool production_stopped_{}, started_{};
  std::optional<std::uint32_t> resume_followup_;
  std::optional<std::uint64_t> last_resume_generation_;
  std::uint32_t heartbeat_sequence_{}, yaw_sequence_{}, safety_sequence_{}, resume_sequence_{};
  std::uint32_t robot_sequence_{}, status_sequence_{}, last_tick_{};
  double last_receipt_{}, current_receipt_{};
  double local_velocity_{}, local_pitch_{}, local_pitch_rate_{};
  std::uint32_t published_state_{}, published_epoch_{}, published_reason_{}, published_pi_lost_{};
  std::uint64_t queue_capacity_rejections_{}, references_dropped_{}, step_count_{};
  double step_wall_total_{}, step_cpu_total_{}, step_wall_max_{}, step_cpu_max_{};
  std::array<double, kStepCapacity> step_wall_{}, step_cpu_{};
  Log safety_events_{2048}, receiver_events_{2048}, producer_events_{16384};
  Log packet_events_{2048}, yaw_events_{2048}, transport_events_{65536};
  Log lifecycle_epochs_{256};
  Log braking_events_{16384};
};
}  // namespace companionbot::simulation
