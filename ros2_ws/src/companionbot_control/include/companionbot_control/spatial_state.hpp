#pragma once

// ROS-free passive spatial contract. No command, plant, GT or actuator input.
#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace companionbot::spatial {

struct SpatialSample {
  std::string epoch;
  std::uint64_t sequence{};
  double time_s{}, distance_m{}, velocity_m_s{};
  double roll_rad{}, pitch_rad{}, yaw_rad{};
  double roll_rate_rad_s{}, pitch_rate_rad_s{}, yaw_rate_rad_s{};
  std::array<double, 2> wheel_angles_rad{}, wheel_velocities_rad_s{};
  double imu_sample_time_s{}, imu_available_time_s{}, imu_age_s{};
  bool imu_valid{true}, imu_stale{false}, encoder_valid{true};
  void validate() const;
};

struct PlanarEstimate {
  double x_m{}, y_m{}, yaw_rad{};
  std::array<double, 9> covariance{};  // row major x/y/yaw
};

struct OdometryConfig {
  double velocity_std{.03}, yaw_rate_std{.02};
  double position_rw{.005}, yaw_rw{.002}, max_gap_s{.12};
};

class PlanarOdometry {
 public:
  explicit PlanarOdometry(OdometryConfig config = {});
  PlanarEstimate update(const SpatialSample & sample);
  const std::string & fault() const noexcept {return fault_;}
  const OdometryConfig & config() const noexcept {return config_;}
  const std::optional<SpatialSample> & previous() const noexcept {return previous_;}
 private:
  OdometryConfig config_;
  std::optional<SpatialSample> previous_;
  PlanarEstimate estimate_;
  std::string fault_;
};

struct Geometry {
  std::string geometry_id;
  double wheel_radius_m{}, wheel_separation_m{};
  std::size_t ros_left_encoder_index{}, ros_right_encoder_index{};
};

// Retains the existing Python json.dumps(sort_keys=True) geometry digest.
Geometry load_geometry(const std::string & path);
std::string canonical_geometry_content(const std::string & path);

struct SourceMetadata {
  std::uint32_t contract_version{2};
  std::uint16_t spatial_contract_version{1};
  std::string frame_id{"base_link"}, geometry_id;
  std::string source_id, clock_domain, clock_id, time_semantics{"control_sample"};
  std::string ros_time_mapping_id;
  bool has_sequence{true}, has_source_time{true};
  double source_time_s{}, ros_time_uncertainty_s{};
};

struct SourceConfig {
  std::string geometry_id;
  std::string expected_source_id{"mujoco:spatial"};
  std::string expected_clock_domain{"mujoco_sim"};
  std::string expected_device_mapping_id;
};

class SourceGate {
 public:
  explicit SourceGate(SourceConfig config);
  void validate(const SourceMetadata & meta, const SpatialSample & sample,
                double ros_now_s) const;
  // Commit only after the sample and odometry have both been accepted.
  void commit(const SourceMetadata & meta, const SpatialSample & sample);
  bool simulated() const noexcept {return config_.expected_clock_domain == "mujoco_sim";}
 private:
  SourceConfig config_;
  std::optional<std::array<std::string, 4>> identity_;
};

}  // namespace companionbot::spatial
