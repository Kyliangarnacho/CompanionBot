#include "companionbot_control/spatial_state.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace companionbot::spatial;
namespace {
void check(bool valid, const char * message) {if (!valid) {throw std::runtime_error(message);}}
void rejects(const std::function<void()> & operation, const std::string & reason) {
  try {operation();} catch (const std::invalid_argument & error) {
    check(std::string(error.what()).find(reason) != std::string::npos, error.what());
    return;
  }
  throw std::runtime_error("expected rejection:" + reason);
}
SpatialSample sample(double time = .02, std::uint64_t sequence = 10) {
  SpatialSample result;
  result.epoch = "test-epoch";
  result.sequence = sequence;
  result.time_s = result.imu_available_time_s = time;
  result.imu_sample_time_s = time - .001;
  result.imu_age_s = .001;
  return result;
}
SourceMetadata metadata(const SpatialSample & source) {
  SourceMetadata result;
  result.geometry_id = "test-geometry";
  result.source_id = "mujoco:spatial";
  result.clock_domain = "mujoco_sim";
  result.clock_id = "sim:" + source.epoch;
  result.ros_time_mapping_id = "identity:" + result.clock_id;
  result.source_time_s = source.time_s;
  return result;
}
void geometry_circle_and_covariance() {
  PlanarOdometry odom;
  const auto first = odom.update(sample());
  check(first.x_m == 0 && first.y_m == 0 && first.yaw_rad == 0, "first source must define origin");
  PlanarEstimate result;
  for (std::uint64_t index = 1; index <= 100; ++index) {
    auto input = sample(.02 + .02*index, 10 + index*10);
    input.distance_m = input.yaw_rad = .01*index;
    input.velocity_m_s = input.yaw_rate_rad_s = .5;
    result = odom.update(input);
  }
  check(std::abs(result.x_m - std::sin(1.)) < 1e-12, "circle x");
  check(std::abs(result.y_m - (1. - std::cos(1.))) < 1e-12, "circle y");
  check(std::abs(result.yaw_rad - 1.) < 1e-12, "circle yaw");
  for (std::size_t i = 0; i < 3; ++i) {check(result.covariance[3*i + i] > 0, "covariance growth");}
}
void faults_and_latch() {
  const auto bad_input = [](const std::function<void(SpatialSample &)> & mutate, const std::string & reason) {
    PlanarOdometry odom;
    odom.update(sample());
    auto invalid = sample(.04, 20);
    mutate(invalid);
    rejects([&] {odom.update(invalid);}, reason);
    check(odom.previous()->sequence == 10, "fault must not advance watermark");
    rejects([&] {odom.update(sample(.06, 30));}, "odometry_fault_latched:");
  };
  bad_input([](auto & value) {value.imu_stale = true;}, "sensor_invalid_or_stale");
  bad_input([](auto & value) {value.imu_valid = false;}, "sensor_invalid_or_stale");
  bad_input([](auto & value) {value.encoder_valid = false;}, "sensor_invalid_or_stale");
  bad_input([](auto & value) {value.distance_m = std::numeric_limits<double>::quiet_NaN();}, "nonfinite_or_malformed");
  bad_input([](auto & value) {value.imu_available_time_s = .05;}, "inconsistent_sensor_source_time");
  bad_input([](auto & value) {value.imu_age_s = .006;}, "inconsistent_sensor_source_time");
  bad_input([](auto & value) {value.pitch_rad = 1.;}, "outside_planar_tilt_envelope");
  bad_input([](auto & value) {value.roll_rad = 1.;}, "outside_planar_tilt_envelope");
  bad_input([](auto & value) {value.sequence = 10;}, "sequence_time_reversal_or_gap");
  bad_input([](auto & value) {value.epoch = "reboot";}, "source_epoch_changed_restart_required");
  bad_input([](auto & value) {value.time_s = .3; value.imu_sample_time_s = .299; value.imu_available_time_s = .3;}, "sequence_time_reversal_or_gap");
  PlanarOdometry odom;
  auto before = sample();
  before.distance_m = -1e308;
  odom.update(before);
  auto after = sample(.04, 20);
  after.distance_m = 1e308;
  rejects([&] {odom.update(after);}, "nonfinite_derived_odometry");
  rejects([&] {odom.update(sample(.06, 30));}, "odometry_fault_latched:");
}
void source_identity_mapping_time() {
  SourceGate gate(SourceConfig{"test-geometry", "mujoco:spatial", "mujoco_sim", ""});
  const auto input = sample();
  auto meta = metadata(input);
  gate.validate(meta, input, .02);
  gate.commit(meta, input);
  auto invalid = meta;
  invalid.ros_time_uncertainty_s = .003;
  rejects([&] {gate.validate(invalid, input, .02);}, "missing_or_uncertain_source_time_mapping");
  invalid = meta;
  invalid.geometry_id = "wrong";
  rejects([&] {gate.validate(invalid, input, .02);}, "state_schema_source_frame_or_geometry");
  invalid = meta;
  invalid.clock_id = "reboot";
  rejects([&] {gate.validate(invalid, input, .02);}, "source_or_time_mapping_changed_restart_required");
  invalid = meta;
  invalid.source_time_s = 1.;
  rejects([&] {gate.validate(invalid, input, .02);}, "invalid_simulation_time_mapping");
  rejects([&] {gate.validate(meta, input, .4);}, "state_source_time_outside_ros_window");
  SourceGate device(SourceConfig{"test-geometry", "device", "device_clock", "measured-map-1"});
  meta.source_id = "device";
  meta.clock_domain = "device_clock";
  meta.clock_id = "mcu-clock-1";
  meta.ros_time_mapping_id = "measured-map-1";
  meta.ros_time_uncertainty_s = .001;
  device.validate(meta, input, .02);
  meta.ros_time_mapping_id = "unapproved";
  rejects([&] {device.validate(meta, input, .02);}, "unapproved_device_time_mapping");
}
}  // namespace

int main() {
  try {
    geometry_circle_and_covariance();
    faults_and_latch();
    source_identity_mapping_time();
    std::cout << "spatial native invariants passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
