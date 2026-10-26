// Diagnostic replay/benchmark only. Not a robot runtime or motion entry point.
#include "companionbot_control/spatial_state.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

using companionbot::spatial::SpatialSample;
using companionbot::spatial::PlanarOdometry;
namespace {
SpatialSample decode(const nlohmann::json & data) {
  SpatialSample result;
  result.epoch = data.at("epoch").get<std::string>();
  result.sequence = data.at("sequence").get<std::uint64_t>();
  result.time_s = data.at("time_s");
  result.distance_m = data.at("distance_m");
  result.velocity_m_s = data.at("velocity_m_s");
  result.roll_rad = data.at("roll_rad");
  result.pitch_rad = data.at("pitch_rad");
  result.yaw_rad = data.at("yaw_rad");
  result.roll_rate_rad_s = data.at("roll_rate_rad_s");
  result.pitch_rate_rad_s = data.at("pitch_rate_rad_s");
  result.yaw_rate_rad_s = data.at("yaw_rate_rad_s");
  result.wheel_angles_rad = data.at("wheel_angles_rad").get<std::array<double, 2>>();
  result.wheel_velocities_rad_s = data.at("wheel_velocities_rad_s").get<std::array<double, 2>>();
  result.imu_sample_time_s = data.at("imu_sample_time_s");
  result.imu_available_time_s = data.at("imu_available_time_s");
  result.imu_age_s = data.at("imu_age_s");
  result.imu_valid = data.at("imu_valid");
  result.imu_stale = data.at("imu_stale");
  result.encoder_valid = data.at("encoder_valid");
  return result;
}
std::uint64_t cpu_ns() {
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value)) {throw std::runtime_error("thread_cpu_clock_unavailable");}
  return static_cast<std::uint64_t>(value.tv_sec)*1000000000u + value.tv_nsec;
}
}

int main(int argc, char ** argv) {
  try {
    if (argc == 3 && std::string(argv[1]) == "--geometry") {
      const auto geometry = companionbot::spatial::load_geometry(argv[2]);
      std::cout << nlohmann::json{{"geometry_id", geometry.geometry_id},
        {"canonical", companionbot::spatial::canonical_geometry_content(argv[2])}}.dump() << '\n';
      return 0;
    }
    if (argc != 3 || (std::string(argv[1]) != "--replay" && std::string(argv[1]) != "--benchmark")) {
      throw std::invalid_argument("usage: spatial_replay --replay|--benchmark input.jsonl OR --geometry config.json");
    }
    std::ifstream stream(argv[2]);
    if (!stream) {throw std::invalid_argument("input unavailable");}
    std::vector<SpatialSample> inputs;
    std::string line;
    while (std::getline(stream, line)) {if (!line.empty()) {inputs.push_back(decode(nlohmann::json::parse(line)));}}
    if (inputs.empty()) {throw std::invalid_argument("empty spatial input");}
    PlanarOdometry odom;
    if (std::string(argv[1]) == "--replay") {
      for (const auto & sample : inputs) {
        const auto estimate = odom.update(sample);
        std::cout << nlohmann::json{{"x_m", estimate.x_m}, {"y_m", estimate.y_m}, {"yaw_rad", estimate.yaw_rad},
          {"covariance", {{estimate.covariance[0], estimate.covariance[1], estimate.covariance[2]},
          {estimate.covariance[3], estimate.covariance[4], estimate.covariance[5]},
          {estimate.covariance[6], estimate.covariance[7], estimate.covariance[8]}}}}.dump() << '\n';
      }
    } else {
      std::vector<double> wall, cpu;
      wall.reserve(inputs.size()*40);
      cpu.reserve(inputs.size()*40);
      double checksum{};
      for (int repeat = -2; repeat < 40; ++repeat) {
        PlanarOdometry current;
        for (const auto & sample : inputs) {
          const auto start = std::chrono::steady_clock::now();
          const auto thread_start = cpu_ns();
          const auto result = current.update(sample);
          const auto thread_end = cpu_ns();
          const auto end = std::chrono::steady_clock::now();
          checksum += result.x_m + result.y_m + result.yaw_rad + result.covariance[0];
          if (repeat >= 0) {
            wall.push_back(std::chrono::duration<double, std::nano>(end - start).count());
            cpu.push_back(static_cast<double>(thread_end - thread_start));
          }
        }
      }
      std::cout << nlohmann::json{{"wall_ns", wall}, {"thread_cpu_ns", cpu}, {"checksum", checksum},
        {"repeats", 40}, {"warmup_repeats", 2}, {"input_samples", inputs.size()},
        {"timed_scope", "PlanarOdometry.update including sample gate; input parsing/output outside timers"}}.dump() << '\n';
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
