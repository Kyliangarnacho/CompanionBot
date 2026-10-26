#include "companionbot_control/spatial_state.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

namespace companionbot::spatial {
namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
bool close(double a, double b, double absolute = 1e-9) {
  return std::abs(a - b) <= std::max(absolute, 1e-9 * std::max(std::abs(a), std::abs(b)));
}
void require(bool valid, const char * reason) {
  if (!valid) {throw std::invalid_argument(reason);}
}
double wrap(double angle) {
  require(std::isfinite(angle), "angle must be finite");
  double result = std::fmod(angle + pi, 2. * pi);
  if (result < 0) {result += 2. * pi;}
  return result - pi;
}

// Both languages use shortest round-trip IEEE754 digits. Python's repr has
// explicit fixed/scientific cutovers and a two-digit minimum exponent. Keep
// these rules here solely to read the existing canonical source hash.
std::string python_float(double value) {
  require(std::isfinite(value), "invalid spatial geometry");
  if (value == 0.) {return std::signbit(value) ? "-0.0" : "0.0";}
  char buffer[128];
  const auto converted = std::to_chars(buffer, buffer + sizeof(buffer), value, std::chars_format::general);
  if (converted.ec != std::errc{}) {throw std::runtime_error("geometry_float_conversion");}
  std::string text(buffer, converted.ptr);
  const bool negative = text.front() == '-';
  if (negative) {text.erase(0, 1);}
  int exponent = 0;
  const auto e = text.find('e');
  if (e != std::string::npos) {
    exponent = std::stoi(text.substr(e + 1));
    text.resize(e);
  }
  const auto dot = text.find('.');
  int decimal = static_cast<int>(dot == std::string::npos ? text.size() : dot) + exponent;
  if (dot != std::string::npos) {text.erase(dot, 1);}
  while (text.size() > 1 && text.front() == '0') {text.erase(0, 1); --decimal;}
  while (text.size() > 1 && text.back() == '0') {text.pop_back();}
  const int scientific = decimal - 1;
  std::string result;
  if (scientific < -4 || scientific >= 16) {
    result = text.substr(0, 1);
    if (text.size() > 1) {result += "." + text.substr(1);}
    result += scientific < 0 ? "e-" : "e+";
    const auto digits = std::to_string(std::abs(scientific));
    if (digits.size() < 2) {result += '0';}
    result += digits;
  } else if (decimal <= 0) {
    result = "0." + std::string(static_cast<std::size_t>(-decimal), '0') + text;
  } else if (static_cast<std::size_t>(decimal) >= text.size()) {
    result = text + std::string(static_cast<std::size_t>(decimal) - text.size(), '0') + ".0";
  } else {
    result = text.substr(0, decimal) + "." + text.substr(decimal);
  }
  return negative ? "-" + result : result;
}

std::string canonical(const nlohmann::json & value) {
  // JSON itself has no binary value. The SAX adapter uses it only for lexical
  // integers beyond uint64, retaining Python's arbitrary-precision integer
  // normalization without converting those metadata values to a double.
  if (value.is_binary()) {
    const auto & raw = value.get_binary();
    return std::string(raw.begin(), raw.end());
  }
  if (value.is_number_float()) {return python_float(value.get<double>());}
  if (value.is_array() || value.is_object()) {
    std::string result = value.is_array() ? "[" : "{";
    bool first = true;
    for (auto it = value.begin(); it != value.end(); ++it) {
      if (!first) {result += ", ";}
      first = false;
      if (value.is_object()) {result += nlohmann::json(it.key()).dump(-1, ' ', true) + ": ";}
      result += canonical(it.value());
    }
    result += value.is_array() ? "]" : "}";
    return result;
  }
  return value.dump(-1, ' ', true);
}

class GeometrySax final : public nlohmann::json_sax<nlohmann::json> {
 public:
  nlohmann::json document;
  bool null() override {add(nullptr); return true;}
  bool boolean(bool value) override {add(value); return true;}
  bool number_integer(number_integer_t value) override {add(value); return true;}
  bool number_unsigned(number_unsigned_t value) override {add(value); return true;}
  bool number_float(number_float_t value, const string_t & token) override {
    if (token.find_first_of(".eE") == std::string::npos) {
      add(nlohmann::json::binary(std::vector<std::uint8_t>(token.begin(), token.end())));
    } else {add(value);}
    return true;
  }
  bool string(string_t & value) override {add(value); return true;}
  bool binary(binary_t &) override {throw std::invalid_argument("binary input is not geometry JSON");}
  bool start_object(std::size_t) override {stack_.push_back(add(nlohmann::json::object())); return true;}
  bool key(string_t & value) override {key_ = value; return true;}
  bool end_object() override {stack_.pop_back(); return true;}
  bool start_array(std::size_t) override {stack_.push_back(add(nlohmann::json::array())); return true;}
  bool end_array() override {stack_.pop_back(); return true;}
  bool parse_error(std::size_t, const std::string &, const nlohmann::detail::exception & error) override {
    throw std::invalid_argument(std::string("invalid spatial geometry JSON:") + error.what());
  }
 private:
  nlohmann::json * add(nlohmann::json value) {
    if (stack_.empty()) {document = std::move(value); return &document;}
    auto & parent = *stack_.back();
    if (parent.is_array()) {parent.push_back(std::move(value)); return &parent.back();}
    parent[key_] = std::move(value);
    return &parent[key_];
  }
  std::vector<nlohmann::json *> stack_;
  std::string key_;
};

nlohmann::json read_geometry(const std::string & path) {
  std::ifstream stream(path);
  if (!stream) {throw std::invalid_argument("cannot read spatial geometry:" + path);}
  GeometrySax parser;
  require(nlohmann::json::sax_parse(stream, &parser), "invalid spatial geometry JSON");
  auto raw = std::move(parser.document);
  require(raw.is_object() && raw.contains("geometry_id"), "invalid spatial geometry");
  return raw;
}

std::string digest(const std::string & content) {
  std::array<unsigned char, EVP_MAX_MD_SIZE> bytes{};
  unsigned int length{};
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1 ||
    EVP_DigestUpdate(context.get(), content.data(), content.size()) != 1 ||
    EVP_DigestFinal_ex(context.get(), bytes.data(), &length) != 1)
  {throw std::runtime_error("geometry_sha256_failed");}
  std::ostringstream out;
  for (unsigned int i = 0; i < length; ++i) {
    out << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(bytes[i]);
  }
  return out.str();
}
}  // namespace

void SpatialSample::validate() const {
  require(!epoch.empty() && sequence > 0, "invalid_source_identity");
  const std::array<double, 16> numbers{time_s, distance_m, velocity_m_s, roll_rad, pitch_rad,
    yaw_rad, roll_rate_rad_s, pitch_rate_rad_s, yaw_rate_rad_s,
    wheel_angles_rad[0], wheel_angles_rad[1], wheel_velocities_rad_s[0], wheel_velocities_rad_s[1],
    imu_sample_time_s, imu_available_time_s, imu_age_s};
  require(std::all_of(numbers.begin(), numbers.end(), [](double x) {return std::isfinite(x);}),
    "nonfinite_or_malformed_state");
  require(imu_valid && !imu_stale && encoder_valid, "sensor_invalid_or_stale");
  require(time_s >= 0 && imu_sample_time_s <= imu_available_time_s + 1e-9 &&
    imu_available_time_s <= time_s + 1e-9 && imu_age_s >= -1e-9 && imu_age_s <= .005 + 1e-9 &&
    close(time_s - imu_sample_time_s, imu_age_s), "inconsistent_sensor_source_time");
  require(std::abs(pitch_rad) < pi / 4 && std::abs(roll_rad) < pi / 6, "outside_planar_tilt_envelope");
}

PlanarOdometry::PlanarOdometry(OdometryConfig config) : config_(config) {
  for (const double value : {config.velocity_std, config.yaw_rate_std, config.position_rw,
    config.yaw_rw, config.max_gap_s})
  {require(std::isfinite(value) && value > 0, "invalid_odometry_uncertainty");}
}

PlanarEstimate PlanarOdometry::update(const SpatialSample & sample) {
  if (!fault_.empty()) {throw std::invalid_argument("odometry_fault_latched:" + fault_);}
  try {
    sample.validate();
    if (previous_) {
      const auto & old = *previous_;
      const double dt = sample.time_s - old.time_s;
      require(old.epoch == sample.epoch, "source_epoch_changed_restart_required");
      require(sample.sequence > old.sequence && dt > 0 && dt <= config_.max_gap_s,
        "sequence_time_reversal_or_gap");
      const double ds = sample.distance_m - old.distance_m;
      const double dyaw = wrap(sample.yaw_rad - old.yaw_rad);
      const double middle = estimate_.yaw_rad + dyaw / 2.;
      // np.sinc(dyaw / (2*pi)): exact zero limit; no approximation threshold.
      const double half = dyaw / 2.;
      const double travel = ds * (half == 0 ? 1. : std::sin(half) / half);
      const double c = std::cos(middle), s = std::sin(middle);
      estimate_.x_m += travel * c;
      estimate_.y_m += travel * s;
      const std::array<double, 9> f{1, 0, -travel*s, 0, 1, travel*c, 0, 0, 1};
      const std::array<double, 6> j{c, -.5*travel*s, s, .5*travel*c, 0, 1};
      const std::array<double, 2> noise{
        std::pow(config_.velocity_std, 2) * dt * dt + std::pow(config_.position_rw, 2) * dt,
        std::pow(config_.yaw_rate_std, 2) * dt * dt + std::pow(config_.yaw_rw, 2) * dt};
      std::array<double, 9> fp{}, propagated{};
      for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
          for (std::size_t k = 0; k < 3; ++k) {fp[3*row + col] += f[3*row + k] * estimate_.covariance[3*k + col];}
        }
      }
      for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
          for (std::size_t k = 0; k < 3; ++k) {propagated[3*row + col] += fp[3*row + k] * f[3*col + k];}
          // j @ diag(noise) @ j.T, evaluated without dynamic allocation.
          double process = 0.;
          for (std::size_t k = 0; k < 2; ++k) {process += (j[2*row + k] * noise[k]) * j[2*col + k];}
          propagated[3*row + col] += process;
        }
      }
      estimate_.covariance = propagated;
      estimate_.yaw_rad += dyaw;
    }
    require(std::isfinite(estimate_.x_m) && std::isfinite(estimate_.y_m) && std::isfinite(estimate_.yaw_rad) &&
      std::all_of(estimate_.covariance.begin(), estimate_.covariance.end(), [](double x) {return std::isfinite(x);}),
      "nonfinite_derived_odometry");
    previous_ = sample;
  } catch (const std::invalid_argument & error) {
    fault_ = error.what();
    throw;
  }
  return estimate_;
}

std::string canonical_geometry_content(const std::string & path) {
  auto raw = read_geometry(path);
  raw.erase("geometry_id");
  return canonical(raw);
}

Geometry load_geometry(const std::string & path) {
  auto raw = read_geometry(path);
  const auto id = raw.at("geometry_id").get<std::string>();
  raw.erase("geometry_id");
  require(id == digest(canonical(raw)), "spatial geometry digest mismatch");
  require(raw.at("schema_version") == 1 && raw.at("mj_to_ros").is_array() &&
    raw.at("mj_to_ros").size() == 3, "invalid spatial geometry");
  std::array<double, 9> matrix{};
  for (std::size_t i = 0; i < 3; ++i) {
    require(raw.at("mj_to_ros").at(i).is_array() && raw.at("mj_to_ros").at(i).size() == 3,
      "invalid spatial geometry");
    for (std::size_t j = 0; j < 3; ++j) {
      matrix[3*i + j] = raw.at("mj_to_ros").at(i).at(j).get<double>();
      require(std::isfinite(matrix[3*i + j]), "invalid spatial geometry");
    }
  }
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      double value = 0.;
      for (std::size_t k = 0; k < 3; ++k) {value += matrix[3*k + row] * matrix[3*k + col];}
      const double expected = row == col ? 1. : 0.;
      require(std::abs(value - expected) <= 1e-8 + 1e-5 * std::abs(expected), "invalid spatial geometry");
    }
  }
  const double determinant = matrix[0]*(matrix[4]*matrix[8] - matrix[5]*matrix[7]) -
    matrix[1]*(matrix[3]*matrix[8] - matrix[5]*matrix[6]) +
    matrix[2]*(matrix[3]*matrix[7] - matrix[4]*matrix[6]);
  require(std::abs(determinant - 1.) <= 1e-8 + 1e-5, "invalid spatial geometry");
  Geometry result{id, raw.at("wheel_radius_m").get<double>(), raw.at("wheel_separation_m").get<double>(),
    raw.at("ros_left_encoder_index").get<std::size_t>(), raw.at("ros_right_encoder_index").get<std::size_t>()};
  require(std::isfinite(result.wheel_radius_m) && std::isfinite(result.wheel_separation_m) &&
    result.wheel_radius_m > 0 && result.wheel_separation_m > 0 &&
    ((result.ros_left_encoder_index == 0 && result.ros_right_encoder_index == 1) ||
    (result.ros_left_encoder_index == 1 && result.ros_right_encoder_index == 0)), "invalid spatial geometry");
  return result;
}

SourceGate::SourceGate(SourceConfig config) : config_(std::move(config)) {
  require(simulated() || !config_.expected_device_mapping_id.empty(),
    "device spatial data requires an explicit measured ROS-time mapping id");
}

void SourceGate::validate(const SourceMetadata & meta, const SpatialSample & sample, double now) const {
  require(meta.spatial_contract_version == 1 && meta.contract_version == 2 &&
    meta.frame_id == "base_link" && meta.geometry_id == config_.geometry_id &&
    meta.source_id == config_.expected_source_id && meta.clock_domain == config_.expected_clock_domain &&
    !sample.epoch.empty() && !meta.clock_id.empty() && meta.has_sequence && meta.has_source_time &&
    std::isfinite(meta.source_time_s) && meta.time_semantics == "control_sample",
    "state_schema_source_frame_or_geometry");
  const std::array<std::string, 4> identity{meta.source_id, sample.epoch, meta.clock_id, meta.ros_time_mapping_id};
  require(!identity_ || *identity_ == identity, "source_or_time_mapping_changed_restart_required");
  require(!meta.ros_time_mapping_id.empty() && std::isfinite(meta.ros_time_uncertainty_s) &&
    meta.ros_time_uncertainty_s >= 0 && meta.ros_time_uncertainty_s <= .002,
    "missing_or_uncertain_source_time_mapping");
  if (simulated()) {
    require(meta.ros_time_mapping_id == "identity:" + meta.clock_id && meta.clock_id == "sim:" + sample.epoch &&
      meta.ros_time_uncertainty_s == 0 && close(sample.time_s, meta.source_time_s), "invalid_simulation_time_mapping");
  } else {
    require(meta.ros_time_mapping_id == config_.expected_device_mapping_id, "unapproved_device_time_mapping");
  }
  require(!(sample.time_s > now + .12 || (now > 0 && now - sample.time_s > .12)),
    "state_source_time_outside_ros_window");
  sample.validate();
}

void SourceGate::commit(const SourceMetadata & meta, const SpatialSample & sample) {
  identity_ = std::array<std::string, 4>{meta.source_id, sample.epoch, meta.clock_id, meta.ros_time_mapping_id};
}

}  // namespace companionbot::spatial
