#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

namespace companionbot::perception {

struct StereoConfig {
  int width{}, height{};
  double fx{}, fy{}, cx{}, cy{}, baseline_m{};
  std::string calibration_id;
  // Rectified input is explicit: right optical coordinates = R*left + t.
  std::array<double, 9> right_rotation_from_left{1.,0.,0.,0.,1.,0.,0.,0.,1.};
  std::array<double, 3> right_translation_from_left{};
  int min_disparity{0}, num_disparities{64}, block_size{3};
  int p1{72}, p2{288}, disp12_max_diff{1}, pre_filter_cap{0};
  int uniqueness_ratio{10}, speckle_window_size{100}, speckle_range{2};
  int mode{cv::StereoSGBM::MODE_SGBM};
  double lr_max_error_px{1.}, texture_min_gradient_rms{1.};
};
struct StereoFrameStamp {
  std::string source_epoch;
  std::uint64_t sequence{};
  double source_time_s{};
  std::string calibration_id;
  std::string clock_domain{"mujoco_sim"};
};
struct StereoCalibrationAudit {
  std::array<double, 9> r1{}, r2{};
  std::array<double, 12> p1{}, p2{};
  std::array<double, 16> q{};
  std::array<int, 4> left_valid_roi{}, right_valid_roi{};
};

enum StereoRejection : std::uint8_t {
  InvalidLeftMatch = 1, SearchBorder = 2, LowLeftTexture = 4,
  InvalidRightMatch = 8, LeftRightInconsistent = 16, LowRightTexture = 32,
  InvalidMetricDepth = 64
};
struct StereoDiagnostics {
  std::uint64_t total_pixels{}, valid_pixels{}, invalid_left_match{}, search_border{},
      low_left_texture{}, invalid_right_match{}, lr_inconsistent{}, low_right_texture{}, invalid_metric_depth{};
  double valid_fraction{}, wall_s{}, thread_cpu_s{};
  double lock_wait_wall_s{}, grayscale_wall_s{}, matching_wall_s{}, quality_wall_s{};
};
// Each result owns its output allocations, independent of processor scratch.
// Binding exports separate owned NumPy copies, never the reusable cv::Mat views.
struct StereoDepthResult {
  StereoFrameStamp stamp;
  cv::Mat depth_m, disparity_px, lr_error_px;
  cv::Mat valid_mask, rejection_mask, texture_mask, lr_consistency_mask;
  StereoDiagnostics diagnostics;
};
struct StereoProcessorSnapshot {
  std::uint64_t accepted_frames{}, rejected_inputs{}, last_sequence{};
  std::optional<std::string> source_epoch;
  std::optional<std::string> clock_domain;
};

// Mature OpenCV StereoSGBM matching with explicit rectified calibration,
// independent reverse matching and conservative quality masks. It accepts no
// actor pose, renderer depth, reference command, ROS clock or actuator input.
class StereoDepthProcessor {
 public:
  explicit StereoDepthProcessor(StereoConfig config);
  StereoDepthResult process(const cv::Mat &left_bgr, const cv::Mat &right_bgr,
                           const StereoFrameStamp &left, const StereoFrameStamp &right);
  StereoProcessorSnapshot snapshot() const;
  const StereoConfig &config() const { return config_; }
  StereoCalibrationAudit calibration_audit() const { return calibration_audit_; }
  static std::string opencv_version();
 private:
  void validate_pair(const cv::Mat &, const cv::Mat &, const StereoFrameStamp &, const StereoFrameStamp &);
  void texture_energy(const cv::Mat &gray, cv::Mat &gradient, cv::Mat &energy, cv::Mat &mean_energy);
  StereoConfig config_;
  StereoCalibrationAudit calibration_audit_;
  cv::Ptr<cv::StereoSGBM> left_matcher_, right_matcher_;
  cv::Mat gray_left_, gray_right_, left_fixed_, right_fixed_;
  cv::Mat gradient_left_, gradient_right_, energy_left_, energy_right_, texture_left_, texture_right_;
  cv::Rect valid_left_, valid_right_;
  mutable std::mutex mutex_;
  std::optional<std::string> epoch_;
  std::optional<std::string> clock_domain_;
  std::optional<std::uint64_t> last_sequence_;
  double last_source_time_{};
  std::uint64_t accepted_frames_{}, rejected_inputs_{};
};
}  // namespace companionbot::perception
