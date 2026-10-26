#include "companionbot_perception/stereo_depth.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <time.h>

namespace companionbot::perception {
namespace {
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double>(b-a).count(); }
double thread_seconds() {
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value)) throw std::runtime_error("stereo_thread_cpu_clock_unavailable");
  return static_cast<double>(value.tv_sec)+static_cast<double>(value.tv_nsec)*1e-9;
}
bool positive(double value) { return std::isfinite(value) && value > 0.; }
}

std::string StereoDepthProcessor::opencv_version() { return CV_VERSION; }
StereoDepthProcessor::StereoDepthProcessor(StereoConfig config) : config_(std::move(config)) {
  const auto &c = config_;
  if (cv::getVersionString() != CV_VERSION)
    throw std::runtime_error("opencv_compiled_runtime_version_mismatch");
  if (c.width < 32 || c.height < 16 || c.width > 1280 || c.height > 720 || !positive(c.fx) || !positive(c.fy) ||
      !std::isfinite(c.cx) || !std::isfinite(c.cy) || c.cx < 0. || c.cx >= c.width || c.cy < 0. || c.cy >= c.height ||
      !positive(c.baseline_m) || c.baseline_m > 1. || c.calibration_id.empty() || c.calibration_id.size() > 128 ||
      c.min_disparity != 0 || c.num_disparities <= 0 || c.num_disparities % 16 || c.num_disparities >= c.width ||
      c.num_disparities > 256 || c.block_size < 1 || c.block_size > 11 || c.block_size % 2 != 1 ||
      c.p1 <= 0 || c.p2 <= c.p1 || c.p2 >= 32767 || c.disp12_max_diff < 1 || c.disp12_max_diff > c.num_disparities ||
      c.pre_filter_cap < 0 || c.pre_filter_cap > 63 || c.uniqueness_ratio < 0 || c.uniqueness_ratio > 100 ||
      c.speckle_window_size < 0 || c.speckle_window_size > c.width*c.height ||
      c.speckle_range < 0 || c.speckle_range > c.num_disparities ||
      !positive(c.lr_max_error_px) || !positive(c.texture_min_gradient_rms) ||
      (c.mode != cv::StereoSGBM::MODE_SGBM && c.mode != cv::StereoSGBM::MODE_SGBM_3WAY))
    throw std::invalid_argument("invalid_stereo_processor_configuration");
  for (std::size_t i = 0; i < 9; ++i) {
    const double expected = i % 4 == 0 ? 1. : 0.;
    if (!std::isfinite(c.right_rotation_from_left[i]) || std::abs(c.right_rotation_from_left[i]-expected) > 1e-10)
      throw std::invalid_argument("stereo_input_requires_parallel_rectified_optical_axes");
  }
  for (std::size_t i = 0; i < 3; ++i) {
    const double expected = i == 0 ? -c.baseline_m : 0.;
    if (!std::isfinite(c.right_translation_from_left[i]) || std::abs(c.right_translation_from_left[i]-expected) > 1e-10)
      throw std::invalid_argument("stereo_input_baseline_direction_or_scale_mismatch");
  }
  const cv::Matx33d k(c.fx,0.,c.cx,0.,c.fy,c.cy,0.,0.,1.);
  const cv::Matx33d rotation(c.right_rotation_from_left.data());
  const cv::Vec3d translation(c.right_translation_from_left.data());
  cv::Mat r1,r2,p1,p2,q;
  cv::Rect left_roi,right_roi;
  cv::stereoRectify(k,cv::Mat(),k,cv::Mat(),{c.width,c.height},rotation,translation,
      r1,r2,p1,p2,q,cv::CALIB_ZERO_DISPARITY,0.,{c.width,c.height},&left_roi,&right_roi);
  if (!cv::checkRange(r1) || !cv::checkRange(r2) || !cv::checkRange(p1) || !cv::checkRange(p2) || !cv::checkRange(q))
    throw std::invalid_argument("stereo_rectification_nonfinite_projection");
  // OpenCV 4.6 stereoRectify constructs image corners as Point2f. Even an
  // already rectified K can therefore acquire a few float32 ULPs in P. This
  // audit permits that representation rounding only; processing continues to
  // use the declared original K, without rectification or a fitted depth scale.
  const double projection_roundoff = 4.*std::numeric_limits<float>::epsilon()*
      std::max({1.,std::abs(c.fx),std::abs(c.fy),static_cast<double>(c.width),static_cast<double>(c.height)});
  for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col) {
    const double identity = row == col ? 1. : 0.;
    if (std::abs(r1.at<double>(row,col)-identity) > 1e-8 || std::abs(r2.at<double>(row,col)-identity) > 1e-8 ||
        std::abs(p1.at<double>(row,col)-k(row,col)) > projection_roundoff ||
        std::abs(p2.at<double>(row,col)-k(row,col)) > projection_roundoff)
      throw std::invalid_argument("stereo_rectification_requires_pixel_remap_not_supported_by_this_input_contract");
  }
  if (std::abs(p2.at<double>(0,3)+c.fx*c.baseline_m) > projection_roundoff*c.baseline_m)
    throw std::invalid_argument("stereo_rectification_projection_baseline_mismatch");
  std::copy_n(r1.ptr<double>(),9,calibration_audit_.r1.begin());
  std::copy_n(r2.ptr<double>(),9,calibration_audit_.r2.begin());
  std::copy_n(p1.ptr<double>(),12,calibration_audit_.p1.begin());
  std::copy_n(p2.ptr<double>(),12,calibration_audit_.p2.begin());
  std::copy_n(q.ptr<double>(),16,calibration_audit_.q.begin());
  calibration_audit_.left_valid_roi = {left_roi.x,left_roi.y,left_roi.width,left_roi.height};
  calibration_audit_.right_valid_roi = {right_roi.x,right_roi.y,right_roi.width,right_roi.height};
  left_matcher_ = cv::StereoSGBM::create(c.min_disparity,c.num_disparities,c.block_size,c.p1,c.p2,
      c.disp12_max_diff,c.pre_filter_cap,c.uniqueness_ratio,c.speckle_window_size,c.speckle_range,c.mode);
  const int right_min = -c.min_disparity-c.num_disparities+1;
  right_matcher_ = cv::StereoSGBM::create(right_min,c.num_disparities,c.block_size,c.p1,c.p2,
      c.disp12_max_diff,c.pre_filter_cap,c.uniqueness_ratio,c.speckle_window_size,c.speckle_range,c.mode);
  valid_left_ = cv::getValidDisparityROI(left_roi,right_roi,c.min_disparity,c.num_disparities,c.block_size);
  valid_right_ = cv::getValidDisparityROI(right_roi,left_roi,right_min,c.num_disparities,c.block_size);
  for (auto *m : {&gray_left_,&gray_right_}) m->create(c.height,c.width,CV_8UC1);
  for (auto *m : {&left_fixed_,&right_fixed_}) m->create(c.height,c.width,CV_16SC1);
  for (auto *m : {&gradient_left_,&gradient_right_,&energy_left_,&energy_right_,&texture_left_,&texture_right_})
    m->create(c.height,c.width,CV_32FC1);
}
void StereoDepthProcessor::validate_pair(const cv::Mat &left_bgr, const cv::Mat &right_bgr,
    const StereoFrameStamp &left, const StereoFrameStamp &right) {
  if (left_bgr.type() != CV_8UC3 || right_bgr.type() != CV_8UC3 ||
      left_bgr.rows != config_.height || left_bgr.cols != config_.width || left_bgr.size() != right_bgr.size() ||
      left.source_epoch.empty() || left.source_epoch.size() > 128 || left.source_epoch != right.source_epoch ||
      left.sequence != right.sequence || !std::isfinite(left.source_time_s) || left.source_time_s < 0. ||
      left.source_time_s != right.source_time_s || left.calibration_id != config_.calibration_id ||
      right.calibration_id != config_.calibration_id || left.clock_domain.empty() || left.clock_domain.size() > 64 ||
      left.clock_domain != right.clock_domain ||
      (epoch_ && (*epoch_ != left.source_epoch || *clock_domain_ != left.clock_domain ||
        left.sequence <= *last_sequence_ || left.source_time_s <= last_source_time_))) {
    ++rejected_inputs_;
    throw std::invalid_argument("stereo_pair_geometry_source_calibration_or_order_mismatch");
  }
}
void StereoDepthProcessor::texture_energy(const cv::Mat &gray, cv::Mat &gradient, cv::Mat &energy, cv::Mat &mean_energy) {
  cv::Sobel(gray,gradient,CV_32F,1,0,3,1./8.,0.,cv::BORDER_REFLECT_101);
  cv::multiply(gradient,gradient,energy);
  cv::boxFilter(energy,mean_energy,CV_32F,{config_.block_size,config_.block_size},{-1,-1},true,cv::BORDER_REFLECT_101);
}
StereoDepthResult StereoDepthProcessor::process(const cv::Mat &left_bgr, const cv::Mat &right_bgr,
    const StereoFrameStamp &left, const StereoFrameStamp &right) {
  const auto begin = Clock::now(); const double cpu_begin = thread_seconds();
  std::lock_guard<std::mutex> guard(mutex_);
  const auto locked = Clock::now();
  validate_pair(left_bgr,right_bgr,left,right);
  const auto &c = config_;
  cv::cvtColor(left_bgr,gray_left_,cv::COLOR_BGR2GRAY); cv::cvtColor(right_bgr,gray_right_,cv::COLOR_BGR2GRAY);
  const auto grayscale_done = Clock::now();
  left_matcher_->compute(gray_left_,gray_right_,left_fixed_);
  right_matcher_->compute(gray_right_,gray_left_,right_fixed_);
  const auto matching_done = Clock::now();
  texture_energy(gray_left_,gradient_left_,energy_left_,texture_left_);
  texture_energy(gray_right_,gradient_right_,energy_right_,texture_right_);
  StereoDepthResult out; out.stamp = left;
  const float unavailable = std::numeric_limits<float>::quiet_NaN();
  for (auto *m : {&out.depth_m,&out.disparity_px,&out.lr_error_px}) {
    m->create(c.height,c.width,CV_32FC1); m->setTo(unavailable);
  }
  for (auto *m : {&out.valid_mask,&out.rejection_mask,&out.texture_mask,&out.lr_consistency_mask}) {
    m->create(c.height,c.width,CV_8UC1); m->setTo(0);
  }
  const int scale = cv::StereoMatcher::DISP_SCALE;
  const int invalid_left = (c.min_disparity-1)*scale;
  const int right_min = -c.min_disparity-c.num_disparities+1;
  const int invalid_right = (right_min-1)*scale;
  const float texture_threshold = static_cast<float>(c.texture_min_gradient_rms*c.texture_min_gradient_rms);
  auto &d = out.diagnostics;
  d.total_pixels = static_cast<std::uint64_t>(c.width)*c.height;
  for (int y = 0; y < c.height; ++y) {
    const auto *left_values = left_fixed_.ptr<std::int16_t>(y), *right_values = right_fixed_.ptr<std::int16_t>(y);
    const auto *left_texture = texture_left_.ptr<float>(y), *right_texture = texture_right_.ptr<float>(y);
    auto *depth = out.depth_m.ptr<float>(y), *disparity = out.disparity_px.ptr<float>(y), *lr_error = out.lr_error_px.ptr<float>(y);
    auto *valid = out.valid_mask.ptr<std::uint8_t>(y), *rejected = out.rejection_mask.ptr<std::uint8_t>(y);
    auto *texture = out.texture_mask.ptr<std::uint8_t>(y), *consistent = out.lr_consistency_mask.ptr<std::uint8_t>(y);
    for (int x = 0; x < c.width; ++x) {
      std::uint8_t reason = 0;
      const bool left_ok = left_values[x] > invalid_left && left_values[x] > 0;
      if (!left_ok) reason |= InvalidLeftMatch;
      if (!valid_left_.contains({x,y})) reason |= SearchBorder;
      const bool texture_ok_left = left_texture[x] >= texture_threshold;
      if (!texture_ok_left) reason |= LowLeftTexture;
      if (left_ok) {
        const float dl = static_cast<float>(left_values[x])/scale;
        disparity[x] = dl;
        const float xr = static_cast<float>(x)-dl;
        const int x0 = static_cast<int>(std::floor(xr)); const int x1 = x0+1;
        if (!valid_right_.contains({x0,y}) || !valid_right_.contains({x1,y}) ||
            right_values[x0] <= invalid_right || right_values[x1] <= invalid_right) reason |= InvalidRightMatch;
        else {
          const float alpha = xr-x0;
          const float dr = ((1.f-alpha)*right_values[x0]+alpha*right_values[x1])/scale;
          lr_error[x] = std::abs(dl+dr);
          if (lr_error[x] > c.lr_max_error_px) reason |= LeftRightInconsistent;
          else consistent[x] = 255;
          const bool texture_ok_right = std::min(right_texture[x0],right_texture[x1]) >= texture_threshold;
          if (!texture_ok_right) reason |= LowRightTexture;
          else if (texture_ok_left) texture[x] = 255;
        }
        const double z = c.fx*c.baseline_m/dl;
        const float metric_depth = static_cast<float>(z);
        if (!positive(z) || z > std::numeric_limits<float>::max() || !std::isfinite(metric_depth) || metric_depth <= 0.f)
          reason |= InvalidMetricDepth;
        else if (!reason) { depth[x] = metric_depth; valid[x] = 255; ++d.valid_pixels; }
      }
      rejected[x] = reason;
      if (reason & InvalidLeftMatch) ++d.invalid_left_match;
      if (reason & SearchBorder) ++d.search_border;
      if (reason & LowLeftTexture) ++d.low_left_texture;
      if (reason & InvalidRightMatch) ++d.invalid_right_match;
      if (reason & LeftRightInconsistent) ++d.lr_inconsistent;
      if (reason & LowRightTexture) ++d.low_right_texture;
      if (reason & InvalidMetricDepth) ++d.invalid_metric_depth;
    }
  }
  d.valid_fraction = static_cast<double>(d.valid_pixels)/d.total_pixels;
  const auto complete = Clock::now();
  d.lock_wait_wall_s = seconds(begin,locked);
  d.grayscale_wall_s = seconds(locked,grayscale_done); d.matching_wall_s = seconds(grayscale_done,matching_done);
  d.quality_wall_s = seconds(matching_done,complete); d.wall_s = seconds(begin,complete);
  d.thread_cpu_s = thread_seconds()-cpu_begin;
  epoch_ = left.source_epoch; clock_domain_ = left.clock_domain;
  last_sequence_ = left.sequence; last_source_time_ = left.source_time_s; ++accepted_frames_;
  return out;
}
StereoProcessorSnapshot StereoDepthProcessor::snapshot() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return {accepted_frames_,rejected_inputs_,last_sequence_.value_or(0),epoch_,clock_domain_};
}
}  // namespace companionbot::perception
