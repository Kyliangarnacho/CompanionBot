#include "companionbot_perception/stereo_depth.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace companionbot::perception;
namespace {
void require(bool value,const char *message) { if (!value) throw std::runtime_error(message); }
void rejected(const std::function<void()> &call) {
  try { call(); } catch (const std::invalid_argument &) { return; }
  throw std::runtime_error("expected_invalid_input_rejection");
}
StereoConfig config() {
  StereoConfig c;
  c.width=320; c.height=120; c.fx=200.; c.fy=200.; c.cx=160.; c.cy=60.;
  c.baseline_m=.15; c.calibration_id="synthetic_rectified_fixture";
  c.right_translation_from_left={-.15,0.,0.};
  return c;
}
std::pair<cv::Mat,cv::Mat> images(const StereoConfig &c,int disparity) {
  cv::Mat left(c.height,c.width,CV_8UC3),right(c.height,c.width,CV_8UC3);
  cv::RNG rng(20261024); rng.fill(left,cv::RNG::UNIFORM,0,256); rng.fill(right,cv::RNG::UNIFORM,0,256);
  left(cv::Rect(disparity,0,c.width-disparity,c.height)).copyTo(right(cv::Rect(0,0,c.width-disparity,c.height)));
  return {left,right};
}
}
int main() {
  try {
    const auto c=config();
    StereoDepthProcessor processor(c);
    auto pair=images(c,20);
    StereoFrameStamp stamp{"synthetic_source",0,0.,c.calibration_id,"declared_software_fixture"};
    auto out=processor.process(pair.first,pair.second,stamp,stamp);
    std::vector<float> depths;
    for (int y=20;y<100;++y) for (int x=100;x<240;++x)
      if (out.valid_mask.at<std::uint8_t>(y,x)) depths.push_back(out.depth_m.at<float>(y,x));
    require(depths.size()>static_cast<std::size_t>(.8*80*140),"textured_shift_valid_coverage");
    std::sort(depths.begin(),depths.end());
    const double median=depths[depths.size()/2];
    require(std::abs(median-1.5)<.01,"axial_depth_focal_baseline_disparity_contract");
    require(out.diagnostics.valid_pixels>0 && out.diagnostics.wall_s>0.,"stereo_diagnostics");
    auto oracle=cv::StereoSGBM::create(c.min_disparity,c.num_disparities,c.block_size,c.p1,c.p2,
        c.disp12_max_diff,c.pre_filter_cap,c.uniqueness_ratio,c.speckle_window_size,c.speckle_range,c.mode);
    cv::Mat gray_left,gray_right,disparity;
    cv::cvtColor(pair.first,gray_left,cv::COLOR_BGR2GRAY); cv::cvtColor(pair.second,gray_right,cv::COLOR_BGR2GRAY);
    oracle->compute(gray_left,gray_right,disparity);
    for (int y=0;y<c.height;++y) for (int x=0;x<c.width;++x)
      if (disparity.at<std::int16_t>(y,x)>0)
        require(out.disparity_px.at<float>(y,x)==static_cast<float>(disparity.at<std::int16_t>(y,x))/16.,"unchanged_opencv_matcher_parity");
    const auto original_depth=out.depth_m.clone();
    cv::Mat flat(c.height,c.width,CV_8UC3,cv::Scalar(127,127,127));
    stamp.sequence=1; stamp.source_time_s=.05;
    auto unobservable=processor.process(flat,flat,stamp,stamp);
    require(unobservable.diagnostics.valid_pixels==0,"flat_scene_must_be_unavailable");
    require(cv::countNonZero(unobservable.texture_mask)==0,"flat_scene_texture_mask");
    require(out.depth_m.data!=unobservable.depth_m.data,"result_storage_must_be_independent");
    for (int y=0;y<c.height;++y) for (int x=0;x<c.width;++x) {
      const float before=original_depth.at<float>(y,x),after=out.depth_m.at<float>(y,x);
      require((std::isnan(before)&&std::isnan(after)) || before==after,"old_result_overwritten");
    }
    rejected([&]{processor.process(flat,flat,stamp,stamp);});
    auto wrong=stamp; wrong.sequence=2; wrong.source_time_s=.10;
    rejected([&]{processor.process(flat,flat,stamp,wrong);});
    auto next=wrong; auto right=next; right.source_epoch="different";
    rejected([&]{processor.process(flat,flat,next,right);});
    right=next; right.source_time_s+=.001;
    rejected([&]{processor.process(flat,flat,next,right);});
    right=next; right.calibration_id="unknown";
    rejected([&]{processor.process(flat,flat,next,right);});
    right=next; right.clock_domain="other_clock";
    rejected([&]{processor.process(flat,flat,next,right);});
    auto after=processor.process(pair.first,pair.second,next,next);
    require(after.diagnostics.valid_pixels>0,"rejected_frame_must_not_consume_watermark");
    auto bad=c; bad.right_translation_from_left[0]=.15;
    rejected([&]{StereoDepthProcessor invalid(bad);});
    bad=c; bad.right_rotation_from_left[1]=.1;
    rejected([&]{StereoDepthProcessor invalid(bad);});
    bad=c; bad.num_disparities=17;
    rejected([&]{StereoDepthProcessor invalid(bad);});
    auto underflow_config=c;
    underflow_config.baseline_m=1e-47;
    underflow_config.right_translation_from_left={-underflow_config.baseline_m,0.,0.};
    StereoDepthProcessor underflow_processor(underflow_config);
    const auto underflow=underflow_processor.process(pair.first,pair.second,next,next);
    require(underflow.diagnostics.valid_pixels==0 && underflow.diagnostics.invalid_metric_depth>0,
        "float32_underflow_must_not_be_valid_metric_depth");
    auto real=c; real.width=640; real.height=360;
    real.fx=real.fy=234.58056711141703;real.cx=320.;real.cy=180.;real.calibration_id="declared_640_stereo";
    StereoDepthProcessor calibrated(real);
    const auto audit=calibrated.calibration_audit();
    require(std::abs(audit.p2[3]+real.fx*real.baseline_m)<1e-7,"native_stereo_rectify_baseline");
    std::cout << "{\"passed\":true,\"opencv\":\"" << StereoDepthProcessor::opencv_version()
              << "\",\"synthetic_median_z_m\":" << median << ",\"synthetic_valid_pixels\":"
              << out.diagnostics.valid_pixels << ",\"scope\":\"software RGB shift and geometry; no rendered accuracy or hardware certification\"}\n";
    return 0;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
