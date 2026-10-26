#include "companionbot_perception/stereo_depth.hpp"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace py = pybind11;
using namespace companionbot::perception;
namespace {
struct PythonStereoDepthResult {
  StereoFrameStamp stamp;
  py::array depth_m, disparity_px, lr_error_px, valid_mask, rejection_mask, texture_mask, lr_consistency_mask;
  py::dict diagnostics;
};
template <typename T> py::array owned_array(const cv::Mat &mat) {
  if (!mat.isContinuous() || mat.channels() != 1 || mat.elemSize() != sizeof(T))
    throw std::runtime_error("unexpected_stereo_output_storage");
  py::array_t<T> out({mat.rows,mat.cols});
  std::memcpy(out.mutable_data(),mat.data,mat.total()*sizeof(T));
  out.attr("setflags")(false);
  return out;
}
cv::Mat input_view(const py::array &array) {
  if (!array.dtype().is(py::dtype::of<std::uint8_t>())) throw std::invalid_argument("stereo_input_must_be_uint8_bgr");
  const auto info = array.request();
  if (info.ndim != 3 || info.shape[2] != 3 || info.shape[0] <= 0 || info.shape[1] <= 0 ||
      info.shape[0] > 720 || info.shape[1] > 1280 || info.strides[2] != 1 || info.strides[1] != 3 ||
      info.strides[0] < info.shape[1]*3)
    throw std::invalid_argument("stereo_input_requires_positive_stride_hwc_bgr");
  return cv::Mat(static_cast<int>(info.shape[0]),static_cast<int>(info.shape[1]),CV_8UC3,info.ptr,
      static_cast<std::size_t>(info.strides[0]));
}
py::dict diagnostics(const StereoDiagnostics &d) {
  py::dict out;
#define FIELD(name) out[#name] = d.name
  FIELD(total_pixels); FIELD(valid_pixels); FIELD(invalid_left_match); FIELD(search_border);
  FIELD(low_left_texture); FIELD(invalid_right_match); FIELD(lr_inconsistent); FIELD(low_right_texture);
  FIELD(invalid_metric_depth); FIELD(valid_fraction); FIELD(wall_s); FIELD(thread_cpu_s);
  FIELD(lock_wait_wall_s); FIELD(grayscale_wall_s); FIELD(matching_wall_s); FIELD(quality_wall_s);
#undef FIELD
  out["metric_convention"] = "left_optical_axial_z_m";
  out["disparity_units"] = "pixel"; out["mask_values"] = "0 invalid / 255 valid; rejection_mask is a bitfield";
  out["thread_cpu_scope"] = "calling thread only; OpenCV helper thread CPU is not included";
  out["fallback"] = false;
  return out;
}
}
PYBIND11_MODULE(_native_stereo, module) {
  module.doc() = "Explicit native OpenCV rectified stereo RGB depth; no GT, no monocular fallback";
  module.attr("ABI_VERSION") = 1;
  module.attr("OPENCV_COMPILED_VERSION") = StereoDepthProcessor::opencv_version();
  module.attr("OPENCV_RUNTIME_VERSION") = cv::getVersionString();
  module.attr("INVALID_LEFT_MATCH") = static_cast<int>(InvalidLeftMatch);
  module.attr("SEARCH_BORDER") = static_cast<int>(SearchBorder);
  module.attr("LOW_LEFT_TEXTURE") = static_cast<int>(LowLeftTexture);
  module.attr("INVALID_RIGHT_MATCH") = static_cast<int>(InvalidRightMatch);
  module.attr("LEFT_RIGHT_INCONSISTENT") = static_cast<int>(LeftRightInconsistent);
  module.attr("LOW_RIGHT_TEXTURE") = static_cast<int>(LowRightTexture);
  module.attr("INVALID_METRIC_DEPTH") = static_cast<int>(InvalidMetricDepth);
  py::class_<StereoConfig>(module,"StereoConfig")
      .def(py::init<>())
#define CONFIG(name) .def_readwrite(#name,&StereoConfig::name)
      CONFIG(width) CONFIG(height) CONFIG(fx) CONFIG(fy) CONFIG(cx) CONFIG(cy) CONFIG(baseline_m) CONFIG(calibration_id)
      CONFIG(right_rotation_from_left) CONFIG(right_translation_from_left)
      CONFIG(min_disparity) CONFIG(num_disparities) CONFIG(block_size) CONFIG(p1) CONFIG(p2) CONFIG(disp12_max_diff)
      CONFIG(pre_filter_cap) CONFIG(uniqueness_ratio) CONFIG(speckle_window_size) CONFIG(speckle_range) CONFIG(mode)
      CONFIG(lr_max_error_px) CONFIG(texture_min_gradient_rms)
#undef CONFIG
      ;
  py::class_<StereoFrameStamp>(module,"StereoFrameStamp")
      .def(py::init([](std::string epoch, const py::object &sequence, double time, std::string calibration, std::string domain) {
        if (PyBool_Check(sequence.ptr()) || !PyLong_Check(sequence.ptr())) throw std::invalid_argument("stereo_sequence_must_be_unsigned_integer");
        return StereoFrameStamp{std::move(epoch),sequence.cast<std::uint64_t>(),time,std::move(calibration),std::move(domain)};
      }),py::arg("source_epoch"),py::arg("sequence"),py::arg("source_time_s"),py::arg("calibration_id"),py::arg("clock_domain")="mujoco_sim")
      .def_readonly("source_epoch",&StereoFrameStamp::source_epoch)
      .def_readonly("sequence",&StereoFrameStamp::sequence)
      .def_readonly("source_time_s",&StereoFrameStamp::source_time_s)
      .def_readonly("calibration_id",&StereoFrameStamp::calibration_id)
      .def_readonly("clock_domain",&StereoFrameStamp::clock_domain);
  py::class_<PythonStereoDepthResult>(module,"StereoDepthResult")
      .def_readonly("stamp",&PythonStereoDepthResult::stamp)
      .def_readonly("depth_m",&PythonStereoDepthResult::depth_m)
      .def_readonly("disparity_px",&PythonStereoDepthResult::disparity_px)
      .def_readonly("lr_error_px",&PythonStereoDepthResult::lr_error_px)
      .def_readonly("valid_mask",&PythonStereoDepthResult::valid_mask)
      .def_readonly("rejection_mask",&PythonStereoDepthResult::rejection_mask)
      .def_readonly("texture_mask",&PythonStereoDepthResult::texture_mask)
      .def_readonly("lr_consistency_mask",&PythonStereoDepthResult::lr_consistency_mask)
      .def_readonly("diagnostics",&PythonStereoDepthResult::diagnostics);
  py::class_<StereoDepthProcessor>(module,"StereoDepthProcessor")
      .def(py::init<StereoConfig>(),py::arg("config"))
      .def("process",[](StereoDepthProcessor &processor,const py::array &left,const py::array &right,
          const StereoFrameStamp &left_stamp,const StereoFrameStamp &right_stamp) {
        const auto started = std::chrono::steady_clock::now();
        const auto left_view = input_view(left),right_view = input_view(right);
        // Own the low-frequency RGB pair before releasing the GIL. The caller
        // must keep each source buffer stable during this entry copy (as for
        // any buffer read); later Python reuse cannot race the native matcher.
        // In particular, no writable NumPy view is borrowed during processing.
        const auto snapshot_started = std::chrono::steady_clock::now();
        const cv::Mat left_snapshot = left_view.clone(),right_snapshot = right_view.clone();
        const auto snapshot_complete = std::chrono::steady_clock::now();
        StereoDepthResult result;
        { py::gil_scoped_release release; result = processor.process(left_snapshot,right_snapshot,left_stamp,right_stamp); }
        const auto copy_started = std::chrono::steady_clock::now();
        PythonStereoDepthResult out;
        out.stamp = result.stamp;
        out.depth_m = owned_array<float>(result.depth_m);
        out.disparity_px = owned_array<float>(result.disparity_px);
        out.lr_error_px = owned_array<float>(result.lr_error_px);
        out.valid_mask = owned_array<std::uint8_t>(result.valid_mask);
        out.rejection_mask = owned_array<std::uint8_t>(result.rejection_mask);
        out.texture_mask = owned_array<std::uint8_t>(result.texture_mask);
        out.lr_consistency_mask = owned_array<std::uint8_t>(result.lr_consistency_mask);
        out.diagnostics = diagnostics(result.diagnostics);
        const auto copied = std::chrono::steady_clock::now();
        out.diagnostics["output_copy_wall_s"] = std::chrono::duration<double>(copied-copy_started).count();
        out.diagnostics["input_snapshot_wall_s"] = std::chrono::duration<double>(snapshot_complete-snapshot_started).count();
        out.diagnostics["input_snapshot_bytes"] = left_snapshot.total()*left_snapshot.elemSize()+right_snapshot.total()*right_snapshot.elemSize();
        out.diagnostics["binding_total_wall_s"] = std::chrono::duration<double>(copied-started).count();
        out.diagnostics["output_owns_memory"] = true;
        return out;
      },py::arg("left_bgr"),py::arg("right_bgr"),py::arg("left_stamp"),py::arg("right_stamp"))
      .def("snapshot",[](const StereoDepthProcessor &processor) {
        StereoProcessorSnapshot value;
        { py::gil_scoped_release release; value = processor.snapshot(); }
        py::dict out;
        out["accepted_frames"] = value.accepted_frames; out["rejected_native_inputs"] = value.rejected_inputs;
        out["last_sequence"] = value.last_sequence; out["source_epoch"] = py::cast(value.source_epoch);
        out["clock_domain"] = py::cast(value.clock_domain);
        return out;
      })
      .def("calibration_audit",[](const StereoDepthProcessor &processor) {
        const auto value = processor.calibration_audit();
        py::dict out;
        out["R1"] = value.r1; out["R2"] = value.r2; out["P1"] = value.p1; out["P2"] = value.p2; out["Q"] = value.q;
        out["left_valid_roi"] = value.left_valid_roi; out["right_valid_roi"] = value.right_valid_roi;
        out["rectification"] = "native OpenCV stereoRectify validates parallel/D0 geometry; no per-frame remap required";
        return out;
      });
}
