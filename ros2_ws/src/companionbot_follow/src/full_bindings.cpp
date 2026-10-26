#include "companionbot_follow/full_reference.hpp"
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace companionbot::follow;
namespace {
FullPlan from_rolling(const py::object &rolling) {
  const py::object plan = rolling.attr("plan");
  const auto states = py::array_t<double, py::array::c_style | py::array::forcecast>(plan.attr("reference_states"));
  const auto acceleration = py::array_t<double, py::array::c_style | py::array::forcecast>(plan.attr("reference_accelerations_m_s2"));
  const auto inputs = py::array_t<double, py::array::c_style | py::array::forcecast>(plan.attr("feedforward_inputs_nm"));
  const auto finished = py::array_t<bool, py::array::c_style | py::array::forcecast>(plan.attr("shaped_reference_finished"));
  if (states.ndim() != 2 || states.shape(1) != 4 || states.shape(0) < 2 || states.shape(0) > 1000000 ||
      acceleration.ndim() != 1 || acceleration.shape(0) != states.shape(0) ||
      finished.ndim() != 1 || finished.shape(0) != states.shape(0) ||
      inputs.ndim() != 1 || inputs.shape(0) + 1 != states.shape(0))
    throw std::invalid_argument("full_solver_array_geometry_mismatch");
  FullPlan result;
  result.target_velocity = py::cast<double>(plan.attr("target_velocity_m_s"));
  result.transient_duration = py::cast<double>(plan.attr("planned_transient_duration_s"));
  result.feedforward_rearmed = py::cast<bool>(plan.attr("feedforward_rearmed"));
  result.ruckig_duration = py::cast<double>(rolling.attr("ruckig_duration_s"));
  result.horizon = py::cast<double>(rolling.attr("dynamic_nominal_horizon_s"));
  result.residual_rms = py::cast<double>(rolling.attr("normalized_residual_rms"));
  const auto n = static_cast<std::size_t>(states.shape(0));
  result.states.resize(n); result.accelerations.resize(n); result.shaped_finished.resize(n); result.inputs.resize(n - 1);
  const auto x = states.unchecked<2>(); const auto a = acceleration.unchecked<1>();
  const auto u = inputs.unchecked<1>(); const auto reached = finished.unchecked<1>();
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < 4; ++j) result.states[i][j] = x(i, j);
    result.accelerations[i] = a(i); result.shaped_finished[i] = reached(i);
    if (i + 1 < n) result.inputs[i] = u(i);
  }
  const auto dumps = py::module_::import("json").attr("dumps");
  result.planning_diagnostics = nlohmann::json::parse(py::cast<std::string>(dumps(rolling.attr("planning_diagnostics"))));
  result.projection_diagnostics = nlohmann::json::parse(py::cast<std::string>(dumps(rolling.attr("projection_diagnostics"))));
  return result;
}
}
void bind_full_reference(py::module_ &module) {
  py::class_<SchedulerConfig>(module, "SchedulerConfig")
    .def(py::init<>())
    .def_readwrite("accept_delta_velocity", &SchedulerConfig::accept_delta_velocity)
    .def_readwrite("full_delta_velocity", &SchedulerConfig::full_delta_velocity)
    .def_readwrite("stable_window_s", &SchedulerConfig::stable_window_s)
    .def_readwrite("stable_range_velocity", &SchedulerConfig::stable_range_velocity)
    .def_readwrite("initial_accepted_velocity", &SchedulerConfig::initial_accepted_velocity)
    .def_readwrite("require_stable_candidate", &SchedulerConfig::require_stable_candidate)
    .def_readwrite("minimum_delta_enabled", &SchedulerConfig::minimum_delta_enabled)
    .def_readwrite("force_full_dynamic", &SchedulerConfig::force_full_dynamic);
  py::class_<TwoPathConfig>(module, "TwoPathConfig")
    .def(py::init<>())
    .def_readwrite("producer", &TwoPathConfig::producer)
    .def_readwrite("scheduler", &TwoPathConfig::scheduler)
    .def_readwrite("feedforward_fade_s", &TwoPathConfig::feedforward_fade_s)
    .def_readwrite("quiet_theta_rad", &TwoPathConfig::quiet_theta_rad)
    .def_readwrite("quiet_theta_dot_rad_s", &TwoPathConfig::quiet_theta_dot_rad_s)
    .def_readwrite("defer_full_accept_until_observed_exit", &TwoPathConfig::defer_full_accept_until_observed_exit);
  py::class_<PlanRequest>(module, "PlanRequest")
    .def_readonly("start_reference", &PlanRequest::start_reference)
    .def_readonly("start_acceleration", &PlanRequest::start_acceleration)
    .def_readonly("target_velocity", &PlanRequest::target_velocity)
    .def_readonly("execution_start_time", &PlanRequest::execution_start_time)
    .def_readonly("request_time", &PlanRequest::request_time);
  py::class_<FullPlan>(module, "FullPlan")
    .def_static("from_rolling", &from_rolling,
      "Copy the existing pure FULL solver's arrays once per accepted plan; no runtime state is imported.");
  py::class_<TwoPathSample>(module, "TwoPathSample")
    .def_readonly("reference_state", &TwoPathSample::reference_state)
    .def_readonly("acceleration", &TwoPathSample::acceleration)
    .def_readonly("raw_feedforward", &TwoPathSample::raw_feedforward)
    .def_readonly("lifecycle_feedforward", &TwoPathSample::lifecycle_feedforward)
    .def_readonly("fade_alpha", &TwoPathSample::fade_alpha)
    .def_readonly("feedforward_phase", &TwoPathSample::feedforward_phase)
    .def_readonly("velocity_phase", &TwoPathSample::velocity_phase)
    .def_readonly("scheduler_mode", &TwoPathSample::scheduler_mode)
    .def_readonly("replanned", &TwoPathSample::replanned)
    .def_readonly("accepted_changed", &TwoPathSample::accepted_changed);
  py::class_<TwoPathBlock, ProducedBlock>(module, "TwoPathBlock")
    .def_readonly("details", &TwoPathBlock::details)
    .def_readonly("raw_velocity", &TwoPathBlock::raw_velocity)
    .def_readonly("candidate_delta", &TwoPathBlock::candidate_delta)
    .def_readonly("candidate_target", &TwoPathBlock::candidate_target)
    .def_readonly("candidate_stable", &TwoPathBlock::candidate_stable)
    .def_readonly("pending_velocity", &TwoPathBlock::pending_velocity)
    .def_readonly("pending_active", &TwoPathBlock::pending_active)
    .def_readonly("block_id", &TwoPathBlock::block_id);
  py::class_<TwoPathProducer>(module, "TwoPathProducer")
    .def(py::init<TwoPathConfig, FullPlanSolver, std::uint32_t>(), py::arg("config"), py::arg("solver") = nullptr, py::arg("epoch") = 0)
    .def("reset", &TwoPathProducer::reset)
    .def("produce", &TwoPathProducer::produce, py::arg("start_tick"), py::arg("generated_time"),
      py::arg("source_time"), py::arg("velocity"), py::arg("yaw"), py::arg("observe_command") = true,
      py::call_guard<py::gil_scoped_release>())
    .def_property_readonly("next_reference", &TwoPathProducer::next_reference)
    .def_property_readonly("next_acceleration", &TwoPathProducer::next_acceleration)
    .def_property_readonly("reference_epoch", &TwoPathProducer::epoch)
    .def_property_readonly("next_sequence", &TwoPathProducer::next_sequence)
    .def("summary_json", &TwoPathProducer::summary_json)
    .def("summary", [](const TwoPathProducer &producer) {
      return py::module_::import("json").attr("loads")(producer.summary_json());
    });
}
