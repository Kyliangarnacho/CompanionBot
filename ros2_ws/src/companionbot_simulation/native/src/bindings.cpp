#include "companionbot_simulation/navigation_runtime.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using companionbot::simulation::NavigationRuntime;
using companionbot::simulation::RuntimeOutput;
using companionbot::simulation::FollowClearanceConfig;

PYBIND11_MODULE(_native_simulation, module) {
  // Import registers the single shared_ptr<MotionAuthority> and ProducerConfig
  // types. This module creates no second Python-facing authority definition.
  py::module_::import("companionbot_follow._native_runtime");
  module.attr("ABI_VERSION") = 1;
  module.doc() = "Native Pi scheduling and shared C99 software MCU; no hardware arm";
  py::class_<FollowClearanceConfig>(module, "FollowClearanceConfig")
      .def(py::init([](double radius, double reaction, std::uint32_t steps) {
        return FollowClearanceConfig{radius, reaction, steps};
      }), py::arg("radius_m"), py::arg("reaction_time_s"), py::arg("maximum_projection_steps"))
      .def_readonly("radius_m", &FollowClearanceConfig::radius_m)
      .def_readonly("reaction_time_s", &FollowClearanceConfig::reaction_time_s)
      .def_readonly("maximum_projection_steps", &FollowClearanceConfig::maximum_projection_steps);
  py::class_<cb_reference>(module, "Reference")
      .def_readonly("p", &cb_reference::p)
      .def_readonly("v", &cb_reference::v)
      .def_readonly("a", &cb_reference::a)
      .def_readonly("theta", &cb_reference::theta)
      .def_readonly("theta_dot", &cb_reference::theta_dot)
      .def_readonly("u_ff", &cb_reference::u_ff);
  py::class_<cb_brake_config>(module, "BrakeConfig")
      .def(py::init<>())
      .def(py::init([](double dt, double deceleration, double jerk, double handoff) {
        return cb_brake_config{dt, deceleration, jerk, handoff};
      }), py::arg("dt"), py::arg("deceleration"), py::arg("jerk"), py::arg("handoff"))
      .def_readwrite("dt", &cb_brake_config::dt)
      .def_readwrite("deceleration", &cb_brake_config::deceleration)
      .def_readwrite("jerk", &cb_brake_config::jerk)
      .def_readwrite("handoff", &cb_brake_config::handoff);
  py::class_<cb_mcu_config>(module, "McuConfig")
      .def(py::init<>())
      .def(py::init([](cb_brake_config brake, double heartbeat, double starvation,
                       double pitch, double yaw_timeout, double yaw_decay) {
        return cb_mcu_config{brake, heartbeat, starvation, pitch, yaw_timeout, yaw_decay};
      }), py::arg("brake"), py::arg("heartbeat_timeout"), py::arg("starvation_margin"),
          py::arg("nominal_pitch"), py::arg("yaw_timeout"), py::arg("yaw_decay"))
      .def_readwrite("brake", &cb_mcu_config::brake)
      .def_readwrite("heartbeat_timeout", &cb_mcu_config::heartbeat_timeout)
      .def_readwrite("starvation_margin", &cb_mcu_config::starvation_margin)
      .def_readwrite("nominal_pitch", &cb_mcu_config::nominal_pitch)
      .def_readwrite("yaw_timeout", &cb_mcu_config::yaw_timeout)
      .def_readwrite("yaw_decay", &cb_mcu_config::yaw_decay);
  py::class_<RuntimeOutput>(module, "RuntimeOutput")
      .def_property_readonly("reference", [](const RuntimeOutput &out) { return out.command.reference; })
      .def_readonly("next_reference", &RuntimeOutput::next_reference)
      .def_property_readonly("yaw_rate", [](const RuntimeOutput &out) { return out.command.yaw_rate; })
      .def_property_readonly("block_sequence", [](const RuntimeOutput &out) { return out.command.block_sequence; })
      .def_property_readonly("sample_index", [](const RuntimeOutput &out) { return out.command.sample_index; })
      .def_property_readonly("epoch", [](const RuntimeOutput &out) { return out.command.epoch; })
      .def_property_readonly("safety_state", [](const RuntimeOutput &out) { return out.command.safety_state; })
      .def_property_readonly("reason", [](const RuntimeOutput &out) { return out.command.reason; })
      .def_property_readonly("pi_lost", [](const RuntimeOutput &out) { return out.command.pi_lost; })
      .def_readonly("block_start_time_s", &RuntimeOutput::block_start_time_s)
      .def_readonly("intent_source_time_s", &RuntimeOutput::intent_source_time_s);
  py::class_<NavigationRuntime>(module, "NavigationRuntime")
      .def(py::init<std::shared_ptr<companionbot::follow::MotionAuthority>,
                    companionbot::follow::ProducerConfig, cb_mcu_config, double>(),
           py::arg("authority"), py::arg("producer_config"), py::arg("mcu_config"),
           py::arg("transport_latency_s") = 0.)
      .def(py::init<std::shared_ptr<companionbot::follow::MotionAuthority>,
                    companionbot::follow::TwoPathConfig, cb_mcu_config, double,
                    companionbot::follow::FullPlanSolver, std::shared_ptr<companionbot::follow::FollowController>,
                    std::optional<FollowClearanceConfig>>(),
           py::arg("authority"), py::arg("producer_config"), py::arg("mcu_config"),
           py::arg("transport_latency_s") = 0., py::arg("full_solver") = nullptr,
           py::arg("follow_controller") = nullptr, py::arg("clearance_config") = py::none())
      .def("step", &NavigationRuntime::step, py::arg("tick"), py::arg("receipt_now_s"),
           py::arg("local_velocity"), py::arg("local_pitch"), py::arg("local_pitch_rate"),
           py::call_guard<py::gil_scoped_release>())
      .def("peek", &NavigationRuntime::peek, py::call_guard<py::gil_scoped_release>())
      .def("reason_name", &NavigationRuntime::reason_name, py::call_guard<py::gil_scoped_release>())
      .def_static("state_name", &NavigationRuntime::state_name)
      .def("report_json", &NavigationRuntime::report_json, py::call_guard<py::gil_scoped_release>())
      .def("report", [](const NavigationRuntime &runtime) {
        std::string report;
        { py::gil_scoped_release release; report = runtime.report_json(); }
        return py::module_::import("json").attr("loads")(report);
      })
      .def_property_readonly("epoch", [](const NavigationRuntime &runtime) {
        py::gil_scoped_release release; return runtime.epoch();
      })
      .def_property_readonly("safety_state", [](const NavigationRuntime &runtime) {
        py::gil_scoped_release release; return runtime.safety_state();
      })
      .def_property_readonly("reason", [](const NavigationRuntime &runtime) {
        py::gil_scoped_release release; return runtime.reason();
      });
}
