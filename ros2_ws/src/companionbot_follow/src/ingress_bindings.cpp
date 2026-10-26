#include "companionbot_follow/navigation_ingress.hpp"

#include <memory>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using companionbot::follow::MotionAuthority;
using companionbot::follow::NavigationIngressRuntime;

void bind_navigation_ingress(py::module_& module) {
  py::class_<NavigationIngressRuntime>(module, "NavigationIngressRuntime")
      .def(py::init<std::shared_ptr<MotionAuthority>, const std::string&, const std::string&, bool>(),
           py::arg("authority"), py::arg("output_dir"),
           py::arg("controller") = "/controller_server", py::arg("use_sim_time") = true)
      .def("start", &NavigationIngressRuntime::start, py::call_guard<py::gil_scoped_release>())
      .def("check_health", &NavigationIngressRuntime::check_health, py::call_guard<py::gil_scoped_release>())
      .def("close", &NavigationIngressRuntime::close, py::call_guard<py::gil_scoped_release>())
      .def("snapshot", [](const NavigationIngressRuntime& runtime, bool include_timings) {
        const auto value = runtime.snapshot(include_timings);
        py::dict result;
#define SNAPSHOT_FIELD(field) result[#field] = value.field
        SNAPSHOT_FIELD(started); SNAPSHOT_FIELD(closed); SNAPSHOT_FIELD(thread_stopped);
        SNAPSHOT_FIELD(reason); SNAPSHOT_FIELD(error); SNAPSHOT_FIELD(node_name);
        SNAPSHOT_FIELD(commands_received); SNAPSHOT_FIELD(commands_accepted); SNAPSHOT_FIELD(commands_rejected);
        SNAPSHOT_FIELD(controls_received); SNAPSHOT_FIELD(health_checks); SNAPSHOT_FIELD(gid_rejections);
        SNAPSHOT_FIELD(last_command_source_age_s); SNAPSHOT_FIELD(last_command_receipt_s); SNAPSHOT_FIELD(max_callback_wall_s);
        SNAPSHOT_FIELD(invalid_scans); SNAPSHOT_FIELD(invalid_odometry); SNAPSHOT_FIELD(invalid_poses);
#undef SNAPSHOT_FIELD
        result["command_qos_depth"] = 1;
        result["middleware_drop_count_available"] = false;
        result["timing_capacity_per_callback"] = 32768;
        if (!value.callback_timings.empty()) {
          py::dict timings;
          for (const auto& timing : value.callback_timings) {
            py::dict sample;
#define TIMING_FIELD(field) sample[#field] = timing.field
            TIMING_FIELD(samples); TIMING_FIELD(retained_samples);
            TIMING_FIELD(wall_total_s); TIMING_FIELD(cpu_total_s);
            TIMING_FIELD(wall_max_s); TIMING_FIELD(cpu_max_s);
            TIMING_FIELD(wall_p50_s); TIMING_FIELD(wall_p95_s); TIMING_FIELD(wall_p99_s);
            TIMING_FIELD(cpu_p50_s); TIMING_FIELD(cpu_p95_s); TIMING_FIELD(cpu_p99_s);
#undef TIMING_FIELD
            sample["quantiles_cover_all_samples"] = timing.samples == timing.retained_samples;
            timings[py::str(timing.name)] = std::move(sample);
          }
          result["callback_timings"] = std::move(timings);
          const auto& age = value.command_source_age;
          py::dict ages;
          ages["samples"] = age.samples; ages["retained_samples"] = age.retained_samples;
          ages["p50_s"] = age.wall_p50_s; ages["p95_s"] = age.wall_p95_s;
          ages["p99_s"] = age.wall_p99_s; ages["max_s"] = age.wall_max_s;
          ages["quantiles_cover_all_samples"] = age.samples == age.retained_samples;
          result["command_source_age_s"] = std::move(ages);
        }
        return result;
      }, py::arg("include_timings") = false);
}
