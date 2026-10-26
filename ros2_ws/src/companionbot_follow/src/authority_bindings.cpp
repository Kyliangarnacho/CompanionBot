#include "companionbot_follow/motion_authority.hpp"

#include <memory>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace companionbot::follow;

namespace {
py::dict event_dict(const AuthorityEvent& event) {
  py::dict result;
  result["event"] = event.event;
  result["received_at_s"] = event.received_at_s;
#define EVENT_OPTIONAL(field) if (event.field) result[#field] = *event.field
  EVENT_OPTIONAL(reason); EVENT_OPTIONAL(task_id); EVENT_OPTIONAL(source_epoch);
  EVENT_OPTIONAL(source_id); EVENT_OPTIONAL(authority_epoch); EVENT_OPTIONAL(source_time_s);
  EVENT_OPTIONAL(requested_v_m_s); EVENT_OPTIONAL(requested_yaw_rad_s);
  EVENT_OPTIONAL(limited_v_m_s); EVENT_OPTIONAL(limited_yaw_rad_s); EVENT_OPTIONAL(saturated);
  EVENT_OPTIONAL(mode);
#undef EVENT_OPTIONAL
  return result;
}

py::dict snapshot_dict(const AuthoritySnapshot& value, bool include_events) {
  py::dict result;
  result["task_id"] = py::cast(value.task_id); result["source_epoch"] = py::cast(value.source_epoch);
  result["authority_epoch"] = value.authority_epoch; result["reason"] = value.reason;
  result["mcu_state"] = py::cast(value.mcu_state); result["mcu_reason"] = value.mcu_reason;
  result["map_id"] = value.map_id; result["map_version"] = value.map_version;
  result["authorized"] = value.authorized; result["retained_events"] = value.retained_events;
  result["recovery_locked"] = value.recovery_locked;
  result["mcu_recovery_owned"] = value.mcu_recovery_owned;
  result["retained_watermarks"] = value.retained_watermarks;
  result["retained_stop_reasons"] = value.retained_stop_reasons;
  result["events_overwritten"] = value.events_overwritten;
  result["capacity_rejections"] = value.capacity_rejections;
  result["safety_capacity_bypasses"] = value.safety_capacity_bypasses;
  result["mode"] = value.mode; result["follow_enabled"] = value.follow_enabled;
  result["follow_health"] = value.follow_health; result["follow_reason"] = value.follow_reason;
  result["master_locked"] = value.master_locked;
  result["follow_source_epoch"] = py::cast(value.follow_source_epoch);
  result["follow_sequence"] = py::cast(value.follow_sequence);
  result["follow_source_time_s"] = py::cast(value.follow_source_time_s);
  result["follow_received_at_s"] = py::cast(value.follow_received_at_s);
  result["follow_source_age_s"] = py::cast(value.follow_source_age_s);
  result["follow_receipt_age_s"] = py::cast(value.follow_receipt_age_s);
  result["follow_health_rejections"] = value.follow_health_rejections;
  result["follow_command_rejections"] = value.follow_command_rejections;
  result["granted_source_time_s"] = py::cast(value.granted_source_time_s);
  result["follow_obstacle_clear"] = value.follow_obstacle_clear;
  result["follow_safety_reason"] = value.follow_safety_reason;
  result["follow_guard_rejections"] = value.follow_guard_rejections;
#define GUARD_OPTIONAL(field) result[#field] = py::cast(value.field)
  GUARD_OPTIONAL(follow_obstacle_clearance_m); GUARD_OPTIONAL(follow_required_clearance_m);
  GUARD_OPTIONAL(follow_obstacle_source_time_s); GUARD_OPTIONAL(follow_obstacle_received_at_s);
  GUARD_OPTIONAL(follow_obstacle_source_age_s); GUARD_OPTIONAL(follow_obstacle_receipt_age_s);
  GUARD_OPTIONAL(follow_braking_received_at_s); GUARD_OPTIONAL(follow_braking_age_s);
#undef GUARD_OPTIONAL
  if (include_events) {
    py::list events;
    for (const auto& event : value.events) events.append(event_dict(event));
    result["events"] = std::move(events);
  }
  return result;
}
}

void bind_motion_authority(py::module_& module) {
  py::class_<FollowAuthorityConfig>(module, "FollowAuthorityConfig")
      .def(py::init([](double velocity, double yaw, double command_timeout, double health_timeout,
                      double obstacle_timeout, double braking_timeout) {
        return FollowAuthorityConfig{velocity, yaw, command_timeout, health_timeout, obstacle_timeout, braking_timeout};
      }), py::arg("max_velocity_m_s"), py::arg("max_yaw_rate_rad_s"),
          py::arg("command_timeout_s"), py::arg("health_timeout_s"),
          py::arg("obstacle_timeout_s"), py::arg("braking_timeout_s"))
      .def_readonly("max_velocity_m_s", &FollowAuthorityConfig::max_velocity_m_s)
      .def_readonly("max_yaw_rate_rad_s", &FollowAuthorityConfig::max_yaw_rate_rad_s)
      .def_readonly("command_timeout_s", &FollowAuthorityConfig::command_timeout_s)
      .def_readonly("health_timeout_s", &FollowAuthorityConfig::health_timeout_s)
      .def_readonly("obstacle_timeout_s", &FollowAuthorityConfig::obstacle_timeout_s)
      .def_readonly("braking_timeout_s", &FollowAuthorityConfig::braking_timeout_s);
  py::class_<MotionIntent>(module, "AuthorityMotionIntent")
      .def_readonly("source_time_s", &MotionIntent::source_time_s)
      .def_readonly("linear_velocity_target_m_s", &MotionIntent::linear_velocity_target_m_s)
      .def_readonly("yaw_rate_target_rad_s", &MotionIntent::yaw_rate_target_rad_s);
  py::class_<MotionAuthority, std::shared_ptr<MotionAuthority>>(module, "MotionAuthority")
      .def(py::init<std::string, std::string, double, double, double, double, std::optional<FollowAuthorityConfig>>(),
           py::arg("map_id"), py::arg("map_version"), py::arg("max_velocity_m_s") = .25,
           py::arg("max_yaw_rate_rad_s") = .5, py::arg("command_timeout_s") = .3,
           py::arg("health_timeout_s") = .35, py::arg("follow_config") = py::none())
      .def_readonly("map_id", &MotionAuthority::map_id)
      .def_readonly("map_version", &MotionAuthority::map_version)
      .def_readonly("max_velocity_m_s", &MotionAuthority::max_velocity_m_s)
      .def_readonly("max_yaw_rate_rad_s", &MotionAuthority::max_yaw_rate_rad_s)
      .def_readonly("command_timeout_s", &MotionAuthority::command_timeout_s)
      .def_readonly("health_timeout_s", &MotionAuthority::health_timeout_s)
      .def("control", [](MotionAuthority& owner, const std::string& task_id,
                         const std::string& source_epoch, std::uint64_t sequence,
                         const std::string& command, double lease_duration_s,
                         const std::string& map_id, const std::string& map_version,
                         double source_now_s, double receipt_now_s) {
        AuthorityAnswer answer;
        {
          py::gil_scoped_release release;
          answer = owner.control({task_id, source_epoch, sequence, command,
                                  lease_duration_s, map_id, map_version,
                                  source_now_s, receipt_now_s});
        }
        py::dict result;
        result["accepted"] = answer.accepted; result["reason"] = answer.reason;
        result["authority_epoch"] = answer.authority_epoch;
        return result;
      }, py::arg("task_id"), py::arg("source_epoch"), py::arg("sequence"),
         py::arg("command"), py::arg("lease_duration_s"), py::arg("map_id"),
         py::arg("map_version"), py::arg("source_now_s"), py::arg("receipt_now_s"))
      .def("update_health", &MotionAuthority::update_health, py::arg("available"),
           py::arg("reason"), py::arg("receipt_now_s"), py::call_guard<py::gil_scoped_release>())
      .def("update_mcu_status", &MotionAuthority::update_mcu_status, py::arg("state"),
           py::arg("reason"), py::arg("receipt_now_s"), py::call_guard<py::gil_scoped_release>())
      .def("update_mcu_status_with_provenance", &MotionAuthority::update_mcu_status_with_provenance,
           py::arg("state"), py::arg("reason"), py::arg("task_owned_stop"),
           py::arg("receipt_now_s"), py::call_guard<py::gil_scoped_release>())
      .def("receive_cmd", &MotionAuthority::receive_cmd, py::arg("linear_m_s"),
           py::arg("yaw_rad_s"), py::arg("source_time_s"), py::arg("source_now_s"),
           py::arg("receipt_now_s"), py::arg("source_id") = "nav2:controller_server",
           py::call_guard<py::gil_scoped_release>())
      .def("update_follow_health", &MotionAuthority::update_follow_health,
           py::arg("available"), py::arg("reason"), py::arg("observation_epoch"),
           py::arg("sequence"), py::arg("source_time_s"), py::arg("source_now_s"), py::arg("receipt_now_s"),
           py::call_guard<py::gil_scoped_release>())
      .def("receive_follow_cmd", &MotionAuthority::receive_follow_cmd,
           py::arg("task_id"), py::arg("task_source_epoch"), py::arg("authority_epoch"),
           py::arg("observation_epoch"), py::arg("observation_sequence"),
           py::arg("source_time_s"), py::arg("source_now_s"), py::arg("receipt_now_s"),
           py::arg("linear_m_s"), py::arg("yaw_rad_s"), py::call_guard<py::gil_scoped_release>())
      .def("update_follow_obstacle", &MotionAuthority::update_follow_obstacle,
           py::arg("clearance_m"), py::arg("source_time_s"), py::arg("source_now_s"),
           py::arg("receipt_now_s"), py::call_guard<py::gil_scoped_release>())
      .def("update_follow_braking", &MotionAuthority::update_follow_braking,
           py::arg("required_clearance_m"), py::arg("receipt_now_s"), py::call_guard<py::gil_scoped_release>())
      .def("invalidate_follow", &MotionAuthority::invalidate_follow,
           py::arg("reason"), py::arg("receipt_now_s"), py::call_guard<py::gil_scoped_release>())
      .def("invalidate_navigation", &MotionAuthority::invalidate_navigation,
           py::arg("reason"), py::arg("receipt_now_s"), py::call_guard<py::gil_scoped_release>())
      .def("sample", &MotionAuthority::sample, py::arg("source_now_s"),
           py::arg("receipt_now_s"), py::call_guard<py::gil_scoped_release>())
      .def("stop", &MotionAuthority::stop, py::arg("reason"), py::arg("receipt_now_s"),
           py::arg("force") = false, py::call_guard<py::gil_scoped_release>())
      .def("take_stop", &MotionAuthority::take_stop, py::call_guard<py::gil_scoped_release>())
      .def("snapshot", [](const MotionAuthority& owner, bool include_events) {
        AuthoritySnapshot value;
        { py::gil_scoped_release release; value = owner.snapshot(include_events); }
        return snapshot_dict(value, include_events);
      }, py::arg("include_events") = true);
}
