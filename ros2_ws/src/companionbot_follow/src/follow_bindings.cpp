#include "companionbot_follow/follow_algorithms.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace companionbot::follow;

namespace {
py::dict state_dict(const GovernorState& value) {
  py::dict result;
#define STATE_FIELD(field) result[#field] = py::cast(value.field)
  STATE_FIELD(state); STATE_FIELD(latched_velocity_m_s); STATE_FIELD(robot_velocity_hat_m_s);
  STATE_FIELD(master_velocity_hat_m_s); STATE_FIELD(closing_speed_m_s); STATE_FIELD(transition_distance_m);
  STATE_FIELD(switch_distance_m); STATE_FIELD(distance_m); STATE_FIELD(beta_rad); STATE_FIELD(distance_rate_hat_m_s);
  STATE_FIELD(robot_radial_velocity_hat_m_s); STATE_FIELD(duplicate_observation_count);
  STATE_FIELD(increasing_distance_samples); STATE_FIELD(unable_to_close_sample_count); STATE_FIELD(slowdown_samples);
  STATE_FIELD(last_sequence_id);
#undef STATE_FIELD
  return result;
}
py::dict event_dict(const GovernorEvent& value) {
  py::dict result;
  result["time_s"] = value.time_s; result["sequence_id"] = value.sequence_id;
  result["event"] = value.event; result["reason"] = value.reason;
  result["previous_state"] = value.previous_state; result["state"] = value.values.state;
#define VALUE_FIELD(field) result[#field] = value.values.field
  VALUE_FIELD(distance_m); VALUE_FIELD(beta_rad); VALUE_FIELD(robot_radial_velocity_hat_m_s);
  VALUE_FIELD(master_velocity_hat_m_s); VALUE_FIELD(robot_velocity_hat_m_s); VALUE_FIELD(closing_speed_m_s);
  VALUE_FIELD(transition_distance_m); VALUE_FIELD(switch_distance_m); VALUE_FIELD(latched_velocity_m_s);
#undef VALUE_FIELD
  result["d_dot_hat_m_s"] = value.values.distance_rate_hat_m_s;
  result["generated_velocity_m_s"] = value.generated_velocity_m_s;
  result["command_saturated"] = value.command_saturated;
#define OPTIONAL_FIELD(field) if (value.field) result[#field] = *value.field
  OPTIONAL_FIELD(single_sample_kf_master_radial_velocity_m_s); OPTIONAL_FIELD(single_sample_shadow_latched_velocity_m_s);
  OPTIONAL_FIELD(cruise_velocity_median_m_s); OPTIONAL_FIELD(cruise_velocity_long_median_m_s);
  OPTIONAL_FIELD(cruise_velocity_window_samples); OPTIONAL_FIELD(cruise_velocity_long_window_samples);
#undef OPTIONAL_FIELD
  return result;
}
py::dict observation_row(const GovernorObservationRow& row) {
  py::dict result;
  result["capture_time_s"] = row.observation.capture_time_s; result["sequence_id"] = row.observation.sequence_id;
  result["version"] = row.observation.version; result["valid"] = row.observation.valid;
  result["confidence"] = row.observation.confidence; result["x_forward_m"] = row.observation.x_forward_m;
  result["y_left_m"] = row.observation.y_left_m;
#define ROW_FIELD(field) result[#field] = row.values.field
  ROW_FIELD(distance_m); ROW_FIELD(beta_rad); ROW_FIELD(robot_radial_velocity_hat_m_s);
  ROW_FIELD(robot_velocity_hat_m_s); ROW_FIELD(master_velocity_hat_m_s); ROW_FIELD(closing_speed_m_s);
  ROW_FIELD(transition_distance_m); ROW_FIELD(switch_distance_m); ROW_FIELD(slowdown_samples);
#undef ROW_FIELD
  result["d_dot_hat_m_s"] = row.values.distance_rate_hat_m_s;
  result["master_radial_velocity_hat_m_s"] = row.values.master_velocity_hat_m_s;
  result["x_rel_dot_hat_m_s"] = py::cast(row.derivative_m_s);
  result["d2_m"] = row.d2_m; result["distance_increasing_samples"] = row.values.increasing_distance_samples;
  result["catch_command_saturated"] = row.catch_command_saturated; result["unable_to_close"] = row.unable_to_close;
  result["governor_state"] = row.values.state; result["latched_v_cmd_m_s"] = row.values.latched_velocity_m_s;
  result["command_changed"] = row.command_changed;
  if (row.slowdown_rejection_reason) result["slowdown_rejection_reason"] = *row.slowdown_rejection_reason;
  return result;
}
py::dict radial_row(const RadialObservationResult& value) {
  py::dict result;
  result["sequence_id"] = value.observation.sequence_id; result["capture_time_s"] = value.observation.capture_time_s;
  result["packet_arrival_time_s"] = value.arrival_time_s; result["d_meas_m"] = value.measured_distance_m;
  result["beta_rad"] = value.beta_rad; result["confidence"] = value.observation.confidence;
  for (const auto* name : {"aligned_robot_state_time_s", "aligned_v_robot_m_s", "kf_d_hat_m",
      "kf_v_master_radial_hat_m_s", "kf_innovation_m", "governor_output_m_s"}) result[name] = py::none();
  result["alignment"] = value.alignment;
  result["raw_radial_velocity_for_diagnostic_only_m_s"] = py::cast(value.raw_radial_velocity_diagnostic_m_s);
  if (value.accepted) {
    const auto& aligned = *value.state_alignment; const auto& estimate = *value.estimate;
    result["aligned_robot_state_time_s"] = aligned.aligned_time_s;
    result["state_left_sequence_id"] = aligned.left_sequence; result["state_right_sequence_id"] = aligned.right_sequence;
    result["state_left_time_s"] = aligned.left_time_s; result["state_right_time_s"] = aligned.right_time_s;
    result["state_interp_alpha"] = aligned.alpha; result["aligned_v_robot_m_s"] = aligned.velocity_m_s;
    result["latest_packet_v_robot_for_diagnostic_only_m_s"] = py::cast(value.latest_robot_velocity_diagnostic_m_s);
    result["kf_d_hat_m"] = estimate.distance_hat_m;
    result["kf_v_master_radial_hat_m_s"] = estimate.master_radial_velocity_hat_m_s;
    result["kf_innovation_m"] = estimate.innovation_m;
  }
  return result;
}
}

void bind_follow_algorithms(py::module_& module) {
  py::class_<FollowObservation>(module, "FollowObservation")
      .def(py::init([](double capture, double x, double y, std::uint32_t version,
                      std::uint64_t sequence, bool valid, double confidence) {
        return FollowObservation{capture, x, y, version, sequence, valid, confidence};
      }), py::arg("capture_time_s"), py::arg("x_forward_m"), py::arg("y_left_m"),
           py::arg("version") = 1, py::arg("sequence_id") = 0, py::arg("valid") = true, py::arg("confidence") = 1.)
      .def_readonly("capture_time_s", &FollowObservation::capture_time_s)
      .def_readonly("x_forward_m", &FollowObservation::x_forward_m)
      .def_readonly("y_left_m", &FollowObservation::y_left_m)
      .def_readonly("version", &FollowObservation::version)
      .def_readonly("sequence_id", &FollowObservation::sequence_id)
      .def_readonly("valid", &FollowObservation::valid)
      .def_readonly("confidence", &FollowObservation::confidence);
  py::class_<FollowRobotState>(module, "FollowRobotState")
      .def(py::init([](std::uint64_t sequence, double time, double velocity) {
        return FollowRobotState{sequence, time, velocity};
      }), py::arg("sequence_id"), py::arg("sample_time_s"), py::arg("v_hat_m_s"));
  py::class_<RadialKFConfig>(module, "RadialKFConfig")
      .def(py::init([](double initial_distance, double initial_velocity, double measurement, double acceleration) {
        return RadialKFConfig{initial_distance, initial_velocity, measurement, acceleration};
      }), py::arg("initial_distance_std"), py::arg("initial_velocity_std"),
           py::arg("measurement_distance_std"), py::arg("master_accel_std"));
  py::class_<RadialKFResult>(module, "RadialKFResult")
      .def_readonly("distance_hat_m", &RadialKFResult::distance_hat_m)
      .def_readonly("master_radial_velocity_hat_m_s", &RadialKFResult::master_radial_velocity_hat_m_s)
      .def_readonly("innovation_m", &RadialKFResult::innovation_m);
  py::class_<RadialEstimate>(module, "RadialEstimate")
      .def(py::init([](std::uint64_t sequence, double time, double distance, double velocity,
                      double robot_velocity, double beta, double innovation) {
        return RadialEstimate{sequence, time, distance, velocity, robot_velocity, beta, innovation};
      }), py::arg("sequence_id"), py::arg("capture_time_s"), py::arg("distance_hat_m"),
           py::arg("master_radial_velocity_hat_m_s"), py::arg("aligned_robot_velocity_m_s"),
           py::arg("beta_rad"), py::arg("innovation_m"))
#define ESTIMATE_FIELD(field) .def_readonly(#field, &RadialEstimate::field)
      ESTIMATE_FIELD(sequence_id) ESTIMATE_FIELD(capture_time_s) ESTIMATE_FIELD(distance_hat_m)
      ESTIMATE_FIELD(master_radial_velocity_hat_m_s) ESTIMATE_FIELD(aligned_robot_velocity_m_s)
      ESTIMATE_FIELD(beta_rad) ESTIMATE_FIELD(innovation_m);
#undef ESTIMATE_FIELD
  py::class_<RadialVelocityKalmanFilter>(module, "RadialVelocityKalmanFilter")
      .def(py::init<RadialKFConfig>())
      .def("update", &RadialVelocityKalmanFilter::update, py::arg("capture_time_s"),
           py::arg("distance_meas_m"), py::arg("robot_radial_velocity_m_s"), py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("x", &RadialVelocityKalmanFilter::x)
      .def_property_readonly("P", &RadialVelocityKalmanFilter::covariance)
      .def_property_readonly("last_capture_time_s", &RadialVelocityKalmanFilter::last_capture_time_s)
      .def_property_readonly("innovation_m", &RadialVelocityKalmanFilter::innovation_m);
  py::class_<PiRobotStateHistory>(module, "PiRobotStateHistory")
      .def(py::init<double>(), py::arg("retention_s") = 2.)
      .def("accept", &PiRobotStateHistory::accept, py::call_guard<py::gil_scoped_release>())
      .def("velocity_at", [](const PiRobotStateHistory& owner, double capture) -> py::object {
        const auto aligned = owner.velocity_at(capture);
        if (!aligned) return py::none();
        return py::make_tuple(aligned->velocity_m_s, aligned->aligned_time_s, aligned->left_sequence,
            aligned->right_sequence, aligned->left_time_s, aligned->right_time_s, aligned->alpha);
      })
      .def("snapshot", [](const PiRobotStateHistory& owner) {
        const auto value = owner.snapshot(); py::dict result;
        result["retained_samples"] = value.retained_samples; result["accepted_count"] = value.accepted_count;
        result["out_of_order_rejections"] = value.out_of_order_rejections; result["capacity_rejections"] = value.capacity_rejections;
        return result;
      });
  py::class_<RadialObservationResult>(module, "RadialObservationResult")
      .def_readonly("accepted", &RadialObservationResult::accepted)
      .def_readonly("alignment", &RadialObservationResult::alignment)
      .def_readonly("observation", &RadialObservationResult::observation)
      .def_readonly("estimate", &RadialObservationResult::estimate)
      .def_property_readonly("row", &radial_row);
  py::class_<RadialObservationPipeline>(module, "RadialObservationPipeline")
      .def(py::init<RadialKFConfig, double>(), py::arg("config"), py::arg("retention_s") = 2.)
      .def("accept_state", &RadialObservationPipeline::accept_state, py::call_guard<py::gil_scoped_release>())
      .def("accept_observation", &RadialObservationPipeline::accept_observation,
           py::arg("observation"), py::arg("arrival_time_s"), py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("latest_observation", &RadialObservationPipeline::latest_observation)
      .def_property_readonly("latest_estimate", &RadialObservationPipeline::latest_estimate)
      .def_property_readonly("kf", &RadialObservationPipeline::kf, py::return_value_policy::reference_internal)
      .def_property_readonly("state_history", &RadialObservationPipeline::state_history, py::return_value_policy::reference_internal);
  py::class_<FollowGovernorConfig>(module, "FollowGovernorConfig")
      .def(py::init([](double desired, double d1, double d2, double catch_time, double max_velocity,
                      double initial_velocity, double braking, double retrigger_margin,
                      std::uint32_t persistence, double slowdown_deadband) {
        return FollowGovernorConfig{desired, d1, d2, catch_time, max_velocity, initial_velocity,
                                    braking, retrigger_margin, persistence, slowdown_deadband};
      }), py::arg("desired_distance_m"), py::arg("d1_m"), py::arg("d2_m"), py::arg("catch_time_s"),
           py::arg("max_abs_velocity_m_s"), py::arg("initial_velocity_m_s") = 0.,
           py::arg("a_brake_effective_m_s2") = .25, py::arg("catch_retrigger_margin_m_s") = .05,
           py::arg("catch_retrigger_persistence_samples") = 3, py::arg("slowdown_velocity_deadband_m_s") = .05);
  py::class_<GovernorResult>(module, "GovernorResult")
      .def_readonly("intent", &GovernorResult::intent)
      .def_readonly("duplicate", &GovernorResult::duplicate)
      .def_property_readonly("event", [](const GovernorResult& value) -> py::object {
        return value.event ? py::object(event_dict(*value.event)) : py::none();
      })
      .def_property_readonly("row", [](const GovernorResult& value) -> py::object {
        return value.row ? py::object(observation_row(*value.row)) : py::none();
      });
  py::class_<FollowGovernor>(module, "FollowGovernor")
      .def(py::init<FollowGovernorConfig, bool>(), py::arg("config"), py::arg("use_radial_estimates") = false)
      .def("set_robot_velocity_hat", &FollowGovernor::set_robot_velocity_hat)
      .def("update", &FollowGovernor::update, py::arg("observation"), py::arg("estimate") = py::none(),
           py::call_guard<py::gil_scoped_release>())
      .def("snapshot", [](const FollowGovernor& governor) { return state_dict(governor.snapshot()); })
      .def_property_readonly("events", [](const FollowGovernor& governor) {
        py::list result; for (const auto& event : governor.events()) result.append(event_dict(event)); return result;
      })
      .def_property_readonly("events_overwritten", &FollowGovernor::events_overwritten);
  py::class_<BearingYawServo>(module, "BearingYawServo")
      .def(py::init<double, double, double>(), py::arg("gain_s_inv"), py::arg("max_rate_rad_s"), py::arg("deadband_rad") = 0.)
      .def("command", &BearingYawServo::command, py::call_guard<py::gil_scoped_release>());
  py::class_<FollowControllerResult>(module, "FollowControllerResult")
      .def_readonly("accepted", &FollowControllerResult::accepted)
      .def_readonly("command_accepted", &FollowControllerResult::command_accepted)
      .def_readonly("reason", &FollowControllerResult::reason)
      .def_readonly("authority_epoch", &FollowControllerResult::authority_epoch)
      .def_readonly("radial", &FollowControllerResult::radial)
      .def_readonly("governor", &FollowControllerResult::governor)
      .def_readonly("bearing_rad", &FollowControllerResult::bearing_rad)
      .def_readonly("yaw_rate_rad_s", &FollowControllerResult::yaw_rate_rad_s);
  py::class_<FollowController, std::shared_ptr<FollowController>>(module, "FollowController")
      .def(py::init<std::shared_ptr<MotionAuthority>, RadialKFConfig, FollowGovernorConfig,
                    double, double, double, double>(),
           py::arg("authority"), py::arg("kf_config"), py::arg("governor_config"),
           py::arg("history_retention_s"), py::arg("yaw_gain_s_inv"),
           py::arg("yaw_max_rate_rad_s"), py::arg("yaw_deadband_rad"))
      .def("accept_state", &FollowController::accept_state, py::arg("state"), py::call_guard<py::gil_scoped_release>())
      .def("accept_measurement", &FollowController::accept_measurement,
           py::arg("epoch"), py::arg("sequence"), py::arg("capture_source_s"),
           py::arg("source_now_s"), py::arg("receipt_now_s"), py::arg("x_forward_m"),
           py::arg("y_left_m"), py::arg("valid"), py::arg("reason"), py::call_guard<py::gil_scoped_release>())
      .def("snapshot", [](const FollowController& owner) {
        FollowControllerSnapshot value;
        { py::gil_scoped_release release; value = owner.snapshot(); }
        py::dict result;
        result["measurement_calls"] = value.measurement_calls;
        result["measurements_accepted"] = value.measurements_accepted;
        result["commands_accepted"] = value.commands_accepted;
        result["invalid_measurements"] = value.invalid_measurements;
        result["out_of_order_measurements"] = value.out_of_order_measurements;
        result["algorithm_resets"] = value.algorithm_resets;
        result["authority_epoch"] = py::cast(value.authority_epoch);
        result["camera_epoch"] = py::cast(value.camera_epoch);
        result["governor"] = state_dict(value.governor);
        py::dict history;
        history["retained_samples"] = value.state_history.retained_samples;
        history["accepted_count"] = value.state_history.accepted_count;
        history["out_of_order_rejections"] = value.state_history.out_of_order_rejections;
        history["capacity_rejections"] = value.state_history.capacity_rejections;
        result["state_history"] = std::move(history);
        return result;
      });
}
