"""Composition boundary between the frozen MuJoCo runner and native motion.

C++ owns Pi scheduling, reference lifecycle, bounded transport and the shared
C99 software MCU. Python passes consumed estimator scalars once per control
tick and converts the result to the runner's existing DTO. Task/ROS callbacks
share the same native authority; no DDS round trip occurs here.
"""
import json
import math

import numpy as np

from companionbot_contracts.paths import repository_root
from companionbot_control.runtime.mcu import WireReferenceCommand
from companionbot_follow import _native_runtime as follow_native
from companionbot_simulation import _native_simulation as native


class NativeNavigationReference:
    """Runner adapter only; contains no motion or recovery state machine."""
    def __init__(self, adapter, producer_config, mcu_config, latency_s, *, common=None, stream=None):
        self.adapter = adapter
        self.dt_s = producer_config.dt
        follow_controller = getattr(adapter, 'follow_controller', None)
        if follow_controller is None:
            self.runtime = native.NavigationRuntime(adapter.native_authority,
                producer_config, mcu_config, latency_s)
        else:
            from companionbot_follow.runtime.native_follow import (
                producer_configuration, full_solver, clearance_configuration)
            clearance = clearance_configuration(common)
            self.runtime = native.NavigationRuntime(adapter.native_authority,
                producer_configuration(common,stream,producer_config), mcu_config, latency_s,
                full_solver(common,stream),follow_controller,
                native.FollowClearanceConfig(clearance['radius_m'],clearance['reaction_time_s'],
                    clearance['maximum_projection_steps']))
        self._local = None
        self._output = None
        self._tick = None

    def mcu_local_state(self, time_s, estimate, yaw_estimate):
        # Same control-time sensorized estimate as the frozen C/Python core.
        self._local = (float(estimate.velocity_hat_m_s), float(estimate.theta_hat_rad),
                       float(estimate.theta_dot_hat_rad_s))

    def before_control_tick(self, time_s):
        if self._local is None:
            raise RuntimeError("native_navigation_requires_local_estimator_sample")
        self._tick = round(time_s / self.dt_s)
        self._output = self.runtime.step(self._tick, self.adapter.clock(), *self._local)
        self._local = None

    def command(self, time_s):
        if self._output is None or round(time_s / self.dt_s) != self._tick:
            raise RuntimeError("native_navigation_command_without_matching_control_tick")
        out = self._output
        ref = out.reference
        normal = out.safety_state == 0
        return WireReferenceCommand(
            np.asarray([ref.p, ref.v, ref.theta, ref.theta_dot]), ref.a,
            ref.u_ff, ref.u_ff, ref.v, out.yaw_rate,
            "WIRE" if normal else "SAFETY", "WIRE" if normal else self.state,
            1., out.block_sequence, out.sample_index, out.block_start_time_s,
            out.intent_source_time_s, planning_path="WIRE_NORMAL" if normal else "SAFETY")

    @property
    def next_reference_state(self):
        ref = self._output.next_reference
        return np.asarray([ref.p, ref.v, ref.theta, ref.theta_dot])

    @property
    def next_reference_acceleration_m_s2(self):
        return self._output.next_reference.a

    @property
    def state(self):
        return native.NavigationRuntime.state_name(
            self._output.safety_state if self._output is not None else self.runtime.safety_state)

    @property
    def reason(self):
        return self.runtime.reason_name(
            self._output.reason if self._output is not None else self.runtime.reason)

    def report(self):
        return self.runtime.report()


def build_navigation_reference(common, stage5, config, adapter):
    """Load canonical limits; instantiate the sole production navigation chain."""
    threshold = float(config["scheduler"]["T_full_delta_v_m_s"])
    if adapter.max_velocity_m_s >= threshold:
        raise ValueError("navigation_envelope_requires_review_of_full_planner_lock")
    if config["scheduler"]["initial_accepted_velocity_m_s"] != 0.:
        raise ValueError("navigation_startup_requires_stationary_reference")
    _, _, dynamic_config, motion_config, *_ = common
    limits = motion_config["reference_limits"]
    producer = follow_native.ProducerConfig()
    producer.dt = float(motion_config["controller_dt_s"])
    producer.block_samples = int(config["reference_block"]["sample_count"])
    if not math.isclose(producer.dt, config["reference_block"]["dt_s"], abs_tol=1e-12):
        raise ValueError("canonical_reference_period_mismatch")
    producer.max_velocity = float(limits["max_velocity_m_s"])
    producer.max_acceleration = float(limits["max_acceleration_m_s2"])
    producer.max_jerk = float(dynamic_config["max_jerk_m_s3"])
    producer.full_delta_velocity = threshold
    path = repository_root() / "models/minisegway/stage6/config/stage6_4_2d_follow_config.json"
    baseline = json.loads(path.read_text())
    safety, yaw = baseline["safety"], baseline["yaw"]
    brake = native.BrakeConfig(producer.dt, safety["safe_max_deceleration_m_s2"],
        safety["safe_max_jerk_m_s3"], safety["hidden_reference_handoff_time_s"])
    mcu = native.McuConfig(brake, safety["heartbeat_timeout_s"],
        safety["reference_starvation_margin_s"], float(common[7]["parameters"]["theta_eq_rad"]),
        yaw["freshness_timeout_s"], yaw["decay_rate_rad_s2"])
    reference = NativeNavigationReference(adapter, producer, mcu, safety["transport_latency_s"],
        common=common, stream=config)
    return reference, reference, dict(command_owner="native_cpp_motion_authority",
        scheduler_owner="native_cpp_navigation_runtime", mcu_owner="shared_native_c99",
        scheduler_policy=dict(require_stable_candidate=False, minimum_delta_enabled=False,
            follow_force_full_dynamic=False, follow_defer_until_full_exit=False),
        planner="existing_LIGHTWEIGHT_Ruckig_0.19.4_cpp", physical_output_gate="unchanged_MCU",
        safety_config_source=str(path), map_id=adapter.map_id, map_version=adapter.map_version,
        max_velocity_m_s=adapter.max_velocity_m_s, max_yaw_rate_rad_s=adapter.max_yaw_rate_rad_s,
        velocity_smoother=False, hardware_execution_ready=False)
