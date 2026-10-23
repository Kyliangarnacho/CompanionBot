"""Host binding of the exact C sources linked into STM32 firmware.

No compilation, ROS, simulator or oracle access at import/run time. The Python
implementations remain explicit numerical references. Build with build_ros.sh or
point COMPANIONBOT_CORE_LIBRARY at a separately built shared library.
"""
from __future__ import annotations
import ctypes as C
from functools import lru_cache
import os
from pathlib import Path
import numpy as np
from companionbot_contracts.paths import repository_root
from companionbot_control.core.yaw_control import (
    RelativeYawEstimate, WheelTorqueAllocation, YawPDCommand,
)
from companionbot_control.core.safety_brake import SafetyReference
from companionbot_control.estimation.longitudinal_estimation import LongitudinalEstimate

D4 = C.c_double * 4
D3 = C.c_double * 3
D2 = C.c_double * 2

class Reference(C.Structure):
    _fields_ = [(x, C.c_double) for x in ("p", "v", "a", "theta", "theta_dot", "u_ff")]

class BrakeConfig(C.Structure):
    _fields_ = [(x, C.c_double) for x in ("dt", "deceleration", "jerk", "handoff")]

class BrakeState(C.Structure):
    _fields_ = [("config", BrakeConfig), ("initial", Reference), ("current", Reference),
                ("step", C.c_uint32), ("settled", C.c_int)]

class Allocation(C.Structure):
    _fields_ = [(x, C.c_double) for x in (
        "u_sum_requested", "u_sum_limited", "u_diff_requested", "u_diff_available",
        "u_diff_used", "left", "right")] + [(x, C.c_int) for x in (
        "sum_clipped", "differential_clipped", "guard_clipped")]

class EstimatorConfig(C.Structure):
    _fields_ = [(x, C.c_double) for x in (
        "dt", "pitch_tau", "pll_bandwidth", "counts_per_rev", "wheel_radius",
        "acceleration_tau", "acceleration_scale")] + [(x, C.c_int) for x in (
        "left_sign", "right_sign", "snap_to_zero", "acceleration_compensation")]

class Sensor(C.Structure):
    _fields_ = [("accel", D3), ("gyro", D3), ("age", C.c_double),
                ("counts", C.c_int64 * 2), ("extrapolate", C.c_int)]

class Estimate(C.Structure):
    _fields_ = [(x, C.c_double) for x in (
        "theta", "theta_dot", "theta_acc", "theta_measurement", "age",
        "extrapolation_age", "position", "velocity")] + [
        (x, D2) for x in ("pll_position", "pll_velocity", "wheel_angle", "wheel_velocity")
    ] + [(x, C.c_double) for x in (
        "acceleration_used", "acceleration_next", "raw_theta_acc", "correction_gain")
    ] + [("compensated_accel", D3)]

class YawEstimatorState(C.Structure):
    _fields_ = [(x, C.c_double) for x in ("psi", "rate", "sample_time")] + [("initialized", C.c_int)]

class McuConfig(C.Structure):
    _fields_ = [("brake", BrakeConfig)] + [(x, C.c_double) for x in (
        "heartbeat_timeout", "starvation_margin", "nominal_pitch", "yaw_timeout", "yaw_decay")]

class McuOutput(C.Structure):
    _fields_ = [("reference", Reference), ("yaw_rate", C.c_double),
                ("block_sequence", C.c_int64), ("sample_index", C.c_int64)] + [
        (x, C.c_uint32) for x in ("epoch", "safety_state", "reason", "pi_lost")]

class Frame(C.Structure):
    _fields_ = [("type", C.c_uint8), ("length", C.c_uint16)] + [
        (x, C.c_uint32) for x in ("session", "sequence", "tick")] + [("payload", C.c_uint8 * 672)]

class Parser(C.Structure):
    _fields_ = [("bytes", C.c_uint8 * 696), ("used", C.c_size_t)] + [
        (x, C.c_uint32) for x in ("accepted", "rejected", "overflows")] + [("dropping", C.c_int)]

@lru_cache(maxsize=1)
def library():
    explicit = os.environ.get("COMPANIONBOT_CORE_LIBRARY")
    candidates = [Path(explicit)] if explicit else []
    if not explicit:
        try:
            from ament_index_python.packages import get_package_prefix
            candidates.append(Path(get_package_prefix("companionbot_control")) / "lib/libcompanionbot_core.so")
        except (ImportError, LookupError):
            pass
        candidates.append(repository_root() / "ros2_ws/install/companionbot_control/lib/libcompanionbot_core.so")
    path = next((p for p in candidates if p.is_file()), None)
    if path is None:
        raise RuntimeError("Build companionbot_control first; no native library found: " + str(candidates))
    lib = C.CDLL(str(path.resolve()))
    def bind(name, args, result=C.c_int):
        fn = getattr(lib, name); fn.argtypes = args; fn.restype = result
    P = C.POINTER
    bind("cb_core_abi_version", [], C.c_uint32)
    bind("cb_core_abi_size", [C.c_uint], C.c_size_t)
    if lib.cb_core_abi_version() != 0x00010000:
        raise RuntimeError("Native ABI version mismatch; rebuild companionbot_control")
    for index, structure in enumerate((Reference, BrakeConfig, BrakeState, Allocation,
            EstimatorConfig, Sensor, Estimate, YawEstimatorState, McuConfig, McuOutput, Frame, Parser), 1):
        if lib.cb_core_abi_size(index) != C.sizeof(structure):
            raise RuntimeError(f"Native ABI size mismatch: {structure.__name__}")
    bind("cb_feedback", [P(C.c_double), P(C.c_double), P(C.c_double)])
    bind("cb_yaw_pd", [C.c_double] * 6 + [P(C.c_double)])
    bind("cb_allocate", [C.c_double] * 3 + [P(Allocation)])
    bind("cb_brake_init", [P(BrakeState), P(BrakeConfig), P(Reference)])
    bind("cb_brake_step", [P(BrakeState), P(Reference)])
    bind("cb_brake_handoff_done", [P(BrakeState)])
    bind("cb_estimator_size", [], C.c_size_t)
    bind("cb_estimator_init", [C.c_void_p, P(EstimatorConfig)])
    for name in ("cb_estimator_reset", "cb_estimator_update"):
        bind(name, [C.c_void_p, P(Sensor), P(Estimate)])
    bind("cb_yaw_estimate", [P(YawEstimatorState)] + [C.c_double] * 3 + [C.c_int] * 3 + [P(C.c_double)])
    bind("cb_mcu_size", [], C.c_size_t)
    bind("cb_mcu_init", [C.c_void_p, P(McuConfig), C.c_uint32])
    bind("cb_mcu_reference", [C.c_void_p, C.c_void_p, C.c_size_t, C.c_uint32])
    bind("cb_mcu_heartbeat", [C.c_void_p, C.c_uint32, C.c_uint32])
    bind("cb_mcu_yaw", [C.c_void_p, C.c_uint32, C.c_double, C.c_uint32])
    bind("cb_mcu_safety", [C.c_void_p, C.c_uint32, C.c_int, C.c_int, C.c_uint32])
    bind("cb_mcu_resume", [C.c_void_p, C.c_uint32, C.c_uint32, C.c_int, C.c_uint32])
    bind("cb_mcu_step", [C.c_void_p, C.c_uint32] + [C.c_double] * 3 + [P(McuOutput)])
    bind("cb_mcu_peek", [C.c_void_p, P(Reference)])
    bind("cb_frame_encode", [P(Frame), C.c_void_p, C.c_size_t], C.c_size_t)
    bind("cb_parser_feed", [P(Parser), C.c_uint8, P(Frame)])
    bind("cb_mcu_dispatch", [C.c_void_p, P(Frame), C.c_uint32, C.c_uint32])
    return lib

def checked(result):
    if result:
        raise ValueError(f"native control rejected input: {result}")

def feedback(gain, error):
    gain, error = np.asarray(gain, dtype=float).reshape(-1), np.asarray(error, dtype=float)
    if gain.shape != (4,) or error.shape != (4,):
        raise ValueError("native LQR requires four gains and four error states")
    result = C.c_double()
    checked(library().cb_feedback(D4(*gain), D4(*error), C.byref(result)))
    return result.value

def allocate(u_sum, u_diff, limit):
    out = Allocation()
    checked(library().cb_allocate(u_sum, u_diff, limit, C.byref(out)))
    return WheelTorqueAllocation(out.u_sum_limited, out.u_diff_requested, out.u_diff_available,
        out.u_diff_used, out.left, out.right, bool(out.differential_clipped), bool(out.guard_clipped))

class NativeYawPD:
    def __init__(self, K_psi_nm_per_rad, K_r_nm_per_rad_s):
        self.K_psi, self.K_r = float(K_psi_nm_per_rad), float(K_r_nm_per_rad_s)
        self.command(0, 0, 0, 0)  # validate configuration immediately
    def command(self, psi_ref_rad, r_ref_rad_s, psi_hat_rad, r_hat_rad_s):
        out = D3()
        checked(library().cb_yaw_pd(self.K_psi, self.K_r, psi_ref_rad, r_ref_rad_s,
                                    psi_hat_rad, r_hat_rad_s, out))
        return YawPDCommand(*out)

class NativeYawEstimator:
    def __init__(self):
        self.state = YawEstimatorState()
    def _run(self, rate, time, age, valid, extrapolate, reset):
        out = D3()
        checked(library().cb_yaw_estimate(C.byref(self.state), rate, time, age,
            int(valid), int(extrapolate), int(reset), out))
        return RelativeYawEstimate(out[0], out[1], self.state.rate, self.state.sample_time,
                                   age, bool(out[2]))
    def reset(self, r_hat_rad_s, sample_time_s, measurement_age_s, *, allow_extrapolation):
        return self._run(r_hat_rad_s, sample_time_s, measurement_age_s, True, allow_extrapolation, True)
    def update(self, r_hat_rad_s, sample_time_s, measurement_age_s, *, valid_new_sample, allow_extrapolation):
        return self._run(r_hat_rad_s, sample_time_s, measurement_age_s, valid_new_sample, allow_extrapolation, False)

class NativeBrake:
    def __init__(self, config, initial):
        self.config, self.initial, self.current = config, initial, initial
        self.state = BrakeState()
        c = BrakeConfig(config.dt_s, config.safe_max_deceleration_m_s2,
                        config.safe_max_jerk_m_s3, config.hidden_reference_handoff_time_s)
        r = Reference(initial.p_m, initial.v_m_s, initial.a_m_s2, initial.theta_rad,
                      initial.theta_dot_rad_s, initial.u_ff_nm)
        checked(library().cb_brake_init(C.byref(self.state), C.byref(c), C.byref(r)))
    def step(self):
        out = Reference()
        checked(library().cb_brake_step(C.byref(self.state), C.byref(out)))
        self.current = SafetyReference(*(getattr(out, x) for x, _ in Reference._fields_))
        return self.current
    @property
    def step_index(self): return self.state.step
    @property
    def velocity_settled(self): return bool(self.state.settled)
    @property
    def hidden_handoff_done(self): return bool(library().cb_brake_handoff_done(C.byref(self.state)))

class NativeLongitudinalEstimator:
    def __init__(self, config, encoder_profile, wheel_radius_m, pll_bandwidth_rad_s=None,
                 *, acceleration_compensation=None):
        self.config, self.acceleration_compensation = config, acceleration_compensation
        comp = acceleration_compensation
        c = EstimatorConfig(config.pitch.sample_period_s, config.pitch.time_constant_s,
            config.encoder_pll_bandwidth_rad_s if pll_bandwidth_rad_s is None else pll_bandwidth_rad_s,
            encoder_profile.output_decoded_counts_per_rev, wheel_radius_m,
            comp.acceleration_time_constant_s if comp else 0.05,
            comp.adaptive_acceleration_scale_m_s2 if comp else 0.25,
            encoder_profile.left_count_sign, encoder_profile.right_count_sign,
            int(config.encoder_pll_snap_to_zero), int(comp is not None))
        self.state = C.create_string_buffer(library().cb_estimator_size())
        self.output = Estimate()
        checked(library().cb_estimator_init(self.state, C.byref(c)))
    def _run(self, reset, accel, gyro, counts, age, extrapolate):
        if np.shape(accel) != (3,) or np.shape(gyro) != (3,) or np.shape(counts) != (2,):
            raise ValueError("native sensors require accel[3], gyro[3], counts[2]")
        s = Sensor(D3(*accel), D3(*gyro), age, (C.c_int64 * 2)(*map(int, counts)), int(extrapolate))
        fn = library().cb_estimator_reset if reset else library().cb_estimator_update
        checked(fn(self.state, C.byref(s), C.byref(self.output)))
        o = self.output
        return LongitudinalEstimate(
            theta_hat_rad=o.theta, theta_dot_hat_rad_s=o.theta_dot, theta_acc_rad=o.theta_acc,
            theta_measurement_time_rad=o.theta_measurement, theta_dot_measurement_time_rad_s=o.theta_dot,
            imu_measurement_age_s=o.age, imu_extrapolation_age_s=o.extrapolation_age,
            imu_kinematic_extrapolation_applied=o.extrapolation_age > 0,
            position_hat_m=o.position, velocity_hat_m_s=o.velocity,
            wheel_pll_position_counts=np.array(o.pll_position), wheel_pll_velocity_counts_s=np.array(o.pll_velocity),
            wheel_relative_angle_hat_rad=np.array(o.wheel_angle), wheel_relative_velocity_hat_rad_s=np.array(o.wheel_velocity))
    def reset(self, accelerometer_m_s2, gyro_rad_s, encoder_counts, *, measurement_age_s=0,
              allow_kinematic_extrapolation=True):
        return self._run(True, accelerometer_m_s2, gyro_rad_s, encoder_counts, measurement_age_s, allow_kinematic_extrapolation)
    def update(self, accelerometer_m_s2, gyro_rad_s, encoder_counts, *, measurement_age_s=0,
               allow_kinematic_extrapolation=True):
        return self._run(False, accelerometer_m_s2, gyro_rad_s, encoder_counts, measurement_age_s, allow_kinematic_extrapolation)
    def acceleration_compensation_log_fields(self):
        o = self.output
        return {"odometry_acceleration_used_m_s2": o.acceleration_used,
            "odometry_acceleration_lpf_next_m_s2": o.acceleration_next,
            "theta_acc_raw_rad": o.raw_theta_acc, "theta_acc_compensated_rad": o.theta_acc,
            "compensated_accelerometer_m_s2": list(o.compensated_accel),
            "accel_correction_gain_factor": o.correction_gain,
            "acceleration_estimate_causal_delay_s": self.config.pitch.sample_period_s}

class NativeMCU:
    """Opaque bounded C runtime; transport clocks/ticks are explicit inputs."""
    def __init__(self, config, epoch=0):
        self.state = C.create_string_buffer(library().cb_mcu_size())
        self.output = McuOutput()
        checked(library().cb_mcu_init(self.state, C.byref(config), epoch))
    def receive(self, channel, payload, tick):
        from companionbot_contracts.packets import (
            HeartbeatPacket, YawCommandPacket, SafetyCommandPacket, ResumeReferenceCommandPacket)
        lib = library()
        if channel == "reference":
            return lib.cb_mcu_reference(self.state, C.c_char_p(payload), len(payload), tick)
        if channel == "heartbeat":
            p = HeartbeatPacket.from_bytes(payload)
            return lib.cb_mcu_heartbeat(self.state, p.sequence_id, tick)
        if channel == "yaw":
            p = YawCommandPacket.from_bytes(payload)
            return lib.cb_mcu_yaw(self.state, p.sequence_id, p.yaw_rate_target_rad_s, tick)
        if channel == "safety":
            p = SafetyCommandPacket.from_bytes(payload)
            return lib.cb_mcu_safety(self.state, p.sequence_id, int(p.action == "LONGITUDINAL_STOP"), 1, tick)
        if channel == "resume":
            p = ResumeReferenceCommandPacket.from_bytes(payload)
            return lib.cb_mcu_resume(self.state, p.sequence_id, p.reference_epoch, int(p.authority == "LONGITUDINAL"), tick)
        raise ValueError(channel)
    def step(self, tick, velocity, pitch, pitch_rate):
        checked(library().cb_mcu_step(self.state, tick, velocity, pitch, pitch_rate, C.byref(self.output)))
        return self.output
    def peek(self):
        out = Reference()
        checked(library().cb_mcu_peek(self.state, C.byref(out)))
        return out
