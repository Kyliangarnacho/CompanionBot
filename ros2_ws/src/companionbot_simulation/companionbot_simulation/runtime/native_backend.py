"""Explicit headless acceptance adapter, never part of the formal interaction.

C outputs drive the plant. Frozen Python receiver/safety runs as a checked
diagnostic mirror to preserve its rich event history and Pi scenario machinery.
Each tick must agree before applying output; disagreement stops the experiment.
The STM32 firmware links only C and has no Python dependency/mirror.
"""
from contextlib import ExitStack, contextmanager
from dataclasses import replace
from unittest.mock import patch
import numpy as np
from companionbot_control.runtime import native as n
from companionbot_control.runtime.mcu import MCUSafetyArbiter

class CheckedNativeArbiter(MCUSafetyArbiter):
    instances = []
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        yaw = self.yaw_source
        if yaw is None:
            raise ValueError("C arbitration acceptance needs the split-yaw contract")
        b = self.brake_config
        config = n.McuConfig(n.BrakeConfig(b.dt_s, b.safe_max_deceleration_m_s2,
            b.safe_max_jerk_m_s3, b.hidden_reference_handoff_time_s), self.heartbeat_timeout_s,
            self.starvation_margin_ticks * b.dt_s, self.nominal_pitch_rad,
            yaw.timeout_ticks * b.dt_s, yaw.decay_step / b.dt_s)
        self.native = n.NativeMCU(config, self.receiver.reference_epoch)
        self.checked_ticks = 0
        self.max_reference_delta = 0.
        self.native_rejections = {}
        self.instances.append(self)
    def receive(self, channel, payload, time_s):
        super().receive(channel, payload, time_s)
        result = self.native.receive(channel, payload, round(time_s / self.receiver.dt_s))
        if result:
            self.native_rejections[result] = self.native_rejections.get(result, 0) + 1
    def command(self, time_s):
        original = super().command(time_s)
        out = self.native.step(round(time_s / self.receiver.dt_s), self.local_v_hat_m_s,
                               self.local_pitch_hat_rad, self.local_pitch_rate_hat_rad_s)
        values = np.array([out.reference.p, out.reference.v, out.reference.a,
            out.reference.theta, out.reference.theta_dot, out.reference.u_ff, out.yaw_rate])
        expected = np.array([original.reference_state[0], original.reference_state[1],
            original.reference_acceleration_m_s2, original.reference_state[2],
            original.reference_state[3], original.u_ff_after_lifecycle_nm, original.yaw_rate_target_rad_s])
        delta = float(np.max(np.abs(values - expected)))
        self.max_reference_delta = max(self.max_reference_delta, delta)
        if not np.isfinite(values).all() or delta > 2e-12:
            raise RuntimeError(f"C/Python reference mismatch at {time_s}: {delta}")
        state = ("NORMAL", "BRAKING", "LONGITUDINAL_STOP", "SAFE_HOLD")[out.safety_state]
        if state != self.state or out.epoch != self.receiver.reference_epoch:
            raise RuntimeError(f"C/Python authority mismatch at {time_s}: {state} / {self.state}")
        self.checked_ticks += 1
        return replace(original, reference_state=values[[0, 1, 3, 4]],
            reference_acceleration_m_s2=out.reference.a, u_ff_raw_nm=out.reference.u_ff,
            u_ff_after_lifecycle_nm=out.reference.u_ff, linear_velocity_target_m_s=out.reference.v,
            yaw_rate_target_rad_s=out.yaw_rate)
    @property
    def next_reference_state(self):
        out = self.native.peek()
        return np.array([out.p, out.v, out.theta, out.theta_dot])
    @property
    def next_reference_acceleration_m_s2(self):
        return self.native.peek().a

@contextmanager
def native_backend(parts=("estimator", "feedback", "yaw", "arbitration")):
    """Install only for an explicitly requested experiment and restore on exit."""
    import companionbot_simulation.scenarios.run_stage3b_yaw_control as motion
    import companionbot_simulation.scenarios.run_stage6_4_2d_follow as two_d
    import companionbot_simulation.scenarios.run_stage6_5_radial_kf as radial
    import companionbot_simulation.scenarios.view_stage6_5_manual_2d_follow as dynamic
    original_run = motion.run_case
    CheckedNativeArbiter.instances = []
    def run(*args, **kwargs):
        kwargs["feedback_function"] = n.feedback
        return original_run(*args, **kwargs)
    with ExitStack() as stack:
        patches={}
        if "estimator" in parts: patches["LongitudinalEstimator"]=n.NativeLongitudinalEstimator
        if "feedback" in parts: patches["run_case"]=run
        if "yaw" in parts:
            patches.update({"GyroRelativeYawEstimator":n.NativeYawEstimator,
                "ParallelYawPD":n.NativeYawPD,"allocate_longitudinal_priority":n.allocate})
        for name,value in patches.items():stack.enter_context(patch.object(motion,name,value))
        if "arbitration" in parts:
            for module in (two_d,radial,dynamic):
                stack.enter_context(patch.object(module,"MCUSafetyArbiter",CheckedNativeArbiter))
        yield

def parity_summary():
    return {"kind": "C_drives_plant_Python_arbitration_diagnostic_mirror",
        "arbiters": len(CheckedNativeArbiter.instances),
        "checked_control_ticks": sum(x.checked_ticks for x in CheckedNativeArbiter.instances),
        "max_reference_delta": max((x.max_reference_delta for x in CheckedNativeArbiter.instances), default=0),
        "native_rejections": [x.native_rejections for x in CheckedNativeArbiter.instances],
        "core_library": n.library()._name,
        "real_hardware_verified": False}
