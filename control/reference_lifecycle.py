"""Finite-position reference lifecycle for commanded-motion experiments."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np
from numpy.typing import NDArray


class PositionLifecyclePhase(str, Enum):
    TRACKING = "TRACKING"
    TERMINAL_HOLD = "TERMINAL_HOLD"
    GOAL_REACHED = "GOAL_REACHED"


@dataclass(frozen=True)
class FinitePositionPlan:
    goal_position_m: float
    reference_states: NDArray[np.float64]
    feedforward_inputs_nm: NDArray[np.float64]

    def __post_init__(self) -> None:
        states = np.asarray(self.reference_states, dtype=float)
        inputs = np.asarray(self.feedforward_inputs_nm, dtype=float)
        if states.ndim != 2 or states.shape[1] != 4 or len(states) < 2:
            raise ValueError("reference_states must have shape (N+1, 4)")
        if inputs.shape != (len(states) - 1,):
            raise ValueError("feedforward_inputs_nm must have shape (N,)")
        if not np.isfinite(np.r_[states.ravel(), inputs]).all():
            raise ValueError("finite-position plan must be finite")
        expected_terminal = np.asarray([self.goal_position_m, 0.0, 0.0, 0.0])
        if not np.allclose(states[-1], expected_terminal, atol=1e-9, rtol=0.0):
            raise ValueError("finite-position plan must end at [p_goal, 0, 0, 0]")


@dataclass(frozen=True)
class PositionLifecycleCommand:
    phase: PositionLifecyclePhase
    reference_state: NDArray[np.float64]
    feedforward_nm: float


class FinitePositionReferenceLifecycle:
    """Separate nominal planner completion from estimated plant acceptance."""

    def __init__(
        self,
        position_error_band_m: float,
        velocity_error_band_m_s: float,
        dwell_s: float,
    ) -> None:
        self.position_error_band_m = float(position_error_band_m)
        self.velocity_error_band_m_s = float(velocity_error_band_m_s)
        self.dwell_s = float(dwell_s)
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self.position_error_band_m,
                self.velocity_error_band_m_s,
                self.dwell_s,
            )
        ):
            raise ValueError("acceptance bands and dwell must be finite and positive")
        self._plan: FinitePositionPlan | None = None
        self._phase: PositionLifecyclePhase | None = None
        self._interval_index = 0
        self._band_enter_time_s: float | None = None
        self.planner_finished_time_s: float | None = None
        self.goal_reached_time_s: float | None = None
        self.terminal_entry_position_error_m: float | None = None
        self.terminal_entry_velocity_error_m_s: float | None = None

    @property
    def phase(self) -> PositionLifecyclePhase | None:
        return self._phase

    @property
    def plan(self) -> FinitePositionPlan:
        if self._plan is None:
            raise RuntimeError("no finite-position plan is active")
        return self._plan

    @property
    def interval_index(self) -> int:
        return self._interval_index

    @property
    def planner_finished(self) -> bool:
        return self.planner_finished_time_s is not None

    @property
    def goal_reached(self) -> bool:
        return self.goal_reached_time_s is not None

    @property
    def terminal_hold_duration_s(self) -> float | None:
        if self.planner_finished_time_s is None or self.goal_reached_time_s is None:
            return None
        return self.goal_reached_time_s - self.planner_finished_time_s

    def begin(self, plan: FinitePositionPlan) -> None:
        self._plan = plan
        self._phase = PositionLifecyclePhase.TRACKING
        self._interval_index = 0
        self._band_enter_time_s = None
        self.planner_finished_time_s = None
        self.goal_reached_time_s = None
        self.terminal_entry_position_error_m = None
        self.terminal_entry_velocity_error_m_s = None

    def command(self) -> PositionLifecycleCommand:
        if self._phase is None:
            raise RuntimeError("finite-position lifecycle has not begun")
        if self._phase == PositionLifecyclePhase.TRACKING:
            return PositionLifecycleCommand(
                phase=self._phase,
                reference_state=self.plan.reference_states[self._interval_index].copy(),
                feedforward_nm=float(
                    self.plan.feedforward_inputs_nm[self._interval_index]
                ),
            )
        return PositionLifecycleCommand(
            phase=self._phase,
            reference_state=np.asarray(
                [self.plan.goal_position_m, 0.0, 0.0, 0.0], dtype=float
            ),
            feedforward_nm=0.0,
        )

    def _update_acceptance(
        self, estimated_position_m: float, estimated_velocity_m_s: float, time_s: float
    ) -> None:
        good = (
            abs(estimated_position_m - self.plan.goal_position_m)
            <= self.position_error_band_m
            and abs(estimated_velocity_m_s) <= self.velocity_error_band_m_s
        )
        if not good:
            self._band_enter_time_s = None
            return
        if self._band_enter_time_s is None:
            self._band_enter_time_s = time_s
        if time_s - self._band_enter_time_s >= self.dwell_s - 1e-12:
            self._phase = PositionLifecyclePhase.GOAL_REACHED
            self.goal_reached_time_s = time_s

    def observe(
        self, estimated_position_m: float, estimated_velocity_m_s: float, time_s: float
    ) -> None:
        if self._phase is None:
            raise RuntimeError("finite-position lifecycle has not begun")
        if self._phase == PositionLifecyclePhase.GOAL_REACHED:
            return
        if self._phase == PositionLifecyclePhase.TRACKING:
            self._interval_index += 1
            if self._interval_index < len(self.plan.feedforward_inputs_nm):
                return
            self._phase = PositionLifecyclePhase.TERMINAL_HOLD
            self.planner_finished_time_s = float(time_s)
            self.terminal_entry_position_error_m = (
                float(estimated_position_m) - self.plan.goal_position_m
            )
            self.terminal_entry_velocity_error_m_s = float(estimated_velocity_m_s)
        self._update_acceptance(
            float(estimated_position_m), float(estimated_velocity_m_s), float(time_s)
        )


class VelocityLifecyclePhase(str, Enum):
    VELOCITY_TRANSIENT = "VELOCITY_TRANSIENT"
    VELOCITY_HOLD = "VELOCITY_HOLD"


@dataclass(frozen=True)
class VelocityTransitionPlan:
    target_velocity_m_s: float
    planned_transient_duration_s: float
    feedforward_rearmed: bool
    reference_states: NDArray[np.float64]
    reference_accelerations_m_s2: NDArray[np.float64]
    shaped_reference_finished: NDArray[np.bool_]
    feedforward_inputs_nm: NDArray[np.float64]

    def __post_init__(self) -> None:
        states = np.asarray(self.reference_states, dtype=float)
        accelerations = np.asarray(self.reference_accelerations_m_s2, dtype=float)
        finished = np.asarray(self.shaped_reference_finished, dtype=bool)
        inputs = np.asarray(self.feedforward_inputs_nm, dtype=float)
        if states.ndim != 2 or states.shape[1] != 4 or len(states) < 2:
            raise ValueError("reference_states must have shape (N+1, 4)")
        if accelerations.shape != (len(states),):
            raise ValueError("reference_accelerations_m_s2 must have shape (N+1,)")
        if finished.shape != (len(states),):
            raise ValueError("shaped_reference_finished must have shape (N+1,)")
        if inputs.shape != (len(states) - 1,):
            raise ValueError("feedforward_inputs_nm must have shape (N,)")
        if not np.isfinite(
            np.r_[
                states.ravel(),
                accelerations,
                inputs,
                self.target_velocity_m_s,
                self.planned_transient_duration_s,
            ]
        ).all():
            raise ValueError("velocity transition plan must be finite")
        if self.planned_transient_duration_s < 0.0:
            raise ValueError("planned transient duration must be non-negative")
        if not np.allclose(
            states[-1, 1:], [self.target_velocity_m_s, 0.0, 0.0],
            atol=1e-9, rtol=0.0,
        ):
            raise ValueError("velocity plan must end on its zero-lean cruise manifold")


@dataclass(frozen=True)
class VelocityLifecycleCommand:
    phase: VelocityLifecyclePhase
    reference_state: NDArray[np.float64]
    reference_acceleration_m_s2: float
    feedforward_nm: float
    fade_active: bool = False
    fade_alpha: float = 0.0
    fade_started: bool = False
    fade_start_time_s: float | None = None
    feedforward_exit_nm: float | None = None
    dynamic_reference_before_fade: NDArray[np.float64] | None = None
    dynamic_acceleration_before_fade_m_s2: float | None = None
    fade_finished: bool = False
    fade_completion_time_s: float | None = None
    feedforward_phase: str = "OFF"
    feedforward_rearmed: bool | None = None


class VelocityReferenceLifecycle:
    """Independent feedforward fade and nominal lean-reference completion."""

    def __init__(
        self,
        controller_dt_s: float,
        feedforward_fade_s: float,
    ) -> None:
        self.dt_s = float(controller_dt_s)
        self.feedforward_fade_s = float(feedforward_fade_s)
        positive = (self.dt_s, self.feedforward_fade_s)
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("velocity lifecycle parameters must be finite and positive")
        self._phase = VelocityLifecyclePhase.VELOCITY_HOLD
        self._plan: VelocityTransitionPlan | None = None
        self._interval_index = 0
        self._fade_start_time_s: float | None = None
        self._fade_completed = False
        self._feedforward_exit_nm = 0.0
        self._hold_position_m = 0.0
        self._hold_velocity_m_s = 0.0
        self._hold_acceleration_m_s2 = 0.0
        self._hold_hidden_state = np.zeros(2, dtype=float)

    @property
    def phase(self) -> VelocityLifecyclePhase:
        return self._phase

    @property
    def feedforward_fade_active(self) -> bool:
        return bool(
            self._fade_start_time_s is not None and not self._fade_completed
        )

    @property
    def reference_state(self) -> NDArray[np.float64]:
        if self._phase == VelocityLifecyclePhase.VELOCITY_TRANSIENT:
            if self._plan is None:
                raise RuntimeError("velocity transition plan is missing")
            if self._interval_index >= len(self._plan.reference_states):
                raise RuntimeError("velocity transition exceeded its scheduled segment")
            return self._plan.reference_states[self._interval_index].copy()
        return np.asarray(
            [
                self._hold_position_m,
                self._hold_velocity_m_s,
                self._hold_hidden_state[0],
                self._hold_hidden_state[1],
            ],
            dtype=float,
        )

    @property
    def reference_acceleration_m_s2(self) -> float:
        if self._phase == VelocityLifecyclePhase.VELOCITY_TRANSIENT:
            if self._plan is None:
                raise RuntimeError("velocity transition plan is missing")
            return float(
                self._plan.reference_accelerations_m_s2[self._interval_index]
            )
        return self._hold_acceleration_m_s2

    @property
    def remaining_interval_count(self) -> int:
        if self._phase != VelocityLifecyclePhase.VELOCITY_TRANSIENT:
            return 0
        assert self._plan is not None
        return len(self._plan.feedforward_inputs_nm) - self._interval_index

    def reset(
        self, position_reference_m: float = 0.0, velocity_reference_m_s: float = 0.0
    ) -> None:
        if not math.isfinite(position_reference_m) or not math.isfinite(
            velocity_reference_m_s
        ):
            raise ValueError("initial velocity reference must be finite")
        self._phase = VelocityLifecyclePhase.VELOCITY_HOLD
        self._plan = None
        self._interval_index = 0
        self._fade_start_time_s = None
        self._fade_completed = False
        self._feedforward_exit_nm = 0.0
        self._hold_position_m = float(position_reference_m)
        self._hold_velocity_m_s = float(velocity_reference_m_s)
        self._hold_acceleration_m_s2 = 0.0
        self._hold_hidden_state = np.zeros(2, dtype=float)

    def begin_transition(self, plan: VelocityTransitionPlan) -> None:
        current = self.reference_state
        if not np.allclose(current, plan.reference_states[0], atol=1e-10, rtol=0.0):
            raise ValueError("velocity transition must start from the current reference")
        self._phase = VelocityLifecyclePhase.VELOCITY_TRANSIENT
        self._plan = plan
        self._interval_index = 0
        self._fade_start_time_s = None
        self._fade_completed = False
        self._feedforward_exit_nm = 0.0

    def enter_quiet_hold(self, time_s: float) -> VelocityLifecycleCommand:
        reference = self.reference_state
        feedforward_before = self.raw_feedforward_nm
        self._hold_position_m = float(reference[0])
        self._hold_velocity_m_s = float(reference[1])
        self._hold_acceleration_m_s2 = 0.0
        self._hold_hidden_state = np.zeros(2, dtype=float)
        self._phase = VelocityLifecyclePhase.VELOCITY_HOLD
        self._fade_start_time_s = float(time_s)
        self._fade_completed = False
        self._feedforward_exit_nm = feedforward_before
        return VelocityLifecycleCommand(
            phase=self._phase,
            reference_state=self.reference_state,
            reference_acceleration_m_s2=0.0,
            feedforward_nm=feedforward_before,
            fade_active=True,
            fade_alpha=1.0,
            fade_started=True,
            fade_start_time_s=float(time_s),
            feedforward_exit_nm=feedforward_before,
            dynamic_reference_before_fade=reference,
            dynamic_acceleration_before_fade_m_s2=0.0,
            feedforward_rearmed=True,
            feedforward_phase="FADING",
        )

    def _shaped_reference_reached(self) -> bool:
        if self._plan is None:
            return False
        return bool(self._plan.shaped_reference_finished[self._interval_index])

    @property
    def shaped_reference_reached(self) -> bool:
        return self._shaped_reference_reached()

    @property
    def raw_feedforward_nm(self) -> float:
        if self._plan is None:
            return 0.0
        return float(self._plan.feedforward_inputs_nm[
            min(self._interval_index, len(self._plan.feedforward_inputs_nm) - 1)
        ])

    def command(self, time_s: float) -> VelocityLifecycleCommand:
        time_s = float(time_s)
        reference = self.reference_state
        acceleration = self.reference_acceleration_m_s2
        if self._fade_start_time_s is not None:
            fade_fraction = float(
                np.clip(
                    (time_s - self._fade_start_time_s) / self.feedforward_fade_s,
                    0.0,
                    1.0,
                )
            )
            if fade_fraction >= 1.0:
                just_finished = not self._fade_completed
                self._fade_completed = True
                return VelocityLifecycleCommand(
                    phase=self._phase,
                    reference_state=reference,
                    reference_acceleration_m_s2=acceleration,
                    feedforward_nm=0.0,
                    fade_alpha=0.0,
                    fade_finished=just_finished,
                    fade_completion_time_s=time_s if just_finished else None,
                    feedforward_rearmed=True,
                )
            alpha = 1.0 - 3.0 * fade_fraction**2 + 2.0 * fade_fraction**3
            return VelocityLifecycleCommand(
                phase=self._phase,
                reference_state=reference,
                reference_acceleration_m_s2=acceleration,
                feedforward_nm=alpha * self._feedforward_exit_nm,
                fade_active=True,
                fade_alpha=alpha,
                fade_start_time_s=self._fade_start_time_s,
                feedforward_exit_nm=self._feedforward_exit_nm,
                feedforward_phase="FADING",
                feedforward_rearmed=True,
            )
        if self._plan is None or not self._plan.feedforward_rearmed:
            return VelocityLifecycleCommand(
                phase=self._phase,
                reference_state=reference,
                reference_acceleration_m_s2=acceleration,
                feedforward_nm=0.0,
                feedforward_rearmed=False,
            )
        if self._interval_index == len(self._plan.reference_states) - 1:
            # T_full has finished: fade the last solved interval input.
            # T_v and T_full are independent; smoothstep/rearm are unchanged.
            feedforward_before = float(
                self._plan.feedforward_inputs_nm[
                    min(self._interval_index, len(self._plan.feedforward_inputs_nm) - 1)
                ]
            )
            self._fade_start_time_s = time_s
            self._feedforward_exit_nm = feedforward_before
            return VelocityLifecycleCommand(
                phase=self._phase,
                reference_state=reference,
                reference_acceleration_m_s2=acceleration,
                feedforward_nm=feedforward_before,
                fade_active=True,
                fade_alpha=1.0,
                fade_started=True,
                fade_start_time_s=time_s,
                feedforward_exit_nm=feedforward_before,
                dynamic_reference_before_fade=reference.copy(),
                dynamic_acceleration_before_fade_m_s2=acceleration,
                feedforward_rearmed=True,
                feedforward_phase="FADING",
            )
        if self._interval_index >= len(self._plan.feedforward_inputs_nm):
            raise RuntimeError("velocity transition did not reach hold in its segment")
        return VelocityLifecycleCommand(
            phase=self._phase,
            reference_state=self.reference_state,
            reference_acceleration_m_s2=float(
                self._plan.reference_accelerations_m_s2[self._interval_index]
            ),
            feedforward_nm=(
                float(self._plan.feedforward_inputs_nm[self._interval_index])
                if self._plan.feedforward_rearmed
                else 0.0
            ),
            feedforward_rearmed=self._plan.feedforward_rearmed,
            feedforward_phase="TRANSIENT",
        )

    def advance(self) -> bool:
        """Advance the original nominal plan; report its terminal-manifold entry.

        This clock never depends on feedforward fade progress or plant state.
        """
        if self._phase == VelocityLifecyclePhase.VELOCITY_TRANSIENT:
            assert self._plan is not None
            self._interval_index += 1
            if self._interval_index == len(self._plan.reference_states) - 1:
                terminal = self._plan.reference_states[-1]
                self._hold_position_m = float(terminal[0])
                self._hold_velocity_m_s = float(self._plan.target_velocity_m_s)
                self._hold_acceleration_m_s2 = 0.0
                self._hold_hidden_state = np.zeros(2, dtype=float)
                self._phase = VelocityLifecyclePhase.VELOCITY_HOLD
                return True
        else:
            self._hold_position_m += self._hold_velocity_m_s * self.dt_s
        return False
