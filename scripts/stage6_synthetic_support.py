"""Shared synthetic-target and artifact helpers for Stage 6 validation runs."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any

import numpy as np

from control.rolling_reference import TargetObservation


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    return value


def schedule_value(schedule: list[dict], time_s: float) -> float:
    value = float(schedule[0]["value_m_s"])
    for entry in schedule:
        if float(entry["time_s"]) > time_s + 1e-12:
            break
        value = float(entry["value_m_s"])
    return value


class SyntheticTargetSensor:
    """Sim-only world truth adapter that publishes body-relative packets."""

    def __init__(self, scenario: dict, observation_frequency_hz: float) -> None:
        self.scenario = scenario
        self.period_s = 1.0 / float(observation_frequency_hz)
        self.sequence_id = 0
        self.latest: TargetObservation | None = None
        self.latest_evaluation: dict | None = None
        self.evaluation_history: list[dict] = []
        self.master_position_xy_m: np.ndarray | None = None
        self.master_heading_rad = 0.0
        self.last_physics_time_s = 0.0
        self.next_capture_time_s = 0.0

    @staticmethod
    def _robot_pose(sim) -> tuple[np.ndarray, np.ndarray]:
        root = sim.model.joint("root").id
        qpos_adr = int(sim.model.jnt_qposadr[root])
        chassis = sim.model.body("chassis").id
        position = np.asarray(sim.data.qpos[qpos_adr : qpos_adr + 3], dtype=float)
        local_to_world = np.asarray(sim.data.xmat[chassis], dtype=float).reshape(3, 3)
        return position, local_to_world

    def setup(self, sim) -> dict:
        robot_position, local_to_world = self._robot_pose(sim)
        forward_world = -local_to_world[:, 1]
        initial_distance = float(self.scenario["initial_distance_m"])
        self.master_position_xy_m = (
            robot_position[:2] + initial_distance * forward_world[:2]
        )
        self.master_heading_rad = math.atan2(
            float(forward_world[1]), float(forward_world[0])
        )
        self.last_physics_time_s = float(sim.data.time)
        self.next_capture_time_s = self.last_physics_time_s
        self._capture(sim)
        self.next_capture_time_s += self.period_s
        return {
            "provider": "Stage6SyntheticTargetSensor",
            "observation_frequency_hz": 1.0 / self.period_s,
            "packet_version": 1,
            "controller_reads_sim_truth": False,
            "truth_boundary": "SyntheticTargetSensor internal; evaluation fields are post-hoc",
        }

    def _capture(self, sim) -> None:
        assert self.master_position_xy_m is not None
        robot_position, local_to_world = self._robot_pose(sim)
        relative_world = np.r_[self.master_position_xy_m - robot_position[:2], 0.0]
        relative_body = local_to_world.T @ relative_world
        observation = TargetObservation(
            capture_time_s=float(sim.data.time),
            x_forward_m=-float(relative_body[1]),
            y_left_m=-float(relative_body[0]),
            version=1,
            sequence_id=self.sequence_id,
            valid=True,
            confidence=1.0,
        )
        self.sequence_id += 1
        self.latest = observation
        # GT remains on this evaluator-only side channel.
        self.latest_evaluation = {
            "capture_time_s": observation.capture_time_s,
            "sequence_id": observation.sequence_id,
            "x_forward_GT_m": observation.x_forward_m,
            "master_velocity_GT_m_s": schedule_value(
                self.scenario["master_speed_schedule"], observation.capture_time_s
            ),
        }
        self.evaluation_history.append(self.latest_evaluation.copy())

    def on_physics_step(self, sim) -> None:
        assert self.master_position_xy_m is not None
        time_s = float(sim.data.time)
        dt_s = time_s - self.last_physics_time_s
        midpoint = self.last_physics_time_s + 0.5 * dt_s
        speed = schedule_value(self.scenario["master_speed_schedule"], midpoint)
        direction = np.asarray([
            math.cos(self.master_heading_rad), math.sin(self.master_heading_rad)
        ])
        self.master_position_xy_m += dt_s * speed * direction
        self.last_physics_time_s = time_s
        while time_s + 1e-12 >= self.next_capture_time_s:
            self._capture(sim)
            self.next_capture_time_s += self.period_s

    def read(self) -> TargetObservation:
        if self.latest is None:
            raise RuntimeError("synthetic target sensor is not initialized")
        return self.latest

    def diagnostic(self, sim) -> dict:
        del sim
        if self.latest is None or self.latest_evaluation is None:
            return {}
        return {
            "target_observation_version": self.latest.version,
            "target_observation_sequence_id": self.latest.sequence_id,
            "target_observation_capture_time_s": self.latest.capture_time_s,
            "target_observation_valid": self.latest.valid,
            "target_observation_confidence": self.latest.confidence,
            "target_observation_x_forward_m": self.latest.x_forward_m,
            "target_observation_y_left_m": self.latest.y_left_m,
            **self.latest_evaluation,
        }


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                fields.append(key)
                seen.add(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(json_safe(rows))
