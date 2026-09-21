"""Run the frozen empty-robot Stage 3 baseline with MuJoCo command sliders.

The two zero-force command actuators shown in the viewer's Control panel are
UI inputs only; the frozen controller still owns the physical wheel actuators.
Two dependency-free SVG plots are saved afterward.
"""

from __future__ import annotations

import argparse
from datetime import datetime
from html import escape
import math
from pathlib import Path
import sys
import tempfile
import time

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import run_stage3b_yaw_control as stage3b


RESULT_DIR = ROOT / "models" / "minisegway" / "stage3" / "results" / "demo"
MODEL_DIR = ROOT / "models" / "minisegway"
V_COMMAND_ACTUATOR = "COMMAND_v_m_s"
W_COMMAND_ACTUATOR = "COMMAND_w_rad_s"
V_LIMIT_M_S = 0.6
W_LIMIT_RAD_S = 0.5
V_STEP_M_S = 0.02
W_STEP_RAD_S = 0.02


def create_command_slider_model() -> Path:
    """Create a temporary, dynamically identical model with two UI channels."""

    spec = mujoco.MjSpec()
    spec.from_file(str(MODEL_DIR / "mini_segway.xml"))
    for name, target, limit in (
        (V_COMMAND_ACTUATOR, "left_wheel_hinge", V_LIMIT_M_S),
        (W_COMMAND_ACTUATOR, "right_wheel_hinge", W_LIMIT_RAD_S),
    ):
        actuator = spec.add_actuator()
        actuator.name = name
        actuator.target = target
        actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT
        actuator.gear[:] = 0.0
        actuator.ctrllimited = 1
        actuator.ctrlrange[:] = [-limit, limit]
        actuator.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        actuator.gainprm[0] = 1.0
        actuator.biastype = mujoco.mjtBias.mjBIAS_NONE
    spec.compile()
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix=".xml",
        prefix="_stage3_command_demo_",
        dir=MODEL_DIR,
        delete=False,
    ) as output:
        output.write(spec.to_xml())
        return Path(output.name)


class RealtimeViewer:
    """Small passive-viewer callback with real-time pacing and ~60 Hz redraw."""

    def __init__(self, realtime_factor: float) -> None:
        self.realtime_factor = realtime_factor
        self.context = None
        self.viewer = None
        self.start_wall_s = None
        self.last_sync_sim_s = -math.inf
        self.closed = False

    def __call__(self, sim) -> None:
        if self.closed:
            return
        if self.viewer is None:
            self.context = mujoco.viewer.launch_passive(sim.model, sim.data)
            self.viewer = self.context.__enter__()
            self.start_wall_s = time.perf_counter() - sim.data.time / self.realtime_factor
        if not self.viewer.is_running():
            self.close()
            return

        target_wall_s = self.start_wall_s + sim.data.time / self.realtime_factor
        remaining_s = target_wall_s - time.perf_counter()
        if remaining_s > 0.0:
            time.sleep(remaining_s)
        if sim.data.time - self.last_sync_sim_s >= 1.0 / 60.0:
            self.viewer.sync()
            self.last_sync_sim_s = float(sim.data.time)

    def close(self) -> None:
        if self.context is not None:
            self.context.__exit__(None, None, None)
        self.context = None
        self.viewer = None
        self.closed = True

    @staticmethod
    def _quantize(value: float, step: float, limit: float) -> float:
        value = max(-limit, min(limit, value))
        return math.copysign(math.floor(abs(value) / step + 0.5) * step, value)

    def read_commands(self, sim, _control_time_s: float) -> tuple[float, float]:
        if self.closed:
            return 0.0, 0.0
        v_id = sim.model.actuator(V_COMMAND_ACTUATOR).id
        w_id = sim.model.actuator(W_COMMAND_ACTUATOR).id
        lock = self.viewer.lock() if self.viewer is not None else None
        if lock is None:
            raw_v = float(sim.data.ctrl[v_id])
            raw_w = float(sim.data.ctrl[w_id])
            v_cmd = self._quantize(raw_v, V_STEP_M_S, V_LIMIT_M_S)
            w_cmd = self._quantize(raw_w, W_STEP_RAD_S, W_LIMIT_RAD_S)
            sim.data.ctrl[v_id], sim.data.ctrl[w_id] = v_cmd, w_cmd
        else:
            with lock:
                raw_v = float(sim.data.ctrl[v_id])
                raw_w = float(sim.data.ctrl[w_id])
                v_cmd = self._quantize(raw_v, V_STEP_M_S, V_LIMIT_M_S)
                w_cmd = self._quantize(raw_w, W_STEP_RAD_S, W_LIMIT_RAD_S)
                sim.data.ctrl[v_id], sim.data.ctrl[w_id] = v_cmd, w_cmd
        return v_cmd, w_cmd


def _points(t: list[float], y: list[float], box: tuple[float, float, float, float],
            t_bounds: tuple[float, float], y_bounds: tuple[float, float]) -> str:
    left, top, width, height = box
    t0, t1 = t_bounds
    y0, y1 = y_bounds
    return " ".join(
        f"{left + width * (tx - t0) / (t1 - t0):.2f},"
        f"{top + height * (y1 - yx) / (y1 - y0):.2f}"
        for tx, yx in zip(t, y)
    )


def write_two_panel_svg(path: Path, title: str, time_s: list[float], panels: list[dict]) -> None:
    width, height = 1200, 720
    left, right = 90, 25
    plot_width = width - left - right
    panel_height = 245
    panel_tops = (100, 420)
    t0, t1 = min(time_s), max(time_s)
    if t1 <= t0:
        t1 = t0 + 1.0

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{width / 2}" y="38" text-anchor="middle" font-family="sans-serif" font-size="22">{escape(title)}</text>',
    ]
    for panel, top in zip(panels, panel_tops):
        series = panel["series"]
        values = [value for item in series for value in item[1] if math.isfinite(value)]
        y_min = min(values + [0.0])
        y_max = max(values + [0.0])
        pad = max(0.05 * (y_max - y_min), 1e-3)
        y_bounds = (y_min - pad, y_max + pad)
        box = (left, top, plot_width, panel_height)

        svg.append(f'<rect x="{left}" y="{top}" width="{plot_width}" height="{panel_height}" fill="#fafafa" stroke="#444"/>')
        for index in range(6):
            x = left + plot_width * index / 5.0
            y = top + panel_height * index / 5.0
            svg.append(f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + panel_height}" stroke="#dddddd"/>')
            svg.append(f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}" stroke="#dddddd"/>')
            svg.append(f'<text x="{x:.2f}" y="{top + panel_height + 22}" text-anchor="middle" font-family="sans-serif" font-size="12">{t0 + (t1 - t0) * index / 5.0:.1f}</text>')
            y_tick = y_bounds[1] - (y_bounds[1] - y_bounds[0]) * index / 5.0
            svg.append(f'<text x="{left - 10}" y="{y + 4:.2f}" text-anchor="end" font-family="sans-serif" font-size="12">{y_tick:.3f}</text>')

        svg.append(f'<text x="{left}" y="{top - 14}" font-family="sans-serif" font-size="16">{escape(panel["label"])}</text>')
        svg.append(f'<text x="{width / 2}" y="{top + panel_height + 45}" text-anchor="middle" font-family="sans-serif" font-size="13">time [s]</text>')
        legend_x = left + 170
        for index, (name, data, color, dashed) in enumerate(series):
            points = _points(time_s, data, box, (t0, t1), y_bounds)
            dash = ' stroke-dasharray="8,5"' if dashed else ""
            svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"{dash}/>' )
            lx = legend_x + index * 205
            svg.append(f'<line x1="{lx}" y1="{top - 19}" x2="{lx + 24}" y2="{top - 19}" stroke="{color}" stroke-width="3"{dash}/>' )
            svg.append(f'<text x="{lx + 30}" y="{top - 14}" font-family="sans-serif" font-size="13">{escape(name)}</text>')
    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")


def history_column(history: list[dict], key: str) -> list[float]:
    return [float(row[key]) for row in history]


def save_plots(history: list[dict]) -> tuple[Path, Path]:
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = RESULT_DIR / f"stage3_demo_{stamp}"
    t = history_column(history, "t")

    response_path = prefix.with_name(prefix.name + "_command_response.svg")
    write_two_panel_svg(
        response_path,
        "Stage 3 baseline slider command/response",
        t,
        [
            {
                "label": "Longitudinal velocity [m/s]",
                "series": [
                    ("v_cmd", history_column(history, "linear_velocity_cmd_m_s"), "#111111", True),
                    ("v_ref", history_column(history, "v_ref_m_s"), "#2563eb", False),
                    ("v_hat", history_column(history, "v_hat_m_s"), "#dc2626", False),
                    ("v_GT", history_column(history, "v_GT_m_s"), "#16a34a", False),
                ],
            },
            {
                "label": "Yaw rate [rad/s]",
                "series": [
                    ("w_cmd", history_column(history, "yaw_rate_cmd_rad_s"), "#111111", True),
                    ("w_hat", history_column(history, "r_hat_rad_s"), "#dc2626", False),
                    ("w_GT", history_column(history, "r_GT_rad_s"), "#16a34a", False),
                ],
            },
        ],
    )

    control_path = prefix.with_name(prefix.name + "_control_torque.svg")
    write_two_panel_svg(
        control_path,
        "Stage 3 baseline control commands and applied wheel torque",
        t,
        [
            {
                "label": "Longitudinal/common-mode torque [N m]",
                "series": [
                    ("u_fb", history_column(history, "u_fb_nm"), "#dc2626", False),
                    ("u_ff", history_column(history, "u_ff_used_nm"), "#7c3aed", False),
                    ("u_sum_cmd", history_column(history, "u_sum_nm"), "#2563eb", False),
                    ("u_sum_actual", history_column(history, "actual_sum_nm"), "#16a34a", True),
                ],
            },
            {
                "label": "Yaw/differential and wheel torque [N m]",
                "series": [
                    ("u_diff_req", history_column(history, "u_diff_request_nm"), "#7c3aed", True),
                    ("u_diff_used", history_column(history, "u_diff_used_nm"), "#2563eb", False),
                    ("actual_left", history_column(history, "actual_left_nm"), "#dc2626", False),
                    ("actual_right", history_column(history, "actual_right_nm"), "#16a34a", False),
                ],
            },
        ],
    )
    return response_path, control_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0, help="total simulation time [s]")
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    args = parser.parse_args()

    for name in ("duration", "realtime_factor"):
        if not math.isfinite(getattr(args, name)):
            parser.error(f"--{name.replace('_', '-')} must be finite")
    if args.realtime_factor <= 0.0:
        parser.error("--realtime-factor must be positive")
    if args.duration < 2.0:
        parser.error("--duration must be at least 2.0 s")

    scenario = {
        "name": "stage3_empty_robot_slider_command_demo",
        "duration_s": args.duration,
        "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
    }
    common = stage3b.load_common()
    manifest = common[0]
    viewer = RealtimeViewer(args.realtime_factor)
    command_model_path = create_command_slider_model()
    print(f"Empty Stage 3 slider demo: duration={args.duration:.1f} s.")
    print("Use the MuJoCo right-panel Control sliders named COMMAND_v_m_s and COMMAND_w_rad_s.")
    print("v: +/-0.60 m/s in 0.02 steps; w: +/-0.50 rad/s in 0.02 steps.")
    print("Closing the viewer hides it; the finite run continues so plots can still be saved.")
    try:
        result = stage3b.run_case(
            scenario,
            float(manifest["yaw"]["K_psi_nm_per_rad"]),
            float(manifest["yaw"]["K_r_nm_per_rad_s"]),
            yaw_enabled=True,
            motor_mismatch_enabled=False,
            common=common,
            keep_history=True,
            payload_mode="empty",
            common_mode_augmentation=None,
            physics_step_callback=viewer,
            command_source=viewer.read_commands,
            empty_model_path=command_model_path,
        )
    finally:
        viewer.close()
        command_model_path.unlink(missing_ok=True)

    response_path, control_path = save_plots(result["history_50hz"])
    print(f"SAVED {response_path}")
    print(f"SAVED {control_path}")


if __name__ == "__main__":
    main()
