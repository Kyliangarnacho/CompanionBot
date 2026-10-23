"""Headless physical acceptance, drift measurement and frozen-control ablation.

GT is captured only by this evaluator, written separately, and never returned
to the sensor/estimate/controller path. No GUI or physical device is accessed.
"""
import argparse
from dataclasses import fields
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from companionbot_contracts.spatial_geometry import load_spatial_geometry
from companionbot_control.estimation.spatial import PlanarOdometry, SpatialSample
from companionbot_simulation.runtime.spatial_session import SpatialSession


class PosthocTruth:
    def __init__(self):
        self.rows = {}
        self.geometry = load_spatial_geometry()

    def __call__(self, sim):
        c = self.geometry.mj_to_ros
        body = sim.model.body("chassis").id
        rotation = c @ sim.data.xmat[body].reshape(3, 3) @ c.T
        self.rows[sim.step_index] = {"time_s": float(sim.data.time),
            "position_ros": (c @ sim.data.xpos[body]).tolist(),
            "rpy_ros": Rotation.from_matrix(rotation).as_euler("xyz").tolist()}


CASES = (("balance", 4.), ("forward", 14.), ("turn", 8.), ("mixed", 14.), ("pitch", 4.))


def evaluate(session, truth, *, write_artifacts=True):
    odom = PlanarOdometry()
    keys = {f.name for f in fields(SpatialSample)}
    rows = []
    first_truth = None
    for raw in session.samples:
        state = SpatialSample(**{k: v for k, v in raw.items() if k in keys})
        estimate = odom.update(state)
        gt = truth.rows[round(state.time_s/.001)]
        if first_truth is None:
            first_truth = gt
        position = np.asarray(gt["position_ros"]) - first_truth["position_ros"]
        position = Rotation.from_euler("z", -first_truth["rpy_ros"][2]).apply(position)
        yaw = gt["rpy_ros"][2] - first_truth["rpy_ros"][2]
        errors = [estimate.x_m-position[0], estimate.y_m-position[1],
                  math.atan2(math.sin(estimate.yaw_rad-yaw), math.cos(estimate.yaw_rad-yaw)),
                  state.roll_rad-gt["rpy_ros"][0], state.pitch_rad-gt["rpy_ros"][1]]
        rows.append({"time_s": state.time_s, "estimated_xy_yaw": [estimate.x_m, estimate.y_m, estimate.yaw_rad],
            "truth_xy_yaw_posthoc": [float(position[0]), float(position[1]), yaw],
            "errors_xy_yaw_roll_pitch": errors, "covariance_xy_yaw": estimate.covariance.tolist()})
    errors = np.asarray([r["errors_xy_yaw_roll_pitch"] for r in rows])
    xy = np.linalg.norm(errors[:, :2], axis=1)
    metrics = {"sample_count": len(rows), "position_error_peak_m": float(np.max(xy)),
        "position_error_final_m": float(xy[-1]), "yaw_error_peak_rad": float(np.max(np.abs(errors[:, 2]))),
        "yaw_error_final_rad": float(errors[-1, 2]),
        "roll_error_rms_rad": float(np.sqrt(np.mean(errors[:, 3]**2))),
        "pitch_error_rms_rad": float(np.sqrt(np.mean(errors[:, 4]**2))),
        "imu_age_peak_s": max(s["imu_age_s"] for s in session.samples),
        "estimate_is_ground_truth": False, "covariance_calibrated": False}
    # Fixed before running the matrix. This is a short flat-ground engineering
    # envelope, not a localization guarantee or a relaxation of existing gates.
    metrics["short_run_envelope"] = {"position_peak_limit_m": .1, "yaw_peak_limit_rad": .1,
        "passed": metrics["position_error_peak_m"] <= .1 and metrics["yaw_error_peak_rad"] <= .1}
    if write_artifacts:
        (session.output_dir/"spatial_evaluation_posthoc.json").write_text(json.dumps({"metrics": metrics, "history": rows}, indent=2)+"\n")
        (session.output_dir/"truth_posthoc.jsonl").write_text("".join(json.dumps(v)+"\n" for v in truth.rows.values()))
    return metrics, rows


def control_gates(result, duration_s):
    """Preserve the frozen controller's existing safety and provenance gates."""
    termination = result["simulation_termination"]
    torque = result["torque_allocation"]
    checks = {
        "finite": result["finite"] is True,
        "no_fall": result["longitudinal"]["fell"] is False,
        "no_runtime_gt_dependency": result["GT_runtime_dependency"] is False,
        "q_actuator_off": result["frozen_longitudinal"]["Q_disturbance_rejection_enabled"] is False,
        "full_duration": termination["terminated_early"] is False
            and termination["reason"] is None
            and math.isclose(termination["actual_duration_s"], duration_s, abs_tol=1e-9)
            and math.isclose(termination["requested_duration_s"], duration_s, abs_tol=1e-9),
        "zero_wheel_saturation": torque["per_wheel_saturation_fraction"] == 0.,
        "zero_differential_clipping": torque["differential_clipping_fraction"] == 0.,
        "zero_final_guard_clipping": torque["final_guard_clipping_fraction"] == 0.,
    }
    return {"checks": checks, "passed": all(checks.values())}


def source_gates(samples, rows, configuration, duration_s):
    """Check saved sensor evidence, rather than inferring timing from FPS."""
    # SpatialSample.validate() and PlanarOdometry.update() have already checked
    # every sensor flag, finite field, packet age, epoch and sequence in evaluate.
    covariance = np.asarray([row["covariance_xy_yaw"] for row in rows])
    errors = np.asarray([row["errors_xy_yaw_roll_pitch"] for row in rows])
    checks = {
        "complete_50hz_source_history": len(samples) == round(duration_s/.02),
        "source_horizon_cadence": all(math.isclose(s["time_s"], (i+1)*.02, abs_tol=1e-9)
                                      for i, s in enumerate(samples)),
        "500hz_control_sequence": all(s["sequence"] == (i+1)*10 for i, s in enumerate(samples)),
        "one_declared_epoch": all(s["epoch"] == configuration["epoch"] for s in samples),
        "source_imu_delay_1ms": all(math.isclose(s["time_s"]-s["imu_sample_time_s"], .001, abs_tol=1e-9)
                                    and math.isclose(s["imu_age_s"], .001, abs_tol=1e-9)
                                    and math.isclose(s["imu_available_time_s"], s["time_s"], abs_tol=1e-9)
                                    for s in samples),
        "raw_imu_finite": all(np.isfinite(s["accel_ros"]).all() and np.isfinite(s["gyro_ros"]).all()
                              for s in samples),
        "evaluation_finite": bool(np.isfinite(errors).all() and np.isfinite(covariance).all()),
        "covariance_symmetric_psd": bool(np.allclose(covariance, covariance.transpose(0, 2, 1), atol=1e-12)
                                          and np.linalg.eigvalsh(covariance).min() >= -1e-12),
        "frozen_configuration": bool(configuration["frozen_checks"])
            and all(value is True for value in configuration["frozen_checks"].values())
            and configuration["physics_dt_s"] == .001 and configuration["control_dt_s"] == .002
            and configuration["state_period_s"] == .02 and configuration["Q_actuator"] is False,
    }
    return {"checks": checks, "passed": all(checks.values())}


def compare_history(a, b):
    if a == b:
        return {"exact": True, "rows": len(a), "maximum_numeric_delta": 0.}
    delta = 0.
    differences = []
    for i, (left, right) in enumerate(zip(a, b)):
        for key in left.keys() | right.keys():
            if left.get(key) == right.get(key):
                continue
            try:
                change = float(np.max(np.abs(np.asarray(left[key], dtype=float)-np.asarray(right[key], dtype=float))))
                delta = max(delta, change)
            except (TypeError, ValueError, KeyError):
                change = None
            differences.append({"row": i, "field": key, "numeric_delta": change})
    return {"exact": False, "rows": [len(a), len(b)], "maximum_numeric_delta": delta, "differences": differences}


def plot(output, histories):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for name, rows in histories.items():
        estimated = np.asarray([r["estimated_xy_yaw"] for r in rows])
        truth = np.asarray([r["truth_xy_yaw_posthoc"] for r in rows])
        line, = axes[0].plot(estimated[:, 0], estimated[:, 1], label=name+" estimate")
        axes[0].plot(truth[:, 0], truth[:, 1], "--", color=line.get_color(), alpha=.7)
        error = np.asarray([r["errors_xy_yaw_roll_pitch"] for r in rows])
        axes[1].plot([r["time_s"] for r in rows], np.linalg.norm(error[:, :2], axis=1)*1000, label=name)
    axes[0].set(xlabel="odom x [m]", ylabel="odom y [m]", title="Solid: sensor estimate; dashed: post-hoc truth")
    axes[0].axis("equal")
    axes[1].set(xlabel="source time [s]", ylabel="position error [mm]", title="Uncorrected wheel/inertial drift")
    for axis in axes:
        axis.grid(True)
        axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output/"spatial_drift.png", dpi=160)
    plt.close(fig)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--output-dir", type=Path, required=True)
    cli.add_argument("--reevaluate-existing", action="store_true",
                     help="audit saved source/truth/control evidence without rerunning physics or replacing raw artifacts")
    args = cli.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    metrics, histories, results = {}, {}, {}
    summary_path = output/("summary_revalidated.json" if args.reevaluate_existing else "summary.json")
    # A malformed first input must not leave a prior run's successful verdict.
    summary_path.write_text(json.dumps({"cases": {}, "passed": False}, indent=2)+"\n")
    for name, duration in CASES:
        print("REEVALUATE" if args.reevaluate_existing else "RUN", name, flush=True)
        if args.reevaluate_existing:
            session = SimpleNamespace(output_dir=output/name,
                samples=[json.loads(line) for line in (output/name/"sensorized_state.jsonl").read_text().splitlines()])
            truth_rows = [json.loads(line) for line in (output/name/"truth_posthoc.jsonl").read_text().splitlines()]
            truth = SimpleNamespace(rows={round(row["time_s"]/.001): row for row in truth_rows})
            result = json.loads((output/name/"frozen_result_posthoc.json").read_text())
        else:
            truth = PosthocTruth()
            session = SpatialSession(output/name, scenario=name, duration_s=duration, posthoc=truth)
            result = session.run()
        values, rows = evaluate(session, truth, write_artifacts=not args.reevaluate_existing)
        values["termination"] = result["simulation_termination"]
        values["actual_torque"] = result["torque_allocation"]
        values["control_gates"] = control_gates(result, duration)
        configuration = json.loads((session.output_dir/"configuration.json").read_text())
        values["source_gates"] = source_gates(session.samples, rows, configuration, duration)
        metrics[name], histories[name], results[name] = values, rows, result
        summary_path.write_text(json.dumps({"cases": metrics, "passed": False}, indent=2)+"\n")
    if args.reevaluate_existing:
        original = json.loads((output/"original_plant_control_posthoc.json").read_text())
    else:
        from companionbot_simulation.scenarios import run_stage3b_yaw_control as baseline
        from companionbot_simulation.scenarios import run_stage5_v1_5_pi_mcu_stream as stage5
        common, _, _, checks = stage5.load_common(stage5.load_json(stage5.CONFIG_PATH))
        assert all(checks.values())
        yaw = common[0]["yaw"]
        original = baseline.run_case(dict(baseline.MIXED_SCENARIO, name="spatial_mixed"),
            yaw["K_psi_nm_per_rad"], yaw["K_r_nm_per_rad_s"], yaw_enabled=True,
            motor_mismatch_enabled=False, common=common, keep_history=True)
        (output/"original_plant_control_posthoc.json").write_text(json.dumps(original, indent=2)+"\n")
    comparison = compare_history(original["history_50hz"], results["mixed"]["history_50hz"])
    if not args.reevaluate_existing:
        (output/"passive_sensor_ablation.json").write_text(json.dumps(comparison, indent=2)+"\n")
        plot(output, histories)
    baseline_gates = control_gates(original, 14.)
    summary = {"cases": metrics, "passive_sensor_ablation": comparison,
               "original_control_gates": baseline_gates,
               "evaluation_mode": "saved_evidence" if args.reevaluate_existing else "physical_run",
               "passed": all(v["short_run_envelope"]["passed"] and v["control_gates"]["passed"]
                             and v["source_gates"]["passed"] for v in metrics.values())
                         and comparison["exact"] and baseline_gates["passed"]}
    summary_path.write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps(summary, indent=2), flush=True)
    if not summary["passed"]:
        raise RuntimeError("spatial acceptance failed; preserve evidence and investigate without retuning frozen controls")


if __name__ == "__main__":
    main()
