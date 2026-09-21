"""Generate, train, evaluate, and close the Stage 4B proprioception pilot.

Simulation modes require the project virtual environment (MuJoCo).  The train
mode uses the machine Python that already provides PyTorch and scikit-learn.
Raw episodes/checkpoints stay below the git-ignored ``generated`` directory.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from learning.environment_dataset import (  # noqa: E402
    LazyEpisodeWindowDataset,
    fit_train_normalization,
    materialize,
    normalize_window,
    statistical_features,
)


MODEL_DIR = ROOT / "models" / "minisegway"
STAGE_DIR = MODEL_DIR / "stage4" / "stage4b"
CONFIG_PATH = STAGE_DIR / "config" / "stage4b_pilot_config.json"
GENERATED_DIR = STAGE_DIR / "generated"
EPISODE_DIR = GENERATED_DIR / "episodes"
WORLD_DIR = GENERATED_DIR / "worlds"
CHECKPOINT_DIR = GENERATED_DIR / "checkpoints"
RESULT_DIR = STAGE_DIR / "results"
MANIFEST_PATH = RESULT_DIR / "dataset_manifest.json"
METRICS_PATH = RESULT_DIR / "model_metrics.json"
SELECTED_MODEL_PATH = RESULT_DIR / "selected_simple_model.json"
ABLATION_PATH = RESULT_DIR / "closed_loop_ablation.json"
SUMMARY_CSV_PATH = RESULT_DIR / "closed_loop_ablation.csv"
REPORT_PATH = RESULT_DIR / "STAGE4B_REPORT.md"
REVISION_MANIFEST_PATH = RESULT_DIR / "dataset_manifest_r.json"
REVISION_METRICS_PATH = RESULT_DIR / "model_metrics_r.json"
REVISION_ABLATION_PATH = RESULT_DIR / "closed_loop_ablation_r.json"
REVISION_REPORT_PATH = RESULT_DIR / "STAGE4B-R_REPORT.md"

FEATURE_NAMES = [
    "imu_ax_m_s2", "imu_ay_m_s2", "imu_az_m_s2",
    "imu_gx_rad_s", "imu_gy_rad_s", "imu_gz_rad_s",
    "wheel_left_surface_m_s", "wheel_right_surface_m_s",
    "requested_left_nm", "requested_right_nm",
    "actual_left_nm", "actual_right_nm",
    "v_hat_m_s", "pitch_hat_rad", "pitch_rate_hat_rad_s",
    "v_ref_m_s", "a_ref_m_s2", "q_filter_estimate_nm",
    "matched_residual_fraction", "scaled_innovation_rms",
]


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: dict | list) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


def slug(value: float) -> str:
    return ("p" if value >= 0 else "m") + f"{abs(value):04.1f}".replace(".", "p")


def build_episode_specs(config: dict) -> list[dict]:
    """Build a small factorized pilot plus explicit OOD/counterfactual groups."""

    rng = np.random.default_rng(int(config["seed"]))
    levels = config["factor_levels"]
    specs: list[dict] = []

    def add(split: str, kind: str, alpha: float, mu: float, load: str,
            surface: str, velocity: float, group: str | None = None,
            terrain_parameters: dict | None = None) -> None:
        index = len(specs)
        specs.append({
            "episode_id": f"ep_{index:03d}_{split}_{slug(alpha)}",
            "split": split,
            "evaluation_kind": kind,
            "evaluation_group": group,
            "alpha_deg": float(alpha),
            "friction": float(mu),
            "payload": load,
            "terrain": surface,
            "speed_m_s": float(velocity),
            "imu_seed": int(config["seed"]) + 1000 + index * 37,
            "terrain_parameters": terrain_parameters,
        })

    train_rows = []
    for alpha in config["train_slopes_deg"]:
        permutations = {
            name: rng.permutation(values).tolist()
            for name, values in levels.items()
        }
        for repeat in range(3):
            train_rows.append((
                alpha,
                permutations["friction"][repeat],
                permutations["payload"][repeat],
                permutations["terrain"][repeat],
                permutations["speed_m_s"][repeat],
            ))
    rng.shuffle(train_rows)
    for row in train_rows:
        add("train", "in_distribution", *row)
    for i, alpha in enumerate(config["train_slopes_deg"]):
        add("val", "in_distribution", alpha, levels["friction"][(i + 1) % 3],
            levels["payload"][(i + 2) % 3], levels["terrain"][i % 3],
            levels["speed_m_s"][(i + 1) % 3])
    for i, alpha in enumerate(config["numeric_ood_slopes_deg"]):
        add("test", "numeric_ood", alpha, levels["friction"][i % 3],
            levels["payload"][(2 * i + 1) % 3], levels["terrain"][(i + 2) % 3],
            levels["speed_m_s"][(i + 1) % 3], "numeric_ood_mixed")

    combo = [
        (10, 0.12, "free", "smooth", 0.55),
        (-10, 0.12, "fixed", "rough", 0.25),
        (6, 1.00, "free", "bump", 0.40),
        (-6, 0.45, "empty", "rough", 0.55),
    ]
    for values in combo:
        add("test", "combination_ood", *values, group="combination_ood")

    for mu in (1.0, 0.45, 0.12):
        add("test", "counterfactual", 6, mu, "empty", "smooth", 0.40,
            "cf_friction")
    for load in ("empty", "fixed", "free"):
        add("test", "counterfactual", 6, 0.45, load, "smooth", 0.40,
            "cf_payload")
    add("test", "counterfactual", 6, 0.08, "empty", "smooth", 0.20,
        "cf_low_mu_slip")
    add("test", "counterfactual", 6, 0.08, "empty", "bump", 0.40,
        "cf_low_mu_slip", {"height_m": 0.022, "spacing_m": 1.0, "length_m": 0.13})
    unseen = [
        ("rough", {"height_m": 0.0035, "spacing_m": 0.23, "length_m": 0.04}),
        ("rough", {"height_m": 0.0090, "spacing_m": 0.47, "length_m": 0.07}),
        ("bump", {"height_m": 0.008, "spacing_m": 1.0, "length_m": 0.06}),
        ("bump", {"height_m": 0.022, "spacing_m": 1.0, "length_m": 0.13}),
    ]
    for surface, parameters in unseen:
        add("test", "counterfactual", 6, 0.45, "fixed", surface, 0.40,
            "cf_unseen_terrain", parameters)
    for alpha in config["numeric_ood_slopes_deg"]:
        add("test", "counterfactual", alpha, 0.45, "fixed", "smooth", 0.40,
            "cf_alpha_monotonic")
    return specs


def build_revision_specs(config: dict) -> list[dict]:
    """Extend Stage4B to twelve independent episodes per known train slope."""

    baseline = load_json(MANIFEST_PATH)
    keep_fields = (
        "episode_id", "split", "evaluation_kind", "evaluation_group",
        "alpha_deg", "friction", "payload", "terrain", "speed_m_s",
        "imu_seed", "terrain_parameters",
        "command_schedule",
    )
    specs = [
        {**{name: episode.get(name) for name in keep_fields},
         "episode_id": f"rbase_{episode['episode_id']}",
         "source_episode_id": episode["episode_id"],
         "payload_mass_scale": float(episode.get("payload_mass_scale", 1.0)),
         "coupling_class": episode.get("coupling_class"),
         "friction_model_version": "wheel_and_terrain_v2",
         "duration_s": float(config["revision"]["episode_duration_s"])}
        for episode in baseline["episodes"]
    ]
    revision = config["revision"]
    rng = np.random.default_rng(int(config["seed"]) + 41002)
    additional_per_slope = (
        int(revision["episodes_per_train_slope"]) - 3
    )
    if additional_per_slope <= 0:
        raise ValueError("revision must expand beyond the three Stage4B episodes per slope")

    def terrain_parameters(kind: str) -> dict | None:
        if kind == "rough":
            return {
                "height_m": float(rng.uniform(*revision["rough_height_range_m"])),
                "spacing_m": float(rng.uniform(*revision["rough_spacing_range_m"])),
                "length_m": float(rng.uniform(*revision["rough_length_range_m"])),
            }
        if kind == "bump":
            return {
                "height_m": float(rng.uniform(*revision["bump_height_range_m"])),
                "spacing_m": 1.0,
                "length_m": float(rng.uniform(*revision["bump_length_range_m"])),
            }
        return None

    new_index = 0
    for alpha in config["train_slopes_deg"]:
        frictions = []
        for low, high in revision["friction_ranges"]:
            frictions.extend(rng.uniform(low, high, 3).tolist())
        rng.shuffle(frictions)
        payloads = list(rng.permutation(["empty", "fixed", "free"] * 3))
        terrains = list(rng.permutation(["smooth", "rough", "bump"] * 3))
        speeds = rng.uniform(*revision["speed_range_m_s"], additional_per_slope)
        for repeat in range(additional_per_slope):
            payload = str(payloads[repeat])
            terrain = str(terrains[repeat])
            scale = (
                float(rng.uniform(*revision["free_payload_mass_scale_range"]))
                if payload == "free" else 1.0
            )
            specs.append({
                "episode_id": f"rtrain_{new_index:03d}_{slug(alpha)}",
                "split": "train",
                "evaluation_kind": "expanded_in_distribution",
                "evaluation_group": None,
                "alpha_deg": float(alpha),
                "friction": float(frictions[repeat]),
                "payload": payload,
                "terrain": terrain,
                "speed_m_s": float(speeds[repeat]),
                "imu_seed": int(config["seed"]) + 20000 + new_index * 53,
                "terrain_parameters": terrain_parameters(terrain),
                "payload_mass_scale": scale,
                "coupling_class": None,
                "command_schedule": None,
                "source_episode_id": None,
                "friction_model_version": "wheel_and_terrain_v2",
                "duration_s": float(revision["episode_duration_s"]),
            })
            new_index += 1

    coupling = [
        ("clean", "empty", 1.00, "smooth"),
        ("payload_only", "fixed", 1.00, "smooth"),
        ("slip_only", "empty", 0.18, "smooth"),
        ("rough_only", "empty", 1.00, "rough"),
        ("payload_slip", "fixed", 0.18, "smooth"),
        ("payload_rough", "fixed", 1.00, "rough"),
        ("slip_rough", "empty", 0.18, "rough"),
        ("payload_slip_rough", "fixed", 0.18, "rough"),
    ]
    for index, (name, payload, friction, terrain) in enumerate(coupling):
        specs.append({
            "episode_id": f"rcf6_{index:02d}_{name}",
            "split": "test",
            "evaluation_kind": "coupled_disturbance_ood",
            "evaluation_group": "cf_coupling",
            "alpha_deg": 8.0,
            "friction": friction,
            "payload": payload,
            "terrain": terrain,
            "speed_m_s": 0.40,
            "imu_seed": int(config["seed"]) + 30000 + index * 71,
            "terrain_parameters": (
                {"height_m": 0.0015, "spacing_m": 0.55, "length_m": 0.10}
                if terrain == "rough" else None
            ),
            "payload_mass_scale": 1.0,
            "coupling_class": name,
            "command_schedule": (
                [
                    {"time_s": 0.0, "command": 0.0},
                    {"time_s": 0.6, "command": 0.60},
                    {"time_s": 4.2, "command": -0.60},
                ] if "slip" in name else None
            ),
            "source_episode_id": None,
            "friction_model_version": "wheel_and_terrain_v2",
            "duration_s": float(revision["episode_duration_s"]),
        })
    return specs


def build_world(spec: dict, config: dict) -> Path:
    """Parameterize the existing MiniSegway MJCF with a finite grade/terrain."""

    import mujoco
    import run_stage4a_slope_robustness as stage4a

    source = MODEL_DIR / (
        "mini_segway_moving_payload.xml"
        if spec["payload"] == "free" else "mini_segway.xml"
    )
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("model", f"Stage4B {spec['episode_id']}")
    compiler = root.find("compiler")
    if compiler is None:
        raise RuntimeError("base MJCF compiler missing")
    compiler.set("meshdir", "../../../../assets/upstream_local")
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("base MJCF worldbody missing")
    floor = worldbody.find("geom[@name='floor']")
    if floor is None:
        raise RuntimeError("base MJCF floor missing")
    worldbody.remove(floor)

    terrain = config["terrain"]
    lead = float(terrain["flat_lead_in_length_m"])
    back = float(terrain["flat_back_margin_m"])
    transition = float(terrain["transition_length_m"])
    grade = float(terrain["constant_grade_length_m"])
    width = float(terrain["half_width_m"])
    thick = float(terrain["half_thickness_m"])
    alpha = math.radians(float(spec["alpha_deg"]))
    friction = f"{spec['friction']:.8g} 0.005 0.0001"
    if spec.get("friction_model_version") == "wheel_and_terrain_v2":
        for wheel_name in ("left_wheel_collision", "right_wheel_collision"):
            wheel = root.find(f".//geom[@name='{wheel_name}']")
            if wheel is None:
                raise RuntimeError(f"base MJCF {wheel_name} missing")
            wheel.set("friction", friction)
    elements: list[ET.Element] = [ET.Element("geom", {
        "name": "terrain_flat", "type": "box",
        "pos": stage4a.fmt((0.0, 0.5 * (back - lead), -thick)),
        "size": stage4a.fmt((width, 0.5 * (back + lead), thick)),
        "material": "ground_mat", "friction": friction, "condim": "3",
    })]
    transition_center, transition_end = stage4a.grade_box_pose(
        -lead, 0.0, transition, alpha, thick
    )
    elements.append(ET.Element("geom", {
        "name": "terrain_transition", "type": "box",
        "pos": stage4a.fmt(transition_center),
        "euler": stage4a.fmt((-alpha, 0.0, 0.0)),
        "size": stage4a.fmt((width, 0.5 * transition, thick)),
        "material": "ground_mat", "friction": friction, "condim": "3",
    }))
    grade_center, _ = stage4a.grade_box_pose(
        transition_end[1], transition_end[2], grade, alpha, thick
    )
    elements.append(ET.Element("geom", {
        "name": "terrain_constant_grade", "type": "box",
        "pos": stage4a.fmt(grade_center),
        "euler": stage4a.fmt((-alpha, 0.0, 0.0)),
        "size": stage4a.fmt((width, 0.5 * grade, thick)),
        "material": "ground_mat", "friction": friction, "condim": "3",
    }))

    parameters = spec["terrain_parameters"]
    if parameters is None:
        parameters = config[
            "rough_train" if spec["terrain"] == "rough" else "bump_train"
        ] if spec["terrain"] != "smooth" else {}
    if spec["terrain"] in {"rough", "bump"}:
        height = float(parameters["height_m"])
        length = float(parameters["length_m"])
        if spec["terrain"] == "rough":
            positions = np.arange(0.45, grade - 0.25, float(parameters["spacing_m"]))
        else:
            positions = np.asarray([1.55])
        tangent = np.asarray([0.0, -math.cos(alpha), math.sin(alpha)])
        normal = np.asarray([0.0, math.sin(alpha), math.cos(alpha)])
        grade_start = np.asarray([0.0, transition_end[1], transition_end[2]])
        for index, along in enumerate(positions):
            top = grade_start + float(along) * tangent
            center = top + 0.5 * height * normal
            elements.append(ET.Element("geom", {
                "name": f"terrain_{spec['terrain']}_{index:02d}", "type": "box",
                "pos": stage4a.fmt(tuple(center.tolist())),
                "euler": stage4a.fmt((-alpha, 0.0, 0.0)),
                "size": stage4a.fmt((width, 0.5 * length, 0.5 * height)),
                "material": "ground_mat", "friction": friction, "condim": "3",
            }))
    insert_at = 1 if len(worldbody) and worldbody[0].tag == "light" else 0
    for element in reversed(elements):
        worldbody.insert(insert_at, element)
    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    path = WORLD_DIR / f"{spec['episode_id']}.xml"
    ET.indent(tree, space="  ")
    tree.write(path, encoding="unicode", xml_declaration=False)
    mujoco.MjModel.from_xml_path(str(path))
    return path


def generate(config: dict, *, revision: bool = False) -> None:
    import run_stage3b_yaw_control as stage3b
    import run_stage4a_slope_robustness as stage4a
    import mujoco

    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    EPISODE_DIR.mkdir(parents=True, exist_ok=True)
    specs = build_revision_specs(config) if revision else build_episode_specs(config)
    output_manifest_path = REVISION_MANIFEST_PATH if revision else MANIFEST_PATH
    prior_episodes = []
    if MANIFEST_PATH.exists():
        prior_episodes.extend(load_json(MANIFEST_PATH)["episodes"])
    if revision and REVISION_MANIFEST_PATH.exists():
        prior_episodes.extend(load_json(REVISION_MANIFEST_PATH)["episodes"])
    prior_by_id = {item["episode_id"]: item for item in prior_episodes}
    common_base = list(stage3b.load_common())
    common_base[3] = copy.deepcopy(common_base[3])
    common_base[3]["history_frequency_hz"] = float(config["sample_frequency_hz"])
    nominal_theta = float(common_base[7]["parameters"]["theta_eq_rad"])
    wheel_radius = float(common_base[7]["parameters"]["wheel_radius_m"])
    manifest_episodes = []
    for number, spec in enumerate(specs, start=1):
        prior = prior_by_id.get(spec["episode_id"])
        identity_fields = (
            "alpha_deg", "friction", "payload", "terrain", "speed_m_s",
            "imu_seed", "terrain_parameters", "payload_mass_scale",
            "command_schedule",
            "friction_model_version", "duration_s",
        )
        if prior is not None and all(
            prior.get(name, 1.0 if name == "payload_mass_scale" else None)
            == spec.get(name, 1.0 if name == "payload_mass_scale" else None)
            for name in identity_fields
        ):
            data_path = (RESULT_DIR / prior["data_file"]).resolve()
            world_path = (ROOT / prior["world_file"]).resolve()
            if data_path.exists() and world_path.exists():
                manifest_episodes.append({**prior, **spec})
                print(f"REUSE {number:02d}/{len(specs):02d} {spec['episode_id']}", flush=True)
                continue
        print(f"GENERATE {number:02d}/{len(specs):02d} {spec['episode_id']}", flush=True)
        common = list(common_base)
        common[3] = copy.deepcopy(common_base[3])
        common[3]["imu_rng_seed"] = int(spec["imu_seed"])
        common = tuple(common)
        world_path = build_world(spec, config)
        adapter = stage4a.make_q_adapter(common, actuator_enabled=False)
        holder: dict[str, object] = {}

        class Recorder(stage4a.SlopeGroundTruthRecorder):
            def __init__(self, sim) -> None:
                super().__init__(sim, spec["alpha_deg"], config, nominal_theta)
                self.TERRAIN_GEOMS = {
                    sim.model.geom(index).name for index in range(sim.model.ngeom)
                    if sim.model.geom(index).name.startswith("terrain_")
                }

            def snapshot(self, sim) -> dict:
                result = super().snapshot(sim)
                grade_start = self.lead_m + self.transition_m
                along = float(result["along_track_position_GT_m"]) - grade_start
                rough = spec["terrain"] == "rough" and result["terrain_segment_GT"] == "constant_grade"
                bump = spec["terrain"] == "bump" and abs(along - 1.55) <= 0.25
                result["rough_GT"] = bool(rough or bump)
                result["terrain_kind_GT"] = spec["terrain"]
                return result

        def setup(sim):
            holder["recorder"] = Recorder(sim)
            left = sim.model.joint("left_wheel_hinge").id
            right = sim.model.joint("right_wheel_hinge").id
            holder["wheel_dofs"] = (
                int(sim.model.jnt_dofadr[left]), int(sim.model.jnt_dofadr[right])
            )
            payload_scale = float(spec.get("payload_mass_scale", 1.0))
            if spec["payload"] == "free" and not math.isclose(payload_scale, 1.0):
                payload_body = sim.model.body("moving_payload").id
                sim.model.body_mass[payload_body] *= payload_scale
                sim.model.body_inertia[payload_body] *= payload_scale
                mujoco.mj_setConst(sim.model, sim.data)
                mujoco.mj_forward(sim.model, sim.data)
            return {"stage4b_episode": spec["episode_id"]}

        def physics_step(sim):
            holder["recorder"].physics_step(sim)

        def diagnostic(sim):
            raw = sim.imu_raw_log_fields()
            snapshot = holder["recorder"].snapshot(sim)
            wheel_dofs = holder["wheel_dofs"]
            theta_dot = float(sim.longitudinal_state(nominal_theta)[3])
            wheel_surface = wheel_radius * (
                np.asarray([sim.data.qvel[wheel_dofs[0]], sim.data.qvel[wheel_dofs[1]]])
                + theta_dot
            )
            body_speed = float(snapshot["v_GT_along_track_m_s"])
            mean_surface = float(np.mean(wheel_surface))
            ratio = abs(mean_surface - body_speed) / max(
                abs(mean_surface), abs(body_speed), 0.10
            )
            active = max(abs(mean_surface), abs(body_speed)) >= 0.10
            return {
                **snapshot,
                "imu_accelerometer_noisy_m_s2": raw["imu_accelerometer_noisy_raw_m_s2"],
                "imu_gyro_noisy_rad_s": raw["imu_gyro_noisy_raw_rad_s"],
                "wheel_surface_velocity_m_s": wheel_surface.tolist(),
                "slip_ratio_GT": ratio,
                "slip_GT": bool(active and ratio >= float(config["slip_ratio_threshold"])),
            }

        def guard(sim):
            return holder["recorder"].boundary_guard(sim)

        scenario = {
            "name": spec["episode_id"],
            "duration_s": float(spec.get("duration_s", config["episode_duration_s"])),
            "linear_velocity_schedule": spec.get("command_schedule") or [
                {"time_s": 0.0, "command": 0.0},
                {"time_s": 0.6, "command": float(spec["speed_m_s"])},
            ],
            "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
        }
        manifest = common[0]
        run = stage3b.run_case(
            scenario,
            float(manifest["yaw"]["K_psi_nm_per_rad"]),
            float(manifest["yaw"]["K_r_nm_per_rad_s"]),
            yaw_enabled=True, motor_mismatch_enabled=False, common=common,
            keep_history=True, payload_mode=spec["payload"],
            common_mode_augmentation=adapter,
            physics_step_callback=physics_step,
            simulation_setup_callback=setup,
            model_path_override=world_path,
            history_diagnostic_callback=diagnostic,
            termination_guard=guard,
        )
        rows = run["history_50hz"]
        times = np.asarray([row["t"] for row in rows], dtype=np.float64)
        v_ref = np.asarray([row["v_ref_m_s"] for row in rows], dtype=np.float64)
        a_ref = np.gradient(v_ref, times)
        features = []
        for index, row in enumerate(rows):
            acc = row["imu_accelerometer_noisy_m_s2"]
            gyro = row["imu_gyro_noisy_rad_s"]
            wheel = row["wheel_surface_velocity_m_s"]
            features.append([
                *acc, *gyro, *wheel,
                row["u_left_nm"], row["u_right_nm"],
                row["actual_left_nm"], row["actual_right_nm"],
                row["v_hat_m_s"], row["theta_hat_rad"], row["theta_dot_hat_rad_s"],
                row["v_ref_m_s"], a_ref[index], row["q_filter_estimate_nm"],
                row["matched_residual_fraction"], row["scaled_innovation_rms"],
            ])
        episode_path = EPISODE_DIR / f"{spec['episode_id']}.npz"
        np.savez_compressed(
            episode_path,
            time_s=times,
            features=np.asarray(features, dtype=np.float32),
            alpha_gt_deg=np.degrees(np.asarray([row["alpha_GT_rad"] for row in rows])),
            slip_ratio_gt=np.asarray([row["slip_ratio_GT"] for row in rows], dtype=np.float32),
            slip_gt=np.asarray([row["slip_GT"] for row in rows], dtype=np.uint8),
            rough_gt=np.asarray([row["rough_GT"] for row in rows], dtype=np.uint8),
            constant_grade=np.asarray([
                row["terrain_segment_GT"] == "constant_grade" for row in rows
            ], dtype=np.uint8),
        )
        constant_count = int(sum(row["terrain_segment_GT"] == "constant_grade" for row in rows))
        status = "usable" if constant_count >= round(
            float(config["window_seconds"]) * float(config["sample_frequency_hz"])
        ) else "insufficient_constant_grade"
        manifest_episodes.append({
            **spec,
            "status": status,
            "data_file": os.path.relpath(episode_path, RESULT_DIR).replace("\\", "/"),
            "world_file": str(world_path.relative_to(ROOT)).replace("\\", "/"),
            "sample_count": len(rows),
            "constant_grade_sample_count": constant_count,
            "slip_sample_fraction": float(np.mean([row["slip_GT"] for row in rows])),
            "fell": bool(run["longitudinal"]["fell"]),
            "termination": run["simulation_termination"],
        })
    train_specs = [item for item in manifest_episodes if item["split"] == "train"]
    factor_names = ("friction", "payload", "terrain", "speed_m_s")
    per_slope_factor_levels = {
        str(alpha): {
            name: len({item[name] for item in train_specs if item["alpha_deg"] == alpha})
            for name in factor_names
        }
        for alpha in sorted({item["alpha_deg"] for item in train_specs})
    }
    manifest = {
        "stage": config["revision"]["name"] if revision else config["stage"],
        "revision_of": str(MANIFEST_PATH.relative_to(ROOT)).replace("\\", "/") if revision else None,
        "feature_names": FEATURE_NAMES,
        "sample_frequency_hz": config["sample_frequency_hz"],
        "window_seconds": config["window_seconds"],
        "step_seconds": config["step_seconds"],
        "split_policy": "episode-exclusive before window indexing",
        "Q_observer": "always ON",
        "Q_actuator": "OFF for all generated episodes",
        "slip_definition": {
            "ratio": "abs(mean wheel-surface speed - GT chassis tangential speed) / max(abs speeds, 0.10 m/s)",
            "threshold": config["slip_ratio_threshold"],
            "minimum_active_speed_m_s": 0.10,
        },
        "factorization_audit": {
            "method": "independent seeded permutation of every nuisance level within each slope's three repeats",
            "per_slope_distinct_level_counts": per_slope_factor_levels,
            "all_three_levels_present_for_each_slope_and_factor": all(
                count >= 3
                for counts in per_slope_factor_levels.values()
                for count in counts.values()
            ),
            "unique_train_full_factor_tuples": len({
                (item["alpha_deg"], item["friction"], item["payload"],
                 item["terrain"], item["speed_m_s"])
                for item in train_specs
            }),
        },
        "episodes": manifest_episodes,
    }
    write_json(output_manifest_path, manifest)
    print(f"WROTE {output_manifest_path}", flush=True)


def binary_metrics(y: np.ndarray, probability: np.ndarray) -> dict:
    truth = np.asarray(y, dtype=int)
    pred = np.asarray(probability >= 0.5, dtype=int)
    tp = int(np.sum((truth == 1) & (pred == 1)))
    fp = int(np.sum((truth == 0) & (pred == 1)))
    tn = int(np.sum((truth == 0) & (pred == 0)))
    fn = int(np.sum((truth == 1) & (pred == 0)))
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    return {
        "precision": precision, "recall": recall,
        "f1": 2 * precision * recall / max(precision + recall, 1e-12),
        "false_positive_rate": fp / max(fp + tn, 1),
        "confusion_matrix": [[tn, fp], [fn, tp]],
        "positive_count": int(np.sum(truth)), "negative_count": int(np.sum(1 - truth)),
    }


def slope_metrics(y: np.ndarray, prediction: np.ndarray) -> dict:
    error = np.asarray(prediction) - np.asarray(y)
    return {
        "mae_deg": float(np.mean(np.abs(error))),
        "rmse_deg": float(np.sqrt(np.mean(error * error))),
        "p95_abs_error_deg": float(np.percentile(np.abs(error), 95)),
        "max_abs_error_deg": float(np.max(np.abs(error))),
        "bias_deg": float(np.mean(error)),
        "count_abs_error_gt_2deg": int(np.sum(np.abs(error) > 2.0)),
        "count_abs_error_gt_3deg": int(np.sum(np.abs(error) > 3.0)),
        "count_abs_error_gt_5deg": int(np.sum(np.abs(error) > 5.0)),
    }


def train_and_evaluate(config: dict) -> None:
    from sklearn.linear_model import LogisticRegression, Ridge
    import joblib
    import torch
    from torch import nn
    from torch.utils.data import DataLoader, TensorDataset
    from learning.tiny_temporal_cnn import TinyTemporalCNN

    np.random.seed(int(config["seed"]))
    torch.manual_seed(int(config["seed"]))
    window = round(float(config["window_seconds"]) * float(config["sample_frequency_hz"]))
    step = round(float(config["step_seconds"]) * float(config["sample_frequency_hz"]))
    datasets = {
        split: LazyEpisodeWindowDataset(MANIFEST_PATH, split, window_samples=window,
                                        step_samples=step)
        for split in ("train", "val", "test")
    }
    normalization = fit_train_normalization(datasets["train"])
    arrays = {split: materialize(ds, normalization) for split, ds in datasets.items()}
    if not all(len(value["alpha"]) for value in arrays.values()):
        raise RuntimeError("one or more splits has no valid constant-grade windows")

    ridge = Ridge(alpha=1.0).fit(arrays["train"]["x_stats"], arrays["train"]["alpha"])
    classifiers = {}
    for target in ("slip", "rough"):
        labels = arrays["train"][target]
        if len(np.unique(labels)) < 2:
            raise RuntimeError(f"train split has only one {target} class")
        classifiers[target] = LogisticRegression(
            C=1.0, class_weight="balanced", max_iter=1000,
            random_state=int(config["seed"]),
        ).fit(arrays["train"]["x_stats"], labels)
    simple_predictions = {}
    for split, values in arrays.items():
        simple_predictions[split] = {
            "alpha": ridge.predict(values["x_stats"]),
            "slip": classifiers["slip"].predict_proba(values["x_stats"])[:, 1],
            "rough": classifiers["rough"].predict_proba(values["x_stats"])[:, 1],
        }

    train_values, val_values = arrays["train"], arrays["val"]
    train_loader = DataLoader(TensorDataset(
        torch.from_numpy(train_values["x_raw"]),
        torch.from_numpy(train_values["alpha"]),
        torch.from_numpy(train_values["slip"]),
        torch.from_numpy(train_values["rough"]),
    ), batch_size=int(config["cnn"]["batch_size"]), shuffle=True,
       generator=torch.Generator().manual_seed(int(config["seed"])))
    model = TinyTemporalCNN(len(FEATURE_NAMES))
    parameter_count = sum(parameter.numel() for parameter in model.parameters())
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=float(config["cnn"]["learning_rate"]),
        weight_decay=float(config["cnn"]["weight_decay"]),
    )
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="min", factor=0.5, patience=4
    )
    slip_pos = float(np.sum(train_values["slip"]))
    rough_pos = float(np.sum(train_values["rough"]))
    slip_weight = torch.tensor((len(train_values["slip"]) - slip_pos) / max(slip_pos, 1.0))
    rough_weight = torch.tensor((len(train_values["rough"]) - rough_pos) / max(rough_pos, 1.0))
    bce_slip = nn.BCEWithLogitsLoss(pos_weight=slip_weight)
    bce_rough = nn.BCEWithLogitsLoss(pos_weight=rough_weight)
    CHECKPOINT_DIR.mkdir(parents=True, exist_ok=True)
    checkpoint = CHECKPOINT_DIR / "tiny_temporal_cnn_best.pt"
    best_loss = math.inf
    best_epoch = 0
    stale = 0
    history = []

    def loss_for(x, alpha, slip, rough):
        pa, ps, pr = model(x)
        alpha_loss = ((pa - alpha) / 15.0).pow(2).mean()
        return (
            float(config["cnn"]["alpha_loss_weight"]) * alpha_loss
            + bce_slip(ps, slip)
            + bce_rough(pr, rough)
        )

    for epoch in range(1, int(config["cnn"]["epochs"]) + 1):
        model.train()
        train_losses = []
        for batch in train_loader:
            optimizer.zero_grad()
            loss = loss_for(*batch)
            loss.backward()
            optimizer.step()
            train_losses.append(float(loss.detach()))
        model.eval()
        with torch.no_grad():
            val_loss = float(loss_for(
                torch.from_numpy(val_values["x_raw"]),
                torch.from_numpy(val_values["alpha"]),
                torch.from_numpy(val_values["slip"]),
                torch.from_numpy(val_values["rough"]),
            ))
        scheduler.step(val_loss)
        history.append({"epoch": epoch, "train_loss": float(np.mean(train_losses)),
                        "val_loss": val_loss, "lr": optimizer.param_groups[0]["lr"]})
        if val_loss < best_loss - 1e-5:
            best_loss, best_epoch, stale = val_loss, epoch, 0
            torch.save(model.state_dict(), checkpoint)
        else:
            stale += 1
            if stale >= int(config["cnn"]["patience"]):
                break
    model.load_state_dict(torch.load(checkpoint, map_location="cpu", weights_only=True))
    model.eval()
    cnn_predictions = {}
    with torch.no_grad():
        for split, values in arrays.items():
            alpha, slip, rough = model(torch.from_numpy(values["x_raw"]))
            cnn_predictions[split] = {
                "alpha": alpha.numpy(),
                "slip": torch.sigmoid(slip).numpy(),
                "rough": torch.sigmoid(rough).numpy(),
            }

    def evaluate(predictions: dict) -> dict:
        return {split: {
            "slope": slope_metrics(arrays[split]["alpha"], predictions[split]["alpha"]),
            "slip": binary_metrics(arrays[split]["slip"], predictions[split]["slip"]),
            "rough": binary_metrics(arrays[split]["rough"], predictions[split]["rough"]),
        } for split in arrays}

    manifest = load_json(MANIFEST_PATH)
    metadata = {item["episode_id"]: item for item in manifest["episodes"]}

    def episode_predictions(predictions: dict) -> list[dict]:
        values = arrays["test"]
        rows = []
        for episode_id in sorted(set(values["episode_ids"])):
            mask = values["episode_ids"] == episode_id
            meta = metadata[episode_id]
            rows.append({
                "episode_id": episode_id,
                "evaluation_kind": meta["evaluation_kind"],
                "evaluation_group": meta["evaluation_group"],
                "alpha_gt_deg": meta["alpha_deg"],
                "friction": meta["friction"], "payload": meta["payload"],
                "terrain": meta["terrain"], "speed_m_s": meta["speed_m_s"],
                "alpha_hat_deg": float(np.mean(predictions["test"]["alpha"][mask])),
                "slip_probability": float(np.max(predictions["test"]["slip"][mask])),
                "rough_probability": float(np.max(predictions["test"]["rough"][mask])),
                "slip_gt": bool(np.max(values["slip"][mask])),
                "rough_gt": bool(np.max(values["rough"][mask])),
            })
        return rows

    def counterfactual(rows: list[dict]) -> dict:
        output = {}
        for group in ("cf_friction", "cf_payload", "cf_alpha_monotonic",
                      "cf_low_mu_slip", "cf_unseen_terrain"):
            selected = [row for row in rows if row["evaluation_group"] == group]
            hats = np.asarray([row["alpha_hat_deg"] for row in selected], dtype=float)
            item = {"episodes": selected}
            if group in {"cf_friction", "cf_payload"} and len(hats):
                item["alpha_prediction_drift_deg"] = float(np.max(hats) - np.min(hats))
            if group == "cf_alpha_monotonic" and len(hats):
                ordered = sorted(selected, key=lambda row: row["alpha_gt_deg"])
                item["strictly_monotonic"] = bool(np.all(np.diff([
                    row["alpha_hat_deg"] for row in ordered
                ]) > 0.0))
            output[group] = item
        return output

    def generalization_breakdown(rows: list[dict]) -> dict:
        def grouped(field: str) -> dict:
            result = {}
            for value in sorted({str(row[field]) for row in rows}):
                selected_rows = [row for row in rows if str(row[field]) == value]
                errors = np.asarray([
                    row["alpha_hat_deg"] - row["alpha_gt_deg"] for row in selected_rows
                ])
                result[value] = {
                    "episode_count": len(selected_rows),
                    "mae_deg": float(np.mean(np.abs(errors))),
                    "bias_deg": float(np.mean(errors)),
                    "p95_abs_error_deg": float(np.percentile(np.abs(errors), 95)),
                }
            return result
        return {
            "per_slope": grouped("alpha_gt_deg"),
            "per_friction": grouped("friction"),
            "per_payload": grouped("payload"),
            "per_evaluation_kind": grouped("evaluation_kind"),
        }

    # CPU latency: batch-one, no data loading.
    sample = torch.from_numpy(arrays["test"]["x_raw"][:1])
    with torch.no_grad():
        for _ in range(20):
            model(sample)
        start = time.perf_counter()
        for _ in range(200):
            model(sample)
        latency_ms = 1000.0 * (time.perf_counter() - start) / 200.0
    checkpoint_bytes = checkpoint.stat().st_size
    simple_rows = episode_predictions(simple_predictions)
    cnn_rows = episode_predictions(cnn_predictions)
    simple_eval = evaluate(simple_predictions)
    cnn_eval = evaluate(cnn_predictions)
    simple_gate = (
        simple_eval["test"]["slope"]["mae_deg"] <= config["gates"]["slope_mae_deg"]
        and simple_eval["test"]["slope"]["p95_abs_error_deg"] <= config["gates"]["slope_p95_deg"]
    )
    cnn_gate = (
        cnn_eval["test"]["slope"]["mae_deg"] <= config["gates"]["slope_mae_deg"]
        and cnn_eval["test"]["slope"]["p95_abs_error_deg"] <= config["gates"]["slope_p95_deg"]
    )
    # Prefer the cheap model unless CNN delivers a material 0.2-degree MAE gain.
    selected = "simple" if simple_gate and (
        not cnn_gate or simple_eval["test"]["slope"]["mae_deg"]
        <= cnn_eval["test"]["slope"]["mae_deg"] + 0.20
    ) else ("cnn" if cnn_gate else "reject")
    metrics = {
        "window_counts": {split: len(ds) for split, ds in datasets.items()},
        "episode_counts": {split: len(ds.episode_metadata) for split, ds in datasets.items()},
        "normalization": normalization,
        "simple": {"metrics": simple_eval, "counterfactual": counterfactual(simple_rows),
                   "generalization_breakdown": generalization_breakdown(simple_rows),
                   "episode_predictions": simple_rows},
        "cnn": {
            "metrics": cnn_eval, "counterfactual": counterfactual(cnn_rows),
            "generalization_breakdown": generalization_breakdown(cnn_rows),
            "episode_predictions": cnn_rows, "parameter_count": parameter_count,
            "checkpoint_bytes": checkpoint_bytes,
            "estimated_fp32_parameter_memory_bytes": 4 * parameter_count,
            "inference_latency_cpu_ms": latency_ms,
            "best_epoch": best_epoch, "best_val_loss": best_loss,
            "training_history": history,
        },
        "decision": {"selected": selected, "simple_slope_gate": simple_gate,
                     "cnn_slope_gate": cnn_gate,
                     "rule": "prefer simple unless CNN improves test MAE by >0.20 deg"},
    }
    write_json(METRICS_PATH, metrics)
    if selected == "simple":
        selected_model = {
            "model": "Ridge(alpha=1.0) over normalized window statistics",
            "feature_names": FEATURE_NAMES,
            "statistics_order": ["mean", "std", "min", "max", "rms", "diff_rms"],
            "window_samples": window, "sample_frequency_hz": config["sample_frequency_hz"],
            "normalization": normalization,
            "alpha_coef": ridge.coef_.tolist(), "alpha_intercept": float(ridge.intercept_),
            "slip_coef": classifiers["slip"].coef_[0].tolist(),
            "slip_intercept": float(classifiers["slip"].intercept_[0]),
            "rough_coef": classifiers["rough"].coef_[0].tolist(),
            "rough_intercept": float(classifiers["rough"].intercept_[0]),
            "source": "train split only",
        }
        write_json(SELECTED_MODEL_PATH, selected_model)
    joblib.dump({"ridge": ridge, **classifiers}, CHECKPOINT_DIR / "simple_models.joblib")
    print(f"WROTE {METRICS_PATH}; selected={selected}", flush=True)


def train_and_evaluate_revision(config: dict) -> None:
    """Final fixed Stage4B-R qualification: long Ridge, event CNN, bounded residual."""

    from sklearn.linear_model import Ridge
    from sklearn.preprocessing import StandardScaler
    import torch
    from torch import nn
    from torch.utils.data import DataLoader, TensorDataset
    from learning.environment_dataset import long_context_statistical_features
    from learning.tiny_temporal_cnn import BoundedSlopeResidual, TinyTemporalCNN

    seed = int(config["seed"]) + 41002
    np.random.seed(seed)
    torch.manual_seed(seed)
    revision = config["revision"]
    short_samples = round(config["window_seconds"] * config["sample_frequency_hz"])
    long_samples = round(revision["long_window_seconds"] * config["sample_frequency_hz"])
    step_samples = round(config["step_seconds"] * config["sample_frequency_hz"])
    label_tail = round(revision["label_tail_seconds"] * config["sample_frequency_hz"])

    long_datasets = {
        split: LazyEpisodeWindowDataset(
            REVISION_MANIFEST_PATH, split, window_samples=long_samples,
            step_samples=step_samples, window_selection="end_constant_grade",
            label_tail_samples=label_tail,
        ) for split in ("train", "val", "test")
    }
    short_datasets = {
        split: LazyEpisodeWindowDataset(
            REVISION_MANIFEST_PATH, split, window_samples=short_samples,
            step_samples=step_samples, window_selection="all",
            label_tail_samples=min(label_tail, short_samples),
        ) for split in ("train", "val", "test")
    }
    normalization = fit_train_normalization(long_datasets["train"])

    def materialize_long(dataset: LazyEpisodeWindowDataset) -> dict:
        values = {name: [] for name in (
            "stats", "short_raw", "alpha", "episode_ids", "steady",
        )}
        for sample in dataset:
            normalized = normalize_window(sample["x"], normalization)
            values["stats"].append(long_context_statistical_features(normalized))
            values["short_raw"].append(normalized[:, -short_samples:])
            values["alpha"].append(sample["alpha_deg"])
            values["episode_ids"].append(sample["episode_id"])
            values["steady"].append(sample["fully_constant_grade"])
        return {
            "stats": np.asarray(values["stats"], dtype=np.float32),
            "short_raw": np.asarray(values["short_raw"], dtype=np.float32),
            "alpha": np.asarray(values["alpha"], dtype=np.float32),
            "episode_ids": np.asarray(values["episode_ids"]),
            "steady": np.asarray(values["steady"], dtype=bool),
        }

    long_values = {split: materialize_long(ds) for split, ds in long_datasets.items()}
    short_values = {split: materialize(ds, normalization) for split, ds in short_datasets.items()}
    if not all(len(value["alpha"]) for value in long_values.values()):
        raise RuntimeError("Stage4B-R long-context split has no windows")

    scaler = StandardScaler().fit(long_values["train"]["stats"])
    scaled_stats = {
        split: scaler.transform(value["stats"]).astype(np.float32)
        for split, value in long_values.items()
    }
    ridge = Ridge(alpha=1.0).fit(
        scaled_stats["train"], long_values["train"]["alpha"]
    )
    ridge_predictions = {
        split: ridge.predict(scaled_stats[split]).astype(np.float32)
        for split in long_values
    }
    manifest = load_json(REVISION_MANIFEST_PATH)
    metadata = {episode["episode_id"]: episode for episode in manifest["episodes"]}

    def episode_slope_rows(predictions: np.ndarray) -> list[dict]:
        values = long_values["test"]
        rows = []
        for episode_id in sorted(set(values["episode_ids"])):
            mask = values["episode_ids"] == episode_id
            meta = metadata[episode_id]
            rows.append({
                "episode_id": episode_id,
                "evaluation_kind": meta["evaluation_kind"],
                "evaluation_group": meta.get("evaluation_group"),
                "coupling_class": meta.get("coupling_class"),
                "alpha_gt_deg": float(np.median(values["alpha"][mask])),
                "alpha_hat_deg": float(np.median(predictions[mask])),
                "friction": meta["friction"], "payload": meta["payload"],
                "terrain": meta["terrain"], "speed_m_s": meta["speed_m_s"],
            })
        return rows

    def counterfactual(rows: list[dict]) -> dict:
        output = {}
        for group in ("cf_friction", "cf_payload", "cf_alpha_monotonic",
                      "cf_low_mu_slip", "cf_unseen_terrain", "cf_coupling"):
            selected = [row for row in rows if row["evaluation_group"] == group]
            item = {"episodes": selected}
            if group in {"cf_friction", "cf_payload"} and selected:
                hats = [row["alpha_hat_deg"] for row in selected]
                item["alpha_prediction_drift_deg"] = float(max(hats) - min(hats))
            if group == "cf_alpha_monotonic" and selected:
                ordered = sorted(selected, key=lambda row: row["alpha_gt_deg"])
                item["strictly_monotonic"] = bool(np.all(np.diff([
                    row["alpha_hat_deg"] for row in ordered
                ]) > 0.0))
            output[group] = item
        return output

    def slope_evaluation(predictions: dict[str, np.ndarray]) -> dict:
        result = {}
        for split in ("train", "val", "test"):
            values = long_values[split]
            result[split] = {
                "online_window": slope_metrics(values["alpha"], predictions[split]),
                "steady_window": slope_metrics(
                    values["alpha"][values["steady"]], predictions[split][values["steady"]]
                ),
            }
        rows = episode_slope_rows(predictions["test"])
        result["test"]["episode_aggregated"] = slope_metrics(
            np.asarray([row["alpha_gt_deg"] for row in rows]),
            np.asarray([row["alpha_hat_deg"] for row in rows]),
        )
        result["test"]["counterfactual"] = counterfactual(rows)
        result["test"]["episode_predictions"] = rows
        by_kind = {}
        test_ids = long_values["test"]["episode_ids"]
        for kind in sorted({metadata[item]["evaluation_kind"] for item in test_ids}):
            mask = np.asarray([
                metadata[item]["evaluation_kind"] == kind for item in test_ids
            ])
            by_kind[kind] = slope_metrics(
                long_values["test"]["alpha"][mask], predictions["test"][mask]
            )
        result["test"]["per_evaluation_kind"] = by_kind
        coupling = {}
        for coupling_class in sorted({
            str(metadata[item].get("coupling_class")) for item in test_ids
            if metadata[item].get("coupling_class") is not None
        }):
            mask = np.asarray([
                metadata[item].get("coupling_class") == coupling_class for item in test_ids
            ])
            coupling[coupling_class] = slope_metrics(
                long_values["test"]["alpha"][mask], predictions["test"][mask]
            )
        result["test"]["coupling_classes"] = coupling
        return result

    def passes_slope_gate(evaluation: dict) -> tuple[bool, dict]:
        test = evaluation["test"]
        gates = config["gates"]
        required_kinds = ("numeric_ood", "combination_ood", "coupled_disturbance_ood")
        checks = {
            "overall_mae": test["online_window"]["mae_deg"] <= gates["slope_mae_deg"],
            "overall_p95": test["online_window"]["p95_abs_error_deg"] <= gates["slope_p95_deg"],
            "friction_drift": test["counterfactual"]["cf_friction"].get(
                "alpha_prediction_drift_deg", math.inf
            ) <= gates["counterfactual_drift_deg"],
            "payload_drift": test["counterfactual"]["cf_payload"].get(
                "alpha_prediction_drift_deg", math.inf
            ) <= gates["counterfactual_drift_deg"],
        }
        for kind in required_kinds:
            item = test["per_evaluation_kind"].get(kind)
            checks[f"{kind}_mae"] = item is not None and item["mae_deg"] <= gates["slope_mae_deg"]
            checks[f"{kind}_p95"] = item is not None and item["p95_abs_error_deg"] <= gates["slope_p95_deg"]
        return all(checks.values()), checks

    ridge_evaluation = slope_evaluation(ridge_predictions)
    ridge_passed, ridge_checks = passes_slope_gate(ridge_evaluation)

    # Train the existing short encoder only for transient event heads.
    train_short, val_short = short_values["train"], short_values["val"]
    train_loader = DataLoader(TensorDataset(
        torch.from_numpy(train_short["x_raw"]),
        torch.from_numpy(train_short["slip"]),
        torch.from_numpy(train_short["rough"]),
    ), batch_size=int(revision["batch_size"]), shuffle=True,
       generator=torch.Generator().manual_seed(seed))
    event_model = TinyTemporalCNN(len(FEATURE_NAMES))
    event_optimizer = torch.optim.AdamW(
        event_model.parameters(), lr=float(revision["learning_rate"]),
        weight_decay=float(revision["weight_decay"]),
    )
    event_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        event_optimizer, mode="min", factor=0.5, patience=4
    )

    def positive_weight(labels: np.ndarray) -> torch.Tensor:
        positives = float(np.sum(labels))
        return torch.tensor((len(labels) - positives) / max(positives, 1.0))

    slip_loss = nn.BCEWithLogitsLoss(pos_weight=positive_weight(train_short["slip"]))
    rough_loss = nn.BCEWithLogitsLoss(pos_weight=positive_weight(train_short["rough"]))

    def event_loss(x, slip, rough):
        slip_logits, rough_logits = event_model.forward_events(x)
        return slip_loss(slip_logits, slip) + rough_loss(rough_logits, rough)

    CHECKPOINT_DIR.mkdir(parents=True, exist_ok=True)
    event_checkpoint = CHECKPOINT_DIR / "stage4br_event_cnn_best.pt"
    best_event_loss, best_event_epoch, stale = math.inf, 0, 0
    event_history = []
    for epoch in range(1, int(revision["event_epochs"]) + 1):
        event_model.train()
        losses = []
        for batch in train_loader:
            event_optimizer.zero_grad()
            loss = event_loss(*batch)
            loss.backward()
            event_optimizer.step()
            losses.append(float(loss.detach()))
        event_model.eval()
        with torch.no_grad():
            val_loss = float(event_loss(
                torch.from_numpy(val_short["x_raw"]),
                torch.from_numpy(val_short["slip"]),
                torch.from_numpy(val_short["rough"]),
            ))
        event_scheduler.step(val_loss)
        event_history.append({"epoch": epoch, "train_loss": float(np.mean(losses)),
                              "val_loss": val_loss})
        if val_loss < best_event_loss - 1e-5:
            best_event_loss, best_event_epoch, stale = val_loss, epoch, 0
            torch.save(event_model.state_dict(), event_checkpoint)
        else:
            stale += 1
            if stale >= int(revision["patience"]):
                break
    event_model.load_state_dict(torch.load(
        event_checkpoint, map_location="cpu", weights_only=True
    ))
    event_model.eval()
    event_evaluation = {}
    with torch.no_grad():
        for split, values in short_values.items():
            slip_logits, rough_logits = event_model.forward_events(
                torch.from_numpy(values["x_raw"])
            )
            event_evaluation[split] = {
                "slip": binary_metrics(values["slip"], torch.sigmoid(slip_logits).numpy()),
                "rough": binary_metrics(values["rough"], torch.sigmoid(rough_logits).numpy()),
            }

    hybrid_predictions = None
    residual_evaluation = None
    residual_training = {"executed": False, "reason": "long Ridge passed all slope gates"}
    if not ridge_passed:
        for parameter in event_model.parameters():
            parameter.requires_grad_(False)
        with torch.no_grad():
            latents = {
                split: event_model.encode(torch.from_numpy(values["short_raw"])).numpy()
                for split, values in long_values.items()
            }
        residual_inputs = {
            split: np.concatenate([
                ridge_predictions[split][:, None], scaled_stats[split], latents[split]
            ], axis=1).astype(np.float32)
            for split in long_values
        }
        residual_targets = {
            split: (long_values[split]["alpha"] - ridge_predictions[split]).astype(np.float32)
            for split in long_values
        }
        residual_model = BoundedSlopeResidual(
            residual_inputs["train"].shape[1],
            float(revision["maximum_residual_correction_deg"]),
        )
        residual_optimizer = torch.optim.AdamW(
            residual_model.parameters(), lr=float(revision["learning_rate"]),
            weight_decay=float(revision["weight_decay"]),
        )
        residual_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
            residual_optimizer, mode="min", factor=0.5, patience=4
        )
        residual_loader = DataLoader(TensorDataset(
            torch.from_numpy(residual_inputs["train"]),
            torch.from_numpy(residual_targets["train"]),
        ), batch_size=int(revision["batch_size"]), shuffle=True,
           generator=torch.Generator().manual_seed(seed + 1))
        residual_checkpoint = CHECKPOINT_DIR / "stage4br_bounded_residual_best.pt"
        best_residual_loss, best_residual_epoch, stale = math.inf, 0, 0
        residual_history = []
        for epoch in range(1, int(revision["residual_epochs"]) + 1):
            residual_model.train()
            losses = []
            for x, target in residual_loader:
                residual_optimizer.zero_grad()
                correction = residual_model(x)
                loss = torch.mean((correction - target) ** 2)
                loss.backward()
                residual_optimizer.step()
                losses.append(float(loss.detach()))
            residual_model.eval()
            with torch.no_grad():
                val_loss = float(torch.mean((
                    residual_model(torch.from_numpy(residual_inputs["val"]))
                    - torch.from_numpy(residual_targets["val"])
                ) ** 2))
            residual_scheduler.step(val_loss)
            residual_history.append({"epoch": epoch, "train_loss": float(np.mean(losses)),
                                     "val_loss": val_loss})
            if val_loss < best_residual_loss - 1e-5:
                best_residual_loss, best_residual_epoch, stale = val_loss, epoch, 0
                torch.save(residual_model.state_dict(), residual_checkpoint)
            else:
                stale += 1
                if stale >= int(revision["patience"]):
                    break
        residual_model.load_state_dict(torch.load(
            residual_checkpoint, map_location="cpu", weights_only=True
        ))
        residual_model.eval()
        with torch.no_grad():
            corrections = {
                split: residual_model(torch.from_numpy(residual_inputs[split])).numpy()
                for split in long_values
            }
        hybrid_predictions = {
            split: ridge_predictions[split] + corrections[split]
            for split in long_values
        }
        residual_evaluation = slope_evaluation(hybrid_predictions)
        hybrid_passed, hybrid_checks = passes_slope_gate(residual_evaluation)
        residual_parameter_count = sum(
            parameter.numel() for parameter in residual_model.parameters()
        )
        residual_training = {
            "executed": True,
            "encoder_frozen": True,
            "joint_finetune": False,
            "maximum_correction_deg": revision["maximum_residual_correction_deg"],
            "best_epoch": best_residual_epoch,
            "best_val_mse_deg2": best_residual_loss,
            "parameter_count": residual_parameter_count,
            "total_learned_parameter_count": residual_parameter_count + sum(
                parameter.numel() for parameter in event_model.parameters()
            ),
            "training_history": residual_history,
            "gate_checks": hybrid_checks,
            "passed": hybrid_passed,
        }
    else:
        hybrid_passed = False

    final_decision = (
        "KEEP_RIDGE" if ridge_passed
        else "KEEP_HYBRID" if hybrid_passed
        else "PERMANENT_REJECT"
    )
    train_ids = {episode["episode_id"] for episode in manifest["episodes"] if episode["split"] == "train"}
    val_ids = {episode["episode_id"] for episode in manifest["episodes"] if episode["split"] == "val"}
    test_ids = {episode["episode_id"] for episode in manifest["episodes"] if episode["split"] == "test"}
    label_audit = {
        "source": "per-timestamp local terrain grade from simulator geometry",
        "window_target": f"median of final {revision['label_tail_seconds']} s local grade",
        "long_window_selection": "final label-tail samples must be on constant-grade segment; earlier context may include flat/entry",
        "GT_in_feature_names": any("GT" in name or "alpha" in name for name in FEATURE_NAMES),
    }
    metrics = {
        "stage": revision["name"],
        "baseline_results_source": str(METRICS_PATH.relative_to(ROOT)).replace("\\", "/"),
        "old_baselines": load_json(METRICS_PATH),
        "data_coverage": {
            "episode_counts": {split: sum(
                episode["split"] == split for episode in manifest["episodes"]
            ) for split in ("train", "val", "test")},
            "train_episodes_per_slope": {
                str(alpha): sum(
                    episode["split"] == "train" and episode["alpha_deg"] == alpha
                    for episode in manifest["episodes"]
                ) for alpha in config["train_slopes_deg"]
            },
            "long_window_counts": {split: len(ds) for split, ds in long_datasets.items()},
            "short_window_counts": {split: len(ds) for split, ds in short_datasets.items()},
        },
        "integrity_audit": {
            "episode_split_disjoint": not (train_ids & val_ids or train_ids & test_ids or val_ids & test_ids),
            "numeric_ood_slopes_absent_from_train": not (
                set(config["numeric_ood_slopes_deg"])
                & {episode["alpha_deg"] for episode in manifest["episodes"] if episode["split"] == "train"}
            ),
            "normalization_fit_split": "train",
            "label_semantics": label_audit,
        },
        "normalization": normalization,
        "long_ridge": {
            "feature_count": int(scaled_stats["train"].shape[1]),
            "ridge_alpha": 1.0,
            "evaluation": ridge_evaluation,
            "gate_checks": ridge_checks,
            "passed": ridge_passed,
        },
        "short_event_cnn": {
            "parameter_count": sum(parameter.numel() for parameter in event_model.parameters()),
            "best_epoch": best_event_epoch,
            "best_val_loss": best_event_loss,
            "evaluation": event_evaluation,
            "training_history": event_history,
        },
        "bounded_residual": {
            **residual_training,
            "evaluation": residual_evaluation,
        },
        "decision": {
            "slope": final_decision,
            "slip": (
                "KEEP_EVENT_CNN" if event_evaluation["test"]["slip"]["recall"]
                >= config["gates"]["slip_recall"] else "DEFER_REJECT_PRODUCTION"
            ),
            "rough": (
                "KEEP_CHEAP_BASELINE" if load_json(METRICS_PATH)["simple"]["metrics"]["test"]["rough"]["f1"]
                >= event_evaluation["test"]["rough"]["f1"] else "KEEP_EVENT_CNN_RESULT_ONLY"
            ),
        },
    }
    write_json(REVISION_METRICS_PATH, metrics)
    print(f"WROTE {REVISION_METRICS_PATH}; slope={final_decision}", flush=True)


def analytic_theta_eq(alpha_rad: float, reduced: dict) -> float:
    """Quasi-static wheel-force/body-moment equilibrium on a grade."""

    p = reduced["parameters"]
    body_mass = float(p["body_mass_kg"])
    wheel_mass = float(p["single_wheel_rotating_mass_kg"])
    total_mass = body_mass + 2.0 * wheel_mass
    ratio = (
        float(p["wheel_radius_m"]) * total_mass
        / (body_mass * float(p["body_com_length_m"]))
    )
    return float(p["theta_eq_rad"]) + math.asin(float(np.clip(
        ratio * math.sin(float(alpha_rad)), -1.0, 1.0
    )))


class RuntimeSimpleSlopeEstimator:
    """Causal 100 Hz rolling-window inference using the exported ridge model."""

    def __init__(self, model: dict, adapter, wheel_radius_m: float) -> None:
        self.model = model
        self.adapter = adapter
        self.wheel_radius_m = float(wheel_radius_m)
        self.rows: list[np.ndarray] = []
        self.last_sample_time = -math.inf
        self.previous_v_ref = 0.0
        self.previous_sample_time: float | None = None
        self.alpha_hat_deg = 0.0

    def __call__(self, context: dict) -> float:
        sim = context["sim"]
        now = float(context["time_s"])
        if now - self.last_sample_time >= 0.01 - 1e-9:
            raw = sim.imu_raw_log_fields()
            estimate = context["estimate"]
            v_ref = float(context["reference_state"][1])
            a_ref = 0.0 if self.previous_sample_time is None else (
                v_ref - self.previous_v_ref
            ) / max(now - self.previous_sample_time, 1e-9)
            dofs = [int(sim.model.jnt_dofadr[sim.model.joint(name).id]) for name in
                    ("left_wheel_hinge", "right_wheel_hinge")]
            wheel = self.wheel_radius_m * (
                np.asarray([sim.data.qvel[dofs[0]], sim.data.qvel[dofs[1]]])
                + float(sim.longitudinal_state(context["nominal_theta_eq_rad"])[3])
            )
            last_q = self.adapter.rows[-1] if self.adapter.rows else {}
            acc = raw["imu_accelerometer_noisy_raw_m_s2"]
            gyro = raw["imu_gyro_noisy_raw_rad_s"]
            requested = np.asarray(sim.data.ctrl[:2], dtype=float)
            actual = np.asarray(sim.data.actuator_force[:2], dtype=float)
            self.rows.append(np.asarray([
                *acc, *gyro, *wheel, *requested, *actual,
                estimate.velocity_hat_m_s, estimate.theta_hat_rad,
                estimate.theta_dot_hat_rad_s, v_ref, a_ref,
                last_q.get("q_filter_estimate_nm", 0.0),
                last_q.get("matched_residual_fraction", 0.0),
                last_q.get("scaled_innovation_rms", 0.0),
            ], dtype=np.float32))
            self.rows = self.rows[-int(self.model["window_samples"]):]
            self.last_sample_time = now
            self.previous_v_ref = v_ref
            self.previous_sample_time = now
            if len(self.rows) == int(self.model["window_samples"]):
                x = np.asarray(self.rows, dtype=np.float32).T
                normalized = normalize_window(x, self.model["normalization"])
                stats = statistical_features(normalized)
                self.alpha_hat_deg = float(
                    np.asarray(self.model["alpha_coef"]) @ stats
                    + float(self.model["alpha_intercept"])
                )
                self.alpha_hat_deg = float(np.clip(self.alpha_hat_deg, -18.0, 18.0))
        return self.alpha_hat_deg


def closed_loop_ablation(config: dict) -> None:
    import run_stage3b_yaw_control as stage3b
    import run_stage4a_slope_robustness as stage4a

    metrics = load_json(METRICS_PATH)
    if metrics["decision"]["selected"] != "simple":
        raise RuntimeError("closed-loop pilot requires the selected simple estimator")
    model = load_json(SELECTED_MODEL_PATH)
    common_base = list(stage3b.load_common())
    common_base[3] = copy.deepcopy(common_base[3])
    common_base[3]["history_frequency_hz"] = float(config["sample_frequency_hz"])
    reduced = common_base[7]
    nominal = float(reduced["parameters"]["theta_eq_rad"])
    validation = []
    stage4a_results = load_json(MODEL_DIR / "stage4" / "results" / "stage4a_slope_robustness_results.json")
    for run in stage4a_results["runs"]:
        if run["Q_state"] != "OFF":
            continue
        angle = float(run["angle_deg"])
        observed = float(run["summary"]["pitch"]["GT_world_slope_deg"]["mean"])
        analytic = math.degrees(analytic_theta_eq(math.radians(angle), reduced))
        validation.append({"alpha_deg": angle, "analytic_theta_eq_deg": analytic,
                           "stage4a_mean_pitch_deg": observed,
                           "error_deg": analytic - observed})
    arms = {
        "A_flat_Q_OFF": ("flat", False),
        "B_oracle_Q_OFF": ("oracle", False),
        "C_estimated_Q_OFF": ("estimated", False),
        "D_estimated_Q_ON": ("estimated", True),
    }
    rows = []
    for angle in config["closed_loop_angles_deg"]:
        base_spec = {
            "episode_id": f"ablation_{slug(angle)}", "payload": "empty",
            "alpha_deg": angle, "friction": 1.0, "terrain": "smooth",
            "terrain_parameters": None,
        }
        world = build_world(base_spec, config)
        for arm, (reference_mode, q_enabled) in arms.items():
            print(f"ABLATE {angle:+g} {arm}", flush=True)
            common = list(common_base)
            common[3] = copy.deepcopy(common_base[3])
            common[3]["imu_rng_seed"] = int(config["seed"]) + int(angle * 10) + 5000
            common = tuple(common)
            adapter = stage4a.make_q_adapter(common, actuator_enabled=q_enabled)
            holder = {}

            def setup(sim):
                holder["recorder"] = stage4a.SlopeGroundTruthRecorder(
                    sim, angle, config, nominal
                )
                if reference_mode == "estimated":
                    holder["estimator"] = RuntimeSimpleSlopeEstimator(
                        model, adapter, float(reduced["parameters"]["wheel_radius_m"])
                    )
                return {"stage4b_ablation": arm}

            def physics_step(sim):
                holder["recorder"].physics_step(sim)

            def diagnostic(sim):
                result = holder["recorder"].snapshot(sim)
                result["alpha_hat_runtime_deg"] = (
                    holder["estimator"].alpha_hat_deg
                    if "estimator" in holder else math.degrees(result["alpha_GT_rad"])
                )
                return result

            def guard(sim):
                return holder["recorder"].boundary_guard(sim)

            def equilibrium(context):
                if reference_mode == "flat":
                    alpha_hat = 0.0
                elif reference_mode == "oracle":
                    alpha_hat = holder["recorder"].snapshot(context["sim"])["alpha_GT_rad"]
                    return analytic_theta_eq(alpha_hat, reduced)
                else:
                    alpha_hat = math.radians(holder["estimator"](context))
                return analytic_theta_eq(alpha_hat, reduced)

            scenario = {
                "name": f"stage4b_{arm}_{slug(angle)}",
                "duration_s": 14.0,
                "linear_velocity_schedule": [
                    {"time_s": 0.0, "command": 0.0},
                    {"time_s": 1.0, "command": 0.4},
                ],
                "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
            }
            manifest = common[0]
            run = stage3b.run_case(
                scenario, float(manifest["yaw"]["K_psi_nm_per_rad"]),
                float(manifest["yaw"]["K_r_nm_per_rad_s"]), yaw_enabled=True,
                motor_mismatch_enabled=False, common=common, keep_history=True,
                payload_mode="empty", common_mode_augmentation=adapter,
                physics_step_callback=physics_step, simulation_setup_callback=setup,
                model_path_override=world, history_diagnostic_callback=diagnostic,
                termination_guard=guard,
                equilibrium_reference_callback=equilibrium,
            )
            history = run["history_50hz"]
            steady = [row for row in history if row["terrain_segment_GT"] == "constant_grade"]
            if steady:
                entry = steady[0]["t"]
                steady = [row for row in steady if row["t"] >= entry + 1.0]
            if not steady:
                steady = history
            pitch_error = np.asarray([
                row["pitch_GT_world_rad"] - row["theta_eq_used_rad"] for row in steady
            ])
            pitch_rate = np.asarray([row["theta_dot_hat_rad_s"] for row in steady])
            velocity_error = np.asarray([
                row["v_GT_along_track_m_s"] - row["v_ref_m_s"] for row in steady
            ])
            position_error = np.asarray([row["p_GT_m"] - row["p_ref_m"] for row in steady])
            wheels = np.asarray([[row["actual_left_nm"], row["actual_right_nm"]]
                                 for row in steady])
            rows.append({
                "angle_deg": angle, "arm": arm,
                "theta_minus_theta_eq_rms_deg": math.degrees(float(np.sqrt(np.mean(pitch_error**2)))),
                "pitch_rate_rms_rad_s": float(np.sqrt(np.mean(pitch_rate**2))),
                "velocity_tracking_rmse_m_s": float(np.sqrt(np.mean(velocity_error**2))),
                "position_error_rms_m": float(np.sqrt(np.mean(position_error**2))),
                "u_LQR_rms_nm": float(np.sqrt(np.mean(np.asarray([row["u_base_nm"] for row in steady])**2))),
                "u_Q_rms_nm": float(np.sqrt(np.mean(np.asarray([row["u_Q_used_nm"] for row in steady])**2))),
                "u_total_rms_nm": float(np.sqrt(np.mean(np.asarray([row["u_sum_nm"] for row in steady])**2))),
                "saturation_fraction": float(np.mean(np.any(np.abs(wheels) >= 0.63 - 1e-12, axis=1))),
                "fell": bool(run["longitudinal"]["fell"] or holder["recorder"].pitch_instability),
                "chassis_contact": bool(holder["recorder"].chassis_terrain_contact),
                "boundary_termination": run["simulation_termination"]["reason"] == "terrain_boundary_guard",
                "alpha_hat_mean_deg": float(np.mean([row["alpha_hat_runtime_deg"] for row in steady])),
            })
    c = [row for row in rows if row["arm"] == "C_estimated_Q_OFF"]
    d = [row for row in rows if row["arm"] == "D_estimated_Q_ON"]
    c_velocity = float(np.mean([row["velocity_tracking_rmse_m_s"] for row in c]))
    d_velocity = float(np.mean([row["velocity_tracking_rmse_m_s"] for row in d]))
    c_pitch_rate = float(np.mean([row["pitch_rate_rms_rad_s"] for row in c]))
    d_pitch_rate = float(np.mean([row["pitch_rate_rms_rad_s"] for row in d]))
    independent_value = d_velocity <= 0.90 * c_velocity and d_pitch_rate <= 0.90 * c_pitch_rate
    result = {
        "analytic_theta_eq": {
            "formula": "theta_flat + asin(r*(m_body+2*m_wheel)*sin(alpha)/(m_body*l_com))",
            "stage4a_validation": validation,
            "mae_deg": float(np.mean(np.abs([item["error_deg"] for item in validation]))),
        },
        "rows": rows,
        "Q_estimated_reference_comparison": {
            "C_mean_velocity_rmse_m_s": c_velocity,
            "D_mean_velocity_rmse_m_s": d_velocity,
            "C_mean_pitch_rate_rms_rad_s": c_pitch_rate,
            "D_mean_pitch_rate_rms_rad_s": d_pitch_rate,
            "requires_both_at_least_10_percent_better": True,
            "independent_value": independent_value,
            "actuator_decision": "KEEP" if independent_value else "PRODUCTION_OFF",
        },
    }
    write_json(ABLATION_PATH, result)
    with SUMMARY_CSV_PATH.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"WROTE {ABLATION_PATH}", flush=True)


def closed_loop_ablation_revision(config: dict) -> None:
    """Honor the Stage4B-R downstream gate and record the terminal decision."""

    del config
    metrics = load_json(REVISION_METRICS_PATH)
    decision = metrics["decision"]["slope"]
    if decision != "PERMANENT_REJECT":
        raise RuntimeError(
            "Stage4B-R closed-loop execution is only implemented after exporting "
            "a qualified revision estimator"
        )
    baseline_ablation = load_json(ABLATION_PATH)
    result = {
        "stage": "Stage4B-R downstream gate",
        "status": "SKIPPED_BY_DECLARED_STOP_RULE",
        "slope_estimator_decision": decision,
        "reason": (
            "Neither 2.4 s Ridge nor Ridge plus bounded residual passed the fixed "
            "online/OOD/counterfactual slope gates; C/D are prohibited."
        ),
        "analytic_theta_eq": baseline_ablation["analytic_theta_eq"],
        "closed_loop_ablation": {
            "executed": False,
            "arms": [
                "A old flat equilibrium / Q OFF",
                "B oracle alpha -> theta_eq / Q OFF",
                "C estimated alpha -> theta_eq / Q OFF",
                "D estimated alpha -> theta_eq / Q ON",
            ],
            "policy": "Stage4B-R section 20: run downstream only after slope Gate passes",
        },
        "Q_decision": {
            "observer": "KEEP_DIAGNOSTIC_LOGGING_FEATURE",
            "actuator": "PRODUCTION_OFF",
            "parameters_retuned": False,
        },
    }
    write_json(REVISION_ABLATION_PATH, result)
    print(f"WROTE {REVISION_ABLATION_PATH}", flush=True)


def render_revision_report(config: dict) -> None:
    manifest = load_json(REVISION_MANIFEST_PATH)
    metrics = load_json(REVISION_METRICS_PATH)
    ablation = load_json(REVISION_ABLATION_PATH)
    old = metrics["old_baselines"]
    ridge = metrics["long_ridge"]["evaluation"]["test"]
    hybrid = metrics["bounded_residual"]["evaluation"]["test"]
    event = metrics["short_event_cnn"]["evaluation"]["test"]
    usable_train = {
        str(alpha): sum(
            episode["split"] == "train"
            and episode["alpha_deg"] == alpha
            and episode["status"] == "usable"
            for episode in manifest["episodes"]
        ) for alpha in config["train_slopes_deg"]
    }
    status_counts = {
        status: sum(episode["status"] == status for episode in manifest["episodes"])
        for status in sorted({episode["status"] for episode in manifest["episodes"]})
    }
    cf4_status = [
        {"episode_id": episode["episode_id"], "status": episode["status"],
         "fell": episode["fell"]}
        for episode in manifest["episodes"]
        if episode.get("evaluation_group") == "cf_low_mu_slip"
    ]
    ridge_cf5_errors = [
        abs(row["alpha_hat_deg"] - row["alpha_gt_deg"])
        for row in ridge["counterfactual"]["cf_unseen_terrain"]["episodes"]
    ]
    hybrid_cf5_errors = [
        abs(row["alpha_hat_deg"] - row["alpha_gt_deg"])
        for row in hybrid["counterfactual"]["cf_unseen_terrain"]["episodes"]
    ]
    coupling_realization = []
    for episode in manifest["episodes"]:
        if episode.get("evaluation_group") != "cf_coupling":
            continue
        path = (REVISION_MANIFEST_PATH.parent / episode["data_file"]).resolve()
        with np.load(path, allow_pickle=False) as data:
            constant = np.asarray(data["constant_grade"], dtype=bool)
            coupling_realization.append({
                "coupling_class": episode["coupling_class"],
                "slip_realized": bool(np.max(data["slip_gt"][constant])) if np.any(constant) else None,
                "rough_realized": bool(np.max(data["rough_gt"][constant])) if np.any(constant) else None,
                "fell": episode["fell"],
            })
    coupling_lookup = {item["coupling_class"]: item for item in coupling_realization}
    coupling_rows = []
    for name in (
        "clean", "payload_only", "slip_only", "rough_only",
        "payload_slip", "payload_rough", "slip_rough", "payload_slip_rough",
    ):
        r = ridge["coupling_classes"][name]
        h = hybrid["coupling_classes"][name]
        actual = coupling_lookup[name]
        coupling_rows.append(
            f"| {name} | {actual['slip_realized']} | {actual['rough_realized']} | "
            f"{r['mae_deg']:.3f}° / {r['p95_abs_error_deg']:.3f}° | "
            f"{h['mae_deg']:.3f}° / {h['p95_abs_error_deg']:.3f}° |"
        )
    old_simple = old["simple"]["metrics"]["test"]
    old_cnn = old["cnn"]["metrics"]["test"]
    report = f"""# Stage 4B-R — Final Proprioceptive Estimator Qualification

## Scope and implementation correction

This is a patch to Stage4B, not a new dataset/trainer stack. It reuses the same parameterized runner, NPZ episode format, episode-exclusive split, lazy sliding-window dataset, train-only normalization, statistics extractor, Ridge baseline, Tiny Conv1D, evaluator and counterfactual definitions. The previously audited [BorealTC](https://github.com/norlab-ulaval/BorealTC), [T_DEEP](https://github.com/Ph0bi0/T_DEEP), [UMich slip_detection_DOB](https://github.com/UMich-CURLY/slip_detection_DOB) and [inekf_wheeled](https://github.com/XihangYU630/inekf_wheeled) decisions remain unchanged; no new literature search or vendoring was performed.

The audit found one material Stage4B physics bug: terrain friction changed while both wheel collision geoms remained at 1.0. Stage4B-R uses `wheel_and_terrain_v2`, setting the same friction on terrain and wheel collisions. Revision episodes use new IDs, so historical Stage4B raw data and reported baselines remain untouched. This correction makes low-friction failures real and means the old and revised absolute scores are informative engineering comparisons, not a pure window-length ablation on identical physics.

## Dataset and label integrity

- Generated 157 independent revision episodes: 108 train, 9 val, 40 test. Nominal train coverage is exactly 12 episodes per known slope; usable train episodes after retaining physical failures are `{usable_train}`.
- Status counts: `{status_counts}`; falls retained: {sum(episode['fell'] for episode in manifest['episodes'])}. No failure was silently discarded from the manifest.
- Long/short windows: {metrics['data_coverage']['long_window_counts']} / {metrics['data_coverage']['short_window_counts']}.
- Split-disjoint audit: **{'PASS' if metrics['integrity_audit']['episode_split_disjoint'] else 'FAIL'}**. Numeric OOD slopes remain absent from train: **{'PASS' if metrics['integrity_audit']['numeric_ood_slopes_absent_from_train'] else 'FAIL'}**.
- GT is per-timestamp local terrain grade: flat samples are 0°, transition/grade samples use their actual local geom angle. The 2.4 s target is the median local grade over the final 0.2 s; earlier window context may contain flat/entry samples. GT/world pose/time/episode identity are not features. Normalization is train-only.

## Final model comparison

| Model | Physics/data | Online alpha MAE | p95 | max | >5° windows | Decision |
|---|---|---:|---:|---:|---:|---|
| OLD 0.8 s Statistics + Ridge | historical Stage4B | {old_simple['slope']['mae_deg']:.3f}° | {old_simple['slope']['p95_abs_error_deg']:.3f}° | not recorded | not recorded | historical reject |
| OLD raw Tiny Conv1D | historical Stage4B | {old_cnn['slope']['mae_deg']:.3f}° | {old_cnn['slope']['p95_abs_error_deg']:.3f}° | not recorded | not recorded | reject |
| NEW 2.4 s Statistics + Ridge | expanded, corrected friction | {ridge['online_window']['mae_deg']:.3f}° | {ridge['online_window']['p95_abs_error_deg']:.3f}° | {ridge['online_window']['max_abs_error_deg']:.3f}° | {ridge['online_window']['count_abs_error_gt_5deg']} | fail |
| NEW Ridge + bounded residual | expanded, corrected friction | {hybrid['online_window']['mae_deg']:.3f}° | {hybrid['online_window']['p95_abs_error_deg']:.3f}° | {hybrid['online_window']['max_abs_error_deg']:.3f}° | {hybrid['online_window']['count_abs_error_gt_5deg']} | fail |

Episode aggregation is diagnostic only, not the Gate: Ridge MAE/p95 {ridge['episode_aggregated']['mae_deg']:.3f}°/{ridge['episode_aggregated']['p95_abs_error_deg']:.3f}°; hybrid {hybrid['episode_aggregated']['mae_deg']:.3f}°/{hybrid['episode_aggregated']['p95_abs_error_deg']:.3f}°. It also fails.

The 2.4 s Ridge did **not** improve qualification. Every fixed Gate check failed: overall, numeric OOD, combination OOD, coupled-disturbance OOD, friction drift and payload drift. Ridge friction/payload drift is {ridge['counterfactual']['cf_friction']['alpha_prediction_drift_deg']:.3f}°/{ridge['counterfactual']['cf_payload']['alpha_prediction_drift_deg']:.3f}°; hybrid is {hybrid['counterfactual']['cf_friction']['alpha_prediction_drift_deg']:.3f}°/{hybrid['counterfactual']['cf_payload']['alpha_prediction_drift_deg']:.3f}°. Fixed-nuisance alpha remains monotonic, but monotonicity alone is insufficient.

CF4's two original same-low-friction arms are retained as `{cf4_status}`: corrected friction made both fall before a qualifying constant-grade window, so no fabricated steady slip/no-slip comparison is reported. On CF5 unseen rough/bump geometry, episode-level absolute error reaches {max(ridge_cf5_errors):.3f}° for Ridge and {max(hybrid_cf5_errors):.3f}° for hybrid.

## Residual behavior and nuisance coupling

The event encoder was frozen and the 6,241-parameter MLP produced a fixed ±3° tanh-bounded correction; total learned parameters are {metrics['bounded_residual']['total_learned_parameter_count']:,}, with no joint fine-tune. It reduced overall p95 and the number of >5° windows, but only marginally changed MAE and increased maximum error from {ridge['online_window']['max_abs_error_deg']:.2f}° to {hybrid['online_window']['max_abs_error_deg']:.2f}°. It helps some slip/rough episodes and harms some payload combinations; it does not disentangle nuisances.

| CF6 intended class | Slip actually realized | Rough realized | Ridge MAE / p95 | Hybrid MAE / p95 |
|---|:---:|:---:|---:|---:|
{chr(10).join(coupling_rows)}

The intended smooth `slip_only` arm did not cross the physical 0.20 slip threshold at a stable constant grade; lowering friction further caused fall/no constant-grade window. Rough contact itself produced slip in the rough arms. This is reported as an observability/reachability limitation, not relabeled. Coupling failure becomes severe once low-friction reversal is combined with payload/rough; the all-nuisance hybrid MAE remains {hybrid['coupling_classes']['payload_slip_rough']['mae_deg']:.3f}°.

Catastrophic tails remain: hybrid has {hybrid['online_window']['count_abs_error_gt_2deg']} / {hybrid['online_window']['count_abs_error_gt_3deg']} / {hybrid['online_window']['count_abs_error_gt_5deg']} windows above 2° / 3° / 5°, with {hybrid['online_window']['max_abs_error_deg']:.2f}° maximum error. Therefore current proprioception is not sufficient for production slope sensing.

## Slip and rough heads

| Method | Slip precision / recall / F1 | Rough precision / recall / F1 |
|---|---:|---:|
| OLD cheap baseline | {old_simple['slip']['precision']:.3f}/{old_simple['slip']['recall']:.3f}/{old_simple['slip']['f1']:.3f} | {old_simple['rough']['precision']:.3f}/{old_simple['rough']['recall']:.3f}/{old_simple['rough']['f1']:.3f} |
| NEW short event CNN | {event['slip']['precision']:.3f}/{event['slip']['recall']:.3f}/{event['slip']['f1']:.3f} | {event['rough']['precision']:.3f}/{event['rough']['recall']:.3f}/{event['rough']['f1']:.3f} |

Slip recall {event['slip']['recall']:.3f} misses the unchanged 0.90 Gate: **DEFER/REJECT production slip detector**. The old cheap rough result remains better than the new CNN; keep its result/code only, with no controller connection absent a product need.

## Engineering answers and terminal decision

1. The old CNN failure was **not mainly data scarcity**. Independent train episodes increased from 27 to 108, yet corrected-friction OOD and nuisance coupling remain poor.
2. Long context did **not** make Ridge production-qualified; its online tail is substantially outside Gate.
3. The bounded residual mostly shifts predictions under slip/rough context but cannot remove payload/friction ambiguity and sometimes worsens the tail.
4. Clean and single payload/rough cases are less bad, but already fail the 1°/2.5° Gate; stable pure-slip could not be physically isolated.
5. Two/three-nuisance coupling, especially low friction with payload/rough, is the dominant collapse mode.
6. Yes, 5°-class catastrophic errors remain frequent.
7. No, current production-available proprioception has not demonstrated sufficient slope observability.
8. Final slope decision: **PERMANENT REJECT**. No more proprioceptive ML expansion; move slope sensing to RGB-D or dual-ToF geometry.
9. Q observer: **KEEP** for diagnostics/logging/possible future feature studies. Q actuator: **PRODUCTION OFF**; no cutoff/authority retuning and no new theta-aware benefit claim.
10. Freeze the existing Stage4 controller/Q production baseline and proceed to the visual/geometric sensing phase.

## Downstream gate

The analytic alpha→theta_eq mapping remains valid at {ablation['analytic_theta_eq']['mae_deg']:.3f}° Stage4A steady-pitch MAE, but estimator qualification failed. Per the declared rule, A/B/C/D closed-loop execution is **skipped**, C/D are not run, and no ML theta_eq model is introduced.
"""
    REVISION_REPORT_PATH.write_text(report, encoding="utf-8")
    print(f"WROTE {REVISION_REPORT_PATH}", flush=True)


def render_report(config: dict) -> None:
    manifest = load_json(MANIFEST_PATH)
    metrics = load_json(METRICS_PATH)
    decision = metrics["decision"]["selected"]
    if decision == "reject":
        reduced = load_json(MODEL_DIR / "reduced_twip.json")
        stage4a_results = load_json(
            MODEL_DIR / "stage4" / "results" / "stage4a_slope_robustness_results.json"
        )
        validation = []
        for run in stage4a_results["runs"]:
            if run["Q_state"] != "OFF":
                continue
            angle = float(run["angle_deg"])
            observed = float(run["summary"]["pitch"]["GT_world_slope_deg"]["mean"])
            analytic = math.degrees(analytic_theta_eq(math.radians(angle), reduced))
            validation.append({"alpha_deg": angle, "analytic_theta_eq_deg": analytic,
                               "stage4a_mean_pitch_deg": observed,
                               "error_deg": analytic - observed})
        ablation = {
            "status": "SKIPPED_BY_DECLARED_STOP_RULE",
            "reason": "neither simple nor tiny Conv1D alpha estimator passed held-out/OOD gate",
            "analytic_theta_eq": {
                "formula": "theta_flat + asin(r*(m_body+2*m_wheel)*sin(alpha)/(m_body*l_com))",
                "stage4a_validation": validation,
                "mae_deg": float(np.mean(np.abs([item["error_deg"] for item in validation]))),
            },
            "closed_loop_ablation": {
                "executed": False,
                "arms": ["A flat/Q OFF", "B oracle/Q OFF", "C estimated/Q OFF", "D estimated/Q ON"],
                "policy": "Section 14 permits alpha->theta_eq and the four-arm test only after alpha gate passes",
            },
            "Q_decision": {
                "observer": "KEEP",
                "actuator": "PRODUCTION_OFF",
                "basis": "no theta-aware evidence may be claimed; retain frozen Stage4A/Stage3 production decision",
            },
        }
        write_json(ABLATION_PATH, ablation)
    else:
        ablation = load_json(ABLATION_PATH)
    display_model = decision if decision != "reject" else "simple"
    selected_metrics = metrics[display_model]["metrics"]["test"]
    episodes = manifest["episodes"]
    split_counts = {split: sum(item["split"] == split for item in episodes)
                    for split in ("train", "val", "test")}
    episode_sets = {split: {item["episode_id"] for item in episodes if item["split"] == split}
                    for split in split_counts}
    leakage = not (
        episode_sets["train"] & episode_sets["val"]
        or episode_sets["train"] & episode_sets["test"]
        or episode_sets["val"] & episode_sets["test"]
    )
    cf = metrics[display_model]["counterfactual"]
    ablation_executed = bool(ablation.get("closed_loop_ablation", {}).get("executed", True))
    if ablation_executed:
        q = ablation["Q_estimated_reference_comparison"]
        q_paragraph = (
            f"Final C→D comparison (estimated alpha reference): velocity RMSE "
            f"{q['C_mean_velocity_rmse_m_s']:.4f} → {q['D_mean_velocity_rmse_m_s']:.4f} m/s; "
            f"pitch-rate RMS {q['C_mean_pitch_rate_rms_rad_s']:.4f} → "
            f"{q['D_mean_pitch_rate_rms_rad_s']:.4f} rad/s. Frozen Q actuator "
            f"independent-value gate: **{'PASS' if q['independent_value'] else 'FAIL'}**."
        )
        q_actuator = q["actuator_decision"]
        stage_recommendation = "freeze and proceed to Webots / Camera / Master Following"
    else:
        q_paragraph = (
            "The declared alpha Gate failed, so the A/B/C/D closed-loop experiment was "
            "**not executed**; running C/D would violate the request's Section 14 Stop Rule. "
            "No theta-aware Q benefit is claimed."
        )
        q_actuator = ablation["Q_decision"]["actuator"]
        stage_recommendation = (
            "freeze the existing controller/Q production baseline and move environment sensing "
            "to ToF or RGB-D/camera geometry; do not continue scaling the proprioceptive model"
        )
    report = f"""# Stage 4B — Proprioceptive Environment Pilot

## Upstream Borrow / Adapt / Reject

- **[BorealTC (MIT)](https://github.com/norlab-ulaval/BorealTC):** read `borealtc.py`, `utils/preprocessing.py`, `utils/models.py`, and README. Borrowed run/episode identity, aligned sequences, lazy `(episode,start,end)` sliding windows, train-only preprocessing, AdamW / ReduceLROnPlateau / early stopping. Adapted its IMU+wheel concept to raw 100 Hz CompanionBot production signals and a 0.8 s Conv1D input. Rejected random-window split, spectrogram Conv2D, LSTM and Mamba.
- **[T_DEEP / Vulpi](https://github.com/Ph0bi0/T_DEEP):** used only as the older proprioceptive terrain-CNN reference identified by BorealTC. Rejected porting its MATLAB/data/network stack.
- **[UMich slip_detection_DOB](https://github.com/UMich-CURLY/slip_detection_DOB):** read README and `slipEstimator_SlipModel`; borrowed wheel/body motion inconsistency as slip semantics. Adapted it to simulator-privileged wheel-surface versus chassis tangential speed GT with a predeclared 0.20 ratio. Rejected ROS, Husky and full DOB/InEKF.
- **[inekf_wheeled](https://github.com/XihangYU630/inekf_wheeled):** borrowed the IMU+encoder slip-observability vocabulary only; rejected the complete InEKF/ROS estimator because Stage3 estimation is frozen.

## Dataset integrity

- Generated {len(episodes)} episodes: train/val/test = {split_counts['train']}/{split_counts['val']}/{split_counts['test']}; usable windows = {metrics['window_counts']}.
- Episode-exclusive split audit: **{'PASS' if leakage else 'FAIL'}**. No episode or overlapping window crosses splits.
- Network inputs contain only noisy IMU, wheel/control, estimated state, command and frozen-Q observer features. Alpha/friction/payload/contact/world pose/time/episode id are absent from features and exist only in manifest/labels/audit.
- Normalization sample count {metrics['normalization']['sample_count']} was fit on the train split only. Raw episodes and checkpoints are under git-ignored `generated/`.
- Each train slope has all three friction, payload, terrain and speed levels via independent seeded permutations ({manifest['factorization_audit']['unique_train_full_factor_tuples']} unique full tuples); audit: **{'PASS' if manifest['factorization_audit']['all_three_levels_present_for_each_slope_and_factor'] else 'FAIL'}**. Numeric OOD uses unseen continuous alpha values {config['numeric_ood_slopes_deg']}; explicit combination OOD and counterfactual groups are test-only.

## Model results and decision

| Model | Test alpha MAE | p95 | Slip P/R/F1 | Rough P/R/F1 |
|---|---:|---:|---:|---:|
| Statistics + Ridge/Logistic | {metrics['simple']['metrics']['test']['slope']['mae_deg']:.3f}° | {metrics['simple']['metrics']['test']['slope']['p95_abs_error_deg']:.3f}° | {metrics['simple']['metrics']['test']['slip']['precision']:.3f}/{metrics['simple']['metrics']['test']['slip']['recall']:.3f}/{metrics['simple']['metrics']['test']['slip']['f1']:.3f} | {metrics['simple']['metrics']['test']['rough']['precision']:.3f}/{metrics['simple']['metrics']['test']['rough']['recall']:.3f}/{metrics['simple']['metrics']['test']['rough']['f1']:.3f} |
| Tiny raw Conv1D ({metrics['cnn']['parameter_count']} params) | {metrics['cnn']['metrics']['test']['slope']['mae_deg']:.3f}° | {metrics['cnn']['metrics']['test']['slope']['p95_abs_error_deg']:.3f}° | {metrics['cnn']['metrics']['test']['slip']['precision']:.3f}/{metrics['cnn']['metrics']['test']['slip']['recall']:.3f}/{metrics['cnn']['metrics']['test']['slip']['f1']:.3f} | {metrics['cnn']['metrics']['test']['rough']['precision']:.3f}/{metrics['cnn']['metrics']['test']['rough']['recall']:.3f}/{metrics['cnn']['metrics']['test']['rough']['f1']:.3f} |

Decision: **{decision.upper()}**. The selection rule deliberately prefers the simple model unless Conv1D improves alpha MAE by more than 0.20°. Conv1D checkpoint is {metrics['cnn']['checkpoint_bytes']} bytes, estimated FP32 parameter memory {metrics['cnn']['estimated_fp32_parameter_memory_bytes']} bytes, measured batch-one CPU latency {metrics['cnn']['inference_latency_cpu_ms']:.3f} ms.

Best cheap-baseline test slope: MAE {selected_metrics['slope']['mae_deg']:.3f}°, RMSE {selected_metrics['slope']['rmse_deg']:.3f}°, p95 {selected_metrics['slope']['p95_abs_error_deg']:.3f}°, bias {selected_metrics['slope']['bias_deg']:+.3f}°.

Best cheap-baseline counterfactual alpha drift: friction {cf['cf_friction'].get('alpha_prediction_drift_deg', float('nan')):.3f}°, payload {cf['cf_payload'].get('alpha_prediction_drift_deg', float('nan')):.3f}°. With friction/payload/terrain held fixed, unseen-alpha prediction is strictly monotonic: {cf['cf_alpha_monotonic'].get('strictly_monotonic')}. Low-mu no-slip versus bump-induced-slip and unseen terrain parameter outputs are preserved in `model_metrics.json`, including failures; no threshold/controller retuning was done.

## Alpha to equilibrium and closed loop

The analytic quasi-static mapping is `theta_flat + asin(r (m_body + 2 m_wheel) sin(alpha) / (m_body l_com))`. Against the four Stage4A Q-OFF steady mean pitches its MAE is {ablation['analytic_theta_eq']['mae_deg']:.3f}°.

{q_paragraph}

## Final decisions

- Proprioceptive environment estimator: **{'KEEP ' + decision if decision != 'reject' else 'REJECT; fallback is dual ToF or RGB-D ground geometry'}**.
- Q observer: **KEEP** as a diagnostic/physics-feature producer; it remained active in every generated episode.
- Q features: **KEEP for logging/future sensing baselines**, but there is no production learned estimator and no claim that Q features alone caused any result.
- Q actuator: **{q_actuator}**. Parameters were not retuned; when the alpha Gate fails this retains the prior production-OFF decision rather than inventing a new ablation claim.
- Stage4 lower layer: **{stage_recommendation}**.

Detailed per-episode OOD/counterfactual predictions and the exact ablation execution/skip record are in the JSON artifacts.
"""
    REPORT_PATH.write_text(report, encoding="utf-8")
    print(f"WROTE {REPORT_PATH}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=(
        "generate", "train", "ablate", "report",
        "generate-r", "train-r", "ablate-r", "report-r",
    ))
    args = parser.parse_args()
    config = load_json(CONFIG_PATH)
    if args.mode == "generate":
        generate(config)
    elif args.mode == "generate-r":
        generate(config, revision=True)
    elif args.mode == "train":
        train_and_evaluate(config)
    elif args.mode == "train-r":
        train_and_evaluate_revision(config)
    elif args.mode == "ablate":
        closed_loop_ablation(config)
    elif args.mode == "ablate-r":
        closed_loop_ablation_revision(config)
    elif args.mode == "report-r":
        render_revision_report(config)
    else:
        render_report(config)


if __name__ == "__main__":
    main()
