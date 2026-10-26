"""Finite official MuJoCo stereo-render ablation; diagnostic, no control API.

Compare two scene updates, one scene plus camera-only update, and official
side-by-side rendering on identical integration snapshots and RGB profiles.
This does not change the production renderer or certify metric depth accuracy.
"""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import sys
import threading
import time
import traceback

import mujoco
from mujoco import gl_context
import numpy as np
from scipy.spatial.transform import Rotation

from companionbot_contracts.paths import repository_root
from companionbot_contracts.spatial_geometry import load_spatial_geometry
from companionbot_simulation.core.stereo_companion_scene import load_stereo_camera, StereoRGBRenderer
from companionbot_simulation.core.visual_companion_scene import VisualCompanionActor


ROOT = repository_root()
CAMERA = ROOT / "models/minisegway/native_runtime/config/stereo_camera.json"
ASSET = ROOT / ".venv/assets/visual_companion_gazebo/derived"
SPEC = int(mujoco.mjtState.mjSTATE_INTEGRATION)
VARIANTS = ("two_scene_updates", "one_scene_camera_update", "one_scene_side_by_side")
SCALAR_FIELDS = ("frustum_center", "frustum_width", "frustum_bottom", "frustum_top",
                 "frustum_near", "frustum_far", "orthographic")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def state(model, data):
    value = np.empty(mujoco.mj_stateSize(model, SPEC), dtype=np.float64)
    mujoco.mj_getState(model, data, value, SPEC)
    return value


def set_camera(destination, source):
    for field in ("pos", "forward", "up"):
        getattr(destination, field)[:] = getattr(source, field)
    for field in SCALAR_FIELDS:
        setattr(destination, field, getattr(source, field))


def camera_record(camera):
    return {**{field: getattr(camera, field).copy().tolist() for field in ("pos", "forward", "up")},
            **{field: getattr(camera, field) for field in SCALAR_FIELDS}}


def cpu(usage):
    return usage.ru_utime + usage.ru_stime


def quantiles(values):
    values = np.asarray(values, dtype=float)
    return dict(count=int(values.size), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), p99=float(np.percentile(values, 99)),
                maximum=float(np.max(values)), mean=float(np.mean(values)))


class DiagnosticRenderer:
    """One thread owns standard MuJoCo GL context/scene; no custom GL code."""

    def __init__(self, model, camera):
        self.model, self.camera = model, camera
        self.width, self.height = camera["width"], camera["height"]
        self.thread = threading.get_native_id()
        self.gl = gl_context.GLContext(2 * self.width, self.height)
        self.gl.make_current()
        self.context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.context)
        self.scene = mujoco.MjvScene(model=model, maxgeom=10000)
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3:6] = 0
        self.option.skingroup[2] = 1
        self.cameras = []
        for name in (camera["name"], camera["right_camera"]["name"]):
            selection = mujoco.MjvCamera()
            selection.type = mujoco.mjtCamera.mjCAMERA_FIXED
            selection.fixedcamid = model.camera(name).id
            self.cameras.append(selection)
        self.mono_rect = mujoco.MjrRect(0, 0, self.width, self.height)
        self.stereo_rect = mujoco.MjrRect(0, 0, 2 * self.width, self.height)
        self.mono_rgb = np.empty((self.height, self.width, 3), np.uint8)
        self.stereo_rgb = np.empty((self.height, 2 * self.width, 3), np.uint8)

    def flags(self):
        for name, flag in (("shadows", mujoco.mjtRndFlag.mjRND_SHADOW),
                           ("reflections", mujoco.mjtRndFlag.mjRND_REFLECTION)):
            if name in self.camera["render_flags"]:
                self.scene.flags[flag] = self.camera["render_flags"][name]

    def full_update(self, data, camera):
        mujoco.mjv_updateScene(self.model, data, self.option, None, camera,
                               mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        self.flags()

    def mono(self):
        mujoco.mjr_render(self.mono_rect, self.scene, self.context)
        mujoco.mjr_readPixels(self.mono_rgb, None, self.mono_rect, self.context)
        return self.mono_rgb[::-1, :, ::-1].copy()

    def render(self, data, variant):
        if threading.get_native_id() != self.thread:
            raise RuntimeError("diagnostic_gl_owner_mismatch")
        started = time.perf_counter()
        usage = resource.getrusage(resource.RUSAGE_SELF)
        thread_started = time.thread_time()
        self.scene.stereo = mujoco.mjtStereo.mjSTEREO_NONE
        self.full_update(data, self.cameras[0])
        left_updated = time.perf_counter()
        left_camera = mujoco.mjv_averageCamera(self.scene.camera[0], self.scene.camera[1])
        if variant == "one_scene_side_by_side":
            mujoco.mjv_updateCamera(self.model, data, self.cameras[1], self.scene)
            right_camera = mujoco.mjv_averageCamera(self.scene.camera[0], self.scene.camera[1])
            set_camera(self.scene.camera[0], left_camera)
            set_camera(self.scene.camera[1], right_camera)
            self.scene.stereo = mujoco.mjtStereo.mjSTEREO_SIDEBYSIDE
            mujoco.mjv_makeLights(self.model, data, self.scene)
            right_updated = time.perf_counter()
            mujoco.mjr_render(self.stereo_rect, self.scene, self.context)
            mujoco.mjr_readPixels(self.stereo_rgb, None, self.stereo_rect, self.context)
            left = self.stereo_rgb[::-1, :self.width, ::-1].copy()
            left_ready = time.perf_counter()
            right = self.stereo_rgb[::-1, self.width:, ::-1].copy()
        else:
            left = self.mono()
            left_ready = time.perf_counter()
            if variant == "two_scene_updates":
                self.full_update(data, self.cameras[1])
            elif variant == "one_scene_camera_update":
                mujoco.mjv_updateCamera(self.model, data, self.cameras[1], self.scene)
                mujoco.mjv_makeLights(self.model, data, self.scene)
            else:
                raise ValueError("unknown_diagnostic_render_variant")
            right_updated = time.perf_counter()
            right_camera = mujoco.mjv_averageCamera(self.scene.camera[0], self.scene.camera[1])
            right = self.mono()
        completed = time.perf_counter()
        thread_completed = time.thread_time()
        after_usage = resource.getrusage(resource.RUSAGE_SELF)
        left.setflags(write=False)
        right.setflags(write=False)
        return left, right, dict(
            pair_wall_s=completed-started,
            left_scene_update_s=left_updated-started,
            left_ready_s=left_ready-started,
            right_update_s=(right_updated-left_updated if variant == "one_scene_side_by_side"
                            else right_updated-left_ready),
            last_render_and_copy_s=completed-right_updated,
            process_cpu_s=cpu(after_usage)-cpu(usage), thread_cpu_s=thread_completed-thread_started,
            voluntary_context_switches=after_usage.ru_nvcsw-usage.ru_nvcsw,
            involuntary_context_switches=after_usage.ru_nivcsw-usage.ru_nivcsw,
            left_gl_camera=camera_record(left_camera), right_gl_camera=camera_record(right_camera),
        )

    def close(self):
        if threading.get_native_id() != self.thread:
            raise RuntimeError("diagnostic_gl_close_owner_mismatch")
        self.context.free()
        self.gl.free()


def snapshots(model):
    data = mujoco.MjData(model)
    actor = VisualCompanionActor(model, asset_dir=ASSET)
    geometry = load_spatial_geometry()
    cases = [dict(pitch=0., yaw=0., time=0., actor=[1.8, 0., 0.]),
             dict(pitch=.12, yaw=.2, time=.5, actor=[1.8, .2, 0.]),
             dict(pitch=-.10, yaw=-.3, time=1., actor=[1.5, -.3, 0.]),
             dict(pitch=.02, yaw=.1, time=1.5, actor=[2.1, 0., 0.])]
    jobs = []
    for case in cases:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        rotation = geometry.mj_to_ros.T @ Rotation.from_euler(
            "zy", [case["yaw"], case["pitch"]]).as_matrix() @ geometry.mj_to_ros
        q = Rotation.from_matrix(rotation).as_quat()
        data.qpos[3:7] = [q[3], *q[:3]]
        data.time = case["time"]
        actor.set_pose(data, case["time"]+.25, case["actor"], np.pi)
        mujoco.mj_forward(model, data)
        jobs.append(dict(case=case, state=state(model, data)))
    return jobs


def run(args):
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    protocol = dict(
        diagnostic_only=True, production_renderer_changed=False,
        order=list(VARIANTS)+list(reversed(VARIANTS)), paired_immutable_states=4,
        calls_per_block=args.calls_per_block, warmup_per_variant=args.warmup,
        camera_path=str(CAMERA.relative_to(ROOT)), camera_sha256=sha(CAMERA),
        source_model=str(args.model.resolve()), source_model_sha256=sha(args.model),
        unchanged_rgb_size=[640, 360], side_by_side_viewport=[1280, 360],
        geometry_source="exact declared original body-attached camera IDs; official averageCamera",
        integration_state_maximum_change=0, required_monocular_pixel_max_difference=0,
        side_by_side_pixel_difference_reported=True, no_ai_or_matching=True,
        timing_window="restore/forward excluded from render window and explicitly reported; all pixel copies included",
        freshness_gate_s=.2, minimum_desired_end_to_end_headroom_s=.02,
        microbenchmark_cannot_certify_end_to_end_freshness=True, hardware_execution_ready=False,
    )
    (output/"protocol.json").write_text(json.dumps(protocol, indent=2)+"\n")
    report = dict(passed=False, protocol=protocol, rows=[], parity=[], source_sha256=sha(__file__),
                  python=sys.executable, mujoco_version=mujoco.__version__, hardware_execution_ready=False)
    renderer = None
    try:
        camera = load_stereo_camera(CAMERA)
        if camera["width"] != 640 or camera["height"] != 360:
            raise ValueError("this_fixed_ablation_requires_original640x360_camera")
        model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
        original_buffer = [model.vis.global_.offwidth, model.vis.global_.offheight]
        model.vis.global_.offwidth = 2 * camera["width"]
        model.vis.global_.offheight = camera["height"]
        report["offscreen_buffer_only_change"] = dict(before=original_buffer, after=[1280, 360])
        jobs = snapshots(model)
        data = mujoco.MjData(model)
        production_references = []
        production = StereoRGBRenderer(model, camera_config=CAMERA)
        try:
            for job in jobs:
                mujoco.mj_setState(model, data, job["state"].copy(), SPEC)
                mujoco.mj_forward(model, data)
                production_references.append(production.render_bgr(data))
        finally:
            production.close()
        renderer = DiagnosticRenderer(model, camera)
        from OpenGL import GL
        report["opengl"] = {name: GL.glGetString(code).decode() for name, code in (
            ("vendor", GL.GL_VENDOR), ("renderer", GL.GL_RENDERER), ("version", GL.GL_VERSION))}
        references = []
        for index, job in enumerate(jobs):
            mujoco.mj_setState(model, data, job["state"].copy(), SPEC)
            mujoco.mj_forward(model, data)
            left, right, timing = renderer.render(data, VARIANTS[0])
            exact_production = (np.array_equal(left, production_references[index][0]) and
                                np.array_equal(right, production_references[index][1]))
            report["parity"].append(dict(index=index, baseline_exact_production_rgb=exact_production))
            references.append((left, right, timing))
            np.savez_compressed(output/f"reference_{index}.npz", left=left, right=right, state=job["state"])
        for variant in VARIANTS:
            for count in range(args.warmup):
                mujoco.mj_setState(model, data, jobs[count % len(jobs)]["state"].copy(), SPEC)
                mujoco.mj_forward(model, data)
                renderer.render(data, variant)
        for block, variant in enumerate(protocol["order"]):
            for count in range(args.calls_per_block):
                index = count % len(jobs)
                restore_started = time.perf_counter()
                mujoco.mj_setState(model, data, jobs[index]["state"].copy(), SPEC)
                mujoco.mj_forward(model, data)
                restored = time.perf_counter()
                before = state(model, data)
                left, right, timing = renderer.render(data, variant)
                unchanged = np.array_equal(before, state(model, data))
                reference_left, reference_right, reference_timing = references[index]
                errors = {}
                for name, actual, reference in (("left", left, reference_left), ("right", right, reference_right)):
                    delta = np.abs(actual.astype(np.int16)-reference.astype(np.int16))
                    errors[name] = dict(maximum=int(delta.max()), differing_channel_values=int(np.count_nonzero(delta)),
                                        fraction=float(np.mean(delta != 0)), mean=float(np.mean(delta)))
                gl_equal = (timing["left_gl_camera"] == reference_timing["left_gl_camera"] and
                            timing["right_gl_camera"] == reference_timing["right_gl_camera"])
                row = dict(block=block, variant=variant, index=index, timing=timing,
                           restore_and_forward_s=restored-restore_started,
                           integration_unchanged=bool(unchanged), gl_cameras_exact=gl_equal,
                           pixel_differences=errors)
                report["rows"].append(row)
                if count == 0:
                    np.savez_compressed(output/f"block_{block}_{variant}.npz", left=left, right=right)
            print(json.dumps(dict(block=block, variant=variant, completed=args.calls_per_block)), flush=True)
        summary = {}
        for variant in VARIANTS:
            rows = [row for row in report["rows"] if row["variant"] == variant]
            summary[variant] = {field: quantiles([row["timing"][field] for row in rows]) for field in (
                "pair_wall_s", "left_scene_update_s", "left_ready_s", "right_update_s",
                "last_render_and_copy_s", "process_cpu_s", "thread_cpu_s")}
            summary[variant].update(
                state_exact=all(row["integration_unchanged"] for row in rows),
                gl_cameras_exact=all(row["gl_cameras_exact"] for row in rows),
                left_pixel_maximum=max(row["pixel_differences"]["left"]["maximum"] for row in rows),
                right_pixel_maximum=max(row["pixel_differences"]["right"]["maximum"] for row in rows),
            )
        report["variants"] = summary
        report["passed"] = all(row["baseline_exact_production_rgb"] for row in report["parity"]) and all(
            row["integration_unchanged"] and row["gl_cameras_exact"]
                               for row in report["rows"]) and all(
            summary[variant]["left_pixel_maximum"] == summary[variant]["right_pixel_maximum"] == 0
            for variant in VARIANTS[:2])
        report["peak_rss_mib"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
    except BaseException as error:
        report.update(error=repr(error), traceback=traceback.format_exc())
        raise
    finally:
        if renderer is not None:
            renderer.close()
        (output/"summary.json").write_text(json.dumps(report, indent=2)+"\n")
    if not report["passed"]:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--calls-per-block", type=int, default=12)
    parser.add_argument("--warmup", type=int, default=4)
    args = parser.parse_args()
    if not 1 <= args.calls_per_block <= 100 or not 0 <= args.warmup <= 20:
        parser.error("bounded diagnostic requires1..100calls/block and0..20warmups/variant")
    run(args)


if __name__ == "__main__":
    main()
