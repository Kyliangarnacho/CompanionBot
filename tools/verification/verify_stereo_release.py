"""Compare an installed stereo build against exact saved RGB/output fixtures.

This diagnostic loads no AI model and supplies no renderer depth, pose or GT to
the processor. Old metric accuracy/freshness failures remain failures. It does
not create or reissue an authorization certificate after a binary change.
"""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import sys
import time
import traceback

import cv2
import numpy as np

from companionbot_perception import _native_stereo as native
from companionbot_perception.runtime.stereo import algorithm_identity, make_native_config


FIELDS = {"stereo_depth_m": "depth_m", "disparity_px": "disparity_px",
          "stereo_valid": "valid_mask", "stereo_rejection": "rejection_mask",
          "lr_error_px": "lr_error_px"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quantiles(values):
    values = np.asarray(values, dtype=float)
    return dict(count=int(values.size), p50_s=float(np.percentile(values, 50)),
                p95_s=float(np.percentile(values, 95)), p99_s=float(np.percentile(values, 99)),
                maximum_s=float(np.max(values)))


def run(args):
    source, output = args.source_dir.resolve(), args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = dict(passed=False, completed=False, rows=[], source=str(source),
                  source_sha256=sha(__file__), python=sys.executable, hardware_execution_ready=False,
                  motion_authorized=False, certificate_reissued=False,
                  scope="same saved RGB and matcher outputs; original Master decisions retained, no new AI or GT processing",
                  timing_comparison="observational only; old timings included concurrent rendering/AI and O0 wrapper; new fresh process isolates Release matcher; not fair speedup evidence")
    try:
        camera_path, protocol_path = source/"camera.json", source/"protocol.json"
        camera = json.loads(camera_path.read_text())
        protocol = json.loads(protocol_path.read_text())
        original = json.loads((source/"summary.json").read_text())
        rows = json.loads((source/"records.json").read_text())
        if len(rows) != 48:
            raise ValueError("requires_exact_original48saved_pose_inputs")
        if original["camera_config_sha256"] != sha(camera_path) or original["protocol_sha256"] != sha(protocol_path):
            raise ValueError("saved_grid_source_identity_mismatch")
        identity = algorithm_identity()
        if identity["source_sha256"] != original["algorithm_identity"]["source_sha256"]:
            raise ValueError("this_build_only_parity_requires_unchanged_native_source")
        processor = native.StereoDepthProcessor(make_native_config(camera, protocol, calibration_id=sha(camera_path)))
        report.update(old_algorithm_identity=original["algorithm_identity"], old_native_module=original["native_stereo"],
                      new_algorithm_identity=identity, camera_sha256=sha(camera_path), protocol_sha256=sha(protocol_path),
                      original_heldout_failure=original["heldout"], original_capture_age=original["measurement_capture_age"])
        for row in rows:
            directory = source/f"{row['index']:03d}_{row['split']}"
            last = row["source_frames"][-1]
            left_path, right_path = directory/"left.png", directory/"right.png"
            left, right = cv2.imread(str(left_path)), cv2.imread(str(right_path))
            if left is None or right is None:
                raise FileNotFoundError("saved_real_stereo_RGB_pair_missing")
            left.setflags(write=False)
            right.setflags(write=False)
            stamp = native.StereoFrameStamp("stereo-fixed-grid", last["sequence"], last["capture_host_s"],
                                           sha(camera_path), "host_perf_counter")
            started = time.perf_counter()
            result = processor.process(left, right, stamp, stamp)
            wrapper_wall = time.perf_counter()-started
            comparisons = {}
            # npz also contains posthoc GT arrays. Only this strict matcher-
            # output allowlist is accessed; none of those GT arrays is loaded.
            archive_path = directory/"depth_and_posthoc.npz"
            with np.load(archive_path) as archive:
                for saved, field in FIELDS.items():
                    expected, actual = archive[saved], getattr(result, field)
                    exact = bool(expected.dtype == actual.dtype and expected.shape == actual.shape and
                                 np.array_equal(expected, actual, equal_nan=True))
                    byte_exact = bool(expected.shape == actual.shape and expected.dtype == actual.dtype and
                                      expected.tobytes() == actual.tobytes())
                    comparisons[saved] = dict(exact_equal_including_nan=exact, byte_exact=byte_exact,
                        expected_sha256=hashlib.sha256(expected.tobytes()).hexdigest(),
                        actual_sha256=hashlib.sha256(actual.tobytes()).hexdigest())
            median_equal = None
            if row.get("roi_xyxy_px") is not None:
                l, t, r, b = row["roi_xyxy_px"]
                depth = result.depth_m[t:b, l:r]
                valid = np.isfinite(depth) & (depth > 0)
                median = float(np.median(depth[valid])) if np.count_nonzero(valid) >= 10 else None
                median_equal = median == row["stereo_depth_m"]
            report["rows"].append(dict(index=row["index"], split=row["split"], sequence=last["sequence"],
                original_source_host_s=last["capture_host_s"], left_sha256=sha(left_path), right_sha256=sha(right_path),
                saved_archive_sha256=sha(archive_path), comparisons=comparisons, original_master_roi_median_equal=median_equal,
                native_diagnostics=dict(result.diagnostics), measured_wrapper_wall_s=wrapper_wall,
                old_concurrent_o0_diagnostics=last["stereo_diagnostics"]))
        report.update(completed=True, samples=len(report["rows"]), compared_arrays=len(FIELDS)*len(report["rows"]),
            passed=all(check["exact_equal_including_nan"] and check["byte_exact"]
                       for row in report["rows"] for check in row["comparisons"].values()) and all(
                       row["original_master_roi_median_equal"] is not False for row in report["rows"]),
            release_native_wall=quantiles([row["native_diagnostics"]["binding_total_wall_s"] for row in report["rows"]]),
            old_recorded_concurrent_o0_wall=quantiles([row["old_concurrent_o0_diagnostics"]["binding_total_wall_s"] for row in report["rows"]]),
            unchanged_original_metric_accuracy_passed=original["heldout"]["accuracy_passed"],
            unchanged_original_coverage_passed=original["heldout"]["coverage_passed"],
            peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
    except BaseException as error:
        report.update(error=repr(error), traceback=traceback.format_exc())
        raise
    finally:
        (output/"summary.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(dict(passed=report["passed"], samples=report["samples"], compared_arrays=report["compared_arrays"],
                          accuracy_passed=report["unchanged_original_metric_accuracy_passed"])), flush=True)
    if not report["passed"]:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
