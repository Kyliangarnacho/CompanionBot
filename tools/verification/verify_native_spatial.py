"""Diagnostic same-input Python/C++ spatial parity and microbenchmark.

This does not simulate a robot, certify DDS latency, or substitute for physical
and ROS regressions. Existing saved physical sensorized inputs are read-only.
"""
from dataclasses import asdict
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from companionbot_contracts.paths import repository_root
from companionbot_control.estimation.spatial import PlanarOdometry, SpatialSample


def stats(values):
    array = np.asarray(values, dtype=float)
    return dict(count=len(values), mean_us=float(array.mean()/1e3),
        p50_us=float(np.percentile(array, 50)/1e3), p95_us=float(np.percentile(array, 95)/1e3),
        p99_us=float(np.percentile(array, 99)/1e3), max_us=float(array.max()/1e3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--executable", type=Path, default=repository_root()/"ros2_ws/build/companionbot_control/spatial_replay")
    parser.add_argument("--inputs", type=Path, nargs="*")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not args.executable.is_file():
        parser.error("native replay executable missing; build first")
    inputs = args.inputs or [repository_root()/"models/minisegway/spatial_slam_foundation/results/session_20261021/physical_initial"/name/"sensorized_state.jsonl"
                            for name in ("balance", "forward", "turn", "mixed", "pitch")]
    summary = dict(scope="WSL host pure passive odometry kernel; no Pi/STM32 or DDS latency claim",
        acceptance_pose_covariance_abs_tolerance=3e-12, scenarios=[], hardware_execution_ready=False)
    for source in inputs:
        name = source.parent.name if source.name == "sensorized_state.jsonl" else source.stem.replace("_input", "")
        raw = [json.loads(line) for line in source.read_text().splitlines()]
        samples = [SpatialSample(**{key: row.get("state", row)[key] for key in SpatialSample.__dataclass_fields__}) for row in raw]
        normalized = output/(name+"_input.jsonl")
        normalized.write_text(''.join(json.dumps(asdict(sample))+'\n' for sample in samples))
        replay = subprocess.run([str(args.executable), "--replay", str(normalized)], check=True, text=True, capture_output=True)
        (output/(name+"_cpp.jsonl")).write_text(replay.stdout)
        actual = [json.loads(line) for line in replay.stdout.splitlines()]
        odom, expected = PlanarOdometry(), []
        for sample in samples:
            estimate = odom.update(sample)
            expected.append(dict(x_m=estimate.x_m, y_m=estimate.y_m, yaw_rad=estimate.yaw_rad, covariance=estimate.covariance.tolist()))
        (output/(name+"_python.jsonl")).write_text(''.join(json.dumps(row)+'\n' for row in expected))
        maximum = 0.
        for a, b in zip(actual, expected, strict=True):
            left = [a["x_m"], a["y_m"], a["yaw_rad"], *np.asarray(a["covariance"]).ravel()]
            right = [b["x_m"], b["y_m"], b["yaw_rad"], *np.asarray(b["covariance"]).ravel()]
            maximum = max(maximum, float(np.max(np.abs(np.asarray(left)-right))))
        if maximum > 3e-12:
            raise AssertionError((name, maximum))
        # Timers have matching placement. Parsing/JSON output is outside scope.
        wall, cpu = [], []
        for repeat in range(-2, 40):
            odom = PlanarOdometry()
            for sample in samples:
                start = time.perf_counter_ns()
                thread_start = time.thread_time_ns()
                odom.update(sample)
                thread_end = time.thread_time_ns()
                end = time.perf_counter_ns()
                if repeat >= 0:
                    wall.append(end-start)
                    cpu.append(thread_end-thread_start)
        native = subprocess.run([str(args.executable), "--benchmark", str(normalized)], check=True, text=True, capture_output=True)
        native_rows = json.loads(native.stdout)
        (output/(name+"_native_benchmark_raw.json")).write_text(native.stdout)
        (output/(name+"_python_benchmark_raw.json")).write_text(json.dumps(dict(wall_ns=wall, thread_cpu_ns=cpu))+"\n")
        summary["scenarios"].append(dict(scenario=name, input_samples=len(samples),
            original_source=str(source.resolve()), original_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            max_pose_covariance_difference=maximum, python_wall=stats(wall), python_cpu=stats(cpu),
            cpp_wall=stats(native_rows["wall_ns"]), cpp_cpu=stats(native_rows["thread_cpu_ns"])))
    summary["passed"] = True
    summary["limitations"] = ["Pure update replay excludes ROS serialization/publication and node/process resources",
        "Timer overhead remains included; native sub-microsecond measurements include thread clock reads",
        "Python then C++ sequential microbenchmark may be affected by host load; full ROS fair-window benchmark remains necessary"]
    (output/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
