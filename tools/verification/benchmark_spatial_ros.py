"""Diagnostic same-input Python/native spatial process comparison.

Replays saved sensorized samples through actual DDS at their original 50 Hz
cadence. The archived Python owner is a benchmark oracle only. Never launches
MuJoCo, Agent, hardware, commands, or two spatial output owners together.
"""
import argparse
from collections import defaultdict
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from companionbot_interfaces.msg import BalanceState
from companionbot_contracts.paths import repository_root
from companionbot_contracts.spatial_geometry import load_spatial_geometry
from companionbot_contracts.stamps import source_stamp
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


def stamp(t):
    ns = round(t*1e9)
    return Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)


def ns(value):
    return value.sec*1_000_000_000 + value.nanosec


def distribution(values):
    array = np.asarray(values, dtype=float)
    return dict(count=len(values), mean=float(np.mean(array)), p50=float(np.percentile(array, 50)),
                p95=float(np.percentile(array, 95)), p99=float(np.percentile(array, 99)), maximum=float(np.max(array)))


def message(row):
    result = BalanceState()
    result.header.stamp, result.header.frame_id = stamp(row["time_s"]), "base_link"
    result.meta = source_stamp("mujoco:spatial", row["epoch"], row["sequence"], row["time_s"],
        clock_domain="mujoco_sim", clock_id="sim:"+row["epoch"], time_semantics="control_sample")
    result.geometry_id = load_spatial_geometry().geometry_id
    result.spatial_contract_version = 1
    result.ros_time_mapping_id = "identity:sim:"+row["epoch"]
    result.traveled_distance_m, result.forward_velocity_m_s = row["distance_m"], row["velocity_m_s"]
    for key in ("roll_rad", "pitch_rad", "yaw_rad", "roll_rate_rad_s", "pitch_rate_rad_s", "yaw_rate_rad_s",
                "imu_sample_time_s", "imu_available_time_s", "imu_valid", "imu_stale", "encoder_valid"):
        setattr(result, key, row[key])
    result.wheel_relative_angle_rad, result.wheel_relative_velocity_rad_s = row["wheel_angles_rad"], row["wheel_velocities_rad_s"]
    result.imu_measurement_age_s = row["imu_age_s"]
    return result


def flatten(value):
    if isinstance(value, dict):
        return [number for key, sub in value.items() for number in flatten(sub)]
    if isinstance(value, (list, tuple)):
        return [number for sub in value for number in flatten(sub)]
    return [value]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--input", type=Path, default=repository_root()/"models/minisegway/spatial_slam_foundation/results/session_20261021/physical_initial/mixed/sensorized_state.jsonl")
    parser.add_argument("--repeats", type=int, default=2)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("repeats must be positive")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    root = repository_root()
    os.environ["ROS_DOMAIN_ID"], os.environ["ROS_LOG_DIR"] = "109", str(output/"ros_logs")
    raw = subprocess.check_output(["git", "show", "HEAD:ros2_ws/src/companionbot_control/companionbot_control/ros/spatial_state.py"], cwd=root)
    if b"class SpatialState(Node)" not in raw:
        raise RuntimeError("HEAD no longer contains the archived pre-migration Python owner; supply a fixed oracle revision")
    snapshot = root/"models/minisegway/native_runtime/results/session_20261023/source_snapshot_before.json"
    original = json.loads(snapshot.read_text())
    source_key = "ros2_ws/src/companionbot_control/companionbot_control/ros/spatial_state.py"
    expected_hash = original.get("files", original).get(source_key)
    if hashlib.sha256(raw).hexdigest() != expected_hash:
        raise RuntimeError("Python owner does not match the pre-migration source snapshot")
    oracle = output/"python_spatial_owner_benchmark_oracle.py"
    oracle.write_bytes(raw + b'\nif __name__ == "__main__":\n    main()\n')
    inputs = [message(json.loads(line)) for line in args.input.read_text().splitlines()]
    rclpy.init()
    probe = Node("native_spatial_benchmark_probe")
    publisher = probe.create_publisher(BalanceState, "/spatialbenchmark/state/balance", qos_profile_sensor_data)
    clock = probe.create_publisher(Clock, "/clock", 10)
    received, received_joints, received_tf, sent = {}, {}, {}, {}
    latency = []
    def odometry(value):
        arrival = time.perf_counter()
        key = ns(value.header.stamp)
        if key in sent:
            latency.append((arrival-sent[key])*1000)
        received[key] = value
    def joints(value):
        received_joints[ns(value.header.stamp)] = value
    def tf(value):
        for transform in value.transforms:
            if transform.child_frame_id in ("base_link", "base_footprint"):
                received_tf[(ns(transform.header.stamp), transform.child_frame_id)] = transform
    probe.create_subscription(Odometry, "/spatialbenchmark/odom", odometry, qos_profile_sensor_data)
    probe.create_subscription(JointState, "/spatialbenchmark/joint_states", joints, qos_profile_sensor_data)
    probe.create_subscription(TFMessage, "/tf", tf, 100)
    summary = dict(scope="WSL host same-physical-input DDS replay, one spatial owner at a time",
        input=str(args.input.resolve()), input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
        input_count=len(inputs), oracle_revision=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        oracle_source_sha256=hashlib.sha256(raw).hexdigest(), source_snapshot=str(snapshot),
        oracle_matches_pre_migration_snapshot=True, cadence_s=.02, repeats=args.repeats,
        parity_tolerance_absolute=3e-12, runs=[], passed=False, hardware_execution_ready=False,
        latency_semantics="same probe: immediately before state DDS publish to odometry callback arrival; includes both DDS legs and probe scheduling; not source acquisition age or MCU latency")
    all_records = defaultdict(list)
    process = None
    def spin_for(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=min(.002, max(0., end-time.monotonic())))
    def until(predicate, seconds=10.):
        deadline = time.monotonic()+seconds
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError("spatial benchmark discovery/exit timeout")
            rclpy.spin_once(probe, timeout_sec=.01)
    try:
        for repeat in range(args.repeats):
            # Alternate order to expose monotonic host-load drift.
            for backend in (("python", "cpp") if repeat%2 == 0 else ("cpp", "python")):
                received.clear(); received_joints.clear(); received_tf.clear(); sent.clear(); latency.clear()
                label = backend+"_"+str(repeat)
                command = [sys.executable, str(oracle)] if backend == "python" else [str(root/"ros2_ws/install/companionbot_control/lib/companionbot_control/spatial_state")]
                command += ["--ros-args", "-r", "__ns:=/spatialbenchmark", "-p", "use_sim_time:=true"]
                with (output/(label+".txt")).open("w") as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    until(lambda: publisher.get_subscription_count() == 1 or process.poll() is not None)
                    if process.poll() is not None:
                        raise RuntimeError((output/(label+".txt")).read_text())
                    clock.publish(Clock(clock=inputs[0].header.stamp)); spin_for(.3)
                    resource = psutil.Process(process.pid)
                    before_cpu = resource.cpu_times(); before_switch = resource.num_ctx_switches()
                    resources = []
                    concurrent = []
                    for candidate in psutil.process_iter(["pid", "cmdline"]):
                        try:
                            arguments = candidate.info["cmdline"] or []
                            if not arguments:
                                continue
                            program = Path(arguments[0]).name
                            if program not in ("python", "python3", "python3.12", "ros2", "spatial_state", "mcu_bridge", "component_container"):
                                continue
                            commandline = " ".join(arguments)
                            if str(root) in commandline and candidate.pid not in (os.getpid(), process.pid):
                                concurrent.append(dict(pid=candidate.pid, command=commandline))
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            pass
                    begin = time.perf_counter()
                    due, sample_due = begin, begin
                    for source in inputs:
                        while time.perf_counter() < due:
                            rclpy.spin_once(probe, timeout_sec=min(.001, max(0., due-time.perf_counter())))
                        if process.poll() is not None:
                            raise RuntimeError((output/(label+".txt")).read_text())
                        clock.publish(Clock(clock=source.header.stamp))
                        sent[ns(source.header.stamp)] = time.perf_counter()
                        publisher.publish(source)
                        now = time.perf_counter()
                        if now >= sample_due:
                            resources.append(dict(elapsed_s=now-begin, rss_bytes=resource.memory_info().rss,
                                threads=resource.num_threads(), cpu=tuple(resource.cpu_times()),
                                context_switches=tuple(resource.num_ctx_switches())))
                            sample_due = now+.1
                        due = max(due+.02, now+.02)
                        rclpy.spin_once(probe, timeout_sec=0)
                    until(lambda: len(received) == len(inputs), 2.)
                    spin_for(.05)
                    elapsed = time.perf_counter()-begin
                    after_cpu, after_switch = resource.cpu_times(), resource.num_ctx_switches()
                    cpu_s = after_cpu.user+after_cpu.system-before_cpu.user-before_cpu.system
                    records = dict(odom={str(k):message_to_ordereddict(v) for k,v in received.items()},
                        joints={str(k):message_to_ordereddict(v) for k,v in received_joints.items()},
                        tf={str(k):message_to_ordereddict(v) for k,v in received_tf.items()})
                    all_records[backend].append(records)
                    (output/(label+"_outputs.json")).write_text(json.dumps(records)+"\n")
                    (output/(label+"_resources.json")).write_text(json.dumps(dict(samples=resources, latency_ms=latency,
                        concurrent_project_processes=concurrent, host_load_average=os.getloadavg()))+"\n")
                    result = dict(label=label, backend=backend, elapsed_s=elapsed, cpu_seconds=cpu_s,
                        cpu_percent_one_core=100*cpu_s/elapsed, peak_rss_bytes=max(r["rss_bytes"] for r in resources),
                        peak_threads=max(r["threads"] for r in resources),
                        voluntary_context_switches=after_switch.voluntary-before_switch.voluntary,
                        involuntary_context_switches=after_switch.involuntary-before_switch.involuntary,
                        odom_samples=len(received), joint_samples=len(received_joints), tf_samples=len(received_tf),
                        roundtrip_ms=distribution(latency), passed=len(received)==len(inputs) and
                        len(received_joints)==len(inputs) and len(received_tf)==2*len(inputs))
                    summary["runs"].append(result)
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=8)
                    result["exit_code"] = process.returncode
                    if process.returncode != 0:
                        result["passed"] = False
                    until(lambda: publisher.get_subscription_count() == 0)
                    process = None
                (output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
        maximum = 0.
        baseline = all_records["python"][0]
        for backend, runs in all_records.items():
            for records in runs:
                for category in ("odom", "joints", "tf"):
                    if set(baseline[category]) != set(records[category]):
                        raise AssertionError("different received source coverage:"+category)
                    for key in baseline[category]:
                        a, b = flatten(baseline[category][key]), flatten(records[category][key])
                        for left, right in zip(a,b,strict=True):
                            if isinstance(left,(float,int)) and isinstance(right,(float,int)):
                                maximum = max(maximum,abs(left-right))
                            elif left != right:
                                raise AssertionError((category,key,left,right))
        summary["max_all_message_numeric_difference"] = maximum
        summary["passed"] = all(row["passed"] for row in summary["runs"]) and maximum <= 3e-12
        if not summary["passed"]:
            raise AssertionError("spatial replay parity/coverage failed")
    except BaseException as error:
        summary["error"] = dict(type=type(error).__name__,message=str(error))
        raise
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            process.wait(timeout=8)
        (output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
        probe.destroy_node(); rclpy.try_shutdown()
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
