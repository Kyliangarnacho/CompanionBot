"""Diagnostic identical-input LIGHTWEIGHT benchmark; no robot command interface.

Run each backend in a fresh process with the same seed, blocks and instrumentation.
Wire bytes and native/legacy full-boundary wall/thread CPU timings are retained.
tracemalloc/cProfile are separate diagnostic runs, not fair speed measurements.
"""
import argparse
import cProfile
import gc
import hashlib
import json
from pathlib import Path
import platform
import resource
import sys
import time
import tracemalloc
from datetime import datetime, timezone

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
BENCHMARK_VERSION = 3


def implementation_fingerprints(backend):
    paths = [
        ROOT/'ros2_ws/src/companionbot_follow/companionbot_follow/core/rolling_reference.py',
        ROOT/'ros2_ws/src/companionbot_follow/companionbot_follow/core/reference_lifecycle.py',
        ROOT/'ros2_ws/src/companionbot_follow/companionbot_follow/runtime/reference_producer.py',
        ROOT/'ros2_ws/src/companionbot_follow/src/full_reference.cpp',
        ROOT/'ros2_ws/src/companionbot_follow/src/full_bindings.cpp',
        ROOT/'ros2_ws/src/companionbot_follow/include/companionbot_follow/full_reference.hpp',
        ROOT/'ros2_ws/src/companionbot_follow/src/lightweight_producer.cpp',
        ROOT/'ros2_ws/install/companionbot_follow/lib/libcompanionbot_follow_native.so',
    ]
    result = {str(path.relative_to(ROOT)): dict(resolved=str(path.resolve()),
        sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for path in paths}
    expected = paths[-1].resolve()
    loaded = []
    for line in Path('/proc/self/maps').read_text().splitlines():
        if 'libcompanionbot_follow_native.so' in line:
            loaded.append(line.split(maxsplit=5)[-1])
    loaded = sorted(set(loaded))
    if backend == 'native' and (not loaded or any(Path(path).resolve() != expected for path in loaded)):
        raise RuntimeError('native_benchmark_did_not_load_the_current_workspace_library')
    return result, loaded


def benchmark_configuration():
    """Read frozen inputs without importing the simulation/physics runtime."""
    paths = {
        "stream": ROOT / "models/minisegway/stage5/config/stage5_v1_5_pi_mcu_stream_config.json",
        "motion": ROOT / "models/minisegway/stage3/results/config/stage3a_commanded_motion_config.json",
        "dynamic": ROOT / "models/minisegway/stage3/results/config/stage3a_dynamic_nominal_reference_config.json",
        "runtime": ROOT / "models/minisegway/stage3/results/config/stage3a_velocity_feedforward_lean_tail_config.json",
    }
    source = {key: json.loads(path.read_text()) for key, path in paths.items()}
    stream, motion, dynamic, runtime = (source[key] for key in ("stream", "motion", "dynamic", "runtime"))
    cfg = dict(dt_s=motion["controller_dt_s"], block_samples=stream["reference_block"]["sample_count"],
        max_velocity_m_s=motion["reference_limits"]["max_velocity_m_s"],
        max_acceleration_m_s2=motion["reference_limits"]["max_acceleration_m_s2"],
        max_jerk_m_s3=dynamic["max_jerk_m_s3"],
        accept_delta_v_m_s=stream["scheduler"]["T_accept_delta_v_m_s"],
        full_delta_v_m_s=stream["scheduler"]["T_full_delta_v_m_s"],
        candidate_stable_window_s=stream["scheduler"]["candidate_stable_window_s"],
        candidate_stable_range_m_s=stream["scheduler"]["candidate_stable_range_m_s"],
        initial_accepted_velocity_m_s=stream["scheduler"]["initial_accepted_velocity_m_s"],
        feedforward_fade_s=runtime["velocity_lifecycle"]["feedforward_fade_s"],
        quiet_theta_rad=float(np.deg2rad(stream["full_dynamic"]["terminal_theta_abs_deg"])),
        quiet_theta_dot_rad_s=float(np.deg2rad(stream["full_dynamic"]["terminal_theta_dot_abs_deg_s"])),
        candidate_stable_gate=False, minimum_delta_gate=False, force_full_dynamic=False,
        defer_full_accept_until_observed_exit=False,
        source_files={key: dict(path=str(path.relative_to(ROOT)),
            sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for key, path in paths.items()})
    if cfg["dt_s"] != stream["reference_block"]["dt_s"] or cfg["block_samples"] != 25:
        raise ValueError("canonical_reference_geometry_mismatch")
    for key in ("controller_dt_s", "max_velocity_m_s", "max_acceleration_m_s2", "max_jerk_m_s3"):
        actual = cfg["dt_s"] if key == "controller_dt_s" else cfg[key]
        if actual != stream["frozen_upstream"][key]:
            raise ValueError("frozen_reference_limit_mismatch:" + key)
    return cfg


def python_producer(cfg):
    from companionbot_follow.core.rolling_reference import (
        MotionIntent, RuckigLightweightVelocityPlanner, SparseTwoPathReferenceSource,
        SparseVelocityCommandScheduler, TargetObservation,
    )
    from companionbot_follow.runtime.reference_producer import PiReferenceBlockProducer
    class Reader:
        time = velocity = yaw = 0.
        def read(self):
            return TargetObservation(self.time, self.velocity, self.yaw)
    reader = Reader()
    planner = RuckigLightweightVelocityPlanner(controller_dt_s=cfg["dt_s"],
        max_velocity_m_s=cfg["max_velocity_m_s"], max_acceleration_m_s2=cfg["max_acceleration_m_s2"],
        max_jerk_m_s3=cfg["max_jerk_m_s3"],
        A=np.eye(4), B=np.zeros((4, 1)), state_scales=np.ones(4), input_scale_nm=1.)
    scheduler = SparseVelocityCommandScheduler(accept_delta_v_m_s=cfg["accept_delta_v_m_s"],
        full_delta_v_m_s=cfg["full_delta_v_m_s"], stable_window_s=cfg["candidate_stable_window_s"],
        stable_range_m_s=cfg["candidate_stable_range_m_s"],
        initial_accepted_velocity_m_s=cfg["initial_accepted_velocity_m_s"],
        require_stable_candidate=cfg["candidate_stable_gate"], minimum_delta_enabled=cfg["minimum_delta_gate"],
        force_full_dynamic=cfg["force_full_dynamic"])
    source = SparseTwoPathReferenceSource(lightweight_planner=planner, full_planner=planner,
        scheduler=scheduler, block_samples=cfg["block_samples"], feedforward_fade_s=cfg["feedforward_fade_s"],
        observation_reader=reader.read,
        follower=lambda o: MotionIntent(o.capture_time_s, o.x_forward_m, o.y_left_m),
        quiet_theta_rad=cfg["quiet_theta_rad"], quiet_theta_dot_rad_s=cfg["quiet_theta_dot_rad_s"],
        defer_full_accept_until_observed_exit=cfg["defer_full_accept_until_observed_exit"])
    producer = PiReferenceBlockProducer(source)
    def run(index, velocity, yaw):
        reader.time, reader.velocity, reader.yaw = index * cfg["block_samples"] * cfg["dt_s"], velocity, yaw
        packet, internal = producer.produce(index * cfg["block_samples"], reader.time)
        return packet.to_bytes(), internal, None
    return run, "25 Python source.command + one existing compiled Ruckig calculate/at_time profile on retarget"


def native_producer(cfg, *, lightweight_only=False):
    from companionbot_follow import _native_runtime as native
    limits = native.ProducerConfig()
    limits.dt, limits.block_samples = cfg["dt_s"], cfg["block_samples"]
    limits.max_velocity, limits.max_acceleration = cfg["max_velocity_m_s"], cfg["max_acceleration_m_s2"]
    limits.max_jerk, limits.full_delta_velocity = cfg["max_jerk_m_s3"], cfg["full_delta_v_m_s"]
    if lightweight_only:
        producer = native.LightweightProducer(limits)
    else:
        config = native.TwoPathConfig()
        config.producer = limits
        scheduler = native.SchedulerConfig()
        scheduler.accept_delta_velocity = cfg["accept_delta_v_m_s"]
        scheduler.full_delta_velocity = cfg["full_delta_v_m_s"]
        scheduler.stable_window_s, scheduler.stable_range_velocity = cfg["candidate_stable_window_s"], cfg["candidate_stable_range_m_s"]
        scheduler.initial_accepted_velocity = cfg["initial_accepted_velocity_m_s"]
        scheduler.require_stable_candidate, scheduler.minimum_delta_enabled = cfg["candidate_stable_gate"], cfg["minimum_delta_gate"]
        scheduler.force_full_dynamic = cfg["force_full_dynamic"]
        config.scheduler = scheduler
        config.feedforward_fade_s, config.quiet_theta_rad = cfg["feedforward_fade_s"], cfg["quiet_theta_rad"]
        config.quiet_theta_dot_rad_s = cfg["quiet_theta_dot_rad_s"]
        config.defer_full_accept_until_observed_exit = cfg["defer_full_accept_until_observed_exit"]
        producer = native.TwoPathProducer(config)
    def run(index, velocity, yaw):
        tick, timestamp = index * cfg["block_samples"], index * cfg["block_samples"] * cfg["dt_s"]
        value = producer.produce(tick, timestamp, timestamp, velocity, yaw)
        return value.bytes, value.compute_wall, value.compute_cpu
    owner = "diagnostic LightweightProducer" if lightweight_only else "production TwoPathProducer with navigation policy"
    return run, owner + "; one produce batch plus three bytes/metric property reads per block; no per-sample Python callback"


def quantiles(values):
    a = np.asarray(values, dtype=float)
    return dict(count=len(a), p50_s=float(np.quantile(a, .5)), p95_s=float(np.quantile(a, .95)),
                p99_s=float(np.quantile(a, .99)), max_s=float(np.max(a)), total_s=float(a.sum()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("python", "native"), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--blocks", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=20261023)
    parser.add_argument("--gc-policy", choices=("enabled", "deferred"), default="deferred")
    parser.add_argument("--allocations", action="store_true")
    parser.add_argument("--profile", action="store_true")
    parser.add_argument("--native-lightweight-only", action="store_true",
                        help="Separate diagnostic kernel only; never a production migration result")
    args = parser.parse_args()
    if args.blocks < 100:
        parser.error("at least 100 blocks required")
    if args.native_lightweight_only and args.backend != "native":
        parser.error("--native-lightweight-only requires --backend native")
    out = args.output_dir.resolve(); out.mkdir(parents=True, exist_ok=True)
    if any(out.iterdir()):
        parser.error("output directory must be empty to preserve earlier evidence")
    cfg = benchmark_configuration()
    rng = np.random.default_rng(args.seed)
    changes = rng.uniform([0., -.5], [.25, .5], (int(np.ceil(args.blocks / 7)), 2))
    commands = np.repeat(changes, 7, axis=0)[:args.blocks]
    np.save(out / "inputs.npy", commands)
    factory = (lambda: python_producer(cfg)) if args.backend == "python" else (
        lambda: native_producer(cfg, lightweight_only=args.native_lightweight_only))
    run, calls = factory()
    # Warm the same algorithm and memory path, then reset both state owners.
    for i in range(50):
        run(i, float(commands[i, 0]), float(commands[i, 1]))
    run, _ = factory()
    implementation_files, loaded_libraries = implementation_fingerprints(args.backend)
    gc.collect()
    old_gc = gc.isenabled()
    if args.gc_policy == "deferred": gc.disable()
    else: gc.enable()
    pauses, cleanup_pauses, active = [], [], {}
    gc_phase = "measured_window"
    def on_gc(phase, info):
        generation = info["generation"]
        if phase == "start": active[generation] = time.perf_counter_ns()
        elif generation in active:
            row = dict(generation=generation, wall_s=(time.perf_counter_ns()-active.pop(generation))*1e-9)
            (pauses if gc_phase == "measured_window" else cleanup_pauses).append(row)
    gc.callbacks.append(on_gc)
    if args.allocations: tracemalloc.start()
    profiler = cProfile.Profile() if args.profile else None
    if profiler: profiler.enable()
    wall, cpu, internal, internal_cpu, payloads = [], [], [], [], []
    usage_before = resource.getrusage(resource.RUSAGE_SELF)
    thread_before = resource.getrusage(resource.RUSAGE_THREAD)
    started = time.perf_counter()
    started_utc = datetime.now(timezone.utc).isoformat()
    try:
        for i, (velocity, yaw) in enumerate(commands):
            before_cpu, before = time.thread_time_ns(), time.perf_counter_ns()
            wire, measured, measured_cpu = run(i, float(velocity), float(yaw))
            wall.append((time.perf_counter_ns()-before)*1e-9)
            cpu.append((time.thread_time_ns()-before_cpu)*1e-9)
            internal.append(measured)
            if measured_cpu is not None: internal_cpu.append(measured_cpu)
            payloads.append(wire)
    finally:
        elapsed = time.perf_counter()-started
        # Close ALL measured-window counters before profiler serialization,
        # GC policy restoration, or diagnostic/file-output cleanup. v2 took
        # process rusage after restoration; its per-call timings remain valid
        # but its process CPU window must not be pooled with this one.
        usage = resource.getrusage(resource.RUSAGE_SELF)
        thread = resource.getrusage(resource.RUSAGE_THREAD)
        gc_phase = "cleanup"
        cleanup_started = time.perf_counter()
        if profiler: profiler.disable(); profiler.dump_stats(str(out / "profile.pstats"))
        if old_gc: gc.enable()
        else: gc.disable()
        cleanup_usage = resource.getrusage(resource.RUSAGE_SELF)
        cleanup_elapsed = time.perf_counter()-cleanup_started
        gc.callbacks.remove(on_gc)
    allocation = None
    if args.allocations:
        current, peak = tracemalloc.get_traced_memory()
        allocation = dict(current_bytes=current, peak_bytes=peak,
            top=[str(row) for row in tracemalloc.take_snapshot().statistics("lineno")[:25]])
        tracemalloc.stop()
    (out / "reference_blocks.bin").write_bytes(b"".join(payloads))
    np.savez_compressed(out / "timings.npz", wall_s=wall, thread_cpu_s=cpu,
                        internal_wall_s=internal, internal_thread_cpu_s=internal_cpu)
    implementation = ("python_frozen_oracle" if args.backend == "python" else
        "native_lightweight_diagnostic_only" if args.native_lightweight_only else "native_cpp_two_path_producer")
    report = dict(scope="WSL host software microbenchmark, not MCU/Pi or physical throughput",
        benchmark_version=BENCHMARK_VERSION, implementation=implementation,
        production_comparison_eligible=not args.native_lightweight_only,
        native_current_production_owner=args.backend=="native" and not args.native_lightweight_only,
        backend=args.backend, python=sys.executable, version=sys.version, platform=platform.platform(),
        blocks=args.blocks, samples=args.blocks*cfg["block_samples"], seed=args.seed, gc_policy=args.gc_policy,
        started_utc=started_utc, finished_utc=datetime.now(timezone.utc).isoformat(),
        instrumentation=dict(tracemalloc=args.allocations, cprofile=args.profile),
        configuration=cfg,
        configuration_sha256=hashlib.sha256(json.dumps(cfg,sort_keys=True).encode()).hexdigest(),
        input_sha256=hashlib.sha256((out/"inputs.npy").read_bytes()).hexdigest(),
        wire_sha256=hashlib.sha256((out/"reference_blocks.bin").read_bytes()).hexdigest(),
        benchmark_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        implementation_files=implementation_files, loaded_native_libraries=loaded_libraries,
        wall=quantiles(wall), thread_cpu=quantiles(cpu), internal_wall=quantiles(internal),
        internal_measurement_note="legacy internal timing excludes rows/validation/encoding; compare full wall/thread_cpu boundaries",
        elapsed_wall_s=elapsed, blocks_per_wall_second=args.blocks/elapsed,
        measured_window_cpu_s=usage.ru_utime+usage.ru_stime-usage_before.ru_utime-usage_before.ru_stime,
        process_cpu_scope="same loop window as elapsed wall; endpoint rusage captured before GC restoration/profiler/file cleanup; clock probes have finite syscall overhead",
        cleanup_wall_s=cleanup_elapsed,
        cleanup_cpu_s=cleanup_usage.ru_utime+cleanup_usage.ru_stime-usage.ru_utime-usage.ru_stime,
        cleanup_gc_pauses=cleanup_pauses,
        peak_process_rss_mib=usage.ru_maxrss/1024,
        rss_scope="process lifetime high-water through measured-window end, including imports and 50-block warmup; excludes later cleanup/file output; not a per-allocator peak",
        voluntary_context_switches=usage.ru_nvcsw-usage_before.ru_nvcsw,
        involuntary_context_switches=usage.ru_nivcsw-usage_before.ru_nivcsw,
        thread_context_switches=dict(voluntary=thread.ru_nvcsw-thread_before.ru_nvcsw,
            involuntary=thread.ru_nivcsw-thread_before.ru_nivcsw),
        gc_pauses=pauses, allocation_diagnostic=allocation,
        cross_language_boundary=calls, gil_wait="not separately measured; do not infer from total CPU",
        native_pybind_calls_in_measured_window=args.blocks*4 if args.backend=="native" else None,
        native_pybind_call_count_note="produce plus bytes, compute_wall and compute_cpu property reads; C++ Ruckig internal calls are not additional language crossings",
        watchdog="microbenchmark has no running MCU; use physical regression for starvation/underrun",
        hardware_execution_ready=False)
    if internal_cpu: report["internal_thread_cpu"] = quantiles(internal_cpu)
    (out / "summary.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == "__main__": main()
