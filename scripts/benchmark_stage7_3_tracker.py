"""Synthetic, hardware-free timing/sequence audit for the ByteTrack baseline."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import statistics
import sys
import time
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.detection_stream import PersonDetectionFrame
from perception.person_detector import PersonDetection
from perception.person_tracker import PersonTracker
from perception.ultralytics_bytetrack import UltralyticsByteTrackBackend


def _frame(sequence_id: int, source_time_s: float, *, detected: bool) -> PersonDetectionFrame:
    boxes = (() if not detected else (
        PersonDetection(
            source_sequence_id=sequence_id,
            source_time_s=source_time_s,
            bbox_xyxy_px=(100.0, 40.0, 142.0, 180.0),
            confidence=0.87,
            class_id=0,
            class_name="person",
        ),
    ))
    return PersonDetectionFrame(
        source_id="synthetic-sequence-benchmark",
        source_sequence_id=sequence_id,
        source_time_s=source_time_s,
        frame_width=640,
        frame_height=480,
        detections=boxes,
        inference_start_time_s=source_time_s + 0.001,
        inference_end_time_s=source_time_s + 0.002,
        result_ready_time_s=source_time_s + 0.003,
    )


def _new_tracker() -> tuple[PersonTracker, UltralyticsByteTrackBackend]:
    backend = UltralyticsByteTrackBackend()
    return PersonTracker(backend), backend


def _run_rate_case(rate_hz: int, updates: int = 120) -> dict[str, Any]:
    tracker, backend = _new_tracker()
    start = time.perf_counter() - (updates / rate_hz + 1.0)
    latencies_ms = []
    observed_ids: list[int] = []
    for index in range(updates):
        result = tracker.update(_frame(index, start + index / rate_hz, detected=True))
        latencies_ms.append(result.diagnostics.tracker_update_wall_time_s * 1000.0)
        observed_ids.extend(item.track_id for item in result.tracks)
    unique_ids = sorted(set(observed_ids))
    data = {
        "simulated_source_rate_hz": rate_hz,
        "source_frames": updates,
        "tracker_update_calls": backend.upstream_update_count,
        "unique_observed_track_ids": unique_ids,
        "stable_single_track_id": len(unique_ids) == 1 and len(observed_ids) == updates,
        "tracker_update_ms_mean": statistics.fmean(latencies_ms),
        "tracker_update_ms_median": statistics.median(latencies_ms),
        "tracker_update_ms_p95": sorted(latencies_ms)[int(0.95 * (len(latencies_ms) - 1))],
        "track_buffer_updates": int(backend.effective_config["track_buffer"]),
    }
    tracker.close()
    return data


def _run_drop_case() -> dict[str, Any]:
    rate_hz = 30
    source_frames = 120
    retained = list(range(0, source_frames, 3))
    tracker, backend = _new_tracker()
    start = time.perf_counter() - (source_frames / rate_hz + 1.0)
    gaps = []
    track_ids: set[int] = set()
    previous_sequence: int | None = None
    for sequence_id in retained:
        result = tracker.update(_frame(sequence_id, start + sequence_id / rate_hz,
                                       detected=True))
        gaps.append(0 if previous_sequence is None else sequence_id - previous_sequence - 1)
        previous_sequence = sequence_id
        track_ids.update(item.track_id for item in result.tracks)
    data = {
        "simulated_source_rate_hz": rate_hz,
        "source_frames": source_frames,
        "retained_detection_frames": len(retained),
        "tracker_update_calls": backend.upstream_update_count,
        "trailing_unprocessed_source_frames": source_frames - retained[-1] - 1,
        "total_unprocessed_source_frames": source_frames - len(retained),
        "sequence_gaps": {
            "total": sum(gaps),
            "max": max(gaps, default=0),
            "mean_between_updates": (sum(gaps) / (len(gaps) - 1) if len(gaps) > 1 else 0.0),
        },
        "unique_observed_track_ids": sorted(track_ids),
        "stable_single_track_id": len(track_ids) == 1,
    }
    tracker.close()
    return data


def _run_buffer_lifecycle(rate_hz: int) -> dict[str, Any]:
    tracker, backend = _new_tracker()
    start = time.perf_counter() - (40 / rate_hz + 1.0)
    initial = tracker.update(_frame(0, start, detected=True))
    track_id = initial.tracks[0].track_id
    lost_sequence = None
    removed_sequence = None
    for sequence_id in range(1, 41):
        result = tracker.update(_frame(sequence_id, start + sequence_id / rate_hz,
                                       detected=False))
        if track_id in result.diagnostics.newly_lost_track_ids and lost_sequence is None:
            lost_sequence = sequence_id
        if track_id in result.diagnostics.newly_removed_track_ids:
            removed_sequence = sequence_id
            break
    expected_buffer = int(backend.effective_config["track_buffer"])
    data = {
        "simulated_source_rate_hz": rate_hz,
        "track_buffer_updates": expected_buffer,
        "last_detection_sequence": 0,
        "first_lost_sequence": lost_sequence,
        "removed_sequence": removed_sequence,
        "updates_from_last_detection_to_removed": removed_sequence,
        "seconds_from_last_detection_to_removed": (
            None if removed_sequence is None else removed_sequence / rate_hz
        ),
        "tracker_update_calls": backend.upstream_update_count,
    }
    tracker.close()
    return data


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "models" / "minisegway" / "stage7" / "stage7_3" / "results")
    args = parser.parse_args(argv)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / ".venv"))
    args.output_dir.mkdir(parents=True, exist_ok=True)

    backend = UltralyticsByteTrackBackend()
    config = backend.effective_config
    upstream_version = backend.upstream_version
    backend.close()
    results = {
        "upstream": "Ultralytics BYTETracker",
        "ultralytics_version": upstream_version,
        "upstream_config": config,
        "scope": "synthetic boxes and timestamps only; no detector, camera, or human-motion claim",
        "constant_detection_rate_cases": [_run_rate_case(rate) for rate in (10, 15, 20, 30)],
        "dropped_frame_case": _run_drop_case(),
        "track_buffer_real_time_lifecycle": [_run_buffer_lifecycle(rate) for rate in (10, 15, 20, 30)],
    }
    output_path = args.output_dir / "stage7_3_synthetic_tracker_benchmark.json"
    output_path.write_text(json.dumps(results, indent=2), encoding="utf-8")
    report_path = args.output_dir / "STAGE7_3_SYNTHETIC_TRACKER_BENCHMARK.md"
    lines = [
        "# Stage 7.3 Synthetic ByteTrack Timing and Gap Audit",
        "",
        f"- Upstream: Ultralytics {results['ultralytics_version']} `BYTETracker`.",
        f"- Installed upstream defaults: `{json.dumps(config, sort_keys=True)}`.",
        "- Synthetic boxes only: this establishes API, update-count, ID continuity, and lifecycle semantics; it is not a human-scene tracking result.",
        "- Simulated timestamps do not drive ByteTrack's Kalman filter; each real API call increments its internal frame counter exactly once.",
        "",
        "## Constant detections at simulated source cadence",
        "",
        "| Cadence | Source frames | Tracker updates | IDs stable | Median update | p95 update |",
        "|---:|---:|---:|:---:|---:|---:|",
    ]
    for item in results["constant_detection_rate_cases"]:
        lines.append(
            f"| {item['simulated_source_rate_hz']} Hz | {item['source_frames']} | "
            f"{item['tracker_update_calls']} | {item['stable_single_track_id']} | "
            f"{item['tracker_update_ms_median']:.3f} ms | {item['tracker_update_ms_p95']:.3f} ms |"
        )
    dropped = results["dropped_frame_case"]
    lines.extend([
        "",
        "## Latest-frame style sequence gaps",
        "",
        f"- {dropped['source_frames']} source-frame indices at {dropped['simulated_source_rate_hz']} Hz; "
        f"{dropped['retained_detection_frames']} calls to the tracker.",
        f"- Between-update gap sum/max/mean: {dropped['sequence_gaps']['total']} / "
        f"{dropped['sequence_gaps']['max']} / {dropped['sequence_gaps']['mean_between_updates']:.2f} frames.",
        f"- Trailing unprocessed frames: {dropped['trailing_unprocessed_source_frames']}; "
        f"total unprocessed source frames: {dropped['total_unprocessed_source_frames']}.",
        f"- Stable single temporary tracklet ID: {dropped['stable_single_track_id']}; "
        "no skipped-frame updates were synthesized.",
        "",
        "## `track_buffer` wall-clock interpretation",
        "",
        "| Simulated cadence | Buffer | Updates until removed after last detection | Simulated seconds |",
        "|---:|---:|---:|---:|",
    ])
    for item in results["track_buffer_real_time_lifecycle"]:
        lines.append(
            f"| {item['simulated_source_rate_hz']} Hz | {item['track_buffer_updates']} | "
            f"{item['updates_from_last_detection_to_removed']} | "
            f"{item['seconds_from_last_detection_to_removed']:.3f} s |"
        )
    lines.extend([
        "",
        "Synthetic schedules advance source timestamps without sleeping. The seconds column is the simulated elapsed time represented by the same number of tracker updates; it demonstrates that the upstream buffer is frame-count based.",
        f"- Machine-readable results: `{output_path.resolve()}`",
        "",
    ])
    report_path.write_text("\n".join(lines), encoding="utf-8")
    print(report_path.read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
