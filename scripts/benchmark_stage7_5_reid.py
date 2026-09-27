"""Measure OSNet x0.25 crop embedding cost on a deterministic synthetic image."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform
import sys
import time
from typing import Sequence

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, default=None)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--iterations", type=int, default=50)
    parser.add_argument("--output", type=Path, default=(
        ROOT / "models" / "minisegway" / "stage7" / "stage7_5" / "results"
        / "reid_cpu_benchmark.json"
    ))
    return parser


def _percentiles(samples_s: list[float]) -> dict[str, float]:
    import numpy as np

    values_ms = np.asarray(samples_s, dtype=np.float64) * 1000.0
    return {
        "median_ms": float(np.median(values_ms)),
        "p95_ms": float(np.percentile(values_ms, 95)),
        "min_ms": float(np.min(values_ms)),
        "max_ms": float(np.max(values_ms)),
    }


def _paired_input(sequence_id: int, image: np.ndarray) -> tuple[ColorFrame, PersonTrackingFrame, TrackedPerson]:
    from perception.camera import ColorFrame
    from perception.person_tracker import (
        PersonTrackingDiagnostics,
        PersonTrackingFrame,
        TrackedPerson,
    )

    source_time = float(sequence_id) * 0.01
    frame = ColorFrame(
        bgr=image,
        sequence_id=sequence_id,
        source_id="synthetic-benchmark",
        width=image.shape[1],
        height=image.shape[0],
        host_receive_time_s=source_time,
    )
    track = TrackedPerson(
        track_id=1,
        bbox_xyxy_px=(8.0, 4.0, float(image.shape[1] - 8), float(image.shape[0] - 4)),
        confidence=1.0,
        class_id=0,
        class_name="person",
    )
    tracking = PersonTrackingFrame(
        source_id=frame.source_id,
        source_sequence_id=sequence_id,
        source_time_s=source_time,
        frame_width=frame.width,
        frame_height=frame.height,
        detection_count=1,
        tracks=(track,),
        diagnostics=PersonTrackingDiagnostics(
            update_index=sequence_id + 1,
            active_track_count=1,
            created_track_ids=(1,) if sequence_id == 0 else (),
            newly_lost_track_ids=(),
            newly_removed_track_ids=(),
            source_sequence_gap=0,
            tracker_update_wall_time_s=0.0,
        ),
        tracking_start_time_s=source_time + 0.001,
        tracking_end_time_s=source_time + 0.002,
        result_ready_time_s=source_time + 0.003,
    )
    return frame, tracking, track


def main(argv: Sequence[str] | None = None) -> int:
    print(f"python: {sys.executable}")
    print(f"version: {sys.version.split()[0]}")
    import pip
    print(f"pip: {pip.__version__} {pip.__file__}")
    if sys.prefix == sys.base_prefix:
        raise SystemExit("run this benchmark with the CompanionBot .venv interpreter")
    import numpy as np

    from perception.person_reid import (
        OSNET_WEIGHT_COMMIT,
        OSNET_WEIGHT_SHA256,
        TORCHREID_SOURCE_COMMIT,
        PersonReIdentifier,
        TorchReIDOSNetBackend,
        default_osnet_weights_path,
    )
    args = _parser().parse_args(argv)
    if args.warmup < 0 or args.iterations < 1:
        raise SystemExit("--warmup must be nonnegative and --iterations must be positive")
    weights_path = args.weights or default_osnet_weights_path()
    backend = TorchReIDOSNetBackend(weights_path)
    reidentifier = PersonReIdentifier(backend)

    rng = np.random.default_rng(202607)
    image = rng.integers(0, 256, size=(256, 128, 3), dtype=np.uint8)
    preprocess_s: list[float] = []
    inference_s: list[float] = []
    total_s: list[float] = []
    for index in range(args.warmup + args.iterations):
        frame, tracking, paired_track = _paired_input(index, image)
        embedding = reidentifier.embed_track(frame, tracking, paired_track)
        if embedding.dimension != 512:
            raise RuntimeError(f"expected a 512D OSNet embedding, got {embedding.dimension}")
        if index >= args.warmup:
            preprocess_s.append(embedding.preprocess_wall_time_s)
            inference_s.append(embedding.inference_wall_time_s)
            total_s.append(embedding.total_wall_time_s)

    try:
        import torch
        torch_version = torch.__version__
        torch_threads = torch.get_num_threads()
    except ImportError:
        torch_version = "unavailable"
        torch_threads = None
    try:
        import torchvision
        torchvision_version = torchvision.__version__
    except ImportError:
        torchvision_version = "unavailable"

    report = {
        "benchmark": "synthetic random uint8 BGR crop; no person-identity evidence",
        "backend": backend.backend_name,
        "model": backend.model_name,
        "upstream_source_commit": TORCHREID_SOURCE_COMMIT,
        "weights_revision": OSNET_WEIGHT_COMMIT,
        "weights_sha256": OSNET_WEIGHT_SHA256,
        "weights": str(weights_path.resolve()),
        "embedding_dimension": 512,
        "device": "cpu",
        "warmup_count": args.warmup,
        "measurement_count": args.iterations,
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "cpu_count": __import__("os").cpu_count(),
        "torch": torch_version,
        "torchvision": torchvision_version,
        "torch_threads": torch_threads,
        "timings": {
            "preprocess": _percentiles(preprocess_s),
            "inference": _percentiles(inference_s),
            "total_embedding": _percentiles(total_s),
        },
        "automatic_master_rebind": False,
        "embedding_vectors_written": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"benchmark output: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
