"""Thin adapter around Ultralytics' installed ByteTrack implementation.

Only this module imports Ultralytics Boxes/BYTETracker. Defaults are read from
the installed upstream ``bytetrack.yaml`` without local threshold changes.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from perception.person_detector import PersonDetection
from perception.person_tracker import BackendTrack, TrackerBackendUpdate


class UltralyticsByteTrackBackend:
    """Adapt backend-neutral person detections to the official BYTETracker API."""

    def __init__(self) -> None:
        try:
            import ultralytics
            import yaml
            from ultralytics.engine.results import Boxes
            from ultralytics.trackers.byte_tracker import BYTETracker
        except ImportError as error:
            raise RuntimeError(
                "Ultralytics and its declared dependencies are required for ByteTrack"
            ) from error

        self._boxes_type = Boxes
        self._upstream_version = str(ultralytics.__version__)
        self._config_path = (
            Path(ultralytics.__file__).resolve().parent
            / "cfg" / "trackers" / "bytetrack.yaml"
        )
        if not self._config_path.is_file():
            raise RuntimeError(f"installed Ultralytics ByteTrack config is missing: {self._config_path}")
        with self._config_path.open("r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
        if not isinstance(config, dict) or config.get("tracker_type") != "bytetrack":
            raise RuntimeError("installed bytetrack.yaml has an unexpected format")
        self._config = dict(config)
        self._tracker = BYTETracker(args=SimpleNamespace(**self._config))
        self._closed = False

    @property
    def name(self) -> str:
        return "ultralytics.bytetrack"

    @property
    def upstream_version(self) -> str:
        return self._upstream_version

    @property
    def config_path(self) -> Path:
        return self._config_path

    @property
    def effective_config(self) -> dict[str, Any]:
        """Return a copy of upstream defaults for run provenance."""
        return dict(self._config)

    @property
    def upstream_update_count(self) -> int:
        """Number of actual update calls; source sequence gaps are not expanded."""
        return int(self._tracker.frame_id)

    def update(
        self,
        detections: tuple[PersonDetection, ...],
        frame_width: int,
        frame_height: int,
    ) -> TrackerBackendUpdate:
        if self._closed:
            raise RuntimeError("ByteTrack backend is closed")
        if any(item.class_name.casefold() != "person" for item in detections):
            raise ValueError("ByteTrack person backend accepts person detections only")
        rows = [
            (*item.bbox_xyxy_px, item.confidence, item.class_id)
            for item in detections
        ]
        # Ultralytics Boxes(N, 6) is [x1, y1, x2, y2, confidence, class_id].
        boxes_data = np.asarray(rows, dtype=np.float32).reshape(-1, 6)
        boxes = self._boxes_type(boxes_data, (frame_height, frame_width))
        previous_lost = {int(item.track_id) for item in self._tracker.lost_stracks}
        previous_removed = {int(item.track_id) for item in self._tracker.removed_stracks}
        output = self._tracker.update(boxes)
        lost_now = {int(item.track_id) for item in self._tracker.lost_stracks}
        removed_now = {int(item.track_id) for item in self._tracker.removed_stracks}

        tracks: list[BackendTrack] = []
        for row in np.asarray(output, dtype=np.float64).reshape(-1, 8):
            track_id = int(round(row[4]))
            class_id = int(round(row[6]))
            tracks.append(BackendTrack(
                track_id=track_id,
                bbox_xyxy_px=tuple(float(value) for value in row[:4]),
                confidence=float(row[5]),
                class_id=class_id,
            ))
        return TrackerBackendUpdate(
            tracks=tuple(tracks),
            newly_lost_track_ids=tuple(sorted(lost_now - previous_lost)),
            newly_removed_track_ids=tuple(sorted(removed_now - previous_removed)),
        )

    def reset(self) -> None:
        if self._closed:
            raise RuntimeError("ByteTrack backend is closed")
        self._tracker.reset()

    def close(self) -> None:
        if not self._closed:
            self._tracker.reset()
            self._closed = True
