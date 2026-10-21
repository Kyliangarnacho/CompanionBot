"""Episode-preserving Stage 4B proprioceptive sliding-window dataset.

The run -> aligned sequence -> lazy window index structure follows the MIT
licensed BorealTC project (https://github.com/norlab-ulaval/BorealTC), adapted
to CompanionBot's raw 100 Hz production-available signals.  Windows are never
materialized or randomly split; the manifest assigns entire episodes first.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterator

import numpy as np


@dataclass(frozen=True)
class WindowIndex:
    episode_id: str
    episode_path: Path
    split: str
    start: int
    end: int
    fully_constant_grade: bool


class LazyEpisodeWindowDataset:
    """Index episode-local windows and slice the source sequence on demand."""

    def __init__(
        self,
        manifest_path: str | Path,
        split: str,
        *,
        window_samples: int,
        step_samples: int,
        constant_grade_only: bool = True,
        window_selection: str | None = None,
        label_tail_samples: int = 1,
    ) -> None:
        self.manifest_path = Path(manifest_path).resolve()
        manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        self.feature_names = tuple(manifest["feature_names"])
        self.window_samples = int(window_samples)
        self.step_samples = int(step_samples)
        self.label_tail_samples = int(label_tail_samples)
        if self.window_samples <= 1 or self.step_samples <= 0:
            raise ValueError("window_samples must exceed one and step_samples be positive")
        if self.label_tail_samples <= 0 or self.label_tail_samples > self.window_samples:
            raise ValueError("label_tail_samples must lie inside the window")
        self.window_selection = window_selection or (
            "constant_grade" if constant_grade_only else "all"
        )
        if self.window_selection not in {"constant_grade", "end_constant_grade", "all"}:
            raise ValueError("unknown window_selection")
        self.indices: list[WindowIndex] = []
        self.episode_metadata: dict[str, dict] = {}
        for episode in manifest["episodes"]:
            if episode["split"] != split or episode["status"] != "usable":
                continue
            path = (self.manifest_path.parent / episode["data_file"]).resolve()
            with np.load(path, allow_pickle=False) as data:
                count = int(data["features"].shape[0])
                valid = np.asarray(data["constant_grade"], dtype=bool)
            self.episode_metadata[episode["episode_id"]] = episode
            for start in range(0, count - self.window_samples + 1, self.step_samples):
                end = start + self.window_samples
                fully_constant = bool(np.all(valid[start:end]))
                if self.window_selection == "constant_grade" and not fully_constant:
                    continue
                if (
                    self.window_selection == "end_constant_grade"
                    and not bool(np.all(valid[end - self.label_tail_samples:end]))
                ):
                    continue
                self.indices.append(WindowIndex(
                    episode["episode_id"], path, split, start, end, fully_constant
                ))
        self._cache_path: Path | None = None
        self._cache: dict[str, np.ndarray] | None = None

    def __len__(self) -> int:
        return len(self.indices)

    def _load(self, path: Path) -> dict[str, np.ndarray]:
        if path != self._cache_path:
            with np.load(path, allow_pickle=False) as data:
                self._cache = {name: np.asarray(data[name]) for name in data.files}
            self._cache_path = path
        assert self._cache is not None
        return self._cache

    def __getitem__(self, index: int) -> dict:
        item = self.indices[index]
        data = self._load(item.episode_path)
        sl = slice(item.start, item.end)
        alpha = np.asarray(data["alpha_gt_deg"][sl], dtype=np.float32)
        slip = np.asarray(data["slip_gt"][sl], dtype=np.float32)
        rough = np.asarray(data["rough_gt"][sl], dtype=np.float32)
        return {
            "x": np.asarray(data["features"][sl], dtype=np.float32).T,
            "alpha_deg": np.float32(np.median(alpha[-self.label_tail_samples:])),
            "slip": np.float32(np.max(slip)),
            "rough": np.float32(np.max(rough)),
            "slip_ratio": np.float32(np.max(data["slip_ratio_gt"][sl])),
            "episode_id": item.episode_id,
            "start": item.start,
            "end": item.end,
            "fully_constant_grade": item.fully_constant_grade,
            "metadata": self.episode_metadata[item.episode_id],
        }

    def __iter__(self) -> Iterator[dict]:
        for index in range(len(self)):
            yield self[index]


def fit_train_normalization(dataset: LazyEpisodeWindowDataset) -> dict:
    """Fit channel normalization once from the train split only."""

    sums = np.zeros(len(dataset.feature_names), dtype=np.float64)
    squares = np.zeros_like(sums)
    count = 0
    seen: set[str] = set()
    for item in dataset.indices:
        if item.episode_id in seen:
            continue
        seen.add(item.episode_id)
        data = dataset._load(item.episode_path)
        valid = np.asarray(data["constant_grade"], dtype=bool)
        values = np.asarray(data["features"][valid], dtype=np.float64)
        sums += np.sum(values, axis=0)
        squares += np.sum(values * values, axis=0)
        count += values.shape[0]
    mean = sums / max(count, 1)
    variance = np.maximum(squares / max(count, 1) - mean * mean, 1e-12)
    return {
        "feature_names": list(dataset.feature_names),
        "mean": mean.tolist(),
        "std": np.sqrt(variance).tolist(),
        "sample_count": count,
        "fit_split": "train",
    }


def normalize_window(x: np.ndarray, normalization: dict) -> np.ndarray:
    mean = np.asarray(normalization["mean"], dtype=np.float32)[:, None]
    std = np.asarray(normalization["std"], dtype=np.float32)[:, None]
    return (np.asarray(x, dtype=np.float32) - mean) / std


def statistical_features(x: np.ndarray) -> np.ndarray:
    """Cheap per-channel mean/std/range/RMS/difference-energy baseline."""

    values = np.asarray(x, dtype=np.float64)
    diff = np.diff(values, axis=1)
    return np.concatenate([
        np.mean(values, axis=1),
        np.std(values, axis=1),
        np.min(values, axis=1),
        np.max(values, axis=1),
        np.sqrt(np.mean(values * values, axis=1)),
        np.sqrt(np.mean(diff * diff, axis=1)),
    ])


def long_context_statistical_features(x: np.ndarray) -> np.ndarray:
    """Existing statistics plus one delta and least-squares trend per channel."""

    values = np.asarray(x, dtype=np.float64)
    base = statistical_features(values)
    delta = values[:, -1] - values[:, 0]
    time_axis = np.linspace(-1.0, 1.0, values.shape[1], dtype=np.float64)
    trend = (values @ time_axis) / float(time_axis @ time_axis)
    return np.concatenate([base, delta, trend])


def materialize(dataset: LazyEpisodeWindowDataset, normalization: dict) -> dict:
    """Materialize only the small pilot's model inputs, never copied raw windows."""

    x_raw, x_stats, alpha, slip, rough, episode_ids = [], [], [], [], [], []
    for sample in dataset:
        normalized = normalize_window(sample["x"], normalization)
        x_raw.append(normalized)
        x_stats.append(statistical_features(normalized))
        alpha.append(sample["alpha_deg"])
        slip.append(sample["slip"])
        rough.append(sample["rough"])
        episode_ids.append(sample["episode_id"])
    return {
        "x_raw": np.asarray(x_raw, dtype=np.float32),
        "x_stats": np.asarray(x_stats, dtype=np.float32),
        "alpha": np.asarray(alpha, dtype=np.float32),
        "slip": np.asarray(slip, dtype=np.float32),
        "rough": np.asarray(rough, dtype=np.float32),
        "episode_ids": np.asarray(episode_ids),
    }
