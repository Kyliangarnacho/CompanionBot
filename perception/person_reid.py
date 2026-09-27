"""Backend-neutral appearance evidence for manually selected Master tracks.

This layer reports appearance similarity only. It never changes MasterManager
state or binds a candidate track to Master.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import importlib.util
from pathlib import Path
import sysconfig
import time
from typing import Iterable, Protocol, runtime_checkable
from urllib.request import urlopen

import cv2
import numpy as np

from perception.camera import ColorFrame
from perception.master_selection import MasterState, MasterTrackingFrame
from perception.person_tracker import PersonTrackingFrame, TrackedPerson


OSNET_MODEL_NAME = "osnet_x0_25_msmt17"
OSNET_WEIGHT_FILENAME = (
    "osnet_x0_25_msmt17_combineall_256x128_amsgrad_ep150_stp60_"
    "lr0.0015_b64_fb10_softmax_labelsmooth_flip_jitter.pth"
)
OSNET_WEIGHT_COMMIT = "01af85e82a9db4f3a4f6ed3a72ed9150bd416d04"
OSNET_WEIGHT_SHA256 = "cf55163d78fc44c62c82f85ab62d39f10438679b5abe8c698ae08cfa84aa6e18"
OSNET_WEIGHT_URL = (
    "https://huggingface.co/kaiyangzhou/osnet/resolve/"
    + OSNET_WEIGHT_COMMIT + "/"
    + OSNET_WEIGHT_FILENAME
)
TORCHREID_SOURCE_COMMIT = "f8cd150fdf77e8d9e1ed143b7f308c2c609ded50"


class ReIDInputError(ValueError):
    """A frame, box, or crop cannot safely be used for appearance inference."""


@dataclass(frozen=True)
class BackendEmbedding:
    """Private backend result; public APIs expose only NumPy-based embeddings."""

    vector: np.ndarray
    preprocess_wall_time_s: float
    inference_wall_time_s: float


@runtime_checkable
class EmbeddingBackend(Protocol):
    """Small image-crop interface implemented by an inference adapter."""

    @property
    def backend_name(self) -> str: ...

    @property
    def model_name(self) -> str: ...

    def embed_crop_bgr(self, crop_bgr: np.ndarray) -> BackendEmbedding: ...


@dataclass(frozen=True)
class AppearanceEmbedding:
    """Normalized appearance feature plus its source and backend provenance."""

    source_id: str
    source_sequence_id: int
    track_id: int
    vector: np.ndarray
    backend_name: str
    model_name: str
    preprocess_wall_time_s: float
    inference_wall_time_s: float
    total_wall_time_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.source_id, str) or not self.source_id.strip():
            raise ValueError("source_id must not be empty")
        if (isinstance(self.source_sequence_id, bool)
                or not isinstance(self.source_sequence_id, int)
                or self.source_sequence_id < 0):
            raise ValueError("source_sequence_id must be nonnegative")
        if (isinstance(self.track_id, bool) or not isinstance(self.track_id, int)
                or self.track_id <= 0):
            raise ValueError("track_id must be positive")
        if not isinstance(self.vector, np.ndarray):
            raise ValueError("embedding vector must be a NumPy array")
        vector = np.asarray(self.vector, dtype=np.float32)
        if vector.ndim != 1 or vector.size == 0:
            raise ValueError("embedding vector must be a nonempty 1D array")
        if not np.isfinite(vector).all():
            raise ValueError("embedding vector must contain only finite values")
        norm = float(np.linalg.norm(vector))
        if not math.isfinite(norm) or norm <= 1e-12:
            raise ValueError("embedding vector must have nonzero L2 norm")
        normalized = np.array(vector / norm, dtype=np.float32, copy=True)
        normalized.setflags(write=False)
        object.__setattr__(self, "vector", normalized)
        for name in ("backend_name", "model_name"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must not be empty")
        for name in (
            "preprocess_wall_time_s", "inference_wall_time_s", "total_wall_time_s"
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and nonnegative")
            object.__setattr__(self, name, value)

    @property
    def dimension(self) -> int:
        return int(self.vector.size)


class PersonReIdentifier:
    """Validate person crops and wrap any compatible appearance backend.

    ``embed`` accepts a BGR image and original-frame xyxy box. The stronger
    ``embed_track`` entry point also verifies the exact ColorFrame / tracking
    result pair and is the required path for Master reference and candidates.
    """

    def __init__(self, backend: EmbeddingBackend) -> None:
        if not isinstance(backend, EmbeddingBackend):
            raise TypeError("backend must implement EmbeddingBackend")
        self._backend = backend

    @property
    def backend_name(self) -> str:
        return self._backend.backend_name

    @property
    def model_name(self) -> str:
        return self._backend.model_name

    def embed(
        self,
        image_bgr: np.ndarray,
        bbox_xyxy_px: tuple[float, float, float, float],
        *,
        source_id: str,
        source_sequence_id: int,
        track_id: int,
    ) -> AppearanceEmbedding:
        """Crop without expansion, then return a finite L2-normalized feature."""
        start_ns = time.perf_counter_ns()
        crop = crop_person_bgr(image_bgr, bbox_xyxy_px)
        result = self._backend.embed_crop_bgr(crop)
        if not isinstance(result, BackendEmbedding):
            raise TypeError("embedding backend must return BackendEmbedding")
        if not isinstance(result.vector, np.ndarray):
            raise ValueError("embedding backend vector must be a NumPy array")
        raw_vector = np.asarray(result.vector, dtype=np.float32)
        if raw_vector.ndim != 1 or raw_vector.size == 0 or not np.isfinite(raw_vector).all():
            raise ValueError("embedding backend must return a finite nonempty 1D vector")
        norm = float(np.linalg.norm(raw_vector))
        if not math.isfinite(norm) or norm <= 1e-12:
            raise ValueError("embedding backend vector must have nonzero L2 norm")
        normalized_vector = np.array(raw_vector / norm, dtype=np.float32, copy=True)
        total_s = (time.perf_counter_ns() - start_ns) * 1e-9
        return AppearanceEmbedding(
            source_id=source_id,
            source_sequence_id=source_sequence_id,
            track_id=track_id,
            vector=normalized_vector,
            backend_name=self.backend_name,
            model_name=self.model_name,
            preprocess_wall_time_s=result.preprocess_wall_time_s,
            inference_wall_time_s=result.inference_wall_time_s,
            total_wall_time_s=max(total_s, result.preprocess_wall_time_s
                                  + result.inference_wall_time_s),
        )

    def embed_track(
        self,
        frame: ColorFrame,
        tracking_frame: PersonTrackingFrame,
        track: TrackedPerson,
    ) -> AppearanceEmbedding:
        """Embed only if the color image and tracking output are exactly paired."""
        _require_frame_alignment(frame, tracking_frame)
        if not isinstance(track, TrackedPerson):
            raise TypeError("track must be a TrackedPerson")
        source_track = next(
            (item for item in tracking_frame.tracks if item.track_id == track.track_id),
            None,
        )
        if source_track != track:
            raise ReIDInputError("track is not the exact active track in this frame")
        return self.embed(
            frame.bgr,
            track.bbox_xyxy_px,
            source_id=frame.source_id,
            source_sequence_id=frame.sequence_id,
            track_id=track.track_id,
        )


def _require_frame_alignment(
    frame: ColorFrame, tracking_frame: PersonTrackingFrame
) -> None:
    if not isinstance(frame, ColorFrame):
        raise TypeError("frame must be a ColorFrame")
    if not isinstance(tracking_frame, PersonTrackingFrame):
        raise TypeError("tracking_frame must be a PersonTrackingFrame")
    if frame.sequence_id != tracking_frame.source_sequence_id:
        raise ReIDInputError(
            "ColorFrame.sequence_id must equal PersonTrackingFrame.source_sequence_id"
        )
    if frame.source_id != tracking_frame.source_id:
        raise ReIDInputError("ColorFrame.source_id must match tracking source_id")
    if frame.host_receive_time_s != tracking_frame.source_time_s:
        raise ReIDInputError("ColorFrame host receive time must match tracking source_time_s")
    if (frame.width, frame.height) != (
        tracking_frame.frame_width, tracking_frame.frame_height
    ):
        raise ReIDInputError("ColorFrame dimensions must match tracking frame dimensions")


def crop_person_bgr(
    image_bgr: np.ndarray,
    bbox_xyxy_px: tuple[float, float, float, float],
) -> np.ndarray:
    """Clip a source-frame xyxy box and reject empty crops; never enlarge it."""
    if not isinstance(image_bgr, np.ndarray):
        raise ReIDInputError("image_bgr must be a NumPy array")
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ReIDInputError("image_bgr must have shape H×W×3")
    if image_bgr.dtype != np.uint8:
        raise ReIDInputError("image_bgr must use uint8 BGR pixels")
    try:
        box = tuple(float(value) for value in bbox_xyxy_px)
    except (TypeError, ValueError) as error:
        raise ReIDInputError("bbox must contain four finite pixel values") from error
    if len(box) != 4 or not all(math.isfinite(value) for value in box):
        raise ReIDInputError("bbox must contain four finite pixel values")
    x1, y1, x2, y2 = box
    height, width = image_bgr.shape[:2]
    left = max(0, min(width, math.floor(x1)))
    top = max(0, min(height, math.floor(y1)))
    right = max(0, min(width, math.ceil(x2)))
    bottom = max(0, min(height, math.ceil(y2)))
    if right <= left or bottom <= top:
        raise ReIDInputError("bbox crop is empty after clipping to the source frame")
    crop = image_bgr[top:bottom, left:right]
    if crop.size == 0:
        raise ReIDInputError("bbox produced an empty crop")
    return crop


@dataclass(frozen=True)
class MasterAppearanceReference:
    """The single embedding created from an explicit Master selection."""

    bound_track_at_selection: int
    source_id: str
    source_sequence_id: int
    source_time_s: float
    embedding: AppearanceEmbedding
    sample_sequence_ids: tuple[int, ...] = ()


@dataclass(frozen=True)
class ReIDCandidateScore:
    """One candidate's evidence; similarity is a cosine score, not a probability."""

    source_id: str
    source_sequence_id: int
    source_time_s: float
    candidate_track_id: int
    bbox_xyxy_px: tuple[float, float, float, float]
    similarity: float
    cosine_distance: float
    rank: int
    preprocess_wall_time_s: float
    inference_wall_time_s: float
    total_embedding_wall_time_s: float
    master_state: MasterState
    retry_count: int = 0


@dataclass(frozen=True)
class ReIDCandidateRejection:
    candidate_track_id: int
    bbox_xyxy_px: tuple[float, float, float, float]
    reason: str


@dataclass(frozen=True)
class CandidateScoringFrame:
    source_id: str
    source_sequence_id: int
    source_time_s: float
    reference_track_id: int
    master_state: MasterState
    scores: tuple[ReIDCandidateScore, ...]
    rejected: tuple[ReIDCandidateRejection, ...] = ()

    @property
    def top1(self) -> ReIDCandidateScore | None:
        return self.scores[0] if self.scores else None

    @property
    def top2(self) -> ReIDCandidateScore | None:
        return self.scores[1] if len(self.scores) > 1 else None

    @property
    def top1_top2_margin(self) -> float | None:
        if self.top1 is None or self.top2 is None:
            return None
        return self.top1.similarity - self.top2.similarity


class MasterReIDEvidence:
    """Single-reference ReID scoring plus temporary pending-reference samples."""

    def __init__(self, reidentifier: PersonReIdentifier) -> None:
        self._reidentifier = reidentifier
        self._reference: MasterAppearanceReference | None = None
        self._lost_episode_started = False
        self._scored_track_ids: set[int] = set()
        self._candidate_scores_by_track_id: dict[int, ReIDCandidateScore] = {}
        self._pending_track_id: int | None = None
        self._pending_started_time_s: float | None = None
        self._pending_embeddings: list[AppearanceEmbedding] = []

    @property
    def reference(self) -> MasterAppearanceReference | None:
        return self._reference

    @property
    def backend_name(self) -> str:
        return self._reidentifier.backend_name

    @property
    def model_name(self) -> str:
        return self._reidentifier.model_name

    @property
    def pending_track_id(self) -> int | None:
        return self._pending_track_id

    @property
    def pending_sample_count(self) -> int:
        return len(self._pending_embeddings)

    @property
    def lost_episode_candidate_scores(self) -> tuple[ReIDCandidateScore, ...]:
        """Latest per-ID scores retained for the current LOST episode only."""
        return tuple(
            self._candidate_scores_by_track_id[track_id]
            for track_id in sorted(self._candidate_scores_by_track_id)
        )

    def create_reference(
        self,
        frame: ColorFrame,
        tracking_frame: PersonTrackingFrame,
        selected_track_id: int,
    ) -> MasterAppearanceReference:
        """Replace the one reference from the exact manually selected frame."""
        self.clear_reference()
        track = next(
            (item for item in tracking_frame.tracks
             if item.track_id == selected_track_id),
            None,
        )
        if track is None:
            raise ReIDInputError("selected track is not active in the source frame")
        embedding = self._reidentifier.embed_track(frame, tracking_frame, track)
        reference = MasterAppearanceReference(
            bound_track_at_selection=track.track_id,
            source_id=frame.source_id,
            source_sequence_id=frame.sequence_id,
            source_time_s=frame.host_receive_time_s,
            embedding=embedding,
            sample_sequence_ids=(frame.sequence_id,),
        )
        self._reference = reference
        return reference

    def begin_pending_reference(
        self,
        frame: ColorFrame,
        tracking_frame: PersonTrackingFrame,
        selected_track_id: int,
    ) -> None:
        """Start the fixed three-embedding, 0.3-second manual reference window."""
        _require_frame_alignment(frame, tracking_frame)
        if isinstance(selected_track_id, bool) or not isinstance(selected_track_id, int):
            raise ValueError("selected_track_id must be an integer")
        if not any(track.track_id == selected_track_id for track in tracking_frame.tracks):
            raise ReIDInputError("selected track is not active in the source frame")
        self.clear_reference()
        self._pending_track_id = selected_track_id
        self._pending_started_time_s = frame.host_receive_time_s

    def collect_pending_reference_sample(
        self,
        frame: ColorFrame,
        tracking_frame: PersonTrackingFrame,
    ) -> MasterAppearanceReference | None:
        """Collect one due, frame-aligned sample and finalize after 0.3 seconds."""
        _require_frame_alignment(frame, tracking_frame)
        if self._pending_track_id is None or self._pending_started_time_s is None:
            return None
        track = next(
            (item for item in tracking_frame.tracks
             if item.track_id == self._pending_track_id),
            None,
        )
        if track is None:
            return None

        sample_offsets_s = (0.0, 0.15, 0.30)
        sample_index = len(self._pending_embeddings)
        if sample_index >= len(sample_offsets_s):
            raise RuntimeError("pending Master reference already has three samples")
        elapsed_s = frame.host_receive_time_s - self._pending_started_time_s
        if elapsed_s < sample_offsets_s[sample_index]:
            return None

        try:
            embedding = self._reidentifier.embed_track(frame, tracking_frame, track)
        except ValueError as error:
            raise ReIDInputError(f"pending appearance embedding was invalid: {error}") from error
        self._pending_embeddings.append(embedding)
        if len(self._pending_embeddings) < 3:
            return None

        samples = tuple(self._pending_embeddings)
        if any(
            sample.source_id != frame.source_id
            or sample.track_id != self._pending_track_id
            or sample.dimension != samples[0].dimension
            for sample in samples
        ):
            raise ReIDInputError("pending reference samples do not share one source and track")
        mean_vector = np.mean(
            np.stack([sample.vector for sample in samples]).astype(np.float64), axis=0
        )
        mean_norm = float(np.linalg.norm(mean_vector))
        if not math.isfinite(mean_norm) or mean_norm <= 1e-12:
            self._pending_embeddings.clear()
            self._pending_started_time_s = frame.host_receive_time_s
            raise ReIDInputError("three pending embeddings have a zero-norm average")
        normalized_mean = np.asarray(mean_vector / mean_norm, dtype=np.float32)
        combined_embedding = AppearanceEmbedding(
            source_id=frame.source_id,
            source_sequence_id=frame.sequence_id,
            track_id=self._pending_track_id,
            vector=normalized_mean,
            backend_name=samples[-1].backend_name,
            model_name=samples[-1].model_name,
            preprocess_wall_time_s=sum(item.preprocess_wall_time_s for item in samples),
            inference_wall_time_s=sum(item.inference_wall_time_s for item in samples),
            total_wall_time_s=sum(item.total_wall_time_s for item in samples),
        )
        reference = MasterAppearanceReference(
            bound_track_at_selection=self._pending_track_id,
            source_id=frame.source_id,
            source_sequence_id=frame.sequence_id,
            source_time_s=frame.host_receive_time_s,
            embedding=combined_embedding,
            sample_sequence_ids=tuple(item.source_sequence_id for item in samples),
        )
        self._reference = reference
        self._clear_pending_reference()
        return reference

    def cancel_pending_reference(self) -> None:
        """Discard only the in-progress three-sample selection window."""
        self._clear_pending_reference()

    def _clear_pending_reference(self) -> None:
        self._pending_track_id = None
        self._pending_started_time_s = None
        self._pending_embeddings.clear()

    def clear_reference(self) -> None:
        self._reference = None
        self._lost_episode_started = False
        self._scored_track_ids.clear()
        self._candidate_scores_by_track_id.clear()
        self._clear_pending_reference()

    def reset(self) -> None:
        """Alias for an explicit Master clear or tracker/session reset."""
        self.clear_reference()

    def score_candidates(
        self,
        frame: ColorFrame,
        tracking_frame: PersonTrackingFrame,
        master_frame: MasterTrackingFrame,
        *,
        stable_candidate_track_ids: Iterable[int],
        accept_threshold: float = 0.68,
        retry_threshold: float = 0.50,
        retry_interval_s: float = 0.20,
    ) -> CandidateScoringFrame | None:
        """Score stable LOST candidates and retry only provisional matches."""
        _require_frame_alignment(frame, tracking_frame)
        if not isinstance(master_frame, MasterTrackingFrame):
            raise TypeError("master_frame must be a MasterTrackingFrame")
        if (master_frame.source_id != frame.source_id
                or master_frame.source_sequence_id != frame.sequence_id):
            raise ReIDInputError("MasterTrackingFrame must match the exact color frame")
        if master_frame.source_time_s != frame.host_receive_time_s:
            raise ReIDInputError("MasterTrackingFrame source time must match the color frame")

        if master_frame.state is not MasterState.LOST:
            self._lost_episode_started = False
            self._scored_track_ids.clear()
            self._candidate_scores_by_track_id.clear()
            return None
        reference = self._reference
        if reference is None:
            return None
        if reference.source_id != master_frame.source_id:
            return None
        if not self._lost_episode_started:
            self._lost_episode_started = True
            self._scored_track_ids.clear()
            self._candidate_scores_by_track_id.clear()

        accept_threshold = float(accept_threshold)
        retry_threshold = float(retry_threshold)
        retry_interval_s = float(retry_interval_s)
        if (not math.isfinite(accept_threshold)
                or not retry_threshold <= accept_threshold <= 1.0
                or retry_threshold < -1.0):
            raise ValueError("ReID thresholds must satisfy -1 <= retry <= accept <= 1")
        if not math.isfinite(retry_interval_s) or retry_interval_s <= 0.0:
            raise ValueError("retry_interval_s must be finite and positive")

        active_tracks = tuple(
            track for track in tracking_frame.tracks
            if track.track_id != master_frame.bound_track_id
        )
        active_ids = {track.track_id for track in active_tracks}
        disappeared_ids = self._scored_track_ids - active_ids
        for track_id in disappeared_ids:
            self._scored_track_ids.discard(track_id)
            self._candidate_scores_by_track_id.pop(track_id, None)

        stable_ids = {int(track_id) for track_id in stable_candidate_track_ids}
        candidates: list[TrackedPerson] = []
        for track in active_tracks:
            if track.track_id not in stable_ids:
                continue
            previous_score = self._candidate_scores_by_track_id.get(track.track_id)
            if previous_score is None:
                candidates.append(track)
                continue
            if (previous_score.similarity < retry_threshold
                    or previous_score.similarity >= accept_threshold):
                continue
            if (frame.host_receive_time_s - previous_score.source_time_s
                    >= retry_interval_s):
                candidates.append(track)
        if not candidates:
            return CandidateScoringFrame(
                source_id=frame.source_id,
                source_sequence_id=frame.sequence_id,
                source_time_s=frame.host_receive_time_s,
                reference_track_id=reference.bound_track_at_selection,
                master_state=master_frame.state,
                scores=(),
            )

        provisional: list[tuple[TrackedPerson, AppearanceEmbedding, float, int]] = []
        rejected: list[ReIDCandidateRejection] = []
        for track in candidates:
            try:
                embedding = self._reidentifier.embed_track(frame, tracking_frame, track)
            except ReIDInputError as error:
                rejected.append(ReIDCandidateRejection(
                    track.track_id, track.bbox_xyxy_px, str(error)
                ))
                continue
            if embedding.dimension != reference.embedding.dimension:
                raise ValueError(
                    "candidate embedding dimension does not match the Master reference"
                )
            similarity = float(np.dot(reference.embedding.vector, embedding.vector))
            similarity = max(-1.0, min(1.0, similarity))
            previous_score = self._candidate_scores_by_track_id.get(track.track_id)
            retry_count = 0 if previous_score is None else previous_score.retry_count + 1
            provisional.append((track, embedding, similarity, retry_count))

        provisional.sort(key=lambda row: (-row[2], row[0].track_id))
        scores = tuple(
            ReIDCandidateScore(
                source_id=frame.source_id,
                source_sequence_id=frame.sequence_id,
                source_time_s=frame.host_receive_time_s,
                candidate_track_id=track.track_id,
                bbox_xyxy_px=track.bbox_xyxy_px,
                similarity=similarity,
                cosine_distance=1.0 - similarity,
                rank=rank,
                preprocess_wall_time_s=embedding.preprocess_wall_time_s,
                inference_wall_time_s=embedding.inference_wall_time_s,
                total_embedding_wall_time_s=embedding.total_wall_time_s,
                master_state=master_frame.state,
                retry_count=retry_count,
            )
            for rank, (track, embedding, similarity, retry_count)
            in enumerate(provisional, start=1)
        )
        self._scored_track_ids.update(item.track_id for item, _, _, _ in provisional)
        self._candidate_scores_by_track_id.update({
            item.candidate_track_id: item for item in scores
        })
        return CandidateScoringFrame(
            source_id=frame.source_id,
            source_sequence_id=frame.sequence_id,
            source_time_s=frame.host_receive_time_s,
            reference_track_id=reference.bound_track_at_selection,
            master_state=master_frame.state,
            scores=scores,
            rejected=tuple(rejected),
        )


def default_osnet_weights_path() -> Path:
    """Return a cache inside the active venv, never a versioned repository path."""
    return (
        Path(sysconfig.get_paths()["purelib"]).parent.parent
        / "reid_models" / OSNET_WEIGHT_FILENAME
    )


def download_default_osnet_weights(destination: Path | None = None) -> Path:
    """Fetch only the published OSNet x0.25 ReID checkpoint over HTTPS."""
    path = default_osnet_weights_path() if destination is None else Path(destination)
    if path.is_file():
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest == OSNET_WEIGHT_SHA256:
            return path
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".download")
    try:
        with urlopen(OSNET_WEIGHT_URL, timeout=60) as response, temporary.open("wb") as output:
            while True:
                chunk = response.read(1024 * 1024)
                if not chunk:
                    break
                output.write(chunk)
        digest = hashlib.sha256(temporary.read_bytes()).hexdigest()
        if digest != OSNET_WEIGHT_SHA256:
            raise RuntimeError(
                "downloaded OSNet checkpoint does not match the pinned upstream SHA256"
            )
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return path


class TorchReIDOSNetBackend:
    """CPU TorchReID OSNet x0.25 adapter with all tensors kept private."""

    backend_name = "torchreid.pytorch.cpu"
    model_name = OSNET_MODEL_NAME
    input_width = 128
    input_height = 256

    def __init__(self, weights_path: str | Path) -> None:
        checkpoint = Path(weights_path)
        if not checkpoint.is_file():
            raise FileNotFoundError(f"OSNet ReID checkpoint not found: {checkpoint}")
        digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
        if digest != OSNET_WEIGHT_SHA256:
            raise ValueError(
                "OSNet checkpoint does not match the pinned MSMT17 baseline "
                f"SHA256 {OSNET_WEIGHT_SHA256}"
            )
        try:
            import torch
        except ImportError as error:
            raise RuntimeError(
                "OSNet inference requires PyTorch in the project .venv."
            ) from error
        source_root = (
            Path(sysconfig.get_paths()["purelib"])
            / "_companionbot_torchreid_source"
        )
        source_file = source_root / "torchreid" / "models" / "osnet.py"
        revision_file = source_root / "COMPANIONBOT_SOURCE_COMMIT"
        if (not source_file.is_file() or not revision_file.is_file()
                or revision_file.read_text(encoding="ascii").strip()
                != TORCHREID_SOURCE_COMMIT):
            raise FileNotFoundError(
                "official TorchReID OSNet source is missing from this .venv; "
                "run `python scripts/install_stage7_5_osnet.py` with the project "
                "interpreter first"
            )
        spec = importlib.util.spec_from_file_location(
            "_companionbot_torchreid_osnet", source_file
        )
        if spec is None or spec.loader is None:
            raise RuntimeError(f"could not load upstream OSNet source: {source_file}")
        osnet_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(osnet_module)
        self._torch = torch
        self._model = osnet_module.osnet_x0_25(
            num_classes=1, pretrained=False
        )
        try:
            checkpoint_state = torch.load(
                str(checkpoint), map_location="cpu", weights_only=True
            )
        except TypeError:
            # Older supported PyTorch builds predate weights_only. The exact
            # upstream artifact SHA256 has already been verified above.
            checkpoint_state = torch.load(
                str(checkpoint), map_location="cpu", weights_only=False
            )
        if isinstance(checkpoint_state, dict) and "state_dict" in checkpoint_state:
            checkpoint_state = checkpoint_state["state_dict"]
        if not isinstance(checkpoint_state, dict):
            raise RuntimeError("OSNet checkpoint does not contain a state dictionary")
        model_state = self._model.state_dict()
        matched_state = {}
        for key, value in checkpoint_state.items():
            if key.startswith("module."):
                key = key[7:]
            if (key in model_state and isinstance(value, torch.Tensor)
                    and model_state[key].shape == value.shape):
                matched_state[key] = value
        feature_keys = [key for key in model_state if not key.startswith("classifier.")]
        matched_feature_keys = [key for key in feature_keys if key in matched_state]
        if len(matched_feature_keys) < 0.95 * len(feature_keys):
            raise RuntimeError(
                "OSNet checkpoint does not match the upstream x0.25 architecture "
                f"({len(matched_feature_keys)}/{len(feature_keys)} feature tensors matched)"
            )
        self._model.load_state_dict(matched_state, strict=False)
        self._model.eval().to(torch.device("cpu"))
        self._mean = np.asarray((0.485, 0.456, 0.406), dtype=np.float32)[None, :, None, None]
        self._std = np.asarray((0.229, 0.224, 0.225), dtype=np.float32)[None, :, None, None]

    def embed_crop_bgr(self, crop_bgr: np.ndarray) -> BackendEmbedding:
        torch = self._torch
        preprocess_start_ns = time.perf_counter_ns()
        rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(
            rgb, (self.input_width, self.input_height), interpolation=cv2.INTER_LINEAR
        )
        image = np.asarray(resized, dtype=np.float32) / np.float32(255.0)
        image = image.transpose(2, 0, 1)[None, :, :, :]
        image = (image - self._mean) / self._std
        tensor = torch.from_numpy(np.ascontiguousarray(image))
        preprocess_end_ns = time.perf_counter_ns()

        inference_start_ns = time.perf_counter_ns()
        with torch.inference_mode():
            result = self._model(tensor)
        inference_end_ns = time.perf_counter_ns()
        if not isinstance(result, torch.Tensor) or result.ndim != 2 or result.shape[0] != 1:
            raise RuntimeError("TorchReID OSNet returned an unexpected feature shape")
        vector = result[0].detach().cpu().numpy().astype(np.float32, copy=True)
        return BackendEmbedding(
            vector=vector,
            preprocess_wall_time_s=(preprocess_end_ns - preprocess_start_ns) * 1e-9,
            inference_wall_time_s=(inference_end_ns - inference_start_ns) * 1e-9,
        )
