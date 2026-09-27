"""Camera optical coordinates → body → pitch-leveled robot observation.

Body/leveled axes: +X forward, +Y left, +Z up. Both origins are the wheel
axle midpoint. Positive pitch is right-handed about +Y (nose down).
The leveled frame retains robot heading; it is not a world/map frame.
"""

from dataclasses import dataclass
import math
import time
from typing import Protocol

import numpy as np
from scipy.spatial.transform import Rotation

from perception.camera import ColorFrame


SOURCE_TIME_SEMANTICS = "host_read_complete"
CLOCK_DOMAIN = "host_perf_counter"


@dataclass(frozen=True)
class CameraExtrinsic:
    """P_body = R_body_camera @ P_camera + t_body_camera_m."""

    rotation_body_camera: np.ndarray
    translation_body_camera_m: np.ndarray
    provisional: bool

    def __post_init__(self):
        if not isinstance(self.provisional, bool):
            raise ValueError("extrinsic provisional flag must be boolean")
        rotation = np.array(self.rotation_body_camera, dtype=float, copy=True)
        translation = np.array(self.translation_body_camera_m, dtype=float, copy=True)
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
                or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8)):
            raise ValueError("camera rotation must be a proper orthonormal 3x3 matrix")
        if translation.shape != (3,) or not np.isfinite(translation).all():
            raise ValueError("camera translation must be a finite 3-vector in metres")
        rotation.setflags(write=False)
        translation.setflags(write=False)
        object.__setattr__(self, "rotation_body_camera", rotation)
        object.__setattr__(self, "translation_body_camera_m", translation)


@dataclass(frozen=True)
class PitchSample:
    """Pitch for a specific source request, with its actual alignment time.

    source_time_s echoes the ColorFrame receipt timestamp. A measured provider
    may map that frame's exposure into the host clock and explicitly report the
    earlier pitch_time_s and capture_time_mapped_to_host basis.
    """
    source_time_s: float
    theta_rad: float
    provider_id: str
    provisional: bool
    time_semantics: str = SOURCE_TIME_SEMANTICS
    clock_domain: str = CLOCK_DOMAIN
    pitch_time_s: float | None = None
    alignment_basis: str = "host_read_complete"


class PitchProvider(Protocol):
    """Return pitch at this source time, or None without time coverage.

    A future measured provider owns frame-time/clock mapping and interpolation. It
    must not substitute the newest pitch for the requested source timestamp.
    """

    def at_source_time(self, source_time_s: float) -> PitchSample | None: ...


@dataclass(frozen=True)
class ConstantPitchProvider:
    """Explicit simulation input, never advertised as measured IMU feedback."""

    theta_rad: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.theta_rad):
            raise ValueError("simulated pitch must be finite")

    def at_source_time(self, source_time_s: float) -> PitchSample:
        return PitchSample(source_time_s, self.theta_rad, "constant_simulated", True)


@dataclass(frozen=True)
class RobotRelativeObservation:
    source_id: str
    source_sequence_id: int
    source_time_s: float
    width: int
    height: int
    master_track_id: int
    x_forward_m: float
    y_left_m: float
    z_up_m: float
    pitch_rad: float
    pitch_provider: str
    extrinsic_provisional: bool
    pitch_provisional: bool
    depth_convention: str
    result_ready_time_s: float
    pitch_time_s: float
    pitch_alignment_basis: str
    time_semantics: str = SOURCE_TIME_SEMANTICS
    clock_domain: str = CLOCK_DOMAIN
    frame_id: str = "robot_leveled_axle"
    depth_convention_provisional: bool = True


def camera_to_robot_observation(
    point_camera_m, frame: ColorFrame, master_track_id: int,
    extrinsic: CameraExtrinsic, pitch_provider: PitchProvider, *, depth_convention: str,
) -> RobotRelativeObservation | None:
    """Transform a source-aligned point; missing pitch coverage is unavailable."""
    point = np.asarray(point_camera_m, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError("camera point must be a finite 3-vector in metres")
    if depth_convention not in ("axial_z", "euclidean_range"):
        raise ValueError("depth convention must be explicit")
    if not isinstance(master_track_id, int) or isinstance(master_track_id, bool) or master_track_id < 1:
        raise ValueError("Master track ID must be positive")
    pitch = pitch_provider.at_source_time(frame.host_receive_time_s)
    if pitch is None:
        return None
    if (pitch.time_semantics != SOURCE_TIME_SEMANTICS or pitch.clock_domain != CLOCK_DOMAIN
            or not math.isfinite(pitch.source_time_s)
            or not math.isfinite(pitch.theta_rad)
            or pitch.source_time_s != frame.host_receive_time_s):
        raise ValueError("pitch must align to the exact source time and clock semantics")
    pitch_time = pitch.source_time_s if pitch.pitch_time_s is None else pitch.pitch_time_s
    if (not math.isfinite(pitch_time)
            or pitch.alignment_basis not in ("host_read_complete", "capture_time_mapped_to_host")
            or (pitch.alignment_basis == "host_read_complete" and pitch_time != pitch.source_time_s)
            or (pitch.alignment_basis == "capture_time_mapped_to_host"
                and (pitch.pitch_time_s is None or pitch_time > pitch.source_time_s))):
        raise ValueError("pitch alignment time/basis must be explicit and consistent")
    point_body = extrinsic.rotation_body_camera @ point + extrinsic.translation_body_camera_m
    # Rotate the translated point around the body/leveled common axle origin.
    point_leveled = Rotation.from_rotvec([0.0, pitch.theta_rad, 0.0]).apply(point_body)
    return RobotRelativeObservation(
        frame.source_id, frame.sequence_id, frame.host_receive_time_s,
        frame.width, frame.height, master_track_id,
        *map(float, point_leveled), pitch.theta_rad, pitch.provider_id,
        extrinsic.provisional, pitch.provisional, depth_convention, time.perf_counter(),
        pitch_time, pitch.alignment_basis,
    )
