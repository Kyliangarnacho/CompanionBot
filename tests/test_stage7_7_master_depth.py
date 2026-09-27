"""Same-source-frame Master depth and calibrated camera-ray regression tests."""

import numpy as np
import pytest

from perception.camera import ColorFrame
from perception.camera_calibration import CameraCalibration
from perception.depth import DepthFrame
from perception.master_depth import (
    back_project_camera_point, representative_master_depth, torso_roi_xyxy,
)
from perception.master_selection import MasterManager, MasterSelector
from perception.person_tracker import (
    PersonTrackingDiagnostics, PersonTrackingFrame, TrackedPerson,
)
from scripts.demo_yolo26n_depth import (
    _SameFramePairer, _paired_preview, _select_preview_track,
)


def _source(seq=1, width=100, height=100, source_id="camera:1"):
    return ColorFrame(np.zeros((height, width, 3), dtype=np.uint8), seq,
                      source_id, width, height, float(seq))


def _tracking(frame, track_id=4):
    track = TrackedPerson(track_id, (40, 20, 60, 80), 0.9, 0, "person")
    return PersonTrackingFrame(
        source_id=frame.source_id, source_sequence_id=frame.sequence_id,
        source_time_s=frame.host_receive_time_s,
        frame_width=frame.width, frame_height=frame.height,
        detection_count=1, tracks=(track,),
        diagnostics=PersonTrackingDiagnostics(
            update_index=frame.sequence_id, active_track_count=1,
            created_track_ids=(track_id,), newly_lost_track_ids=(),
            newly_removed_track_ids=(), source_sequence_gap=0,
            tracker_update_wall_time_s=0.001),
        tracking_start_time_s=float(frame.sequence_id),
        tracking_end_time_s=float(frame.sequence_id),
        result_ready_time_s=float(frame.sequence_id),
    )


def _depth(frame, values=None):
    if values is None:
        values = np.full((frame.height, frame.width), np.nan, dtype=np.float32)
    return DepthFrame(frame.source_id, frame.sequence_id,
                      frame.host_receive_time_s, frame.width, frame.height,
                      values, "test", "test-depth")


def _locked(frame):
    tracking = _tracking(frame)
    manager = MasterManager()
    manager.update(tracking)
    manager.lock(tracking, 4)
    return tracking, manager.result(tracking)


def test_torso_median_rejects_invalid_pixels_and_single_pixel_outlier():
    frame = _source()
    _, master = _locked(frame)
    values = np.full((100, 100), np.nan, dtype=np.float32)
    roi = torso_roi_xyxy(master.master_track.bbox_xyxy_px, 100, 100)
    assert roi == (46, 38, 54, 62)
    values[38:62, 46:54] = 2.0
    values[39, 47] = 50.0
    values[40, 48] = np.nan
    sample = representative_master_depth(master, _depth(frame, values))
    assert sample.depth_m == 2.0
    assert sample.anchor_uv_px == (50.0, 50.0)
    assert sample.valid_pixel_count == 191


def test_insufficient_valid_depth_and_invisible_master_are_unavailable():
    frame = _source()
    tracking, master = _locked(frame)
    values = np.full((100, 100), np.nan, dtype=np.float32)
    values[38, 46:54] = 2.0
    values[39, 46] = 2.0
    assert representative_master_depth(master, _depth(frame, values)) is None
    manager = MasterManager()
    manager.update(tracking)
    assert representative_master_depth(manager.result(tracking), _depth(frame, values)) is None


@pytest.mark.parametrize("changed", ["sequence", "source", "resolution", "time"])
def test_depth_refuses_bbox_from_another_source_frame(changed):
    frame = _source()
    _, master = _locked(frame)
    other = {
        "sequence": _source(seq=2),
        "source": _source(source_id="camera:other"),
        "resolution": _source(width=101),
        "time": ColorFrame(np.zeros((100, 100, 3), dtype=np.uint8), 1,
                           "camera:1", 100, 100, 1.1),
    }[changed]
    with pytest.raises(ValueError, match="same source frame"):
        representative_master_depth(master, _depth(other))


def test_calibrated_backprojection_axial_and_ray_range_have_distinct_formulas():
    K = np.array([[100.0, 0, 50.0], [0, 100.0, 50.0], [0, 0, 1.0]])
    calibration = CameraCalibration(K, np.zeros(5), 100, 100, "test", "npz")
    axial = back_project_camera_point((60, 40), 2.0, calibration,
                                       depth_convention="axial_z")
    distance = back_project_camera_point((60, 40), 2.0, calibration,
                                          depth_convention="euclidean_range")
    np.testing.assert_allclose(axial, [0.2, -0.2, 2.0])
    np.testing.assert_allclose(distance, 2 * np.array([0.1, -0.1, 1]) /
                               np.linalg.norm([0.1, -0.1, 1]))
    assert np.linalg.norm(distance) == pytest.approx(2.0)
    with pytest.raises(ValueError, match="depth_convention"):
        back_project_camera_point((60, 40), 2.0, calibration,
                                  depth_convention="unknown")


def test_bounded_pairer_matches_only_identical_source_frame_in_either_order():
    frame = _source()
    tracking, master = _locked(frame)
    depth = _depth(frame)
    pairer = _SameFramePairer(capacity=2)
    assert pairer.add_depth(frame, depth, {"result_age_ms": 1}) is None
    pair = pairer.add_tracking(frame, tracking, master)
    assert pair is not None and pair.color is frame and pair.master is master
    later = _source(seq=2)
    later_tracking, later_master = _locked(later)
    assert pairer.add_tracking(later, later_tracking, later_master) is None
    assert pairer.add_depth(later, _depth(later), {}) is not None
    with pytest.raises(ValueError, match="source metadata"):
        pairer.add_depth(frame, _depth(_source(seq=3)), {})


def test_pairer_does_not_cross_sources_or_keep_evicted_old_depth():
    first = _source(1)
    other_source = _source(1, source_id="camera:other")
    pairer = _SameFramePairer(capacity=2)
    pairer.add_depth(first, _depth(first), {})
    other_tracking, other_master = _locked(other_source)
    assert pairer.add_tracking(other_source, other_tracking, other_master) is None
    for seq in (2, 3):
        newer = _source(seq)
        pairer.add_depth(newer, _depth(newer), {})
    first_tracking, first_master = _locked(first)
    assert pairer.add_tracking(first, first_tracking, first_master) is None


def test_same_frame_preview_shows_master_depth_and_camera_point_without_gui():
    frame = _source()
    tracking, master = _locked(frame)
    depth_values = np.ones((100, 100), dtype=np.float32) * 2
    pairer = _SameFramePairer()
    pairer.add_tracking(frame, tracking, master)
    paired = pairer.add_depth(frame, _depth(frame, depth_values), {
        "inference_adapter_wall_ms": 82.0, "result_age_ms": 88.0,
    })
    K = np.array([[100.0, 0, 50.0], [0, 100.0, 50.0], [0, 0, 1.0]])
    calibration = CameraCalibration(K, np.zeros(5), 100, 100, "test", "npz")
    image, sample, point = _paired_preview(
        paired, calibration, 11.0, 20.0)
    assert image.shape == (100, 100, 3)
    assert sample.depth_m == 2.0
    np.testing.assert_allclose(point, [0, 0, 2.0])


def test_click_on_displayed_old_frame_only_binds_id_still_visible_now():
    shown = _source(1)
    now = _source(2)
    shown_tracking = _tracking(shown)
    current_tracking = _tracking(now)
    manager = MasterManager()
    manager.update(shown_tracking)
    manager.update(current_tracking)
    selected = _select_preview_track((50, 50), shown_tracking, current_tracking,
                                     MasterSelector(), manager)
    assert selected == 4 and manager.bound_track_id == 4
    assert _select_preview_track((120, 50), shown_tracking, current_tracking,
                                 MasterSelector(), manager) is None
