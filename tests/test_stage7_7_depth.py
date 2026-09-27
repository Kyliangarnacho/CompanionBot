"""Stage 7.7 source fanout keeps channel inputs independent."""

import numpy as np
import pytest
from types import SimpleNamespace

from perception.camera import ColorFrame
from perception.camera_calibration import CameraCalibration
from perception.depth import DepthFrame
from perception.detection_stream import LatestFrameSlot
from perception.master_selection import MasterManager, MasterSelector
from perception.person_detector import PersonDetector, PersonDetectorConfig
from perception.person_reid import crop_person_bgr
from perception.person_tracker import (
    PersonTrackingDiagnostics, PersonTrackingFrame, TrackedPerson,
)
from scripts.demo_yolo26n_master_lock import _draw_preview, _process_click
from scripts.demo_yolo26n_depth import _FrameFanout, _depth_preview_lines


def _frame(seq: int, width: int = 1280, height: int = 720) -> ColorFrame:
    return ColorFrame(np.zeros((height, width, 3), dtype=np.uint8), seq,
                      "camera:1", width, height, float(seq))


def test_fanout_gives_each_slow_channel_the_latest_source_frame():
    depth_slot, detector_slot = LatestFrameSlot(), LatestFrameSlot()
    fanout = _FrameFanout((depth_slot, detector_slot))
    fanout.put(_frame(1))
    fanout.put(_frame(2))
    assert depth_slot.get(timeout_s=0).sequence_id == 2
    assert detector_slot.get(timeout_s=0).sequence_id == 2
    assert depth_slot.overwritten_count == detector_slot.overwritten_count == 1


def test_fanout_refuses_frames_outside_calibration_resolution():
    calibration = CameraCalibration(np.eye(3), np.zeros(5), 1280, 720,
                                    "test.npz", "npz")
    slot = LatestFrameSlot()
    fanout = _FrameFanout((slot,), calibration)
    with pytest.raises(ValueError, match="resolution"):
        fanout.put(_frame(3, 640, 480))
    assert slot.pending_count == 0


def test_depth_preview_uses_result_ready_age_instead_of_display_staleness():
    depth = DepthFrame("camera:1", 1012, 100.0, 2, 1,
                       np.ones((1, 2), dtype=np.float32),
                       "ultralytics.depth", "yolo26n-depth.pt")
    row = {"inference_adapter_wall_ms": 82.0, "result_age_ms": 88.0}

    lines = _depth_preview_lines(depth, row, 11.0)

    assert lines == ["Depth seq=1012 2x1 meters",
                     "11.00 Hz  infer=82ms  age=88ms"]


def test_1280_source_pixels_reach_detector_master_crop_and_preview_unchanged():
    frame = _frame(5)
    box = (1100.0, 500.0, 1250.0, 700.0)

    class _DetectorModel:
        names = {0: "person"}

        def predict(self, **kwargs):
            assert kwargs["source"] is frame.bgr
            assert kwargs["imgsz"] == 640
            boxes = SimpleNamespace(
                xyxy=np.asarray([box], dtype=np.float32),
                conf=np.asarray([0.9], dtype=np.float32),
                cls=np.asarray([0], dtype=np.float32),
            )
            return [SimpleNamespace(orig_shape=(720, 1280), boxes=boxes)]

    detection = PersonDetector(PersonDetectorConfig(imgsz=640),
                               model=_DetectorModel()).process(frame)[0]
    assert detection.bbox_xyxy_px == box
    assert crop_person_bgr(frame.bgr, detection.bbox_xyxy_px).shape == (200, 150, 3)

    track = TrackedPerson(4, box, 0.9, 0, "person")
    tracking = PersonTrackingFrame(
        source_id=frame.source_id, source_sequence_id=frame.sequence_id,
        source_time_s=frame.host_receive_time_s,
        frame_width=frame.width, frame_height=frame.height,
        detection_count=1, tracks=(track,),
        diagnostics=PersonTrackingDiagnostics(
            update_index=1, active_track_count=1,
            created_track_ids=(4,), newly_lost_track_ids=(),
            newly_removed_track_ids=(), source_sequence_gap=0,
            tracker_update_wall_time_s=0.001),
        tracking_start_time_s=5.0, tracking_end_time_s=5.0,
        result_ready_time_s=5.0,
    )
    manager = MasterManager()
    manager.update(tracking)
    selection, event = _process_click((1200, 650), tracking,
                                      MasterSelector(), manager)
    assert selection.selected_track_id == 4
    assert event.new_track_id == 4
    preview = _draw_preview(frame, tracking, manager.result(tracking),
                            None, "")
    assert preview.shape == frame.bgr.shape
