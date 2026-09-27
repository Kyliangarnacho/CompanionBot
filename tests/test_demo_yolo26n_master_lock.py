"""Hardware-free tests for the Stage 7.4 preview input boundary."""

from __future__ import annotations

import queue
from types import SimpleNamespace

import cv2

from scripts.demo_yolo26n_master_lock import (
    REID_ACCEPT_THRESHOLD,
    REID_RETRY_THRESHOLD,
    _format_reid_summary,
    _mouse_callback,
)


def test_mouse_callback_only_enqueues_left_click_pixel_coordinates():
    clicks: queue.SimpleQueue = queue.SimpleQueue()

    _mouse_callback(cv2.EVENT_MOUSEMOVE, 11, 19, 0, clicks)
    try:
        clicks.get_nowait()
        raise AssertionError("mouse movement should not enqueue a selection")
    except queue.Empty:
        pass

    _mouse_callback(cv2.EVENT_LBUTTONDOWN, 37, 52, 0, clicks)
    assert clicks.get_nowait() == (37, 52)


def test_reid_run_summary_reports_current_accept_and_retry_thresholds(tmp_path):
    evidence = SimpleNamespace(model_name="OSNet", backend_name="test")

    summary = _format_reid_summary(True, evidence, tmp_path / "reid.jsonl")

    assert f"accept threshold: `{REID_ACCEPT_THRESHOLD}`" in summary
    assert f"retry threshold: `{REID_RETRY_THRESHOLD}`" in summary
    assert "provisional engineering thresholds" in summary
