# Stage 7.7 final_live_smoke

- Mode: full; source: camera:1
- Actual source: 1280x720 BGR uint8; 27.09 Hz; 300 frames
- Calibration: {'path': 'D:\\project\\CompanionBot\\models\\minisegway\\stage7\\stage7_7\\config\\c920e_camera_params_1280x720.npz', 'resolution': [1280, 720], 'status': 'loaded_and_exact_source_resolution_validated'}
- Depth: {'total_results': 125, 'steady_results': 123, 'warmup_excluded': 2, 'effective_hz': 14.03093323957769, 'inference_adapter_wall_ms_median': 68.59059992711991, 'inference_adapter_wall_ms_p95': 85.0436199689284, 'result_age_ms_median': 91.54569997917861, 'result_age_ms_p95': 129.14439996238798, 'first_source_sequence_id': 0, 'last_source_sequence_id': 299}
- Detector/tracker: {'total_results': 239, 'steady_results': 237, 'warmup_excluded': 2, 'effective_hz': 27.22529652757876, 'detector_wall_ms_median': 27.484800084494054, 'detector_wall_ms_p95': 33.96053994074464, 'tracker_wall_ms_median': 0.5083000287413597, 'tracker_wall_ms_p95': 0.7755199680104852, 'result_age_ms_median': 31.601800001226366, 'result_age_ms_p95': 49.75253995507954, 'first_source_sequence_id': 0, 'last_source_sequence_id': 299}
- ReID: {'model': 'osnet_x0_25_msmt17', 'event_count': 13, 'reference_count': 1, 'reacquire_count': 1, 'accept_threshold': 0.68, 'retry_threshold': 0.5, 'threshold_status': 'both are provisional engineering thresholds; no different-person negative calibration has been performed'}
- Robot observations: 104; mount/pitch/depth convention provisional; Stage 6 disconnected.
- Detector comparison: None
- Source time is host read-complete, not exposure time. Depth values are single-image model estimates in metres, not range-sensor ground truth.
- Master torso median uses only exact same-source-frame bbox/depth pairs; camera XYZ assumes axial Z provisionally (checkpoint convention unconfirmed).
- Metrics: D:\project\CompanionBot\models\minisegway\stage7\results\final_live_smoke\final_live_smoke_metrics.json
