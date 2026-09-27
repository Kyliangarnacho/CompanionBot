# Stage 7.7 final_verified

- Mode: full; source: camera:1
- Actual source: 1280x720 BGR uint8; 25.18 Hz; 180 frames
- Calibration: {'path': 'D:\\project\\CompanionBot\\models\\minisegway\\stage7\\stage7_7\\config\\c920e_camera_params_1280x720.npz', 'resolution': [1280, 720], 'status': 'loaded_and_exact_source_resolution_validated'}
- Depth: {'total_results': 56, 'steady_results': 54, 'warmup_excluded': 2, 'effective_hz': 12.039091064877905, 'inference_adapter_wall_ms_median': 82.18924998072907, 'inference_adapter_wall_ms_p95': 86.74611002206802, 'result_age_ms_median': 100.15520005254075, 'result_age_ms_p95': 132.26031997473905, 'first_source_sequence_id': 0, 'last_source_sequence_id': 179}
- Detector/tracker: {'total_results': 116, 'steady_results': 114, 'warmup_excluded': 2, 'effective_hz': 25.601639664598242, 'detector_wall_ms_median': 30.860700004268438, 'detector_wall_ms_p95': 36.016275064321235, 'tracker_wall_ms_median': 0.5509000620804727, 'tracker_wall_ms_p95': 0.7731449499260634, 'result_age_ms_median': 34.71259999787435, 'result_age_ms_p95': 56.22219503275119, 'first_source_sequence_id': 0, 'last_source_sequence_id': 179}
- ReID: {'model': 'osnet_x0_25_msmt17', 'event_count': 3, 'reference_count': 1, 'reacquire_count': 0, 'accept_threshold': 0.68, 'retry_threshold': 0.5, 'threshold_status': 'both are provisional engineering thresholds; no different-person negative calibration has been performed'}
- Robot observations: 54; see per-observation provisional flags; Stage 6 disconnected.
- Detector comparison: None
- Source time is host read-complete, not exposure time. Depth values are single-image model estimates in metres, not range-sensor ground truth.
- Master torso median uses only exact same-source-frame bbox/depth pairs; camera XYZ assumes axial_z provisionally (checkpoint convention unconfirmed).
- Metrics: D:\project\CompanionBot\models\minisegway\stage7\results\final_verified\final_verified_metrics.json
