# Stage 7.7 full

- Mode: full; source: camera:1
- Actual source: 1280x720 BGR uint8; 29.39 Hz; 5134 frames
- Calibration: {'path': 'D:\\project\\CompanionBot\\models\\minisegway\\stage7\\stage7_7\\config\\c920e_camera_params_1280x720.npz', 'resolution': [1280, 720], 'status': 'loaded_and_exact_source_resolution_validated'}
- Depth: {'total_results': 2426, 'steady_results': 2424, 'warmup_excluded': 2, 'effective_hz': 14.059604723460893, 'inference_adapter_wall_ms_median': 67.00589996762574, 'inference_adapter_wall_ms_p95': 84.24303003703244, 'result_age_ms_median': 89.12025002064183, 'result_age_ms_p95': 125.92794999945909, 'first_source_sequence_id': 0, 'last_source_sequence_id': 5130}
- Detector/tracker: {'total_results': 4807, 'steady_results': 4805, 'warmup_excluded': 2, 'effective_hz': 27.8637799054517, 'detector_wall_ms_median': 27.653199969790876, 'detector_wall_ms_p95': 33.530920022167265, 'tracker_wall_ms_median': 0.49050000961869955, 'tracker_wall_ms_p95': 0.7732199504971505, 'result_age_ms_median': 35.02960002515465, 'result_age_ms_p95': 54.617979982867844, 'first_source_sequence_id': 0, 'last_source_sequence_id': 5131}
- ReID: {'model': 'osnet_x0_25_msmt17', 'event_count': 123, 'reference_count': 3, 'reacquire_count': 12, 'accept_threshold': 0.68, 'retry_threshold': 0.5, 'threshold_status': 'both are provisional engineering thresholds; no different-person negative calibration has been performed'}
- Robot observations: 879; see per-observation provisional flags; Stage 6 disconnected.
- Detector comparison: None
- Source time is host read-complete, not exposure time. Depth values are single-image model estimates in metres, not range-sensor ground truth.
- Master torso median uses only exact same-source-frame bbox/depth pairs; camera XYZ assumes axial_z provisionally (checkpoint convention unconfirmed).
- Metrics: D:\project\CompanionBot\models\minisegway\stage7\results\full_metrics.json
