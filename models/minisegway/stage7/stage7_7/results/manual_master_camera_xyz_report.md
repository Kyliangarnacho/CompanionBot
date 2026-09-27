# Stage 7.7 manual_master_camera_xyz

- Mode: concurrent; source: camera:1
- Actual source: 1280x720 BGR uint8; 29.49 Hz; 2203 frames
- Calibration: {'path': 'D:\\project\\CompanionBot\\models\\minisegway\\stage7\\stage7_7\\config\\c920e_camera_params_1280x720.npz', 'resolution': [1280, 720], 'status': 'loaded_and_exact_source_resolution_validated'}
- Depth: {'total_results': 998, 'steady_results': 996, 'warmup_excluded': 2, 'effective_hz': 13.887292489957035, 'inference_adapter_wall_ms_median': 68.23179993079975, 'inference_adapter_wall_ms_p95': 82.7325250429567, 'result_age_ms_median': 91.42060001613572, 'result_age_ms_p95': 123.39879994397052, 'first_source_sequence_id': 0, 'last_source_sequence_id': 2200}
- Detector/tracker: {'total_results': 1998, 'steady_results': 1996, 'warmup_excluded': 2, 'effective_hz': 27.81452881801759, 'detector_wall_ms_median': 28.416950022801757, 'detector_wall_ms_p95': 35.49449998536147, 'tracker_wall_ms_median': 0.4774500266648829, 'tracker_wall_ms_p95': 0.8128750196192414, 'result_age_ms_median': 35.884450015146285, 'result_age_ms_p95': 55.48212499707006, 'first_source_sequence_id': 0, 'last_source_sequence_id': 2201}
- Detector comparison: None
- Source time is host read-complete, not exposure time. Depth values are single-image model estimates in metres, not range-sensor ground truth.
- Master torso median uses only exact same-source-frame bbox/depth pairs; camera XYZ assumes axial Z provisionally (checkpoint convention unconfirmed).
- Metrics: D:\project\CompanionBot\models\minisegway\stage7\stage7_7\results\manual_master_camera_xyz_metrics.json
