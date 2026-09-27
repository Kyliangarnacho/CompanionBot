# Stage 7.7 depth_detector_concurrent_1280

- Mode: concurrent; source: camera:1
- Actual source: 1280x720 BGR uint8; 22.32 Hz; 240 frames
- Calibration: {'path': 'D:\\project\\CompanionBot\\models\\minisegway\\stage7\\stage7_7\\config\\c920e_camera_params_1280x720.npz', 'resolution': [1280, 720], 'status': 'loaded_and_exact_source_resolution_validated'}
- Depth: {'total_results': 108, 'steady_results': 106, 'warmup_excluded': 2, 'effective_hz': 12.637750274307846, 'inference_adapter_wall_ms_median': 79.10390000324696, 'inference_adapter_wall_ms_p95': 84.91527504520491, 'result_age_ms_median': 102.7989000431262, 'result_age_ms_p95': 136.82700000936165, 'first_source_sequence_id': 0, 'last_source_sequence_id': 239}
- Detector/tracker: {'total_results': 185, 'steady_results': 183, 'warmup_excluded': 2, 'effective_hz': 21.819366646814423, 'detector_wall_ms_median': 30.943999998271465, 'detector_wall_ms_p95': 35.47555007971823, 'tracker_wall_ms_median': 0.5449999589473009, 'tracker_wall_ms_p95': 0.8088400005362928, 'result_age_ms_median': 32.93999994639307, 'result_age_ms_p95': 54.193730023689575, 'first_source_sequence_id': 0, 'last_source_sequence_id': 239}
- Detector comparison: {'baseline_path': 'D:\\project\\CompanionBot\\models\\minisegway\\stage7\\stage7_7\\results\\detector_only_1280_metrics.json', 'processed_hz_change_percent': -6.121549663006009, 'median_call_wall_ms_change': 6.316799903288484, 'median_result_age_ms_change': 7.4001000029966235}
- Source time is host read-complete, not exposure time. Depth values are single-image model estimates in metres, not range-sensor ground truth.
- Metrics: D:\project\CompanionBot\models\minisegway\stage7\stage7_7\results\depth_detector_concurrent_1280_metrics.json
