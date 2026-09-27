# Stage 7.2 Stream Benchmark

- Source: `camera:1`
- Frame profile: 640×480 BGR uint8
- Detector: `yolo26n.pt`; imgsz=640; conf=0.25; device=automatic
- Source frames / measured source rate: 1137 / 30.00 fps
- Processed results: 974 total, 971 steady-state after 3 warmup results
- Steady-state processed FPS: 27.64 fps
- Processed results/source frames: 974/1137 (85.7%)
- Steady-state processed/source FPS ratio: 0.921
- Sequence gaps / overwritten / final pending: 163 / 162 / 1 (skipped/source 14.3%)
- Steady-state person detections: 973
- Empty steady-state results: 0/971 (0.0%)
- Processing wall mean / median / p95: 25.96 / 24.95 / 30.56 ms
- Host receive-to-result age mean / median / p95: 39.19 / 35.09 / 60.29 ms
- Latency summary sample window: last 971 steady-state results (maximum 10000)
- Frame times use the host monotonic read-complete clock; video playback pacing is a simulation and does not provide camera exposure timestamps.
- Preprocess/inference/postprocess split is unavailable through the backend-neutral Stage 7.1 detector API; full detector-call and frame-result wall times are recorded in the CSV.
- Per-frame/detection CSV: `D:\project\CompanionBot\models\minisegway\stage7\stage7_2\results\person_detection_stream.csv`
- Metrics JSON: `D:\project\CompanionBot\models\minisegway\stage7\stage7_2\results\stage7_2_stream_metrics.json`
