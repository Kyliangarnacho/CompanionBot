# Stage 7.6 ReID Auto Reacquire Run

- Source: `camera:1`
- Tracking frames processed: 7041
- Final Master state: `LOCKED`; bound track ID: `43`
- Preview stopped early: `True`
- Lifecycle events: `{'MASTER_PENDING_LOCK_STARTED': 6, 'MASTER_SELECTED': 2, 'MASTER_TEMP_LOST': 16, 'MASTER_REACQUIRED_SAME_TRACK': 9, 'MASTER_LOST': 7, 'MASTER_SWITCHED': 4, 'MASTER_REID_REACQUIRED': 3}`
- Manual human acceptance is not inferred from this run report; the operator must perform the scenes.
- ReID evidence enabled: `True`
- ReID model/backend: `osnet_x0_25_msmt17` / `torchreid.pytorch.cpu`
- Automatic Master rebind: `True`; cosine threshold: `0.68` (provisional, no different-person negative calibration)
- Reference/candidate log: `D:\project\CompanionBot\models\minisegway\stage7\stage7_6\results\master_reid_evidence.jsonl`
- Event log: `D:\project\CompanionBot\models\minisegway\stage7\stage7_6\results\master_lifecycle_events.jsonl`
- Run config: `D:\project\CompanionBot\models\minisegway\stage7\stage7_6\results\stage7_6_run_config.json`
