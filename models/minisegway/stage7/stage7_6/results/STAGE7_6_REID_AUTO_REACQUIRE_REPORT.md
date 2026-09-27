# Stage 7.6 ReID Auto Reacquire Patch

## Scope

Stage 7.6 adds a pending manual reference confirmation and an automatic LOST
reacquisition gate to the existing Stage 7.5 OSNet evidence path. A click enters
`PENDING_LOCK`. The same active track must provide three exact-frame-aligned
embeddings across at least 0.30 seconds. Their arithmetic mean is L2-normalized
to form the single Master reference; only then does the manager enter `LOCKED`.
If that track disappears while pending, the selection is cancelled and Master
returns to `UNSELECTED`.

In `LOST`, a candidate must persist for three consecutive tracker updates before
the first current-frame, exact-crop OSNet score is computed. A score at least
0.68 binds the new temporary ID and emits `MASTER_REID_REACQUIRED`. A score in
`[0.50, 0.68)` remains `LOST` and may be rescored from a new current frame about
every 0.20 seconds while that track persists; any retry at least 0.68 reacquires.
A score below 0.50 is retained for display but ends retries for that track. Both
thresholds are provisional engineering thresholds: no different-person negative
calibration has been performed, and similarity is not a probability.

## Preserved boundaries

The patch does not change YOLO, ByteTrack, tracker configuration or buffer,
OSNet weights/model/crop rules, Master temporary-loss/LOST semantics, or any
Stage 3–6 controller, planner, estimator, safety, or parameter baseline. There
is no gallery, long-term reference bank, margin rule, new model, or controller
integration. The original single reference remains the only identity reference;
temporary candidate retry status and latest score are retained only while that
track remains active in the current LOST episode.

## Verification

- Hardware-free tests cover delayed `LOCKED`, three-sample 512D normalized
  averaging, pending disappearance cancellation, post-stability first scoring,
  0.70 immediate acceptance, 0.60 retry to 0.71, 0.40 retry stop, removed-track
  retry-state cleanup, new-ID binding, and reacquisition event fields.
- Project `.venv` root suite: `121 passed` (Python 3.11.9, pip 26.2.1; same
  `.venv` interpreter used for `python -m pytest tests -q`).
- Real C920 identity/reacquisition scenes: pending human acceptance. No
  different-person calibration or identity-accuracy claim is made by synthetic
  contract tests.

## Operator acceptance

Run the existing preview with Stage 7.6 ReID enabled, then manually verify the
pending lock delay, pending-track cancellation, Master loss, below-threshold
non-rebinding, and same-person ID reacquisition. Keep the scene and detector
setup otherwise consistent with the Stage 7.5 manual evidence runs.
