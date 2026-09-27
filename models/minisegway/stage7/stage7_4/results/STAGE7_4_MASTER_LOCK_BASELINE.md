# Stage 7.4 Manual Master Lock Baseline

## Scope

This stage adds an explicit user-selected Master binding after person tracking.
It does not change camera capture, detection, tracking, ByteTrack configuration,
or any Stage 3–6 controller code. Master is a business-layer selection and is
not a person identity claim.

## Data path

`ColorFrame → LatestFrameSlot → PersonDetector → PersonDetectionFrame → PersonTracker → PersonTrackingFrame → MasterSelector / MasterManager → MasterTrackingFrame`

The new Master layer consumes only the backend-neutral `PersonTrackingFrame`
and its lifecycle diagnostics. The preview callback forwards click coordinates
only; selection and state transitions run in the main processing loop.

## Contracts and behavior

- `MasterSelector.select_at_pixel()` hit-tests original source-frame pixel
  coordinates (`x` right, `y` down) against current `TrackedPerson` boxes.
- If boxes overlap, the selected candidate is the one whose center is nearest
  the click; equal distances are resolved by the lower temporary track ID.
- `MasterManager` stores a `bound_track_id`. `master_track` directly references
  the current `TrackedPerson` only while state is `LOCKED`.
- A missing bound track becomes `TEMPORARILY_LOST`. The same track ID returning
  restores `LOCKED`; a tracker `newly_removed_track_ids` event makes the state
  terminal `LOST` until the user explicitly clicks a current track.
- No other track can become Master automatically. A visible duplicate track
  does not change the binding.
- Selection and lifecycle events are written to
  `master_lifecycle_events.jsonl` when the preview demo runs. Each event carries
  source sequence/time, old/new track IDs, event type, and click candidates
  when selection came from an overlapping-box hit test. Lost events use
  `new_track_id: null` while the frame contract continues to retain the old
  `bound_track_id` for explicit recovery/reselection.

## Preview geometry

The demo shows an annotated copy with the original frame dimensions. It does
not resize, letterbox, mirror, or crop the preview image. Click coordinates are
interpreted directly in source-frame pixels. If a future UI applies a geometric
transform, its inverse mapping must be added before passing clicks to the
selector.

## Verification boundary

The Master contract is verified with hardware-free synthetic tracking frames.
No human-camera acceptance is claimed by this report. Operator scenes for
normal walking, short occlusion, leaving/re-entering the field of view, manual
reselection, multiple people, and duplicate tracks remain to be performed.

Run configuration and event logs are generated at demo runtime in this
Stage 7.4 results directory. The Stage 7.1–7.3 source files and configs remain
unchanged by this stage.
