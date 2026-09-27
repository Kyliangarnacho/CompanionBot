"""Thin Stage 7.6 orchestration for the final perception worker.

One worker owns Master/OSNet state. GUI commands carry a selected track ID;
all crops and lifecycle updates use that worker's current exact ColorFrame.
"""

from dataclasses import asdict, dataclass
import queue

from perception.camera import ColorFrame
from perception.master_selection import MasterManager, MasterState, MasterTrackingFrame
from perception.person_reid import MasterReIDEvidence, ReIDInputError
from perception.person_tracker import PersonTrackingFrame


@dataclass(frozen=True)
class MasterPerceptionResult:
    tracking: PersonTrackingFrame
    master: MasterTrackingFrame
    pending_samples: int
    latest_similarity: float | None


class MasterReIDSession:
    """Reuse frozen pending-reference and accept/retry semantics, without a gallery."""

    def __init__(self, evidence: MasterReIDEvidence, *, initial_track_id: int | None = None,
                 accept_threshold=0.68, retry_threshold=0.50,
                 retry_interval_s=0.20, stable_updates=3):
        self.manager = MasterManager()
        self.evidence = evidence
        self.initial_track_id = initial_track_id
        self.accept_threshold = accept_threshold
        self.retry_threshold = retry_threshold
        self.retry_interval_s = retry_interval_s
        self.stable_updates = stable_updates
        self.commands = queue.SimpleQueue()
        self.events: list[dict] = []

    def select(self, source_id: str, track_id: int) -> None:
        self.commands.put((source_id, track_id))

    def clear(self, source_id: str) -> None:
        self.commands.put((source_id, None))

    def _event(self, event):
        if event is not None:
            record = asdict(event)
            record["event"] = event.event.value
            self.events.append(record)

    def process(self, frame: ColorFrame, tracking: PersonTrackingFrame) -> MasterPerceptionResult:
        if (frame.source_id, frame.sequence_id, frame.host_receive_time_s,
                frame.width, frame.height) != (
                tracking.source_id, tracking.source_sequence_id, tracking.source_time_s,
                tracking.frame_width, tracking.frame_height):
            raise ValueError("Master/ReID requires the exact tracking source ColorFrame")
        for event in self.manager.update(tracking):
            self._event(event)
            if event.event.value == "MASTER_PENDING_LOCK_CANCELLED":
                self.evidence.cancel_pending_reference()
        visible_ids = {track.track_id for track in tracking.tracks}
        if self.initial_track_id in visible_ids:
            self.select(frame.source_id, self.initial_track_id)
            self.initial_track_id = None
        while not self.commands.empty():
            source_id, track_id = self.commands.get_nowait()
            if source_id != frame.source_id:
                continue
            if track_id is None:
                self._event(self.manager.clear(tracking))
                self.evidence.clear_reference()
            elif track_id in visible_ids:
                self._event(self.manager.begin_pending_lock(tracking, track_id))
                self.evidence.begin_pending_reference(frame, tracking, track_id)
        if self.manager.state is MasterState.PENDING_LOCK:
            try:
                reference = self.evidence.collect_pending_reference_sample(frame, tracking)
            except ReIDInputError as error:
                self.events.append({"event": "MASTER_REFERENCE_SAMPLE_REJECTED",
                                    "source_sequence_id": frame.sequence_id, "reason": str(error)})
            else:
                if reference is not None:
                    self._event(self.manager.confirm_pending_lock(tracking))
                    self.events.append({
                        "event": "MASTER_REFERENCE_CREATED", "source_id": frame.source_id,
                        "source_sequence_id": frame.sequence_id,
                        "sample_sequence_ids": reference.sample_sequence_ids,
                        "embedding_dimension": reference.embedding.dimension,
                    })
        master = self.manager.result(tracking)
        stable = self.manager.stable_reid_candidate_ids(tracking, stable_updates=self.stable_updates)
        scoring = self.evidence.score_candidates(
            frame, tracking, master, stable_candidate_track_ids=stable,
            accept_threshold=self.accept_threshold, retry_threshold=self.retry_threshold,
            retry_interval_s=self.retry_interval_s,
        )
        if scoring is not None:
            for item in scoring.scores:
                self.events.append({"event": "REID_CANDIDATE_SCORE", **asdict(item)})
            for rejected in scoring.rejected:
                self.events.append({"event": "REID_CANDIDATE_REJECTED",
                                    "source_sequence_id": frame.sequence_id, **asdict(rejected)})
        self._event(self.manager.consider_reid_candidate_scores(
            tracking,
            {item.candidate_track_id: item.similarity
             for item in self.evidence.lost_episode_candidate_scores},
            threshold=self.accept_threshold, stable_updates=self.stable_updates,
        ))
        scores = self.evidence.lost_episode_candidate_scores
        latest_similarity = max((item.similarity for item in scores), default=None)
        return MasterPerceptionResult(tracking, self.manager.result(tracking),
                                      self.evidence.pending_sample_count, latest_similarity)
