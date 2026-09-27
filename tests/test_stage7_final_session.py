import numpy as np

from perception.master_reid_session import MasterReIDSession
from perception.master_selection import MasterState
from perception.person_reid import MasterReIDEvidence, PersonReIdentifier
from test_stage7_6_reid import FakeEmbeddingBackend, _pair, _track


def test_final_worker_runs_pending_reference_then_lost_reid_to_new_track():
    evidence = MasterReIDEvidence(PersonReIdentifier(FakeEmbeddingBackend()))
    session = MasterReIDSession(evidence, initial_track_id=1)
    for seq, stamp in enumerate((1.0, 1.16, 1.32)):
        frame, tracked = _pair(seq, (_track(1),), time_s=stamp)
        result = session.process(frame, tracked)
        if seq < 2:
            assert result.master.state is MasterState.PENDING_LOCK
    assert result.master.state is MasterState.LOCKED
    assert np.linalg.norm(evidence.reference.embedding.vector) == 1.0
    frame, tracked = _pair(3, (), time_s=1.4, removed=(1,))
    assert session.process(frame, tracked).master.state is MasterState.LOST
    for seq in (4, 5, 6):
        frame, tracked = _pair(seq, (_track(2),), time_s=1.4 + seq * 0.1)
        result = session.process(frame, tracked)
        if seq < 6:
            assert result.master.state is MasterState.LOST
    assert result.master.bound_track_id == 2
    assert result.master.state is MasterState.LOCKED
    reacquired = [e for e in session.events if e["event"] == "MASTER_REID_REACQUIRED"]
    assert len(reacquired) == 1
    assert (reacquired[0]["old_track_id"], reacquired[0]["new_track_id"]) == (1, 2)


def test_final_pending_disappearance_cancels_reference_and_clear_clears_both():
    evidence = MasterReIDEvidence(PersonReIdentifier(FakeEmbeddingBackend()))
    session = MasterReIDSession(evidence, initial_track_id=1)
    frame, tracked = _pair(0, (_track(1),), time_s=1.0)
    assert session.process(frame, tracked).master.state is MasterState.PENDING_LOCK
    frame, tracked = _pair(1, (), time_s=1.1)
    assert session.process(frame, tracked).master.state is MasterState.UNSELECTED
    assert evidence.pending_sample_count == 0
    session.select("test-camera", 2)
    frame, tracked = _pair(2, (_track(2),), time_s=1.2)
    assert session.process(frame, tracked).master.state is MasterState.PENDING_LOCK
    session.clear("test-camera")
    frame, tracked = _pair(3, (_track(2),), time_s=1.3)
    assert session.process(frame, tracked).master.state is MasterState.UNSELECTED
    assert evidence.reference is None and evidence.pending_sample_count == 0
