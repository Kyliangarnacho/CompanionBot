"""Finite mapping worker must restore process policy even on interruption."""
import gc

import pytest

from companionbot_simulation.runtime.spatial_session import SpatialSession


def test_mapping_interruption_restores_gc_and_closes_source_log(tmp_path):
    enabled = gc.isenabled()

    def interrupted(frame):
        assert frame['state'].imu_valid
        raise InterruptedError('injected_mapping_shutdown')

    session = SpatialSession(tmp_path, scenario='mapping', duration_s=.2, on_frame=interrupted)
    with pytest.raises(InterruptedError, match='injected_mapping_shutdown'):
        session.run()
    assert gc.isenabled() == enabled
    assert session._log.closed
    assert len((tmp_path/'sensorized_state.jsonl').read_text().splitlines()) == 1
    assert not (tmp_path/'frozen_result_posthoc.json').exists()
