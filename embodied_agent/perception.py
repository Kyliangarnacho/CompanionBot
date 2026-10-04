"""Read-only Stage 7 Master metadata. This does not grant actuator authority."""
from __future__ import annotations

import threading
import time
from uuid import uuid4

from perception.master_selection import MasterTrackingFrame
from .behavior import FakeRobotBackend


class PerceptionAwareFakeRobot(FakeRobotBackend):
    """Fake execution permission with the actual read-only Master prerequisite."""
    def __init__(self, master_status):
        super().__init__()
        self.master_status = master_status

    async def latest_state(self):
        state = await super().latest_state()
        master = self.master_status()
        return state.model_copy(update={"master_locked": bool(master.get("available")
                                and master.get("visible") and master.get("state") == "LOCKED")})


class MasterStatusBuffer:
    def __init__(self, *, clock=time.perf_counter, max_age_s=1.0):
        self.clock, self.max_age_s = clock, max_age_s
        self._lock = threading.Lock()
        self._value = None
        self.source_epoch = uuid4().hex
        self._received_at_s = None

    def publish(self, result: MasterTrackingFrame, *, source_epoch: str | None = None) -> None:
        if not isinstance(result, MasterTrackingFrame):
            raise ValueError("master_contract_mismatch")
        with self._lock:
            if source_epoch is not None and source_epoch != self.source_epoch:
                raise ValueError("master_source_epoch_mismatch")
            previous = self._value
            if previous and (previous.source_id != result.source_id or
                    result.source_sequence_id < previous.source_sequence_id or result.source_time_s < previous.source_time_s):
                raise ValueError("master_source_restart_or_out_of_order")
            # Same-frame explicit selection/clear is legitimate. Duplicate
            # callbacks are idempotent; new state may replace that same frame.
            if previous is result:
                return
            self._value = result
            self._received_at_s = self.clock()

    def clear(self):
        with self._lock:
            self._value = None
            self.source_epoch = uuid4().hex

    def status(self) -> dict:
        with self._lock:
            value = self._value
            received_at_s, source_epoch = self._received_at_s, self.source_epoch
        if value is None:
            return {"available": False, "state": "UNAVAILABLE", "reason": "no_stage7_result",
                    "hardware_execution_ready": False}
        age = self.clock() - value.source_time_s
        fresh = 0 <= age <= self.max_age_s
        return {"available": fresh, "state": value.state.value, "track_id": value.bound_track_id,
                "adapter_contract_version": 2, "source_epoch": source_epoch, "received_at_s": received_at_s,
                "produced_at_s": None, "produced_clock_domain": None,
                "visible": value.master_track is not None and fresh,
                "source_id": value.source_id, "source_sequence_id": value.source_sequence_id,
                "source_time_s": value.source_time_s, "width": value.frame_width, "height": value.frame_height,
                "age_s": age, "clock_domain": "host_perf_counter", "time_semantics": "host_read_complete",
                "reason": "stage7_metadata" if fresh else "stale_or_future_master",
                "hardware_execution_ready": False}
