"""Explicit clocks and bounded input watermark; no cross-host clock guesses."""
from pathlib import Path
import math
import time
from uuid import uuid4

from companionbot_interfaces.msg import SourceStamp


def host_clock_id():
    try:
        return "linux-boot:" + Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except OSError:
        # No shared-clock proof on other platforms: accept only this process.
        return "process:" + _PROCESS_EPOCH


_PROCESS_EPOCH = uuid4().hex
HOST_CLOCK_ID = host_clock_id()


def source_stamp(source_id, epoch, sequence, source_time_s, *,
                 clock_domain="host_perf_counter", clock_id=None,
                 time_semantics="host_read_complete", received_at_s=None,
                 produced_at_s=None, produced_clock_domain="", produced_clock_id=None, version=2):
    result = SourceStamp()
    result.contract_version, result.source_id, result.source_epoch = version, str(source_id), epoch
    result.has_sequence, result.sequence = sequence is not None, sequence or 0
    result.has_source_time, result.source_time_s = source_time_s is not None, source_time_s or 0.0
    result.clock_domain = clock_domain
    result.clock_id = clock_id if clock_id is not None else (HOST_CLOCK_ID if clock_domain == "host_perf_counter" else "")
    result.time_semantics = time_semantics
    result.received_at_s = time.perf_counter() if received_at_s is None else received_at_s
    result.received_clock_domain, result.received_clock_id = "host_perf_counter", HOST_CLOCK_ID
    result.has_produced_time, result.produced_at_s = produced_at_s is not None, produced_at_s or 0.0
    result.produced_clock_domain = produced_clock_domain
    result.produced_clock_id = produced_clock_id if produced_clock_id is not None else (
        HOST_CLOCK_ID if produced_clock_domain == "host_perf_counter" else "")
    return result


class InputGuard:
    """A single configured source epoch; rebinding requires runtime restart.

    Reject unknown domains before touching the watermark. This provides an
    honest local interface until a measured cross-machine clock adapter exists.
    """
    def __init__(self, clock=time.perf_counter, max_age_s=1.0):
        self.clock, self.max_age_s = clock, max_age_s
        self.source = None
        self.sequence, self.source_time = -1, float("-inf")

    def accept(self, meta):
        if meta.contract_version != 2 or not meta.source_id or not meta.source_epoch or not meta.has_sequence:
            return "invalid_source_contract"
        if (meta.clock_domain != "host_perf_counter" or meta.clock_id != HOST_CLOCK_ID
                or meta.time_semantics != "text_submitted" or not meta.has_source_time):
            return "unmapped_clock_or_time_semantics"
        if not math.isfinite(meta.source_time_s) or not 0 <= self.clock() - meta.source_time_s <= self.max_age_s:
            return "stale_or_future_input"
        source = (meta.source_id, meta.source_epoch)
        if self.source is not None and source != self.source:
            return "source_restart_requires_rebind"
        if meta.sequence <= self.sequence or meta.source_time_s < self.source_time:
            return "duplicate_or_out_of_order"
        self.source, self.sequence, self.source_time = source, meta.sequence, meta.source_time_s
        return None
