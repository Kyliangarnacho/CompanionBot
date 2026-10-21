"""Existing local text source/freshness guard; queueing is not authority."""
import math
import time
from companionbot_contracts.stamps import HOST_CLOCK_ID


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

