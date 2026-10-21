"""Explicit clocks and bounded input watermark; no cross-host clock guesses."""
from pathlib import Path
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


