"""Existing Stage 9 ROS contract regression, relocated to its owner."""
from companionbot_contracts.stamps import source_stamp

def test_unmapped_device_clock_never_inherits_host_clock_identity():
    from companionbot_contracts.stamps import HOST_CLOCK_ID
    stamp = source_stamp("mcu", "epoch", 1, .2, clock_domain="mcu_ticks",
                         produced_at_s=10., produced_clock_domain="host_perf_counter")
    assert stamp.clock_id == ""
    assert stamp.received_clock_id == stamp.produced_clock_id == HOST_CLOCK_ID
    assert stamp.source_time_s == .2 and stamp.produced_at_s == 10.


