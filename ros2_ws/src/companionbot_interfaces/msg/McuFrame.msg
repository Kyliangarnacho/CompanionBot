# Embedded transport v2 only. Not the frozen simulation PiMcuPacket v1 envelope.
# Commands: 1 ReferenceBlock v1 bytes; 2 heartbeat; 3 yaw f64;
# 4 safety (authority u8, reason u16); 5 resume (epoch u32, authority u8).
# Receive: 128 status. Little endian, COBS/CRC32 added by the C++ transport.
# For TX, control_tick is a deadline in MCU ticks (2 ms), at most 125 ticks ahead.
# For RX, control_tick is the MCU sample tick. Never subtract it from ROS time.
uint8 kind
uint32 session_id
uint32 sequence_id
uint32 control_tick
# TX: absolute CLOCK_MONOTONIC deadline on this Linux host; RX: local arrival.
string<=64 host_clock_id
uint64 host_deadline_monotonic_ns
uint8[<=672] payload
