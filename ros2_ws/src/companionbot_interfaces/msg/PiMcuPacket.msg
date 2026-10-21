# Diagnostic transport of the EXISTING Stage 6 codec, not a new controller wire format.
# channel: reference/yaw/heartbeat/safety/status/state/observation (see INTERFACES.md).
# Reference payload includes epoch/start tick/2 ms dt and six float32 sample fields.
SourceStamp meta
string direction
string channel
uint8[] payload
bool hardware_execution_ready
