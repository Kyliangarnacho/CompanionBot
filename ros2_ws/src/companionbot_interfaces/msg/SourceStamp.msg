# Source time is never silently replaced by ROS time. Clock ID names one clock instance.
# Legacy business-message Headers use ROS publication time. Standard spatial
# messages and BalanceState explicitly use source time mapped into ROS time.
uint32 contract_version
string source_id
string source_epoch
bool has_sequence
uint64 sequence
bool has_source_time
float64 source_time_s
string clock_domain
string clock_id
string time_semantics
float64 received_at_s
string received_clock_domain
string received_clock_id
bool has_produced_time
float64 produced_at_s
string produced_clock_domain
string produced_clock_id
