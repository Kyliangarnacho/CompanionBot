# Simulation/diagnostic only. No subscription grants actuator authority.
std_msgs/Header header
SourceStamp meta
float64 position_hat_m
float64 velocity_hat_m_s
float64 pitch_hat_rad
float64 pitch_rate_hat_rad_s
# Requested below is allocator output; pre-allocation sum/differential stay in history.
float64[2] requested_wheels_nm
float64[2] actuator_wheels_nm
bool saturated
float64 physics_dt_s
float64 controller_dt_s
bool hardware_execution_ready
