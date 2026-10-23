# Passive spatial estimate, never an actuator command. Contract version 1.
# Header.stamp is the estimate horizon in ROS time; frame_id is base_link.
# The producer must prove its source->ROS mapping. MCU arrival time is insufficient.
std_msgs/Header header
SourceStamp meta
uint16 spatial_contract_version
string geometry_id
# Identity mapping for simulation; a measured mapping is required on devices.
string ros_time_mapping_id
float64 ros_time_uncertainty_s
# Pitch-corrected wheel path distance (arc length), not world x.
float64 traveled_distance_m
float64 forward_velocity_m_s
# Physical body roll/pitch and navigation Euler yaw; radians, no theta_eq subtraction.
float64 roll_rad
float64 pitch_rad
float64 yaw_rad
float64 roll_rate_rad_s
float64 pitch_rate_rad_s
float64 yaw_rate_rad_s
# Historical control channel order: MJ left (ROS right), MJ right (ROS left).
# Continuous wheel angle/velocity relative to body from the existing encoder PLL.
float64[2] wheel_relative_angle_rad
float64[2] wheel_relative_velocity_rad_s
# Packet times mapped into the same ROS time domain as Header, not arrival time.
# meta separately retains the control source's raw clock/time and identity.
float64 imu_sample_time_s
float64 imu_available_time_s
float64 imu_measurement_age_s
bool imu_valid
bool imu_stale
bool encoder_valid
