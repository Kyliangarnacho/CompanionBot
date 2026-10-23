#ifndef COMPANIONBOT_BOARD_IO_H
#define COMPANIONBOT_BOARD_IO_H
#include "companionbot/control.h"
/* Hardware adapters must transform physical IMU axes into the frozen controller
 * sensor axes (pitch gyro[0], accel tilt atan2(y,z)), unwrap signed counts, remove
 * startup gyro bias, and report real sample age in the MCU monotonic domain. */
int cb_board_read_sensor(cb_sensor *sensor);
int cb_board_actuator_ready(void);
/* Success means actual torques are measured/confirmed, never copy request. */
int cb_board_write_torques(double left_nm,double right_nm,double actual_nm[2]);
void cb_board_disable_actuators(void);
int cb_board_physical_arm_requested(void);
uint32_t cb_board_boot_session(void);
#endif
