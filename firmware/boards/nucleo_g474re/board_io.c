#include "board_io.h"
#include "main.h"
#include <math.h>
/* No sensors, pin assignment, motor torque calibration or physical arm circuit
 * have been purchased/verified. These explicit unavailable adapters are the only
 * missing physical boundary; they do not claim sensor/motor simulation. */
int cb_board_read_sensor(cb_sensor *sensor) { (void)sensor; return 0; }
int cb_board_actuator_ready(void) { return 0; }
int cb_board_write_torques(double left,double right,double actual[2]) {
    (void)left; (void)right; actual[0]=NAN; actual[1]=NAN; return 0;
}
void cb_board_disable_actuators(void) { /* No motor pins configured. */ }
int cb_board_physical_arm_requested(void) { return 0; }
uint32_t cb_board_boot_session(void) {
    uint32_t counter,session;
    __HAL_RCC_PWR_CLK_ENABLE(); HAL_PWR_EnableBkUpAccess(); __HAL_RCC_RTCAPB_CLK_ENABLE();
    counter=TAMP->BKP0R+1u; if(!counter) counter=1; TAMP->BKP0R=counter;
    session=HAL_GetUIDw0()^HAL_GetUIDw1()^HAL_GetUIDw2()^counter;
    return session?session:counter;
}
