#ifndef COMPANIONBOT_APP_H
#define COMPANIONBOT_APP_H
#include <stdint.h>
/* DWT maxima for SWD inspection. Values are observations, not certified WCET.
 * Callback timing excludes exception entry/HAL prologue; unavailable board_io
 * cannot exercise the full control path. No new wire/arming authority. */
typedef struct {
    uint32_t control_calls,full_control_calls;
    uint32_t release_latency_max,control_max,release_to_output_max,service_max;
    uint32_t sensor_max,estimator_max,arbitration_max,feedback_max,actuator_max;
    uint32_t parser_dispatch_max,status_max,rx_callback_max,rx_high_water_bytes;
} cb_timing_stats;
extern volatile cb_timing_stats cb_timing;
void cb_app_init(void);
void cb_app_poll(void);
void cb_app_timer_isr(void);
void cb_app_rx_isr(uint16_t size);
void cb_app_tx_done_isr(void);
void cb_app_uart_fault_isr(void);
#endif
