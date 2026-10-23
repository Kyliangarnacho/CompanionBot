#include "app.h"
#include "main.h"
#include "board_io.h"
#include "frozen_config.h"
#include "companionbot/wire.h"
#include <math.h>
#include <string.h>

enum { FAULT_SENSOR=1, FAULT_ACTUATOR=2, FAULT_OVERRUN=4, FAULT_UART=8, FAULT_RX_OVERFLOW=16,
       FAULT_NOT_ARMED=32, FAULT_CONTROLLER=64, FAULT_TILT=128, FAULT_TICK_EXHAUSTED=256 };
#define RX_DMA_SIZE 1024u
#define RX_RING_SIZE 4096u
static uint8_t dma_rx[RX_DMA_SIZE],ring[RX_RING_SIZE],tx[CB_WIRE_ENCODED];
static volatile uint32_t head,tail,scheduled,irq_faults;
static volatile int tx_busy;
static uint16_t dma_previous;
static uint32_t consumed,session,status_sequence,last_result,overruns,rejections,faults,latched_faults;
static uint32_t rejection_reasons[8];
static int armed,estimator_started,yaw_started,previous_arm;
static cb_parser parser;
static cb_mcu mcu;
static cb_estimator estimator;
static cb_yaw_estimator yaw;
static cb_estimate estimate;
static cb_mcu_output command;
static cb_allocation allocation;
static double yaw_values[3],psi_ref,actual[2]={NAN,NAN};
static int actual_valid;
static volatile uint32_t release_cycle;
static int tick_overrun_recorded;
volatile cb_timing_stats cb_timing;

static void timing_max(volatile uint32_t *maximum,uint32_t value) { if(value>*maximum) *maximum=value; }
static int deadline_missed(uint32_t tick,uint32_t released) {
    return scheduled!=tick || (uint32_t)(DWT->CYCCNT-released)>=SystemCoreClock/500u;
}
static void timing_fault(void) {
    if(!tick_overrun_recorded) { overruns++; tick_overrun_recorded=1; }
    faults|=FAULT_OVERRUN; latched_faults|=FAULT_OVERRUN; armed=0;
    actual_valid=0; actual[0]=NAN; actual[1]=NAN; cb_board_disable_actuators();
}

void cb_app_timer_isr(void) { release_cycle=DWT->CYCCNT; scheduled++; }
void cb_app_tx_done_isr(void) { tx_busy=0; }
void cb_app_uart_fault_isr(void) { irq_faults|=FAULT_UART; }
void cb_app_rx_isr(uint16_t size) {
    uint32_t started=DWT->CYCCNT;
    uint16_t position;
    if(size>RX_DMA_SIZE) { irq_faults|=FAULT_UART; return; }
    position=dma_previous;
    while(position!=size) {
        uint32_t next;
        if(position==RX_DMA_SIZE) position=0;
        if(position==size) break;
        next=(head+1u)%RX_RING_SIZE;
        if(next==tail) { irq_faults|=FAULT_RX_OVERFLOW; break; }
        ring[head]=dma_rx[position++]; __DMB(); head=next;
    }
    /* Keep the HAL callback position in [0, SIZE], including SIZE. TC and IDLE
     * may report SIZE twice; mapping it to zero would replay a full DMA buffer. */
    dma_previous=size;
    timing_max(&cb_timing.rx_callback_max,DWT->CYCCNT-started);
    timing_max(&cb_timing.rx_high_water_bytes,(head+RX_RING_SIZE-tail)%RX_RING_SIZE);
}
void cb_app_init(void) {
    const cb_mcu_config mc={ {CB_DT,CB_SAFE_DECEL,CB_SAFE_JERK,CB_SAFE_HANDOFF},
        CB_HEARTBEAT_TIMEOUT,CB_STARVATION_MARGIN,CB_NOMINAL_PITCH,CB_YAW_TIMEOUT,CB_YAW_DECAY };
    const cb_estimator_config ec={ CB_DT,CB_PITCH_TAU,CB_PLL_BANDWIDTH,CB_COUNTS_PER_REV,CB_WHEEL_RADIUS,
        CB_ACCEL_TAU,CB_ACCEL_SCALE,CB_LEFT_SIGN,CB_RIGHT_SIGN,CB_PLL_SNAP,1 };
    cb_board_disable_actuators(); session=cb_board_boot_session();
    if(cb_mcu_init(&mcu,&mc,0)!=CB_OK || cb_estimator_init(&estimator,&ec)!=CB_OK) Error_Handler();
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk; DWT->CYCCNT=0; DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    if(HAL_UARTEx_ReceiveToIdle_DMA(&huart2,dma_rx,RX_DMA_SIZE)!=HAL_OK || HAL_TIM_Base_Start_IT(&htim6)!=HAL_OK) Error_Handler();
}
static void control_tick(uint32_t tick,uint32_t released) {
    cb_sensor sensor; double error[4],fb=0,requested,sum,yaw_pd[3]={0};
    const double gain[4]=CB_LQR_GAIN;
    uint32_t measured=DWT->CYCCNT;
    int sensors=cb_board_read_sensor(&sensor);
    timing_max(&cb_timing.sensor_max,DWT->CYCCNT-measured);
    faults=irq_faults;
    if(!sensors || !isfinite(sensor.age) || sensor.age<0 || sensor.age>0.010) faults|=FAULT_SENSOR;
    if(!cb_board_actuator_ready()) faults|=FAULT_ACTUATOR;
    if(overruns) faults|=FAULT_OVERRUN;
    if(tick>=UINT32_MAX-1000u) faults|=FAULT_TICK_EXHAUSTED;
    actual_valid=0; actual[0]=NAN; actual[1]=NAN;
    measured=DWT->CYCCNT;
    if(!(faults&FAULT_SENSOR)) {
        int result=estimator_started?cb_estimator_update(&estimator,&sensor,&estimate):cb_estimator_reset(&estimator,&sensor,&estimate);
        if(result) faults|=FAULT_SENSOR;
        else estimator_started=1;
    }
    if(estimator_started && !(faults&FAULT_SENSOR)) {
        double sample_time=tick*CB_DT-sensor.age;
        if(cb_yaw_estimate(&yaw,sensor.gyro[2],sample_time,sensor.age,1,sensor.extrapolate,!yaw_started,yaw_values)) faults|=FAULT_SENSOR;
        else if(!yaw_started) { yaw_started=1; psi_ref=yaw_values[1]; }
        if(fabs(estimate.theta-CB_NOMINAL_PITCH)>0.5) faults|=FAULT_TILT;
    }
    timing_max(&cb_timing.estimator_max,DWT->CYCCNT-measured);
    /* Hard I/O/timing faults latch. Startup with missing devices stays inhibited.
     * Software braking is handled by mcu safety and keeps balancing authority. */
    latched_faults|=faults&(FAULT_OVERRUN|FAULT_UART|FAULT_RX_OVERFLOW|FAULT_CONTROLLER|FAULT_TILT|FAULT_TICK_EXHAUSTED);
    if(armed && (faults&FAULT_SENSOR)) latched_faults|=FAULT_SENSOR;
    faults|=latched_faults;
    if(faults) armed=0;
    if(!estimator_started || (faults&FAULT_SENSOR)) {
        faults|=FAULT_NOT_ARMED; previous_arm=cb_board_physical_arm_requested();
        cb_board_disable_actuators(); return;
    }
    /* Wait for the first scheduled block without inventing a running controller
     * or invalidating its epoch before its start tick. Once started, consume the
     * reference stream even while disarmed, so the bounded receiver cannot fill
     * forever waiting for a human to press an exact 2 ms tick. */
    if(!mcu.has_last && (!mcu.has_active || mcu.active.start_tick>tick)) {
        faults|=FAULT_NOT_ARMED; previous_arm=cb_board_physical_arm_requested();
        cb_board_disable_actuators(); return;
    }
    measured=DWT->CYCCNT;
    if(cb_mcu_step(&mcu,tick,estimate.velocity,estimate.theta,estimate.theta_dot,&command)) faults|=FAULT_CONTROLLER;
    timing_max(&cb_timing.arbitration_max,DWT->CYCCNT-measured);
    { int arm=cb_board_physical_arm_requested();
    if(!armed && !faults && arm && !previous_arm && mcu.has_heartbeat && mcu.state==CB_NORMAL &&
       (tick-mcu.heartbeat_tick)*CB_DT<=CB_HEARTBEAT_TIMEOUT &&
       fabs(estimate.velocity)<=0.03 && fabs(estimate.theta-CB_NOMINAL_PITCH)<=0.08 &&
       fabs(estimate.theta_dot)<=0.20 && fabs(command.reference.p-estimate.position)<=0.02 &&
       fabs(command.reference.v)<=0.03 && fabs(command.yaw_rate)<=0.02) armed=1;
    previous_arm=arm; }
    if(!armed) { faults|=FAULT_NOT_ARMED; psi_ref=yaw_values[1]; }
    cb_timing.full_control_calls++; measured=DWT->CYCCNT;
    error[0]=estimate.position-command.reference.p; error[1]=estimate.velocity-command.reference.v;
    error[2]=estimate.theta-CB_NOMINAL_PITCH-command.reference.theta;
    error[3]=estimate.theta_dot-command.reference.theta_dot;
    if(cb_feedback(gain,error,&fb)!=CB_OK || cb_yaw_pd(CB_YAW_KP,CB_YAW_KR,psi_ref,command.yaw_rate,yaw_values[1],yaw.rate,yaw_pd)!=CB_OK) faults|=FAULT_CONTROLLER;
    requested=fb+CB_LAMBDA_FF*command.reference.u_ff;
    sum=fmax(-2*CB_WHEEL_LIMIT,fmin(requested,2*CB_WHEEL_LIMIT));
    if(cb_allocate(sum,yaw_pd[2],CB_WHEEL_LIMIT,&allocation)!=CB_OK) faults|=FAULT_CONTROLLER;
    allocation.u_sum_requested=requested; allocation.sum_clipped=fabs(requested-sum)>1e-12;
    if(armed) psi_ref+=command.yaw_rate*CB_DT;
    latched_faults|=faults&FAULT_CONTROLLER;
    timing_max(&cb_timing.feedback_max,DWT->CYCCNT-measured);
    timing_max(&cb_timing.release_to_output_max,DWT->CYCCNT-released);
    // Include waiting in the main loop and a newly released tick, not only the
    // time spent inside control_tick. Never write a known-late actuator request.
    if(deadline_missed(tick,released)) timing_fault();
    if(!faults) {
        measured=DWT->CYCCNT;
        actual_valid=cb_board_write_torques(allocation.left,allocation.right,actual);
        timing_max(&cb_timing.actuator_max,DWT->CYCCNT-measured);
        actual_valid=actual_valid && isfinite(actual[0]) && isfinite(actual[1]);
        if(!actual_valid) {
            actual[0]=NAN; actual[1]=NAN;
            faults|=FAULT_ACTUATOR; latched_faults|=FAULT_ACTUATOR; armed=0; cb_board_disable_actuators();
        }
    } else { armed=0; cb_board_disable_actuators(); }
}
static void publish_status(uint32_t tick) {
    cb_frame f; uint8_t *p=f.payload; size_t n; unsigned i;
    float values[20];
    if(tx_busy) return;
    memset(&f,0,sizeof(f)); f.type=CB_MSG_STATUS; f.length=CB_STATUS_BYTES; f.session=session;
    f.sequence=status_sequence++; f.tick=tick;
    cb_put_u32(p,mcu.epoch); cb_put_u32(p+4,(uint32_t)mcu.state); cb_put_u32(p+8,(uint32_t)mcu.reason);
    cb_put_u32(p+12,faults); cb_put_u32(p+16,last_result); cb_put_u32(p+20,rejections+parser.rejected);
    cb_put_u32(p+24,overruns);
    cb_put_u32(p+28,(actual_valid?1u:0u)|(armed?2u:0u)|
        (estimator_started && !(faults&FAULT_SENSOR)?4u:0u)|
        (allocation.sum_clipped?8u:0u)|(allocation.differential_clipped?16u:0u)|(allocation.guard_clipped?32u:0u));
    values[0]=estimator_started?(float)estimate.position:NAN; values[1]=estimator_started?(float)estimate.velocity:NAN;
    values[2]=estimator_started?(float)estimate.theta:NAN; values[3]=estimator_started?(float)estimate.theta_dot:NAN;
    values[4]=yaw_started?(float)yaw_values[1]:NAN; values[5]=yaw_started?(float)yaw.rate:NAN;
    values[6]=(float)command.reference.p; values[7]=(float)command.reference.v; values[8]=(float)command.reference.a;
    values[9]=(float)command.reference.theta; values[10]=(float)command.reference.theta_dot; values[11]=(float)command.reference.u_ff;
    values[12]=(float)allocation.u_sum_requested; values[13]=(float)allocation.u_sum_limited;
    values[14]=(float)allocation.u_diff_requested; values[15]=(float)allocation.u_diff_used;
    values[16]=(float)allocation.left; values[17]=(float)allocation.right;
    values[18]=(float)actual[0]; values[19]=(float)actual[1];
    for(i=0;i<20;i++) cb_put_f32(p+32+4*i,values[i]);
    for(i=0;i<8;i++) cb_put_u32(p+112+4*i,rejection_reasons[i]);
    n=cb_frame_encode(&f,tx,sizeof(tx));
    if(n) { tx_busy=1; if(HAL_UART_Transmit_DMA(&huart2,tx,(uint16_t)n)!=HAL_OK) { tx_busy=0; irq_faults|=FAULT_UART; } }
}
void cb_app_poll(void) {
    uint32_t now,before,released,start,measured; unsigned budget=768; cb_frame frame;
    /* Service the deadline before bounded ingress; never catch up missed control
     * steps with multiple stale sensor samples. Timing faults remain visible. */
    do { before=scheduled; released=release_cycle; now=scheduled; } while(before!=now);
    if(now!=consumed) {
        start=DWT->CYCCNT;
        tick_overrun_recorded=0; cb_timing.control_calls++;
        timing_max(&cb_timing.release_latency_max,start-released);
        if(now-consumed>1) overruns+=now-consumed-1;
        consumed=now;
        if(deadline_missed(now,released)) timing_fault();
        measured=DWT->CYCCNT; control_tick(now,released);
        timing_max(&cb_timing.control_max,DWT->CYCCNT-measured);
        if(deadline_missed(now,released)) timing_fault();
        (void)HAL_IWDG_Refresh(&hiwdg);
        if(now%10u==0) {
            measured=DWT->CYCCNT; publish_status(now);
            timing_max(&cb_timing.status_max,DWT->CYCCNT-measured);
        }
        if(now%250u==0) HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
        timing_max(&cb_timing.service_max,DWT->CYCCNT-start);
    }
    while(budget-- && tail!=head) {
        uint8_t b;
        __DMB(); b=ring[tail]; __DMB(); tail=(tail+1u)%RX_RING_SIZE;
        measured=DWT->CYCCNT;
        { int parsed=cb_parser_feed(&parser,b,&frame);
        if(parsed==1) {
            uint32_t result=(uint32_t)cb_mcu_dispatch(&mcu,&frame,session,scheduled);
            if(result) { last_result=result; rejections++; if(result<8) rejection_reasons[result]++; }
        } else if(parsed<0) { last_result=CB_INVALID; rejection_reasons[CB_INVALID]++; }
        }
        timing_max(&cb_timing.parser_dispatch_max,DWT->CYCCNT-measured);
        if(scheduled!=consumed) break;
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer) { if(timer->Instance==TIM6) cb_app_timer_isr(); }
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart,uint16_t size) { if(uart->Instance==USART2) cb_app_rx_isr(size); }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) { if(uart->Instance==USART2) cb_app_tx_done_isr(); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) { if(uart->Instance==USART2) cb_app_uart_fault_isr(); }
