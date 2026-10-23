/* Executes the real app.c against explicit HAL/board fixtures. Virtual cycles
 * test deadline logic only: none of these values measures CPU execution speed. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../app/app.c"

TestCoreDebug cb_test_debug;
TestDwt cb_test_dwt;
uint32_t SystemCoreClock=170000000u;
UART_HandleTypeDef huart2={USART2};
TIM_HandleTypeDef htim6={TIM6};
IWDG_HandleTypeDef hiwdg={0};
static uint32_t sensor_cycles,release_again_at;
static unsigned writes,disables,refreshes;
static int test_arm=1;
static uint8_t last_status[CB_WIRE_ENCODED];
static uint16_t last_status_length;

int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n) {
    (void)u;assert(p==dma_rx && n==1024);return HAL_OK;
}
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *t) { (void)t;return HAL_OK; }
int HAL_IWDG_Refresh(IWDG_HandleTypeDef *w) { (void)w;refreshes++;return HAL_OK; }
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n) {
    (void)u;assert(n<=sizeof(last_status));memcpy(last_status,p,n);last_status_length=n;
    cb_app_tx_done_isr();return HAL_OK;
}
void HAL_GPIO_TogglePin(int p,int n) { (void)p;(void)n; }
void Error_Handler(void) { abort(); }
int cb_board_read_sensor(cb_sensor *s) {
    memset(s,0,sizeof(*s));
    s->accel[1]=9.81*sin(CB_NOMINAL_PITCH);s->accel[2]=9.81*cos(CB_NOMINAL_PITCH);
    if(release_again_at) {
        uint32_t end=DWT->CYCCNT+sensor_cycles;
        DWT->CYCCNT=release_again_at;cb_app_timer_isr();DWT->CYCCNT=end;
        release_again_at=0;
    } else DWT->CYCCNT+=sensor_cycles;
    return 1;
}
int cb_board_actuator_ready(void) { return 1; }
int cb_board_physical_arm_requested(void) { return test_arm; }
int cb_board_write_torques(double left,double right,double out[2]) {
    /* Explicit fake actuator for a host logic assertion, never an actual torque
     * implementation or part of production board_io. */
    writes++;out[0]=left;out[1]=right;return 1;
}
void cb_board_disable_actuators(void) { disables++; }
uint32_t cb_board_boot_session(void) { return 77; }

static void put_double(uint8_t *p,double value) {
    uint64_t bits;unsigned i;memcpy(&bits,&value,8);
    for(i=0;i<8;i++)p[i]=(uint8_t)(bits>>(8*i));
}
static void prepare(void) {
    uint8_t p[640]={0};
    cb_app_init();
    cb_put_u32(p,1);put_double(p+24,.002);cb_put_u32(p+32,25);
    cb_put_u32(p+12,1);assert(cb_mcu_reference(&mcu,p,sizeof(p),0)==CB_OK);
    cb_put_u32(p+8,1);cb_put_u32(p+12,26);assert(cb_mcu_reference(&mcu,p,sizeof(p),0)==CB_OK);
    assert(cb_mcu_heartbeat(&mcu,0,0)==CB_OK);
}
int main(int argc,char **argv) {
    const char *mode=argc>1?argv[1]:"timely";
    prepare();
    if(!strcmp(mode,"dma_duplicate")) {
        cb_app_rx_isr(1024);assert(head==1024);
        cb_app_rx_isr(1024);assert(head==1024);
        cb_app_rx_isr(512);assert(head==1536 && !irq_faults);
        printf("DMA duplicate callback and wrap passed\n");return 0;
    }
    if(!strcmp(mode,"ring_overflow")) {
        unsigned i;for(i=0;i<6;i++){cb_app_rx_isr(1024);cb_app_rx_isr(512);}
        assert(irq_faults&FAULT_RX_OVERFLOW);
    }
    if(!strcmp(mode,"dma_protocol")) {
        cb_frame frame={0};size_t length;
        frame.type=CB_MSG_HEARTBEAT;frame.session=77;frame.sequence=1;frame.tick=100;
        length=cb_frame_encode(&frame,dma_rx,sizeof(dma_rx));assert(length>10);
        cb_app_rx_isr(10);cb_app_rx_isr((uint16_t)length);
    }
    DWT->CYCCNT=!strcmp(mode,"cycle_wrap")?UINT32_MAX-500u:340000u;
    cb_app_timer_isr();
    if(!strcmp(mode,"missed_releases")) { DWT->CYCCNT+=340000u;cb_app_timer_isr(); }
    if(!strcmp(mode,"late_start") || !strcmp(mode,"cross_release")) {
        DWT->CYCCNT+=300000;sensor_cycles=100000;
    } else if(!strcmp(mode,"long_compute")) sensor_cycles=400000;
    else sensor_cycles=1000;
    if(!strcmp(mode,"cross_release")) release_again_at=680000;
    cb_app_poll();
    if(!strcmp(mode,"dma_protocol")) assert(parser.accepted==1 && mcu.heartbeat_sequence==1 && head==tail);
    printf("{\"mode\":\"%s\",\"virtual_cycles\":%u,\"armed\":%d,\"actuator_writes\":%u,"
           "\"overruns\":%u,\"faults\":%u,\"disables\":%u,\"watchdog_refreshes\":%u}\n",
           mode,DWT->CYCCNT,armed,writes,overruns,faults,disables,refreshes);
#ifdef CB_EXPECT_STRICT_DEADLINE
    if(!strcmp(mode,"late_start") || !strcmp(mode,"cross_release") || !strcmp(mode,"long_compute") || !strcmp(mode,"missed_releases")) {
        assert(!armed && writes==0 && overruns>0 && (latched_faults&FAULT_OVERRUN));
    } else if(!strcmp(mode,"ring_overflow")) assert(!armed && writes==0 && (faults&FAULT_RX_OVERFLOW));
    else assert(armed && writes==1 && overruns==0);
#endif
    return 0;
}
