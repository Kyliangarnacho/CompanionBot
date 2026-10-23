#ifndef CB_TEST_HAL_MAIN_H
#define CB_TEST_HAL_MAIN_H
/* Host-only logical fixture. Never included by the ARM firmware target. */
#include <stdint.h>
typedef struct { uintptr_t Instance; } UART_HandleTypeDef;
typedef UART_HandleTypeDef TIM_HandleTypeDef;
typedef UART_HandleTypeDef IWDG_HandleTypeDef;
typedef struct { uint32_t DEMCR; } TestCoreDebug;
typedef struct { uint32_t CTRL, CYCCNT; } TestDwt;
extern TestCoreDebug cb_test_debug;
extern TestDwt cb_test_dwt;
extern uint32_t SystemCoreClock;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;
extern IWDG_HandleTypeDef hiwdg;
#define CoreDebug (&cb_test_debug)
#define DWT (&cb_test_dwt)
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define HAL_OK 0
#define GPIOA 0
#define GPIO_PIN_5 5
#define TIM6 6u
#define USART2 2u
static inline void __DMB(void) { }
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *);
int HAL_IWDG_Refresh(IWDG_HandleTypeDef *);
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
void HAL_GPIO_TogglePin(int, int);
void Error_Handler(void);
#endif
