#ifndef COMPANIONBOT_MAIN_H
#define COMPANIONBOT_MAIN_H
#include "stm32g4xx_hal.h"
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx, hdma_usart2_tx;
extern TIM_HandleTypeDef htim6;
extern IWDG_HandleTypeDef hiwdg;
void Error_Handler(void);
#endif
