#include "main.h"
void NMI_Handler(void) { Error_Handler(); }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
void SysTick_Handler(void) { HAL_IncTick(); }
void DMA1_Channel1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart2_rx); }
void DMA1_Channel2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart2_tx); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
void TIM6_DAC_IRQHandler(void) { HAL_TIM_IRQHandler(&htim6); }
