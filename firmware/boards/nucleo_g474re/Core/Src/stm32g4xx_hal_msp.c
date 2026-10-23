#include "main.h"
void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE(); __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    HAL_PWREx_DisableUCPDDeadBattery();
}
void HAL_UART_MspInit(UART_HandleTypeDef *uart) {
    GPIO_InitTypeDef gpio={0};
    if(uart->Instance!=USART2) return;
    __HAL_RCC_USART2_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin=GPIO_PIN_2|GPIO_PIN_3; gpio.Mode=GPIO_MODE_AF_PP; gpio.Pull=GPIO_PULLUP;
    gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH; gpio.Alternate=GPIO_AF7_USART2; HAL_GPIO_Init(GPIOA,&gpio);
    hdma_usart2_rx.Instance=DMA1_Channel1; hdma_usart2_rx.Init.Request=DMA_REQUEST_USART2_RX;
    hdma_usart2_rx.Init.Direction=DMA_PERIPH_TO_MEMORY; hdma_usart2_rx.Init.PeriphInc=DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc=DMA_MINC_ENABLE; hdma_usart2_rx.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE; hdma_usart2_rx.Init.Mode=DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority=DMA_PRIORITY_HIGH;
    if(HAL_DMA_Init(&hdma_usart2_rx)!=HAL_OK) Error_Handler();
    __HAL_LINKDMA(uart,hdmarx,hdma_usart2_rx);
    hdma_usart2_tx.Instance=DMA1_Channel2; hdma_usart2_tx.Init.Request=DMA_REQUEST_USART2_TX;
    hdma_usart2_tx.Init.Direction=DMA_MEMORY_TO_PERIPH; hdma_usart2_tx.Init.PeriphInc=DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc=DMA_MINC_ENABLE; hdma_usart2_tx.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE; hdma_usart2_tx.Init.Mode=DMA_NORMAL;
    hdma_usart2_tx.Init.Priority=DMA_PRIORITY_LOW;
    if(HAL_DMA_Init(&hdma_usart2_tx)!=HAL_OK) Error_Handler();
    __HAL_LINKDMA(uart,hdmatx,hdma_usart2_tx);
    HAL_NVIC_SetPriority(USART2_IRQn,2,0); HAL_NVIC_EnableIRQ(USART2_IRQn);
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *timer) {
    if(timer->Instance!=TIM6) return;
    __HAL_RCC_TIM6_CLK_ENABLE(); HAL_NVIC_SetPriority(TIM6_DAC_IRQn,1,0); HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}
