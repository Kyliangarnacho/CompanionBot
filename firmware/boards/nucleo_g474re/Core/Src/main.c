/* Board configuration adapted from ST CubeG4 NUCLEO-G474RE HAL templates.
 * Peripheral drivers/startup/system remain unmodified external ST sources. */
#include "main.h"
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx, hdma_usart2_tx;
TIM_HandleTypeDef htim6;
IWDG_HandleTypeDef hiwdg;

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc={0}; RCC_ClkInitTypeDef clk={0};
    if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST)!=HAL_OK) Error_Handler();
    osc.OscillatorType=RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
    osc.HSIState=RCC_HSI_ON; osc.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT; osc.LSIState=RCC_LSI_ON;
    osc.PLL.PLLState=RCC_PLL_ON; osc.PLL.PLLSource=RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM=RCC_PLLM_DIV4; osc.PLL.PLLN=85;
    osc.PLL.PLLP=RCC_PLLP_DIV2; osc.PLL.PLLQ=RCC_PLLQ_DIV2; osc.PLL.PLLR=RCC_PLLR_DIV2;
    if(HAL_RCC_OscConfig(&osc)!=HAL_OK) Error_Handler();
    clk.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK; clk.AHBCLKDivider=RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider=RCC_HCLK_DIV1; clk.APB2CLKDivider=RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&clk,FLASH_LATENCY_4)!=HAL_OK) Error_Handler();
}
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef gpio={0}; __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
    gpio.Pin=GPIO_PIN_5; gpio.Mode=GPIO_MODE_OUTPUT_PP; gpio.Pull=GPIO_NOPULL;
    gpio.Speed=GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOA,&gpio);
}
static void MX_DMA_Init(void) {
    __HAL_RCC_DMAMUX1_CLK_ENABLE(); __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn,2,0); HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn,2,0); HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}
static void MX_USART2_UART_Init(void) {
    huart2.Instance=USART2; huart2.Init.BaudRate=921600; huart2.Init.WordLength=UART_WORDLENGTH_8B;
    huart2.Init.StopBits=UART_STOPBITS_1; huart2.Init.Parity=UART_PARITY_NONE;
    huart2.Init.Mode=UART_MODE_TX_RX; huart2.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart2.Init.OverSampling=UART_OVERSAMPLING_16; huart2.Init.OneBitSampling=UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler=UART_PRESCALER_DIV1; huart2.AdvancedInit.AdvFeatureInit=UART_ADVFEATURE_NO_INIT;
    if(HAL_UART_Init(&huart2)!=HAL_OK || HAL_UARTEx_DisableFifoMode(&huart2)!=HAL_OK) Error_Handler();
}
static void MX_TIM6_Init(void) {
    htim6.Instance=TIM6; htim6.Init.Prescaler=169; htim6.Init.CounterMode=TIM_COUNTERMODE_UP;
    htim6.Init.Period=1999; htim6.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_Base_Init(&htim6)!=HAL_OK) Error_Handler();
}
static void MX_IWDG_Init(void) {
    hiwdg.Instance=IWDG; hiwdg.Init.Prescaler=IWDG_PRESCALER_64;
    hiwdg.Init.Window=IWDG_WINDOW_DISABLE; hiwdg.Init.Reload=124;
    if(HAL_IWDG_Init(&hiwdg)!=HAL_OK) Error_Handler();
}
int main(void) {
    HAL_Init(); SystemClock_Config(); MX_GPIO_Init(); MX_DMA_Init();
    MX_USART2_UART_Init(); MX_TIM6_Init(); MX_IWDG_Init();
    /* USER CODE BEGIN 2 */
    cb_app_init();
    /* USER CODE END 2 */
    /* USER CODE BEGIN WHILE */
    while(1) {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
        cb_app_poll();
    }
    /* USER CODE END 3 */
}
void Error_Handler(void) { __disable_irq(); for(;;) {} }
