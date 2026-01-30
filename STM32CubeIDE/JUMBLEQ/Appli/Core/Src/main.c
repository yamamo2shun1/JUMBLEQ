/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "adc.h"
#include "gpdma.h"
#include "hpdma.h"
#include "i2c.h"
#include "sai.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "audio_control.h"
#include "led_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// デバッグ用 - Watchウィンドウで確認するためのextern宣言
extern volatile uint32_t dbg_usb_task_count;
extern volatile uint32_t dbg_audio_task_count;
extern volatile uint32_t dbg_tx_half_count;
extern volatile uint32_t dbg_tx_cplt_count;
extern volatile uint32_t dbg_fill_tx_count;
extern volatile uint32_t dbg_usb2ring_bytes;
extern volatile int32_t dbg_ring_used;
extern volatile uint32_t dbg_fill_underrun;
extern volatile uint32_t dbg_fill_copied;
extern volatile uint32_t dbg_usb2ring_count;
extern volatile uint32_t dbg_usb2ring_total;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(uint8_t ch)
{
    return ITM_SendChar(ch);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* Enable the CPU Cache */

    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();

    /* Enable D-Cache---------------------------------------------------------*/
    SCB_EnableDCache();

    /* MCU Configuration--------------------------------------------------------*/

    /* Update SystemCoreClock variable according to RCC registers values. */
    SystemCoreClockUpdate();

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_GPDMA1_Init();
    MX_HPDMA1_Init();
    MX_USB_OTG_HS_PCD_Init();
    MX_I2C3_Init();
    MX_SPI5_Init();
    MX_SAI1_Init();
    MX_TIM1_Init();
    MX_SAI2_Init();
    MX_ADC1_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
    MX_FREERTOS_Init();

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // FreeRTOS使用時はここに到達しない
        // すべての処理はタスク（freertos.c）で実行される
    }
    /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
