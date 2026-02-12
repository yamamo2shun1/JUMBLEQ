/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7rsxx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Configure the system Power Supply */

  if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK)
  {
    /* Initialization error */
    Error_Handler();
  }

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SBS_CLK_ENABLE();

  /* System interrupt init*/

  /* Enable the XSPIM_P1 interface */
  HAL_PWREx_EnableXSPIM1();

  /* Enable USB Voltage detector */
  if(HAL_PWREx_EnableUSBVoltageDetector() != HAL_OK)
  {
   /* Initialization error */
   Error_Handler();
  }

  /* The CSI is used by the compensation cells and must be enabled before enabling the
     compensation cells.
     For more details refer to RM0477 [SBS I/O compensation cell management] chapter.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure the compensation cell */
  HAL_SBS_ConfigCompensationCell(SBS_IO_XSPI1_CELL, SBS_IO_CELL_CODE, 0U, 0U);

  /* Enable compensation cell */
  HAL_SBS_EnableCompensationCell(SBS_IO_XSPI1_CELL);

  /* wait ready before enabled IO */
  while(HAL_SBS_GetCompensationCellReadyStatus(SBS_IO_XSPI1_CELL_READY) != 1U);

  /* high speed low voltage config */
  HAL_SBS_EnableIOSpeedOptimize(SBS_IO_XSPI1_HSLV);

  HAL_PWREx_EnableUSBHSregulator();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
