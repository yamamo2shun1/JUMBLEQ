/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    extmemloader_init.h
  * @author  MCD Application Team
  * @brief   Header file of Loader_Src.c
  *
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef EXTMEMLOADER_INIT_H
#define EXTMEMLOADER_INIT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7rsxx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/

/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

uint32_t extmemloader_Init(void);
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#define SW2_Pin GPIO_PIN_14
#define SW2_GPIO_Port GPIOD
#define SW1_Pin GPIO_PIN_15
#define SW1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOD
#define LED0_Pin GPIO_PIN_2
#define LED0_GPIO_Port GPIOD
#define UCPD_PWR_EN_Pin GPIO_PIN_9
#define UCPD_PWR_EN_GPIO_Port GPIOM
#endif /* EXTMEMLOADER_INIT_H */
