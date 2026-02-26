/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7rsxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

#include "tusb.h"
#include "usb_descriptors.h"

#include "main_oled.h"
#include "sub_oled.h"
#include "ssd1306_fonts.h"

#include "SEGGER_RTT.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define RESET_FROM_FW 1  // SigmaStudio+からのリセットを有効にする場合は0に設定
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DSP_RESET_Pin GPIO_PIN_1
#define DSP_RESET_GPIO_Port GPIOH
#define S0_Pin GPIO_PIN_8
#define S0_GPIO_Port GPIOD
#define S1_Pin GPIO_PIN_9
#define S1_GPIO_Port GPIOD
#define S2_Pin GPIO_PIN_10
#define S2_GPIO_Port GPIOD
#define CODEC_RESET_Pin GPIO_PIN_13
#define CODEC_RESET_GPIO_Port GPIOB
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
#define UCPD_FLG_n_Pin GPIO_PIN_8
#define UCPD_FLG_n_GPIO_Port GPIOM
#define UCPD_FLG_n_EXTI_IRQn EXTI8_IRQn
#define UCPD_PWR_EN_Pin GPIO_PIN_9
#define UCPD_PWR_EN_GPIO_Port GPIOM

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
