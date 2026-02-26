/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sbs.c
  * @brief   This file provides code for the configuration
  *          of the SBS instances.
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
#include "sbs.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* SBS init function */
void MX_SBS_Init(void)
{

  /* USER CODE BEGIN SBS_Init 0 */
  /* Configure the compensation cell */
  HAL_SBS_ConfigCompensationCell(SBS_IO_XSPI1_CELL, SBS_IO_CELL_CODE, 0U, 0U);

  /* Enable compensation cell */
  HAL_SBS_EnableCompensationCell(SBS_IO_XSPI1_CELL);

  /* wait ready before enabled IO */
  while(HAL_SBS_GetCompensationCellReadyStatus(SBS_IO_XSPI1_CELL_READY) != 1U);

  uint32_t code1, nmos1, pmos1;

  HAL_SBS_GetCompensationCell(SBS_IO_XSPI1_CELL, &code1, &nmos1, &pmos1);

  HAL_SBS_ConfigCompensationCell(SBS_IO_XSPI1_CELL, SBS_IO_REGISTER_CODE, nmos1, pmos1);

  HAL_SBS_DisableCompensationCell(SBS_IO_XSPI1_CELL);

  /* high speed low voltage config */
  HAL_SBS_EnableIOSpeedOptimize(SBS_IO_XSPI1_HSLV);
  /* USER CODE END SBS_Init 0 */

  /* USER CODE BEGIN SBS_Init 1 */

  /* USER CODE END SBS_Init 1 */
  /* USER CODE BEGIN SBS_Init 2 */

  /* USER CODE END SBS_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

