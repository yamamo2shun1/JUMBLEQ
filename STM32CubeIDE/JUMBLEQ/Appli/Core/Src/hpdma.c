/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hpdma.c
  * @brief   This file provides code for the configuration
  *          of the HPDMA instances.
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
#include "hpdma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

DMA_HandleTypeDef handle_HPDMA1_Channel0;

/* HPDMA1 init function */
void MX_HPDMA1_Init(void)
{

  /* USER CODE BEGIN HPDMA1_Init 0 */
  __HAL_RCC_HPDMA1_CLK_ENABLE();
  /* USER CODE END HPDMA1_Init 0 */

  /* USER CODE BEGIN HPDMA1_Init 1 */
  HAL_NVIC_SetPriority(HPDMA1_Channel0_IRQn, 8, 0);  // ADCは時間制約が緩いため最低優先度
  HAL_NVIC_EnableIRQ(HPDMA1_Channel0_IRQn);
  /* USER CODE END HPDMA1_Init 1 */
  handle_HPDMA1_Channel0.Instance = HPDMA1_Channel0;
  handle_HPDMA1_Channel0.InitLinkedList.Priority = DMA_LOW_PRIORITY_MID_WEIGHT;
  handle_HPDMA1_Channel0.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  handle_HPDMA1_Channel0.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  handle_HPDMA1_Channel0.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
  handle_HPDMA1_Channel0.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&handle_HPDMA1_Channel0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_HPDMA1_Channel0, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HPDMA1_Init 2 */

  /* USER CODE END HPDMA1_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
