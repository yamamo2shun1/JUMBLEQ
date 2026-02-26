/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : linked_list.c
 * Description        : This file provides code for the configuration
 *                      of the LinkedList.
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
#include "linked_list.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "audio_control.h"
/* USER CODE END Includes */

DMA_NodeTypeDef Node_GPDMA1_Channel2 __attribute__((section("noncacheable_buffer")));
DMA_QListTypeDef List_GPDMA1_Channel2;
DMA_NodeTypeDef Node_GPDMA1_Channel3 __attribute__((section("noncacheable_buffer")));
DMA_QListTypeDef List_GPDMA1_Channel3;
DMA_NodeTypeDef Node_HPDMA1_Channel0 __attribute__((section("noncacheable_buffer")));
DMA_QListTypeDef List_HPDMA1_Channel0;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern int32_t stereo_out_buf[];  // RX バッファ（main.c）
extern int32_t stereo_in_buf[];   // TX バッファ（main.c）

extern uint32_t adc_val[];
/* USER CODE END PM */

/**
  * @brief  DMA Linked-list List_GPDMA1_Channel2 configuration
  * @param  None
  * @retval None
  */
HAL_StatusTypeDef MX_List_GPDMA1_Channel2_Config(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  /* DMA node configuration declaration */
  DMA_NodeConfTypeDef pNodeConfig;

  /* Rebuilding the queue is required when sample-rate switch reconfigures DMA multiple times. */
  ret |= HAL_DMAEx_List_ResetQ(&List_GPDMA1_Channel2);

  /* Set node configuration ################################################*/
  pNodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
  pNodeConfig.Init.Request = GPDMA1_REQUEST_SAI2_A;
  pNodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  pNodeConfig.Init.Direction = DMA_MEMORY_TO_PERIPH;
  pNodeConfig.Init.SrcInc = DMA_SINC_INCREMENTED;
  pNodeConfig.Init.DestInc = DMA_DINC_FIXED;
  pNodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  pNodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  pNodeConfig.Init.SrcBurstLength = 1;
  pNodeConfig.Init.DestBurstLength = 1;
  pNodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  pNodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  pNodeConfig.Init.Mode = DMA_NORMAL;
  pNodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
  pNodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  pNodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  pNodeConfig.SrcAddress = (uint32_t) stereo_out_buf;
  pNodeConfig.DstAddress = (uint32_t) &SAI2_Block_A->DR;
  pNodeConfig.DataSize = SAI_TX_BUF_SIZE * 4U;

  /* Build Node_GPDMA1_Channel2 Node */
  ret |= HAL_DMAEx_List_BuildNode(&pNodeConfig, &Node_GPDMA1_Channel2);

  /* Insert Node_GPDMA1_Channel2 to Queue */
  ret |= HAL_DMAEx_List_InsertNode_Tail(&List_GPDMA1_Channel2, &Node_GPDMA1_Channel2);

  ret |= HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel2);

   return ret;
}

/**
  * @brief  DMA Linked-list List_GPDMA1_Channel3 configuration
  * @param  None
  * @retval None
  */
HAL_StatusTypeDef MX_List_GPDMA1_Channel3_Config(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  /* DMA node configuration declaration */
  DMA_NodeConfTypeDef pNodeConfig;

  /* Rebuilding the queue is required when sample-rate switch reconfigures DMA multiple times. */
  ret |= HAL_DMAEx_List_ResetQ(&List_GPDMA1_Channel3);

  /* Set node configuration ################################################*/
  pNodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
  pNodeConfig.Init.Request = GPDMA1_REQUEST_SAI1_A;
  pNodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  pNodeConfig.Init.Direction = DMA_PERIPH_TO_MEMORY;
  pNodeConfig.Init.SrcInc = DMA_SINC_FIXED;
  pNodeConfig.Init.DestInc = DMA_DINC_INCREMENTED;
  pNodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  pNodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  pNodeConfig.Init.SrcBurstLength = 1;
  pNodeConfig.Init.DestBurstLength = 1;
  pNodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  pNodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  pNodeConfig.Init.Mode = DMA_NORMAL;
  pNodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
  pNodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  pNodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  pNodeConfig.SrcAddress = (uint32_t) &SAI1_Block_A->DR;
  pNodeConfig.DstAddress = (uint32_t) stereo_in_buf;
  pNodeConfig.DataSize = SAI_RX_BUF_SIZE * 4U;

  /* Build Node_GPDMA1_Channel3 Node */
  ret |= HAL_DMAEx_List_BuildNode(&pNodeConfig, &Node_GPDMA1_Channel3);

  /* Insert Node_GPDMA1_Channel3 to Queue */
  ret |= HAL_DMAEx_List_InsertNode_Tail(&List_GPDMA1_Channel3, &Node_GPDMA1_Channel3);

  ret |= HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel3);

   return ret;
}

/**
  * @brief  DMA Linked-list List_HPDMA1_Channel0 configuration
  * @param  None
  * @retval None
  */
HAL_StatusTypeDef MX_List_HPDMA1_Channel0_Config(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  /* DMA node configuration declaration */
  DMA_NodeConfTypeDef pNodeConfig;

  /* Rebuilding the queue is required when sample-rate switch reconfigures DMA multiple times. */
  ret |= HAL_DMAEx_List_ResetQ(&List_HPDMA1_Channel0);

  /* Set node configuration ################################################*/
  pNodeConfig.NodeType = DMA_HPDMA_LINEAR_NODE;
  pNodeConfig.Init.Request = HPDMA1_REQUEST_ADC1;
  pNodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  pNodeConfig.Init.Direction = DMA_PERIPH_TO_MEMORY;
  pNodeConfig.Init.SrcInc = DMA_SINC_FIXED;
  pNodeConfig.Init.DestInc = DMA_DINC_INCREMENTED;
  pNodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  pNodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  pNodeConfig.Init.SrcBurstLength = 1;
  pNodeConfig.Init.DestBurstLength = 1;
  pNodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT1|DMA_DEST_ALLOCATED_PORT0;
  pNodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  pNodeConfig.Init.Mode = DMA_NORMAL;
  pNodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
  pNodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  pNodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  pNodeConfig.SrcAddress = (uint32_t) &ADC1->DR;
  pNodeConfig.DstAddress = (uint32_t) adc_val;
  pNodeConfig.DataSize = ADC_NUM * 4U;

  /* Build Node_HPDMA1_Channel0 Node */
  ret |= HAL_DMAEx_List_BuildNode(&pNodeConfig, &Node_HPDMA1_Channel0);

  /* Insert Node_HPDMA1_Channel0 to Queue */
  ret |= HAL_DMAEx_List_InsertNode_Tail(&List_HPDMA1_Channel0, &Node_HPDMA1_Channel0);

  ret |= HAL_DMAEx_List_SetCircularMode(&List_HPDMA1_Channel0);

   return ret;
}

