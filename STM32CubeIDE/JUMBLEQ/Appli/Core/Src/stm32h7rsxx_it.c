/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7rsxx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7rsxx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// HardFault情報を保存する構造体（リセット後も確認可能にBKPSRAMに配置することも可能）
typedef struct {
    uint32_t magic;        // 0xDEADBEEF なら有効なデータ
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t stacked_pc;
    uint32_t stacked_lr;
    uint32_t stacked_psr;
    uint32_t stacked_r0;
    uint32_t stacked_r1;
    uint32_t stacked_r2;
    uint32_t stacked_r3;
    uint32_t stacked_r12;
    uint32_t psp;
    uint32_t msp;
    // ISR関連のデバッグ情報
    uint32_t usb_isr_count;     // クラッシュ時のISR呼び出し回数
    uint32_t usb_isr_msp_min;   // ISR中のMSP最小値
    uint32_t usb_isr_msp_start; // ISR開始時のMSP値
} HardFaultInfo_t;

// noinit属性でリセット後も保持（デバッグ用）
__attribute__((section(".noinit"))) volatile HardFaultInfo_t g_hardFaultInfo;

// USB ISRデバッグ用変数（HardFaultハンドラからも参照されるため先頭で定義）
volatile uint32_t dbg_usb_isr_count = 0;       // USB ISR呼び出し回数
volatile uint32_t dbg_usb_isr_msp_min = 0xFFFFFFFF;  // ISR内MSP最小値
volatile uint32_t dbg_usb_isr_msp_start = 0;   // ISR開始時MSP
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel3;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
extern DMA_HandleTypeDef handle_HPDMA1_Channel0;
extern I2C_HandleTypeDef hi2c3;
extern SAI_HandleTypeDef hsai_BlockA1;
extern SAI_HandleTypeDef hsai_BlockA2;
extern SPI_HandleTypeDef hspi5;
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  // デバッグ用：フォールトの詳細情報を取得
  volatile uint32_t cfsr = SCB->CFSR;    // Configurable Fault Status Register
  volatile uint32_t hfsr = SCB->HFSR;    // Hard Fault Status Register
  volatile uint32_t mmfar = SCB->MMFAR;  // MemManage Fault Address
  volatile uint32_t bfar = SCB->BFAR;    // Bus Fault Address
  volatile uint32_t afsr = SCB->AFSR;    // Auxiliary Fault Status Register
  
  // UFSR (Usage Fault Status Register) は CFSR の上位16ビット
  volatile uint16_t ufsr = (cfsr >> 16) & 0xFFFF;
  
  // UNDEFINSTRビットをチェック (UFSR bit 0)
  volatile uint8_t is_undefinstr = (ufsr & 0x0001) ? 1 : 0;
  
  // スタックポインタを取得
  volatile uint32_t sp;
  __asm volatile ("mov %0, sp" : "=r" (sp));
  
  // スタックフレームから例外発生時のレジスタを取得
  // PSPまたはMSPのどちらが使われているかを確認
  volatile uint32_t *fault_stack;
  __asm volatile (
    "TST lr, #4 \n"
    "ITE EQ \n"
    "MRSEQ %0, MSP \n"
    "MRSNE %0, PSP \n"
    : "=r" (fault_stack)
  );
  
  // 例外スタックフレーム: R0, R1, R2, R3, R12, LR, PC, xPSR
  volatile uint32_t stacked_r0  = fault_stack[0];
  volatile uint32_t stacked_r1  = fault_stack[1];
  volatile uint32_t stacked_r2  = fault_stack[2];
  volatile uint32_t stacked_r3  = fault_stack[3];
  volatile uint32_t stacked_r12 = fault_stack[4];
  volatile uint32_t stacked_lr  = fault_stack[5];  // 戻りアドレス
  volatile uint32_t stacked_pc  = fault_stack[6];  // フォールト発生時のPC ← 重要！
  volatile uint32_t stacked_psr = fault_stack[7];
  
  // これらの変数はデバッガで確認できます
  (void)cfsr;
  (void)hfsr;
  (void)mmfar;
  (void)bfar;
  (void)afsr;
  (void)ufsr;
  (void)is_undefinstr;
  (void)sp;
  (void)stacked_r0;
  (void)stacked_r1;
  (void)stacked_r2;
  (void)stacked_r3;
  (void)stacked_r12;
  (void)stacked_lr;
  (void)stacked_pc;
  (void)stacked_psr;
  
  // グローバル変数に保存（リセット後も確認可能）
  g_hardFaultInfo.magic = 0xDEADBEEF;
  g_hardFaultInfo.cfsr = cfsr;
  g_hardFaultInfo.hfsr = hfsr;
  g_hardFaultInfo.mmfar = mmfar;
  g_hardFaultInfo.bfar = bfar;
  g_hardFaultInfo.stacked_pc = stacked_pc;
  g_hardFaultInfo.stacked_lr = stacked_lr;
  g_hardFaultInfo.stacked_psr = stacked_psr;
  g_hardFaultInfo.stacked_r0 = stacked_r0;
  g_hardFaultInfo.stacked_r1 = stacked_r1;
  g_hardFaultInfo.stacked_r2 = stacked_r2;
  g_hardFaultInfo.stacked_r3 = stacked_r3;
  g_hardFaultInfo.stacked_r12 = stacked_r12;
  __asm volatile ("MRS %0, PSP" : "=r" (g_hardFaultInfo.psp));
  __asm volatile ("MRS %0, MSP" : "=r" (g_hardFaultInfo.msp));
  // ISR関連のデバッグ情報を保存
  g_hardFaultInfo.usb_isr_count = dbg_usb_isr_count;
  g_hardFaultInfo.usb_isr_msp_min = dbg_usb_isr_msp_min;
  g_hardFaultInfo.usb_isr_msp_start = dbg_usb_isr_msp_start;
  
  __BKPT(0);  // ここでブレーク（デバッガ接続時）
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  // MemManage Fault情報を収集
  volatile uint32_t cfsr = SCB->CFSR;
  volatile uint32_t mmfar = SCB->MMFAR;
  volatile uint8_t mmfsr = cfsr & 0xFF;  // MMFSR は CFSR の下位8ビット
  
  // 直接MSPとPSPを取得
  volatile uint32_t msp_val, psp_val, lr_val;
  __asm volatile ("MRS %0, MSP" : "=r" (msp_val));
  __asm volatile ("MRS %0, PSP" : "=r" (psp_val));
  __asm volatile ("MOV %0, lr" : "=r" (lr_val));
  
  // LRのbit2でどちらのスタックを使うか判断
  volatile uint32_t *fault_stack;
  if (lr_val & 0x4) {
    fault_stack = (uint32_t *)psp_val;  // PSPを使用（タスクコンテキスト）
  } else {
    fault_stack = (uint32_t *)msp_val;  // MSPを使用（ハンドラモード）
  }
  
  // グローバル変数に保存（先にMSP/PSP/LRを保存）
  g_hardFaultInfo.magic = 0xDEADBEEF;
  g_hardFaultInfo.cfsr = cfsr;
  g_hardFaultInfo.mmfar = mmfar;
  g_hardFaultInfo.msp = msp_val;
  g_hardFaultInfo.psp = psp_val;
  g_hardFaultInfo.usb_isr_count = dbg_usb_isr_count;
  g_hardFaultInfo.usb_isr_msp_min = dbg_usb_isr_msp_min;
  g_hardFaultInfo.usb_isr_msp_start = lr_val;  // LR値を保存（デバッグ用に流用）
  
  // スタックポインタが有効なRAM範囲か確認してからアクセス
  // DTCM: 0x20000000-0x20010000, AXI SRAM: 0x24000000-0x24072000
  uint32_t sp = (uint32_t)fault_stack;
  if ((sp >= 0x20000000 && sp < 0x20010000) || (sp >= 0x24000000 && sp < 0x24080000)) {
    g_hardFaultInfo.stacked_r0 = fault_stack[0];
    g_hardFaultInfo.stacked_r1 = fault_stack[1];
    g_hardFaultInfo.stacked_r2 = fault_stack[2];
    g_hardFaultInfo.stacked_r3 = fault_stack[3];
    g_hardFaultInfo.stacked_r12 = fault_stack[4];
    g_hardFaultInfo.stacked_lr = fault_stack[5];
    g_hardFaultInfo.stacked_pc = fault_stack[6];
    g_hardFaultInfo.stacked_psr = fault_stack[7];
  } else {
    // スタックポインタが無効 - 値を0xFFFFFFFFでマーク
    g_hardFaultInfo.stacked_r0 = 0xFFFFFFFF;
    g_hardFaultInfo.stacked_r1 = 0xFFFFFFFF;
    g_hardFaultInfo.stacked_r2 = sp;  // 無効なSP値を保存
    g_hardFaultInfo.stacked_r3 = 0xFFFFFFFF;
    g_hardFaultInfo.stacked_r12 = 0xFFFFFFFF;
    g_hardFaultInfo.stacked_lr = 0xFFFFFFFF;
    g_hardFaultInfo.stacked_pc = 0xFFFFFFFF;
    g_hardFaultInfo.stacked_psr = 0xFFFFFFFF;
  }
  
  (void)mmfsr;
  
  __BKPT(0);  // デバッガでブレーク
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7RSxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7rsxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 2 global interrupt.
  */
void GPDMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 0 */

  /* USER CODE END GPDMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel2);
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 1 */

  /* USER CODE END GPDMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 3 global interrupt.
  */
void GPDMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 0 */

  /* USER CODE END GPDMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel3);
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 1 */

  /* USER CODE END GPDMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 4 global interrupt.
  */
void GPDMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 0 */

  /* USER CODE END GPDMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel4);
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 1 */

  /* USER CODE END GPDMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles SPI5 global interrupt.
  */
void SPI5_IRQHandler(void)
{
  /* USER CODE BEGIN SPI5_IRQn 0 */

  /* USER CODE END SPI5_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi5);
  /* USER CODE BEGIN SPI5_IRQn 1 */

  /* USER CODE END SPI5_IRQn 1 */
}

/**
  * @brief This function handles HPDMA1 Channel 0 global interrupt.
  */
void HPDMA1_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN HPDMA1_Channel0_IRQn 0 */

  /* USER CODE END HPDMA1_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_HPDMA1_Channel0);
  /* USER CODE BEGIN HPDMA1_Channel0_IRQn 1 */

  /* USER CODE END HPDMA1_Channel0_IRQn 1 */
}

/**
  * @brief This function handles Serial Audio Interface 1 block A interrupt.
  */
void SAI1_A_IRQHandler(void)
{
  /* USER CODE BEGIN SAI1_A_IRQn 0 */

  /* USER CODE END SAI1_A_IRQn 0 */
  HAL_SAI_IRQHandler(&hsai_BlockA1);
  /* USER CODE BEGIN SAI1_A_IRQn 1 */

  /* USER CODE END SAI1_A_IRQn 1 */
}

/**
  * @brief This function handles Serial Audio Interface 2 block A interrupt.
  */
void SAI2_A_IRQHandler(void)
{
  /* USER CODE BEGIN SAI2_A_IRQn 0 */

  /* USER CODE END SAI2_A_IRQn 0 */
  HAL_SAI_IRQHandler(&hsai_BlockA2);
  /* USER CODE BEGIN SAI2_A_IRQn 1 */

  /* USER CODE END SAI2_A_IRQn 1 */
}

/**
  * @brief This function handles I2C3 event interrupt.
  */
void I2C3_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_EV_IRQn 0 */

  /* USER CODE END I2C3_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_EV_IRQn 1 */

  /* USER CODE END I2C3_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C3 error interrupt.
  */
void I2C3_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_ER_IRQn 0 */

  /* USER CODE END I2C3_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_ER_IRQn 1 */

  /* USER CODE END I2C3_ER_IRQn 1 */
}

/**
  * @brief This function handles USB OTG HS interrupt.
  */
void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */
  tusb_int_handler(BOARD_TUD_RHPORT, true);
  return;
  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
