/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "tusb.h"
#include "audio_control.h"
#include "led_control.h"
#include "oled_control.h"
#include "adc.h"
#include "SigmaStudioFW.h"
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
/* USER CODE BEGIN Variables */
// FreeRTOSヒープを通常RAMに配置
// configAPPLICATION_ALLOCATED_HEAP=1 で有効化
__attribute__((aligned(8)))
uint8_t ucHeap[configTOTAL_HEAP_SIZE];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t) osPriorityNormal,
};
/* Definitions for usbTask */
osThreadId_t usbTaskHandle;
const osThreadAttr_t usbTask_attributes = {
    .name       = "usbTask",
    .stack_size = 512 * 4,
    .priority   = (osPriority_t) osPriorityHigh,
};
/* Definitions for audioTask */
osThreadId_t audioTaskHandle;
const osThreadAttr_t audioTask_attributes = {
    .name       = "audioTask",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t) osPriorityHigh,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
    .name       = "ledTask",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t) osPriorityNormal,
};
/* Definitions for adcTask */
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
    .name       = "adcTask",
    .stack_size = 512 * 4,
    .priority   = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
    .name       = "oledTask",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t) osPriorityNormal,
};
/* Definitions for spiMutex */
osMutexId_t spiMutexHandle;
const osMutexAttr_t spiMutex_attributes = {
    .name = "spiMutex"};
/* Definitions for i2cMutex */
osMutexId_t i2cMutexHandle;
const osMutexAttr_t i2cMutex_attributes = {
    .name = "i2cMutex"};
/* Definitions for spiTxBinarySem */
osSemaphoreId_t spiTxBinarySemHandle;
const osSemaphoreAttr_t spiTxBinarySem_attributes = {
    .name = "spiTxBinarySem"};
/* Definitions for spiTxRxBinarySem */
osSemaphoreId_t spiTxRxBinarySemHandle;
const osSemaphoreAttr_t spiTxRxBinarySem_attributes = {
    .name = "spiTxRxBinarySem"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void* argument);
void StartUSBTask(void* argument);
void StartAudioTask(void* argument);
void StartLEDTask(void* argument);
void StartADCTask(void* argument);
void StartOLEDTask(void* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char* pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char* pcTaskName)
{
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
    called if a stack overflow is detected. */
    (void) xTask;
    (void) pcTaskName;
    __BKPT(0);  // Stack overflow detected - break here
    for (;;)
        ;
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created. It is also called by various parts of the
    demo application. If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    __BKPT(0);  // Malloc failed - break here
    for (;;)
        ;
}
/* USER CODE END 5 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */
    /* Create the mutex(es) */
    /* creation of spiMutex */
    spiMutexHandle = osMutexNew(&spiMutex_attributes);

    /* creation of i2cMutex */
    i2cMutexHandle = osMutexNew(&i2cMutex_attributes);

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* creation of spiTxBinarySem */
    spiTxBinarySemHandle = osSemaphoreNew(1, 1, &spiTxBinarySem_attributes);

    /* creation of spiTxRxBinarySem */
    spiTxRxBinarySemHandle = osSemaphoreNew(1, 1, &spiTxRxBinarySem_attributes);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of usbTask */
    usbTaskHandle = osThreadNew(StartUSBTask, NULL, &usbTask_attributes);

    /* creation of audioTask */
    audioTaskHandle = osThreadNew(StartAudioTask, NULL, &audioTask_attributes);

    /* creation of ledTask */
    ledTaskHandle = osThreadNew(StartLEDTask, NULL, &ledTask_attributes);

    /* creation of adcTask */
    adcTaskHandle = osThreadNew(StartADCTask, NULL, &adcTask_attributes);

    /* creation of oledTask */
    oledTaskHandle = osThreadNew(StartOLEDTask, NULL, &oledTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */

// スタック監視情報（デバッガのWatchウィンドウで確認可能）
volatile uint32_t dbg_free_heap          = 0;
volatile uint32_t dbg_min_free_heap      = 0xFFFFFFFF;  // ヒープ最小空き容量
volatile uint32_t dbg_usb_stack_free     = 0;
volatile uint32_t dbg_audio_stack_free   = 0;
volatile uint32_t dbg_led_stack_free     = 0;
volatile uint32_t dbg_adc_stack_free     = 0;
volatile uint32_t dbg_default_stack_free = 0;
volatile uint32_t dbg_min_usb_stack      = 0xFFFFFFFF;  // USB最小スタック残量
volatile uint32_t dbg_min_audio_stack    = 0xFFFFFFFF;  // Audio最小スタック残量

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void* argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;)
    {
        dbg_free_heap = xPortGetFreeHeapSize();
        if (dbg_free_heap < dbg_min_free_heap)
        {
            dbg_min_free_heap = dbg_free_heap;
        }

        // 各タスクのスタック残量を監視（High Water Mark）
        dbg_usb_stack_free     = uxTaskGetStackHighWaterMark(usbTaskHandle) * 4;
        dbg_audio_stack_free   = uxTaskGetStackHighWaterMark(audioTaskHandle) * 4;
        dbg_led_stack_free     = uxTaskGetStackHighWaterMark(ledTaskHandle) * 4;
        dbg_adc_stack_free     = uxTaskGetStackHighWaterMark(adcTaskHandle) * 4;
        dbg_default_stack_free = uxTaskGetStackHighWaterMark(NULL) * 4;

        // 最小値を記録（問題発生時の最悪ケースを追跡）
        if (dbg_usb_stack_free < dbg_min_usb_stack)
        {
            dbg_min_usb_stack = dbg_usb_stack_free;
        }
        if (dbg_audio_stack_free < dbg_min_audio_stack)
        {
            dbg_min_audio_stack = dbg_audio_stack_free;
        }

        // デバッガのWatchウィンドウまたはLive Expressionsで監視
        // SWOが動作しない環境ではprintfを使わない

        update_color_state();
        osDelay(1000);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUSBTask */
/**
 * @brief Function implementing the usbTask thread.
 * @param argument: Not used
 * @retval None
 */
// デバッグ用カウンタ
volatile uint32_t dbg_usb_task_count   = 0;
volatile uint32_t dbg_audio_task_count = 0;

// stm32h7rsxx_it.cで定義されたUSB ISRカウンタとMSPモニター
extern volatile uint32_t dbg_usb_isr_count;
extern volatile uint32_t dbg_usb_isr_msp_min;
extern volatile uint32_t dbg_usb_isr_msp_start;

/* USER CODE END Header_StartUSBTask */
void StartUSBTask(void* argument)
{
    /* USER CODE BEGIN StartUSBTask */
    (void) argument;

    tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_HIGH};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    /* Infinite loop */
    for (;;)
    {
        dbg_usb_task_count++;
        // 短いタイムアウトを使用して無限ブロックを防ぐ
        tud_task();  //_ext(10, false);  // 10msタイムアウト
        // osDelay(1) は不要（tud_task_ext内で待機するため）
    }
    /* USER CODE END StartUSBTask */
}

/* USER CODE BEGIN Header_StartAudioTask */
/**
 * @brief Function implementing the audioTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAudioTask */
void StartAudioTask(void* argument)
{
    /* USER CODE BEGIN StartAudioTask */
    (void) argument;

    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 0);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    reset_audio_buffer();

    AUDIO_Init_AK4619(96000);

    /* もし、SigmaStudio+からUSBi経由で書き込み、デバッグを行う場合は
     * RESET_FROMFWを0に設定し、ここ以下の行で一旦ブレークして、
     * SigmaStudio+からダウンロードを実行すること。
     */
    AUDIO_Init_ADAU1466(48000);
    osDelay(500);

    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    start_adc();
    osDelay(100);

    start_sai();
    start_audio_control();
    osDelay(100);

    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);

    /* Infinite loop */
    for (;;)
    {
        dbg_audio_task_count++;
        audio_task();
        osDelay(1);  // 毎回1ms待機してUSBとAudioのバランスを取る
    }
    /* USER CODE END StartAudioTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
 * @brief Function implementing the ledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void* argument)
{
    /* USER CODE BEGIN StartLEDTask */
    reset_led_buffer();

    set_led_color(0, 0, 0, 0);
    renew();
    osDelay(100);

    /* Infinite loop */
    for (;;)
    {
        led_tx_blinking_task();
        led_rx_blinking_task();
        rgb_led_task();
        osDelay(10);  // LED更新は10ms間隔で十分
    }
    /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartADCTask */
/**
 * @brief Function implementing the adcTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartADCTask */
void StartADCTask(void* argument)
{
    /* USER CODE BEGIN StartADCTask */

    /* Infinite loop */
    for (;;)
    {
        ui_control_task();
        osDelay(1);
    }
    /* USER CODE END StartADCTask */
}

/* USER CODE BEGIN Header_StartOLEDTask */
/**
 * @brief Function implementing the oledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartOLEDTask */
void StartOLEDTask(void* argument)
{
    /* USER CODE BEGIN StartOLEDTask */
    (void) argument;

    OLED_Init();

    /* Infinite loop */
    for (;;)
    {
        OLED_UpdateTask();
        osDelay(20);
    }
    /* USER CODE END StartOLEDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
