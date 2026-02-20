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
#define TASK_INIT_DONE_TARGET_COUNT 4U  /* USB, Audio, LED, ADC */

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
static volatile uint32_t s_task_init_done_count = 0U;

static void mark_task_init_done(void)
{
    taskENTER_CRITICAL();
    if (s_task_init_done_count < TASK_INIT_DONE_TARGET_COUNT)
    {
        s_task_init_done_count++;
    }
    taskEXIT_CRITICAL();
}
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
    .priority   = (osPriority_t) osPriorityRealtime,
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
    // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void* argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;)
    {
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
/* USER CODE END Header_StartUSBTask */
void StartUSBTask(void* argument)
{
    /* USER CODE BEGIN StartUSBTask */
    (void) argument;

    tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_HIGH};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    mark_task_init_done();

    uint32_t usb_loop_count      = 0;
    uint32_t usb_ready_count     = 0;
    uint32_t usb_not_ready_count = 0;
    uint32_t usb_lag_count       = 0;
    uint32_t usb_max_gap_ms      = 0;
    uint32_t log_last_tick       = HAL_GetTick();
    uint32_t prev_tick           = log_last_tick;

    /* Infinite loop */
    for (;;)
    {
        uint32_t now = HAL_GetTick();
        uint32_t dt  = now - prev_tick;
        prev_tick    = now;

        if (dt > usb_max_gap_ms)
        {
            usb_max_gap_ms = dt;
        }
        if (dt >= 3)
        {
            usb_lag_count++;
        }

        if (tud_task_event_ready())
        {
            usb_ready_count++;
        }
        else
        {
            usb_not_ready_count++;
        }

        tud_task_ext(1, false);
        usb_loop_count++;

        if (0 && (now - log_last_tick >= 1000))
        {
            SEGGER_RTT_printf(0, "[USB] loop=%lu ready=%lu idle=%lu lag=%lu max_gap=%lu\r\n", (unsigned long) usb_loop_count, (unsigned long) usb_ready_count, (unsigned long) usb_not_ready_count, (unsigned long) usb_lag_count, (unsigned long) usb_max_gap_ms);

            usb_loop_count      = 0;
            usb_ready_count     = 0;
            usb_not_ready_count = 0;
            usb_lag_count       = 0;
            usb_max_gap_ms      = 0;
            log_last_tick       = now;
        }
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
    AUDIO_LoadAndApplyRoutingFromEEPROM();
    osDelay(500);

    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    start_sai();
    start_audio_control();
    osDelay(100);

    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
    mark_task_init_done();

    /* Infinite loop */
    for (;;)
    {
        audio_task();
        osDelay(1);
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
    mark_task_init_done();

    /* Infinite loop */
    for (;;)
    {
        led_tx_blinking_task();
        led_rx_blinking_task();
        rgb_led_task();
        osDelay(5);
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
    (void) argument;

    start_adc();
    osDelay(100);
    mark_task_init_done();

    /* Infinite loop */
    for (;;)
    {
        ui_control_task();
        osDelay(2);
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
    OLED_ShowInitStatus("Waiting tasks...");

    while (s_task_init_done_count < TASK_INIT_DONE_TARGET_COUNT)
    {
        osDelay(20);
    }

    OLED_ShowInitStatus("Init complete");
    osDelay(200);

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
