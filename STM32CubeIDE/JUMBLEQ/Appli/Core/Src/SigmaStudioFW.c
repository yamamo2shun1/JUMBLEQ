/*
 * SigmaStudioFW.c
 *
 *  Created on: Aug 8, 2025
 *      Author: shun
 */

#include "SigmaStudioFW.h"

#include "spi.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "semphr.h"
#include "task.h"

extern osMutexId_t spiMutexHandle;
extern osSemaphoreId_t spiTxBinarySemHandle;
extern osSemaphoreId_t spiTxRxBinarySemHandle;

// 静的バッファ（IT転送中にスコープ外にならないようにするため）
static uint8_t spi_tx_buf[16];

volatile uint32_t sigma_spi_it_write_calls          = 0;
volatile uint32_t sigma_spi_it_write_errors         = 0;
volatile uint32_t sigma_spi_it_write_timeouts       = 0;
volatile uint32_t sigma_spi_it_mutex_timeouts       = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi == &hspi5)
    {
        osSemaphoreRelease(spiTxBinarySemHandle);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi == &hspi5)
    {
        osSemaphoreRelease(spiTxRxBinarySemHandle);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi == &hspi5)
    {
        osSemaphoreRelease(spiTxBinarySemHandle);
        osSemaphoreRelease(spiTxRxBinarySemHandle);
    }
}

void SIGMA_WRITE_REGISTER_BLOCK(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData)
{
    // Use static buffer to avoid stack overflow
    static uint8_t data[2560];

    data[0] = devAddress;
    data[1] = (uint8_t) ((address >> 8) & 0x00FF);
    data[2] = (uint8_t) (address & 0x00FF);
    for (int i = 0; i < length; i++)
    {
        data[i + 3] = pData[i];
    }
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi5, data, 1 + 2 + length, 100);

    if (status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "[%X] spi write error\n", address);
    }
}

void SIGMA_WRITE_REGISTER_BLOCK_IT(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData)
{
    sigma_spi_it_write_calls++;

    if (spiMutexHandle == NULL || spiTxBinarySemHandle == NULL)
    {
        SEGGER_RTT_printf(0, "[%X] spi not initialized\n", address);
        sigma_spi_it_write_errors++;
        return;
    }

    // ミューテックスで排他制御（最大200ms待機）
    if (osMutexAcquire(spiMutexHandle, pdMS_TO_TICKS(200)) == osOK)
    {
        spi_tx_buf[0] = devAddress;
        spi_tx_buf[1] = (uint8_t) ((address >> 8) & 0x00FF);
        spi_tx_buf[2] = (uint8_t) (address & 0x00FF);
        for (int i = 0; i < length; i++)
        {
            spi_tx_buf[i + 3] = pData[i];
        }

        while (osSemaphoreAcquire(spiTxBinarySemHandle, 0) == osOK)
        {}
        HAL_StatusTypeDef status = HAL_SPI_Transmit_IT(&hspi5, spi_tx_buf, 1 + 2 + length);
        if (status == HAL_OK)
        {
            // 送信完了を待機（最大100ms）- CPUを解放して他タスクに譲る
            if (osSemaphoreAcquire(spiTxBinarySemHandle, pdMS_TO_TICKS(100)) != osOK)
            {
                SEGGER_RTT_printf(0, "[%X] spi write timeout\n", address);
                sigma_spi_it_write_timeouts++;
            }
        }
        else
        {
            SEGGER_RTT_printf(0, "[%X] spi write error\n", address);
            sigma_spi_it_write_errors++;
        }

        osMutexRelease(spiMutexHandle);
    }
    else
    {
        SEGGER_RTT_printf(0, "[%X] spi mutex timeout\n", address);
        sigma_spi_it_mutex_timeouts++;
    }
}

void SIGMA_SAFELOAD_WRITE_DATA(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData)
{
    // Use static buffer to avoid stack overflow
    static uint8_t data[64];  // SAFELOAD typically uses small data size

    // スケジューラ起動前はポーリングモード、起動後はFreeRTOS同期を使用
    if (osKernelGetState() != osKernelRunning || spiMutexHandle == NULL)
    {
        // ポーリングモード（初期化時用）
        data[0] = devAddress;
        data[1] = (uint8_t) ((dataAddress >> 8) & 0x00FF);
        data[2] = (uint8_t) (dataAddress & 0x00FF);
        for (int i = 0; i < length; i++)
        {
            data[i + 3] = pData[i];
        }
        HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi5, data, 1 + 2 + length, 100);
        if (status != HAL_OK)
        {
            SEGGER_RTT_printf(0, "SAFELOAD::[%X] spi write error\n", dataAddress);
        }
        return;
    }

    // FreeRTOSモード
    if (osMutexAcquire(spiMutexHandle, pdMS_TO_TICKS(200)) == osOK)
    {
        data[0] = devAddress;
        data[1] = (uint8_t) ((dataAddress >> 8) & 0x00FF);
        data[2] = (uint8_t) (dataAddress & 0x00FF);
        for (int i = 0; i < length; i++)
        {
            data[i + 3] = pData[i];
        }

        HAL_StatusTypeDef status = HAL_SPI_Transmit_IT(&hspi5, data, 1 + 2 + length);
        if (status == HAL_OK)
        {
            if (osSemaphoreAcquire(spiTxBinarySemHandle, pdMS_TO_TICKS(100)) != osOK)
            {
                SEGGER_RTT_printf(0, "SAFELOAD::[%X] spi write timeout\n", dataAddress);
            }
        }
        else
        {
            SEGGER_RTT_printf(0, "SAFELOAD::[%X] spi write error\n", dataAddress);
        }

        osMutexRelease(spiMutexHandle);
    }
}

void SIGMA_WRITE_DELAY(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData)
{
    HAL_Delay(15);
}

void SIGMA_READ_REGISTER(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData)
{
    // SPI Read用の静的バッファ（IT転送中にスコープ外にならないようにするため）
    static uint8_t tx_buf[64];
    static uint8_t rx_buf[64];

    // バッファサイズチェック
    if (length > 64 - 3)
    {
        SEGGER_RTT_printf(0, "[%X] spi read error: length too large\n", address);
        return;
    }

    if (spiMutexHandle == NULL || spiTxRxBinarySemHandle == NULL)
    {
        SEGGER_RTT_printf(0, "[%X] spi not initialized\n", address);
        return;
    }

    // ミューテックスで排他制御（最大200ms待機）
    if (osMutexAcquire(spiMutexHandle, pdMS_TO_TICKS(200)) == osOK)
    {
        // ADAU1466 SPI Read: Chip Address with R/W bit = 1 (read)
        // Format: [Chip Addr | 0x01] [Addr High] [Addr Low] [Dummy bytes for read]

        // Set R/W bit to 1 for read operation (bit 0)
        tx_buf[0] = devAddress | 0x01;
        tx_buf[1] = (uint8_t) ((address >> 8) & 0x00FF);
        tx_buf[2] = (uint8_t) (address & 0x00FF);
        // Send dummy bytes (0x00) during read phase to clock out data from ADAU1466
        for (int i = 0; i < length; i++)
        {
            tx_buf[i + 3] = 0x00;
        }

        while (osSemaphoreAcquire(spiTxRxBinarySemHandle, 0) == osOK)
        {}
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_IT(&hspi5, tx_buf, rx_buf, 1 + 2 + length);

        if (status == HAL_OK)
        {
            // 送受信完了を待機（最大100ms）- CPUを解放して他タスクに譲る
            if (osSemaphoreAcquire(spiTxRxBinarySemHandle, pdMS_TO_TICKS(100)) == osOK)
            {
                for (int i = 0; i < length; i++)
                {
                    pData[i] = rx_buf[i + 3];
                }
#if 0
                SEGGER_RTT_printf(0, "[%04X] Read: ", address);
                for (int i = 0; i < length; i++)
                {
                    SEGGER_RTT_printf(0, "%02X ", pData[i]);
                }
                SEGGER_RTT_printf(0, "\n");
#endif
            }
            else
            {
                SEGGER_RTT_printf(0, "[%X] spi read timeout\n", address);
            }
        }
        else
        {
            SEGGER_RTT_printf(0, "[%X] spi read error: %d\n", address, status);
        }

        osMutexRelease(spiMutexHandle);
    }
    else
    {
        SEGGER_RTT_printf(0, "[%X] spi mutex timeout\n", address);
    }
}
