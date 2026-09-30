/*
 * ak4619.c
 *
 *  Created on: 2026/01/24
 *      Author: Shunichi Yamamoto
 */

#include "ak4619.h"
#include <stdbool.h>
#include "i2c.h"
#include "cmsis_os2.h"

extern osMutexId_t i2cMutexHandle;

#define AK4619_REG_MIC_AMP_GAIN_ADC1 0x04U
#define AK4619_REG_MIC_AMP_GAIN_ADC2 0x05U
#define AK4619_REG_ADC_DIGITAL_FILTER 0x0AU
#define AK4619_MIC_GAIN_BITS_0DB     0x02U
#define AK4619_MIC_GAIN_BITS_27DB    0x0BU
#define AK4619_ADC_FILTER_SHORT_DELAY_SHARP 0x22U

static HAL_StatusTypeDef ak4619_write_reg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t sndData[1]       = {value};
    bool mutex_locked        = false;

    if (osKernelGetState() == osKernelRunning && i2cMutexHandle != NULL)
    {
        if (osMutexAcquire(i2cMutexHandle, 200U) != osOK)
        {
            return HAL_TIMEOUT;
        }
        mutex_locked = true;
    }

    for (uint8_t retry = 0; retry < 3; retry++)
    {
        status = HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), reg, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 100);
        if (status == HAL_OK)
        {
            break;
        }
        osDelay(1);
    }

    if (mutex_locked)
    {
        osMutexRelease(i2cMutexHandle);
    }

    return status;
}

static HAL_StatusTypeDef ak4619_write_mic_gain_reg(uint8_t reg, uint8_t gain_bits)
{
    const uint8_t value = (uint8_t)((gain_bits << 4) | gain_bits);

    return ak4619_write_reg(reg, value);
}

void ak4619_init(uint32_t hz)
{
    // AK4619 HW Reset
    HAL_GPIO_WritePin(CODEC_RESET_GPIO_Port, CODEC_RESET_Pin, 0);
    osDelay(10);
    HAL_GPIO_WritePin(CODEC_RESET_GPIO_Port, CODEC_RESET_Pin, 1);
    osDelay(500);

    // Power Management
    // sndData[0] = 0x36;  // 00 11 0 11 0
    // HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // Audio I/F format
    if (ak4619_write_reg(0x01, 0xAC) != HAL_OK)  // 1 010 11 00 (TDM, 32bit, TDM128 I2S compatible, Falling, Slow)
    {
        return;
    }

    // Reset Control
    if (ak4619_write_reg(0x02, 0x10) != HAL_OK)  // 000 1 00 00
    {
        return;
    }

    // System Clock Setting
    if (hz == 48000)
    {
        if (ak4619_write_reg(0x03, 0x00) != HAL_OK)  // 00000 000 (48kHz)
        {
            return;
        }
    }
    else if (hz == 96000)
    {
        if (ak4619_write_reg(0x03, 0x01) != HAL_OK)  // 00000 001 (96kHz)
        {
            return;
        }
    }

    // ADC digital volume
    if (ak4619_write_reg(0x06, 0x30) != HAL_OK)  // ADC1 Lch 0x00(24dB) -> 0x18(12dB) -> 0x30(0dB)
    {
        return;
    }
    if (ak4619_write_reg(0x07, 0x30) != HAL_OK)  // ADC1 Rch 0x00(24dB) -> 0x18(12dB) -> 0x30(0dB)
    {
        return;
    }
    if (ak4619_write_reg(0x08, 0x30) != HAL_OK)  // ADC2 Lch 0x00(24dB) -> 0x18(12dB) -> 0x30(0dB)
    {
        return;
    }
    if (ak4619_write_reg(0x09, 0x30) != HAL_OK)  // ADC2 Rch 0x00(24dB) -> 0x18(12dB) -> 0x30(0dB)
    {
        return;
    }

    // ADC1/2: Short Delay Sharp Roll-Off Filter (22/fs -> 8/fs group delay)
    if (ak4619_write_reg(AK4619_REG_ADC_DIGITAL_FILTER, AK4619_ADC_FILTER_SHORT_DELAY_SHARP) != HAL_OK)
    {
        return;
    }

    // ADC Input Setting
    if (ak4619_write_reg(0x0B, 0x55) != HAL_OK)  // 01 01 01 01 (AIN1L, AIN1R, AIN4L, AIN4R)
    {
        return;
    }

    // DAC Input Select Setting
    // sndData[0] = 0x0E;  // 00 00 11 10 (ADC1 -> DAC1, ADC2 -> DAC2)
    // Swap DAC1/DAC2 here to match the current PCB output jack order.
    if (ak4619_write_reg(0x12, 0x01) != HAL_OK)  // 00 00 00 01 (SDIN2 -> DAC1, SDIN1 -> DAC2)
    {
        return;
    }

    // DAC digital volume
    if (ak4619_write_reg(0x0E, 0x14) != HAL_OK)  // DAC1 Lch 0x14(+2dB), 0x18(0dB), 0xFF(-inf dB)
    {
        return;
    }
    if (ak4619_write_reg(0x0F, 0x14) != HAL_OK)  // DAC1 Rch 0x14(+2dB), 0x18(0dB), 0xFF(-inf dB)
    {
        return;
    }
    if (ak4619_write_reg(0x10, 0x14) != HAL_OK)  // DAC2 Lch 0x14(+2dB), 0x18(0dB), 0xFF(-inf dB)
    {
        return;
    }
    if (ak4619_write_reg(0x11, 0x14) != HAL_OK)  // DAC2 Rch 0x14(+2dB), 0x18(0dB), 0xFF(-inf dB)
    {
        return;
    }

    // Power Management
    (void) ak4619_write_reg(0x00, 0x37);  // 00 11 0 11 1
    // HAL_I2C_Mem_Read(&hi2c3, (0b0010001 << 1) | 1, 0x00, I2C_MEMADD_SIZE_8BIT, rcvData, sizeof(rcvData), 10000);
}

void ak4619_set_mic_amp_gain(uint8_t ch, uint8_t gain_db)
{
    uint8_t reg;
    uint8_t gain_bits;

    switch (ch)
    {
    case AK4619_MIC_GAIN_CH1:
        reg = AK4619_REG_MIC_AMP_GAIN_ADC1;
        break;
    case AK4619_MIC_GAIN_CH2:
        reg = AK4619_REG_MIC_AMP_GAIN_ADC2;
        break;
    default:
        return;
    }

    switch (gain_db)
    {
    case AK4619_MIC_GAIN_DB_0:
        gain_bits = AK4619_MIC_GAIN_BITS_0DB;
        break;
    case AK4619_MIC_GAIN_DB_27:
        gain_bits = AK4619_MIC_GAIN_BITS_27DB;
        break;
    default:
        return;
    }

    (void)ak4619_write_mic_gain_reg(reg, gain_bits);
}
