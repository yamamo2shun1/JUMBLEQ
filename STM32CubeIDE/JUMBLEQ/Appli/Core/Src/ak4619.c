/*
 * ak4619.c
 *
 *  Created on: 2026/01/24
 *      Author: Shunichi Yamamoto
 */

#include "ak4619.h"

void AUDIO_Init_AK4619(uint32_t hz)
{
    uint8_t sndData[1] = {0x00};

    // AK4619 HW Reset
    HAL_GPIO_WritePin(CODEC_RESET_GPIO_Port, CODEC_RESET_Pin, 0);
    HAL_Delay(10);
    HAL_GPIO_WritePin(CODEC_RESET_GPIO_Port, CODEC_RESET_Pin, 1);
    HAL_Delay(500);

    // Power Management
    // sndData[0] = 0x36;  // 00 11 0 11 0
    // HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // Audio I/F format
    sndData[0] = 0xAC;  // 1 010 11 00 (TDM, 32bit, TDM128 I2S compatible, Falling, Slow)
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // Reset Control
    sndData[0] = 0x10;  // 000 1 00 00
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x02, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // System Clock Setting
    if (hz == 48000)
    {
        sndData[0] = 0x00;  // 00000 000 (48kHz)
    }
    else if (hz == 96000)
    {
        sndData[0] = 0x01;  // 00000 001 (96kHz)
    }
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x03, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // ADC Input Setting
    sndData[0] = 0x55;  // 01 01 01 01 (AIN1L, AIN1R, AIN4L, AIN4R)
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x0B, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // DAC Input Select Setting
    // sndData[0] = 0x0E;  // 00 00 11 10 (ADC1 -> DAC1, ADC2 -> DAC2)
    sndData[0] = 0x04;  // 00 00 01 00 (SDIN2 -> DAC2, SDIN1 -> DAC1)
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x12, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);

    // Power Management
    sndData[0] = 0x37;  // 00 11 0 11 1
    HAL_I2C_Mem_Write(&hi2c3, (0b0010001 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, sndData, sizeof(sndData), 10000);
    // HAL_I2C_Mem_Read(&hi2c3, (0b0010001 << 1) | 1, 0x00, I2C_MEMADD_SIZE_8BIT, rcvData, sizeof(rcvData), 10000);
}
