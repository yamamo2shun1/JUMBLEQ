/*
 * adau1466.c
 *
 *  Created on: 2026/01/24
 *      Author: Shunichi Yamamoto
 */

#include "adau1466.h"

#include "SigmaStudioFW.h"
#include "oto_no_ita_dsp_ADAU146xSchematic_1.h"
#include "oto_no_ita_dsp_ADAU146xSchematic_1_Defines.h"
#include "oto_no_ita_dsp_ADAU146xSchematic_1_PARAM.h"

double convert_pot2dB(uint16_t adc_val)
{
    double x  = (double) adc_val / 1023.0;
    double db = 0.0;
    if (x < 0.7)
    {
        db = -80.0 + (x / 0.7) * 80.0;
    }
    else
    {
        db = (x - 0.7) / 0.3 * 10.0;
    }
    return db;
}

double convert_dB2gain(double db)
{
    return pow(10.0, db / 20.0);
}

void write_q8_24(const uint16_t addr, const double val)
{
    uint8_t gain_array[4] = {0x00};
    gain_array[0]         = ((uint32_t) (val * pow(2, 23)) >> 24) & 0x000000FF;
    gain_array[1]         = ((uint32_t) (val * pow(2, 23)) >> 16) & 0x000000FF;
    gain_array[2]         = ((uint32_t) (val * pow(2, 23)) >> 8) & 0x000000FF;
    gain_array[3]         = (uint32_t) (val * pow(2, 23)) & 0x000000FF;

    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, addr, 4, gain_array);
}

void AUDIO_Init_ADAU1466(uint32_t hz)
{
    // ADAU1466 HW Reset
    HAL_GPIO_WritePin(DSP_RESET_GPIO_Port, DSP_RESET_Pin, 0);
    HAL_Delay(10);
    HAL_GPIO_WritePin(DSP_RESET_GPIO_Port, DSP_RESET_Pin, 1);
    HAL_Delay(500);
#if RESET_FROM_FW
    default_download_ADAU146XSCHEMATIC_1();
    HAL_Delay(100);
#endif

    if (hz == 48000)
    {
        ADI_REG_TYPE Mode0_0[2] = {0x00, 0x06};
        ADI_REG_TYPE Mode0_1[2] = {0x00, 0x05};
        ADI_REG_TYPE Mode0_2[2] = {0x00, 0x00};
        ADI_REG_TYPE Mode0_3[2] = {0x00, 0x01};

        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, 0xF020, 2, Mode0_0); /* CLK_GEN1_M */
        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, 0xF005, 2, Mode0_1); /* MCLK_OUT */
        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, 0xF003, 2, Mode0_2); /* PLL_ENABLE */
        __DSB();
        HAL_Delay(100);
        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, 0xF003, 2, Mode0_3); /* PLL_ENABLE */
    }
    else if (hz == 96000)
    {
        ADI_REG_TYPE Mode1_0[2] = {0x00, 0x03};
        ADI_REG_TYPE Mode1_1[2] = {0x00, 0x07};
        ADI_REG_TYPE Mode1_2[2] = {0x00, 0x00};
        ADI_REG_TYPE Mode1_3[2] = {0x00, 0x01};

        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, 0xF020, 2, Mode1_0); /* CLK_GEN1_M */
        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, 0xF005, 2, Mode1_1); /* MCLK_OUT */
        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, 0xF003, 2, Mode1_2); /* PLL_ENABLE */
        __DSB();
        HAL_Delay(100);
        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, 0xF003, 2, Mode1_3); /* PLL_ENABLE */
    }

    HAL_Delay(50);  // Wait for PLL to lock and stabilize
    __DSB();
}

void set_dc_inputA(float xf_pos)
{
    write_q8_24(MOD_DCINPUT_A_DCVALUE_ADDR, xf_pos);
}

void set_dc_inputB(float xf_pos)
{
    write_q8_24(MOD_DCINPUT_B_DCVALUE_ADDR, xf_pos);
}

void control_input_from_usb_gain(uint8_t ch, int16_t db)
{
    SEGGER_RTT_printf(0, "USB CH%d Gain: %.2f dB\n", ch, db);

    const double gain = convert_dB2gain(db);

    switch (ch)
    {
    case 1:
        write_q8_24(MOD_INPUT_FROM_USB1_GAIN_ADDR, gain);
        break;
    case 2:
        write_q8_24(MOD_INPUT_FROM_USB2_GAIN_ADDR, gain);
        break;
    case 3:
        write_q8_24(MOD_INPUT_FROM_USB3_GAIN_ADDR, gain);
        break;
    case 4:
        write_q8_24(MOD_INPUT_FROM_USB4_GAIN_ADDR, gain);
        break;
    default:
        break;
    }
}

void control_input_from_ch1_gain(const uint16_t adc_val)
{
    const double db   = convert_pot2dB(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_INPUT_FROM_CH1_GAIN_ADDR, gain);
}

void control_input_from_ch2_gain(const uint16_t adc_val)
{
    const double db   = convert_pot2dB(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_INPUT_FROM_CH2_GAIN_ADDR, gain);
}

void control_send1_out_gain(const uint16_t adc_val)
{
    const double db   = convert_pot2dB(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_SEND1_OUTPUT_GAIN_ADDR, gain);
}

void control_send2_out_gain(const uint16_t adc_val)
{
    const double db   = convert_pot2dB(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_SEND2_OUTPUT_GAIN_ADDR, gain);
}

void control_dryA_out_gain(const uint16_t adc_val)
{
    const float rate = cos(pow(adc_val / 1023.0f, 2.0f) * M_PI_2);
    write_q8_24(MOD_DCINPUT_DRYA_DCVALUE_ADDR, rate);
}

void control_dryB_out_gain(const uint16_t adc_val)
{
    const float rate = cos(pow(adc_val / 1023.0f, 2.0f) * M_PI_2);
    write_q8_24(MOD_DCINPUT_DRYB_DCVALUE_ADDR, rate);
}

void control_wet_out_gain(const uint16_t adc_val)
{
    const float rate = sin(pow(adc_val / 1023.0f, 2.0f) * M_PI_2);
    write_q8_24(MOD_DCINPUT_WET_DCVALUE_ADDR, rate);
}

void control_master_out_gain(const uint16_t adc_val)
{
    const double db   = convert_pot2dB(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_MASTER_OUTPUT_GAIN_ADDR, gain);
}

void set_ch1_line()
{
    ADI_REG_TYPE Mode0_0[4] = {0x01, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4] = {0x00, 0x00, 0x00, 0x00};

    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_LN_PN_SW_1_INDEX_CHANNEL0_ADDR, 4, Mode0_0);
    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_LN_PN_SW_1_INDEX_CHANNEL1_ADDR, 4, Mode0_1);
}

void set_ch1_phono()
{
    ADI_REG_TYPE Mode0_0[4] = {0x00, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4] = {0x01, 0x00, 0x00, 0x00};

    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_LN_PN_SW_1_INDEX_CHANNEL0_ADDR, 4, Mode0_0);
    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_LN_PN_SW_1_INDEX_CHANNEL1_ADDR, 4, Mode0_1);
}

void set_ch2_line()
{
    ADI_REG_TYPE Mode0_0[4] = {0x01, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4] = {0x00, 0x00, 0x00, 0x00};

    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_LN_PN_SW_2_INDEX_CHANNEL0_ADDR, 4, Mode0_0);
    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_LN_PN_SW_2_INDEX_CHANNEL1_ADDR, 4, Mode0_1);
}

void set_ch2_phono()
{
    ADI_REG_TYPE Mode0_0[4] = {0x00, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4] = {0x01, 0x00, 0x00, 0x00};

    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_LN_PN_SW_2_INDEX_CHANNEL0_ADDR, 4, Mode0_0);
    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_LN_PN_SW_2_INDEX_CHANNEL1_ADDR, 4, Mode0_1);
}

void select_input_type(uint8_t ch, uint8_t type)
{
    if (ch == INPUT_CH1)
    {
        switch (type)
        {
        case INPUT_TYPE_LINE:
            set_ch1_line();
            break;
        case INPUT_TYPE_PHONO:
            set_ch1_phono();
            break;
        default:
            break;
        }
    }
    else if (ch == INPUT_CH2)
    {
        switch (type)
        {
        case INPUT_TYPE_LINE:
            set_ch2_line();
            break;
        case INPUT_TYPE_PHONO:
            set_ch2_phono();
            break;
        default:
            break;
        }
    }
}

void select_xf_assignA_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case INPUT_CH1:
        Mode0[3] = 0x00;
        break;
    case INPUT_CH2:
        Mode0[3] = 0x01;
        break;
    case INPUT_USB:
        Mode0[3] = 0x02;
        break;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_XF_ASSIGN_SW_A_INDEX_ADDR, 4, Mode0);
}

void select_xf_assignB_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case INPUT_CH1:
        Mode0[3] = 0x00;
        break;
    case INPUT_CH2:
        Mode0[3] = 0x01;
        break;
    case INPUT_USB:
        Mode0[3] = 0x02;
        break;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_XF_ASSIGN_SW_B_INDEX_ADDR, 4, Mode0);
}

void select_xf_assignPost_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case INPUT_CH1:
        Mode0[3] = 0x00;
        break;
    case INPUT_CH2:
        Mode0[3] = 0x01;
        break;
    case INPUT_USB:
        Mode0[3] = 0x02;
        break;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_XF_ASSIGN_SW_POST_INDEX_ADDR, 4, Mode0);
}
