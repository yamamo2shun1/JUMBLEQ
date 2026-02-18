/*
 * adau1466.h
 *
 *  Created on: 2026/01/24
 *      Author: Shunichi Yamamoto
 */

#ifndef INC_ADAU1466_H_
#define INC_ADAU1466_H_

#include "main.h"
#include <stdbool.h>

enum
{
    INPUT_CH1 = 0,
    INPUT_CH2,
    INPUT_USB,
};

enum
{
    INPUT_TYPE_LINE = 0,
    INPUT_TYPE_PHONO,
};

enum
{
    CH1_LINE = 0,
    CH1_PHONO,
    CH2_LINE,
    CH2_PHONO,
    XF_ASSIGN_A_CH1,
    XF_ASSIGN_A_CH2,
    XF_ASSIGN_A_USB,
    XF_ASSIGN_B_CH1,
    XF_ASSIGN_B_CH2,
    XF_ASSIGN_B_USB,
    XF_ASSIGN_POST_CH1,
    XF_ASSIGN_POST_CH2,
    XF_ASSIGN_POST_USB,
};

double convert_pot2dB(uint16_t adc_val);
int16_t convert_pot2dB_int(uint16_t adc_val);

void AUDIO_Init_ADAU1466(uint32_t hz);
bool AUDIO_Update_ADAU1466_SampleRate(uint32_t hz);

void set_dc_inputA(float xf_pos);
void set_dc_inputB(float xf_pos);

void control_input_from_usb_gain(uint8_t ch, int16_t db);
void control_input_from_ch1_gain(const uint16_t adc_val);
void control_input_from_ch2_gain(const uint16_t adc_val);

void control_send1_out_gain(const uint16_t adc_val);
void control_send2_out_gain(const uint16_t adc_val);

void control_dryA_out_gain(const uint16_t adc_val);
void control_dryB_out_gain(const uint16_t adc_val);

void control_wet_out_gain(const uint16_t adc_val);
void control_master_out_gain(const uint16_t adc_val);

void select_input_type(uint8_t ch, uint8_t type);

void select_xf_assignA_source(uint8_t ch);
void select_xf_assignB_source(uint8_t ch);
void select_xf_assignPost_source(uint8_t ch);

#endif /* INC_ADAU1466_H_ */
