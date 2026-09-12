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
    INPUT_USB12,
    INPUT_USB34,
    INPUT_MASTER,
};

enum
{
    INPUT_TYPE_LINE = 0,
    INPUT_TYPE_PHONO,
};

enum
{
    CH1_LINE             = 0,
    CH1_PHONO            = 1,
    CH2_LINE             = 2,
    CH2_PHONO            = 3,
    CH_FADER_ASSIGN_A_CH1      = 4,
    CH_FADER_ASSIGN_A_CH2      = 5,
    CH_FADER_ASSIGN_A_USB12    = 6,
    CH_FADER_ASSIGN_A_USB34    = 7,
    CH_FADER_ASSIGN_B_CH1      = 8,
    CH_FADER_ASSIGN_B_CH2      = 9,
    CH_FADER_ASSIGN_B_USB12    = 10,
    CH_FADER_ASSIGN_B_USB34    = 11,
    CH_FADER_ASSIGN_POST_CH1   = 12,
    CH_FADER_ASSIGN_POST_CH2   = 13,
    CH_FADER_ASSIGN_POST_USB12 = 14,
    CH_FADER_ASSIGN_POST_USB34 = 15,
    CH1_MODE_DISABLE           = 16,
    CH1_MODE_DVS               = 17,
    CH1_MODE_SYNTH             = 18,
    CH2_MODE_DISABLE           = 19,
    CH2_MODE_DVS               = 20,
    CH2_MODE_SYNTH             = 21,
    RETURN_CH_USB12            = 22,
    RETURN_CH_USB34            = 23,
    RETURN_CH_NONE             = 24,
    HP_OUT_CH_FADER_A          = 25,
    HP_OUT_CH_FADER_B          = 26,
    HP_OUT_THRU                = 27,
    HP_OUT_MASTER              = 28,
    CH_FADER_AUX_SENSOR2_TO_A  = 29,
    CH_FADER_AUX_SENSOR2_TO_B  = 30,
    CH_FADER_AUX_SENSOR3_TO_A  = 31,
    CH_FADER_AUX_SENSOR3_TO_B  = 32,
    CH_FADER_REVERSE_A_OFF     = 33,
    CH_FADER_REVERSE_A_ON      = 34,
    CH_FADER_REVERSE_B_OFF     = 35,
    CH_FADER_REVERSE_B_ON      = 36,
    SYNTH_RATIO_OCTAVE         = 37,
    SYNTH_RATIO_HARMONIC       = 38,
    SYNTH_RATIO_CHORD          = 39,
    SYNTH_WARP_CLEAN           = 40,
    SYNTH_WARP_CROSSFOLD       = 41,
    SYNTH_WARP_RING_MOD        = 42,
    SYNTH_WARP_COMPARATOR      = 43,
};

enum
{
    CUE_SEL_CH_FADER_A = 0,
    CUE_SEL_CH_FADER_B = 1,
    CUE_SEL_THRU = 2,
	CUE_SEL_MST  = 3,
};

// 10-bit POTs on the muxed ADC can stop slightly short of full-scale on hardware.
#define POT_10BIT_ADC_MAX            1023U
#define POT_10BIT_MIN_DEADZONE      10U
#define POT_10BIT_DB_MAX_SNAP_START 1005U
#define POT_10BIT_DW_MAX_SNAP_START 1005U

double convert_pot2dB(uint16_t adc_val);
int16_t convert_pot2dB_int(uint16_t adc_val);

void AUDIO_Init_ADAU1466(uint32_t hz);
bool AUDIO_Update_ADAU1466_SampleRate(uint32_t hz);

void set_dc_inputA(float ch_fader_position);
void set_dc_inputB(float ch_fader_position);
void safeload_write_q8_24(uint16_t addr, uint8_t mem_page, double val);

void control_input_from_usb_gain(uint8_t ch, int16_t db);
void control_input_from_ch1_gain(const uint16_t adc_val);
void control_input_from_ch2_gain(const uint16_t adc_val);
void control_input_from_return_gain(const uint16_t adc_val);
void mute_input_from_return(void);

void control_send1_out_gain(const uint16_t adc_val);
void control_send2_out_gain(const uint16_t adc_val);

void control_dryA_out_gain(const uint16_t adc_val);
void control_dryB_out_gain(const uint16_t adc_val);

void control_wet_out_gain(const uint16_t adc_val);
void control_ch1_out_gain(const uint16_t adc_val);
void control_ch2_out_gain(const uint16_t adc_val);
void control_hp_out_gain(const uint16_t adc_val);

void select_input_type(uint8_t ch, uint8_t type);
void set_input_insert_enabled(uint8_t ch, bool enabled);
void select_send_source(uint8_t ch, bool select_insert);

void select_ch_fader_assign_a_source(uint8_t ch);
void select_ch_fader_assign_b_source(uint8_t ch);
void select_ch_fader_assign_post_source(uint8_t ch);
void select_return_ch_source(uint8_t ch);
void select_hp_out_source(uint8_t ch);

#endif /* INC_ADAU1466_H_ */
