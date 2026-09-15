/*
 * ui_control.h
 *
 *  Created on: Feb 18, 2026
 */

#ifndef INC_UI_CONTROL_H_
#define INC_UI_CONTROL_H_

#include "main.h"
#include <stdbool.h>

#define ADC_NUM 8
#define UI_CH_FADER_AUX_ASSIGN_A 0U
#define UI_CH_FADER_AUX_ASSIGN_B 1U
#define UI_CH_FADER_DVS_DELAY_DEFAULT_MS 50U
#define UI_CH_FADER_DVS_DELAY_MAX_MS     120U

typedef enum
{
    UI_UF2_TRANSITION_IDLE = 0,
    UI_UF2_TRANSITION_WAIT_RELEASE,
    UI_UF2_TRANSITION_WAIT_HOLD,
    UI_UF2_TRANSITION_HOLDING,
    UI_UF2_TRANSITION_CLEARING_DISPLAYS,
    UI_UF2_TRANSITION_CANCELLED,
    UI_UF2_TRANSITION_TIMED_OUT,
} UI_Uf2TransitionState_t;

typedef enum
{
    UI_INPUT_MODE_DISABLED = 0,
    UI_INPUT_MODE_DVS,
    UI_INPUT_MODE_SYNTH,
} UI_InputMode_t;

typedef struct
{
    uint8_t current_ch1_input_type;
    uint8_t current_ch2_input_type;
    uint8_t current_ch_fader_a_assign;
    uint8_t current_ch_fader_b_assign;
    uint8_t current_ch_fader_post_assign;
    uint8_t current_return_assign;
    uint8_t current_hp_out_source;
    uint8_t current_ch1_input_mode;
    uint8_t current_ch2_input_mode;
    uint8_t ch_fader_dvs_delay_ms;
    uint8_t sensor2_aux_fade_down_assign;
    uint8_t sensor3_aux_fade_down_assign;
    bool ch_fader_reverse_a;
    bool ch_fader_reverse_b;
    bool mag_out_as_note;
    float current_ch_fader_curve_width_a;
    float current_ch_fader_curve_width_b;
} UI_ControlPersistState_t;

/* Legacy EEPROM representation: Configurator 50% -> MIDI CC64 -> width 0.30771654. */
#define UI_CH_FADER_CURVE_WIDTH_A_DEFAULT (0.30771654f)
#define UI_CH_FADER_CURVE_WIDTH_B_DEFAULT (0.30771654f)

uint8_t get_current_ch_fader_a_position(void);
uint8_t get_current_ch_fader_b_position(void);
int16_t get_current_ch1_in_db(void);
int16_t get_current_ch2_in_db(void);
int16_t get_current_ch1_out_db(void);
int16_t get_current_ch2_out_db(void);
int16_t get_current_return_db(void);
bool get_current_return_enabled(void);
int16_t get_current_hp_out_db(void);
int16_t get_current_dry_wet(void);
uint8_t get_current_ch_fader_sensor2_cc_value(void);  // same value as send_control_change for ch_fader[2]
uint8_t get_current_ch_fader_sensor3_cc_value(void);  // same value as send_control_change for ch_fader[3]
char* get_current_input_typeA_str(void);
char* get_current_input_typeB_str(void);
char* get_current_input_srcA_str(void);
char* get_current_input_srcB_str(void);
char* get_current_input_srcP_str(void);
char* get_current_return_src_str(void);
char* get_current_hp_out_src_str(void);
uint8_t get_current_input_srcA_channel(void);  // 0:none, 1:CH1, 2:CH2
uint8_t get_current_input_srcB_channel(void);  // 0:none, 1:CH1, 2:CH2
bool get_current_ch1_dvs_enabled(void);
bool get_current_ch2_dvs_enabled(void);
UI_InputMode_t get_current_ch1_input_mode(void);
UI_InputMode_t get_current_ch2_input_mode(void);
uint8_t ui_control_get_ch_fader_dvs_delay_ms(void);
bool ui_control_is_ch_fader_reverse_a_enabled(void);
bool ui_control_is_ch_fader_reverse_b_enabled(void);
bool ui_control_is_curve_edit_mode_enabled(void);
UI_Uf2TransitionState_t ui_control_get_uf2_transition_state(void);
uint8_t ui_control_get_uf2_seconds_remaining(void);
void ui_control_notify_uf2_displays_cleared(void);
uint8_t ui_control_get_ch_fader_curve_a_cc(void);
uint8_t ui_control_get_ch_fader_curve_b_cc(void);
float ui_control_evaluate_ch_fader_curve_preview(uint8_t cc_value, float normalized_preview_position);

void start_adc(void);
void ui_control_task(void);
void start_audio_control(void);
bool is_started_audio_control(void);
void ui_control_get_persist_state(UI_ControlPersistState_t* state);
bool ui_control_apply_persist_state(const UI_ControlPersistState_t* state);

#endif /* INC_UI_CONTROL_H_ */
