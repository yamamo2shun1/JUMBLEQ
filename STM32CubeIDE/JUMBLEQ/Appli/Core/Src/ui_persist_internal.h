/*
 * ui_persist_internal.h
 *
 * UI persist contract shared by the UI owner modules and the EEPROM
 * configuration module. This is an internal contract and is not part of the
 * OLED/RTOS public UI API.
 */

#ifndef UI_PERSIST_INTERNAL_H_
#define UI_PERSIST_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>

#define UI_CH_FADER_AUX_ASSIGN_A 0U
#define UI_CH_FADER_AUX_ASSIGN_B 1U
#define UI_CH_FADER_DVS_DELAY_DEFAULT_MS 50U
#define UI_CH_FADER_DVS_DELAY_MAX_MS     120U

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

#endif /* UI_PERSIST_INTERNAL_H_ */
