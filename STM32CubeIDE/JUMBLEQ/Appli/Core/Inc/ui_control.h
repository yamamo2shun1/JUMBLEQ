/*
 * ui_control.h
 *
 *  Created on: Feb 18, 2026
 */

#ifndef INC_UI_CONTROL_H_
#define INC_UI_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

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

// OLED表示用の1フレーム分の状態。値のコピーであり、UI内部状態への
// pointerやEEPROM用UI_ControlPersistState_tとは兼用しない。
typedef struct
{
    bool curve_edit_mode;
    UI_Uf2TransitionState_t uf2_transition_state;
    uint8_t uf2_seconds_remaining;

    uint32_t sample_rate_hz;
    int16_t ch1_input_db;
    int16_t ch2_input_db;
    int16_t ch1_output_db;
    int16_t ch2_output_db;
    int16_t return_db;
    int16_t hp_output_db;
    bool return_enabled;

    uint8_t ch_fader_curve_a_cc;
    uint8_t ch_fader_curve_b_cc;
    uint8_t ch_fader_dvs_delay_ms;
    bool dvs_enabled;
    bool ch_fader_reverse_a;
    bool ch_fader_reverse_b;

    const char* input_source_a_text;
    const char* input_source_b_text;
    const char* input_type_a_text;
    const char* input_type_b_text;
    const char* thru_source_text;
    const char* return_source_text;
    const char* hp_source_text;

    bool input_source_a_mode_visible;
    UI_InputMode_t input_source_a_mode;
    bool input_source_b_mode_visible;
    UI_InputMode_t input_source_b_mode;
} UI_DisplaySnapshot_t;

// OLED Task専用。1回の呼出で一貫した時点の表示状態を取得する。
bool ui_control_get_display_snapshot(UI_DisplaySnapshot_t* snapshot);
void ui_control_notify_uf2_displays_cleared(void);
float ui_control_evaluate_ch_fader_curve_preview(uint8_t cc_value, float normalized_preview_position);

void start_adc(void);
void ui_control_task(void);
void ui_control_enable_runtime_processing(void);

#endif /* INC_UI_CONTROL_H_ */
