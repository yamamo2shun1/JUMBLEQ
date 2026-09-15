/*
 * ui_ch_fader_internal.h
 *
 * Private API of the magnetic-switch ch_fader module.
 */

#ifndef UI_CH_FADER_INTERNAL_H_
#define UI_CH_FADER_INTERNAL_H_

#include "main.h"
#include "ui_control.h"

// ADC完了時のch_fader処理（磁気calibration/gesture/curve/DSP/MIDI出力）。
void ui_ch_fader_process(const uint32_t* adc_samples, bool output_as_note);

// DVS遅延queueの周期service。ADC完了が無くても2ms Task周期で呼ぶ。
void ui_ch_fader_service_dsp_outputs(void);

// 現在値でDSP出力を即時再同期する。
void ui_ch_fader_reapply_outputs(void);

bool ui_ch_fader_apply_aux_assignments(uint8_t sensor2_assign, uint8_t sensor3_assign);
uint8_t ui_ch_fader_get_aux_assign(uint8_t sensor_idx);
void ui_ch_fader_apply_dvs_delay(uint8_t delay_ms);

uint8_t ui_ch_fader_curve_width_to_midi_cc(float width);
void ui_ch_fader_set_curve_width(uint8_t pair_idx, float width);
float ui_ch_fader_get_curve_width(uint8_t pair_idx);
bool ui_ch_fader_set_curve_width_from_cc(uint8_t pair_idx, uint8_t cc_value);
void ui_ch_fader_set_reverse(uint8_t pair_idx, bool enabled);
void ui_ch_fader_mark_curve_dirty(void);

bool ui_ch_fader_validate_persist(const UI_ControlPersistState_t* state);
void ui_ch_fader_capture_persist(UI_ControlPersistState_t* state);

void ui_ch_fader_reset(void);

// OLED表示用: curve幅、DVS delay、Reverseの軽量コピー。scheduler停止区間専用。
typedef struct
{
    float curve_width_a;
    float curve_width_b;
    uint8_t dvs_delay_ms;
    bool reverse_a;
    bool reverse_b;
} UI_ChFaderDisplayState_t;

void ui_ch_fader_capture_display_state(UI_ChFaderDisplayState_t* state);

#endif /* UI_CH_FADER_INTERNAL_H_ */
