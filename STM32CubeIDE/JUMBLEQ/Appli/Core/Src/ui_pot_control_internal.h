/*
 * ui_pot_control_internal.h
 *
 * Private API of the pot/MUX module. ADC samples are received as a read-only
 * view; routing values are passed by value.
 */

#ifndef UI_POT_CONTROL_INTERNAL_H_
#define UI_POT_CONTROL_INTERNAL_H_

#include "main.h"
#include "ui_control.h"

// start_adc()のMUX初期化直後に、初期スキャンchannelを設定する。
void ui_pot_control_set_initial_channel(void);

// ADC完了ごとの1channelスキャンとDSP/MIDI反映。
void ui_pot_control_process(const uint32_t adc_samples[ADC_NUM],
                            bool synth_mode_active,
                            bool output_as_note,
                            uint8_t return_assign);

// Dry/Wet、Return、HP等のpot出力再適用。
void ui_pot_control_reapply_outputs(bool synth_mode_active, bool output_as_note, uint8_t return_assign);

// Return source変更後のReturn gainとDry/Wet再適用。
void ui_pot_control_apply_return_outputs(uint8_t return_assign);

// 起動時の初期状態へ戻す。
void ui_pot_control_reset(void);

// OLED表示用: 6個のpot生値(ADC値)を軽量コピーする。scheduler停止区間専用。
typedef struct
{
    uint16_t ch1_input;
    uint16_t ch2_input;
    uint16_t ch1_output;
    uint16_t ch2_output;
    uint16_t return_input;
    uint16_t hp_output;
} UI_PotDisplayState_t;

void ui_pot_control_capture_display_state(UI_PotDisplayState_t* state);

#endif /* UI_POT_CONTROL_INTERNAL_H_ */
