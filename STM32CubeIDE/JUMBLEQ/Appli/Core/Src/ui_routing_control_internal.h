/*
 * ui_routing_control_internal.h
 *
 * Private API of the routing/input-selection module. pot/ch_fader may use the
 * read-only getters; the facade applies changes and coordinates reapply order.
 */

#ifndef UI_ROUTING_CONTROL_INTERNAL_H_
#define UI_ROUTING_CONTROL_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>

#include "adau1466.h"
#include "ui_control.h"
#include "ui_persist_internal.h"

// UI/EEPROMで使う入力source ID。値は保存互換のため変更しない。
typedef enum
{
    UI_ROUTING_SOURCE_CH1_LINE = 0,
    UI_ROUTING_SOURCE_CH1_PHONO = 1,
    UI_ROUTING_SOURCE_CH2_LINE = 2,
    UI_ROUTING_SOURCE_CH2_PHONO = 3,
    UI_ROUTING_SOURCE_USB12 = 4,
    UI_ROUTING_SOURCE_USB34 = 5,
    UI_ROUTING_SOURCE_NONE = 6,
} ui_routing_source_t;

// read-only view for pot/ch_fader.
ui_routing_source_t ui_routing_get_ch_fader_assign(uint8_t pair_idx);  // 0:A, 1:B
ui_control_input_mode_t ui_routing_get_input_mode(adau1466_input_source_t audio_input_source);
ui_routing_source_t ui_routing_get_return_assign(void);
bool ui_routing_is_synth_mode_active(void);

// Audio Task向けInput Mode getter（定義はui_routing_control.c）。
ui_control_input_mode_t ui_routing_get_ch1_input_mode(void);
ui_control_input_mode_t ui_routing_get_ch2_input_mode(void);

// persist: 適用前に全項目検証する。
bool ui_routing_validate_persist(const ui_persist_state_t* state);
void ui_routing_capture_persist(ui_persist_state_t* state);

// UI_ROUTING_SOURCE値からDSP入力sourceへの変換（validate成功後にfacadeが使用）。
bool ui_routing_source_to_audio_input_source(ui_routing_source_t routing_source,
                                             adau1466_input_source_t* audio_input_source);

// facadeが既存順序で呼ぶ適用API。
void ui_routing_apply_input_type(adau1466_input_source_t audio_input_source, uint8_t input_type);
void ui_routing_apply_ch_fader_assign_a(adau1466_input_source_t audio_input_source);
void ui_routing_apply_ch_fader_assign_b(adau1466_input_source_t audio_input_source);
void ui_routing_apply_ch_fader_assign_post(adau1466_input_source_t audio_input_source);
ui_routing_source_t ui_routing_apply_return_source(ui_routing_source_t routing_source);  // 適用後のreturn sourceを返す
void ui_routing_apply_hp_out_source(adau1466_hp_source_t hp_source);
void ui_routing_apply_input_mode(adau1466_input_source_t audio_input_source, ui_control_input_mode_t mode);

void ui_routing_reset(void);

// OLED表示用: 同一時点のrouting状態から導出した表示値。
// 文字列は既存の文字列リテラルを指す。scheduler停止区間専用。
typedef struct
{
    const char* input_source_a_text;
    const char* input_source_b_text;
    const char* input_type_a_text;
    const char* input_type_b_text;
    const char* thru_source_text;
    const char* return_source_text;
    const char* hp_source_text;
    bool return_enabled;
    bool dvs_enabled;
    bool input_source_a_mode_visible;
    ui_control_input_mode_t input_source_a_mode;
    bool input_source_b_mode_visible;
    ui_control_input_mode_t input_source_b_mode;
} ui_routing_display_state_t;

void ui_routing_capture_display_state(ui_routing_display_state_t* state);

#endif /* UI_ROUTING_CONTROL_INTERNAL_H_ */
