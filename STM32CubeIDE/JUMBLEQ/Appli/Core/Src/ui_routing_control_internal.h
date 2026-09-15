/*
 * ui_routing_control_internal.h
 *
 * Private API of the routing/input-selection module. pot/ch_fader may use the
 * read-only getters; the facade applies changes and coordinates reapply order.
 */

#ifndef UI_ROUTING_CONTROL_INTERNAL_H_
#define UI_ROUTING_CONTROL_INTERNAL_H_

#include "main.h"
#include "ui_control.h"

enum
{
    INPUT_SRC_CH1_LN = 0,
    INPUT_SRC_CH1_PN,
    INPUT_SRC_CH2_LN,
    INPUT_SRC_CH2_PN,
    INPUT_SRC_USB12,
    INPUT_SRC_USB34,
    INPUT_SRC_NONE,
};

// read-only view for pot/ch_fader.
uint8_t ui_routing_get_ch_fader_assign(uint8_t pair_idx);  // 0:A, 1:B
uint8_t ui_routing_get_input_mode(uint8_t input_ch);
uint8_t ui_routing_get_return_assign(void);
bool ui_routing_is_synth_mode_active(void);

// persist: 適用前に全項目検証する。
bool ui_routing_validate_persist(const UI_ControlPersistState_t* state);
void ui_routing_capture_persist(UI_ControlPersistState_t* state);

// assign値からDSP入力channelへの変換（validate成功後にfacadeが使用）。
bool ui_routing_assign_to_input_ch(uint8_t assign, uint8_t* input_ch);
bool ui_routing_assign_to_return_input_ch(uint8_t assign, uint8_t* input_ch);

// facadeが既存順序で呼ぶ適用API。
void ui_routing_apply_input_type(uint8_t input_ch, uint8_t input_type);
void ui_routing_apply_ch_fader_assign_a(uint8_t input_ch);
void ui_routing_apply_ch_fader_assign_b(uint8_t input_ch);
void ui_routing_apply_ch_fader_assign_post(uint8_t input_ch);
uint8_t ui_routing_apply_return_source(uint8_t input_ch);  // 適用後のreturn assignを返す
void ui_routing_apply_hp_out_source(uint8_t source);
void ui_routing_apply_input_mode(uint8_t input_ch, UI_InputMode_t mode);

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
    UI_InputMode_t input_source_a_mode;
    bool input_source_b_mode_visible;
    UI_InputMode_t input_source_b_mode;
} UI_RoutingDisplayState_t;

void ui_routing_capture_display_state(UI_RoutingDisplayState_t* state);

#endif /* UI_ROUTING_CONTROL_INTERNAL_H_ */
