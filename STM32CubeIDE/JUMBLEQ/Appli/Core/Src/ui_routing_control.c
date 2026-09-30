/*
 * ui_routing_control.c
 *
 * Product routing state: input type, ch_fader A/B/Post assignment, Return,
 * Headphone source and Input Mode. Applies the corresponding codec/DSP
 * selection; multi-module reapply order is coordinated by ui_control.c.
 */

#include "ui_routing_control_internal.h"

#include "ak4619.h"
#include "adau1466.h"

#include <stddef.h>

typedef struct
{
    uint8_t current_ch1_input_type;
    uint8_t current_ch2_input_type;
    ui_routing_source_t current_ch_fader_a_assign;
    ui_routing_source_t current_ch_fader_b_assign;
    ui_routing_source_t current_ch_fader_post_assign;
    ui_routing_source_t current_return_assign;
    adau1466_hp_source_t current_hp_out_source;
    uint8_t current_ch1_input_mode;
    uint8_t current_ch2_input_mode;
} ui_routing_state_t;

static ui_routing_state_t s_routing = {
    .current_ch1_input_type = INPUT_TYPE_LINE,
    .current_ch2_input_type = INPUT_TYPE_LINE,
    .current_ch_fader_a_assign     = UI_ROUTING_SOURCE_CH1_LINE,
    .current_ch_fader_b_assign     = UI_ROUTING_SOURCE_CH2_LINE,
    .current_ch_fader_post_assign  = UI_ROUTING_SOURCE_USB12,
    .current_return_assign  = UI_ROUTING_SOURCE_USB34,
    .current_hp_out_source  = HP_SOURCE_MASTER,
    .current_ch1_input_mode = UI_INPUT_MODE_DISABLED,
    .current_ch2_input_mode = UI_INPUT_MODE_DISABLED,
};

static ui_routing_source_t input_src_from_channel_type(adau1466_input_source_t audio_input_source,
                                                      uint8_t input_type)
{
    switch (audio_input_source)
    {
    case AUDIO_INPUT_SOURCE_CH1:
        return (input_type == INPUT_TYPE_PHONO) ? UI_ROUTING_SOURCE_CH1_PHONO
                                                : UI_ROUTING_SOURCE_CH1_LINE;
    case AUDIO_INPUT_SOURCE_CH2:
        return (input_type == INPUT_TYPE_PHONO) ? UI_ROUTING_SOURCE_CH2_PHONO
                                                : UI_ROUTING_SOURCE_CH2_LINE;
    case AUDIO_INPUT_SOURCE_USB12:
        return UI_ROUTING_SOURCE_USB12;
    case AUDIO_INPUT_SOURCE_USB34:
        return UI_ROUTING_SOURCE_USB34;
    default:
        return UI_ROUTING_SOURCE_NONE;
    }
}

static ui_routing_source_t current_input_src_from_channel(adau1466_input_source_t audio_input_source)
{
    switch (audio_input_source)
    {
    case AUDIO_INPUT_SOURCE_CH1:
        return input_src_from_channel_type(AUDIO_INPUT_SOURCE_CH1, s_routing.current_ch1_input_type);
    case AUDIO_INPUT_SOURCE_CH2:
        return input_src_from_channel_type(AUDIO_INPUT_SOURCE_CH2, s_routing.current_ch2_input_type);
    case AUDIO_INPUT_SOURCE_USB12:
        return UI_ROUTING_SOURCE_USB12;
    case AUDIO_INPUT_SOURCE_USB34:
        return UI_ROUTING_SOURCE_USB34;
    default:
        return UI_ROUTING_SOURCE_NONE;
    }
}

static void replace_assign_for_input_channel(ui_routing_source_t* routing_source,
                                             adau1466_input_source_t audio_input_source,
                                             ui_routing_source_t new_source)
{
    const ui_routing_source_t line_source =
        input_src_from_channel_type(audio_input_source, INPUT_TYPE_LINE);
    const ui_routing_source_t phono_source =
        input_src_from_channel_type(audio_input_source, INPUT_TYPE_PHONO);

    if ((*routing_source == line_source) || (*routing_source == phono_source))
    {
        *routing_source = new_source;
    }
}

static void apply_mic_gain_amp_setting(adau1466_input_source_t audio_input_source, uint8_t input_type)
{
    uint8_t codec_ch;
    uint8_t gain_db;

    switch (audio_input_source)
    {
    case AUDIO_INPUT_SOURCE_CH1:
        codec_ch = AK4619_MIC_GAIN_CH1;
        break;
    case AUDIO_INPUT_SOURCE_CH2:
        codec_ch = AK4619_MIC_GAIN_CH2;
        break;
    default:
        return;
    }

    switch (input_type)
    {
    case INPUT_TYPE_LINE:
        gain_db = AK4619_MIC_GAIN_DB_0;
        break;
    case INPUT_TYPE_PHONO:
        gain_db = AK4619_MIC_GAIN_DB_27;
        break;
    default:
        return;
    }

    ak4619_set_mic_amp_gain(codec_ch, gain_db);
}

void ui_routing_apply_input_type(adau1466_input_source_t audio_input_source, uint8_t input_type)
{
    const ui_routing_source_t new_source =
        input_src_from_channel_type(audio_input_source, input_type);

    adau1466_select_input_type(audio_input_source, input_type);
    apply_mic_gain_amp_setting(audio_input_source, input_type);

    if (audio_input_source == AUDIO_INPUT_SOURCE_CH1)
    {
        s_routing.current_ch1_input_type = input_type;
    }
    else if (audio_input_source == AUDIO_INPUT_SOURCE_CH2)
    {
        s_routing.current_ch2_input_type = input_type;
    }

    replace_assign_for_input_channel(&s_routing.current_ch_fader_a_assign,
                                     audio_input_source, new_source);
    replace_assign_for_input_channel(&s_routing.current_ch_fader_b_assign,
                                     audio_input_source, new_source);
    replace_assign_for_input_channel(&s_routing.current_ch_fader_post_assign,
                                     audio_input_source, new_source);
}

static bool is_usb_assign(ui_routing_source_t routing_source)
{
    return (routing_source == UI_ROUTING_SOURCE_USB12) ||
           (routing_source == UI_ROUTING_SOURCE_USB34);
}

static bool input_mode_uses_insert(uint8_t mode)
{
    return (mode == UI_INPUT_MODE_DVS) || (mode == UI_INPUT_MODE_SYNTH);
}

static void apply_send_source_selection(adau1466_input_source_t audio_input_source)
{
    if (audio_input_source == AUDIO_INPUT_SOURCE_CH1)
    {
        const bool select_insert =
            input_mode_uses_insert(s_routing.current_ch1_input_mode) ||
            is_usb_assign(s_routing.current_ch_fader_a_assign);
        adau1466_select_send_source(AUDIO_INPUT_SOURCE_CH1, select_insert);
    }
    else if (audio_input_source == AUDIO_INPUT_SOURCE_CH2)
    {
        const bool select_insert =
            input_mode_uses_insert(s_routing.current_ch2_input_mode) ||
            is_usb_assign(s_routing.current_ch_fader_b_assign);
        adau1466_select_send_source(AUDIO_INPUT_SOURCE_CH2, select_insert);
    }
}

void ui_routing_apply_ch_fader_assign_a(adau1466_input_source_t audio_input_source)
{
    adau1466_select_ch_fader_assign_a_source(audio_input_source);
    s_routing.current_ch_fader_a_assign = current_input_src_from_channel(audio_input_source);
    apply_send_source_selection(AUDIO_INPUT_SOURCE_CH1);
}

void ui_routing_apply_ch_fader_assign_b(adau1466_input_source_t audio_input_source)
{
    adau1466_select_ch_fader_assign_b_source(audio_input_source);
    s_routing.current_ch_fader_b_assign = current_input_src_from_channel(audio_input_source);
    apply_send_source_selection(AUDIO_INPUT_SOURCE_CH2);
}

void ui_routing_apply_ch_fader_assign_post(adau1466_input_source_t audio_input_source)
{
    adau1466_select_ch_fader_assign_post_source(audio_input_source);
    s_routing.current_ch_fader_post_assign = current_input_src_from_channel(audio_input_source);
}

void ui_routing_apply_hp_out_source(adau1466_hp_source_t hp_source)
{
    if (hp_source > HP_SOURCE_MASTER)
    {
        return;
    }

    adau1466_select_hp_out_source(hp_source);
    s_routing.current_hp_out_source = hp_source;
}

void ui_routing_apply_input_mode(adau1466_input_source_t audio_input_source, ui_control_input_mode_t mode)
{
    const bool enable_insert = input_mode_uses_insert((uint8_t) mode);

    adau1466_set_input_insert_enabled(audio_input_source, enable_insert);
    if (audio_input_source == AUDIO_INPUT_SOURCE_CH1)
    {
        s_routing.current_ch1_input_mode = (uint8_t) mode;
    }
    else if (audio_input_source == AUDIO_INPUT_SOURCE_CH2)
    {
        s_routing.current_ch2_input_mode = (uint8_t) mode;
    }
    apply_send_source_selection(audio_input_source);
}

static bool routing_source_to_return_audio_input_source(ui_routing_source_t routing_source,
                                                       adau1466_input_source_t* audio_input_source)
{
    if (audio_input_source == NULL)
    {
        return false;
    }

    switch (routing_source)
    {
    case UI_ROUTING_SOURCE_USB12:
        *audio_input_source = AUDIO_INPUT_SOURCE_USB12;
        return true;
    case UI_ROUTING_SOURCE_USB34:
        *audio_input_source = AUDIO_INPUT_SOURCE_USB34;
        return true;
    default:
        return false;
    }
}

ui_routing_source_t ui_routing_apply_return_source(ui_routing_source_t routing_source)
{
    adau1466_input_source_t audio_input_source;

    if (!routing_source_to_return_audio_input_source(routing_source, &audio_input_source))
    {
        s_routing.current_return_assign = UI_ROUTING_SOURCE_NONE;
        adau1466_mute_input_from_return();
        return UI_ROUTING_SOURCE_NONE;
    }

    adau1466_select_return_ch_source(audio_input_source);
    s_routing.current_return_assign = routing_source;
    return routing_source;
}

bool ui_routing_source_to_audio_input_source(ui_routing_source_t routing_source,
                                             adau1466_input_source_t* audio_input_source)
{
    if (audio_input_source == NULL)
    {
        return false;
    }

    switch (routing_source)
    {
    case UI_ROUTING_SOURCE_CH1_LINE:
    case UI_ROUTING_SOURCE_CH1_PHONO:
        *audio_input_source = AUDIO_INPUT_SOURCE_CH1;
        return true;
    case UI_ROUTING_SOURCE_CH2_LINE:
    case UI_ROUTING_SOURCE_CH2_PHONO:
        *audio_input_source = AUDIO_INPUT_SOURCE_CH2;
        return true;
    case UI_ROUTING_SOURCE_USB12:
        *audio_input_source = AUDIO_INPUT_SOURCE_USB12;
        return true;
    case UI_ROUTING_SOURCE_USB34:
        *audio_input_source = AUDIO_INPUT_SOURCE_USB34;
        return true;
    default:
        return false;
    }
}

ui_routing_source_t ui_routing_get_ch_fader_assign(uint8_t pair_idx)
{
    return (pair_idx == 0U) ? s_routing.current_ch_fader_a_assign
                            : s_routing.current_ch_fader_b_assign;
}

ui_control_input_mode_t ui_routing_get_input_mode(adau1466_input_source_t audio_input_source)
{
    return (audio_input_source == AUDIO_INPUT_SOURCE_CH1)
               ? (ui_control_input_mode_t) s_routing.current_ch1_input_mode
               : (ui_control_input_mode_t) s_routing.current_ch2_input_mode;
}

ui_routing_source_t ui_routing_get_return_assign(void)
{
    return s_routing.current_return_assign;
}

bool ui_routing_is_synth_mode_active(void)
{
    return (s_routing.current_ch1_input_mode == UI_INPUT_MODE_SYNTH) ||
           (s_routing.current_ch2_input_mode == UI_INPUT_MODE_SYNTH);
}

static char* get_current_input_type_a_str(void)
{
    switch (s_routing.current_ch_fader_a_assign)
    {
    case UI_ROUTING_SOURCE_CH1_LINE:
    case UI_ROUTING_SOURCE_CH2_LINE:
        return "[line]";
    case UI_ROUTING_SOURCE_CH1_PHONO:
    case UI_ROUTING_SOURCE_CH2_PHONO:
        return "[phono]";
    case UI_ROUTING_SOURCE_USB12:
        return "[1/2]";
    case UI_ROUTING_SOURCE_USB34:
        return "[3/4]";
    default:
        return "[]";
    }
}

static char* get_current_input_type_b_str(void)
{
    switch (s_routing.current_ch_fader_b_assign)
    {
    case UI_ROUTING_SOURCE_CH1_LINE:
    case UI_ROUTING_SOURCE_CH2_LINE:
        return " [line]";
    case UI_ROUTING_SOURCE_CH1_PHONO:
    case UI_ROUTING_SOURCE_CH2_PHONO:
        return "[phono]";
    case UI_ROUTING_SOURCE_USB12:
        return "  [1/2]";
    case UI_ROUTING_SOURCE_USB34:
        return "  [3/4]";
    default:
        return "     []";
    }
}

static char* get_current_input_src_a_str(void)
{
    switch (s_routing.current_ch_fader_a_assign)
    {
    case UI_ROUTING_SOURCE_CH1_LINE:
    case UI_ROUTING_SOURCE_CH1_PHONO:
        return "A:Ch1";
    case UI_ROUTING_SOURCE_CH2_LINE:
    case UI_ROUTING_SOURCE_CH2_PHONO:
        return "A:Ch2";
    case UI_ROUTING_SOURCE_USB12:
    case UI_ROUTING_SOURCE_USB34:
        return "A:USB";
    default:
        return "A:";
    }
}

static char* get_current_input_src_b_str(void)
{
    switch (s_routing.current_ch_fader_b_assign)
    {
    case UI_ROUTING_SOURCE_CH1_LINE:
    case UI_ROUTING_SOURCE_CH1_PHONO:
        return "B:Ch1";
    case UI_ROUTING_SOURCE_CH2_LINE:
    case UI_ROUTING_SOURCE_CH2_PHONO:
        return "B:Ch2";
    case UI_ROUTING_SOURCE_USB12:
    case UI_ROUTING_SOURCE_USB34:
        return "B:USB";
    default:
        return "B:";
    }
}

static char* get_current_input_src_p_str(void)
{
    switch (s_routing.current_ch_fader_post_assign)
    {
    case UI_ROUTING_SOURCE_CH1_LINE:
        return "THRU:Ch1[line]";
    case UI_ROUTING_SOURCE_CH1_PHONO:
        return "THRU:Ch1[phono]";
    case UI_ROUTING_SOURCE_CH2_LINE:
        return "THRU:Ch2[line]";
    case UI_ROUTING_SOURCE_CH2_PHONO:
        return "THRU:Ch2[phono]";
    case UI_ROUTING_SOURCE_USB12:
        return "THRU:USB[1/2]";
    case UI_ROUTING_SOURCE_USB34:
        return "THRU:USB[3/4]";
    default:
        return "THRU:";
    }
}

static char* get_current_return_src_str(void)
{
    switch (s_routing.current_return_assign)
    {
    case UI_ROUTING_SOURCE_USB12:
        return "U12";
    case UI_ROUTING_SOURCE_USB34:
        return "U34";
    case UI_ROUTING_SOURCE_NONE:
        return "OFF";
    default:
        return "U--";
    }
}

static char* get_current_hp_out_src_str(void)
{
    switch (s_routing.current_hp_out_source)
    {
    case HP_SOURCE_CH_FADER_A:
        return "A";
    case HP_SOURCE_CH_FADER_B:
        return "B";
    case HP_SOURCE_THRU:
        return "T";
    case HP_SOURCE_MASTER:
        return "M";
    default:
        return "?";
    }
}

static uint8_t get_current_input_src_a_channel(void)
{
    switch (s_routing.current_ch_fader_a_assign)
    {
    case UI_ROUTING_SOURCE_CH1_LINE:
    case UI_ROUTING_SOURCE_CH1_PHONO:
    case UI_ROUTING_SOURCE_USB12:
        return 1U;
    case UI_ROUTING_SOURCE_CH2_LINE:
    case UI_ROUTING_SOURCE_CH2_PHONO:
    case UI_ROUTING_SOURCE_USB34:
        return 2U;
    default:
        return 0U;
    }
}

static uint8_t get_current_input_src_b_channel(void)
{
    switch (s_routing.current_ch_fader_b_assign)
    {
    case UI_ROUTING_SOURCE_CH1_LINE:
    case UI_ROUTING_SOURCE_CH1_PHONO:
    case UI_ROUTING_SOURCE_USB12:
        return 1U;
    case UI_ROUTING_SOURCE_CH2_LINE:
    case UI_ROUTING_SOURCE_CH2_PHONO:
    case UI_ROUTING_SOURCE_USB34:
        return 2U;
    default:
        return 0U;
    }
}

static bool get_current_ch1_dvs_enabled(void)
{
    return (s_routing.current_ch1_input_mode == UI_INPUT_MODE_DVS);
}

static bool get_current_ch2_dvs_enabled(void)
{
    return (s_routing.current_ch2_input_mode == UI_INPUT_MODE_DVS);
}

ui_control_input_mode_t ui_routing_get_ch1_input_mode(void)
{
    return (ui_control_input_mode_t) s_routing.current_ch1_input_mode;
}

ui_control_input_mode_t ui_routing_get_ch2_input_mode(void)
{
    return (ui_control_input_mode_t) s_routing.current_ch2_input_mode;
}

static bool get_current_return_enabled(void)
{
    return s_routing.current_return_assign != UI_ROUTING_SOURCE_NONE;
}

bool ui_routing_validate_persist(const ui_persist_state_t* state)
{
    if (state == NULL)
    {
        return false;
    }

    adau1466_input_source_t audio_input_source_a;
    adau1466_input_source_t audio_input_source_b;
    adau1466_input_source_t audio_input_source_post;

    if ((state->current_ch1_input_type > INPUT_TYPE_PHONO) ||
        (state->current_ch2_input_type > INPUT_TYPE_PHONO) ||
        (state->current_hp_out_source > HP_SOURCE_MASTER) ||
        (state->current_ch1_input_mode > UI_INPUT_MODE_SYNTH) ||
        (state->current_ch2_input_mode > UI_INPUT_MODE_SYNTH))
    {
        return false;
    }

    // ReturnはUI_ROUTING_SOURCE_USB12/USB34/NONEのいずれかだけを許可する。
    if ((state->current_return_assign < UI_ROUTING_SOURCE_USB12) ||
        (state->current_return_assign > UI_ROUTING_SOURCE_NONE))
    {
        return false;
    }

    if (!ui_routing_source_to_audio_input_source(
            (ui_routing_source_t) state->current_ch_fader_a_assign, &audio_input_source_a) ||
        !ui_routing_source_to_audio_input_source(
            (ui_routing_source_t) state->current_ch_fader_b_assign, &audio_input_source_b) ||
        !ui_routing_source_to_audio_input_source(
            (ui_routing_source_t) state->current_ch_fader_post_assign, &audio_input_source_post))
    {
        return false;
    }

    return true;
}

void ui_routing_capture_persist(ui_persist_state_t* state)
{
    if (state == NULL)
    {
        return;
    }

    state->current_ch1_input_type = s_routing.current_ch1_input_type;
    state->current_ch2_input_type = s_routing.current_ch2_input_type;
    state->current_ch_fader_a_assign     = s_routing.current_ch_fader_a_assign;
    state->current_ch_fader_b_assign     = s_routing.current_ch_fader_b_assign;
    state->current_ch_fader_post_assign  = s_routing.current_ch_fader_post_assign;
    state->current_return_assign  = s_routing.current_return_assign;
    state->current_hp_out_source  = s_routing.current_hp_out_source;
    state->current_ch1_input_mode = s_routing.current_ch1_input_mode;
    state->current_ch2_input_mode = s_routing.current_ch2_input_mode;
}

void ui_routing_reset(void)
{
    s_routing.current_ch1_input_type = INPUT_TYPE_LINE;
    s_routing.current_ch2_input_type = INPUT_TYPE_LINE;
    s_routing.current_ch_fader_a_assign     = UI_ROUTING_SOURCE_CH1_LINE;
    s_routing.current_ch_fader_b_assign     = UI_ROUTING_SOURCE_CH2_LINE;
    s_routing.current_ch_fader_post_assign  = UI_ROUTING_SOURCE_USB12;
    s_routing.current_return_assign  = UI_ROUTING_SOURCE_USB34;
    s_routing.current_hp_out_source  = HP_SOURCE_MASTER;
    s_routing.current_ch1_input_mode = UI_INPUT_MODE_DISABLED;
    s_routing.current_ch2_input_mode = UI_INPUT_MODE_DISABLED;
}

// OLED表示用: 同一時点のrouting状態から導出した表示値。
// 文字列は既存の文字列リテラルを指す。scheduler停止区間専用。
void ui_routing_capture_display_state(ui_routing_display_state_t* state)
{
    if (state == NULL)
    {
        return;
    }

    state->input_source_a_text = get_current_input_src_a_str();
    state->input_source_b_text = get_current_input_src_b_str();
    state->input_type_a_text   = get_current_input_type_a_str();
    state->input_type_b_text   = get_current_input_type_b_str();
    state->thru_source_text    = get_current_input_src_p_str();
    state->return_source_text  = get_current_return_src_str();
    state->hp_source_text      = get_current_hp_out_src_str();
    state->return_enabled      = get_current_return_enabled();
    state->dvs_enabled         = get_current_ch1_dvs_enabled() || get_current_ch2_dvs_enabled();

    const uint8_t src_a_channel = get_current_input_src_a_channel();
    state->input_source_a_mode_visible = (src_a_channel != 0U);
    state->input_source_a_mode = (src_a_channel == 1U) ? ui_routing_get_ch1_input_mode()
                                                       : ((src_a_channel == 2U) ? ui_routing_get_ch2_input_mode()
                                                                                : UI_INPUT_MODE_DISABLED);

    const uint8_t src_b_channel = get_current_input_src_b_channel();
    state->input_source_b_mode_visible = (src_b_channel != 0U);
    state->input_source_b_mode = (src_b_channel == 1U) ? ui_routing_get_ch1_input_mode()
                                                       : ((src_b_channel == 2U) ? ui_routing_get_ch2_input_mode()
                                                                                : UI_INPUT_MODE_DISABLED);
}
