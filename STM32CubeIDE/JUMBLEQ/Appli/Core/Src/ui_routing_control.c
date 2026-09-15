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
} ui_routing_state_t;

static ui_routing_state_t s_routing = {
    .current_ch1_input_type = INPUT_TYPE_LINE,
    .current_ch2_input_type = INPUT_TYPE_LINE,
    .current_ch_fader_a_assign     = INPUT_SRC_CH1_LN,
    .current_ch_fader_b_assign     = INPUT_SRC_CH2_LN,
    .current_ch_fader_post_assign  = INPUT_SRC_USB12,
    .current_return_assign  = INPUT_SRC_USB34,
    .current_hp_out_source  = CUE_SEL_MST,
    .current_ch1_input_mode = UI_INPUT_MODE_DISABLED,
    .current_ch2_input_mode = UI_INPUT_MODE_DISABLED,
};

static uint8_t input_src_from_channel_type(uint8_t input_ch, uint8_t input_type)
{
    switch (input_ch)
    {
    case INPUT_CH1:
        return (input_type == INPUT_TYPE_PHONO) ? INPUT_SRC_CH1_PN : INPUT_SRC_CH1_LN;
    case INPUT_CH2:
        return (input_type == INPUT_TYPE_PHONO) ? INPUT_SRC_CH2_PN : INPUT_SRC_CH2_LN;
    case INPUT_USB12:
        return INPUT_SRC_USB12;
    case INPUT_USB34:
        return INPUT_SRC_USB34;
    default:
        return INPUT_SRC_NONE;
    }
}

static uint8_t current_input_src_from_channel(uint8_t input_ch)
{
    switch (input_ch)
    {
    case INPUT_CH1:
        return input_src_from_channel_type(INPUT_CH1, s_routing.current_ch1_input_type);
    case INPUT_CH2:
        return input_src_from_channel_type(INPUT_CH2, s_routing.current_ch2_input_type);
    case INPUT_USB12:
        return INPUT_SRC_USB12;
    case INPUT_USB34:
        return INPUT_SRC_USB34;
    default:
        return INPUT_SRC_NONE;
    }
}

static void replace_assign_for_input_channel(uint8_t* assign, uint8_t input_ch, uint8_t new_src)
{
    const uint8_t ln_src = input_src_from_channel_type(input_ch, INPUT_TYPE_LINE);
    const uint8_t pn_src = input_src_from_channel_type(input_ch, INPUT_TYPE_PHONO);

    if (*assign == ln_src || *assign == pn_src)
    {
        *assign = new_src;
    }
}

static void apply_mic_gain_amp_setting(uint8_t input_ch, uint8_t input_type)
{
    uint8_t codec_ch;
    uint8_t gain_db;

    switch (input_ch)
    {
    case INPUT_CH1:
        codec_ch = AK4619_MIC_GAIN_CH1;
        break;
    case INPUT_CH2:
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

    AUDIO_Mic_Gain_AMP_Setting_Channel(codec_ch, gain_db);
}

void ui_routing_apply_input_type(uint8_t input_ch, uint8_t input_type)
{
    const uint8_t new_src = input_src_from_channel_type(input_ch, input_type);

    select_input_type(input_ch, input_type);
    apply_mic_gain_amp_setting(input_ch, input_type);

    if (input_ch == INPUT_CH1)
    {
        s_routing.current_ch1_input_type = input_type;
    }
    else if (input_ch == INPUT_CH2)
    {
        s_routing.current_ch2_input_type = input_type;
    }

    replace_assign_for_input_channel(&s_routing.current_ch_fader_a_assign, input_ch, new_src);
    replace_assign_for_input_channel(&s_routing.current_ch_fader_b_assign, input_ch, new_src);
    replace_assign_for_input_channel(&s_routing.current_ch_fader_post_assign, input_ch, new_src);
}

static bool is_usb_assign(uint8_t assign)
{
    return (assign == INPUT_SRC_USB12) || (assign == INPUT_SRC_USB34);
}

static bool input_mode_uses_insert(uint8_t mode)
{
    return (mode == UI_INPUT_MODE_DVS) || (mode == UI_INPUT_MODE_SYNTH);
}

static void apply_send_source_selection(uint8_t input_ch)
{
    if (input_ch == INPUT_CH1)
    {
        const bool select_insert = input_mode_uses_insert(s_routing.current_ch1_input_mode) || is_usb_assign(s_routing.current_ch_fader_a_assign);
        select_send_source(INPUT_CH1, select_insert);
    }
    else if (input_ch == INPUT_CH2)
    {
        const bool select_insert = input_mode_uses_insert(s_routing.current_ch2_input_mode) || is_usb_assign(s_routing.current_ch_fader_b_assign);
        select_send_source(INPUT_CH2, select_insert);
    }
}

void ui_routing_apply_ch_fader_assign_a(uint8_t input_ch)
{
    select_ch_fader_assign_a_source(input_ch);
    s_routing.current_ch_fader_a_assign = current_input_src_from_channel(input_ch);
    apply_send_source_selection(INPUT_CH1);
}

void ui_routing_apply_ch_fader_assign_b(uint8_t input_ch)
{
    select_ch_fader_assign_b_source(input_ch);
    s_routing.current_ch_fader_b_assign = current_input_src_from_channel(input_ch);
    apply_send_source_selection(INPUT_CH2);
}

void ui_routing_apply_ch_fader_assign_post(uint8_t input_ch)
{
    select_ch_fader_assign_post_source(input_ch);
    s_routing.current_ch_fader_post_assign = current_input_src_from_channel(input_ch);
}

void ui_routing_apply_hp_out_source(uint8_t source)
{
    if (source > CUE_SEL_MST)
    {
        return;
    }

    select_hp_out_source(source);
    s_routing.current_hp_out_source = source;
}

void ui_routing_apply_input_mode(uint8_t input_ch, UI_InputMode_t mode)
{
    const bool enable_insert = input_mode_uses_insert((uint8_t) mode);

    set_input_insert_enabled(input_ch, enable_insert);
    if (input_ch == INPUT_CH1)
    {
        s_routing.current_ch1_input_mode = (uint8_t) mode;
    }
    else if (input_ch == INPUT_CH2)
    {
        s_routing.current_ch2_input_mode = (uint8_t) mode;
    }
    apply_send_source_selection(input_ch);
}

uint8_t ui_routing_apply_return_source(uint8_t input_ch)
{
    if ((input_ch != INPUT_USB12) && (input_ch != INPUT_USB34))
    {
        s_routing.current_return_assign = INPUT_SRC_NONE;
        mute_input_from_return();
        return INPUT_SRC_NONE;
    }

    select_return_ch_source(input_ch);
    s_routing.current_return_assign = current_input_src_from_channel(input_ch);
    return s_routing.current_return_assign;
}

bool ui_routing_assign_to_input_ch(uint8_t assign, uint8_t* input_ch)
{
    if (input_ch == NULL)
    {
        return false;
    }

    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        *input_ch = INPUT_CH1;
        return true;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        *input_ch = INPUT_CH2;
        return true;
    case INPUT_SRC_USB12:
        *input_ch = INPUT_USB12;
        return true;
    case INPUT_SRC_USB34:
        *input_ch = INPUT_USB34;
        return true;
    default:
        return false;
    }
}

bool ui_routing_assign_to_return_input_ch(uint8_t assign, uint8_t* input_ch)
{
    if (input_ch == NULL)
    {
        return false;
    }

    switch (assign)
    {
    case INPUT_SRC_USB12:
        *input_ch = INPUT_USB12;
        return true;
    case INPUT_SRC_USB34:
        *input_ch = INPUT_USB34;
        return true;
    case INPUT_SRC_NONE:
        *input_ch = INPUT_SRC_NONE;
        return true;
    default:
        return false;
    }
}

uint8_t ui_routing_get_ch_fader_assign(uint8_t pair_idx)
{
    return (pair_idx == 0U) ? s_routing.current_ch_fader_a_assign
                            : s_routing.current_ch_fader_b_assign;
}

uint8_t ui_routing_get_input_mode(uint8_t input_ch)
{
    return (input_ch == INPUT_CH1) ? s_routing.current_ch1_input_mode
                                   : s_routing.current_ch2_input_mode;
}

uint8_t ui_routing_get_return_assign(void)
{
    return s_routing.current_return_assign;
}

bool ui_routing_is_synth_mode_active(void)
{
    return (s_routing.current_ch1_input_mode == UI_INPUT_MODE_SYNTH) ||
           (s_routing.current_ch2_input_mode == UI_INPUT_MODE_SYNTH);
}

char* get_current_input_typeA_str(void)
{
    switch (s_routing.current_ch_fader_a_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH2_LN:
        return "[line]";
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_CH2_PN:
        return "[phono]";
    case INPUT_SRC_USB12:
        return "[1/2]";
    case INPUT_SRC_USB34:
        return "[3/4]";
    default:
        return "[]";
    }
}

char* get_current_input_typeB_str(void)
{
    switch (s_routing.current_ch_fader_b_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH2_LN:
        return " [line]";
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_CH2_PN:
        return "[phono]";
    case INPUT_SRC_USB12:
        return "  [1/2]";
    case INPUT_SRC_USB34:
        return "  [3/4]";
    default:
        return "     []";
    }
}

char* get_current_input_srcA_str(void)
{
    switch (s_routing.current_ch_fader_a_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return "A:Ch1";
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return "A:Ch2";
    case INPUT_SRC_USB12:
    case INPUT_SRC_USB34:
        return "A:USB";
    default:
        return "A:";
    }
}

char* get_current_input_srcB_str(void)
{
    switch (s_routing.current_ch_fader_b_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return "B:Ch1";
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return "B:Ch2";
    case INPUT_SRC_USB12:
    case INPUT_SRC_USB34:
        return "B:USB";
    default:
        return "B:";
    }
}

char* get_current_input_srcP_str(void)
{
    switch (s_routing.current_ch_fader_post_assign)
    {
    case INPUT_SRC_CH1_LN:
        return "THRU:Ch1[line]";
    case INPUT_SRC_CH1_PN:
        return "THRU:Ch1[phono]";
    case INPUT_SRC_CH2_LN:
        return "THRU:Ch2[line]";
    case INPUT_SRC_CH2_PN:
        return "THRU:Ch2[phono]";
    case INPUT_SRC_USB12:
        return "THRU:USB[1/2]";
    case INPUT_SRC_USB34:
        return "THRU:USB[3/4]";
    default:
        return "THRU:";
    }
}

char* get_current_return_src_str(void)
{
    switch (s_routing.current_return_assign)
    {
    case INPUT_SRC_USB12:
        return "U12";
    case INPUT_SRC_USB34:
        return "U34";
    case INPUT_SRC_NONE:
        return "OFF";
    default:
        return "U--";
    }
}

char* get_current_hp_out_src_str(void)
{
    switch (s_routing.current_hp_out_source)
    {
    case CUE_SEL_CH_FADER_A:
        return "A";
    case CUE_SEL_CH_FADER_B:
        return "B";
    case CUE_SEL_THRU:
        return "T";
    case CUE_SEL_MST:
        return "M";
    default:
        return "?";
    }
}

uint8_t get_current_input_srcA_channel(void)
{
    switch (s_routing.current_ch_fader_a_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_USB12:
        return 1U;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
    case INPUT_SRC_USB34:
        return 2U;
    default:
        return 0U;
    }
}

uint8_t get_current_input_srcB_channel(void)
{
    switch (s_routing.current_ch_fader_b_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_USB12:
        return 1U;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
    case INPUT_SRC_USB34:
        return 2U;
    default:
        return 0U;
    }
}

bool get_current_ch1_dvs_enabled(void)
{
    return (s_routing.current_ch1_input_mode == UI_INPUT_MODE_DVS);
}

bool get_current_ch2_dvs_enabled(void)
{
    return (s_routing.current_ch2_input_mode == UI_INPUT_MODE_DVS);
}

UI_InputMode_t get_current_ch1_input_mode(void)
{
    return (UI_InputMode_t) s_routing.current_ch1_input_mode;
}

UI_InputMode_t get_current_ch2_input_mode(void)
{
    return (UI_InputMode_t) s_routing.current_ch2_input_mode;
}

bool get_current_return_enabled(void)
{
    return s_routing.current_return_assign != INPUT_SRC_NONE;
}

bool ui_routing_validate_persist(const UI_ControlPersistState_t* state)
{
    if (state == NULL)
    {
        return false;
    }

    uint8_t input_ch_a;
    uint8_t input_ch_b;
    uint8_t input_ch_post;
    uint8_t input_ch_return;

    if ((state->current_ch1_input_type > INPUT_TYPE_PHONO) ||
        (state->current_ch2_input_type > INPUT_TYPE_PHONO) ||
        (state->current_hp_out_source > CUE_SEL_MST) ||
        (state->current_ch1_input_mode > UI_INPUT_MODE_SYNTH) ||
        (state->current_ch2_input_mode > UI_INPUT_MODE_SYNTH))
    {
        return false;
    }

    if (!ui_routing_assign_to_input_ch(state->current_ch_fader_a_assign, &input_ch_a) ||
        !ui_routing_assign_to_input_ch(state->current_ch_fader_b_assign, &input_ch_b) ||
        !ui_routing_assign_to_input_ch(state->current_ch_fader_post_assign, &input_ch_post) ||
        !ui_routing_assign_to_return_input_ch(state->current_return_assign, &input_ch_return))
    {
        return false;
    }

    return true;
}

void ui_routing_capture_persist(UI_ControlPersistState_t* state)
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
    s_routing.current_ch_fader_a_assign     = INPUT_SRC_CH1_LN;
    s_routing.current_ch_fader_b_assign     = INPUT_SRC_CH2_LN;
    s_routing.current_ch_fader_post_assign  = INPUT_SRC_USB12;
    s_routing.current_return_assign  = INPUT_SRC_USB34;
    s_routing.current_hp_out_source  = CUE_SEL_MST;
    s_routing.current_ch1_input_mode = UI_INPUT_MODE_DISABLED;
    s_routing.current_ch2_input_mode = UI_INPUT_MODE_DISABLED;
}
