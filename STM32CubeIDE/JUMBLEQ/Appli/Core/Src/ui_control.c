/*
 * ui_control.c
 *
 *  Created on: Feb 18, 2026
 */

#include "ui_control.h"
#include "ui_control_internal.h"

#include "audio_control.h"

#include "adc.h"
#include "eeprom.h"
#include "hpdma.h"
#include "i2c.h"
#include "ak4619.h"
#include "led_control.h"
#include "linked_list.h"

#include "adau1466.h"
#include "SigmaStudioFW.h"

#include <math.h>
#include <string.h>

#define POT_CH_SEL_WAIT           1
#define POT_MA_SIZE               4  // 移動平均のサンプル数
#define POT_NUM                   16
#define POT_HYSTERESIS_NUM        12
#define POT_CC_HYSTERESIS_RAW     8U
#define POT_CC_MIN_DEADZONE_VALUE 2U
#define MAG_SW_NUM                6
#define MAG_CALIBRATION_COUNT_MAX 100
#define MAG_CH_FADER_CUTOFF          16
#define MAG_CH_FADER_RANGE           1400
#define CH_FADER_FADE_DOWN_SOURCE_COUNT 3U
// At most one update per pair per ADC task iteration (2 ms minimum).
#define CH_FADER_DSP_QUEUE_CAPACITY  512U

extern DMA_QListTypeDef List_HPDMA1_Channel0;

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

enum
{
    POT_CH_CC0 = 0,
    POT_CH_CC1,
    POT_CH_CH1_IN,
    POT_CH_CC2,
    POT_CH_CC3,
    POT_CH_CC4,
    POT_CH_CH2_IN,
    POT_CH_CH1_OUT,
    POT_CH_CH2_OUT,
    POT_CH_DRY_WET,
    POT_CH_RETURN_IN,
    POT_CH_HP_OUT,
    POT_CH_MAG0,
    POT_CH_MAG1,
    POT_CH_MAG2,
    POT_CH_MAG3,
};

typedef enum
{
    CH_FADER_GESTURE_PARENT_NONE = 0,
    CH_FADER_GESTURE_PARENT_FADE_UP,
    CH_FADER_GESTURE_PARENT_FADE_DOWN,
} ch_fader_gesture_parent_t;

__attribute__((section("noncacheable_buffer"), aligned(32))) uint32_t adc_val[ADC_NUM] = {0};

// State used by magnetic-switch channel fader processing.
typedef struct
{
    uint8_t position_a;  // Last quantized output value sent to ch_fader pair A.
    uint8_t position_b;  // Last quantized output value sent to ch_fader pair B.
    float raw[MAG_SW_NUM];  // Per-sensor normalized raw value after magnetic conversion.
    float prev[MAG_SW_NUM];  // Previous raw value used for outgoing MIDI CC change detection.
    float down_floor[MAG_SW_NUM];  // Per-sensor held minimum used by paired fade-down tracking.
    float up_peak[MAG_SW_NUM];  // Per-sensor held maximum used by paired fade-up tracking.
    float pair_hold_value[2];  // Current held output value for each ch_fader pair.
    float pair_bottom_restore_value[2];  // Held output value to restore after leaving the fade-down bottom.
    float pair_top_restore_value[2];  // Held output value to restore after leaving the fade-up top.
    float fade_up_prev[2];  // Previous fade-up value used to detect when pair output needs recomputing.
    float fade_down_prev[2];  // Previous fade-down value used to detect when pair output needs recomputing.
    float fade_down_combined_raw[2];  // Per-pair fade-down value after dual-source arbitration.
    float fade_down_source_prev[2][CH_FADER_FADE_DOWN_SOURCE_COUNT];  // Previous fade-down source values used for retrigger detection.
    uint8_t fade_down_active_source[2];  // Last fade-down source that started a cut gesture.
    uint8_t fade_down_force_release_reads[2];  // Number of reads that should force a release before retriggering.
    uint8_t pair_gesture_parent[2];  // First-pressed side that owns the current two-finger gesture.
    bool pair_gesture_armed[2];  // Whether both sides were released and a new parent can be selected.
    bool pair_gesture_child_active[2];  // Whether the child side has been operated during the current gesture.
    bool pair_fade_down_cut_active[2];  // Whether each pair is inside the current fade-down cut gesture.
    bool pair_bottom_hold_active[2];  // Whether each pair is still forced muted at the bottom of a fade-down gesture.
    bool pair_fade_down_bottomed[2];  // Whether each pair is currently in the fade-down bottom zone.
    bool pair_top_hold_active[2];  // Whether each pair is still forced muted at the top of a fade-up gesture.
    bool pair_fade_up_topped[2];  // Whether each pair is currently in the fade-up top zone.
    bool pair_reverse_fade_down_takeover[2];  // Whether fade-down owns reverse momentary output while fade-up is released.
    bool fade_prev_valid;  // Whether the previous pair fade values have been initialized.
    uint8_t note_peak_vel[MAG_SW_NUM];  // Peak velocity captured while scanning one ch_fader sensor note-on edge.
    uint32_t note_scan_start_ms[MAG_SW_NUM];  // Start tick for one ch_fader sensor note velocity scan window.
    bool note_is_on[MAG_SW_NUM];  // Whether each ch_fader sensor note output is currently on.
    bool note_scan_active[MAG_SW_NUM];  // Whether each ch_fader sensor is accumulating note-on velocity.
} ch_fader_state_t;

// Aggregated UI runtime state (ADC-derived controls + persisted selections).
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
    uint8_t pot_ch;
    uint8_t pot_ch_counter;
    uint16_t pot_ma_index[POT_NUM];
    uint8_t pot_sample_count[POT_NUM];
    uint32_t pot_val_ma[POT_NUM][POT_MA_SIZE];
    uint16_t pot_val[POT_NUM];
    uint16_t pot_val_prev[POT_NUM][2];
    uint16_t pot_hysteresis_raw_ma[POT_HYSTERESIS_NUM][POT_MA_SIZE];
    uint16_t pot_hysteresis_last_sent[POT_HYSTERESIS_NUM];
    bool pot_hysteresis_has_last_sent[POT_HYSTERESIS_NUM];
    uint16_t pot_mag_calibration_count[4];
    uint32_t pot_mag_offset_sum[4];
    uint16_t pot_mag_offset[4];
    uint8_t pot_mag_candidate[4];
    uint8_t pot_mag_stable_count[4];
    uint8_t pot_mag_state[4];
    uint16_t mag_calibration_count;
    uint16_t mag_val[MAG_SW_NUM];
    uint32_t mag_offset_sum[MAG_SW_NUM];
    uint16_t mag_offset[MAG_SW_NUM];
    ch_fader_state_t ch_fader;
    float ch_fader_curve_width_a;
    float ch_fader_curve_width_b;
    bool ch_fader_reverse_a;
    bool ch_fader_reverse_b;
    bool mag_out_as_note;
    bool curve_edit_mode;
    bool is_start_audio_control;
} ui_control_state_t;

typedef struct
{
    volatile UI_Uf2TransitionState_t state;
    volatile uint32_t arm_started_ms;
    volatile bool displays_cleared;
    uint32_t raw_changed_ms;
    uint32_t hold_started_ms;
    uint32_t notice_started_ms;
    bool last_raw_pressed;
    bool stable_pressed;
    bool release_observed;
} uf2_bootloader_control_t;

static ui_control_state_t s_ui = {
    .current_ch1_input_type = INPUT_TYPE_LINE,
    .current_ch2_input_type = INPUT_TYPE_LINE,
    .current_ch_fader_a_assign     = INPUT_SRC_CH1_LN,
    .current_ch_fader_b_assign     = INPUT_SRC_CH2_LN,
    .current_ch_fader_post_assign  = INPUT_SRC_USB12,
    .current_return_assign  = INPUT_SRC_USB34,
    .current_hp_out_source  = CUE_SEL_MST,
    .current_ch1_input_mode = UI_INPUT_MODE_DISABLED,
    .current_ch2_input_mode = UI_INPUT_MODE_DISABLED,
    .ch_fader_dvs_delay_ms  = UI_CH_FADER_DVS_DELAY_DEFAULT_MS,
    .sensor2_aux_fade_down_assign = UI_CH_FADER_AUX_ASSIGN_A,
    .sensor3_aux_fade_down_assign = UI_CH_FADER_AUX_ASSIGN_B,
    .ch_fader_curve_width_a    = UI_CH_FADER_CURVE_WIDTH_A_DEFAULT,
    .ch_fader_curve_width_b    = UI_CH_FADER_CURVE_WIDTH_B_DEFAULT,
    .ch_fader_reverse_a     = false,
    .ch_fader_reverse_b     = false,
    .mag_out_as_note        = false,
    .curve_edit_mode        = false,
    .ch_fader.position_a          = 0,
    .ch_fader.position_b          = 0,
    .ch_fader.raw                 = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .ch_fader.prev                = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .ch_fader.down_floor          = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .ch_fader.up_peak             = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .ch_fader.pair_hold_value          = {0.0f, 0.0f},
    .ch_fader.pair_bottom_restore_value = {0.0f, 0.0f},
    .ch_fader.pair_top_restore_value    = {0.0f, 0.0f},
    .ch_fader.fade_up_prev             = {0.0f, 0.0f},
    .ch_fader.fade_down_prev           = {1.0f, 1.0f},
    .ch_fader.fade_down_combined_raw   = {1.0f, 1.0f},
    .ch_fader.fade_down_source_prev    = {{1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
    .ch_fader.fade_down_active_source  = {0xFFU, 0xFFU},
    .ch_fader.fade_down_force_release_reads = {0U, 0U},
    .ch_fader.pair_gesture_parent      = {CH_FADER_GESTURE_PARENT_NONE, CH_FADER_GESTURE_PARENT_NONE},
    .ch_fader.pair_gesture_armed       = {true, true},
    .ch_fader.pair_gesture_child_active = {false, false},
    .ch_fader.pair_fade_down_cut_active = {false, false},
    .ch_fader.pair_bottom_hold_active  = {false, false},
    .ch_fader.pair_fade_down_bottomed  = {false, false},
    .ch_fader.pair_top_hold_active     = {false, false},
    .ch_fader.pair_fade_up_topped      = {false, false},
    .ch_fader.pair_reverse_fade_down_takeover = {false, false},
    .ch_fader.fade_prev_valid          = false,
    .is_start_audio_control = false,
};

static uf2_bootloader_control_t s_uf2 = {
    .state = UI_UF2_TRANSITION_IDLE,
};

static volatile bool is_adc_complete  = false;
static const uint8_t POT_MAG_CH_FIRST = POT_CH_MAG0;
static const uint8_t POT_MAG_CH_LAST  = POT_CH_MAG3;

static const float CH_FADER_CC_UPDATE_THRESHOLD    = 0.01f;
static const float CH_FADER_EXTREMA_HYSTERESIS     = 0.002f;
static const float CH_FADER_SEND_THRESHOLD         = 0.002f;
static const float CH_FADER_PAIR_RESET_THRESHOLD   = 0.98f;
static const float CH_FADER_MIN_RESET_CUTOFF       = 0.05f;
static const float CH_FADER_PAIR_ONSET_DEADBAND    = 0.10f;
static const float CH_FADER_PAIR_FADE_DOWN_PRESS_THRESHOLD     = 0.95f;
static const float CH_FADER_PAIR_FADE_DOWN_DEEPER_DELTA        = 0.03f;
static const float CH_FADER_PAIR_CURVE_WIDTH_MIN   = 0.02f;
static const float CH_FADER_PAIR_CURVE_WIDTH_MAX   = 0.60f;
// At the maximum curve setting, 2% key travel reaches 90% output.
static const float CH_FADER_PAIR_CURVE_EXPONENT_MAX = 115.12925465f;
static const float CH_FADER_PAIR_CURVE_LINEAR_EPSILON = 0.0001f;
static const float CH_FADER_PAIR_BOTTOM_HOLD_RELEASE_THRESHOLD = 0.0025f;
static const float CH_FADER_PAIR_BOTTOM_REHOLD_THRESHOLD       = 0.0008f;

static const uint8_t MIDI_CH_15                  = 14U;  // zero-based MIDI channel index.
static const uint8_t MIDI_CC_CH_FADER_CURVE_A        = 20U;
static const uint8_t MIDI_CC_CH_FADER_CURVE_B        = 21U;
static const uint8_t MIDI_CC_CH_FADER_DVS_DELAY      = 22U;
static const uint8_t MIDI_PC_CURVE_EDIT_MODE_OFF = 120U;
static const uint8_t MIDI_PC_CURVE_EDIT_MODE_ON  = 121U;
static const uint8_t MIDI_PC_MUX_OUTPUT_CC       = 122U;
static const uint8_t MIDI_PC_MUX_OUTPUT_NOTE     = 123U;
static const uint8_t MIDI_PC_ARM_UF2_BOOTLOADER  = 124U;
static const uint8_t MIDI_PC_CANCEL_UF2_BOOTLOADER = 125U;
static const uint8_t MIDI_PC_REQUEST_EEPROM_DUMP = 126U;
static const uint8_t MIDI_PC_SAVE_EEPROM         = 127U;
static const uint32_t UF2_ARM_WINDOW_MS          = 10000U;
static const uint32_t UF2_SWITCH_DEBOUNCE_MS     = 30U;
static const uint32_t UF2_SWITCH_HOLD_MS         = 2000U;
static const uint32_t UF2_NOTICE_MS              = 2000U;
static const uint8_t MIDI_NOTE_ON_THRESHOLD      = 4U;
static const uint8_t MIDI_NOTE_OFF_THRESHOLD     = 2U;
static const uint32_t MIDI_NOTE_VEL_WINDOW_MS    = 12U;
static const float MIDI_NOTE_VEL_GAMMA           = 0.65f;
static const uint8_t CH_FADER_FADE_DOWN_SOURCE_NONE    = 0xFFU;
static const uint8_t CH_FADER_FADE_DOWN_RETRIGGER_RELEASE_READS_WITH_FADE_UP = 16U;
static const uint8_t CH_FADER_FADE_DOWN_RETRIGGER_RELEASE_READS_MOMENTARY    = 16U;

static uint8_t s_note_peak_vel[128];
static uint32_t s_note_scan_start_ms[128];
static bool s_note_is_on[128];
static bool s_note_scan_active[128];

typedef struct
{
    uint8_t fade_up_idx;
    uint8_t fade_down_idx;
    uint8_t aux_fade_down_idx;
    uint8_t aux2_fade_down_idx;
    uint8_t prev_idx;
    uint8_t* current_position;
    void (*set_dc)(float ch_fader_position);
} ch_fader_pair_runtime_t;

typedef enum
{
    CH_FADER_PAIR_A = 0,
    CH_FADER_PAIR_B = 1,
    CH_FADER_PAIR_COUNT
} ch_fader_pair_index_t;

// Runtime mapping for each ch_fader bus:
// Change these indices to reassign the magnetic switches used by each pair.
// fade_up_idx and fade_down_idx produce the held scalar. Each valid auxiliary
// fade-down index can retrigger the same fade-down gesture path.
static ch_fader_pair_runtime_t s_ch_fader_pairs[] = {
    {
     .fade_up_idx       = 0,
     .fade_down_idx     = 1,
     .aux_fade_down_idx = 2,
     .aux2_fade_down_idx = MAG_SW_NUM,
     .prev_idx          = CH_FADER_PAIR_A,
     .current_position  = &s_ui.ch_fader.position_a,
     .set_dc            = set_dc_inputA,
     },
    {
     .fade_up_idx       = 5,
     .fade_down_idx     = 4,
     .aux_fade_down_idx = 3,
     .aux2_fade_down_idx = MAG_SW_NUM,
     .prev_idx          = CH_FADER_PAIR_B,
     .current_position  = &s_ui.ch_fader.position_b,
     .set_dc            = set_dc_inputB,
     },
};

typedef struct
{
    uint32_t captured_ms;
    float value;
} ch_fader_dsp_update_t;

typedef struct
{
    ch_fader_dsp_update_t queue[CH_FADER_DSP_QUEUE_CAPACITY];
    uint16_t head;
    uint16_t count;
    uint8_t target_position;
    float target_value;
    uint8_t input_assign;
    bool delay_enabled;
    bool context_valid;
    bool target_valid;
} ch_fader_dsp_state_t;

static ch_fader_dsp_state_t s_ch_fader_dsp[CH_FADER_PAIR_COUNT];

static uint8_t get_ch_fader_dsp_assign(const ch_fader_pair_runtime_t* pair)
{
    return (pair->prev_idx == CH_FADER_PAIR_A) ? s_ui.current_ch_fader_a_assign
                                             : s_ui.current_ch_fader_b_assign;
}

static bool ch_fader_assign_uses_dvs(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_USB12:  // PC return for Ch. 1 DVS, including direct USB routing.
        return s_ui.current_ch1_input_mode == UI_INPUT_MODE_DVS;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
    case INPUT_SRC_USB34:  // PC return for Ch. 2 DVS, including direct USB routing.
        return s_ui.current_ch2_input_mode == UI_INPUT_MODE_DVS;
    default:
        return false;
    }
}

static void write_ch_fader_dsp_output(const ch_fader_pair_runtime_t* pair, float value)
{
    pair->set_dc(value);
    *pair->current_position = (uint8_t) (value * 128.0f);
}

// Configuration/reapply operations synchronize immediately and cancel old gestures.
static void sync_ch_fader_dsp_output(const ch_fader_pair_runtime_t* pair, float value)
{
    ch_fader_dsp_state_t* state = &s_ch_fader_dsp[pair->prev_idx];
    state->head = 0U;
    state->count = 0U;
    state->target_value = value;
    state->target_position = (uint8_t) (value * 128.0f);
    state->target_valid = true;
    state->input_assign = get_ch_fader_dsp_assign(pair);
    state->delay_enabled = ch_fader_assign_uses_dvs(state->input_assign);
    state->context_valid = true;
    write_ch_fader_dsp_output(pair, value);
}

static void refresh_ch_fader_dsp_context(const ch_fader_pair_runtime_t* pair)
{
    ch_fader_dsp_state_t* state = &s_ch_fader_dsp[pair->prev_idx];
    const uint8_t assign = get_ch_fader_dsp_assign(pair);
    const bool delay_enabled = ch_fader_assign_uses_dvs(assign);

    if (state->context_valid &&
        ((state->input_assign != assign) || (state->delay_enabled != delay_enabled)))
    {
        state->head = 0U;
        state->count = 0U;
        if (state->target_valid)
        {
            write_ch_fader_dsp_output(pair, state->target_value);
        }
    }
    state->input_assign = assign;
    state->delay_enabled = delay_enabled;
    state->context_valid = true;
}

static void submit_ch_fader_dsp_output(const ch_fader_pair_runtime_t* pair, float value)
{
    ch_fader_dsp_state_t* state = &s_ch_fader_dsp[pair->prev_idx];
    const uint8_t position = (uint8_t) (value * 128.0f);
    refresh_ch_fader_dsp_context(pair);

    // Compare with the last requested value, not the still-delayed DSP position.
    // Otherwise a quick return to the DSP's current value would lose its edge.
    if (state->target_valid && (position == state->target_position))
    {
        return;
    }
    state->target_value = value;
    state->target_position = position;
    state->target_valid = true;

    if (!state->delay_enabled || (s_ui.ch_fader_dvs_delay_ms == 0U) ||
        (state->count == CH_FADER_DSP_QUEUE_CAPACITY))
    {
        // Capacity exceeds the configured delay history. If overloaded, recover to
        // the current value instead of leaving a stale cut waiting in the queue.
        sync_ch_fader_dsp_output(pair, value);
        return;
    }

    const uint16_t tail = (uint16_t) ((state->head + state->count) % CH_FADER_DSP_QUEUE_CAPACITY);
    state->queue[tail].captured_ms = HAL_GetTick();
    state->queue[tail].value = value;
    state->count++;
}

static void service_ch_fader_dsp_outputs(void)
{
    for (uint8_t i = 0U; i < CH_FADER_PAIR_COUNT; i++)
    {
        const ch_fader_pair_runtime_t* pair = &s_ch_fader_pairs[i];
        ch_fader_dsp_state_t* state = &s_ch_fader_dsp[i];
        refresh_ch_fader_dsp_context(pair);
        const uint32_t now = HAL_GetTick();

        while (state->count != 0U)
        {
            const ch_fader_dsp_update_t* update = &state->queue[state->head];
            // Unsigned elapsed time also handles HAL tick wraparound.
            if ((uint32_t) (now - update->captured_ms) < s_ui.ch_fader_dvs_delay_ms)
            {
                break;
            }
            write_ch_fader_dsp_output(pair, update->value);
            state->head = (uint16_t) ((state->head + 1U) % CH_FADER_DSP_QUEUE_CAPACITY);
            state->count--;
        }
    }
}

static void append_ch_fader_aux_sensor(uint8_t pair_idx, uint8_t sensor_idx)
{
    ch_fader_pair_runtime_t* pair;

    if ((pair_idx >= CH_FADER_PAIR_COUNT) || (sensor_idx >= MAG_SW_NUM))
    {
        return;
    }

    pair = &s_ch_fader_pairs[pair_idx];
    if (pair->aux_fade_down_idx >= MAG_SW_NUM)
    {
        pair->aux_fade_down_idx = sensor_idx;
    }
    else
    {
        pair->aux2_fade_down_idx = sensor_idx;
    }
}

static void reset_ch_fader_aux_assignment_runtime(void)
{
    for (uint8_t pair_idx = 0U; pair_idx < CH_FADER_PAIR_COUNT; pair_idx++)
    {
        s_ui.ch_fader.fade_down_prev[pair_idx] = -1.0f;
        s_ui.ch_fader.fade_down_combined_raw[pair_idx] = 1.0f;
        s_ui.ch_fader.fade_down_active_source[pair_idx] = CH_FADER_FADE_DOWN_SOURCE_NONE;
        s_ui.ch_fader.fade_down_force_release_reads[pair_idx] = 0U;
        s_ui.ch_fader.pair_gesture_parent[pair_idx] = CH_FADER_GESTURE_PARENT_NONE;
        s_ui.ch_fader.pair_gesture_armed[pair_idx] = true;
        s_ui.ch_fader.pair_gesture_child_active[pair_idx] = false;
        s_ui.ch_fader.pair_fade_down_cut_active[pair_idx] = false;
        s_ui.ch_fader.pair_bottom_hold_active[pair_idx] = false;
        s_ui.ch_fader.pair_fade_down_bottomed[pair_idx] = false;
        s_ui.ch_fader.pair_reverse_fade_down_takeover[pair_idx] = false;

        for (uint8_t source = 0U; source < CH_FADER_FADE_DOWN_SOURCE_COUNT; source++)
        {
            s_ui.ch_fader.fade_down_source_prev[pair_idx][source] = 1.0f;
        }
    }
}

static bool apply_ch_fader_aux_assignments(uint8_t sensor2_assign, uint8_t sensor3_assign)
{
    if ((sensor2_assign > UI_CH_FADER_AUX_ASSIGN_B) ||
        (sensor3_assign > UI_CH_FADER_AUX_ASSIGN_B))
    {
        return false;
    }

    s_ui.sensor2_aux_fade_down_assign = sensor2_assign;
    s_ui.sensor3_aux_fade_down_assign = sensor3_assign;

    for (uint8_t pair_idx = 0U; pair_idx < CH_FADER_PAIR_COUNT; pair_idx++)
    {
        s_ch_fader_pairs[pair_idx].aux_fade_down_idx  = MAG_SW_NUM;
        s_ch_fader_pairs[pair_idx].aux2_fade_down_idx = MAG_SW_NUM;
    }

    append_ch_fader_aux_sensor(sensor2_assign, 2U);
    append_ch_fader_aux_sensor(sensor3_assign, 3U);
    reset_ch_fader_aux_assignment_runtime();
    return true;
}

static void send_control_change(uint8_t number, uint8_t value, uint8_t channel)
{
    uint8_t control_change[3] = {0xB0 | channel, number, value};
    tud_midi_stream_write(0, control_change, 3);
}

static void send_note(uint8_t note, uint8_t velocity, uint8_t channel)
{
    uint8_t note_msg[3];

    if (velocity == 0U)
    {
        note_msg[0] = 0x80U | channel;
        note_msg[1] = note;
        note_msg[2] = 0U;
    }
    else
    {
        note_msg[0] = 0x90U | channel;
        note_msg[1] = note;
        note_msg[2] = velocity;
    }

    tud_midi_stream_write(0, note_msg, 3);
}

static void clear_note_edge_state(uint8_t note)
{
    s_note_peak_vel[note]      = 0U;
    s_note_scan_start_ms[note] = 0U;
    s_note_is_on[note]         = false;
    s_note_scan_active[note]   = false;
}

static void emit_note_edge_if_needed(uint8_t note, uint8_t value)
{
    const uint32_t now_ms = HAL_GetTick();

    if (s_note_is_on[note])
    {
        if (value <= MIDI_NOTE_OFF_THRESHOLD)
        {
            send_note(note, 0U, 0U);
            clear_note_edge_state(note);
        }
        return;
    }

    if (!s_note_scan_active[note])
    {
        if (value >= MIDI_NOTE_ON_THRESHOLD)
        {
            s_note_scan_active[note]   = true;
            s_note_scan_start_ms[note] = now_ms;
            s_note_peak_vel[note]      = value;
        }
        return;
    }

    if (value > s_note_peak_vel[note])
    {
        s_note_peak_vel[note] = value;
    }

    if (value <= MIDI_NOTE_OFF_THRESHOLD)
    {
        clear_note_edge_state(note);
        return;
    }

    if ((now_ms - s_note_scan_start_ms[note]) >= MIDI_NOTE_VEL_WINDOW_MS)
    {
        uint8_t velocity = s_note_peak_vel[note];
        if (velocity == 0U)
        {
            velocity = 1U;
        }

        float n  = (float) velocity / 127.0f;
        n        = powf(n, MIDI_NOTE_VEL_GAMMA);
        velocity = (uint8_t) (1.0f + n * 126.0f);

        send_note(note, velocity, 0U);
        s_note_is_on[note]       = true;
        s_note_scan_active[note] = false;
    }
}
static void emit_mag_output(uint8_t cc_number, uint8_t note_number, uint8_t value)
{
    if (s_ui.mag_out_as_note)
    {
        emit_note_edge_if_needed(note_number, value);
    }
    else
    {
        send_control_change(cc_number, value, 0U);
    }
}

static void clear_ch_fader_note_state(uint8_t i)
{
    s_ui.ch_fader.note_peak_vel[i]      = 0U;
    s_ui.ch_fader.note_scan_start_ms[i] = 0U;
    s_ui.ch_fader.note_is_on[i]         = false;
    s_ui.ch_fader.note_scan_active[i]   = false;
}

static void emit_ch_fader_note_if_needed(uint8_t i, uint8_t note, uint8_t value)
{
    const uint32_t now_ms = HAL_GetTick();

    if (s_ui.ch_fader.note_is_on[i])
    {
        if (value <= MIDI_NOTE_OFF_THRESHOLD)
        {
            send_note(note, 0U, 0U);
            clear_ch_fader_note_state(i);
        }
        return;
    }

    if (!s_ui.ch_fader.note_scan_active[i])
    {
        if (value >= MIDI_NOTE_ON_THRESHOLD)
        {
            s_ui.ch_fader.note_scan_active[i]   = true;
            s_ui.ch_fader.note_scan_start_ms[i] = now_ms;
            s_ui.ch_fader.note_peak_vel[i]      = value;
        }
        return;
    }

    if (value > s_ui.ch_fader.note_peak_vel[i])
    {
        s_ui.ch_fader.note_peak_vel[i] = value;
    }

    if (value <= MIDI_NOTE_OFF_THRESHOLD)
    {
        clear_ch_fader_note_state(i);
        return;
    }

    if ((now_ms - s_ui.ch_fader.note_scan_start_ms[i]) >= MIDI_NOTE_VEL_WINDOW_MS)
    {
        uint8_t velocity = s_ui.ch_fader.note_peak_vel[i];
        if (velocity == 0U)
        {
            velocity = 1U;
        }

        float n  = (float) velocity / 127.0f;
        n        = powf(n, MIDI_NOTE_VEL_GAMMA);
        velocity = (uint8_t) (1.0f + n * 126.0f);

        send_note(note, velocity, 0U);
        s_ui.ch_fader.note_is_on[i]       = true;
        s_ui.ch_fader.note_scan_active[i] = false;
    }
}
static void send_program_change(uint8_t program, uint8_t channel)
{
    uint8_t program_change[2] = {0xC0 | channel, program};
    tud_midi_stream_write(0, program_change, 2);
}

static uint8_t ch_fader_to_cc(float ch_fader)
{
    if (ch_fader < 0.0f)
    {
        ch_fader = 0.0f;
    }
    else if (ch_fader > 1.0f)
    {
        ch_fader = 1.0f;
    }

    return (uint8_t) (127.0f - ch_fader * 127.0f);
}

static void mark_ch_fader_curve_dirty(void)
{
    for (uint8_t i = 0; i < CH_FADER_PAIR_COUNT; i++)
    {
        s_ui.ch_fader.fade_up_prev[i]   = -1.0f;
        s_ui.ch_fader.fade_down_prev[i] = -1.0f;
    }
}

static float midi_cc_to_ch_fader_curve_width(uint8_t value)
{
    const float t = (float) value / 127.0f;

    return CH_FADER_PAIR_CURVE_WIDTH_MAX + ((CH_FADER_PAIR_CURVE_WIDTH_MIN - CH_FADER_PAIR_CURVE_WIDTH_MAX) * t);
}

static uint8_t ch_fader_curve_width_to_midi_cc(float width)
{
    float t;

    if (width < CH_FADER_PAIR_CURVE_WIDTH_MIN)
    {
        width = CH_FADER_PAIR_CURVE_WIDTH_MIN;
    }
    else if (width > CH_FADER_PAIR_CURVE_WIDTH_MAX)
    {
        width = CH_FADER_PAIR_CURVE_WIDTH_MAX;
    }

    t = (CH_FADER_PAIR_CURVE_WIDTH_MAX - width) / (CH_FADER_PAIR_CURVE_WIDTH_MAX - CH_FADER_PAIR_CURVE_WIDTH_MIN);
    if (t < 0.0f)
    {
        t = 0.0f;
    }
    else if (t > 1.0f)
    {
        t = 1.0f;
    }

    return (uint8_t) ((t * 127.0f) + 0.5f);
}

static float clamp_ch_fader_curve_width(float value)
{
    if (value < CH_FADER_PAIR_CURVE_WIDTH_MIN)
    {
        return CH_FADER_PAIR_CURVE_WIDTH_MIN;
    }
    if (value > CH_FADER_PAIR_CURVE_WIDTH_MAX)
    {
        return CH_FADER_PAIR_CURVE_WIDTH_MAX;
    }
    return value;
}

uint8_t get_current_ch_fader_a_position(void)
{
    return s_ui.ch_fader.position_a;
}

uint8_t get_current_ch_fader_b_position(void)
{
    return s_ui.ch_fader.position_b;
}

int16_t get_current_ch1_in_db(void)
{
    // Input display order follows the current physical POT assignment.
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_CH1_IN]);
}

int16_t get_current_ch2_in_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_CH2_IN]);
}

int16_t get_current_ch1_out_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_CH1_OUT]);
}

int16_t get_current_ch2_out_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_CH2_OUT]);
}

int16_t get_current_return_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_RETURN_IN]);
}

bool get_current_return_enabled(void)
{
    return s_ui.current_return_assign != INPUT_SRC_NONE;
}

int16_t get_current_hp_out_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[POT_CH_HP_OUT]);
}

int16_t get_current_dry_wet(void)
{
    if (s_ui.pot_val[POT_CH_DRY_WET] <= POT_10BIT_MIN_DEADZONE)
    {
        return 0;
    }
    if (s_ui.pot_val[POT_CH_DRY_WET] >= POT_10BIT_DW_MAX_SNAP_START)
    {
        return 100;
    }

    int16_t pct = (int16_t) ((((double) (s_ui.pot_val[POT_CH_DRY_WET] - POT_10BIT_MIN_DEADZONE)) /
                              ((double) (POT_10BIT_DW_MAX_SNAP_START - POT_10BIT_MIN_DEADZONE)) * 100.0) + 0.5);
    if (pct < 0)
    {
        pct = 0;
    }
    if (pct > 100)
    {
        pct = 100;
    }
    return pct;
}

uint8_t get_current_ch_fader_sensor2_cc_value(void)
{
    return ch_fader_to_cc(s_ui.ch_fader.raw[2]);
}

uint8_t get_current_ch_fader_sensor3_cc_value(void)
{
    return ch_fader_to_cc(s_ui.ch_fader.raw[3]);
}

char* get_current_input_typeA_str(void)
{
    switch (s_ui.current_ch_fader_a_assign)
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
    switch (s_ui.current_ch_fader_b_assign)
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
    switch (s_ui.current_ch_fader_a_assign)
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
    switch (s_ui.current_ch_fader_b_assign)
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
    switch (s_ui.current_ch_fader_post_assign)
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
    switch (s_ui.current_return_assign)
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
    switch (s_ui.current_hp_out_source)
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
    switch (s_ui.current_ch_fader_a_assign)
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
    switch (s_ui.current_ch_fader_b_assign)
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
    return (s_ui.current_ch1_input_mode == UI_INPUT_MODE_DVS);
}

bool get_current_ch2_dvs_enabled(void)
{
    return (s_ui.current_ch2_input_mode == UI_INPUT_MODE_DVS);
}

UI_InputMode_t get_current_ch1_input_mode(void)
{
    return (UI_InputMode_t) s_ui.current_ch1_input_mode;
}

UI_InputMode_t get_current_ch2_input_mode(void)
{
    return (UI_InputMode_t) s_ui.current_ch2_input_mode;
}

uint8_t ui_control_get_ch_fader_dvs_delay_ms(void)
{
    return s_ui.ch_fader_dvs_delay_ms;
}

bool ui_control_is_ch_fader_reverse_a_enabled(void)
{
    return s_ui.ch_fader_reverse_a;
}

bool ui_control_is_ch_fader_reverse_b_enabled(void)
{
    return s_ui.ch_fader_reverse_b;
}

bool ui_control_is_curve_edit_mode_enabled(void)
{
    return s_ui.curve_edit_mode;
}

UI_Uf2TransitionState_t ui_control_get_uf2_transition_state(void)
{
    const UI_Uf2TransitionState_t state = s_uf2.state;
    __DMB();
    return state;
}

uint8_t ui_control_get_uf2_seconds_remaining(void)
{
    const UI_Uf2TransitionState_t state = s_uf2.state;
    __DMB();
    if ((state != UI_UF2_TRANSITION_WAIT_RELEASE) &&
        (state != UI_UF2_TRANSITION_WAIT_HOLD) &&
        (state != UI_UF2_TRANSITION_HOLDING))
    {
        return 0U;
    }

    const uint32_t elapsed_ms = HAL_GetTick() - s_uf2.arm_started_ms;
    if (elapsed_ms >= UF2_ARM_WINDOW_MS)
    {
        return 0U;
    }

    const uint32_t remaining_ms = UF2_ARM_WINDOW_MS - elapsed_ms;
    return (uint8_t) ((remaining_ms + 999U) / 1000U);
}

void ui_control_notify_uf2_displays_cleared(void)
{
    if (s_uf2.state != UI_UF2_TRANSITION_CLEARING_DISPLAYS)
    {
        return;
    }

    __DMB();
    s_uf2.displays_cleared = true;
}

uint8_t ui_control_get_ch_fader_curve_a_cc(void)
{
    return ch_fader_curve_width_to_midi_cc(s_ui.ch_fader_curve_width_a);
}

uint8_t ui_control_get_ch_fader_curve_b_cc(void)
{
    return ch_fader_curve_width_to_midi_cc(s_ui.ch_fader_curve_width_b);
}

void ui_control_dma_adc_cplt(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    is_adc_complete = true;
    __DSB();
}

void ui_control_set_adc_complete(bool complete)
{
    is_adc_complete = complete;
    __DMB();
}

void start_adc(void)
{
    if (MX_List_HPDMA1_Channel0_Config() != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMAEx_List_LinkQ(&handle_HPDMA1_Channel0, &List_HPDMA1_Channel0) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 0);
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
    s_ui.pot_ch = POT_CH_CC1;

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMAEN);
    SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMACFG);

    handle_HPDMA1_Channel0.XferCpltCallback = ui_control_dma_adc_cplt;
    if (HAL_DMAEx_List_Start_IT(&handle_HPDMA1_Channel0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
}

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
        return input_src_from_channel_type(INPUT_CH1, s_ui.current_ch1_input_type);
    case INPUT_CH2:
        return input_src_from_channel_type(INPUT_CH2, s_ui.current_ch2_input_type);
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

static void apply_input_type_change(uint8_t input_ch, uint8_t input_type)
{
    const uint8_t new_src = input_src_from_channel_type(input_ch, input_type);

    select_input_type(input_ch, input_type);
    apply_mic_gain_amp_setting(input_ch, input_type);

    if (input_ch == INPUT_CH1)
    {
        s_ui.current_ch1_input_type = input_type;
    }
    else if (input_ch == INPUT_CH2)
    {
        s_ui.current_ch2_input_type = input_type;
    }

    replace_assign_for_input_channel(&s_ui.current_ch_fader_a_assign, input_ch, new_src);
    replace_assign_for_input_channel(&s_ui.current_ch_fader_b_assign, input_ch, new_src);
    replace_assign_for_input_channel(&s_ui.current_ch_fader_post_assign, input_ch, new_src);
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
        const bool select_insert = input_mode_uses_insert(s_ui.current_ch1_input_mode) || is_usb_assign(s_ui.current_ch_fader_a_assign);
        select_send_source(INPUT_CH1, select_insert);
    }
    else if (input_ch == INPUT_CH2)
    {
        const bool select_insert = input_mode_uses_insert(s_ui.current_ch2_input_mode) || is_usb_assign(s_ui.current_ch_fader_b_assign);
        select_send_source(INPUT_CH2, select_insert);
    }
}

static void apply_ch_fader_assign_a(uint8_t input_ch)
{
    select_ch_fader_assign_a_source(input_ch);
    s_ui.current_ch_fader_a_assign = current_input_src_from_channel(input_ch);
    apply_send_source_selection(INPUT_CH1);
}

static void apply_ch_fader_assign_b(uint8_t input_ch)
{
    select_ch_fader_assign_b_source(input_ch);
    s_ui.current_ch_fader_b_assign = current_input_src_from_channel(input_ch);
    apply_send_source_selection(INPUT_CH2);
}

static void apply_ch_fader_assign_post(uint8_t input_ch)
{
    select_ch_fader_assign_post_source(input_ch);
    s_ui.current_ch_fader_post_assign = current_input_src_from_channel(input_ch);
}

static void apply_dry_wet_value(uint16_t value)
{
    switch (s_ui.current_return_assign)
    {
    case INPUT_SRC_USB12:
        control_dryA_out_gain(value);
        control_dryB_out_gain(0U);
        break;
    case INPUT_SRC_USB34:
        control_dryA_out_gain(0U);
        control_dryB_out_gain(value);
        break;
    case INPUT_SRC_NONE:
    default:
        // A zero Dry/Wet value maps to unity dry and zero wet.
        control_dryA_out_gain(0U);
        control_dryB_out_gain(0U);
        control_wet_out_gain(0U);
        return;
    }
    control_wet_out_gain(value);
}

static void apply_return_source(uint8_t input_ch)
{
    if ((input_ch != INPUT_USB12) && (input_ch != INPUT_USB34))
    {
        s_ui.current_return_assign = INPUT_SRC_NONE;
        mute_input_from_return();
        apply_dry_wet_value(s_ui.pot_val[POT_CH_DRY_WET]);
        return;
    }

    select_return_ch_source(input_ch);
    s_ui.current_return_assign = current_input_src_from_channel(input_ch);
    control_input_from_return_gain(s_ui.pot_val[POT_CH_RETURN_IN]);
    apply_dry_wet_value(s_ui.pot_val[POT_CH_DRY_WET]);
}

static void apply_hp_out_source(uint8_t source)
{
    if (source > CUE_SEL_MST)
    {
        return;
    }

    select_hp_out_source(source);
    s_ui.current_hp_out_source = source;
}

static void apply_input_mode(uint8_t input_ch, UI_InputMode_t mode)
{
    const bool enable_insert = input_mode_uses_insert((uint8_t) mode);

    set_input_insert_enabled(input_ch, enable_insert);
    if (input_ch == INPUT_CH1)
    {
        s_ui.current_ch1_input_mode = (uint8_t) mode;
    }
    else if (input_ch == INPUT_CH2)
    {
        s_ui.current_ch2_input_mode = (uint8_t) mode;
    }
    apply_send_source_selection(input_ch);
    ui_control_reapply_ch_fader_outputs();
}

static void apply_ch_fader_dvs_delay(uint8_t delay_ms)
{
    const uint8_t clamped_delay_ms = (delay_ms > UI_CH_FADER_DVS_DELAY_MAX_MS)
                                         ? UI_CH_FADER_DVS_DELAY_MAX_MS
                                         : delay_ms;

    if (s_ui.ch_fader_dvs_delay_ms == clamped_delay_ms)
    {
        return;
    }

    s_ui.ch_fader_dvs_delay_ms = clamped_delay_ms;
    // A new delay must not reinterpret updates captured with the old setting.
    ui_control_reapply_ch_fader_outputs();
}

static bool assign_to_input_ch(uint8_t assign, uint8_t* input_ch)
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

static bool assign_to_return_input_ch(uint8_t assign, uint8_t* input_ch)
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

static uint8_t midi_program_for_input_type(uint8_t input_ch, uint8_t input_type)
{
    if (input_ch == INPUT_CH1)
    {
        return (input_type == INPUT_TYPE_PHONO) ? CH1_PHONO : CH1_LINE;
    }

    if (input_ch == INPUT_CH2)
    {
        return (input_type == INPUT_TYPE_PHONO) ? CH2_PHONO : CH2_LINE;
    }

    return CH1_LINE;
}

static uint8_t midi_program_for_ch_fader_assign_a(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return CH_FADER_ASSIGN_A_CH1;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return CH_FADER_ASSIGN_A_CH2;
    case INPUT_SRC_USB12:
        return CH_FADER_ASSIGN_A_USB12;
    case INPUT_SRC_USB34:
        return CH_FADER_ASSIGN_A_USB34;
    default:
        return CH_FADER_ASSIGN_A_CH1;
    }
}

static uint8_t midi_program_for_ch_fader_assign_b(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return CH_FADER_ASSIGN_B_CH1;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return CH_FADER_ASSIGN_B_CH2;
    case INPUT_SRC_USB12:
        return CH_FADER_ASSIGN_B_USB12;
    case INPUT_SRC_USB34:
        return CH_FADER_ASSIGN_B_USB34;
    default:
        return CH_FADER_ASSIGN_B_CH1;
    }
}

static uint8_t midi_program_for_ch_fader_assign_post(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return CH_FADER_ASSIGN_POST_CH1;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return CH_FADER_ASSIGN_POST_CH2;
    case INPUT_SRC_USB12:
        return CH_FADER_ASSIGN_POST_USB12;
    case INPUT_SRC_USB34:
        return CH_FADER_ASSIGN_POST_USB34;
    default:
        return CH_FADER_ASSIGN_POST_CH1;
    }
}

static uint8_t midi_program_for_input_mode(uint8_t input_ch, uint8_t mode)
{
    if (input_ch == INPUT_CH1)
    {
        if (mode == UI_INPUT_MODE_DVS)
        {
            return CH1_MODE_DVS;
        }
        if (mode == UI_INPUT_MODE_SYNTH)
        {
            return CH1_MODE_SYNTH;
        }
        return CH1_MODE_DISABLE;
    }

    if (input_ch == INPUT_CH2)
    {
        if (mode == UI_INPUT_MODE_DVS)
        {
            return CH2_MODE_DVS;
        }
        if (mode == UI_INPUT_MODE_SYNTH)
        {
            return CH2_MODE_SYNTH;
        }
        return CH2_MODE_DISABLE;
    }

    return CH1_MODE_DISABLE;
}

static uint8_t midi_program_for_return_assign(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_USB12:
        return RETURN_CH_USB12;
    case INPUT_SRC_USB34:
        return RETURN_CH_USB34;
    case INPUT_SRC_NONE:
        return RETURN_CH_NONE;
    default:
        return RETURN_CH_USB34;
    }
}

static uint8_t midi_program_for_hp_out_source(uint8_t source)
{
    switch (source)
    {
    case CUE_SEL_CH_FADER_A:
        return HP_OUT_CH_FADER_A;
    case CUE_SEL_CH_FADER_B:
        return HP_OUT_CH_FADER_B;
    case CUE_SEL_THRU:
        return HP_OUT_THRU;
    case CUE_SEL_MST:
        return HP_OUT_MASTER;
    default:
        return HP_OUT_MASTER;
    }
}

static uint8_t midi_program_for_ch_fader_aux_assignment(uint8_t sensor_idx, uint8_t assign)
{
    if (sensor_idx == 2U)
    {
        return (assign == UI_CH_FADER_AUX_ASSIGN_A) ? CH_FADER_AUX_SENSOR2_TO_A : CH_FADER_AUX_SENSOR2_TO_B;
    }

    return (assign == UI_CH_FADER_AUX_ASSIGN_A) ? CH_FADER_AUX_SENSOR3_TO_A : CH_FADER_AUX_SENSOR3_TO_B;
}

static uint8_t midi_program_for_timecode_synth_ratio_set(uint8_t ratio_set)
{
    if (ratio_set == TIMECODE_RATIO_HARMONIC)
    {
        return SYNTH_RATIO_HARMONIC;
    }
    if (ratio_set == TIMECODE_RATIO_CHORD)
    {
        return SYNTH_RATIO_CHORD;
    }
    return SYNTH_RATIO_OCTAVE;
}

static uint8_t midi_program_for_timecode_synth_warp_algorithm(uint8_t warp_algorithm)
{
    switch (warp_algorithm)
    {
    case TIMECODE_WARP_CLEAN:
        return SYNTH_WARP_CLEAN;
    case TIMECODE_WARP_RING_MOD:
        return SYNTH_WARP_RING_MOD;
    case TIMECODE_WARP_COMPARATOR:
        return SYNTH_WARP_COMPARATOR;
    case TIMECODE_WARP_CROSSFOLD:
    default:
        return SYNTH_WARP_CROSSFOLD;
    }
}

static void send_midi_config_dump(const EEPROM_DeviceConfig_t* cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    send_program_change(midi_program_for_input_type(INPUT_CH1, cfg->current_ch1_input_type), MIDI_CH_15);
    send_program_change(midi_program_for_input_type(INPUT_CH2, cfg->current_ch2_input_type), MIDI_CH_15);
    send_program_change(midi_program_for_ch_fader_assign_a(cfg->current_ch_fader_a_assign), MIDI_CH_15);
    send_program_change(midi_program_for_ch_fader_assign_b(cfg->current_ch_fader_b_assign), MIDI_CH_15);
    send_program_change(midi_program_for_ch_fader_assign_post(cfg->current_ch_fader_post_assign), MIDI_CH_15);
    send_program_change(midi_program_for_return_assign(cfg->current_return_assign), MIDI_CH_15);
    send_program_change(midi_program_for_hp_out_source(cfg->current_hp_out_source), MIDI_CH_15);
    send_program_change(midi_program_for_input_mode(INPUT_CH1, cfg->current_ch1_input_mode), MIDI_CH_15);
    send_program_change(midi_program_for_input_mode(INPUT_CH2, cfg->current_ch2_input_mode), MIDI_CH_15);
    send_program_change(midi_program_for_ch_fader_aux_assignment(2U, cfg->sensor2_aux_fade_down_assign), MIDI_CH_15);
    send_program_change(midi_program_for_ch_fader_aux_assignment(3U, cfg->sensor3_aux_fade_down_assign), MIDI_CH_15);
    send_program_change((cfg->ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_A) != 0U ? CH_FADER_REVERSE_A_ON : CH_FADER_REVERSE_A_OFF, MIDI_CH_15);
    send_program_change((cfg->ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_B) != 0U ? CH_FADER_REVERSE_B_ON : CH_FADER_REVERSE_B_OFF, MIDI_CH_15);
    send_program_change(midi_program_for_timecode_synth_ratio_set(cfg->timecode_synth_ratio_set), MIDI_CH_15);
    send_program_change(midi_program_for_timecode_synth_warp_algorithm(cfg->timecode_synth_warp_algorithm), MIDI_CH_15);
    send_control_change(MIDI_CC_CH_FADER_CURVE_A, ch_fader_curve_width_to_midi_cc(cfg->current_ch_fader_curve_width_a), MIDI_CH_15);
    send_control_change(MIDI_CC_CH_FADER_CURVE_B, ch_fader_curve_width_to_midi_cc(cfg->current_ch_fader_curve_width_b), MIDI_CH_15);
    send_control_change(MIDI_CC_CH_FADER_DVS_DELAY, cfg->ch_fader_dvs_delay_ms, MIDI_CH_15);
    if ((cfg->mag_output_mode_flags & EEPROM_CFG_FLAG_MAG_OUT_AS_NOTE) != 0U)
    {
        send_program_change(MIDI_PC_MUX_OUTPUT_NOTE, MIDI_CH_15);
    }
    else
    {
        send_program_change(MIDI_PC_MUX_OUTPUT_CC, MIDI_CH_15);
    }
}

static void set_pot_mux_channel(uint8_t channel)
{
    static const uint8_t mux_bits[POT_NUM][4] = {
        {0, 0, 0, 0}, // 0  l0
        {1, 0, 0, 0}, // 1  l1
        {0, 1, 0, 0}, // 2  l2
        {1, 1, 0, 0}, // 3  l3
        {0, 0, 1, 0}, // 4  l4
        {1, 0, 1, 0}, // 5  l5
        {0, 1, 1, 0}, // 6  r0
        {1, 1, 1, 0}, // 7  r1
        {0, 0, 0, 1}, // 8  l2
        {1, 0, 0, 1}, // 9  r3
        {0, 1, 0, 1}, // 10 r4
        {1, 1, 0, 1}, // 11 r5
        {0, 0, 1, 1}, // 12 sub_keys
        {0, 1, 1, 1}, // 14
        {1, 0, 1, 1}, // 13
        {1, 1, 1, 1}, // 15
    };

    if (channel >= POT_NUM)
    {
        channel = 0;
    }

    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, mux_bits[channel][0]);
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, mux_bits[channel][1]);
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, mux_bits[channel][2]);
    HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, mux_bits[channel][3]);
}

static void apply_pot_value(uint8_t channel, uint16_t value)
{
    const bool synth_mode_active =
        (s_ui.current_ch1_input_mode == UI_INPUT_MODE_SYNTH) ||
        (s_ui.current_ch2_input_mode == UI_INPUT_MODE_SYNTH);

    switch (channel)
    {
    case POT_CH_CC0:
        audio_control_set_timecode_synth_control(TIMECODE_SYNTH_CONTROL_ROOT, (uint8_t) value);
        if (!synth_mode_active)
        {
            send_control_change(0, value, 0);
        }
        break;
    case POT_CH_CC1:
        audio_control_set_timecode_synth_control(TIMECODE_SYNTH_CONTROL_MORPH, (uint8_t) value);
        if (!synth_mode_active)
        {
            send_control_change(1, value, 0);
        }
        break;
    case POT_CH_CH1_IN:
        control_input_from_ch1_gain(value);
        break;
    case POT_CH_CC2:
        audio_control_set_timecode_synth_control(TIMECODE_SYNTH_CONTROL_SLOPE, (uint8_t) value);
        if (!synth_mode_active)
        {
            send_control_change(2, value, 0);
        }
        break;
    case POT_CH_CC3:
        audio_control_set_timecode_synth_control(TIMECODE_SYNTH_CONTROL_SMOOTH_FOLD, (uint8_t) value);
        if (!synth_mode_active)
        {
            send_control_change(3, value, 0);
        }
        break;
    case POT_CH_CC4:
        audio_control_set_timecode_synth_control(TIMECODE_SYNTH_CONTROL_WARP_AMOUNT, (uint8_t) value);
        if (!synth_mode_active)
        {
            send_control_change(4, value, 0);
        }
        break;
    case POT_CH_CH2_IN:
        control_input_from_ch2_gain(value);
        break;
    case POT_CH_CH1_OUT:
        control_ch1_out_gain(value);
        break;
    case POT_CH_CH2_OUT:
        control_ch2_out_gain(value);
        break;
    case POT_CH_DRY_WET:
        apply_dry_wet_value(value);
        break;
    case POT_CH_RETURN_IN:
        if (s_ui.current_return_assign == INPUT_SRC_NONE)
        {
            mute_input_from_return();
        }
        else
        {
            control_input_from_return_gain(value);
        }
        break;
    case POT_CH_HP_OUT:
        control_hp_out_gain(value);
        break;
    case POT_CH_MAG0:
    case POT_CH_MAG1:
    case POT_CH_MAG2:
    case POT_CH_MAG3:
        emit_mag_output(channel, (uint8_t) (68U + (channel - POT_MAG_CH_FIRST)), (uint8_t) value);
        break;
    default:
        break;
    }
}

void ui_control_reapply_pot_outputs(void)
{
    static const uint8_t s_pot_output_channels[] = {
        POT_CH_CH1_IN,
        POT_CH_CH2_IN,
        POT_CH_CH1_OUT,
        POT_CH_CH2_OUT,
        POT_CH_DRY_WET,
        POT_CH_RETURN_IN,
        POT_CH_HP_OUT,
    };

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_pot_output_channels); i++)
    {
        const uint8_t ch = s_pot_output_channels[i];
        apply_pot_value(ch, s_ui.pot_val[ch]);
    }
}

static bool is_pot_mag_channel(uint8_t channel)
{
    return (channel >= POT_MAG_CH_FIRST) && (channel <= POT_MAG_CH_LAST);
}

static uint8_t pot_mag_index(uint8_t channel)
{
    return (uint8_t) (channel - POT_MAG_CH_FIRST);
}

static bool is_pot_hysteresis_channel(uint8_t channel)
{
    return channel < POT_HYSTERESIS_NUM;
}

static bool is_pot_cc_channel(uint8_t channel)
{
    switch (channel)
    {
    case POT_CH_CC0:
    case POT_CH_CC1:
    case POT_CH_CC2:
    case POT_CH_CC3:
    case POT_CH_CC4:
        return true;
    default:
        return false;
    }
}

static uint32_t read_pot_sample_from_adc(uint8_t channel, uint32_t adc_raw)
{
    if (is_pot_cc_channel(channel))
    {
        return adc_raw >> 5;
    }

    switch (channel)
    {
    case POT_CH_CH1_IN:   // l2
    case POT_CH_CH2_IN:   // r0
    case POT_CH_CH1_OUT:  // r1
    case POT_CH_CH2_OUT:  // r2
    case POT_CH_DRY_WET:  // r3
    case POT_CH_RETURN_IN: // r4
    case POT_CH_HP_OUT:   // r5
        return adc_raw >> 2;
    case POT_CH_MAG0: // sub_key6
    case POT_CH_MAG1: // sub_key7
    case POT_CH_MAG2: // sub_key8
    case POT_CH_MAG3: // sub_key9
        return adc_raw;
    default:
        return 0;
    }
}

static uint16_t quantize_pot_hysteresis_value(uint8_t channel, uint16_t adc_raw)
{
    if (is_pot_cc_channel(channel))
    {
        uint32_t value = ((uint32_t) adc_raw + 16U) >> 5;
        if (value > 127U)
        {
            value = 127U;
        }
        if (value <= POT_CC_MIN_DEADZONE_VALUE)
        {
            value = 0U;
        }
        return (uint16_t) value;
    }

    {
        uint32_t value = (uint32_t) adc_raw >> 2;
        if (value > 1023U)
        {
            value = 1023U;
        }
        return (uint16_t) value;
    }
}

static bool should_apply_pot_hysteresis(uint8_t channel, uint16_t raw_avg, uint16_t* value_out)
{
    const uint8_t idx = channel;
    const uint8_t shift = is_pot_cc_channel(channel) ? 5U : 2U;
    const uint16_t candidate = quantize_pot_hysteresis_value(channel, raw_avg);

    if (!s_ui.pot_hysteresis_has_last_sent[idx])
    {
        s_ui.pot_hysteresis_last_sent[idx]     = candidate;
        s_ui.pot_hysteresis_has_last_sent[idx] = true;
        *value_out = candidate;
        return true;
    }

    const uint16_t last_sent = s_ui.pot_hysteresis_last_sent[idx];
    if (candidate == last_sent)
    {
        return false;
    }

    if (candidate > last_sent)
    {
        const uint32_t threshold = (((uint32_t) last_sent + 1U) << shift) + POT_CC_HYSTERESIS_RAW;
        if ((uint32_t) raw_avg < threshold)
        {
            return false;
        }
    }
    else
    {
        const uint32_t threshold = ((uint32_t) last_sent << shift);
        if (((uint32_t) raw_avg + POT_CC_HYSTERESIS_RAW) >= threshold)
        {
            return false;
        }
    }

    s_ui.pot_hysteresis_last_sent[idx] = candidate;
    *value_out = candidate;
    return true;
}

static void process_pot(void)
{
    if (s_ui.pot_ch_counter < POT_CH_SEL_WAIT)
    {
        set_pot_mux_channel(s_ui.pot_ch);
        s_ui.pot_ch_counter++;
    }
    else if (s_ui.pot_ch_counter >= POT_CH_SEL_WAIT)
    {
        const uint8_t ch = s_ui.pot_ch;
        const uint16_t ma_index = s_ui.pot_ma_index[ch];
        const uint16_t sample_now = (uint16_t) read_pot_sample_from_adc(ch, adc_val[6]);

        s_ui.pot_val_ma[ch][ma_index] = sample_now;
        if (s_ui.pot_sample_count[ch] < POT_MA_SIZE)
        {
            s_ui.pot_sample_count[ch]++;
        }

        if (is_pot_hysteresis_channel(ch))
        {
            s_ui.pot_hysteresis_raw_ma[ch][ma_index] = (uint16_t) adc_val[6];
        }

        s_ui.pot_ma_index[ch] = (ma_index + 1U) % POT_MA_SIZE;

        if (is_pot_mag_channel(ch))
        {
            // Pot-mag channels (12-15): prioritize tracking speed over MA smoothing.
            s_ui.pot_val[ch] = sample_now;
        }
        else
        {
            float pot_sum = 0.0f;
            const uint8_t sample_count = s_ui.pot_sample_count[ch];
            for (uint8_t j = 0; j < sample_count; j++)
            {
                pot_sum += (float) s_ui.pot_val_ma[ch][j];
            }
            s_ui.pot_val[ch] = round(pot_sum / (float) sample_count);
        }

        if (!is_pot_mag_channel(ch))
        {
            if (is_pot_hysteresis_channel(ch))
            {
                uint32_t raw_sum = 0U;
                const uint8_t sample_count = s_ui.pot_sample_count[ch];
                uint16_t stabilized_value = s_ui.pot_val[ch];

                for (uint8_t j = 0; j < sample_count; j++)
                {
                    raw_sum += s_ui.pot_hysteresis_raw_ma[ch][j];
                }

                if (should_apply_pot_hysteresis(ch, (uint16_t) (raw_sum / sample_count), &stabilized_value))
                {
                    s_ui.pot_val[ch] = stabilized_value;
                    apply_pot_value(ch, stabilized_value);
                }
            }
        }
        else
        {
            const uint8_t idx     = pot_mag_index(ch);
            const uint16_t sample = s_ui.pot_val[ch];

            if (s_ui.pot_mag_calibration_count[idx] < MAG_CALIBRATION_COUNT_MAX)
            {
                s_ui.pot_mag_offset_sum[idx] += sample;
                s_ui.pot_mag_calibration_count[idx]++;
            }
            else if (s_ui.pot_mag_calibration_count[idx] == MAG_CALIBRATION_COUNT_MAX)
            {
                s_ui.pot_mag_offset[idx] = (uint16_t) (s_ui.pot_mag_offset_sum[idx] / MAG_CALIBRATION_COUNT_MAX);
                s_ui.pot_mag_calibration_count[idx]++;
            }
            else
            {
                const uint16_t offset = s_ui.pot_mag_offset[idx];
                uint8_t candidate     = 0U;

                if (sample < (uint16_t) (offset + MAG_CH_FADER_CUTOFF))
                {
                    candidate = 0U;
                }
                else if (sample <= (uint16_t) (offset + MAG_CH_FADER_RANGE))
                {
                    const uint16_t normalized = (uint16_t) (sample - offset - MAG_CH_FADER_CUTOFF);
                    candidate                 = (uint8_t) ((uint32_t) normalized * 127U / MAG_CH_FADER_RANGE);
                }
                else
                {
                    candidate = 127U;
                }

                // Continuous control path for pot-mag channels:
                // no extra smoothing for faster tracking.
                s_ui.pot_mag_candidate[idx] = candidate;

                {
                    const uint8_t filtered = s_ui.pot_mag_candidate[idx];
                    const uint8_t diff     = (filtered > s_ui.pot_mag_state[idx]) ? (uint8_t) (filtered - s_ui.pot_mag_state[idx]) : (uint8_t) (s_ui.pot_mag_state[idx] - filtered);
                    if (diff >= 1U)
                    {
                        s_ui.pot_mag_state[idx] = filtered;
                        apply_pot_value(ch, filtered);
                    }
                }
            }
        }

        s_ui.pot_ch         = (s_ui.pot_ch + 1) % POT_NUM;
        s_ui.pot_ch_counter = 0;
    }
}

static void update_mag_samples(void)
{
    for (int i = 0; i < MAG_SW_NUM; i++)
    {
        s_ui.mag_val[i] = (uint16_t) adc_val[i];

        if (s_ui.mag_calibration_count < MAG_CALIBRATION_COUNT_MAX)
        {
            s_ui.mag_offset_sum[i] += adc_val[i];
        }
        else if (s_ui.mag_calibration_count == MAG_CALIBRATION_COUNT_MAX)
        {
            s_ui.mag_offset[i] = s_ui.mag_offset_sum[i] / MAG_CALIBRATION_COUNT_MAX;
        }
    }
    if (s_ui.mag_calibration_count <= MAG_CALIBRATION_COUNT_MAX)
    {
        s_ui.mag_calibration_count++;
    }
}

// Lookup for paired ch_fader endpoints (fade-up side -> fade-down side).
static int8_t get_pair_fade_down_index_from_up(uint8_t fade_up_idx)
{
    if (fade_up_idx >= MAG_SW_NUM)
    {
        return -1;
    }

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
    {
        if (s_ch_fader_pairs[i].fade_up_idx == fade_up_idx)
        {
            return (int8_t) s_ch_fader_pairs[i].fade_down_idx;
        }
    }

    return -1;
}

// Reverse lookup for paired ch_fader endpoints (fade-down side -> fade-up side).
static int8_t get_pair_fade_up_index_from_down(uint8_t fade_down_idx)
{
    if (fade_down_idx >= MAG_SW_NUM)
    {
        return -1;
    }

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
    {
        if (s_ch_fader_pairs[i].fade_down_idx == fade_down_idx)
        {
            return (int8_t) s_ch_fader_pairs[i].fade_up_idx;
        }
    }

    return -1;
}

// Add a small touch-onset deadband for pair tracking so untouched sensors do not
// perturb the held channel fader state at extreme curve settings.
static float apply_ch_fader_pair_onset_deadband(uint8_t i, float raw)
{
    if (raw < 0.0f)
    {
        raw = 0.0f;
    }
    else if (raw > 1.0f)
    {
        raw = 1.0f;
    }

    if (get_pair_fade_down_index_from_up(i) >= 0)
    {
        if (raw <= CH_FADER_PAIR_ONSET_DEADBAND)
        {
            return 0.0f;
        }

        return (raw - CH_FADER_PAIR_ONSET_DEADBAND) / (1.0f - CH_FADER_PAIR_ONSET_DEADBAND);
    }

    if (get_pair_fade_up_index_from_down(i) >= 0)
    {
        const float high_deadband = 1.0f - CH_FADER_PAIR_ONSET_DEADBAND;

        if (raw >= high_deadband)
        {
            return 1.0f;
        }

        return raw / high_deadband;
    }

    return raw;
}

// Convert one magnetic sensor sample into normalized ch_fader raw [0..1].
static void update_raw_ch_fader_from_mag(uint8_t i)
{
    // Fade-up sensors rise from 0->1; fade-down and aux sensors invert 1->0.
    if (get_pair_fade_down_index_from_up(i) >= 0)
    {
        if (s_ui.mag_val[i] < s_ui.mag_offset[i] + MAG_CH_FADER_CUTOFF)
        {
            s_ui.ch_fader.raw[i] = 0.0f;
        }
        else if (s_ui.mag_val[i] >= s_ui.mag_offset[i] + MAG_CH_FADER_CUTOFF && s_ui.mag_val[i] <= s_ui.mag_offset[i] + MAG_CH_FADER_RANGE)
        {
            s_ui.ch_fader.raw[i] = (float) (s_ui.mag_val[i] - s_ui.mag_offset[i] - MAG_CH_FADER_CUTOFF) / (float) MAG_CH_FADER_RANGE;
        }
        else if (s_ui.mag_val[i] > s_ui.mag_offset[i] + MAG_CH_FADER_RANGE)
        {
            s_ui.ch_fader.raw[i] = 1.0f;
        }
    }
    else
    {
        if (s_ui.mag_val[i] < s_ui.mag_offset[i] + MAG_CH_FADER_CUTOFF)
        {
            s_ui.ch_fader.raw[i] = 1.0f;
        }
        else if (s_ui.mag_val[i] >= s_ui.mag_offset[i] + MAG_CH_FADER_CUTOFF && s_ui.mag_val[i] <= s_ui.mag_offset[i] + MAG_CH_FADER_RANGE)
        {
            s_ui.ch_fader.raw[i] = 1.0f - ((float) (s_ui.mag_val[i] - s_ui.mag_offset[i] - MAG_CH_FADER_CUTOFF) / (float) MAG_CH_FADER_RANGE);
        }
        else if (s_ui.mag_val[i] > s_ui.mag_offset[i] + MAG_CH_FADER_RANGE)
        {
            s_ui.ch_fader.raw[i] = 0.0f;
        }
    }
}

// Update peak/valley trackers used to build stable pair outputs.
static void update_ch_fader_extrema(uint8_t i)
{
    const float pair_raw = apply_ch_fader_pair_onset_deadband(i, s_ui.ch_fader.raw[i]);

    if (get_pair_fade_down_index_from_up(i) >= 0)
    {
        // Track fade-up peak; paired fade-down floor is synchronized from this edge.
        if (pair_raw > (s_ui.ch_fader.up_peak[i] + CH_FADER_EXTREMA_HYSTERESIS))
        {
            s_ui.ch_fader.up_peak[i] = pair_raw;

            const int8_t fade_down_idx = get_pair_fade_down_index_from_up(i);
            if (fade_down_idx >= 0)
            {
                s_ui.ch_fader.down_floor[(uint8_t) fade_down_idx] = s_ui.ch_fader.up_peak[i];
            }
        }

        // Keep paired minimum re-synchronized while the source side stays near full scale.
        // This avoids "stuck min" when ch_fader[0]/ch_fader[5] remains high and only the paired side moves.
        if (pair_raw >= CH_FADER_PAIR_RESET_THRESHOLD)
        {
            const int8_t fade_down_idx = get_pair_fade_down_index_from_up(i);
            if (fade_down_idx >= 0)
            {
                s_ui.ch_fader.down_floor[(uint8_t) fade_down_idx] = pair_raw;
            }
        }
    }
    else if (get_pair_fade_up_index_from_down(i) >= 0)
    {
        // Track fade-down floor; when near zero, clear paired fade-up peak.
        if (pair_raw < (s_ui.ch_fader.down_floor[i] - CH_FADER_EXTREMA_HYSTERESIS))
        {
            s_ui.ch_fader.down_floor[i] = pair_raw;

            if (s_ui.ch_fader.down_floor[i] < CH_FADER_MIN_RESET_CUTOFF)
            {
                const int8_t fade_up_idx = get_pair_fade_up_index_from_down(i);
                if (fade_up_idx >= 0)
                {
                    s_ui.ch_fader.up_peak[(uint8_t) fade_up_idx] = 0.0f;
                }
            }
        }
    }
    else
    {
        // No extrema tracking for pair-independent aux sensors.
    }
}

static void update_one_ch_fader_index(uint8_t i, bool processed[MAG_SW_NUM])
{
    if ((i >= MAG_SW_NUM) || processed[i])
    {
        return;
    }

    update_raw_ch_fader_from_mag(i);
    update_ch_fader_extrema(i);
    processed[i] = true;
}

// Full per-scan ch_fader update pipeline: raw normalization then extrema tracking.
static void update_ch_fader_from_mag(void)
{
    bool processed[MAG_SW_NUM] = {false};

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
    {
        update_one_ch_fader_index(s_ch_fader_pairs[i].fade_up_idx, processed);
    }

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
    {
        update_one_ch_fader_index(s_ch_fader_pairs[i].fade_down_idx, processed);
    }

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
    {
        update_one_ch_fader_index(s_ch_fader_pairs[i].aux_fade_down_idx, processed);
        update_one_ch_fader_index(s_ch_fader_pairs[i].aux2_fade_down_idx, processed);
    }

    for (uint8_t i = 0; i < MAG_SW_NUM; i++)
    {
        update_one_ch_fader_index(i, processed);
    }
}

// Emit MIDI CC only when raw ch_fader changes enough to justify traffic.
static void emit_ch_fader_cc_if_needed(uint8_t i)
{
    const uint8_t note = (uint8_t) (60U + i);
    uint8_t value      = ch_fader_to_cc(s_ui.ch_fader.raw[i]);

    if (get_pair_fade_down_index_from_up(i) >= 0)
    {
        value = (uint8_t) (127U - value);
    }

    if (s_ui.mag_out_as_note)
    {
        emit_ch_fader_note_if_needed(i, note, value);
        s_ui.ch_fader.prev[i] = s_ui.ch_fader.raw[i];
        return;
    }

    if (s_ui.ch_fader.note_is_on[i] || s_ui.ch_fader.note_scan_active[i])
    {
        if (s_ui.ch_fader.note_is_on[i])
        {
            send_note(note, 0U, 0U);
        }
        clear_ch_fader_note_state(i);
    }

    // MIDI CC updates use a larger threshold to limit traffic and jitter.
    if (fabs(s_ui.ch_fader.raw[i] - s_ui.ch_fader.prev[i]) > CH_FADER_CC_UPDATE_THRESHOLD)
    {
        emit_mag_output((uint8_t) (20U + i), note, value);
        s_ui.ch_fader.prev[i] = s_ui.ch_fader.raw[i];
    }
}

static float clamp01(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }
    if (value > 1.0f)
    {
        return 1.0f;
    }

    return value;
}

// Convert MIDI's unipolar 0..127 curve setting into a bipolar amount while
// keeping CC64 exactly linear. Negative values rise near the end of travel;
// positive values rise near the start.
static float midi_cc_to_ch_fader_curve_amount(uint8_t value)
{
    if (value <= 64U)
    {
        return ((float) value - 64.0f) / 64.0f;
    }

    return ((float) value - 64.0f) / 63.0f;
}

// Stable early-rise exponential curve: (1 - exp(-k*t)) / (1 - exp(-k)).
// The late-rise side is produced by point reflection, avoiding exp(+k) and
// preserving exact symmetry between the two curve extremes.
static float compute_ch_fader_exponential_curve(float position, float curve_amount)
{
    const float t = clamp01(position);
    const float magnitude = fabsf(curve_amount);

    if (magnitude <= CH_FADER_PAIR_CURVE_LINEAR_EPSILON)
    {
        return t;
    }

    // Cubic shaping allocates more CC resolution around the linear midpoint
    // while preserving the same maximum curvature at both endpoints.
    const float shaped_magnitude = magnitude * magnitude * magnitude;
    const float k = CH_FADER_PAIR_CURVE_EXPONENT_MAX * shaped_magnitude;
    const float denominator = -expm1f(-k);
    float result;

    if (curve_amount > 0.0f)
    {
        result = -expm1f(-k * t) / denominator;
    }
    else
    {
        result = 1.0f - (-expm1f(-k * (1.0f - t)) / denominator);
    }

    return clamp01(result);
}

float ui_control_evaluate_ch_fader_curve_preview(uint8_t cc_value, float normalized_preview_position)
{
    const float curve_amount = midi_cc_to_ch_fader_curve_amount(cc_value);
    return compute_ch_fader_exponential_curve(normalized_preview_position, curve_amount);
}

// Fade-down sensors idle near 1.0 and move toward 0.0 when pressed.
// Keep a small high-end deadband so light touch/noise does not start a cut.
static float apply_ch_fader_fade_down_onset_deadband(float raw)
{
    const float high_deadband = 1.0f - CH_FADER_PAIR_ONSET_DEADBAND;

    raw = clamp01(raw);
    if (raw >= high_deadband)
    {
        return 1.0f;
    }

    return raw / high_deadband;
}

// Arbitrate the fade-down sensors assigned to one ch_fader pair.
//
// source0 is the main fade-down sensor, and later sources are auxiliary sensors.
// The active source follows the most recently started cut gesture. When
// control switches to the other sensor, briefly force fade-down to "released"
// so repeated flick cuts remain audible even if the previous sensor is still
// held near the bottom.
static void update_multi_fade_down_source(uint8_t pair_idx,
                                          const float source[CH_FADER_FADE_DOWN_SOURCE_COUNT],
                                          bool fade_up_active)
{
    uint8_t active  = s_ui.ch_fader.fade_down_active_source[pair_idx];
    uint8_t started = CH_FADER_FADE_DOWN_SOURCE_NONE;
    bool force_release = false;

    for (uint8_t i = 0; i < CH_FADER_FADE_DOWN_SOURCE_COUNT; i++)
    {
        const float prev = s_ui.ch_fader.fade_down_source_prev[pair_idx][i];
        // Detect a new cut either from the idle zone or from a clear deeper
        // push on the non-active sensor.
        const bool newly_pressed = (prev >= CH_FADER_PAIR_FADE_DOWN_PRESS_THRESHOLD) &&
                                   (source[i] < CH_FADER_PAIR_FADE_DOWN_PRESS_THRESHOLD);
        const bool other_pressed_deeper = (i != active) &&
                                          (source[i] < CH_FADER_PAIR_RESET_THRESHOLD) &&
                                          ((prev - source[i]) >= CH_FADER_PAIR_FADE_DOWN_DEEPER_DELTA);

        if (newly_pressed || other_pressed_deeper)
        {
            started = i;
        }
    }

    if (started != CH_FADER_FADE_DOWN_SOURCE_NONE)
    {
        if ((active != CH_FADER_FADE_DOWN_SOURCE_NONE) && (started != active))
        {
            // Switching sources while the previous one is bottomed would
            // otherwise keep the audio muted and hide the second cut.
            s_ui.ch_fader.pair_fade_down_cut_active[pair_idx] = false;
            s_ui.ch_fader.pair_bottom_hold_active[pair_idx] = false;
            s_ui.ch_fader.pair_fade_down_bottomed[pair_idx] = false;
            s_ui.ch_fader.pair_top_restore_value[pair_idx] = 0.0f;
            s_ui.ch_fader.pair_top_hold_active[pair_idx] = false;
            s_ui.ch_fader.pair_fade_up_topped[pair_idx] = false;
            s_ui.ch_fader.fade_down_force_release_reads[pair_idx] =
                fade_up_active ? CH_FADER_FADE_DOWN_RETRIGGER_RELEASE_READS_WITH_FADE_UP
                               : CH_FADER_FADE_DOWN_RETRIGGER_RELEASE_READS_MOMENTARY;
        }
        active = started;
    }

    // Once all fade-down sensors are released, the next press can claim
    // ownership without being treated as a source switch.
    bool all_released = true;
    for (uint8_t i = 0; i < CH_FADER_FADE_DOWN_SOURCE_COUNT; i++)
    {
        if (source[i] < CH_FADER_PAIR_RESET_THRESHOLD)
        {
            all_released = false;
            break;
        }
    }
    if (all_released)
    {
        active = CH_FADER_FADE_DOWN_SOURCE_NONE;
    }

    s_ui.ch_fader.fade_down_active_source[pair_idx] = active;
    for (uint8_t i = 0; i < CH_FADER_FADE_DOWN_SOURCE_COUNT; i++)
    {
        s_ui.ch_fader.fade_down_source_prev[pair_idx][i] = source[i];
    }

    force_release = (s_ui.ch_fader.fade_down_force_release_reads[pair_idx] > 0U);
    if (force_release)
    {
        // Emit a short "released" window before applying the newly active
        // source. This creates the audible return between rapid cuts.
        s_ui.ch_fader.fade_down_force_release_reads[pair_idx]--;
        s_ui.ch_fader.fade_down_combined_raw[pair_idx] = 1.0f;
        return;
    }

    if (active != CH_FADER_FADE_DOWN_SOURCE_NONE)
    {
        // During a gesture, only the active source drives fade-down. This lets
        // the other sensor retrigger even if the first one remains deeper.
        s_ui.ch_fader.fade_down_combined_raw[pair_idx] = source[active];
        return;
    }

    // Idle/fallback behavior: use the deepest pressed fade-down source.
    float combined = source[0];
    for (uint8_t i = 1U; i < CH_FADER_FADE_DOWN_SOURCE_COUNT; i++)
    {
        if (source[i] < combined)
        {
            combined = source[i];
        }
    }
    s_ui.ch_fader.fade_down_combined_raw[pair_idx] = combined;
}

// Convert each pair's physical fade-down sensor(s) into one cached value.
// This is done once per ch_fader scan so retrigger state is not consumed by
// multiple later reads of get_ch_fader_pair_fade_down_raw().
static void update_ch_fader_pair_fade_down_source(const ch_fader_pair_runtime_t* pair)
{
    const uint8_t source_idx[CH_FADER_FADE_DOWN_SOURCE_COUNT] = {
        pair->fade_down_idx,
        pair->aux_fade_down_idx,
        pair->aux2_fade_down_idx,
    };
    float source[CH_FADER_FADE_DOWN_SOURCE_COUNT] = {1.0f, 1.0f, 1.0f};

    source[0] = apply_ch_fader_pair_onset_deadband(source_idx[0], s_ui.ch_fader.raw[source_idx[0]]);
    for (uint8_t i = 1U; i < CH_FADER_FADE_DOWN_SOURCE_COUNT; i++)
    {
        if (source_idx[i] < MAG_SW_NUM)
        {
            source[i] = apply_ch_fader_fade_down_onset_deadband(s_ui.ch_fader.raw[source_idx[i]]);
        }
    }

    const bool fade_up_active =
        apply_ch_fader_pair_onset_deadband(pair->fade_up_idx, s_ui.ch_fader.raw[pair->fade_up_idx]) > 0.0f;
    update_multi_fade_down_source(pair->prev_idx, source, fade_up_active);
}

static void update_ch_fader_fade_down_sources(void)
{
    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
    {
        update_ch_fader_pair_fade_down_source(&s_ch_fader_pairs[i]);
    }
}

static float get_ch_fader_pair_fade_down_raw(const ch_fader_pair_runtime_t* pair)
{
    return s_ui.ch_fader.fade_down_combined_raw[pair->prev_idx];
}

// Fade-up sensors idle near 0.0 and rise toward 1.0 when pressed.
static float get_ch_fader_pair_fade_up_raw(const ch_fader_pair_runtime_t* pair)
{
    return apply_ch_fader_pair_onset_deadband(pair->fade_up_idx, s_ui.ch_fader.raw[pair->fade_up_idx]);
}

static bool is_ch_fader_pair_reverse_enabled(const ch_fader_pair_runtime_t* pair)
{
    return (pair->prev_idx == CH_FADER_PAIR_A) ? s_ui.ch_fader_reverse_a : s_ui.ch_fader_reverse_b;
}

static float apply_ch_fader_pair_reverse(const ch_fader_pair_runtime_t* pair, float value)
{
    value = clamp01(value);
    return is_ch_fader_pair_reverse_enabled(pair) ? (1.0f - value) : value;
}

// Update the held output value for one ch_fader pair.
//
// The output is not a direct mix of sensors. Fade-up can raise the held value,
// fade-down can lower it, and releasing either side leaves the output where it
// was. A full fade-down press enters a bottom-hold state so the cut stays muted
// until the sensor leaves the bottom zone.
static float compute_ch_fader_pair_value(const ch_fader_pair_runtime_t* pair)
{
    // Current hold-value model:
    // - fade-up raises the held output toward its current ramped value
    // - fade-down lowers the held output toward its current ramped value
    // - releasing either side leaves the held output where it was
    // - pushing fade-down into the bottom zone forces the held output to 0
    // - when fade-down owns a two-finger gesture, fade-up stays open at the
    //   top and cuts when it backs away from the top
    const float curve_width = (pair->prev_idx == CH_FADER_PAIR_A) ? s_ui.ch_fader_curve_width_a : s_ui.ch_fader_curve_width_b;
    const uint8_t curve_cc  = ch_fader_curve_width_to_midi_cc(curve_width);
    const float curve_amount = midi_cc_to_ch_fader_curve_amount(curve_cc);
    const float fade_up          = get_ch_fader_pair_fade_up_raw(pair);
    const float fade_down        = get_ch_fader_pair_fade_down_raw(pair);
    // Bottom/top gesture states now follow the physical endpoints instead of
    // the curve setting, so CC64 remains linear across the complete travel.
    const float down_bottom_threshold       = 0.0f;
    const float down_bottom_hold_release_threshold = CH_FADER_PAIR_BOTTOM_HOLD_RELEASE_THRESHOLD;
    const float down_bottom_rehold_threshold       = CH_FADER_PAIR_BOTTOM_REHOLD_THRESHOLD;
    const float up_top_threshold            = 1.0f;
    const float up_top_hold_release_threshold = 1.0f - CH_FADER_PAIR_BOTTOM_HOLD_RELEASE_THRESHOLD;
    const float up_top_rehold_threshold       = 1.0f - CH_FADER_PAIR_BOTTOM_REHOLD_THRESHOLD;
    // After bottoming out, ignore further fade-down cuts until the sensor returns to unpressed.
    const float down_bottom_release_threshold = 1.0f;
    bool cut_active         = s_ui.ch_fader.pair_fade_down_cut_active[pair->prev_idx];
    bool bottom_hold_active = s_ui.ch_fader.pair_bottom_hold_active[pair->prev_idx];
    bool bottomed           = s_ui.ch_fader.pair_fade_down_bottomed[pair->prev_idx];
    bool top_hold_active    = s_ui.ch_fader.pair_top_hold_active[pair->prev_idx];
    bool topped             = s_ui.ch_fader.pair_fade_up_topped[pair->prev_idx];
    bool reverse_fade_down_takeover = s_ui.ch_fader.pair_reverse_fade_down_takeover[pair->prev_idx];
    bool bottom_entered     = false;
    bool top_entered        = false;
    float hold_value        = s_ui.ch_fader.pair_hold_value[pair->prev_idx];
    float restore_value     = s_ui.ch_fader.pair_bottom_restore_value[pair->prev_idx];
    float top_restore_value = s_ui.ch_fader.pair_top_restore_value[pair->prev_idx];
    const float up_gain     = compute_ch_fader_exponential_curve(fade_up, curve_amount);
    const float down_gain   = 1.0f - compute_ch_fader_exponential_curve(1.0f - fade_down, curve_amount);
    const bool fade_up_active = (up_gain > 0.0f);
    const bool fade_down_released = (fade_down >= CH_FADER_PAIR_RESET_THRESHOLD);
    const bool fade_down_pressed = !fade_down_released;
    const bool both_released = !fade_up_active && !fade_down_pressed;
    uint8_t gesture_parent = s_ui.ch_fader.pair_gesture_parent[pair->prev_idx];
    bool gesture_armed = s_ui.ch_fader.pair_gesture_armed[pair->prev_idx];
    bool gesture_child_active = s_ui.ch_fader.pair_gesture_child_active[pair->prev_idx];

    if ((gesture_parent == CH_FADER_GESTURE_PARENT_FADE_UP) && !fade_up_active)
    {
        gesture_parent = fade_down_pressed ? CH_FADER_GESTURE_PARENT_FADE_DOWN : CH_FADER_GESTURE_PARENT_NONE;
        gesture_armed  = (gesture_parent == CH_FADER_GESTURE_PARENT_NONE) && both_released;
        gesture_child_active = false;
    }
    else if ((gesture_parent == CH_FADER_GESTURE_PARENT_FADE_DOWN) && !fade_down_pressed)
    {
        gesture_parent = fade_up_active ? CH_FADER_GESTURE_PARENT_FADE_UP : CH_FADER_GESTURE_PARENT_NONE;
        gesture_armed  = (gesture_parent == CH_FADER_GESTURE_PARENT_NONE) && both_released;
        gesture_child_active = false;
    }

    if (gesture_parent == CH_FADER_GESTURE_PARENT_NONE)
    {
        if (both_released)
        {
            gesture_armed = true;
        }
        else if (gesture_armed)
        {
            if (fade_up_active && !fade_down_pressed)
            {
                gesture_parent = CH_FADER_GESTURE_PARENT_FADE_UP;
                gesture_armed  = false;
                gesture_child_active = false;
            }
            else if (fade_down_pressed && !fade_up_active)
            {
                gesture_parent = CH_FADER_GESTURE_PARENT_FADE_DOWN;
                gesture_armed  = false;
                gesture_child_active = false;
            }
            else
            {
                gesture_armed = false;
                gesture_child_active = false;
            }
        }
    }

    // With fade-up released, fade-down works as a reverse momentary control.
    // Soft takeover keeps the fade-up hold behavior unchanged until the
    // reversed fade-down curve reaches the current held value.
    if (!fade_up_active)
    {
        const float reverse_down_gain = 1.0f - down_gain;

        if (!reverse_fade_down_takeover &&
            fade_down_pressed &&
            ((reverse_down_gain + CH_FADER_SEND_THRESHOLD) >= hold_value))
        {
            reverse_fade_down_takeover = true;
        }

        if (reverse_fade_down_takeover)
        {
            if (fade_down_released)
            {
                hold_value = 0.0f;
                reverse_fade_down_takeover = false;
            }
            else
            {
                hold_value = reverse_down_gain;
            }
        }

        // The normal fade-down cut state is only used while fade-up is active.
        cut_active          = false;
        bottom_hold_active  = false;
        bottomed            = false;
        top_hold_active     = false;
        topped              = false;
        restore_value       = 0.0f;
        top_restore_value   = 0.0f;
        gesture_child_active = false;

        s_ui.ch_fader.pair_hold_value[pair->prev_idx] = hold_value;
        s_ui.ch_fader.pair_bottom_restore_value[pair->prev_idx] = restore_value;
        s_ui.ch_fader.pair_top_restore_value[pair->prev_idx] = top_restore_value;
        s_ui.ch_fader.pair_gesture_parent[pair->prev_idx] = gesture_parent;
        s_ui.ch_fader.pair_gesture_armed[pair->prev_idx] = gesture_armed;
        s_ui.ch_fader.pair_gesture_child_active[pair->prev_idx] = gesture_child_active;
        s_ui.ch_fader.pair_fade_down_cut_active[pair->prev_idx] = cut_active;
        s_ui.ch_fader.pair_bottom_hold_active[pair->prev_idx] = bottom_hold_active;
        s_ui.ch_fader.pair_fade_down_bottomed[pair->prev_idx] = bottomed;
        s_ui.ch_fader.pair_top_hold_active[pair->prev_idx] = top_hold_active;
        s_ui.ch_fader.pair_fade_up_topped[pair->prev_idx] = topped;
        s_ui.ch_fader.pair_reverse_fade_down_takeover[pair->prev_idx] = reverse_fade_down_takeover;
        return hold_value;
    }

    // Fade-up is active, so preserve the existing fade-down behavior.
    reverse_fade_down_takeover = false;

    // Capture the value to restore if this fade-down gesture reaches bottom.
    if (!cut_active && fade_down_pressed)
    {
        cut_active    = true;
        restore_value = fade_up_active ? hold_value : 0.0f;
    }

    // Bottom entry is a hard cut. The restore value is kept separately so a
    // retrigger or bottom release can bring the sound back before the next cut.
    if (!bottomed && (fade_down <= down_bottom_threshold))
    {
        bottomed           = true;
        bottom_hold_active = true;
        bottom_entered     = true;
        hold_value         = 0.0f;
    }

    if (bottom_hold_active)
    {
        // Keep output muted while the sensor remains very close to the bottom.
        if (!bottom_entered && (fade_down >= down_bottom_hold_release_threshold))
        {
            bottom_hold_active = false;
            if (fade_up_active && (restore_value > hold_value))
            {
                hold_value = restore_value;
            }
            else if (!fade_up_active)
            {
                hold_value    = 0.0f;
                restore_value = 0.0f;
            }
        }
        else
        {
            hold_value = 0.0f;
        }
    }
    else if (bottomed && (fade_down <= down_bottom_rehold_threshold))
    {
        // If a bottomed gesture dips back into the bottom zone, mute again.
        bottom_hold_active = true;
        hold_value         = 0.0f;
    }

    if (!bottom_hold_active)
    {
        // Fully releasing fade-down arms the next normal cut gesture.
        if (bottomed && (fade_down >= down_bottom_release_threshold))
        {
            bottomed           = false;
            bottom_hold_active = false;
            cut_active         = false;
        }

        if (up_gain > hold_value)
        {
            hold_value = up_gain;
        }
        // Fade-down follows the configured ramp in both directions regardless
        // of which side started the gesture. Only an active bottom-hold forces
        // the output to remain at zero.
        if (down_gain < hold_value)
        {
            hold_value = down_gain;
        }
    }

    if ((gesture_parent == CH_FADER_GESTURE_PARENT_FADE_DOWN) && fade_down_pressed && fade_up_active)
    {
        gesture_child_active = true;
    }

    if ((gesture_parent == CH_FADER_GESTURE_PARENT_FADE_DOWN) && fade_down_pressed && gesture_child_active)
    {
        hold_value = (up_gain > down_gain) ? up_gain : down_gain;
    }

    const bool fade_up_top_hold_enabled = (gesture_parent == CH_FADER_GESTURE_PARENT_FADE_DOWN) &&
                                          fade_down_pressed &&
                                          gesture_child_active &&
                                          fade_up_active;
    if (!fade_up_top_hold_enabled)
    {
        top_hold_active    = false;
        topped             = false;
        top_restore_value  = 0.0f;
    }
    else
    {
        if (!topped && (fade_up >= up_top_threshold))
        {
            topped            = true;
            top_hold_active   = false;
            top_entered       = true;
            top_restore_value = up_gain;
        }

        if (topped && !top_entered && !top_hold_active && (fade_up <= up_top_hold_release_threshold))
        {
            top_hold_active = true;
            hold_value      = 0.0f;
        }

        if (top_hold_active)
        {
            // Once fade-up has backed away from the top, keep it cut until it
            // returns to the top zone.
            if (fade_up >= up_top_rehold_threshold)
            {
                top_hold_active  = false;
                top_restore_value = (up_gain > down_gain) ? up_gain : down_gain;
                hold_value       = top_restore_value;
            }
            else
            {
                hold_value = 0.0f;
            }
        }
    }

    s_ui.ch_fader.pair_hold_value[pair->prev_idx]         = hold_value;
    s_ui.ch_fader.pair_bottom_restore_value[pair->prev_idx] = restore_value;
    s_ui.ch_fader.pair_top_restore_value[pair->prev_idx] = top_restore_value;
    s_ui.ch_fader.pair_gesture_parent[pair->prev_idx] = gesture_parent;
    s_ui.ch_fader.pair_gesture_armed[pair->prev_idx] = gesture_armed;
    s_ui.ch_fader.pair_gesture_child_active[pair->prev_idx] = gesture_child_active;
    s_ui.ch_fader.pair_fade_down_cut_active[pair->prev_idx] = cut_active;
    s_ui.ch_fader.pair_bottom_hold_active[pair->prev_idx] = bottom_hold_active;
    s_ui.ch_fader.pair_fade_down_bottomed[pair->prev_idx] = bottomed;
    s_ui.ch_fader.pair_top_hold_active[pair->prev_idx] = top_hold_active;
    s_ui.ch_fader.pair_fade_up_topped[pair->prev_idx] = topped;
    s_ui.ch_fader.pair_reverse_fade_down_takeover[pair->prev_idx] = reverse_fade_down_takeover;
    return hold_value;
}

// Compute and commit one pair output (A or B) from the current fade-up/fade-down drive values.
static void update_ch_fader_pair_output(const ch_fader_pair_runtime_t* pair)
{
    const float up_now          = get_ch_fader_pair_fade_up_raw(pair);
    const float down_now        = get_ch_fader_pair_fade_down_raw(pair);
    const bool fade_changed     = (fabs(up_now - s_ui.ch_fader.fade_up_prev[pair->prev_idx]) > CH_FADER_SEND_THRESHOLD) || (fabs(down_now - s_ui.ch_fader.fade_down_prev[pair->prev_idx]) > CH_FADER_SEND_THRESHOLD);

    if (fade_changed)
    {
        const float ch_fader_value = apply_ch_fader_pair_reverse(pair, compute_ch_fader_pair_value(pair));
        submit_ch_fader_dsp_output(pair, ch_fader_value);
    }

    s_ui.ch_fader.fade_up_prev[pair->prev_idx]   = up_now;
    s_ui.ch_fader.fade_down_prev[pair->prev_idx] = down_now;
}

// Apply outgoing updates for ch_fader: MIDI CC stream and DSP DC controls.
static void apply_ch_fader_updates(void)
{
    update_ch_fader_fade_down_sources();

    for (uint32_t i = 0; i < MAG_SW_NUM; i++)
    {
        emit_ch_fader_cc_if_needed((uint8_t) i);
    }

    if (!s_ui.ch_fader.fade_prev_valid)
    {
        for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
        {
            s_ui.ch_fader.fade_up_prev[s_ch_fader_pairs[i].prev_idx]   = get_ch_fader_pair_fade_up_raw(&s_ch_fader_pairs[i]);
            s_ui.ch_fader.fade_down_prev[s_ch_fader_pairs[i].prev_idx] = get_ch_fader_pair_fade_down_raw(&s_ch_fader_pairs[i]);
        }
        s_ui.ch_fader.fade_prev_valid = true;
    }
    else
    {
        for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
        {
            update_ch_fader_pair_output(&s_ch_fader_pairs[i]);
        }
    }
}

void ui_control_reapply_ch_fader_outputs(void)
{
    update_ch_fader_fade_down_sources();

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
    {
        const ch_fader_pair_runtime_t* pair = &s_ch_fader_pairs[i];
        const float ch_fader_value = apply_ch_fader_pair_reverse(pair, compute_ch_fader_pair_value(pair));

        sync_ch_fader_dsp_output(pair, ch_fader_value);
        s_ui.ch_fader.fade_up_prev[pair->prev_idx]   = get_ch_fader_pair_fade_up_raw(pair);
        s_ui.ch_fader.fade_down_prev[pair->prev_idx] = get_ch_fader_pair_fade_down_raw(pair);
    }

    s_ui.ch_fader.fade_prev_valid = true;
}

static void process_mag(void)
{
    update_mag_samples();

    if (s_ui.mag_calibration_count > MAG_CALIBRATION_COUNT_MAX)
    {
        update_ch_fader_from_mag();
        apply_ch_fader_updates();
    }
}

static bool uf2_transition_is_armed(UI_Uf2TransitionState_t state)
{
    return (state == UI_UF2_TRANSITION_WAIT_RELEASE) ||
           (state == UI_UF2_TRANSITION_WAIT_HOLD) ||
           (state == UI_UF2_TRANSITION_HOLDING) ||
           (state == UI_UF2_TRANSITION_CLEARING_DISPLAYS);
}

static bool uf2_confirmation_switch_pressed(void)
{
    // The front-panel control documented as SW3 is named SW2 in the MCU GPIO symbols.
    return (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET);
}

static void start_uf2_bootloader_request(void)
{
    if (uf2_transition_is_armed(s_uf2.state))
    {
        SEGGER_RTT_printf(0, "UF2 bootloader request already armed\r\n");
        return;
    }

    const uint32_t now = HAL_GetTick();
    const bool pressed = uf2_confirmation_switch_pressed();

    s_uf2.arm_started_ms  = now;
    s_uf2.displays_cleared = false;
    s_uf2.raw_changed_ms  = now;
    s_uf2.hold_started_ms = 0U;
    s_uf2.notice_started_ms = 0U;
    s_uf2.last_raw_pressed = pressed;
    s_uf2.stable_pressed   = pressed;
    s_uf2.release_observed = false;
    __DMB();
    s_uf2.state            = UI_UF2_TRANSITION_WAIT_RELEASE;

    SEGGER_RTT_printf(0, "UF2 bootloader request armed for %lu ms (PC%u Ch15)\r\n",
                      (unsigned long) UF2_ARM_WINDOW_MS,
                      (unsigned) MIDI_PC_ARM_UF2_BOOTLOADER);
}

static void set_uf2_transition_notice(UI_Uf2TransitionState_t state, const char* reason)
{
    s_uf2.hold_started_ms  = 0U;
    s_uf2.displays_cleared = false;
    s_uf2.notice_started_ms = HAL_GetTick();
    __DMB();
    s_uf2.state            = state;

    SEGGER_RTT_printf(0, "UF2 bootloader request %s\r\n", reason);
}

static void cancel_uf2_bootloader_request(const char* reason)
{
    if (!uf2_transition_is_armed(s_uf2.state))
    {
        return;
    }

    set_uf2_transition_notice(UI_UF2_TRANSITION_CANCELLED, reason);
}

static void service_uf2_bootloader_request(void)
{
    const uint32_t now = HAL_GetTick();
    UI_Uf2TransitionState_t state = s_uf2.state;

    if ((state == UI_UF2_TRANSITION_CANCELLED) || (state == UI_UF2_TRANSITION_TIMED_OUT))
    {
        if ((now - s_uf2.notice_started_ms) >= UF2_NOTICE_MS)
        {
            s_uf2.state = UI_UF2_TRANSITION_IDLE;
            __DMB();
        }
        return;
    }

    if (!uf2_transition_is_armed(state))
    {
        return;
    }

    if (!tud_mounted())
    {
        cancel_uf2_bootloader_request("cancelled because USB was disconnected");
        return;
    }

    if ((now - s_uf2.arm_started_ms) >= UF2_ARM_WINDOW_MS)
    {
        set_uf2_transition_notice(UI_UF2_TRANSITION_TIMED_OUT, "timed out");
        return;
    }

    const bool raw_pressed = uf2_confirmation_switch_pressed();
    if (raw_pressed != s_uf2.last_raw_pressed)
    {
        s_uf2.last_raw_pressed = raw_pressed;
        s_uf2.raw_changed_ms   = now;
    }

    if ((raw_pressed != s_uf2.stable_pressed) &&
        ((now - s_uf2.raw_changed_ms) >= UF2_SWITCH_DEBOUNCE_MS))
    {
        s_uf2.stable_pressed = raw_pressed;

        if (!raw_pressed)
        {
            s_uf2.release_observed = true;
            s_uf2.hold_started_ms  = 0U;
            s_uf2.displays_cleared = false;
            __DMB();
            s_uf2.state            = UI_UF2_TRANSITION_WAIT_HOLD;
            SEGGER_RTT_printf(0, "UF2 confirmation switch released; waiting for hold\r\n");
        }
        else if (s_uf2.release_observed)
        {
            s_uf2.hold_started_ms = now;
            __DMB();
            s_uf2.state           = UI_UF2_TRANSITION_HOLDING;
            SEGGER_RTT_printf(0, "UF2 confirmation switch hold started\r\n");
        }
    }

    state = s_uf2.state;
    if ((state == UI_UF2_TRANSITION_WAIT_RELEASE) &&
        !s_uf2.stable_pressed &&
        ((now - s_uf2.raw_changed_ms) >= UF2_SWITCH_DEBOUNCE_MS))
    {
        s_uf2.release_observed = true;
        __DMB();
        s_uf2.state            = UI_UF2_TRANSITION_WAIT_HOLD;
        SEGGER_RTT_printf(0, "UF2 confirmation switch release observed\r\n");
        return;
    }

    if ((state == UI_UF2_TRANSITION_HOLDING) &&
        s_uf2.stable_pressed &&
        ((now - s_uf2.hold_started_ms) >= UF2_SWITCH_HOLD_MS))
    {
        s_uf2.displays_cleared = false;
        __DMB();
        s_uf2.state = UI_UF2_TRANSITION_CLEARING_DISPLAYS;
        SEGGER_RTT_printf(0, "UF2 confirmation accepted; clearing displays before reset\r\n");
        return;
    }

    if ((state == UI_UF2_TRANSITION_CLEARING_DISPLAYS) &&
        s_uf2.displays_cleared &&
        raw_pressed &&
        s_uf2.stable_pressed)
    {
        __DMB();
        __DSB();
        NVIC_SystemReset();
        while (1)
        {
        }
    }
}

typedef void (*midi_program_handler_t)(uint8_t arg);

typedef struct
{
    uint8_t command;
    midi_program_handler_t handler;
    uint8_t arg;
} midi_program_cmd_t;

static void midi_program_set_input_type(uint8_t arg)
{
    uint8_t input_ch   = (arg >> 4) & 0x0F;
    uint8_t input_type = arg & 0x0F;
    apply_input_type_change(input_ch, input_type);
}

static void midi_program_apply_ch_fader_a(uint8_t input_ch)
{
    apply_ch_fader_assign_a(input_ch);
}

static void midi_program_apply_ch_fader_b(uint8_t input_ch)
{
    apply_ch_fader_assign_b(input_ch);
}

static void midi_program_apply_ch_fader_post(uint8_t input_ch)
{
    apply_ch_fader_assign_post(input_ch);
}

static void midi_program_apply_return(uint8_t input_ch)
{
    apply_return_source(input_ch);
}

static void midi_program_apply_hp_out(uint8_t source)
{
    apply_hp_out_source(source);
}

static void midi_program_apply_input_mode(uint8_t arg)
{
    const uint8_t input_ch = (arg >> 4) & 0x0FU;
    const uint8_t mode     = arg & 0x0FU;

    if (mode <= UI_INPUT_MODE_SYNTH)
    {
        apply_input_mode(input_ch, (UI_InputMode_t) mode);
    }
}

static void midi_program_apply_ch_fader_aux_assignment(uint8_t arg)
{
    const uint8_t sensor_idx = (arg >> 4) & 0x0FU;
    const uint8_t assign     = arg & 0x0FU;

    if (sensor_idx == 2U)
    {
        (void) apply_ch_fader_aux_assignments(assign, s_ui.sensor3_aux_fade_down_assign);
    }
    else if (sensor_idx == 3U)
    {
        (void) apply_ch_fader_aux_assignments(s_ui.sensor2_aux_fade_down_assign, assign);
    }

    SEGGER_RTT_printf(0,
                      "Channel fader aux assignment: sensor2=%c sensor3=%c\r\n",
                      (s_ui.sensor2_aux_fade_down_assign == UI_CH_FADER_AUX_ASSIGN_A) ? 'A' : 'B',
                      (s_ui.sensor3_aux_fade_down_assign == UI_CH_FADER_AUX_ASSIGN_A) ? 'A' : 'B');
}

static void midi_program_apply_timecode_synth_ratio_set(uint8_t arg)
{
    audio_control_set_timecode_synth_ratio_set((TimecodeOscillatorRatioSet_t) arg);
    SEGGER_RTT_printf(0, "SYNTH ratio set: %u\r\n", (unsigned) arg);
}

static void midi_program_apply_timecode_synth_warp_algorithm(uint8_t arg)
{
    audio_control_set_timecode_synth_warp_algorithm((TimecodeOscillatorWarpAlgorithm_t) arg);
    SEGGER_RTT_printf(0, "SYNTH warp algorithm: %u\r\n", (unsigned) arg);
}

static bool dispatch_midi_program_change(uint8_t channel, uint8_t program)
{
    if (channel != MIDI_CH_15)
    {
        return false;
    }

    if (program == MIDI_PC_CURVE_EDIT_MODE_OFF)
    {
        s_ui.curve_edit_mode = false;
        SEGGER_RTT_printf(0, "Curve edit mode OFF (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_CURVE_EDIT_MODE_ON)
    {
        s_ui.curve_edit_mode = true;
        SEGGER_RTT_printf(0, "Curve edit mode ON (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_MUX_OUTPUT_CC)
    {
        s_ui.mag_out_as_note = false;
        SEGGER_RTT_printf(0, "Mag/pot_mag output mode: CC (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_MUX_OUTPUT_NOTE)
    {
        s_ui.mag_out_as_note = true;
        SEGGER_RTT_printf(0, "Mag/pot_mag output mode: Note (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_ARM_UF2_BOOTLOADER)
    {
        start_uf2_bootloader_request();
        return true;
    }

    if (program == MIDI_PC_CANCEL_UF2_BOOTLOADER)
    {
        cancel_uf2_bootloader_request("cancelled by PC125 Ch15");
        return true;
    }

    if ((program == CH_FADER_REVERSE_A_OFF) || (program == CH_FADER_REVERSE_A_ON))
    {
        s_ui.ch_fader_reverse_a = (program == CH_FADER_REVERSE_A_ON);
        ui_control_reapply_ch_fader_outputs();
        SEGGER_RTT_printf(0, "Channel fader A Reverse: %s (PC%u)\r\n", s_ui.ch_fader_reverse_a ? "ON" : "OFF", (unsigned) program);
        return true;
    }

    if ((program == CH_FADER_REVERSE_B_OFF) || (program == CH_FADER_REVERSE_B_ON))
    {
        s_ui.ch_fader_reverse_b = (program == CH_FADER_REVERSE_B_ON);
        ui_control_reapply_ch_fader_outputs();
        SEGGER_RTT_printf(0, "Channel fader B Reverse: %s (PC%u)\r\n", s_ui.ch_fader_reverse_b ? "ON" : "OFF", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_REQUEST_EEPROM_DUMP)
    {
        EEPROM_DeviceConfig_t cfg;

        EEPROM_ConfigCaptureCurrent(&cfg);
        send_midi_config_dump(&cfg);
        SEGGER_RTT_printf(0, "Current config dumped by MIDI PC126: CH1=%u CH2=%u CH_FADER_A=%u CH_FADER_B=%u CH_FADER_POST=%u RTN=%u HP=%u MODE1=%u MODE2=%u MAG_AS_NOTE=%u AUX2=%u AUX3=%u REVERSE_A=%u REVERSE_B=%u CURVE_WIDTH_A=%.4f CURVE_WIDTH_B=%.4f\r\n", (unsigned) cfg.current_ch1_input_type, (unsigned) cfg.current_ch2_input_type, (unsigned) cfg.current_ch_fader_a_assign, (unsigned) cfg.current_ch_fader_b_assign, (unsigned) cfg.current_ch_fader_post_assign, (unsigned) cfg.current_return_assign, (unsigned) cfg.current_hp_out_source, (unsigned) cfg.current_ch1_input_mode, (unsigned) cfg.current_ch2_input_mode, (unsigned) ((cfg.mag_output_mode_flags & EEPROM_CFG_FLAG_MAG_OUT_AS_NOTE) != 0U), (unsigned) cfg.sensor2_aux_fade_down_assign, (unsigned) cfg.sensor3_aux_fade_down_assign, (unsigned) ((cfg.ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_A) != 0U), (unsigned) ((cfg.ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_B) != 0U), (double) cfg.current_ch_fader_curve_width_a, (double) cfg.current_ch_fader_curve_width_b);

        return true;
    }

    if (program == MIDI_PC_SAVE_EEPROM)
    {
        EEPROM_DeviceConfig_t cfg;

        EEPROM_ConfigCaptureCurrent(&cfg);
        if (EEPROM_SaveConfig(&hi2c2, &cfg) == HAL_OK)
        {
            led_notify_save_success();
            SEGGER_RTT_printf(0, "EEPROM config saved by MIDI PC127: CH1=%u CH2=%u CH_FADER_A=%u CH_FADER_B=%u CH_FADER_POST=%u RTN=%u HP=%u MODE1=%u MODE2=%u AUX2=%u AUX3=%u REVERSE_A=%u REVERSE_B=%u CURVE_WIDTH_A=%.4f CURVE_WIDTH_B=%.4f\r\n", (unsigned) cfg.current_ch1_input_type, (unsigned) cfg.current_ch2_input_type, (unsigned) cfg.current_ch_fader_a_assign, (unsigned) cfg.current_ch_fader_b_assign, (unsigned) cfg.current_ch_fader_post_assign, (unsigned) cfg.current_return_assign, (unsigned) cfg.current_hp_out_source, (unsigned) cfg.current_ch1_input_mode, (unsigned) cfg.current_ch2_input_mode, (unsigned) cfg.sensor2_aux_fade_down_assign, (unsigned) cfg.sensor3_aux_fade_down_assign, (unsigned) ((cfg.ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_A) != 0U), (unsigned) ((cfg.ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_B) != 0U), (double) cfg.current_ch_fader_curve_width_a, (double) cfg.current_ch_fader_curve_width_b);
        }
        else
        {
            SEGGER_RTT_printf(0, "EEPROM config save failed by MIDI PC127\r\n");
        }
        return true;
    }

    static const midi_program_cmd_t commands[] = {
        {CH1_LINE,             midi_program_set_input_type, (uint8_t) ((INPUT_CH1 << 4) | INPUT_TYPE_LINE) },
        {CH1_PHONO,            midi_program_set_input_type, (uint8_t) ((INPUT_CH1 << 4) | INPUT_TYPE_PHONO)},
        {CH2_LINE,             midi_program_set_input_type, (uint8_t) ((INPUT_CH2 << 4) | INPUT_TYPE_LINE) },
        {CH2_PHONO,            midi_program_set_input_type, (uint8_t) ((INPUT_CH2 << 4) | INPUT_TYPE_PHONO)},
        {CH_FADER_ASSIGN_A_CH1,      midi_program_apply_ch_fader_a,     INPUT_CH1                                      },
        {CH_FADER_ASSIGN_A_CH2,      midi_program_apply_ch_fader_a,     INPUT_CH2                                      },
        {CH_FADER_ASSIGN_A_USB12,    midi_program_apply_ch_fader_a,     INPUT_USB12                                    },
        {CH_FADER_ASSIGN_A_USB34,    midi_program_apply_ch_fader_a,     INPUT_USB34                                    },
        {CH_FADER_ASSIGN_B_CH1,      midi_program_apply_ch_fader_b,     INPUT_CH1                                      },
        {CH_FADER_ASSIGN_B_CH2,      midi_program_apply_ch_fader_b,     INPUT_CH2                                      },
        {CH_FADER_ASSIGN_B_USB12,    midi_program_apply_ch_fader_b,     INPUT_USB12                                    },
        {CH_FADER_ASSIGN_B_USB34,    midi_program_apply_ch_fader_b,     INPUT_USB34                                    },
        {CH_FADER_ASSIGN_POST_CH1,   midi_program_apply_ch_fader_post,  INPUT_CH1                                      },
        {CH_FADER_ASSIGN_POST_CH2,   midi_program_apply_ch_fader_post,  INPUT_CH2                                      },
        {CH_FADER_ASSIGN_POST_USB12, midi_program_apply_ch_fader_post,  INPUT_USB12                                    },
        {CH_FADER_ASSIGN_POST_USB34, midi_program_apply_ch_fader_post,  INPUT_USB34                                    },
        {CH1_MODE_DISABLE,     midi_program_apply_input_mode, (uint8_t) ((INPUT_CH1 << 4) | UI_INPUT_MODE_DISABLED)},
        {CH1_MODE_DVS,         midi_program_apply_input_mode, (uint8_t) ((INPUT_CH1 << 4) | UI_INPUT_MODE_DVS)     },
        {CH1_MODE_SYNTH,       midi_program_apply_input_mode, (uint8_t) ((INPUT_CH1 << 4) | UI_INPUT_MODE_SYNTH)   },
        {CH2_MODE_DISABLE,     midi_program_apply_input_mode, (uint8_t) ((INPUT_CH2 << 4) | UI_INPUT_MODE_DISABLED)},
        {CH2_MODE_DVS,         midi_program_apply_input_mode, (uint8_t) ((INPUT_CH2 << 4) | UI_INPUT_MODE_DVS)     },
        {CH2_MODE_SYNTH,       midi_program_apply_input_mode, (uint8_t) ((INPUT_CH2 << 4) | UI_INPUT_MODE_SYNTH)   },
        {RETURN_CH_USB12,      midi_program_apply_return,   INPUT_USB12                                    },
        {RETURN_CH_USB34,      midi_program_apply_return,   INPUT_USB34                                    },
        {RETURN_CH_NONE,       midi_program_apply_return,   INPUT_SRC_NONE                                 },
        {HP_OUT_CH_FADER_A,          midi_program_apply_hp_out,   CUE_SEL_CH_FADER_A                                   },
        {HP_OUT_CH_FADER_B,          midi_program_apply_hp_out,   CUE_SEL_CH_FADER_B                                   },
        {HP_OUT_THRU,          midi_program_apply_hp_out,   CUE_SEL_THRU                                   },
        {HP_OUT_MASTER,        midi_program_apply_hp_out,   CUE_SEL_MST                                    },
        {CH_FADER_AUX_SENSOR2_TO_A,  midi_program_apply_ch_fader_aux_assignment, (uint8_t) ((2U << 4) | UI_CH_FADER_AUX_ASSIGN_A)},
        {CH_FADER_AUX_SENSOR2_TO_B,  midi_program_apply_ch_fader_aux_assignment, (uint8_t) ((2U << 4) | UI_CH_FADER_AUX_ASSIGN_B)},
        {CH_FADER_AUX_SENSOR3_TO_A,  midi_program_apply_ch_fader_aux_assignment, (uint8_t) ((3U << 4) | UI_CH_FADER_AUX_ASSIGN_A)},
        {CH_FADER_AUX_SENSOR3_TO_B,  midi_program_apply_ch_fader_aux_assignment, (uint8_t) ((3U << 4) | UI_CH_FADER_AUX_ASSIGN_B)},
        {SYNTH_RATIO_OCTAVE,         midi_program_apply_timecode_synth_ratio_set, TIMECODE_RATIO_OCTAVE                  },
        {SYNTH_RATIO_HARMONIC,       midi_program_apply_timecode_synth_ratio_set, TIMECODE_RATIO_HARMONIC                },
        {SYNTH_RATIO_CHORD,          midi_program_apply_timecode_synth_ratio_set, TIMECODE_RATIO_CHORD                   },
        {SYNTH_WARP_CLEAN,           midi_program_apply_timecode_synth_warp_algorithm, TIMECODE_WARP_CLEAN                },
        {SYNTH_WARP_CROSSFOLD,       midi_program_apply_timecode_synth_warp_algorithm, TIMECODE_WARP_CROSSFOLD            },
        {SYNTH_WARP_RING_MOD,        midi_program_apply_timecode_synth_warp_algorithm, TIMECODE_WARP_RING_MOD             },
        {SYNTH_WARP_COMPARATOR,      midi_program_apply_timecode_synth_warp_algorithm, TIMECODE_WARP_COMPARATOR           },
    };

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(commands); i++)
    {
        if (commands[i].command == program)
        {
            commands[i].handler(commands[i].arg);
            return true;
        }
    }
    return false;
}

static bool dispatch_midi_control_change(uint8_t channel, uint8_t number, uint8_t value)
{
    float new_ch_fader_curve_width;

    if (!s_ui.curve_edit_mode)
    {
        return false;
    }

    if (channel != MIDI_CH_15)
    {
        return false;
    }

    if (number == MIDI_CC_CH_FADER_CURVE_A)
    {
        new_ch_fader_curve_width = clamp_ch_fader_curve_width(midi_cc_to_ch_fader_curve_width(value));
        if (fabsf(new_ch_fader_curve_width - s_ui.ch_fader_curve_width_a) > 0.0001f)
        {
            s_ui.ch_fader_curve_width_a = new_ch_fader_curve_width;
            mark_ch_fader_curve_dirty();
            SEGGER_RTT_printf(0, "Curve A updated by CC%u Ch15 -> width %.4f\r\n", (unsigned) number, (double) s_ui.ch_fader_curve_width_a);
        }
        return true;
    }

    if (number == MIDI_CC_CH_FADER_CURVE_B)
    {
        new_ch_fader_curve_width = clamp_ch_fader_curve_width(midi_cc_to_ch_fader_curve_width(value));
        if (fabsf(new_ch_fader_curve_width - s_ui.ch_fader_curve_width_b) > 0.0001f)
        {
            s_ui.ch_fader_curve_width_b = new_ch_fader_curve_width;
            mark_ch_fader_curve_dirty();
            SEGGER_RTT_printf(0, "Curve B updated by CC%u Ch15 -> width %.4f\r\n", (unsigned) number, (double) s_ui.ch_fader_curve_width_b);
        }
        return true;
    }

    if (number == MIDI_CC_CH_FADER_DVS_DELAY)
    {
        apply_ch_fader_dvs_delay(value);
        SEGGER_RTT_printf(0,
                          "DVS fader delay updated by CC%u Ch15 -> %u ms\r\n",
                          (unsigned) number,
                          (unsigned) s_ui.ch_fader_dvs_delay_ms);
        return true;
    }

    return false;
}

static void process_midi_rx(void)
{
    while (tud_midi_available())
    {
        uint8_t packet[4];
        tud_midi_packet_read(packet);

        const uint8_t status  = (uint8_t) (packet[1] & 0xF0U);
        const uint8_t channel = (uint8_t) (packet[1] & 0x0FU);

        if (status == 0xC0U)
        {
            (void) dispatch_midi_program_change(channel, packet[2]);
        }
        else if (status == 0xB0U)
        {
            (void) dispatch_midi_control_change(channel, packet[2], packet[3]);
        }

        SEGGER_RTT_printf(0, "MIDI RX: 0x%02X 0x%02X 0x%02X(%d) 0x%02X(%d)\n", packet[0], packet[1], packet[2], packet[2], packet[3], packet[3]);
    }
}

void ui_control_task(void)
{
#if !ENABLE_DSP_RUNTIME_CONTROL
    return;
#endif

    if (!is_started_audio_control())
    {
        return;
    }

    if (is_adc_complete)
    {
        process_pot();
        process_mag();
        process_midi_rx();
        is_adc_complete = false;
    }

    service_uf2_bootloader_request();

    // Keep due writes running even when no new ADC sample has arrived. Handle
    // MIDI routing changes first so queued values cannot replay on a new input.
    service_ch_fader_dsp_outputs();
}

void start_audio_control(void)
{
    s_ui.is_start_audio_control = true;
    __DMB();
}

bool is_started_audio_control(void)
{
    return s_ui.is_start_audio_control;
}

void ui_control_get_persist_state(UI_ControlPersistState_t* state)
{
    if (state == NULL)
    {
        return;
    }

    state->current_ch1_input_type = s_ui.current_ch1_input_type;
    state->current_ch2_input_type = s_ui.current_ch2_input_type;
    state->current_ch_fader_a_assign     = s_ui.current_ch_fader_a_assign;
    state->current_ch_fader_b_assign     = s_ui.current_ch_fader_b_assign;
    state->current_ch_fader_post_assign  = s_ui.current_ch_fader_post_assign;
    state->current_return_assign  = s_ui.current_return_assign;
    state->current_hp_out_source  = s_ui.current_hp_out_source;
    state->current_ch1_input_mode    = s_ui.current_ch1_input_mode;
    state->current_ch2_input_mode    = s_ui.current_ch2_input_mode;
    state->ch_fader_dvs_delay_ms     = s_ui.ch_fader_dvs_delay_ms;
    state->sensor2_aux_fade_down_assign = s_ui.sensor2_aux_fade_down_assign;
    state->sensor3_aux_fade_down_assign = s_ui.sensor3_aux_fade_down_assign;
    state->current_ch_fader_curve_width_a = s_ui.ch_fader_curve_width_a;
    state->current_ch_fader_curve_width_b = s_ui.ch_fader_curve_width_b;
    state->ch_fader_reverse_a        = s_ui.ch_fader_reverse_a;
    state->ch_fader_reverse_b        = s_ui.ch_fader_reverse_b;
    state->mag_out_as_note           = s_ui.mag_out_as_note;
}

bool ui_control_apply_persist_state(const UI_ControlPersistState_t* state)
{
    uint8_t input_ch_a;
    uint8_t input_ch_b;
    uint8_t input_ch_post;
    uint8_t input_ch_return;

    if (state == NULL)
    {
        return false;
    }

    if ((state->current_ch1_input_type > INPUT_TYPE_PHONO) ||
        (state->current_ch2_input_type > INPUT_TYPE_PHONO) ||
        (state->current_hp_out_source > CUE_SEL_MST) ||
        (state->current_ch1_input_mode > UI_INPUT_MODE_SYNTH) ||
        (state->current_ch2_input_mode > UI_INPUT_MODE_SYNTH) ||
        (state->sensor2_aux_fade_down_assign > UI_CH_FADER_AUX_ASSIGN_B) ||
        (state->sensor3_aux_fade_down_assign > UI_CH_FADER_AUX_ASSIGN_B))
    {
        return false;
    }

    if (!assign_to_input_ch(state->current_ch_fader_a_assign, &input_ch_a) ||
        !assign_to_input_ch(state->current_ch_fader_b_assign, &input_ch_b) ||
        !assign_to_input_ch(state->current_ch_fader_post_assign, &input_ch_post) ||
        !assign_to_return_input_ch(state->current_return_assign, &input_ch_return))
    {
        return false;
    }

    apply_input_type_change(INPUT_CH1, state->current_ch1_input_type);
    apply_input_type_change(INPUT_CH2, state->current_ch2_input_type);
    apply_ch_fader_assign_a(input_ch_a);
    apply_ch_fader_assign_b(input_ch_b);
    apply_ch_fader_assign_post(input_ch_post);
    apply_return_source(input_ch_return);
    apply_hp_out_source(state->current_hp_out_source);
    apply_input_mode(INPUT_CH1, (UI_InputMode_t) state->current_ch1_input_mode);
    apply_input_mode(INPUT_CH2, (UI_InputMode_t) state->current_ch2_input_mode);
    apply_ch_fader_dvs_delay(state->ch_fader_dvs_delay_ms);
    (void) apply_ch_fader_aux_assignments(state->sensor2_aux_fade_down_assign,
                                       state->sensor3_aux_fade_down_assign);
    s_ui.ch_fader_curve_width_a = clamp_ch_fader_curve_width(state->current_ch_fader_curve_width_a);
    s_ui.ch_fader_curve_width_b = clamp_ch_fader_curve_width(state->current_ch_fader_curve_width_b);
    s_ui.ch_fader_reverse_a = state->ch_fader_reverse_a;
    s_ui.ch_fader_reverse_b = state->ch_fader_reverse_b;
    s_ui.mag_out_as_note    = state->mag_out_as_note;
    mark_ch_fader_curve_dirty();

    return true;
}

void ui_control_reset_state(void)
{
    memset(s_ch_fader_dsp, 0, sizeof(s_ch_fader_dsp));
    memset(&s_uf2, 0, sizeof(s_uf2));
    s_uf2.state = UI_UF2_TRANSITION_IDLE;

    for (uint16_t i = 0; i < ADC_NUM; i++)
    {
        adc_val[i] = 0;
    }

    for (uint16_t i = 0; i < POT_NUM; i++)
    {
        s_ui.pot_ma_index[i]    = 0;
        s_ui.pot_sample_count[i] = 0U;
        s_ui.pot_val[i]         = 0;
        s_ui.pot_val_prev[i][0] = 0;
        s_ui.pot_val_prev[i][1] = 0;
        for (uint16_t j = 0; j < POT_MA_SIZE; j++)
        {
            s_ui.pot_val_ma[i][j] = 0;
        }
    }
    for (uint16_t i = 0; i < POT_HYSTERESIS_NUM; i++)
    {
        s_ui.pot_hysteresis_last_sent[i]     = 0U;
        s_ui.pot_hysteresis_has_last_sent[i] = false;
        for (uint16_t j = 0; j < POT_MA_SIZE; j++)
        {
            s_ui.pot_hysteresis_raw_ma[i][j] = 0U;
        }
    }

    s_ui.mag_calibration_count = 0;
    for (uint16_t i = 0; i < MAG_SW_NUM; i++)
    {
        s_ui.mag_val[i]        = 0;
        s_ui.mag_offset_sum[i] = 0;
        s_ui.mag_offset[i]     = 0;
    }
    for (uint16_t i = 0; i < 4U; i++)
    {
        s_ui.pot_mag_calibration_count[i] = 0;
        s_ui.pot_mag_offset_sum[i]        = 0;
        s_ui.pot_mag_offset[i]            = 0;
        s_ui.pot_mag_candidate[i]         = 0U;
        s_ui.pot_mag_stable_count[i]      = 0U;
        s_ui.pot_mag_state[i]             = 0U;
    }

    s_ui.current_ch1_input_type = INPUT_TYPE_LINE;
    s_ui.current_ch2_input_type = INPUT_TYPE_LINE;
    s_ui.current_ch_fader_a_assign     = INPUT_SRC_CH1_LN;
    s_ui.current_ch_fader_b_assign     = INPUT_SRC_CH2_LN;
    s_ui.current_ch_fader_post_assign  = INPUT_SRC_USB12;
    s_ui.current_return_assign  = INPUT_SRC_USB34;
    s_ui.current_hp_out_source  = CUE_SEL_MST;
    s_ui.current_ch1_input_mode = UI_INPUT_MODE_DISABLED;
    s_ui.current_ch2_input_mode = UI_INPUT_MODE_DISABLED;
    s_ui.ch_fader_dvs_delay_ms  = UI_CH_FADER_DVS_DELAY_DEFAULT_MS;
    s_ui.sensor2_aux_fade_down_assign = UI_CH_FADER_AUX_ASSIGN_A;
    s_ui.sensor3_aux_fade_down_assign = UI_CH_FADER_AUX_ASSIGN_B;
    s_ui.ch_fader_curve_width_a    = UI_CH_FADER_CURVE_WIDTH_A_DEFAULT;
    s_ui.ch_fader_curve_width_b    = UI_CH_FADER_CURVE_WIDTH_B_DEFAULT;
    s_ui.ch_fader_reverse_a     = false;
    s_ui.ch_fader_reverse_b     = false;
    s_ui.curve_edit_mode        = false;
    s_ui.ch_fader.position_a          = 0;
    s_ui.ch_fader.position_b          = 0;
    (void) apply_ch_fader_aux_assignments(s_ui.sensor2_aux_fade_down_assign,
                                       s_ui.sensor3_aux_fade_down_assign);

    for (uint16_t i = 0; i < MAG_SW_NUM; i++)
    {
        s_ui.ch_fader.raw[i]                = 1.0f;
        s_ui.ch_fader.prev[i]               = 1.0f;
        s_ui.ch_fader.down_floor[i]         = 1.0f;
        s_ui.ch_fader.up_peak[i]            = 0.0f;
        s_ui.ch_fader.note_peak_vel[i]      = 0U;
        s_ui.ch_fader.note_scan_start_ms[i] = 0U;
        s_ui.ch_fader.note_is_on[i]         = false;
        s_ui.ch_fader.note_scan_active[i]   = false;
    }
    for (uint8_t i = 0; i < CH_FADER_PAIR_COUNT; i++)
    {
        s_ui.ch_fader.fade_up_prev[i]           = 0.0f;
        s_ui.ch_fader.fade_down_prev[i]         = 1.0f;
        s_ui.ch_fader.fade_down_combined_raw[i] = 1.0f;
        for (uint8_t source = 0; source < CH_FADER_FADE_DOWN_SOURCE_COUNT; source++)
        {
            s_ui.ch_fader.fade_down_source_prev[i][source] = 1.0f;
        }
        s_ui.ch_fader.fade_down_active_source[i] = CH_FADER_FADE_DOWN_SOURCE_NONE;
        s_ui.ch_fader.fade_down_force_release_reads[i] = 0U;
        s_ui.ch_fader.pair_gesture_parent[i] = CH_FADER_GESTURE_PARENT_NONE;
        s_ui.ch_fader.pair_gesture_armed[i] = true;
        s_ui.ch_fader.pair_gesture_child_active[i] = false;
        s_ui.ch_fader.pair_hold_value[i]         = 0.0f;
        s_ui.ch_fader.pair_bottom_restore_value[i] = 0.0f;
        s_ui.ch_fader.pair_top_restore_value[i] = 0.0f;
        s_ui.ch_fader.pair_fade_down_cut_active[i] = false;
        s_ui.ch_fader.pair_bottom_hold_active[i] = false;
        s_ui.ch_fader.pair_fade_down_bottomed[i] = false;
        s_ui.ch_fader.pair_top_hold_active[i] = false;
        s_ui.ch_fader.pair_fade_up_topped[i] = false;
        s_ui.ch_fader.pair_reverse_fade_down_takeover[i] = false;
    }
    mark_ch_fader_curve_dirty();
    s_ui.ch_fader.fade_prev_valid = false;

    for (uint16_t i = 0; i < 128U; i++)
    {
        s_note_peak_vel[i]      = 0U;
        s_note_scan_start_ms[i] = 0U;
        s_note_is_on[i]         = false;
        s_note_scan_active[i]   = false;
    }
    s_ui.pot_ch         = POT_CH_CC0;
    s_ui.pot_ch_counter = 0;
    is_adc_complete     = false;
}
