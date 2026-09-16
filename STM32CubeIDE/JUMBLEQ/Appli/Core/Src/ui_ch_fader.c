/*
 * ui_ch_fader.c
 *
 * Magnetic-switch channel fader processing: calibration, normalized sensor
 * values, gesture arbitration, curve/Reverse handling, aux fade-down
 * assignment, DVS delay queue and the DSP/MIDI output application.
 */

#include "ui_control.h"
#include "ui_ch_fader_internal.h"

#include "ui_midi_control_internal.h"
#include "ui_routing_control_internal.h"
#include "adau1466.h"

#include "stm32h7rsxx_hal.h"
#include "tusb.h"

#include <math.h>
#include <string.h>

#define MAG_SW_NUM                6
#define MAG_CALIBRATION_COUNT_MAX 100
#define MAG_CH_FADER_CUTOFF          16
#define MAG_CH_FADER_RANGE           1400
#define CH_FADER_FADE_DOWN_SOURCE_COUNT 3U
// At most one update per pair per ADC task iteration (2 ms minimum).
#define CH_FADER_DSP_QUEUE_CAPACITY  512U

typedef enum
{
    CH_FADER_GESTURE_PARENT_NONE = 0,
    CH_FADER_GESTURE_PARENT_FADE_UP,
    CH_FADER_GESTURE_PARENT_FADE_DOWN,
} ch_fader_gesture_parent_t;

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

static ch_fader_state_t s_ch_fader = {
    .position_a          = 0,
    .position_b          = 0,
    .raw                 = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .prev                = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .down_floor          = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .up_peak             = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .pair_hold_value          = {0.0f, 0.0f},
    .pair_bottom_restore_value = {0.0f, 0.0f},
    .pair_top_restore_value    = {0.0f, 0.0f},
    .fade_up_prev             = {0.0f, 0.0f},
    .fade_down_prev           = {1.0f, 1.0f},
    .fade_down_combined_raw   = {1.0f, 1.0f},
    .fade_down_source_prev    = {{1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},
    .fade_down_active_source  = {0xFFU, 0xFFU},
    .fade_down_force_release_reads = {0U, 0U},
    .pair_gesture_parent      = {CH_FADER_GESTURE_PARENT_NONE, CH_FADER_GESTURE_PARENT_NONE},
    .pair_gesture_armed       = {true, true},
    .pair_gesture_child_active = {false, false},
    .pair_fade_down_cut_active = {false, false},
    .pair_bottom_hold_active  = {false, false},
    .pair_fade_down_bottomed  = {false, false},
    .pair_top_hold_active     = {false, false},
    .pair_fade_up_topped      = {false, false},
    .pair_reverse_fade_down_takeover = {false, false},
    .fade_prev_valid          = false,
};

static uint16_t s_mag_calibration_count;
static uint16_t s_mag_val[MAG_SW_NUM];
static uint32_t s_mag_offset_sum[MAG_SW_NUM];
static uint16_t s_mag_offset[MAG_SW_NUM];
static float s_ch_fader_curve_width_a = UI_CH_FADER_CURVE_WIDTH_A_DEFAULT;
static float s_ch_fader_curve_width_b = UI_CH_FADER_CURVE_WIDTH_B_DEFAULT;
static bool s_ch_fader_reverse_a = false;
static bool s_ch_fader_reverse_b = false;
static uint8_t s_ch_fader_dvs_delay_ms = UI_CH_FADER_DVS_DELAY_DEFAULT_MS;
static uint8_t s_sensor2_aux_fade_down_assign = UI_CH_FADER_AUX_ASSIGN_A;
static uint8_t s_sensor3_aux_fade_down_assign = UI_CH_FADER_AUX_ASSIGN_B;

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
static const uint8_t CH_FADER_FADE_DOWN_SOURCE_NONE    = 0xFFU;
static const uint8_t CH_FADER_FADE_DOWN_RETRIGGER_RELEASE_READS_WITH_FADE_UP = 16U;
static const uint8_t CH_FADER_FADE_DOWN_RETRIGGER_RELEASE_READS_MOMENTARY    = 16U;

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
     .current_position  = &s_ch_fader.position_a,
     .set_dc            = set_dc_inputA,
     },
    {
     .fade_up_idx       = 5,
     .fade_down_idx     = 4,
     .aux_fade_down_idx = 3,
     .aux2_fade_down_idx = MAG_SW_NUM,
     .prev_idx          = CH_FADER_PAIR_B,
     .current_position  = &s_ch_fader.position_b,
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
    return ui_routing_get_ch_fader_assign((pair->prev_idx == CH_FADER_PAIR_A) ? 0U : 1U);
}

static bool ch_fader_assign_uses_dvs(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_USB12:  // PC return for Ch. 1 DVS, including direct USB routing.
        return ui_routing_get_input_mode(INPUT_CH1) == UI_INPUT_MODE_DVS;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
    case INPUT_SRC_USB34:  // PC return for Ch. 2 DVS, including direct USB routing.
        return ui_routing_get_input_mode(INPUT_CH2) == UI_INPUT_MODE_DVS;
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

    if (!state->delay_enabled || (s_ch_fader_dvs_delay_ms == 0U) ||
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
            if ((uint32_t) (now - update->captured_ms) < s_ch_fader_dvs_delay_ms)
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
        s_ch_fader.fade_down_prev[pair_idx] = -1.0f;
        s_ch_fader.fade_down_combined_raw[pair_idx] = 1.0f;
        s_ch_fader.fade_down_active_source[pair_idx] = CH_FADER_FADE_DOWN_SOURCE_NONE;
        s_ch_fader.fade_down_force_release_reads[pair_idx] = 0U;
        s_ch_fader.pair_gesture_parent[pair_idx] = CH_FADER_GESTURE_PARENT_NONE;
        s_ch_fader.pair_gesture_armed[pair_idx] = true;
        s_ch_fader.pair_gesture_child_active[pair_idx] = false;
        s_ch_fader.pair_fade_down_cut_active[pair_idx] = false;
        s_ch_fader.pair_bottom_hold_active[pair_idx] = false;
        s_ch_fader.pair_fade_down_bottomed[pair_idx] = false;
        s_ch_fader.pair_reverse_fade_down_takeover[pair_idx] = false;

        for (uint8_t source = 0U; source < CH_FADER_FADE_DOWN_SOURCE_COUNT; source++)
        {
            s_ch_fader.fade_down_source_prev[pair_idx][source] = 1.0f;
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

    s_sensor2_aux_fade_down_assign = sensor2_assign;
    s_sensor3_aux_fade_down_assign = sensor3_assign;

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

static void clear_ch_fader_note_state(uint8_t i)
{
    s_ch_fader.note_peak_vel[i]      = 0U;
    s_ch_fader.note_scan_start_ms[i] = 0U;
    s_ch_fader.note_is_on[i]         = false;
    s_ch_fader.note_scan_active[i]   = false;
}

static void emit_ch_fader_note_if_needed(uint8_t i, uint8_t note, uint8_t value)
{
    const uint32_t now_ms = HAL_GetTick();

    if (s_ch_fader.note_is_on[i])
    {
        if (value <= MIDI_NOTE_OFF_THRESHOLD)
        {
            ui_midi_control_send_note(note, 0U, 0U);
            clear_ch_fader_note_state(i);
        }
        return;
    }

    if (!s_ch_fader.note_scan_active[i])
    {
        if (value >= MIDI_NOTE_ON_THRESHOLD)
        {
            s_ch_fader.note_scan_active[i]   = true;
            s_ch_fader.note_scan_start_ms[i] = now_ms;
            s_ch_fader.note_peak_vel[i]      = value;
        }
        return;
    }

    if (value > s_ch_fader.note_peak_vel[i])
    {
        s_ch_fader.note_peak_vel[i] = value;
    }

    if (value <= MIDI_NOTE_OFF_THRESHOLD)
    {
        clear_ch_fader_note_state(i);
        return;
    }

    if ((now_ms - s_ch_fader.note_scan_start_ms[i]) >= MIDI_NOTE_VEL_WINDOW_MS)
    {
        uint8_t velocity = s_ch_fader.note_peak_vel[i];
        if (velocity == 0U)
        {
            velocity = 1U;
        }

        float n  = (float) velocity / 127.0f;
        n        = powf(n, MIDI_NOTE_VEL_GAMMA);
        velocity = (uint8_t) (1.0f + n * 126.0f);

        ui_midi_control_send_note(note, velocity, 0U);
        s_ch_fader.note_is_on[i]       = true;
        s_ch_fader.note_scan_active[i] = false;
    }
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
        s_ch_fader.fade_up_prev[i]   = -1.0f;
        s_ch_fader.fade_down_prev[i] = -1.0f;
    }
}

static float midi_cc_to_ch_fader_curve_width(uint8_t value)
{
    const float t = (float) value / 127.0f;

    return CH_FADER_PAIR_CURVE_WIDTH_MAX + ((CH_FADER_PAIR_CURVE_WIDTH_MIN - CH_FADER_PAIR_CURVE_WIDTH_MAX) * t);
}

uint8_t ui_ch_fader_curve_width_to_midi_cc(float width)
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
    return s_ch_fader.position_a;
}

uint8_t get_current_ch_fader_b_position(void)
{
    return s_ch_fader.position_b;
}

uint8_t ui_control_get_ch_fader_dvs_delay_ms(void)
{
    return s_ch_fader_dvs_delay_ms;
}

bool ui_control_is_ch_fader_reverse_a_enabled(void)
{
    return s_ch_fader_reverse_a;
}

bool ui_control_is_ch_fader_reverse_b_enabled(void)
{
    return s_ch_fader_reverse_b;
}

static void apply_ch_fader_dvs_delay(uint8_t delay_ms)
{
    const uint8_t clamped_delay_ms = (delay_ms > UI_CH_FADER_DVS_DELAY_MAX_MS)
                                         ? UI_CH_FADER_DVS_DELAY_MAX_MS
                                         : delay_ms;

    if (s_ch_fader_dvs_delay_ms == clamped_delay_ms)
    {
        return;
    }

    s_ch_fader_dvs_delay_ms = clamped_delay_ms;
    // A new delay must not reinterpret updates captured with the old setting.
    ui_ch_fader_reapply_outputs();
}

static void update_mag_samples(const uint32_t* adc_samples)
{
    for (int i = 0; i < MAG_SW_NUM; i++)
    {
        s_mag_val[i] = (uint16_t) adc_samples[i];

        if (s_mag_calibration_count < MAG_CALIBRATION_COUNT_MAX)
        {
            s_mag_offset_sum[i] += adc_samples[i];
        }
        else if (s_mag_calibration_count == MAG_CALIBRATION_COUNT_MAX)
        {
            s_mag_offset[i] = s_mag_offset_sum[i] / MAG_CALIBRATION_COUNT_MAX;
        }
    }
    if (s_mag_calibration_count <= MAG_CALIBRATION_COUNT_MAX)
    {
        s_mag_calibration_count++;
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
        if (s_mag_val[i] < s_mag_offset[i] + MAG_CH_FADER_CUTOFF)
        {
            s_ch_fader.raw[i] = 0.0f;
        }
        else if (s_mag_val[i] >= s_mag_offset[i] + MAG_CH_FADER_CUTOFF && s_mag_val[i] <= s_mag_offset[i] + MAG_CH_FADER_RANGE)
        {
            s_ch_fader.raw[i] = (float) (s_mag_val[i] - s_mag_offset[i] - MAG_CH_FADER_CUTOFF) / (float) MAG_CH_FADER_RANGE;
        }
        else if (s_mag_val[i] > s_mag_offset[i] + MAG_CH_FADER_RANGE)
        {
            s_ch_fader.raw[i] = 1.0f;
        }
    }
    else
    {
        if (s_mag_val[i] < s_mag_offset[i] + MAG_CH_FADER_CUTOFF)
        {
            s_ch_fader.raw[i] = 1.0f;
        }
        else if (s_mag_val[i] >= s_mag_offset[i] + MAG_CH_FADER_CUTOFF && s_mag_val[i] <= s_mag_offset[i] + MAG_CH_FADER_RANGE)
        {
            s_ch_fader.raw[i] = 1.0f - ((float) (s_mag_val[i] - s_mag_offset[i] - MAG_CH_FADER_CUTOFF) / (float) MAG_CH_FADER_RANGE);
        }
        else if (s_mag_val[i] > s_mag_offset[i] + MAG_CH_FADER_RANGE)
        {
            s_ch_fader.raw[i] = 0.0f;
        }
    }
}

// Update peak/valley trackers used to build stable pair outputs.
static void update_ch_fader_extrema(uint8_t i)
{
    const float pair_raw = apply_ch_fader_pair_onset_deadband(i, s_ch_fader.raw[i]);

    if (get_pair_fade_down_index_from_up(i) >= 0)
    {
        // Track fade-up peak; paired fade-down floor is synchronized from this edge.
        if (pair_raw > (s_ch_fader.up_peak[i] + CH_FADER_EXTREMA_HYSTERESIS))
        {
            s_ch_fader.up_peak[i] = pair_raw;

            const int8_t fade_down_idx = get_pair_fade_down_index_from_up(i);
            if (fade_down_idx >= 0)
            {
                s_ch_fader.down_floor[(uint8_t) fade_down_idx] = s_ch_fader.up_peak[i];
            }
        }

        // Keep paired minimum re-synchronized while the source side stays near full scale.
        // This avoids "stuck min" when ch_fader[0]/ch_fader[5] remains high and only the paired side moves.
        if (pair_raw >= CH_FADER_PAIR_RESET_THRESHOLD)
        {
            const int8_t fade_down_idx = get_pair_fade_down_index_from_up(i);
            if (fade_down_idx >= 0)
            {
                s_ch_fader.down_floor[(uint8_t) fade_down_idx] = pair_raw;
            }
        }
    }
    else if (get_pair_fade_up_index_from_down(i) >= 0)
    {
        // Track fade-down floor; when near zero, clear paired fade-up peak.
        if (pair_raw < (s_ch_fader.down_floor[i] - CH_FADER_EXTREMA_HYSTERESIS))
        {
            s_ch_fader.down_floor[i] = pair_raw;

            if (s_ch_fader.down_floor[i] < CH_FADER_MIN_RESET_CUTOFF)
            {
                const int8_t fade_up_idx = get_pair_fade_up_index_from_down(i);
                if (fade_up_idx >= 0)
                {
                    s_ch_fader.up_peak[(uint8_t) fade_up_idx] = 0.0f;
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
static void emit_ch_fader_cc_if_needed(uint8_t i, bool output_as_note)
{
    const uint8_t note = (uint8_t) (60U + i);
    uint8_t value      = ch_fader_to_cc(s_ch_fader.raw[i]);

    if (get_pair_fade_down_index_from_up(i) >= 0)
    {
        value = (uint8_t) (127U - value);
    }

    if (output_as_note)
    {
        emit_ch_fader_note_if_needed(i, note, value);
        s_ch_fader.prev[i] = s_ch_fader.raw[i];
        return;
    }

    if (s_ch_fader.note_is_on[i] || s_ch_fader.note_scan_active[i])
    {
        if (s_ch_fader.note_is_on[i])
        {
            ui_midi_control_send_note(note, 0U, 0U);
        }
        clear_ch_fader_note_state(i);
    }

    // MIDI CC updates use a larger threshold to limit traffic and jitter.
    if (fabs(s_ch_fader.raw[i] - s_ch_fader.prev[i]) > CH_FADER_CC_UPDATE_THRESHOLD)
    {
        ui_midi_control_send_cc((uint8_t) (20U + i), value, 0U);
        s_ch_fader.prev[i] = s_ch_fader.raw[i];
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
    uint8_t active  = s_ch_fader.fade_down_active_source[pair_idx];
    uint8_t started = CH_FADER_FADE_DOWN_SOURCE_NONE;
    bool force_release = false;

    for (uint8_t i = 0; i < CH_FADER_FADE_DOWN_SOURCE_COUNT; i++)
    {
        const float prev = s_ch_fader.fade_down_source_prev[pair_idx][i];
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
            s_ch_fader.pair_fade_down_cut_active[pair_idx] = false;
            s_ch_fader.pair_bottom_hold_active[pair_idx] = false;
            s_ch_fader.pair_fade_down_bottomed[pair_idx] = false;
            s_ch_fader.pair_top_restore_value[pair_idx] = 0.0f;
            s_ch_fader.pair_top_hold_active[pair_idx] = false;
            s_ch_fader.pair_fade_up_topped[pair_idx] = false;
            s_ch_fader.fade_down_force_release_reads[pair_idx] =
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

    s_ch_fader.fade_down_active_source[pair_idx] = active;
    for (uint8_t i = 0; i < CH_FADER_FADE_DOWN_SOURCE_COUNT; i++)
    {
        s_ch_fader.fade_down_source_prev[pair_idx][i] = source[i];
    }

    force_release = (s_ch_fader.fade_down_force_release_reads[pair_idx] > 0U);
    if (force_release)
    {
        // Emit a short "released" window before applying the newly active
        // source. This creates the audible return between rapid cuts.
        s_ch_fader.fade_down_force_release_reads[pair_idx]--;
        s_ch_fader.fade_down_combined_raw[pair_idx] = 1.0f;
        return;
    }

    if (active != CH_FADER_FADE_DOWN_SOURCE_NONE)
    {
        // During a gesture, only the active source drives fade-down. This lets
        // the other sensor retrigger even if the first one remains deeper.
        s_ch_fader.fade_down_combined_raw[pair_idx] = source[active];
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
    s_ch_fader.fade_down_combined_raw[pair_idx] = combined;
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

    source[0] = apply_ch_fader_pair_onset_deadband(source_idx[0], s_ch_fader.raw[source_idx[0]]);
    for (uint8_t i = 1U; i < CH_FADER_FADE_DOWN_SOURCE_COUNT; i++)
    {
        if (source_idx[i] < MAG_SW_NUM)
        {
            source[i] = apply_ch_fader_fade_down_onset_deadband(s_ch_fader.raw[source_idx[i]]);
        }
    }

    const bool fade_up_active =
        apply_ch_fader_pair_onset_deadband(pair->fade_up_idx, s_ch_fader.raw[pair->fade_up_idx]) > 0.0f;
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
    return s_ch_fader.fade_down_combined_raw[pair->prev_idx];
}

// Fade-up sensors idle near 0.0 and rise toward 1.0 when pressed.
static float get_ch_fader_pair_fade_up_raw(const ch_fader_pair_runtime_t* pair)
{
    return apply_ch_fader_pair_onset_deadband(pair->fade_up_idx, s_ch_fader.raw[pair->fade_up_idx]);
}

static bool is_ch_fader_pair_reverse_enabled(const ch_fader_pair_runtime_t* pair)
{
    return (pair->prev_idx == CH_FADER_PAIR_A) ? s_ch_fader_reverse_a : s_ch_fader_reverse_b;
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
    const float curve_width = (pair->prev_idx == CH_FADER_PAIR_A) ? s_ch_fader_curve_width_a : s_ch_fader_curve_width_b;
    const uint8_t curve_cc  = ui_ch_fader_curve_width_to_midi_cc(curve_width);
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
    bool cut_active         = s_ch_fader.pair_fade_down_cut_active[pair->prev_idx];
    bool bottom_hold_active = s_ch_fader.pair_bottom_hold_active[pair->prev_idx];
    bool bottomed           = s_ch_fader.pair_fade_down_bottomed[pair->prev_idx];
    bool top_hold_active    = s_ch_fader.pair_top_hold_active[pair->prev_idx];
    bool topped             = s_ch_fader.pair_fade_up_topped[pair->prev_idx];
    bool reverse_fade_down_takeover = s_ch_fader.pair_reverse_fade_down_takeover[pair->prev_idx];
    bool bottom_entered     = false;
    bool top_entered        = false;
    float hold_value        = s_ch_fader.pair_hold_value[pair->prev_idx];
    float restore_value     = s_ch_fader.pair_bottom_restore_value[pair->prev_idx];
    float top_restore_value = s_ch_fader.pair_top_restore_value[pair->prev_idx];
    const float up_gain     = compute_ch_fader_exponential_curve(fade_up, curve_amount);
    const float down_gain   = 1.0f - compute_ch_fader_exponential_curve(1.0f - fade_down, curve_amount);
    const bool fade_up_active = (up_gain > 0.0f);
    const bool fade_down_released = (fade_down >= CH_FADER_PAIR_RESET_THRESHOLD);
    const bool fade_down_pressed = !fade_down_released;
    const bool both_released = !fade_up_active && !fade_down_pressed;
    uint8_t gesture_parent = s_ch_fader.pair_gesture_parent[pair->prev_idx];
    bool gesture_armed = s_ch_fader.pair_gesture_armed[pair->prev_idx];
    bool gesture_child_active = s_ch_fader.pair_gesture_child_active[pair->prev_idx];

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

        s_ch_fader.pair_hold_value[pair->prev_idx] = hold_value;
        s_ch_fader.pair_bottom_restore_value[pair->prev_idx] = restore_value;
        s_ch_fader.pair_top_restore_value[pair->prev_idx] = top_restore_value;
        s_ch_fader.pair_gesture_parent[pair->prev_idx] = gesture_parent;
        s_ch_fader.pair_gesture_armed[pair->prev_idx] = gesture_armed;
        s_ch_fader.pair_gesture_child_active[pair->prev_idx] = gesture_child_active;
        s_ch_fader.pair_fade_down_cut_active[pair->prev_idx] = cut_active;
        s_ch_fader.pair_bottom_hold_active[pair->prev_idx] = bottom_hold_active;
        s_ch_fader.pair_fade_down_bottomed[pair->prev_idx] = bottomed;
        s_ch_fader.pair_top_hold_active[pair->prev_idx] = top_hold_active;
        s_ch_fader.pair_fade_up_topped[pair->prev_idx] = topped;
        s_ch_fader.pair_reverse_fade_down_takeover[pair->prev_idx] = reverse_fade_down_takeover;
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

    s_ch_fader.pair_hold_value[pair->prev_idx]         = hold_value;
    s_ch_fader.pair_bottom_restore_value[pair->prev_idx] = restore_value;
    s_ch_fader.pair_top_restore_value[pair->prev_idx] = top_restore_value;
    s_ch_fader.pair_gesture_parent[pair->prev_idx] = gesture_parent;
    s_ch_fader.pair_gesture_armed[pair->prev_idx] = gesture_armed;
    s_ch_fader.pair_gesture_child_active[pair->prev_idx] = gesture_child_active;
    s_ch_fader.pair_fade_down_cut_active[pair->prev_idx] = cut_active;
    s_ch_fader.pair_bottom_hold_active[pair->prev_idx] = bottom_hold_active;
    s_ch_fader.pair_fade_down_bottomed[pair->prev_idx] = bottomed;
    s_ch_fader.pair_top_hold_active[pair->prev_idx] = top_hold_active;
    s_ch_fader.pair_fade_up_topped[pair->prev_idx] = topped;
    s_ch_fader.pair_reverse_fade_down_takeover[pair->prev_idx] = reverse_fade_down_takeover;
    return hold_value;
}

// Compute and commit one pair output (A or B) from the current fade-up/fade-down drive values.
static void update_ch_fader_pair_output(const ch_fader_pair_runtime_t* pair)
{
    const float up_now          = get_ch_fader_pair_fade_up_raw(pair);
    const float down_now        = get_ch_fader_pair_fade_down_raw(pair);
    const bool fade_changed     = (fabs(up_now - s_ch_fader.fade_up_prev[pair->prev_idx]) > CH_FADER_SEND_THRESHOLD) || (fabs(down_now - s_ch_fader.fade_down_prev[pair->prev_idx]) > CH_FADER_SEND_THRESHOLD);

    if (fade_changed)
    {
        const float ch_fader_value = apply_ch_fader_pair_reverse(pair, compute_ch_fader_pair_value(pair));
        submit_ch_fader_dsp_output(pair, ch_fader_value);
    }

    s_ch_fader.fade_up_prev[pair->prev_idx]   = up_now;
    s_ch_fader.fade_down_prev[pair->prev_idx] = down_now;
}

// Apply outgoing updates for ch_fader: MIDI CC stream and DSP DC controls.
static void apply_ch_fader_updates(bool output_as_note)
{
    update_ch_fader_fade_down_sources();

    for (uint32_t i = 0; i < MAG_SW_NUM; i++)
    {
        emit_ch_fader_cc_if_needed((uint8_t) i, output_as_note);
    }

    if (!s_ch_fader.fade_prev_valid)
    {
        for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
        {
            s_ch_fader.fade_up_prev[s_ch_fader_pairs[i].prev_idx]   = get_ch_fader_pair_fade_up_raw(&s_ch_fader_pairs[i]);
            s_ch_fader.fade_down_prev[s_ch_fader_pairs[i].prev_idx] = get_ch_fader_pair_fade_down_raw(&s_ch_fader_pairs[i]);
        }
        s_ch_fader.fade_prev_valid = true;
    }
    else
    {
        for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
        {
            update_ch_fader_pair_output(&s_ch_fader_pairs[i]);
        }
    }
}

void ui_ch_fader_process(const uint32_t* adc_samples, bool output_as_note)
{
    update_mag_samples(adc_samples);

    if (s_mag_calibration_count > MAG_CALIBRATION_COUNT_MAX)
    {
        update_ch_fader_from_mag();
        apply_ch_fader_updates(output_as_note);
    }
}

void ui_ch_fader_service_dsp_outputs(void)
{
    service_ch_fader_dsp_outputs();
}

void ui_ch_fader_reapply_outputs(void)
{
    update_ch_fader_fade_down_sources();

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(s_ch_fader_pairs); i++)
    {
        const ch_fader_pair_runtime_t* pair = &s_ch_fader_pairs[i];
        const float ch_fader_value = apply_ch_fader_pair_reverse(pair, compute_ch_fader_pair_value(pair));

        sync_ch_fader_dsp_output(pair, ch_fader_value);
        s_ch_fader.fade_up_prev[pair->prev_idx]   = get_ch_fader_pair_fade_up_raw(pair);
        s_ch_fader.fade_down_prev[pair->prev_idx] = get_ch_fader_pair_fade_down_raw(pair);
    }

    s_ch_fader.fade_prev_valid = true;
}

bool ui_ch_fader_apply_aux_assignments(uint8_t sensor2_assign, uint8_t sensor3_assign)
{
    return apply_ch_fader_aux_assignments(sensor2_assign, sensor3_assign);
}

uint8_t ui_ch_fader_get_aux_assign(uint8_t sensor_idx)
{
    return (sensor_idx == 2U) ? s_sensor2_aux_fade_down_assign
                              : s_sensor3_aux_fade_down_assign;
}

void ui_ch_fader_apply_dvs_delay(uint8_t delay_ms)
{
    apply_ch_fader_dvs_delay(delay_ms);
}

void ui_ch_fader_set_curve_width(uint8_t pair_idx, float width)
{
    width = clamp_ch_fader_curve_width(width);

    if (pair_idx == CH_FADER_PAIR_A)
    {
        s_ch_fader_curve_width_a = width;
    }
    else
    {
        s_ch_fader_curve_width_b = width;
    }
}

float ui_ch_fader_get_curve_width(uint8_t pair_idx)
{
    return (pair_idx == CH_FADER_PAIR_A) ? s_ch_fader_curve_width_a
                                         : s_ch_fader_curve_width_b;
}

bool ui_ch_fader_set_curve_width_from_cc(uint8_t pair_idx, uint8_t cc_value)
{
    float* width = (pair_idx == CH_FADER_PAIR_A) ? &s_ch_fader_curve_width_a
                                                 : &s_ch_fader_curve_width_b;
    const float new_width = clamp_ch_fader_curve_width(midi_cc_to_ch_fader_curve_width(cc_value));

    if (fabsf(new_width - *width) > 0.0001f)
    {
        *width = new_width;
        return true;
    }

    return false;
}

void ui_ch_fader_set_reverse(uint8_t pair_idx, bool enabled)
{
    if (pair_idx == CH_FADER_PAIR_A)
    {
        s_ch_fader_reverse_a = enabled;
    }
    else
    {
        s_ch_fader_reverse_b = enabled;
    }
}

void ui_ch_fader_mark_curve_dirty(void)
{
    mark_ch_fader_curve_dirty();
}

bool ui_ch_fader_validate_persist(const UI_ControlPersistState_t* state)
{
    if (state == NULL)
    {
        return false;
    }

    return (state->sensor2_aux_fade_down_assign <= UI_CH_FADER_AUX_ASSIGN_B) &&
           (state->sensor3_aux_fade_down_assign <= UI_CH_FADER_AUX_ASSIGN_B);
}

void ui_ch_fader_capture_persist(UI_ControlPersistState_t* state)
{
    if (state == NULL)
    {
        return;
    }

    state->ch_fader_dvs_delay_ms     = s_ch_fader_dvs_delay_ms;
    state->sensor2_aux_fade_down_assign = s_sensor2_aux_fade_down_assign;
    state->sensor3_aux_fade_down_assign = s_sensor3_aux_fade_down_assign;
    state->current_ch_fader_curve_width_a = s_ch_fader_curve_width_a;
    state->current_ch_fader_curve_width_b = s_ch_fader_curve_width_b;
    state->ch_fader_reverse_a        = s_ch_fader_reverse_a;
    state->ch_fader_reverse_b        = s_ch_fader_reverse_b;
}

void ui_ch_fader_reset(void)
{
    memset(s_ch_fader_dsp, 0, sizeof(s_ch_fader_dsp));

    s_mag_calibration_count = 0;
    for (uint16_t i = 0; i < MAG_SW_NUM; i++)
    {
        s_mag_val[i]        = 0;
        s_mag_offset_sum[i] = 0;
        s_mag_offset[i]     = 0;
    }

    s_ch_fader_dvs_delay_ms  = UI_CH_FADER_DVS_DELAY_DEFAULT_MS;
    s_sensor2_aux_fade_down_assign = UI_CH_FADER_AUX_ASSIGN_A;
    s_sensor3_aux_fade_down_assign = UI_CH_FADER_AUX_ASSIGN_B;
    s_ch_fader_curve_width_a    = UI_CH_FADER_CURVE_WIDTH_A_DEFAULT;
    s_ch_fader_curve_width_b    = UI_CH_FADER_CURVE_WIDTH_B_DEFAULT;
    s_ch_fader_reverse_a     = false;
    s_ch_fader_reverse_b     = false;
    s_ch_fader.position_a          = 0;
    s_ch_fader.position_b          = 0;
    (void) ui_ch_fader_apply_aux_assignments(s_sensor2_aux_fade_down_assign,
                                             s_sensor3_aux_fade_down_assign);

    for (uint16_t i = 0; i < MAG_SW_NUM; i++)
    {
        s_ch_fader.raw[i]                = 1.0f;
        s_ch_fader.prev[i]               = 1.0f;
        s_ch_fader.down_floor[i]         = 1.0f;
        s_ch_fader.up_peak[i]            = 0.0f;
        s_ch_fader.note_peak_vel[i]      = 0U;
        s_ch_fader.note_scan_start_ms[i] = 0U;
        s_ch_fader.note_is_on[i]         = false;
        s_ch_fader.note_scan_active[i]   = false;
    }
    for (uint8_t i = 0; i < CH_FADER_PAIR_COUNT; i++)
    {
        s_ch_fader.fade_up_prev[i]           = 0.0f;
        s_ch_fader.fade_down_prev[i]         = 1.0f;
        s_ch_fader.fade_down_combined_raw[i] = 1.0f;
        for (uint8_t source = 0; source < CH_FADER_FADE_DOWN_SOURCE_COUNT; source++)
        {
            s_ch_fader.fade_down_source_prev[i][source] = 1.0f;
        }
        s_ch_fader.fade_down_active_source[i] = CH_FADER_FADE_DOWN_SOURCE_NONE;
        s_ch_fader.fade_down_force_release_reads[i] = 0U;
        s_ch_fader.pair_gesture_parent[i] = CH_FADER_GESTURE_PARENT_NONE;
        s_ch_fader.pair_gesture_armed[i] = true;
        s_ch_fader.pair_gesture_child_active[i] = false;
        s_ch_fader.pair_hold_value[i]         = 0.0f;
        s_ch_fader.pair_bottom_restore_value[i] = 0.0f;
        s_ch_fader.pair_top_restore_value[i] = 0.0f;
        s_ch_fader.pair_fade_down_cut_active[i] = false;
        s_ch_fader.pair_bottom_hold_active[i] = false;
        s_ch_fader.pair_fade_down_bottomed[i] = false;
        s_ch_fader.pair_top_hold_active[i] = false;
        s_ch_fader.pair_fade_up_topped[i] = false;
        s_ch_fader.pair_reverse_fade_down_takeover[i] = false;
    }
    mark_ch_fader_curve_dirty();
    s_ch_fader.fade_prev_valid = false;
}

// OLED表示用: curve幅、DVS delay、Reverseの軽量コピー。scheduler停止区間専用。
void ui_ch_fader_capture_display_state(UI_ChFaderDisplayState_t* state)
{
    if (state == NULL)
    {
        return;
    }

    state->curve_width_a = s_ch_fader_curve_width_a;
    state->curve_width_b = s_ch_fader_curve_width_b;
    state->dvs_delay_ms  = s_ch_fader_dvs_delay_ms;
    state->reverse_a     = s_ch_fader_reverse_a;
    state->reverse_b     = s_ch_fader_reverse_b;
}
