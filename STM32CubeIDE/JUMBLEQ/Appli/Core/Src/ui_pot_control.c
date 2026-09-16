/*
 * ui_pot_control.c
 *
 * 16ch analog MUX and pot input processing: moving average, hysteresis,
 * quantization, pot-mag calibration and the DSP/MIDI output application.
 */

#include "ui_pot_control_internal.h"

#include "ui_midi_control_internal.h"
#include "ui_routing_control_internal.h"
#include "adau1466.h"
#include "main.h"
#include "timecode_synth.h"

#include <math.h>

#define POT_CH_SEL_WAIT           1
#define POT_MA_SIZE               4  // 移動平均のサンプル数
#define POT_NUM                   16
#define POT_HYSTERESIS_NUM        12
#define POT_CC_HYSTERESIS_RAW     8U
#define POT_CC_MIN_DEADZONE_VALUE 2U
#define MAG_CALIBRATION_COUNT_MAX 100
#define MAG_CH_FADER_CUTOFF          16
#define MAG_CH_FADER_RANGE           1400

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

typedef struct
{
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
} ui_pot_state_t;

static ui_pot_state_t s_pot;

static uint8_t s_note_peak_vel[128];
static uint32_t s_note_scan_start_ms[128];
static bool s_note_is_on[128];
static bool s_note_scan_active[128];

static const uint8_t POT_MAG_CH_FIRST = POT_CH_MAG0;
static const uint8_t POT_MAG_CH_LAST  = POT_CH_MAG3;

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
            ui_midi_control_send_note(note, 0U, 0U);
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

        ui_midi_control_send_note(note, velocity, 0U);
        s_note_is_on[note]       = true;
        s_note_scan_active[note] = false;
    }
}

static void emit_mag_output(uint8_t cc_number, uint8_t note_number, uint8_t value, bool output_as_note)
{
    if (output_as_note)
    {
        emit_note_edge_if_needed(note_number, value);
    }
    else
    {
        ui_midi_control_send_cc(cc_number, value, 0U);
    }
}

static void apply_dry_wet_value(uint16_t value, uint8_t return_assign)
{
    switch (return_assign)
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

void ui_pot_control_apply_return_outputs(uint8_t return_assign)
{
    if (return_assign != INPUT_SRC_NONE)
    {
        control_input_from_return_gain(s_pot.pot_val[POT_CH_RETURN_IN]);
    }
    apply_dry_wet_value(s_pot.pot_val[POT_CH_DRY_WET], return_assign);
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

static void apply_pot_value(uint8_t channel, uint16_t value, bool synth_mode_active, bool output_as_note, uint8_t return_assign)
{
    switch (channel)
    {
    case POT_CH_CC0:
        timecode_synth_set_control(TIMECODE_SYNTH_CONTROL_ROOT, (uint8_t) value);
        if (!synth_mode_active)
        {
            ui_midi_control_send_cc(0, value, 0);
        }
        break;
    case POT_CH_CC1:
        timecode_synth_set_control(TIMECODE_SYNTH_CONTROL_MORPH, (uint8_t) value);
        if (!synth_mode_active)
        {
            ui_midi_control_send_cc(1, value, 0);
        }
        break;
    case POT_CH_CH1_IN:
        control_input_from_ch1_gain(value);
        break;
    case POT_CH_CC2:
        timecode_synth_set_control(TIMECODE_SYNTH_CONTROL_SLOPE, (uint8_t) value);
        if (!synth_mode_active)
        {
            ui_midi_control_send_cc(2, value, 0);
        }
        break;
    case POT_CH_CC3:
        timecode_synth_set_control(TIMECODE_SYNTH_CONTROL_SMOOTH_FOLD, (uint8_t) value);
        if (!synth_mode_active)
        {
            ui_midi_control_send_cc(3, value, 0);
        }
        break;
    case POT_CH_CC4:
        timecode_synth_set_control(TIMECODE_SYNTH_CONTROL_WARP_AMOUNT, (uint8_t) value);
        if (!synth_mode_active)
        {
            ui_midi_control_send_cc(4, value, 0);
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
        apply_dry_wet_value(value, return_assign);
        break;
    case POT_CH_RETURN_IN:
        if (return_assign == INPUT_SRC_NONE)
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
        emit_mag_output(channel,
                        (uint8_t) (68U + (channel - POT_MAG_CH_FIRST)),
                        (uint8_t) value,
                        output_as_note);
        break;
    default:
        break;
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

    if (!s_pot.pot_hysteresis_has_last_sent[idx])
    {
        s_pot.pot_hysteresis_last_sent[idx]     = candidate;
        s_pot.pot_hysteresis_has_last_sent[idx] = true;
        *value_out = candidate;
        return true;
    }

    const uint16_t last_sent = s_pot.pot_hysteresis_last_sent[idx];
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

    s_pot.pot_hysteresis_last_sent[idx] = candidate;
    *value_out = candidate;
    return true;
}

void ui_pot_control_process(const uint32_t adc_samples[ADC_NUM],
                            bool synth_mode_active,
                            bool output_as_note,
                            uint8_t return_assign)
{
    if (s_pot.pot_ch_counter < POT_CH_SEL_WAIT)
    {
        set_pot_mux_channel(s_pot.pot_ch);
        s_pot.pot_ch_counter++;
    }
    else if (s_pot.pot_ch_counter >= POT_CH_SEL_WAIT)
    {
        const uint8_t ch = s_pot.pot_ch;
        const uint16_t ma_index = s_pot.pot_ma_index[ch];
        const uint16_t sample_now = (uint16_t) read_pot_sample_from_adc(ch, adc_samples[6]);

        s_pot.pot_val_ma[ch][ma_index] = sample_now;
        if (s_pot.pot_sample_count[ch] < POT_MA_SIZE)
        {
            s_pot.pot_sample_count[ch]++;
        }

        if (is_pot_hysteresis_channel(ch))
        {
            s_pot.pot_hysteresis_raw_ma[ch][ma_index] = (uint16_t) adc_samples[6];
        }

        s_pot.pot_ma_index[ch] = (ma_index + 1U) % POT_MA_SIZE;

        if (is_pot_mag_channel(ch))
        {
            // Pot-mag channels (12-15): prioritize tracking speed over MA smoothing.
            s_pot.pot_val[ch] = sample_now;
        }
        else
        {
            float pot_sum = 0.0f;
            const uint8_t sample_count = s_pot.pot_sample_count[ch];
            for (uint8_t j = 0; j < sample_count; j++)
            {
                pot_sum += (float) s_pot.pot_val_ma[ch][j];
            }
            s_pot.pot_val[ch] = round(pot_sum / (float) sample_count);
        }

        if (!is_pot_mag_channel(ch))
        {
            if (is_pot_hysteresis_channel(ch))
            {
                uint32_t raw_sum = 0U;
                const uint8_t sample_count = s_pot.pot_sample_count[ch];
                uint16_t stabilized_value = s_pot.pot_val[ch];

                for (uint8_t j = 0; j < sample_count; j++)
                {
                    raw_sum += s_pot.pot_hysteresis_raw_ma[ch][j];
                }

                if (should_apply_pot_hysteresis(ch, (uint16_t) (raw_sum / sample_count), &stabilized_value))
                {
                    s_pot.pot_val[ch] = stabilized_value;
                    apply_pot_value(ch, stabilized_value, synth_mode_active, output_as_note, return_assign);
                }
            }
        }
        else
        {
            const uint8_t idx     = pot_mag_index(ch);
            const uint16_t sample = s_pot.pot_val[ch];

            if (s_pot.pot_mag_calibration_count[idx] < MAG_CALIBRATION_COUNT_MAX)
            {
                s_pot.pot_mag_offset_sum[idx] += sample;
                s_pot.pot_mag_calibration_count[idx]++;
            }
            else if (s_pot.pot_mag_calibration_count[idx] == MAG_CALIBRATION_COUNT_MAX)
            {
                s_pot.pot_mag_offset[idx] = (uint16_t) (s_pot.pot_mag_offset_sum[idx] / MAG_CALIBRATION_COUNT_MAX);
                s_pot.pot_mag_calibration_count[idx]++;
            }
            else
            {
                const uint16_t offset = s_pot.pot_mag_offset[idx];
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
                s_pot.pot_mag_candidate[idx] = candidate;

                {
                    const uint8_t filtered = s_pot.pot_mag_candidate[idx];
                    const uint8_t diff     = (filtered > s_pot.pot_mag_state[idx]) ? (uint8_t) (filtered - s_pot.pot_mag_state[idx]) : (uint8_t) (s_pot.pot_mag_state[idx] - filtered);
                    if (diff >= 1U)
                    {
                        s_pot.pot_mag_state[idx] = filtered;
                        apply_pot_value(ch, filtered, synth_mode_active, output_as_note, return_assign);
                    }
                }
            }
        }

        s_pot.pot_ch         = (s_pot.pot_ch + 1) % POT_NUM;
        s_pot.pot_ch_counter = 0;
    }
}

void ui_pot_control_set_initial_channel(void)
{
    s_pot.pot_ch = POT_CH_CC1;
}

void ui_pot_control_reset(void)
{
    for (uint16_t i = 0; i < POT_NUM; i++)
    {
        s_pot.pot_ma_index[i]    = 0;
        s_pot.pot_sample_count[i] = 0U;
        s_pot.pot_val[i]         = 0;
        s_pot.pot_val_prev[i][0] = 0;
        s_pot.pot_val_prev[i][1] = 0;
        for (uint16_t j = 0; j < POT_MA_SIZE; j++)
        {
            s_pot.pot_val_ma[i][j] = 0;
        }
    }
    for (uint16_t i = 0; i < POT_HYSTERESIS_NUM; i++)
    {
        s_pot.pot_hysteresis_last_sent[i]     = 0U;
        s_pot.pot_hysteresis_has_last_sent[i] = false;
        for (uint16_t j = 0; j < POT_MA_SIZE; j++)
        {
            s_pot.pot_hysteresis_raw_ma[i][j] = 0U;
        }
    }

    for (uint16_t i = 0; i < 4U; i++)
    {
        s_pot.pot_mag_calibration_count[i] = 0;
        s_pot.pot_mag_offset_sum[i]        = 0;
        s_pot.pot_mag_offset[i]            = 0;
        s_pot.pot_mag_candidate[i]         = 0U;
        s_pot.pot_mag_stable_count[i]      = 0U;
        s_pot.pot_mag_state[i]             = 0U;
    }

    for (uint16_t i = 0; i < 128U; i++)
    {
        s_note_peak_vel[i]      = 0U;
        s_note_scan_start_ms[i] = 0U;
        s_note_is_on[i]         = false;
        s_note_scan_active[i]   = false;
    }

    s_pot.pot_ch         = POT_CH_CC0;
    s_pot.pot_ch_counter = 0;
}

// OLED表示用: 6個のpot生値(ADC値)を軽量コピーする。scheduler停止区間専用。
void ui_pot_control_capture_display_state(UI_PotDisplayState_t* state)
{
    if (state == NULL)
    {
        return;
    }

    state->ch1_input    = s_pot.pot_val[POT_CH_CH1_IN];
    state->ch2_input    = s_pot.pot_val[POT_CH_CH2_IN];
    state->ch1_output   = s_pot.pot_val[POT_CH_CH1_OUT];
    state->ch2_output   = s_pot.pot_val[POT_CH_CH2_OUT];
    state->return_input = s_pot.pot_val[POT_CH_RETURN_IN];
    state->hp_output    = s_pot.pot_val[POT_CH_HP_OUT];
}