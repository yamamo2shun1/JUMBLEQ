/*
 * timecode_oscillator.c
 *
 * Relative-motion timecode tracker and oscillator.
 */

#include "timecode_oscillator.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O3")
#endif

#define TIMECODE_PI        3.14159265358979323846f
#define TIMECODE_TWO_PI    6.28318530717958647692f
#define TIMECODE_INT_SCALE (1.0f / 2147483648.0f)

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float maxf(float a, float b)
{
    return (a > b) ? a : b;
}

static float fold_bipolar(float value)
{
    float folded = fmodf(value + 1.0f, 4.0f);
    if (folded < 0.0f)
    {
        folded += 4.0f;
    }
    if (folded > 2.0f)
    {
        folded = 4.0f - folded;
    }
    return folded - 1.0f;
}

static float shape_wave(float phase, float morph, float slope)
{
    const float width = clampf(slope, 0.05f, 0.95f);
    float triangle;
    if (phase < width)
    {
        triangle = -1.0f + (2.0f * phase / width);
    }
    else
    {
        triangle = 1.0f - (2.0f * (phase - width) / (1.0f - width));
    }

    const float sine_shape     = sinf((TIMECODE_PI * 0.5f) * triangle);
    const float rising_saw     = phase * 2.0f - 1.0f;
    const float amount         = clampf(morph, 0.0f, 1.0f);
    const float sine_to_tri    = clampf(amount * 2.0f, 0.0f, 1.0f);
    const float triangle_to_saw = clampf(amount * 2.0f - 1.0f, 0.0f, 1.0f);
    const float rounded        = sine_shape + (triangle - sine_shape) * sine_to_tri;

    return rounded + (rising_saw - rounded) * triangle_to_saw;
}

static int32_t normalized_to_s32(float sample)
{
    if (sample >= 1.0f)
    {
        return INT32_MAX;
    }
    if (sample <= -1.0f)
    {
        return INT32_MIN;
    }
    return (int32_t) (sample * 2147483647.0f);
}

static void update_derived_parameters(TimecodeOscillator_t* oscillator)
{
    const float sample_rate = maxf(oscillator->sample_rate_hz, 1.0f);
    const float smooth_seconds = maxf(oscillator->parameters.smooth_ms, 0.1f) * 0.001f;
    const float release_seconds = maxf(oscillator->parameters.release_ms, 0.1f) * 0.001f;

    oscillator->dc_coefficient = expf(-TIMECODE_TWO_PI * 10.0f / sample_rate);
    oscillator->tracker_coefficient = expf(-1.0f / (smooth_seconds * sample_rate));
    oscillator->shape_coefficient = expf(-1.0f / (0.01f * sample_rate));
    oscillator->gate_attack_coefficient = expf(-1.0f / (0.005f * sample_rate));
    oscillator->gate_release_coefficient = expf(-1.0f / (release_seconds * sample_rate));
    oscillator->nominal_phase_step =
        TIMECODE_TWO_PI * maxf(oscillator->parameters.nominal_carrier_hz, 1.0f) / sample_rate;
    oscillator->threshold_squared = oscillator->parameters.threshold * oscillator->parameters.threshold;
    oscillator->output_gain = powf(10.0f, oscillator->parameters.output_gain_db / 20.0f);
    oscillator->hold_samples =
        (uint32_t) (oscillator->parameters.hold_ms * 0.001f * sample_rate + 0.5f);
}

static TimecodeOscillatorParameters_t default_parameters(void)
{
    TimecodeOscillatorParameters_t parameters = {
        .root_hz           = 110.0f,
        .nominal_carrier_hz = 1000.0f,
        .smooth_ms         = 3.0f,
        .hold_ms           = 10.0f,
        .release_ms        = 20.0f,
        .threshold         = 0.00562341325f,  // -45 dBFS
        .confidence_min    = 0.2f,
        .morph             = 0.0f,
        .ratio_set         = TIMECODE_RATIO_OCTAVE,
        .slope             = 0.5f,
        .smooth_fold       = 0.0f,
        .warp_algorithm    = TIMECODE_WARP_CLEAN,
        .warp_amount       = 0.0f,
        .direction         = 1.0f,
        .output_gain_db    = -18.0f,
    };
    return parameters;
}

void timecode_oscillator_init(TimecodeOscillator_t* oscillator, uint32_t sample_rate_hz)
{
    if (oscillator == NULL)
    {
        return;
    }

    memset(oscillator, 0, sizeof(*oscillator));
    oscillator->sample_rate_hz = (sample_rate_hz > 0U) ? (float) sample_rate_hz : 48000.0f;
    TimecodeOscillatorParameters_t parameters = default_parameters();
    timecode_oscillator_set_parameters(oscillator, &parameters);
    timecode_oscillator_reset(oscillator);
}

void timecode_oscillator_reset(TimecodeOscillator_t* oscillator)
{
    if (oscillator == NULL)
    {
        return;
    }

    const TimecodeOscillatorParameters_t parameters = oscillator->parameters;
    const float sample_rate_hz = oscillator->sample_rate_hz;
    const bool enabled = oscillator->enabled;

    memset(oscillator, 0, sizeof(*oscillator));
    oscillator->parameters     = parameters;
    oscillator->sample_rate_hz = sample_rate_hz;
    oscillator->enabled        = enabled;
    oscillator->morph_smooth   = parameters.morph;
    oscillator->slope_smooth   = parameters.slope;
    oscillator->smooth_fold_smooth = parameters.smooth_fold;
    oscillator->diagnostics.state = TIMECODE_TRACK_NO_SIGNAL;
    update_derived_parameters(oscillator);
}

void timecode_oscillator_set_enabled(TimecodeOscillator_t* oscillator, bool enabled)
{
    if (oscillator == NULL || oscillator->enabled == enabled)
    {
        return;
    }

    oscillator->enabled = enabled;
    timecode_oscillator_reset(oscillator);
}

void timecode_oscillator_set_parameters(TimecodeOscillator_t* oscillator,
                                        const TimecodeOscillatorParameters_t* parameters)
{
    if (oscillator == NULL || parameters == NULL)
    {
        return;
    }

    oscillator->parameters = *parameters;
    oscillator->parameters.root_hz = clampf(parameters->root_hz, 20.0f, 2000.0f);
    oscillator->parameters.nominal_carrier_hz =
        clampf(parameters->nominal_carrier_hz, 100.0f, 4000.0f);
    oscillator->parameters.smooth_ms = clampf(parameters->smooth_ms, 0.1f, 100.0f);
    oscillator->parameters.hold_ms = clampf(parameters->hold_ms, 0.0f, 500.0f);
    oscillator->parameters.release_ms = clampf(parameters->release_ms, 1.0f, 1000.0f);
    oscillator->parameters.threshold = clampf(parameters->threshold, 0.00001f, 1.0f);
    oscillator->parameters.confidence_min = clampf(parameters->confidence_min, 0.0f, 1.0f);
    oscillator->parameters.morph = clampf(parameters->morph, 0.0f, 1.0f);
    if (oscillator->parameters.ratio_set > TIMECODE_RATIO_CHORD)
    {
        oscillator->parameters.ratio_set = TIMECODE_RATIO_OCTAVE;
    }
    oscillator->parameters.slope = clampf(parameters->slope, 0.05f, 0.95f);
    oscillator->parameters.smooth_fold = clampf(parameters->smooth_fold, -1.0f, 1.0f);
    if (oscillator->parameters.warp_algorithm > TIMECODE_WARP_COMPARATOR)
    {
        oscillator->parameters.warp_algorithm = TIMECODE_WARP_CLEAN;
    }
    oscillator->parameters.warp_amount = clampf(parameters->warp_amount, 0.0f, 1.0f);
    oscillator->parameters.direction = clampf(parameters->direction, -1.0f, 1.0f);
    oscillator->parameters.output_gain_db = clampf(parameters->output_gain_db, -80.0f, 0.0f);
    update_derived_parameters(oscillator);
}

const TimecodeOscillatorParameters_t* timecode_oscillator_get_parameters(const TimecodeOscillator_t* oscillator)
{
    return (oscillator != NULL) ? &oscillator->parameters : NULL;
}

const TimecodeOscillatorDiagnostics_t* timecode_oscillator_get_diagnostics(const TimecodeOscillator_t* oscillator)
{
    return (oscillator != NULL) ? &oscillator->diagnostics : NULL;
}

static void process_chunk(TimecodeOscillator_t* oscillator,
                          const int32_t* input_l,
                          const int32_t* input_r,
                          uint32_t input_stride_words,
                          int32_t* output,
                          uint32_t frame_count)
{
    float raw_modulator[TIMECODE_OSCILLATOR_MAX_BLOCK_FRAMES];
    const float tracker_mix = 1.0f - oscillator->tracker_coefficient;

    for (uint32_t frame = 0; frame < frame_count; frame++)
    {
        const uint32_t input_index = frame * input_stride_words;
        const float input_l_normalized = (float) input_l[input_index] * TIMECODE_INT_SCALE;
        const float input_r_normalized = (float) input_r[input_index] * TIMECODE_INT_SCALE;
        const float l = input_l_normalized - oscillator->previous_l_input +
                        oscillator->dc_coefficient * oscillator->previous_l_dc;
        const float r = input_r_normalized - oscillator->previous_r_input +
                        oscillator->dc_coefficient * oscillator->previous_r_dc;

        oscillator->previous_l_input = input_l_normalized;
        oscillator->previous_r_input = input_r_normalized;
        oscillator->previous_l_dc    = l;
        oscillator->previous_r_dc    = r;

        const float cross_now = oscillator->previous_l * r - oscillator->previous_r * l;
        const float dot_now = oscillator->previous_l * l + oscillator->previous_r * r;
        const float energy_now = l * l + r * r;
        oscillator->previous_l = l;
        oscillator->previous_r = r;

        oscillator->cross_smooth =
            oscillator->cross_smooth * oscillator->tracker_coefficient + cross_now * tracker_mix;
        oscillator->dot_smooth =
            oscillator->dot_smooth * oscillator->tracker_coefficient + dot_now * tracker_mix;
        oscillator->energy_smooth =
            oscillator->energy_smooth * oscillator->tracker_coefficient + energy_now * tracker_mix;
        raw_modulator[frame] = tanhf((l + r) * 2.0f);
    }

    const float phase_step = atan2f(oscillator->cross_smooth, oscillator->dot_smooth);
    float raw_speed = (phase_step / oscillator->nominal_phase_step) * oscillator->parameters.direction;
    raw_speed = clampf(raw_speed, -4.0f, 4.0f);

    const float correlation = sqrtf(oscillator->cross_smooth * oscillator->cross_smooth +
                                    oscillator->dot_smooth * oscillator->dot_smooth);
    const float confidence = clampf(correlation / maxf(oscillator->energy_smooth, 1.0e-12f),
                                    0.0f,
                                    1.0f);
    const bool valid = oscillator->energy_smooth > oscillator->threshold_squared &&
                       confidence >= oscillator->parameters.confidence_min &&
                       fabsf(raw_speed) >= 0.005f && fabsf(raw_speed) <= 4.0f;

    float gate_target = 0.0f;
    if (valid)
    {
        const float block_coefficient =
            powf(oscillator->tracker_coefficient, (float) frame_count);
        oscillator->speed_memory = oscillator->speed_memory * block_coefficient +
                                   raw_speed * (1.0f - block_coefficient);
        oscillator->hold_left_samples = oscillator->hold_samples;
        oscillator->diagnostics.state = TIMECODE_TRACK_TRACKING;
        gate_target = 1.0f;
    }
    else if (oscillator->hold_left_samples > 0U)
    {
        if (oscillator->hold_left_samples > frame_count)
        {
            oscillator->hold_left_samples -= frame_count;
        }
        else
        {
            oscillator->hold_left_samples = 0U;
        }
        oscillator->diagnostics.state = TIMECODE_TRACK_HOLD;
        gate_target = 1.0f;
    }
    else
    {
        oscillator->diagnostics.state = TIMECODE_TRACK_NO_SIGNAL;
    }

    float ratios[TIMECODE_OSCILLATOR_VOICE_COUNT] = {1.0f, 0.5f, 0.25f, 0.125f};
    if (oscillator->parameters.ratio_set == TIMECODE_RATIO_HARMONIC)
    {
        ratios[1] = 2.0f;
        ratios[2] = 3.0f;
        ratios[3] = 4.0f;
    }
    else if (oscillator->parameters.ratio_set == TIMECODE_RATIO_CHORD)
    {
        ratios[1] = 1.25f;
        ratios[2] = 1.5f;
        ratios[3] = 2.0f;
    }

    float filter_alpha[TIMECODE_OSCILLATOR_VOICE_COUNT];
    for (uint32_t voice = 0; voice < TIMECODE_OSCILLATOR_VOICE_COUNT; voice++)
    {
        const float voice_frequency =
            fabsf(oscillator->parameters.root_hz * oscillator->speed_memory * ratios[voice]);
        const float cutoff = clampf(maxf(voice_frequency, 5.0f) * 1.5f,
                                    20.0f,
                                    oscillator->sample_rate_hz * 0.45f);
        filter_alpha[voice] = 1.0f - expf(-TIMECODE_TWO_PI * cutoff / oscillator->sample_rate_hz);
    }

    const float shape_mix = 1.0f - oscillator->shape_coefficient;
    const float base_increment =
        oscillator->parameters.root_hz * oscillator->speed_memory / oscillator->sample_rate_hz;

    for (uint32_t frame = 0; frame < frame_count; frame++)
    {
        oscillator->morph_smooth = oscillator->morph_smooth * oscillator->shape_coefficient +
                                   oscillator->parameters.morph * shape_mix;
        oscillator->slope_smooth = oscillator->slope_smooth * oscillator->shape_coefficient +
                                   oscillator->parameters.slope * shape_mix;
        oscillator->smooth_fold_smooth =
            oscillator->smooth_fold_smooth * oscillator->shape_coefficient +
            oscillator->parameters.smooth_fold * shape_mix;

        const float smooth_amount = maxf(-oscillator->smooth_fold_smooth, 0.0f);
        const float fold_amount = maxf(oscillator->smooth_fold_smooth, 0.0f);
        const float fold_drive = 1.0f + fold_amount * 4.0f;
        float voice_output[TIMECODE_OSCILLATOR_VOICE_COUNT];

        for (uint32_t voice = 0; voice < TIMECODE_OSCILLATOR_VOICE_COUNT; voice++)
        {
            oscillator->phase[voice] += base_increment * ratios[voice];
            oscillator->phase[voice] -= floorf(oscillator->phase[voice]);

            const float shaped = shape_wave(oscillator->phase[voice],
                                            oscillator->morph_smooth,
                                            oscillator->slope_smooth);
            oscillator->voice_lp_a[voice] +=
                filter_alpha[voice] * (shaped - oscillator->voice_lp_a[voice]);
            oscillator->voice_lp_b[voice] +=
                filter_alpha[voice] * (oscillator->voice_lp_a[voice] - oscillator->voice_lp_b[voice]);
            const float smoothed = shaped * (1.0f - smooth_amount) +
                                   oscillator->voice_lp_b[voice] * smooth_amount;
            const float folded = fold_bipolar(smoothed * fold_drive);
            voice_output[voice] = smoothed * (1.0f - fold_amount) + folded * fold_amount;
        }

        const float carrier_output = voice_output[0] * 0.45f + voice_output[1] * 0.25f +
                                     voice_output[2] * 0.17f + voice_output[3] * 0.13f;
        float processed = carrier_output;
        switch (oscillator->parameters.warp_algorithm)
        {
            case TIMECODE_WARP_CROSSFOLD:
            {
                const float amount = oscillator->parameters.warp_amount;
                // Use the timecode waveform to modulate wavefolder drive instead
                // of adding it to the carrier. This keeps the timecode tone from
                // appearing by itself while retaining audio-rate cross modulation.
                const float cross_drive =
                    1.0f + amount * 2.0f * (raw_modulator[frame] + 1.0f);
                const float folded = fold_bipolar(carrier_output * cross_drive);
                processed = carrier_output + (folded - carrier_output) * amount;
                break;
            }
            case TIMECODE_WARP_RING_MOD:
            {
                const float ringed = carrier_output * raw_modulator[frame];
                processed = carrier_output * (1.0f - oscillator->parameters.warp_amount) +
                            ringed * oscillator->parameters.warp_amount;
                break;
            }
            case TIMECODE_WARP_COMPARATOR:
            {
                const float compared = (carrier_output > raw_modulator[frame]) ? 0.7f : -0.7f;
                processed = carrier_output * (1.0f - oscillator->parameters.warp_amount) +
                            compared * oscillator->parameters.warp_amount;
                break;
            }
            case TIMECODE_WARP_CLEAN:
            default:
                break;
        }

        const float gate_coefficient =
            (gate_target > oscillator->gate) ? oscillator->gate_attack_coefficient
                                             : oscillator->gate_release_coefficient;
        oscillator->gate = oscillator->gate * gate_coefficient +
                           gate_target * (1.0f - gate_coefficient);
        const float safe_output = tanhf(processed) * oscillator->gate * oscillator->output_gain;
        output[frame] = normalized_to_s32(safe_output);
    }

    oscillator->diagnostics.signed_speed = oscillator->speed_memory;
    oscillator->diagnostics.confidence = confidence;
    oscillator->diagnostics.input_level = sqrtf(maxf(oscillator->energy_smooth, 0.0f));
}

void timecode_oscillator_process_interleaved(TimecodeOscillator_t* oscillator,
                                             const int32_t* input_l,
                                             const int32_t* input_r,
                                             uint32_t input_stride_words,
                                             int32_t* output,
                                             uint32_t frame_count)
{
    if (oscillator == NULL || output == NULL || frame_count == 0U)
    {
        return;
    }

    if (!oscillator->enabled || input_l == NULL || input_r == NULL || input_stride_words == 0U)
    {
        memset(output, 0, frame_count * sizeof(*output));
        return;
    }

    while (frame_count > 0U)
    {
        const uint32_t chunk_frames =
            (frame_count > TIMECODE_OSCILLATOR_MAX_BLOCK_FRAMES)
                ? TIMECODE_OSCILLATOR_MAX_BLOCK_FRAMES
                : frame_count;
        process_chunk(oscillator,
                      input_l,
                      input_r,
                      input_stride_words,
                      output,
                      chunk_frames);
        input_l += chunk_frames * input_stride_words;
        input_r += chunk_frames * input_stride_words;
        output += chunk_frames;
        frame_count -= chunk_frames;
    }
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
