/*
 * timecode_synth.c
 *
 * Two-channel Timecode Synth management, control mapping, and buffering.
 */

#include "timecode_synth.h"

#include "main.h"

#include <math.h>
#include <string.h>

#if ENABLE_TIMECODE_OSCILLATOR
enum
{
    TIMECODE_SYNTH_FIFO_FRAMES          = 256u,
    TIMECODE_SYNTH_PROCESS_BLOCK_FRAMES = 32u,
};

_Static_assert((TIMECODE_SYNTH_FIFO_FRAMES & (TIMECODE_SYNTH_FIFO_FRAMES - 1u)) == 0u,
               "Timecode synth FIFO size must be a power of two");
_Static_assert(TIMECODE_SYNTH_PROCESS_BLOCK_FRAMES <= TIMECODE_OSCILLATOR_MAX_BLOCK_FRAMES,
               "Timecode oscillator block buffer is too small");

typedef struct
{
    int32_t samples[TIMECODE_SYNTH_FIFO_FRAMES];
    uint32_t write_index;
    uint32_t read_index;
    volatile uint32_t overflow_frames;
    volatile uint32_t underrun_frames;
} TimecodeSynthFifo_t;

static TimecodeOscillator_t s_oscillator[TIMECODE_SYNTH_CHANNEL_COUNT];
static TimecodeSynthFifo_t s_fifo[TIMECODE_SYNTH_CHANNEL_COUNT];
static int32_t s_process_block[TIMECODE_SYNTH_CHANNEL_COUNT]
                              [TIMECODE_SYNTH_PROCESS_BLOCK_FRAMES];
static bool s_channel_enabled[TIMECODE_SYNTH_CHANNEL_COUNT] = {false, false};
static volatile uint8_t s_control[TIMECODE_SYNTH_CONTROL_COUNT] = {
    [TIMECODE_SYNTH_CONTROL_ROOT]        = 64u,
    [TIMECODE_SYNTH_CONTROL_MORPH]       = 0u,
    [TIMECODE_SYNTH_CONTROL_SLOPE]       = 64u,
    [TIMECODE_SYNTH_CONTROL_SMOOTH_FOLD] = 64u,
    [TIMECODE_SYNTH_CONTROL_WARP_AMOUNT] = 0u,
};
static volatile uint8_t s_ratio_set = TIMECODE_RATIO_OCTAVE;
static volatile uint8_t s_warp_algorithm = TIMECODE_WARP_CROSSFOLD;
static volatile uint32_t s_control_revision = 1u;
static uint32_t s_applied_revision = 0u;

static float normalize_control(uint8_t value)
{
    return (float) value * (1.0f / 127.0f);
}

static float smooth_fold_from_control(uint8_t value)
{
    enum
    {
        SMOOTH_FOLD_DEAD_ZONE_LOW  = 60u,
        SMOOTH_FOLD_DEAD_ZONE_HIGH = 67u,
    };

    if (value < SMOOTH_FOLD_DEAD_ZONE_LOW)
    {
        return -1.0f + ((float) value / (float) SMOOTH_FOLD_DEAD_ZONE_LOW);
    }
    if (value > SMOOTH_FOLD_DEAD_ZONE_HIGH)
    {
        return (float) (value - SMOOTH_FOLD_DEAD_ZONE_HIGH) /
               (float) (127u - SMOOTH_FOLD_DEAD_ZONE_HIGH);
    }
    return 0.0f;
}

static void apply_pending_controls(void)
{
    uint8_t controls[TIMECODE_SYNTH_CONTROL_COUNT];
    uint8_t ratio_set;
    uint8_t warp_algorithm;
    uint32_t revision_before;
    uint32_t revision_after;

    do
    {
        revision_before = s_control_revision;
        __DMB();
        for (uint32_t i = 0u; i < TIMECODE_SYNTH_CONTROL_COUNT; i++)
        {
            controls[i] = s_control[i];
        }
        ratio_set = s_ratio_set;
        warp_algorithm = s_warp_algorithm;
        __DMB();
        revision_after = s_control_revision;
    } while (revision_before != revision_after);

    if (revision_after == s_applied_revision)
    {
        return;
    }

    const float root_normalized = normalize_control(controls[TIMECODE_SYNTH_CONTROL_ROOT]);
    const float root_hz = 27.5f * powf(2.0f, 4.0f * root_normalized);
    const float morph = normalize_control(controls[TIMECODE_SYNTH_CONTROL_MORPH]);
    const float slope = 0.1f + 0.8f * normalize_control(controls[TIMECODE_SYNTH_CONTROL_SLOPE]);
    const float smooth_fold =
        smooth_fold_from_control(controls[TIMECODE_SYNTH_CONTROL_SMOOTH_FOLD]);
    const float warp_amount = normalize_control(controls[TIMECODE_SYNTH_CONTROL_WARP_AMOUNT]);

    for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
    {
        TimecodeOscillatorParameters_t parameters =
            *timecode_oscillator_get_parameters(&s_oscillator[channel]);
        parameters.root_hz        = root_hz;
        parameters.morph          = morph;
        parameters.slope          = slope;
        parameters.smooth_fold    = smooth_fold;
        parameters.ratio_set      = (TimecodeOscillatorRatioSet_t) ratio_set;
        parameters.warp_algorithm = (TimecodeOscillatorWarpAlgorithm_t) warp_algorithm;
        parameters.warp_amount    = warp_amount;
        timecode_oscillator_set_parameters(&s_oscillator[channel], &parameters);
    }

    s_applied_revision = revision_after;
}

static void fifo_reset(TimecodeSynthFifo_t* fifo)
{
    fifo->write_index = 0u;
    fifo->read_index  = 0u;
    memset(fifo->samples, 0, sizeof(fifo->samples));
}

static uint32_t fifo_used(TimecodeSynthFifo_t* fifo)
{
    uint32_t used = fifo->write_index - fifo->read_index;
    if (used > TIMECODE_SYNTH_FIFO_FRAMES)
    {
        fifo->read_index = fifo->write_index;
        used = 0u;
    }
    return used;
}

static void fifo_push(TimecodeSynthFifo_t* fifo, const int32_t* samples, uint32_t frame_count)
{
    if (frame_count > TIMECODE_SYNTH_FIFO_FRAMES)
    {
        samples += frame_count - TIMECODE_SYNTH_FIFO_FRAMES;
        frame_count = TIMECODE_SYNTH_FIFO_FRAMES;
    }

    const uint32_t used = fifo_used(fifo);
    const uint32_t free = TIMECODE_SYNTH_FIFO_FRAMES - used;
    if (frame_count > free)
    {
        const uint32_t drop_frames = frame_count - free;
        fifo->read_index += drop_frames;
        fifo->overflow_frames += drop_frames;
    }

    for (uint32_t frame = 0u; frame < frame_count; frame++)
    {
        fifo->samples[fifo->write_index & (TIMECODE_SYNTH_FIFO_FRAMES - 1u)] = samples[frame];
        fifo->write_index++;
    }
}

static int32_t fifo_pop(TimecodeSynthFifo_t* fifo)
{
    if (fifo_used(fifo) == 0u)
    {
        fifo->underrun_frames++;
        return 0;
    }

    const int32_t sample = fifo->samples[fifo->read_index & (TIMECODE_SYNTH_FIFO_FRAMES - 1u)];
    fifo->read_index++;
    return sample;
}
#endif

void timecode_synth_init(uint32_t sample_rate_hz)
{
#if ENABLE_TIMECODE_OSCILLATOR
    for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
    {
        s_channel_enabled[channel] = false;
        s_fifo[channel].overflow_frames = 0u;
        s_fifo[channel].underrun_frames = 0u;
    }
    timecode_synth_reset_for_sample_rate(sample_rate_hz);
#else
    (void) sample_rate_hz;
#endif
}

void timecode_synth_reset_for_sample_rate(uint32_t sample_rate_hz)
{
#if ENABLE_TIMECODE_OSCILLATOR
    for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
    {
        timecode_oscillator_init(&s_oscillator[channel], sample_rate_hz);
        timecode_oscillator_set_enabled(&s_oscillator[channel], s_channel_enabled[channel]);
        fifo_reset(&s_fifo[channel]);
    }
    s_applied_revision = 0u;
#else
    (void) sample_rate_hz;
#endif
}

void timecode_synth_update(void)
{
#if ENABLE_TIMECODE_OSCILLATOR
    apply_pending_controls();
#endif
}

void timecode_synth_set_channel_enabled(uint32_t channel, bool enabled)
{
#if ENABLE_TIMECODE_OSCILLATOR
    if (channel >= TIMECODE_SYNTH_CHANNEL_COUNT || s_channel_enabled[channel] == enabled)
    {
        return;
    }

    s_channel_enabled[channel] = enabled;
    fifo_reset(&s_fifo[channel]);
    timecode_oscillator_set_enabled(&s_oscillator[channel], enabled);
#else
    (void) channel;
    (void) enabled;
#endif
}

bool timecode_synth_is_channel_enabled(uint32_t channel)
{
#if ENABLE_TIMECODE_OSCILLATOR
    return channel < TIMECODE_SYNTH_CHANNEL_COUNT && s_channel_enabled[channel];
#else
    (void) channel;
    return false;
#endif
}

void timecode_synth_process_input(const int32_t* input,
                                  uint32_t frame_stride_words,
                                  uint32_t frame_count)
{
#if ENABLE_TIMECODE_OSCILLATOR
    if (input == NULL || frame_stride_words < (TIMECODE_SYNTH_CHANNEL_COUNT * 2u))
    {
        return;
    }

    uint32_t processed_frames = 0u;
    while (processed_frames < frame_count)
    {
        uint32_t block_frames = frame_count - processed_frames;
        if (block_frames > TIMECODE_SYNTH_PROCESS_BLOCK_FRAMES)
        {
            block_frames = TIMECODE_SYNTH_PROCESS_BLOCK_FRAMES;
        }

        const int32_t* block_input = input + processed_frames * frame_stride_words;
        for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
        {
            if (!s_channel_enabled[channel])
            {
                continue;
            }

            const uint32_t channel_offset = channel * 2u;
            timecode_oscillator_process_interleaved(&s_oscillator[channel],
                                                    block_input + channel_offset,
                                                    block_input + channel_offset + 1u,
                                                    frame_stride_words,
                                                    s_process_block[channel],
                                                    block_frames);
            fifo_push(&s_fifo[channel], s_process_block[channel], block_frames);
        }
        processed_frames += block_frames;
    }
#else
    (void) input;
    (void) frame_stride_words;
    (void) frame_count;
#endif
}

void timecode_synth_render_output(int32_t* output,
                                  uint32_t frame_stride_words,
                                  uint32_t frame_count)
{
#if ENABLE_TIMECODE_OSCILLATOR
    if (output == NULL || frame_stride_words < (TIMECODE_SYNTH_CHANNEL_COUNT * 2u))
    {
        return;
    }

    for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
    {
        if (!s_channel_enabled[channel])
        {
            continue;
        }

        const uint32_t channel_offset = channel * 2u;
        for (uint32_t frame = 0u; frame < frame_count; frame++)
        {
            const int32_t sample = fifo_pop(&s_fifo[channel]);
            output[frame * frame_stride_words + channel_offset] = sample;
            output[frame * frame_stride_words + channel_offset + 1u] = sample;
        }
    }
#else
    (void) output;
    (void) frame_stride_words;
    (void) frame_count;
#endif
}

void timecode_synth_set_control(TimecodeSynthControl_t control, uint8_t value)
{
#if ENABLE_TIMECODE_OSCILLATOR
    if ((uint32_t) control >= TIMECODE_SYNTH_CONTROL_COUNT)
    {
        return;
    }

    if (value > 127u)
    {
        value = 127u;
    }
    if (s_control[control] == value)
    {
        return;
    }

    s_control[control] = value;
    __DMB();
    s_control_revision++;
#else
    (void) control;
    (void) value;
#endif
}

void timecode_synth_set_ratio_set(TimecodeOscillatorRatioSet_t ratio_set)
{
#if ENABLE_TIMECODE_OSCILLATOR
    if ((uint32_t) ratio_set > TIMECODE_RATIO_CHORD || s_ratio_set == (uint8_t) ratio_set)
    {
        return;
    }

    s_ratio_set = (uint8_t) ratio_set;
    __DMB();
    s_control_revision++;
#else
    (void) ratio_set;
#endif
}

void timecode_synth_set_warp_algorithm(TimecodeOscillatorWarpAlgorithm_t warp_algorithm)
{
#if ENABLE_TIMECODE_OSCILLATOR
    if ((uint32_t) warp_algorithm > TIMECODE_WARP_COMPARATOR ||
        s_warp_algorithm == (uint8_t) warp_algorithm)
    {
        return;
    }

    s_warp_algorithm = (uint8_t) warp_algorithm;
    __DMB();
    s_control_revision++;
#else
    (void) warp_algorithm;
#endif
}

TimecodeOscillatorRatioSet_t timecode_synth_get_ratio_set(void)
{
#if ENABLE_TIMECODE_OSCILLATOR
    return (TimecodeOscillatorRatioSet_t) s_ratio_set;
#else
    return TIMECODE_RATIO_OCTAVE;
#endif
}

TimecodeOscillatorWarpAlgorithm_t timecode_synth_get_warp_algorithm(void)
{
#if ENABLE_TIMECODE_OSCILLATOR
    return (TimecodeOscillatorWarpAlgorithm_t) s_warp_algorithm;
#else
    return TIMECODE_WARP_CROSSFOLD;
#endif
}
