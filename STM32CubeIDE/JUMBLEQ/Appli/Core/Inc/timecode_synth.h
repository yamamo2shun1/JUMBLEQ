/*
 * timecode_synth.h
 *
 * Timecode Synth subsystem facade. Hardware audio routing remains in
 * audio_control; oscillator instances, controls, and buffering live here.
 */

#ifndef INC_TIMECODE_SYNTH_H_
#define INC_TIMECODE_SYNTH_H_

#include <stdbool.h>
#include <stdint.h>

#include "timecode_oscillator.h"

// Timecode input is interpreted as relative speed in SYNTH input mode.
// Keep the legacy build flag name so existing build overrides remain valid.
#ifndef ENABLE_TIMECODE_OSCILLATOR
#define ENABLE_TIMECODE_OSCILLATOR 1
#endif

enum
{
    TIMECODE_SYNTH_CHANNEL_COUNT = 2u,
};

typedef enum
{
    TIMECODE_SYNTH_CONTROL_ROOT = 0,
    TIMECODE_SYNTH_CONTROL_MORPH,
    TIMECODE_SYNTH_CONTROL_SLOPE,
    TIMECODE_SYNTH_CONTROL_SMOOTH_FOLD,
    TIMECODE_SYNTH_CONTROL_WARP_AMOUNT,
    TIMECODE_SYNTH_CONTROL_COUNT,
} TimecodeSynthControl_t;

void timecode_synth_init(uint32_t sample_rate_hz);
void timecode_synth_reset_for_sample_rate(uint32_t sample_rate_hz);
void timecode_synth_update(void);

void timecode_synth_set_channel_enabled(uint32_t channel, bool enabled);
bool timecode_synth_is_channel_enabled(uint32_t channel);

// Frames use the JUMBLEQ layout CH1-L, CH1-R, CH2-L, CH2-R. The stride is
// supplied by audio_control so this module does not depend on SAI/DMA sizes.
void timecode_synth_process_input(const int32_t* input,
                                  uint32_t frame_stride_words,
                                  uint32_t frame_count);
void timecode_synth_render_output(int32_t* output,
                                  uint32_t frame_stride_words,
                                  uint32_t frame_count);

void timecode_synth_set_control(TimecodeSynthControl_t control, uint8_t value);
void timecode_synth_set_ratio_set(TimecodeOscillatorRatioSet_t ratio_set);
void timecode_synth_set_warp_algorithm(TimecodeOscillatorWarpAlgorithm_t warp_algorithm);
TimecodeOscillatorRatioSet_t timecode_synth_get_ratio_set(void);
TimecodeOscillatorWarpAlgorithm_t timecode_synth_get_warp_algorithm(void);

#endif /* INC_TIMECODE_SYNTH_H_ */
