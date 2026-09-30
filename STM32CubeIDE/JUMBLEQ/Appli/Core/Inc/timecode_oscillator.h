/*
 * timecode_oscillator.h
 *
 * Relative-motion timecode tracker and oscillator.
 */

#ifndef INC_TIMECODE_OSCILLATOR_H_
#define INC_TIMECODE_OSCILLATOR_H_

#include <stdbool.h>
#include <stdint.h>

#define TIMECODE_OSCILLATOR_VOICE_COUNT      4U
#define TIMECODE_OSCILLATOR_MAX_BLOCK_FRAMES 32U

typedef enum
{
    TIMECODE_RATIO_OCTAVE = 0,
    TIMECODE_RATIO_HARMONIC,
    TIMECODE_RATIO_CHORD,
} timecode_oscillator_ratio_set_t;

typedef enum
{
    TIMECODE_WARP_CLEAN = 0,
    TIMECODE_WARP_CROSSFOLD,
    TIMECODE_WARP_RING_MOD,
    TIMECODE_WARP_COMPARATOR,
} timecode_oscillator_warp_algorithm_t;

typedef enum
{
    TIMECODE_TRACK_NO_SIGNAL = 0,
    TIMECODE_TRACK_TRACKING,
    TIMECODE_TRACK_HOLD,
} timecode_oscillator_track_state_t;

typedef struct
{
    float root_hz;
    float nominal_carrier_hz;
    float smooth_ms;
    float hold_ms;
    float release_ms;
    float threshold;
    float confidence_min;
    float morph;
    timecode_oscillator_ratio_set_t ratio_set;
    float slope;
    float smooth_fold;
    timecode_oscillator_warp_algorithm_t warp_algorithm;
    float warp_amount;
    float direction;
    float output_gain_db;
} timecode_oscillator_parameters_t;

typedef struct
{
    float signed_speed;
    float confidence;
    float input_level;
    timecode_oscillator_track_state_t state;
} timecode_oscillator_diagnostics_t;

typedef struct
{
    timecode_oscillator_parameters_t parameters;
    timecode_oscillator_diagnostics_t diagnostics;

    float sample_rate_hz;
    float dc_coefficient;
    float tracker_coefficient;
    float shape_coefficient;
    float gate_attack_coefficient;
    float gate_release_coefficient;
    float nominal_phase_step;
    float threshold_squared;
    float output_gain;
    uint32_t hold_samples;
    bool enabled;

    float previous_l_input;
    float previous_r_input;
    float previous_l_dc;
    float previous_r_dc;
    float previous_l;
    float previous_r;
    float cross_smooth;
    float dot_smooth;
    float energy_smooth;
    float speed_memory;
    uint32_t hold_left_samples;
    float gate;

    float phase[TIMECODE_OSCILLATOR_VOICE_COUNT];
    float morph_smooth;
    float slope_smooth;
    float smooth_fold_smooth;
    float voice_lp_a[TIMECODE_OSCILLATOR_VOICE_COUNT];
    float voice_lp_b[TIMECODE_OSCILLATOR_VOICE_COUNT];
} timecode_oscillator_t;

void timecode_oscillator_init(timecode_oscillator_t* oscillator, uint32_t sample_rate_hz);
void timecode_oscillator_reset(timecode_oscillator_t* oscillator);
void timecode_oscillator_set_enabled(timecode_oscillator_t* oscillator, bool enabled);
void timecode_oscillator_set_parameters(timecode_oscillator_t* oscillator,
                                        const timecode_oscillator_parameters_t* parameters);
const timecode_oscillator_parameters_t* timecode_oscillator_get_parameters(const timecode_oscillator_t* oscillator);
const timecode_oscillator_diagnostics_t* timecode_oscillator_get_diagnostics(const timecode_oscillator_t* oscillator);

void timecode_oscillator_process_interleaved(timecode_oscillator_t* oscillator,
                                             const int32_t* input_l,
                                             const int32_t* input_r,
                                             uint32_t input_stride_words,
                                             int32_t* output,
                                             uint32_t frame_count);

#endif /* INC_TIMECODE_OSCILLATOR_H_ */
