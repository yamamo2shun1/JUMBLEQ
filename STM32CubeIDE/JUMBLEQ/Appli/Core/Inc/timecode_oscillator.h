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
} TimecodeOscillatorRatioSet_t;

typedef enum
{
    TIMECODE_WARP_CLEAN = 0,
    TIMECODE_WARP_CROSSFOLD,
    TIMECODE_WARP_RING_MOD,
    TIMECODE_WARP_COMPARATOR,
} TimecodeOscillatorWarpAlgorithm_t;

typedef enum
{
    TIMECODE_TRACK_NO_SIGNAL = 0,
    TIMECODE_TRACK_TRACKING,
    TIMECODE_TRACK_HOLD,
} TimecodeOscillatorTrackState_t;

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
    TimecodeOscillatorRatioSet_t ratio_set;
    float slope;
    float smooth_fold;
    TimecodeOscillatorWarpAlgorithm_t warp_algorithm;
    float warp_amount;
    float direction;
    float output_gain_db;
} TimecodeOscillatorParameters_t;

typedef struct
{
    float signed_speed;
    float confidence;
    float input_level;
    TimecodeOscillatorTrackState_t state;
} TimecodeOscillatorDiagnostics_t;

typedef struct
{
    TimecodeOscillatorParameters_t parameters;
    TimecodeOscillatorDiagnostics_t diagnostics;

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
} TimecodeOscillator_t;

void timecode_oscillator_init(TimecodeOscillator_t* oscillator, uint32_t sample_rate_hz);
void timecode_oscillator_reset(TimecodeOscillator_t* oscillator);
void timecode_oscillator_set_enabled(TimecodeOscillator_t* oscillator, bool enabled);
void timecode_oscillator_set_parameters(TimecodeOscillator_t* oscillator,
                                        const TimecodeOscillatorParameters_t* parameters);
const TimecodeOscillatorParameters_t* timecode_oscillator_get_parameters(const TimecodeOscillator_t* oscillator);
const TimecodeOscillatorDiagnostics_t* timecode_oscillator_get_diagnostics(const TimecodeOscillator_t* oscillator);

void timecode_oscillator_process_interleaved(TimecodeOscillator_t* oscillator,
                                             const int32_t* input_l,
                                             const int32_t* input_r,
                                             uint32_t input_stride_words,
                                             int32_t* output,
                                             uint32_t frame_count);

#endif /* INC_TIMECODE_OSCILLATOR_H_ */
