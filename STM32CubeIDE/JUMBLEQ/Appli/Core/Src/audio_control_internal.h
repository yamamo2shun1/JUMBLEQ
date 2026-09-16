/*
 * audio_control_internal.h
 *
 * Private API of audio_control.c for the Core audio modules.
 */

#ifndef AUDIO_CONTROL_INTERNAL_H_
#define AUDIO_CONTROL_INTERNAL_H_

#include <stdint.h>

// UAC2 clock SET_CUR entry point. The USB control module validates the
// requested rate against its supported list and hands the request over here.
// Only the Audio Task applies the change; the control callback must not
// reinitialize SAI/GPDMA or the DSP path.
void audio_control_request_sample_rate(uint32_t sample_rate_hz);

#endif /* AUDIO_CONTROL_INTERNAL_H_ */
