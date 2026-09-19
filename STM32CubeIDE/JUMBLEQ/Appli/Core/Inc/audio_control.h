/*
 * audio_control.h
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#ifndef INC_AUDIO_CONTROL_H_
#define INC_AUDIO_CONTROL_H_

#include <stdint.h>

uint32_t audio_control_requested_sample_rate_hz(void);
void audio_control_reset_runtime_state(void);
void audio_control_load_config_or_restore_defaults(void);

void start_sai(void);

void audio_control_register_task(void);
void audio_task(void);

#endif /* INC_AUDIO_CONTROL_H_ */
