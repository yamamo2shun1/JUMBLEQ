/*
 * ui_control.h
 *
 *  Created on: Feb 18, 2026
 */

#ifndef INC_UI_CONTROL_H_
#define INC_UI_CONTROL_H_

#include "main.h"

uint8_t get_current_xfA_position(void);
uint8_t get_current_xfB_position(void);
int16_t get_current_ch1_db(void);
int16_t get_current_ch2_db(void);
int16_t get_current_master_db(void);
int16_t get_current_dry_wet(void);
char* get_current_input_typeA_str(void);
char* get_current_input_typeB_str(void);
char* get_current_input_srcA_str(void);
char* get_current_input_srcB_str(void);
char* get_current_input_srcP_str(void);

void start_adc(void);
void ui_control_task(void);
void start_audio_control(void);
bool is_started_audio_control(void);

#endif /* INC_UI_CONTROL_H_ */
