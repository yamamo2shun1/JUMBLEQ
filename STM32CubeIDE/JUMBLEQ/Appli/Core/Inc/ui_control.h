/*
 * ui_control.h
 *
 *  Created on: Feb 18, 2026
 */

#ifndef INC_UI_CONTROL_H_
#define INC_UI_CONTROL_H_

#include "main.h"
#include <stdbool.h>

typedef struct
{
    uint8_t current_ch1_input_type;
    uint8_t current_ch2_input_type;
    uint8_t current_xfA_assign;
    uint8_t current_xfB_assign;
    uint8_t current_xfpost_assign;
    uint8_t current_ch1_dvs_enable;
    uint8_t current_ch2_dvs_enable;
} UI_ControlPersistState_t;

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
uint8_t get_current_input_srcA_channel(void);  // 0:none, 1:CH1, 2:CH2
uint8_t get_current_input_srcB_channel(void);  // 0:none, 1:CH1, 2:CH2
bool get_current_ch1_dvs_enabled(void);
bool get_current_ch2_dvs_enabled(void);

void start_adc(void);
void ui_control_task(void);
void start_audio_control(void);
bool is_started_audio_control(void);
void ui_control_get_persist_state(UI_ControlPersistState_t *state);
bool ui_control_apply_persist_state(const UI_ControlPersistState_t *state);

#endif /* INC_UI_CONTROL_H_ */
