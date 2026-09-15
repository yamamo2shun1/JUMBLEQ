/*
 * ui_control_internal.h
 *
 *  Created on: Feb 18, 2026
 */

#ifndef UI_CONTROL_INTERNAL_H_
#define UI_CONTROL_INTERNAL_H_

#include "main.h"
#include "ui_control.h"

// 非破壊のpersist検証。DSP/codecへ書き込まず、UI状態も変更しない。
bool ui_control_validate_persist_state(const UI_ControlPersistState_t* state);

void ui_control_reset_state(void);
void ui_control_set_adc_complete(bool complete);
void ui_control_dma_adc_cplt(DMA_HandleTypeDef* hdma);
void ui_control_reapply_ch_fader_outputs(void);
void ui_control_reapply_pot_outputs(void);

#endif /* UI_CONTROL_INTERNAL_H_ */
