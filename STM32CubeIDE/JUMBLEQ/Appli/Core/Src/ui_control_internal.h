/*
 * ui_control_internal.h
 *
 *  Created on: Feb 18, 2026
 */

#ifndef UI_CONTROL_INTERNAL_H_
#define UI_CONTROL_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7rsxx.h"
#include "stm32h7rsxx_hal_dma.h"
#include "ui_persist_internal.h"

// Runtime DSP parameter update switch for A/B diagnosis.
// 0: disable ui_control_task() DSP writes (noise root-cause test mode)
// 1: enable normal runtime control updates
#define ENABLE_DSP_RUNTIME_CONTROL 1

// persist: UI facadeがsnapshotを集約/検証/適用する。
void ui_control_get_persist_state(UI_ControlPersistState_t* state);
bool ui_control_validate_persist_state(const UI_ControlPersistState_t* state);
bool ui_control_apply_persist_state(const UI_ControlPersistState_t* state);

void ui_control_reset_state(void);
void ui_control_set_adc_complete(bool complete);
void ui_control_dma_adc_cplt(DMA_HandleTypeDef* hdma);

#endif /* UI_CONTROL_INTERNAL_H_ */
