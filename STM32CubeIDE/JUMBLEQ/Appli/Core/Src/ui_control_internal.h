/*
 * ui_control_internal.h
 *
 *  Created on: Feb 18, 2026
 */

#ifndef UI_CONTROL_INTERNAL_H_
#define UI_CONTROL_INTERNAL_H_

#include "main.h"

void ui_control_reset_state(void);
void ui_control_set_adc_complete(bool complete);
void ui_control_dma_adc_cplt(DMA_HandleTypeDef* hdma);

#endif /* UI_CONTROL_INTERNAL_H_ */
