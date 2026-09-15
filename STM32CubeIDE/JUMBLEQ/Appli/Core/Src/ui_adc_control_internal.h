/*
 * ui_adc_control_internal.h
 *
 * Private API of the UI ADC/HPDMA module. The DMA destination symbol adc_val
 * keeps its name, section and alignment for linked_list.c.
 */

#ifndef UI_ADC_CONTROL_INTERNAL_H_
#define UI_ADC_CONTROL_INTERNAL_H_

#include "main.h"

// 完了flagは処理前にclearせず、pot/ch_fader/MIDI処理後にclearする。
bool ui_adc_control_is_complete(void);
void ui_adc_control_clear_complete(void);

// DMAが書き込むADCサンプルへのread-only view。
const uint32_t* ui_adc_control_samples(void);

// 起動時の初期状態へ戻す（adc_valと完了flag）。
void ui_adc_control_reset(void);

#endif /* UI_ADC_CONTROL_INTERNAL_H_ */
