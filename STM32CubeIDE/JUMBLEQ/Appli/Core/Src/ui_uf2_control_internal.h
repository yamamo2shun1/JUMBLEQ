/*
 * ui_uf2_control_internal.h
 *
 * Private API of the UF2 bootloader transition state machine.
 */

#ifndef UI_UF2_CONTROL_INTERNAL_H_
#define UI_UF2_CONTROL_INTERNAL_H_

#include "main.h"

// UF2移行要求をarmする。program_changeはRTTログ用のPC番号。
void ui_uf2_control_arm(uint8_t program_change);

// arm中のUF2移行要求を取り消してnotice状態へ移す。
void ui_uf2_control_cancel(const char* reason);

// ADC完了とは独立に、2ms Task周期で呼ばれる状態機械service。
void ui_uf2_control_service(void);

// 起動時の初期状態へ戻す。
void ui_uf2_control_reset(void);

#endif /* UI_UF2_CONTROL_INTERNAL_H_ */
