/*
 * eeprom_config_internal.h
 *
 * Private API of the EEPROM configuration module.
 */

#ifndef EEPROM_CONFIG_INTERNAL_H_
#define EEPROM_CONFIG_INTERNAL_H_

#include "main.h"
#include "eeprom.h"
#include "ui_control.h"

// EEPROM設定全体のsemantic validation。DSP/codec/UI/Timecodeへ書き込まない。
bool eeprom_config_validate(const EEPROM_DeviceConfig_t* cfg);

// 検証済み設定をUIとTimecode Synthへ適用する。
bool eeprom_config_apply(const EEPROM_DeviceConfig_t* cfg);

// EEPROM表現からUI persist stateへの変換（書き込みなし）。
bool eeprom_config_to_ui_persist_state(const EEPROM_DeviceConfig_t* cfg,
                                       UI_ControlPersistState_t* ui_state);

// read-only flag decode。
bool eeprom_config_is_mag_out_as_note(const EEPROM_DeviceConfig_t* cfg);
bool eeprom_config_is_ch_fader_reverse_a_enabled(const EEPROM_DeviceConfig_t* cfg);
bool eeprom_config_is_ch_fader_reverse_b_enabled(const EEPROM_DeviceConfig_t* cfg);

#endif /* EEPROM_CONFIG_INTERNAL_H_ */
