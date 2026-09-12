#ifndef __EEPROM_H__
#define __EEPROM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define EEPROM_I2C_ADDR_7BIT          (0x50U)
#define EEPROM_I2C_ADDR_8BIT          (EEPROM_I2C_ADDR_7BIT << 1)
#define EEPROM_READY_TRIALS_DEFAULT   (3U)
#define EEPROM_READY_TIMEOUT_MS       (100U)
#define EEPROM_XFER_TIMEOUT_MS        (100U)
#define EEPROM_WRITE_CYCLE_TIMEOUT_MS (20U)
#define EEPROM_PAGE_SIZE_BYTES        (128U)
#define EEPROM_TOTAL_SIZE_BYTES       (65536U)

typedef struct
{
    uint8_t current_ch1_input_type;
    uint8_t current_ch2_input_type;
    uint8_t current_ch_fader_a_assign;
    uint8_t current_ch_fader_b_assign;
    uint8_t current_ch_fader_post_assign;
    uint8_t current_return_assign;
    uint8_t current_hp_out_source;
    uint8_t current_ch1_input_mode;
    uint8_t current_ch2_input_mode;
    uint8_t ch_fader_dvs_delay_ms;
    uint8_t mag_output_mode_flags;
    float current_ch_fader_curve_width_a;
    float current_ch_fader_curve_width_b;
    uint8_t sensor2_aux_fade_down_assign;
    uint8_t sensor3_aux_fade_down_assign;
    uint8_t ch_fader_reverse_flags;
    uint8_t timecode_synth_ratio_set;
    uint8_t timecode_synth_warp_algorithm;
} EEPROM_DeviceConfig_t;

#define EEPROM_CONFIG_ADDR               (0x0000U)
#define EEPROM_CONFIG_MAGIC              (0x51424D4AU) /* "JMBQ" */
#define EEPROM_CONFIG_VERSION            (0x0009U)
#define EEPROM_CONFIG_VERSION_V8         (0x0008U)
#define EEPROM_CONFIG_VERSION_V7         (0x0007U)

#define EEPROM_CFG_FLAG_MAG_OUT_AS_NOTE   (0x01U)
#define EEPROM_CFG_FLAG_CH_FADER_REVERSE_A (0x01U)
#define EEPROM_CFG_FLAG_CH_FADER_REVERSE_B (0x02U)
HAL_StatusTypeDef EEPROM_CheckConnection(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef EEPROM_WaitReady(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms);
HAL_StatusTypeDef EEPROM_Read(I2C_HandleTypeDef *hi2c, uint16_t mem_addr, uint8_t *buf, uint16_t len);
HAL_StatusTypeDef EEPROM_Write(I2C_HandleTypeDef *hi2c, uint16_t mem_addr, const uint8_t *buf, uint16_t len);
void EEPROM_ConfigSetDefaults(EEPROM_DeviceConfig_t *cfg);
void EEPROM_ConfigCaptureCurrent(EEPROM_DeviceConfig_t *cfg);
HAL_StatusTypeDef EEPROM_SaveConfig(I2C_HandleTypeDef *hi2c, const EEPROM_DeviceConfig_t *cfg);
HAL_StatusTypeDef EEPROM_LoadConfig(I2C_HandleTypeDef *hi2c, EEPROM_DeviceConfig_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H__ */
