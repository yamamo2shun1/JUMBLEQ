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
    uint8_t current_xfA_assign;
    uint8_t current_xfB_assign;
    uint8_t current_xfpost_assign;
    uint8_t reserved[3];
} EEPROM_DeviceConfig_t;

#define EEPROM_CONFIG_ADDR               (0x0000U)
#define EEPROM_CONFIG_MAGIC              (0x51424D4AU) /* "JMBQ" */
#define EEPROM_CONFIG_VERSION            (0x0001U)

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
