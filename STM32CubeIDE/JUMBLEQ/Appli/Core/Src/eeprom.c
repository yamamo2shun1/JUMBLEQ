#include "eeprom.h"

#include <stddef.h>
#include <string.h>

#include "ui_control.h"

typedef struct
{
    uint32_t             magic;
    uint16_t             version;
    uint16_t             payload_size;
    EEPROM_DeviceConfig_t payload;
    uint32_t             crc32;
} EEPROM_ConfigRecord_t;

static uint32_t EEPROM_CRC32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0U; j < 8U; j++)
        {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1UL));
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }

    return ~crc;
}

static HAL_StatusTypeDef EEPROM_CheckRange(uint16_t mem_addr, uint16_t len)
{
    uint32_t end_addr = (uint32_t)mem_addr + (uint32_t)len;

    if (end_addr > EEPROM_TOTAL_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void EEPROM_ConfigSetDefaults(EEPROM_DeviceConfig_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->current_ch1_input_type = 0U; /* INPUT_TYPE_LINE */
    cfg->current_ch2_input_type = 0U; /* INPUT_TYPE_LINE */
    cfg->current_xfA_assign     = 2U; /* INPUT_SRC_CH2_LN */
    cfg->current_xfB_assign     = 0U; /* INPUT_SRC_CH1_LN */
    cfg->current_xfpost_assign  = 4U; /* INPUT_SRC_USB */
    cfg->reserved[0]    = 0U;
    cfg->reserved[1]    = 0U;
    cfg->reserved[2]    = 0U;
}

void EEPROM_ConfigCaptureCurrent(EEPROM_DeviceConfig_t *cfg)
{
    UI_ControlPersistState_t state;

    if (cfg == NULL)
    {
        return;
    }

    ui_control_get_persist_state(&state);
    cfg->current_ch1_input_type = state.current_ch1_input_type;
    cfg->current_ch2_input_type = state.current_ch2_input_type;
    cfg->current_xfA_assign     = state.current_xfA_assign;
    cfg->current_xfB_assign     = state.current_xfB_assign;
    cfg->current_xfpost_assign  = state.current_xfpost_assign;
    cfg->reserved[0]            = 0U;
    cfg->reserved[1]            = 0U;
    cfg->reserved[2]            = 0U;
}

HAL_StatusTypeDef EEPROM_CheckConnection(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_IsDeviceReady(hi2c,
                                  EEPROM_I2C_ADDR_8BIT,
                                  EEPROM_READY_TRIALS_DEFAULT,
                                  EEPROM_READY_TIMEOUT_MS);
}

HAL_StatusTypeDef EEPROM_WaitReady(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms)
{
    uint32_t start_tick;
    HAL_StatusTypeDef status;

    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    start_tick = HAL_GetTick();
    do
    {
        status = HAL_I2C_IsDeviceReady(hi2c, EEPROM_I2C_ADDR_8BIT, 1U, EEPROM_READY_TIMEOUT_MS);
        if (status == HAL_OK)
        {
            return HAL_OK;
        }
    } while ((HAL_GetTick() - start_tick) < timeout_ms);

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef EEPROM_Read(I2C_HandleTypeDef *hi2c, uint16_t mem_addr, uint8_t *buf, uint16_t len)
{
    if ((hi2c == NULL) || (buf == NULL))
    {
        return HAL_ERROR;
    }

    if (len == 0U)
    {
        return HAL_OK;
    }

    if (EEPROM_CheckRange(mem_addr, len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(hi2c,
                            EEPROM_I2C_ADDR_8BIT,
                            mem_addr,
                            I2C_MEMADD_SIZE_16BIT,
                            buf,
                            len,
                            EEPROM_XFER_TIMEOUT_MS);
}

HAL_StatusTypeDef EEPROM_Write(I2C_HandleTypeDef *hi2c, uint16_t mem_addr, const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint16_t current_addr = mem_addr;
    uint16_t remain = len;
    const uint8_t *current_buf = buf;

    if ((hi2c == NULL) || (buf == NULL))
    {
        return HAL_ERROR;
    }

    if (len == 0U)
    {
        return HAL_OK;
    }

    if (EEPROM_CheckRange(mem_addr, len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    while (remain > 0U)
    {
        uint16_t page_offset = (uint16_t)(current_addr % EEPROM_PAGE_SIZE_BYTES);
        uint16_t page_space = (uint16_t)(EEPROM_PAGE_SIZE_BYTES - page_offset);
        uint16_t chunk = (remain < page_space) ? remain : page_space;

        status = HAL_I2C_Mem_Write(hi2c,
                                   EEPROM_I2C_ADDR_8BIT,
                                   current_addr,
                                   I2C_MEMADD_SIZE_16BIT,
                                   (uint8_t *)current_buf,
                                   chunk,
                                   EEPROM_XFER_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            return status;
        }

        status = EEPROM_WaitReady(hi2c, EEPROM_WRITE_CYCLE_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            return status;
        }

        current_addr = (uint16_t)(current_addr + chunk);
        current_buf += chunk;
        remain = (uint16_t)(remain - chunk);
    }

    return HAL_OK;
}

HAL_StatusTypeDef EEPROM_SaveConfig(I2C_HandleTypeDef *hi2c, const EEPROM_DeviceConfig_t *cfg)
{
    EEPROM_ConfigRecord_t rec;
    uint32_t crc_input_len;

    if ((hi2c == NULL) || (cfg == NULL))
    {
        return HAL_ERROR;
    }

    rec.magic        = EEPROM_CONFIG_MAGIC;
    rec.version      = EEPROM_CONFIG_VERSION;
    rec.payload_size = (uint16_t)sizeof(EEPROM_DeviceConfig_t);
    rec.payload      = *cfg;

    crc_input_len = (uint32_t)offsetof(EEPROM_ConfigRecord_t, crc32);
    rec.crc32 = EEPROM_CRC32((const uint8_t *)&rec, crc_input_len);

    return EEPROM_Write(hi2c, EEPROM_CONFIG_ADDR, (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

HAL_StatusTypeDef EEPROM_LoadConfig(I2C_HandleTypeDef *hi2c, EEPROM_DeviceConfig_t *cfg)
{
    EEPROM_ConfigRecord_t rec;
    uint32_t expected_crc;
    HAL_StatusTypeDef status;

    if ((hi2c == NULL) || (cfg == NULL))
    {
        return HAL_ERROR;
    }

    status = EEPROM_Read(hi2c, EEPROM_CONFIG_ADDR, (uint8_t *)&rec, (uint16_t)sizeof(rec));
    if (status != HAL_OK)
    {
        return status;
    }

    if ((rec.magic != EEPROM_CONFIG_MAGIC) ||
        (rec.version != EEPROM_CONFIG_VERSION) ||
        (rec.payload_size != (uint16_t)sizeof(EEPROM_DeviceConfig_t)))
    {
        return HAL_ERROR;
    }

    expected_crc = EEPROM_CRC32((const uint8_t *)&rec, (uint32_t)offsetof(EEPROM_ConfigRecord_t, crc32));
    if (expected_crc != rec.crc32)
    {
        return HAL_ERROR;
    }

    memcpy(cfg, &rec.payload, sizeof(*cfg));
    return HAL_OK;
}
