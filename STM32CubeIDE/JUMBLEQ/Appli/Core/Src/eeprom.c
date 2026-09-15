/*
 * eeprom.c
 *
 * Low-level I2C EEPROM access: connection check, ready wait, address range
 * check, page-split read/write. Product settings live in eeprom_config.c.
 */

#include "eeprom.h"

static HAL_StatusTypeDef EEPROM_CheckRange(uint16_t mem_addr, uint16_t len)
{
    uint32_t end_addr = (uint32_t) mem_addr + (uint32_t) len;

    if (end_addr > EEPROM_TOTAL_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef EEPROM_CheckConnection(I2C_HandleTypeDef* hi2c)
{
    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_IsDeviceReady(hi2c, EEPROM_I2C_ADDR_8BIT, EEPROM_READY_TRIALS_DEFAULT, EEPROM_READY_TIMEOUT_MS);
}

HAL_StatusTypeDef EEPROM_WaitReady(I2C_HandleTypeDef* hi2c, uint32_t timeout_ms)
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

HAL_StatusTypeDef EEPROM_Read(I2C_HandleTypeDef* hi2c, uint16_t mem_addr, uint8_t* buf, uint16_t len)
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

    return HAL_I2C_Mem_Read(hi2c, EEPROM_I2C_ADDR_8BIT, mem_addr, I2C_MEMADD_SIZE_16BIT, buf, len, EEPROM_XFER_TIMEOUT_MS);
}

HAL_StatusTypeDef EEPROM_Write(I2C_HandleTypeDef* hi2c, uint16_t mem_addr, const uint8_t* buf, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint16_t current_addr      = mem_addr;
    uint16_t remain            = len;
    const uint8_t* current_buf = buf;

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
        uint16_t page_offset = (uint16_t) (current_addr % EEPROM_PAGE_SIZE_BYTES);
        uint16_t page_space  = (uint16_t) (EEPROM_PAGE_SIZE_BYTES - page_offset);
        uint16_t chunk       = (remain < page_space) ? remain : page_space;

        status = HAL_I2C_Mem_Write(hi2c, EEPROM_I2C_ADDR_8BIT, current_addr, I2C_MEMADD_SIZE_16BIT, (uint8_t*) current_buf, chunk, EEPROM_XFER_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            return status;
        }

        status = EEPROM_WaitReady(hi2c, EEPROM_WRITE_CYCLE_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            return status;
        }

        current_addr = (uint16_t) (current_addr + chunk);
        current_buf += chunk;
        remain = (uint16_t) (remain - chunk);
    }

    return HAL_OK;
}
