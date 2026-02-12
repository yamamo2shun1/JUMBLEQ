#include "boot_meta.h"
#include "boot_config.h"
#include "crc.h"
#include "stm32_extmem.h"
#include "stm32_extmem_conf.h"
#include <string.h>

static bool boot_meta_read_raw(uint8_t *buf, uint32_t size)
{
  if (EXTMEM_OK != EXTMEM_Read(EXTMEM_MEMORY_BOOTXIP, BOOT_META_OFFSET, buf, size))
  {
    return false;
  }
  return true;
}

bool boot_meta_read(BootMetaHeader *out)
{
  uint8_t raw[BOOT_META_HEADER_SIZE];

  if (!boot_meta_read_raw(raw, sizeof(raw)))
  {
    return false;
  }

  memcpy(out, raw, sizeof(BootMetaHeader));
  return true;
}

bool boot_meta_write(const BootMetaHeader *in)
{
  uint8_t raw[BOOT_META_HEADER_SIZE];

  memset(raw, 0, sizeof(raw));
  memcpy(raw, in, sizeof(BootMetaHeader));

  if (EXTMEM_OK != EXTMEM_EraseSector(EXTMEM_MEMORY_BOOTXIP, BOOT_META_OFFSET, BOOT_META_ERASE_SIZE))
  {
    return false;
  }

  if (EXTMEM_OK != EXTMEM_Write(EXTMEM_MEMORY_BOOTXIP, BOOT_META_OFFSET, raw, sizeof(raw)))
  {
    return false;
  }

  return true;
}

bool boot_meta_is_valid(const BootMetaHeader *meta)
{
  if (meta == NULL)
  {
    return false;
  }
  if (meta->magic != BOOT_META_MAGIC)
  {
    return false;
  }
  if (meta->version != BOOT_META_VERSION)
  {
    return false;
  }
  if (meta->header_size != BOOT_META_HEADER_SIZE)
  {
    return false;
  }
  if ((meta->flags & BOOT_META_FLAG_VALID) == 0u)
  {
    return false;
  }
  if (meta->app_base != BOOT_APP_BASE)
  {
    return false;
  }
  if (meta->app_size > BOOT_APP_SIZE)
  {
    return false;
  }
  if (meta->image_size > BOOT_APP_SIZE)
  {
    return false;
  }
  return true;
}

uint32_t boot_crc32_extmem(uint32_t offset, uint32_t size)
{
  uint8_t buf[256];
  uint32_t remaining = size;
  uint32_t addr = offset;
  uint32_t crc = 0u;
  bool first = true;

  __HAL_CRC_DR_RESET(&hcrc);

  while (remaining > 0u)
  {
    uint32_t chunk = (remaining > sizeof(buf)) ? sizeof(buf) : remaining;
    uint32_t padded = (chunk + 3u) & ~3u;

    memset(buf, 0, sizeof(buf));

    if (EXTMEM_OK != EXTMEM_Read(EXTMEM_MEMORY_BOOTXIP, addr, buf, chunk))
    {
      return 0u;
    }

    if (first)
    {
      crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)buf, padded / 4u);
      first = false;
    }
    else
    {
      crc = HAL_CRC_Accumulate(&hcrc, (uint32_t *)buf, padded / 4u);
    }

    addr += chunk;
    remaining -= chunk;
  }

  return crc;
}
