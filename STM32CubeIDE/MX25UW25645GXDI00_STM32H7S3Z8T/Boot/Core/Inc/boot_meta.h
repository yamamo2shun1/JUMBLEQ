#ifndef BOOT_META_H
#define BOOT_META_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t header_size;
  uint32_t flags;
  uint32_t app_base;
  uint32_t app_size;
  uint32_t app_crc32;
  uint32_t build_id;
  uint32_t image_size;
  uint32_t reserved[6];
} BootMetaHeader;

#define BOOT_META_MAGIC 0x4D544131u
#define BOOT_META_VERSION 0x0001u
#define BOOT_META_FLAG_VALID (1u << 0)

bool boot_meta_read(BootMetaHeader *out);
bool boot_meta_write(const BootMetaHeader *in);
bool boot_meta_is_valid(const BootMetaHeader *meta);

uint32_t boot_crc32_extmem(uint32_t offset, uint32_t size);

#endif /* BOOT_META_H */
