#include "uf2.h"
#include "boot_config.h"
#include "boot_meta.h"
#include "stm32_extmem.h"
#include "stm32_extmem_conf.h"
#include <string.h>

#define UF2_MAGIC_START0 0x0A324655u
#define UF2_MAGIC_START1 0x9E5D5157u
#define UF2_MAGIC_END    0x0AB16F30u

typedef struct
{
  uint32_t magic0;
  uint32_t magic1;
  uint32_t flags;
  uint32_t target_addr;
  uint32_t payload_size;
  uint32_t block_no;
  uint32_t num_blocks;
  uint32_t file_size;
  uint8_t data[476];
  uint32_t magic_end;
} Uf2Block;

typedef struct
{
  uint32_t expected_blocks;
  uint32_t received_blocks;
  uint32_t payload_size;
  uint32_t min_addr;
  uint32_t max_addr;
  uint8_t block_bitmap[BOOT_APP_SIZE / BOOT_UF2_PAYLOAD_SIZE / 8u];
  uint64_t erased_bitmap;
} Uf2Context;

static Uf2Context g_uf2;

static bool uf2_block_valid(const Uf2Block *b)
{
  if (b->magic0 != UF2_MAGIC_START0 || b->magic1 != UF2_MAGIC_START1 || b->magic_end != UF2_MAGIC_END)
  {
    return false;
  }
  if (b->payload_size == 0u || b->payload_size > BOOT_UF2_PAYLOAD_SIZE)
  {
    return false;
  }
  return true;
}

static bool uf2_addr_in_range(uint32_t addr, uint32_t size)
{
  if (addr < BOOT_APP_BASE)
  {
    return false;
  }
  if ((addr + size) > (BOOT_APP_BASE + BOOT_APP_SIZE))
  {
    return false;
  }
  return true;
}

static void uf2_mark_block(uint32_t index)
{
  g_uf2.block_bitmap[index / 8u] |= (uint8_t)(1u << (index % 8u));
}

static bool uf2_is_block_marked(uint32_t index)
{
  return (g_uf2.block_bitmap[index / 8u] & (uint8_t)(1u << (index % 8u))) != 0u;
}

static bool uf2_erase_if_needed(uint32_t addr_offset)
{
  uint32_t block_index = addr_offset / BOOT_ERASE_BLOCK_SIZE;
  uint64_t mask = (1ull << block_index);

  if ((g_uf2.erased_bitmap & mask) != 0ull)
  {
    return true;
  }

  if (EXTMEM_OK != EXTMEM_EraseSector(EXTMEM_MEMORY_BOOTXIP, BOOT_APP_OFFSET + block_index * BOOT_ERASE_BLOCK_SIZE,
                                      BOOT_ERASE_BLOCK_SIZE))
  {
    return false;
  }

  g_uf2.erased_bitmap |= mask;
  return true;
}

void uf2_init(void)
{
  memset(&g_uf2, 0, sizeof(g_uf2));
  g_uf2.min_addr = 0xFFFFFFFFu;
}

bool uf2_process_block(const uint8_t *block512)
{
  Uf2Block b;
  uint32_t addr_offset;
  uint32_t block_index;

  memcpy(&b, block512, sizeof(Uf2Block));

  if (!uf2_block_valid(&b))
  {
    return false;
  }

  if (!uf2_addr_in_range(b.target_addr, b.payload_size))
  {
    return false;
  }

  if (g_uf2.expected_blocks == 0u)
  {
    g_uf2.expected_blocks = b.num_blocks;
    g_uf2.payload_size = b.payload_size;
  }

  if (b.payload_size != g_uf2.payload_size)
  {
    return false;
  }

  addr_offset = b.target_addr - BOOT_APP_BASE;
  block_index = addr_offset / g_uf2.payload_size;

  if (uf2_is_block_marked(block_index))
  {
    return true;
  }

  if (!uf2_erase_if_needed(addr_offset))
  {
    return false;
  }

  if (EXTMEM_OK != EXTMEM_Write(EXTMEM_MEMORY_BOOTXIP, BOOT_APP_OFFSET + addr_offset, b.data, b.payload_size))
  {
    return false;
  }

  uf2_mark_block(block_index);
  g_uf2.received_blocks++;

  if (b.target_addr < g_uf2.min_addr)
  {
    g_uf2.min_addr = b.target_addr;
  }
  if ((b.target_addr + b.payload_size) > g_uf2.max_addr)
  {
    g_uf2.max_addr = b.target_addr + b.payload_size;
  }

  return true;
}

bool uf2_is_complete(void)
{
  if (g_uf2.expected_blocks == 0u)
  {
    return false;
  }
  return (g_uf2.received_blocks >= g_uf2.expected_blocks);
}

uint32_t uf2_received_blocks(void)
{
  return g_uf2.received_blocks;
}

uint32_t uf2_expected_blocks(void)
{
  return g_uf2.expected_blocks;
}

uint32_t uf2_image_size(void)
{
  if (g_uf2.max_addr <= BOOT_APP_BASE)
  {
    return 0u;
  }
  return g_uf2.max_addr - BOOT_APP_BASE;
}

bool uf2_finalize(void)
{
  BootMetaHeader meta;
  uint32_t image_size = uf2_image_size();
  uint32_t crc;

  if (image_size == 0u)
  {
    return false;
  }

  crc = boot_crc32_extmem(BOOT_APP_OFFSET, image_size);
  if (crc == 0u)
  {
    return false;
  }

  memset(&meta, 0, sizeof(meta));
  meta.magic = BOOT_META_MAGIC;
  meta.version = BOOT_META_VERSION;
  meta.header_size = BOOT_META_HEADER_SIZE;
  meta.flags = BOOT_META_FLAG_VALID;
  meta.app_base = BOOT_APP_BASE;
  meta.app_size = BOOT_APP_SIZE;
  meta.app_crc32 = crc;
  meta.build_id = 0u;
  meta.image_size = image_size;

  return boot_meta_write(&meta);
}
