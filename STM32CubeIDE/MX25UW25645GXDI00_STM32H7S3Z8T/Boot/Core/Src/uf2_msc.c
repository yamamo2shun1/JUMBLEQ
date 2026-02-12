#include "uf2_msc.h"
#include "uf2.h"
#include "boot_config.h"
#include "tusb.h"
#include <string.h>
#include <stdio.h>

// Disk geometry (FAT16)
#define UF2_SECTOR_SIZE           512u
#define UF2_SECTORS_PER_CLUSTER   8u
#define UF2_RESERVED_SECTORS      1u
#define UF2_FAT_COUNT             2u
#define UF2_ROOT_ENTRIES          128u
#define UF2_ROOT_DIR_SECTORS      ((UF2_ROOT_ENTRIES * 32u + UF2_SECTOR_SIZE - 1u) / UF2_SECTOR_SIZE)
#define UF2_TOTAL_SECTORS         (BOOT_UF2_DISK_SIZE_BYTES / UF2_SECTOR_SIZE)
#define UF2_FAT_SECTORS           16u
#define UF2_FAT_START             (UF2_RESERVED_SECTORS)
#define UF2_FAT2_START            (UF2_FAT_START + UF2_FAT_SECTORS)
#define UF2_ROOT_DIR_START        (UF2_FAT_START + (UF2_FAT_COUNT * UF2_FAT_SECTORS))
#define UF2_DATA_START            (UF2_ROOT_DIR_START + UF2_ROOT_DIR_SECTORS)

#define UF2_INFO_CLUSTER          2u
#define UF2_INDEX_CLUSTER         3u

static const char info_txt[] =
    "UF2 Bootloader\r\n"
    "Model: MX25UW25645GXDI00_STM32H7S3Z8T\r\n"
    "Board-ID: STM32H7S3Z8T\r\n";

static const char index_html[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<title>UF2BOOT</title></head><body>"
    "<h3>UF2BOOT</h3>"
    "<p>Drag and drop a UF2 file to update the application.</p>"
    "</body></html>\r\n";

static void uf2_write_u16(uint8_t* buf, uint32_t offset, uint16_t value)
{
  buf[offset + 0u] = (uint8_t)(value & 0xFFu);
  buf[offset + 1u] = (uint8_t)((value >> 8) & 0xFFu);
}

static void uf2_write_u32(uint8_t* buf, uint32_t offset, uint32_t value)
{
  buf[offset + 0u] = (uint8_t)(value & 0xFFu);
  buf[offset + 1u] = (uint8_t)((value >> 8) & 0xFFu);
  buf[offset + 2u] = (uint8_t)((value >> 16) & 0xFFu);
  buf[offset + 3u] = (uint8_t)((value >> 24) & 0xFFu);
}

static void uf2_fill_padded_name(char* dst, size_t len, const char* src)
{
  size_t i = 0;
  for (; i < len && src[i] != '\0'; i++)
  {
    dst[i] = src[i];
  }
  for (; i < len; i++)
  {
    dst[i] = ' ';
  }
}

static void uf2_write_dir_entry(uint8_t* dir, uint32_t entry_index, const char* name8, const char* ext3,
                                uint8_t attr, uint16_t cluster, uint32_t size)
{
  uint32_t offset = entry_index * 32u;
  uf2_fill_padded_name((char*)&dir[offset], 8u, name8);
  uf2_fill_padded_name((char*)&dir[offset + 8u], 3u, ext3);
  dir[offset + 11u] = attr;
  uf2_write_u16(dir, offset + 26u, cluster);
  uf2_write_u32(dir, offset + 28u, size);
}

static void uf2_read_boot_sector(uint8_t* buf)
{
  memset(buf, 0, UF2_SECTOR_SIZE);
  buf[0x00] = 0xEB;
  buf[0x01] = 0x3C;
  buf[0x02] = 0x90;
  memcpy(&buf[0x03], "MSDOS5.0", 8);
  uf2_write_u16(buf, 0x0B, UF2_SECTOR_SIZE);
  buf[0x0D] = (uint8_t)UF2_SECTORS_PER_CLUSTER;
  uf2_write_u16(buf, 0x0E, UF2_RESERVED_SECTORS);
  buf[0x10] = (uint8_t)UF2_FAT_COUNT;
  uf2_write_u16(buf, 0x11, UF2_ROOT_ENTRIES);
  uf2_write_u16(buf, 0x13, (uint16_t)UF2_TOTAL_SECTORS);
  buf[0x15] = 0xF8;
  uf2_write_u16(buf, 0x16, UF2_FAT_SECTORS);
  uf2_write_u16(buf, 0x18, 0x003F);
  uf2_write_u16(buf, 0x1A, 0x00FF);
  uf2_write_u32(buf, 0x1C, 0);
  uf2_write_u32(buf, 0x20, 0);
  buf[0x24] = 0x80;
  buf[0x25] = 0x00;
  buf[0x26] = 0x29;
  uf2_write_u32(buf, 0x27, 0x12345678);

  {
    char label[11];
    uf2_fill_padded_name(label, sizeof(label), BOOT_UF2_VOLUME_LABEL);
    memcpy(&buf[0x2B], label, sizeof(label));
  }
  memcpy(&buf[0x36], "FAT16   ", 8);
  buf[0x1FE] = 0x55;
  buf[0x1FF] = 0xAA;
}

static void uf2_read_fat_sector(uint32_t fat_index, uint8_t* buf)
{
  memset(buf, 0, UF2_SECTOR_SIZE);

  // Only first sector needs entries
  if (fat_index != 0u)
  {
    return;
  }

  // FAT[0] and FAT[1]
  buf[0] = 0xF8;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;

  // Cluster 2 -> EOC
  buf[4] = 0xFF;
  buf[5] = 0xFF;

  // Cluster 3 -> EOC
  buf[6] = 0xFF;
  buf[7] = 0xFF;
}

static void uf2_read_root_dir(uint32_t lba, uint8_t* buf)
{
  memset(buf, 0, UF2_SECTOR_SIZE);

  // Only first sector contains entries in this simple layout
  if (lba != UF2_ROOT_DIR_START)
  {
    return;
  }

  // Volume label
  uf2_write_dir_entry(buf, 0, BOOT_UF2_VOLUME_LABEL, "", 0x08, 0, 0);

  // INFO_UF2.TXT
  uf2_write_dir_entry(buf, 1, "INFO_UF2", "TXT", 0x20, UF2_INFO_CLUSTER, (uint32_t)sizeof(info_txt) - 1u);

  // INDEX.HTM
  uf2_write_dir_entry(buf, 2, "INDEX", "HTM", 0x20, UF2_INDEX_CLUSTER, (uint32_t)sizeof(index_html) - 1u);
}

static void uf2_read_data(uint32_t lba, uint8_t* buf)
{
  memset(buf, 0, UF2_SECTOR_SIZE);

  uint32_t data_sector = lba - UF2_DATA_START;
  uint32_t cluster = (data_sector / UF2_SECTORS_PER_CLUSTER) + 2u;
  uint32_t sector_in_cluster = data_sector % UF2_SECTORS_PER_CLUSTER;

  if (cluster == UF2_INFO_CLUSTER)
  {
    uint32_t offset = sector_in_cluster * UF2_SECTOR_SIZE;
    uint32_t remaining = (uint32_t)sizeof(info_txt) - 1u;
    if (offset < remaining)
    {
      uint32_t copy_len = remaining - offset;
      if (copy_len > UF2_SECTOR_SIZE)
      {
        copy_len = UF2_SECTOR_SIZE;
      }
      memcpy(buf, info_txt + offset, copy_len);
    }
  }
  else if (cluster == UF2_INDEX_CLUSTER)
  {
    uint32_t offset = sector_in_cluster * UF2_SECTOR_SIZE;
    uint32_t remaining = (uint32_t)sizeof(index_html) - 1u;
    if (offset < remaining)
    {
      uint32_t copy_len = remaining - offset;
      if (copy_len > UF2_SECTOR_SIZE)
      {
        copy_len = UF2_SECTOR_SIZE;
      }
      memcpy(buf, index_html + offset, copy_len);
    }
  }
}

static void uf2_disk_read_block(uint32_t lba, uint8_t* buf)
{
  if (lba == 0u)
  {
    uf2_read_boot_sector(buf);
  }
  else if (lba >= UF2_FAT_START && lba < (UF2_FAT_START + UF2_FAT_SECTORS))
  {
    uf2_read_fat_sector(lba - UF2_FAT_START, buf);
  }
  else if (lba >= UF2_FAT2_START && lba < (UF2_FAT2_START + UF2_FAT_SECTORS))
  {
    uf2_read_fat_sector(lba - UF2_FAT2_START, buf);
  }
  else if (lba >= UF2_ROOT_DIR_START && lba < (UF2_ROOT_DIR_START + UF2_ROOT_DIR_SECTORS))
  {
    uf2_read_root_dir(lba, buf);
  }
  else if (lba >= UF2_DATA_START && lba < UF2_TOTAL_SECTORS)
  {
    uf2_read_data(lba, buf);
  }
  else
  {
    memset(buf, 0, UF2_SECTOR_SIZE);
  }
}

// TinyUSB MSC callbacks
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
  (void) lun;
  memcpy(vendor_id, "JUMBLEQ ", 8);
  memcpy(product_id, "UF2BOOT         ", 16);
  memcpy(product_rev, "1.0 ", 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
  (void) lun;
  return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size)
{
  (void) lun;
  *block_count = UF2_TOTAL_SECTORS;
  *block_size = UF2_SECTOR_SIZE;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
  (void) lun;
  (void) power_condition;
  (void) start;
  (void) load_eject;
  return true;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
  (void) lun;

  if (lba >= UF2_TOTAL_SECTORS)
  {
    return -1;
  }

  if (offset == 0u && bufsize == UF2_SECTOR_SIZE)
  {
    uf2_disk_read_block(lba, (uint8_t*)buffer);
    return (int32_t)bufsize;
  }

  uint8_t temp[UF2_SECTOR_SIZE];
  uf2_disk_read_block(lba, temp);
  memcpy(buffer, temp + offset, bufsize);
  return (int32_t)bufsize;
}

bool tud_msc_is_writable_cb(uint8_t lun)
{
  (void) lun;
  return true;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
  (void) lun;
  (void) lba;
  (void) offset;

  if (offset == 0u && bufsize == UF2_SECTOR_SIZE)
  {
    (void)uf2_process_block(buffer);
  }

  return (int32_t)bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
  (void) lun;
  (void) scsi_cmd;
  (void) buffer;
  (void) bufsize;

  (void) tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
  return -1;
}

void uf2_msc_init(void)
{
  tusb_rhport_init_t dev_init = {
      .role = TUSB_ROLE_DEVICE,
      .speed = TUSB_SPEED_HIGH
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);
}

void uf2_msc_task(void)
{
  tud_task();
}

void uf2_msc_on_block_write(const uint8_t* buf, uint32_t len)
{
  if (len == UF2_SECTOR_SIZE)
  {
    (void)uf2_process_block(buf);
  }
}
