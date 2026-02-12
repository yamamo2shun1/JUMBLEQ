#ifndef BOOT_CONFIG_H
#define BOOT_CONFIG_H

#include <stdint.h>

#define BOOT_EXTMEM_BASE            (0x90000000u)
#define BOOT_APP_OFFSET             (0x00010000u)  /* 64KB */
#define BOOT_APP_BASE               (BOOT_EXTMEM_BASE + BOOT_APP_OFFSET)
#define BOOT_APP_SIZE               (0x00400000u)  /* 4MB */

#define BOOT_META_OFFSET            (0x00000000u)
#define BOOT_META_SIZE              (0x00010000u)  /* 64KB */
#define BOOT_META_HEADER_SIZE       (64u)

#define BOOT_UF2_BLOCK_SIZE         (512u)
#define BOOT_UF2_PAYLOAD_SIZE       (256u)
#define BOOT_UF2_DISK_SIZE_BYTES    (16u * 1024u * 1024u)

#define BOOT_USB_VID                (0x31BFu)
#define BOOT_USB_PID                (0x0100u)
#define BOOT_UF2_VOLUME_LABEL       "UF2BOOT"

#define BOOT_ERASE_BLOCK_SIZE       (0x00010000u)  /* 64KB */
#define BOOT_META_ERASE_SIZE        (0x00001000u)  /* 4KB */

#endif /* BOOT_CONFIG_H */
