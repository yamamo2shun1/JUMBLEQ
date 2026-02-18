#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_descriptors.h"

//--------------------------------------------------------------------+
// Board Specific Configuration
//--------------------------------------------------------------------+
#define CFG_TUSB_DEBUG 0

#define CFG_TUSB_MCU              OPT_MCU_STM32H7RS
#define CFG_TUSB_OS               OPT_OS_NONE
#define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_HIGH_SPEED
#define BOARD_TUD_MAX_SPEED       OPT_MODE_HIGH_SPEED
#define BOARD_DEVICE_RHPORT_NUM   1
#define BOARD_TUD_RHPORT          1
#define CFG_TUSB_RHPORT1_MODE     (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)

// DWC2 DMA mode
#define CFG_TUD_DWC2_SLAVE_ENABLE 0
#define CFG_TUD_DWC2_DMA_ENABLE   1

// Enable DCache maintenance in DMA mode
#define CFG_TUD_MEM_DCACHE_ENABLE    1
#define CFG_TUD_MEM_DCACHE_LINE_SIZE 32

// Place DMA buffers into non-cacheable region
#define CFG_TUD_MEM_SECTION __attribute__((section("noncacheable_buffer"), aligned(32)))
#define CFG_TUD_MEM_ALIGN   TU_ATTR_ALIGNED(32)

// Uncached region range for TinyUSB DWC2 driver
#define CFG_DWC2_MEM_UNCACHED_REGIONS \
    {.start = 0x24040000, .end = 0x2405FFFF},

// TinyUSB internal mem section should be empty
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(32)))

//--------------------------------------------------------------------
// Common Configuration
//--------------------------------------------------------------------
#define CFG_TUD_TASK_QUEUE_SZ 16

#ifndef CFG_TUSB_MCU
    #error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUSB_OS
    #define CFG_TUSB_OS OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
    #define CFG_TUSB_DEBUG 0
#endif

// Enable Device stack
#define CFG_TUD_ENABLED 1

// Default is max speed that hardware controller could support with on-chip PHY
#define CFG_TUD_MAX_SPEED BOARD_TUD_MAX_SPEED

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
    #define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC    0
#define CFG_TUD_MSC    1
#define CFG_TUD_HID    0
#define CFG_TUD_MIDI   0
#define CFG_TUD_AUDIO  0
#define CFG_TUD_VENDOR 0

#define CFG_TUD_MSC_EP_BUFSIZE 512

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H */
