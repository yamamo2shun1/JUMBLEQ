/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ha Thach (tinyusb.org)
 * Copyright (c) 2020 Jerzy Kasenberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "usb_descriptors.h"

//--------------------------------------------------------------------+
// Board Specific Configuration
//--------------------------------------------------------------------+
#define CFG_TUSB_DEBUG 0

#define CFG_TUSB_MCU              OPT_MCU_STM32H7RS
#define CFG_TUSB_OS               OPT_OS_FREERTOS
#ifndef pdTICKS_TO_MS
    // TinyUSB 0.21.0 uses this macro, which is absent from the bundled FreeRTOS.
    #define pdTICKS_TO_MS(ticks) ((uint32_t) ((((uint64_t) (ticks)) * 1000ULL) / configTICK_RATE_HZ))
#endif
#define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_HIGH_SPEED
#define BOARD_TUD_MAX_SPEED       OPT_MODE_HIGH_SPEED
#define BOARD_DEVICE_RHPORT_NUM   1
#define BOARD_TUD_RHPORT          1
#define CFG_TUSB_RHPORT1_MODE     (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)

// DWC2 DMAモード
#define CFG_TUD_DWC2_SLAVE_ENABLE 0
#define CFG_TUD_DWC2_DMA_ENABLE   1

// DMAモードではDCacheメンテナンスを有効化
#define CFG_TUD_MEM_DCACHE_ENABLE    1
#define CFG_TUD_MEM_DCACHE_LINE_SIZE 32

// DMAバッファをnoncacheable領域に配置
#define CFG_TUD_MEM_SECTION __attribute__((section("noncacheable_buffer"), aligned(32)))
#define CFG_TUD_MEM_ALIGN   TU_ATTR_ALIGNED(32)

// noncacheable_buffer領域をuncached regionsに追加
// TinyUSBのDCacheメンテナンスをスキップするための設定
#define CFG_DWC2_MEM_UNCACHED_REGIONS \
    {.start = 0x24040000, .end = 0x24071FFF},

// RHPort number used for device can be defined by board.mk, default to port 0
#ifndef BOARD_TUD_RHPORT
    #define BOARD_TUD_RHPORT 0
#endif

// RHPort max operational speed can defined by board.mk
#ifndef BOARD_TUD_MAX_SPEED
    #define BOARD_TUD_MAX_SPEED OPT_MODE_DEFAULT_SPEED
#endif

//--------------------------------------------------------------------
// Common Configuration
//--------------------------------------------------------------------

// TinyUSB task queue size - increase for audio streaming
#define CFG_TUD_TASK_QUEUE_SZ 64

// defined by compiler flags for flexibility
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

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
// 注意: CFG_TUSB_MEM_SECTION は空にする（構造体に関数ポインタが含まれるため）
// DMAバッファは CFG_TUD_MEM_SECTION で別途 noncacheable 領域に配置
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(32)))

    //--------------------------------------------------------------------
    // DEVICE CONFIGURATION
    //--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
    #define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC    0
#define CFG_TUD_MSC    0
#define CFG_TUD_HID    0
#define CFG_TUD_MIDI   1
#define CFG_TUD_AUDIO  1
#define CFG_TUD_VENDOR 0

// MIDI Endpoint/FIFO sizes
#if CFG_TUD_MIDI
    // Bulk endpoints: 64 bytes (FS) / 512 bytes (HS)
    #define CFG_TUD_MIDI_EP_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)
    // FIFO size for tud_midi_{read,write} (bytes)
    #define CFG_TUD_MIDI_RX_BUFSIZE CFG_TUD_MIDI_EP_BUFSIZE
    #define CFG_TUD_MIDI_TX_BUFSIZE CFG_TUD_MIDI_EP_BUFSIZE
#endif

//--------------------------------------------------------------------
// AUDIO CLASS DRIVER CONFIGURATION
//--------------------------------------------------------------------

// Allow volume controlled by on-baord button
#define CFG_TUD_AUDIO_ENABLE_INTERRUPT_EP 1

// How many formats are used, need to adjust USB descriptor if changed
#define CFG_TUD_AUDIO_FUNC_1_N_FORMATS 1

// Audio format type I specifications
/* 24bit/48kHz is the best quality for headset or 24bit/96kHz for 2ch speaker,
   high-speed is needed beyond this */
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE 96000
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX   4
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX   4

// 24bit in 32bit slots
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX 4
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX         24
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX 4
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX         24

// EP and buffer size - for isochronous EP´s, the buffer and EP size are equal (different sizes would not make sense)
#define CFG_TUD_AUDIO_ENABLE_EP_IN 1

// UAC2 High-Speed IN interval: 2^(3-1) microframes = 500 us.
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_INTERVAL         3
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_INTERVAL_UFRAMES (1U << (CFG_TUD_AUDIO_FUNC_1_EP_IN_INTERVAL - 1U))

// Maximum IN packet size for the configured interval.
#define CFG_TUD_AUDIO20_FUNC_1_FORMAT_1_EP_SZ_IN (((((CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE * CFG_TUD_AUDIO_FUNC_1_EP_IN_INTERVAL_UFRAMES) + 7999U) / 8000U) + 1U) * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX CFG_TUD_AUDIO20_FUNC_1_FORMAT_1_EP_SZ_IN
#if CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX > 1024U
    #error High-Speed isochronous IN packet exceeds 1024 bytes
#endif
// Tx flow control needs buffer size >= 4* EP size to work correctly
// Keep eight maximum packets to absorb scheduling jitter.
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ (8 * CFG_TUD_AUDIO20_FUNC_1_FORMAT_1_EP_SZ_IN)

// EP and buffer size - for isochronous EP´s, the buffer and EP size are equal (different sizes would not make sense)
#define CFG_TUD_AUDIO_ENABLE_EP_OUT 1

// Enable explicit feedback EP for UAC2 asynchronous OUT stream
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP 1

// UAC2 (High-Speed) Endpoint size calculation
#define CFG_TUD_AUDIO20_FUNC_1_FORMAT_1_EP_SZ_OUT TUD_AUDIO_EP_SIZE(true, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)

// Maximum EP OUT size for all AS alternate settings used
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX CFG_TUD_AUDIO20_FUNC_1_FORMAT_1_EP_SZ_OUT

// Rx flow control needs buffer size >= 4* EP size to work correctly
// USB OUT FIFOは唯一の再生バッファ（#260）。feedback目標・DMA half・最大packetに加え、
// feedbackに従わないホスト向けの上側ドリフト補正の閾値（96kHzで160 frame）を
// 収めるため16 packet分（3328 byte、208 frame）を確保する。
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ (16 * CFG_TUD_AUDIO20_FUNC_1_FORMAT_1_EP_SZ_OUT)

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H_ */
