/*
 * audio_control.h
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#ifndef INC_AUDIO_CONTROL_H_
#define INC_AUDIO_CONTROL_H_

#include "main.h"
#include "ui_control.h"

// バッファサイズ設定 - 小さいほど低レイテンシーだがアンダーラン/オーバーランのリスク増
// 96kHz再生の安定性を優先し、TX/RING は余裕を持たせる。
// 48kHz時のレイテンシー目安: SAI_RNG_BUF_SIZE / sample_rate * 1000 [ms]
#define SAI_RNG_BUF_SIZE 8192  // リングバッファ（2のべき乗必須）
#define SAI_TX_BUF_SIZE  256  // 4ch DMAバッファ (USB->SAI)
#define SAI_RX_BUF_SIZE  256  // 4ch DMAバッファ (SAI->USB)
// TXリングの目標水位（word単位）。
// DMAバッファ縮小後は half-buffer より少し低めにして平均滞留量をさらに下げる。
#define SAI_TX_TARGET_LEVEL_WORDS 96

// Runtime DSP parameter update switch for A/B diagnosis.
// 0: disable ui_control_task() DSP writes (noise root-cause test mode)
// 1: enable normal runtime control updates
#define ENABLE_DSP_RUNTIME_CONTROL 1

// Timecode入力を相対速度として解析し、SYNTHモード時に発音する。
// 0にすると既存のUSB/SAI経路だけで動作する。
#ifndef ENABLE_TIMECODE_OSCILLATOR
#define ENABLE_TIMECODE_OSCILLATOR 1
#endif

// USB OUT -> SAI TX経路の軽量診断。RTT出力は行わず、デバッガから参照する。
enum
{
    AUDIO_TX_DIAG_EVENT_BOTH_PENDING = (1u << 0),
    AUDIO_TX_DIAG_EVENT_HALF_REWRITE = (1u << 1),
    AUDIO_TX_DIAG_EVENT_CPLT_REWRITE = (1u << 2),
    AUDIO_TX_DIAG_EVENT_UNDERRUN     = (1u << 3),
    AUDIO_TX_DIAG_EVENT_PARTIAL_FILL = (1u << 4),
    AUDIO_TX_DIAG_EVENT_DRIFT_UP     = (1u << 5),
    AUDIO_TX_DIAG_EVENT_DRIFT_DOWN   = (1u << 6),
    AUDIO_TX_DIAG_EVENT_DMA_ERROR    = (1u << 7),
    AUDIO_TX_DIAG_EVENT_SAI_ERROR    = (1u << 8),
};

typedef struct
{
    uint32_t reset_count;
    uint32_t tx_half_callbacks;
    uint32_t tx_cplt_callbacks;
    uint32_t both_pending_events;
    uint32_t half_rewrite_events;
    uint32_t cplt_rewrite_events;
    uint32_t underrun_events;
    uint32_t partial_fill_events;
    uint32_t drift_up_events;
    uint32_t drift_down_events;
    uint32_t dma_error_events;
    uint32_t sai_error_events;
    uint32_t tx_used_min_words;
    uint32_t tx_used_max_words;
    int32_t  last_event_used_words;
    uint32_t last_event_flags;
    uint32_t last_event_tick_ms;
    uint32_t last_event_cycle;
    uint32_t last_pending_callback;  // 1: half, 2: complete
    uint32_t both_pending_last_callback;
    uint32_t half_service_cycles_max;
    uint32_t cplt_service_cycles_max;
    uint32_t last_dma_error_code;
    uint32_t last_sai_error_code;
    uint32_t last_sai_status_flags;
} AudioTxDiagnostics_t;

extern volatile AudioTxDiagnostics_t g_audio_tx_diagnostics;

uint32_t get_tx_blink_interval_ms(void);
uint32_t get_rx_blink_interval_ms(void);
uint32_t get_current_sample_rate_hz(void);
void reset_audio_buffer(void);
void AUDIO_LoadAndApplyRoutingFromEEPROM(void);

void AUDIO_Init_AK4619(uint32_t hz);
void AUDIO_Init_ADAU1466(uint32_t hz);

void start_sai(void);

void AUDIO_SAI_Reset_ForNewRate(void);
void audio_control_register_task(void);
void audio_task(void);

#endif /* INC_AUDIO_CONTROL_H_ */
