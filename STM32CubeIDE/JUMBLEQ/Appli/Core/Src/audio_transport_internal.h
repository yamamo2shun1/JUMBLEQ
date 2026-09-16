/*
 * audio_transport_internal.h
 *
 * Private API of the realtime audio transport module (USB OUT/IN <-> ring
 * buffers <-> SAI TX/RX). Requests may be published from TinyUSB callbacks;
 * the Audio Task applies them via audio_transport_service().
 */

#ifndef AUDIO_TRANSPORT_INTERNAL_H_
#define AUDIO_TRANSPORT_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>

// バッファサイズ設定 - 小さいほど低レイテンシーだがアンダーラン/オーバーランのリスク増
// 96kHz再生の安定性を優先し、TX/RING は余裕を持たせる。
// 48kHz時のレイテンシー目安: SAI_RNG_BUF_SIZE / sample_rate * 1000 [ms]
#define SAI_RNG_BUF_SIZE 8192  // リングバッファ（2のべき乗必須）
#define SAI_TX_BUF_SIZE  256  // 4ch DMAバッファ (USB->SAI)
#define SAI_RX_BUF_SIZE  256  // 4ch DMAバッファ (SAI->USB)
// DMA halfを消費した後のTXリング目標水位（word単位）。
// 消費前の判定基準は、この値にDMA half-buffer分を加えた水位になる。
#define SAI_TX_TARGET_LEVEL_WORDS 96

// DMA転送先storage。linked_list.cがaddressを参照する。
extern int32_t stereo_out_buf[SAI_TX_BUF_SIZE];
extern int32_t stereo_in_buf[SAI_RX_BUF_SIZE];

typedef enum
{
    AUDIO_TRANSPORT_STREAM_OUT = 0,
    AUDIO_TRANSPORT_STREAM_IN,
} AudioTransportStream_t;

// UAC2 function id shared by the control plane (feedback) and the transport
// (tud_audio_n_* data path calls).
enum
{
    AUDIO_FUNC_ID = 0u,
};

// Registers the Audio Task and its notification helper. Must be called before
// any request/ISR notification.
void audio_transport_register_current_task(void);

// Wakes the Audio Task from task context.
void audio_transport_notify_task(void);

// Publishes a UAC2 interface alt setting change. This only records the request
// (and wakes the Audio Task); the applied stream state is owned by the task.
void audio_transport_request_stream(AudioTransportStream_t stream, bool enabled);

// Applies pending stream requests. Audio Task context only. Returns true when
// a request was consumed so the facade can refresh the LED blink intervals.
bool audio_transport_apply_requested_stream_state(void);

// Audio Task context. Clears pending USB events while the USB stack is not
// initialized yet.
void audio_transport_clear_pending_events(void);

// reset_audio_buffer() のバッファ消去部。__DSB() まで含む。
void audio_transport_reset_buffers(void);

// start_sai() のSAI/GPDMA開始シーケンス（TX開始→500ms→LED→RX開始）。
void audio_transport_start(void);

// サンプルレート変更の停止・バッファ消去部。SAI/GPDMA停止 → ring/event/
// 診断リセット → buffer消去 → timecodeリセット → __DSB()。再開は
// audio_transport_restart_after_rate_change() で行う。
void audio_transport_reset_for_sample_rate(uint32_t sample_rate_hz);

// サンプルレート変更の再開部。DMA再初期化 → SAI再初期化 → TX prefill/開始 →
// 10ms待機 → RX開始。失敗時は Error_Handler()。
void audio_transport_restart_after_rate_change(void);

// Audio Taskから呼ぶデータ搬送サービス（USB OUT読み出し、ring/SAIコピー、
// USB IN書き込み）。
void audio_transport_service(uint32_t sample_rate_hz);

// 診断ログ・LED制御用。
bool audio_transport_is_output_streaming(void);
bool audio_transport_is_input_streaming(void);
int32_t audio_transport_tx_used_words(void);

#endif /* AUDIO_TRANSPORT_INTERNAL_H_ */
