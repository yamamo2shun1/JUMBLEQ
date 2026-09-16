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

#include "stm32h7rsxx_hal.h"

// バッファサイズ設定 - 小さいほど低レイテンシーだがアンダーラン/オーバーランのリスク増
// 96kHz再生の安定性を優先し、TX/RING は余裕を持たせる。
// サイズはすべてword単位で、1 frame = 4ch × 32bit = 4 word = 16 byte。
// リング容量相当は SAI_RNG_BUF_SIZE / 4 / sample_rate 秒。8192 word = 2048 frame なので
// 48kHzで約42.67ms、96kHzで約21.33ms。これは滞留可能な上限であり、通常の滞留水位や
// end-to-end遅延を表す値ではない。
#define SAI_RNG_BUF_SIZE 8192  // リングバッファ（word単位、2のべき乗必須）
#define SAI_TX_BUF_SIZE  256  // 4ch DMAバッファ (USB->SAI, word単位)
#define SAI_RX_BUF_SIZE  256  // 4ch DMAバッファ (SAI->USB, word単位)
// DMA halfを消費した後のTXリング目標水位（word単位、24 frame）。
// 消費前の判定基準は、この値にDMA half-buffer分を加えた水位になる。
// 0.5ms相当（48kHz、24 frame）／0.25ms相当（96kHz）で、DMA half期間（32 frame、
// 48kHz約0.667ms／96kHz約0.333ms）や容量相当時間とは合算しない。
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
// USB IN書き込み）。DMA half処理をTinyUSB FIFO操作より先に行う。
void audio_transport_service(uint32_t sample_rate_hz);

// ---- DMA/SAIエラーからの復旧 ----

typedef struct
{
    uint32_t sequence;
    uint32_t cause_mask;
    uint32_t dma_error_code;
    uint32_t sai_error_code;
    uint32_t sai_status_flags;
} AudioRecoveryRequest_t;

// ISRがpublishする復旧原因bitmask。
enum
{
    AUDIO_RECOVERY_CAUSE_TX_DMA = (1u << 0),
    AUDIO_RECOVERY_CAUSE_RX_DMA = (1u << 1),
    AUDIO_RECOVERY_CAUSE_TX_SAI = (1u << 2),
    AUDIO_RECOVERY_CAUSE_RX_SAI = (1u << 3),
};

// Audio Task context only. 未acknowledgeの復旧要求があればsnapshotを返してtrue。
// snapshotはpending payloadを切り離して返すため、以後に届いたエラーは新しい
// payloadとして蓄積される。復旧・再構築の成功時にacknowledgeと組み合わせて使う。
bool audio_transport_take_recovery_request(AudioRecoveryRequest_t* request);

// Audio Task context only. sequence以前に受理した要求を完了扱いにする。
// snapshot以後にpublishされた新しいエラーはpendingのまま残る。
void audio_transport_ack_recovery_request(uint32_t sequence);

// SAIエラー割り込みのISR処理。フラグの保存・クリア、対象割り込みのマスク、診断記録、
// 復旧要求のpublishのみを行い、HAL標準ハンドラの停止待ち（SAI_DMAAbort→SAI_Disable）へ
// 渡さない。処理した場合はtrueを返すので、SAI IRQハンドラのUSER CODE領域から呼び、
// trueならHAL_SAI_IRQHandlerをスキップする。停止・再初期化はAudio Taskへ集約する。
bool audio_transport_sai_error_isr(SAI_HandleTypeDef* hsai);

// 復旧・レート変更共通の停止と初期化。Audio Task context only。
// SAI/GPDMA停止（HAL_SAI_Abortで停止完了確認・フラグ/FIFO整理）、DMAイベント・
// リングindexリセット、DMA/リング/USBバッファ消去。timecode設定には触れない。
// deinit_sai=trueはSAIをDeInitする（レート変更用）。復旧はfalseにしてSAIのMSP資源と
// 設定を維持し、HAL_SAI_MspInit（Error_Handlerを含む生成コード）を再実行させない。
// 戻り値はSAI停止完了確認の成否（falseでもDMA abortとバッファ消去は行う）。
bool audio_transport_stop_and_clear_paths(bool deinit_sai);

// DMA channel再構築とTXリングprefill・TX開始。Audio Task context only。
// init_sai=trueはレート変更用でCubeMX生成のMX_SAIx_Init()を呼ぶ。
// falseは復旧用で、READY状態からのHAL_SAI_Init（MspInitをスキップ）によりMSP資源を
// 維持したままSAI設定とErrorCodeを再初期化し、DMAリンクを再実行して再始動する。
// 失敗時は両経路を停止してfalseを返す。
bool audio_transport_rebuild_and_start_tx(bool init_sai);

// TX同期待ち後のRX開始。Audio Task context only。成功時は現在レートの
// USB IN FIFO目標を再適用する。失敗時は両経路を停止（MSP維持）してfalseを返す。
bool audio_transport_start_rx_after_tx_sync(void);

// 診断ログ・LED制御用。
bool audio_transport_is_output_streaming(void);
bool audio_transport_is_input_streaming(void);
int32_t audio_transport_tx_used_words(void);

// USB OUT feedback用の目標FIFO水位(byte単位)。現在のサンプルレートから
// 0.5 ms相当を計算し、16 byte frame境界・FIFO容量・最大packet余白・uint16_t
// 範囲へクランプして返す。副作用なし。UAC2 feedback callbackから呼べる。
uint16_t audio_transport_usb_out_fifo_target_bytes(uint32_t sample_rate_hz);

#endif /* AUDIO_TRANSPORT_INTERNAL_H_ */
