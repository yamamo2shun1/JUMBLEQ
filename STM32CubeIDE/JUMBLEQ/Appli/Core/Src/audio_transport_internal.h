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
#include "audio_diagnostics_internal.h"

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

// audio_control_reset_runtime_state() のバッファ消去部。__DSB() まで含む。
void audio_transport_reset_buffers(void);

// start_sai() のSAI/GPDMA開始シーケンス（TX開始→500ms→LED→RX開始）。
void audio_transport_start(void);

// 停止・再構築プリミティブの失敗内容は diagnostics 側の AudioTransportFailure_t を使う。
// 呼出側はゼロ初期化して渡し、失敗した操作とHAL結果を診断へ引き継ぐ。

// サンプルレート変更の停止・バッファ消去部。SAI/GPDMA停止を確認できた場合のみ
// ring/event/診断リセット → buffer消去 → timecodeリセット → 新レートのIN FIFO目標適用
// → __DSB() を行いtrueを返す。停止未確認なら参照バッファへ触れずfalseを返す。
// 再開は audio_transport_rebuild_and_start_tx() から行う。
bool audio_transport_reset_for_sample_rate(uint32_t sample_rate_hz,
                                           AudioTransportFailure_t* failure);

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
// SAIのMSP資源と設定を維持し、HAL_SAI_MspInit（Error_Handlerを含む生成コード）を
// 再実行させない。
// 戻り値はSAI/GPDMA停止完了確認の成否。falseの場合は参照バッファを消去・再利用せず、
// 呼出側は再構築を行わないこと。failureへ失敗した操作とHAL結果を格納する。
bool audio_transport_stop_and_clear_paths(AudioTransportFailure_t* failure);

// DMA channel再構築とTXリングprefill・TX開始。Audio Task context only。
// READY状態からのHAL_SAI_Init（MspInitをスキップ）によりMSP資源を維持したまま
// SAI設定とErrorCodeを再初期化し、DMAリンクを再実行して再始動する。
// 失敗時は両経路を停止してfalseを返し、failureへ失敗した操作とHAL結果を格納する。
bool audio_transport_rebuild_and_start_tx(AudioTransportFailure_t* failure);

// DMA転送停止の確認。HAL_DMA_Abortの結果がHAL_OK、または未開始/停止済みを示す
// HAL_DMA_ERROR_NO_XFERの場合だけtrueを返す。falseは転送が停止したと確認できず、
// 参照バッファの消去・再構成を行ってはならない。
bool audio_transport_dma_abort_confirmed(DMA_HandleTypeDef* hdma);

// ISRからも参照できる、未acknowledgeの復旧要求sequence。レート切り替え開始時に
// 固定し、切り替え成功後だけ audio_transport_ack_recovery_request() へ渡す。
uint32_t audio_transport_recovery_request_sequence(void);

// TX同期待ち後のRX開始。Audio Task context only。成功時は現在レートの
// USB IN FIFO目標を再適用する。失敗時は両経路を停止（MSP維持）してfalseを返し、
// failureへ失敗した操作とHAL結果を格納する。
bool audio_transport_start_rx_after_tx_sync(AudioTransportFailure_t* failure);

// TinyUSBのIN/OUT FIFOに残る切り替え期間中のデータを公開APIで破棄する。
// TinyUSBが内部で使用しているバッファには触れない。Audio Task context only。
void audio_transport_clear_usb_fifos(void);

// 診断ログ・LED制御用。
bool audio_transport_is_output_streaming(void);
bool audio_transport_is_input_streaming(void);
int32_t audio_transport_tx_used_words(void);

// USB OUT feedback用の目標FIFO水位(byte単位)。現在のサンプルレートから
// 0.5 ms相当を計算し、16 byte frame境界・FIFO容量・最大packet余白・uint16_t
// 範囲へクランプして返す。副作用なし。UAC2 feedback callbackから呼べる。
uint16_t audio_transport_usb_out_fifo_target_bytes(uint32_t sample_rate_hz);

#endif /* AUDIO_TRANSPORT_INTERNAL_H_ */
