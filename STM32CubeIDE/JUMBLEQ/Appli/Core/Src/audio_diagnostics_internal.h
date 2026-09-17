/*
 * audio_diagnostics_internal.h
 *
 * Private recording API for the audio diagnostics counters. The functions are
 * called from USB/DMA ISR context (transport) and from the Audio Task; they
 * only update fixed-size counters, min/max values and timestamps.
 *
 * AUDIO_DIAG_LOG selects the RTT interval summary. The default lives here so
 * every audio module sees the same value from the build flags.
 */

#ifndef AUDIO_DIAGNOSTICS_INTERNAL_H_
#define AUDIO_DIAGNOSTICS_INTERNAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef AUDIO_DIAG_LOG
#define AUDIO_DIAG_LOG 0
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
    uint32_t dma_events_dropped;
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
    uint32_t half_deadline_overruns;
    uint32_t cplt_deadline_overruns;
    uint32_t priming_wait_events;         // priming待機で無音出力したhalf数
    uint32_t priming_completed_events;    // priming完了回数
    uint32_t drift_up_threshold_events;   // 上側開始閾値への逸脱回数（補正の有無に依らない）
    uint32_t drift_down_threshold_events; // 下側開始閾値への逸脱回数
    uint32_t drift_up_suppressed_events;  // 逸脱継続したが最小間隔制限で見送った回数
    uint32_t drift_down_suppressed_events;
} AudioTxDiagnostics_t;

extern volatile AudioTxDiagnostics_t g_audio_tx_diagnostics;

// RX DMA搬送の永続診断。stream/rate変更や復旧のバッファ消去ではリセットせず、
// AUDIO_DIAG_LOG=0でもデバッガから参照できる。
typedef struct
{
    uint32_t rx_half_callbacks;
    uint32_t rx_cplt_callbacks;
    uint32_t rx_half_rewrite_events;
    uint32_t rx_cplt_rewrite_events;
    uint32_t rx_both_pending_events;
    uint32_t rx_dma_events_dropped;
    uint32_t rx_both_pending_last_callback;
    uint32_t rx_half_service_cycles_max;
    uint32_t rx_cplt_service_cycles_max;
    uint32_t rx_half_deadline_overruns;
    uint32_t rx_cplt_deadline_overruns;
    uint32_t rx_ring_discard_events;
    uint32_t rx_ring_discard_words;
    uint32_t rx_ring_full_discards;
    uint32_t rx_last_discard_words;
    uint32_t rx_last_discard_tick_ms;
    uint32_t dma_error_events;
    uint32_t sai_error_events;
    uint32_t last_dma_error_code;
    uint32_t last_sai_error_code;
    uint32_t last_sai_status_flags;
} AudioRxDiagnostics_t;

extern volatile AudioRxDiagnostics_t g_audio_rx_diagnostics;

// 復旧要求・試行・結果の永続診断。復旧時のバッファ消去では消去しない。
typedef struct
{
    uint32_t request_count;
    uint32_t attempt_count;
    uint32_t success_count;
    uint32_t failure_count;
    uint32_t consecutive_failures;
    uint32_t latched;
    uint32_t last_cause_mask;
    uint32_t last_request_dma_error_code;
    uint32_t last_request_sai_error_code;
    uint32_t last_request_sai_status_flags;
    uint32_t last_failed_stage;
    uint32_t last_request_tick_ms;
    uint32_t last_attempt_tick_ms;
    uint32_t last_success_tick_ms;
    uint32_t last_failure_tick_ms;
    uint32_t unknown_dma_error_events;
    uint32_t last_unknown_dma_error_code;
} AudioRecoveryDiagnostics_t;

extern volatile AudioRecoveryDiagnostics_t g_audio_recovery_diagnostics;

// 復旧失敗時の段階。どの処理で失敗したかを診断値へ残す。
enum
{
    AUDIO_RECOVERY_STAGE_NONE     = 0u,
    AUDIO_RECOVERY_STAGE_PREPARE  = 1u,
    AUDIO_RECOVERY_STAGE_TX_START = 2u,
    AUDIO_RECOVERY_STAGE_RX_START = 3u,
};

// サンプルレート切り替えの永続診断。切替・失敗・cleanupの結果をAUDIO_DIAG_LOG=0でも
// デバッガから参照できる形で残す。バッファ消去や復旧では消去しない。
typedef struct
{
    uint32_t attempt_count;
    uint32_t success_count;
    uint32_t failure_count;
    uint32_t noop_count;
    uint32_t last_target_hz;
    uint32_t last_sequence;
    uint32_t last_applied_hz;
    uint32_t last_failed_result;   // 失敗したAPIの結果コード（sigma_spi_result_t等）
    uint32_t last_failed_stage;
    uint32_t last_failed_hal_status;
    uint32_t last_failed_aux_status;
    uint32_t last_cleanup_status;
    uint32_t last_attempt_tick_ms;
    uint32_t last_success_tick_ms;
    uint32_t last_failure_tick_ms;
    uint32_t state;
    uint32_t applied_valid;
    uint32_t clock_valid;
} AudioRateSwitchDiagnostics_t;

extern volatile AudioRateSwitchDiagnostics_t g_audio_rate_switch_diagnostics;

// レート切り替えの失敗段階。
enum
{
    AUDIO_RATE_STAGE_NONE           = 0u,
    AUDIO_RATE_STAGE_ADC_STOP       = 1u,
    AUDIO_RATE_STAGE_TRANSPORT_STOP = 2u,
    AUDIO_RATE_STAGE_DSP_UPDATE     = 3u,
    AUDIO_RATE_STAGE_TX_START       = 4u,
    AUDIO_RATE_STAGE_RX_START       = 5u,
    AUDIO_RATE_STAGE_ADC_RESTART    = 6u,
};

// cleanup状態bit。停止確認・ADC復帰の成否を切り替え失敗と別に残す。
enum
{
    AUDIO_RATE_CLEANUP_TRANSPORT_STOPPED = (1u << 0),
    AUDIO_RATE_CLEANUP_ADC_RESTARTED     = (1u << 1),
};

// 失敗操作ID。last_failed_aux_statusの上位16bitへ格納し、どの操作で失敗したかを
// 下位16bitのErrorCodeと合わせて区別する。
enum
{
    AUDIO_RATE_OP_NONE = 0u,
    AUDIO_RATE_OP_ADC_STOP,
    AUDIO_RATE_OP_HPDMA_STOP,
    AUDIO_RATE_OP_SAI_ABORT_TX,
    AUDIO_RATE_OP_SAI_ABORT_RX,
    AUDIO_RATE_OP_GPDMA_ABORT_TX,
    AUDIO_RATE_OP_GPDMA_ABORT_RX,
    AUDIO_RATE_OP_GPDMA_INIT_TX,
    AUDIO_RATE_OP_GPDMA_INIT_RX,
    AUDIO_RATE_OP_SAI_INIT_TX,
    AUDIO_RATE_OP_SAI_INIT_RX,
    AUDIO_RATE_OP_TX_PATH_START,
    AUDIO_RATE_OP_RX_PATH_START,
    AUDIO_RATE_OP_ADC_RESTART_LIST_CONFIG,
    AUDIO_RATE_OP_ADC_RESTART_LINKQ,
    AUDIO_RATE_OP_ADC_RESTART_START,
    AUDIO_RATE_OP_ADC_RESTART_ADC,
    AUDIO_RATE_OP_DSP_MUX,
    AUDIO_RATE_OP_DSP_SOUT_SOURCE,
    AUDIO_RATE_OP_DSP_CLK_GEN,
    AUDIO_RATE_OP_DSP_PLL_LOCK,
    AUDIO_RATE_OP_DSP_UNSUPPORTED,
    AUDIO_RATE_OP_DSP_SKIPPED,
};

// last_failed_aux_statusのエンコード: (操作ID << 16) | (ErrorCode & 0xFFFF)。
static inline uint32_t audio_rate_aux_pack(uint32_t operation, uint32_t error_code)
{
    return (operation << 16) | (error_code & 0xFFFFu);
}

// 停止・再構築処理の失敗内容。呼出側はゼロ初期化して渡す。
typedef struct
{
    uint32_t operation;   // AUDIO_RATE_OP_*
    uint32_t hal_status;  // 失敗したHAL APIの戻り値
    uint32_t error_code;  // 対象ハンドルのErrorCode
} AudioTransportFailure_t;

// 最初の失敗だけを保持する。failureがNULLなら何もしない。
static inline void audio_transport_failure_record(AudioTransportFailure_t* failure,
                                                  uint32_t operation,
                                                  uint32_t hal_status,
                                                  uint32_t error_code)
{
    if ((failure != NULL) && (failure->operation == AUDIO_RATE_OP_NONE))
    {
        failure->operation  = operation;
        failure->hal_status = hal_status;
        failure->error_code = error_code;
    }
}

// レート状態遷移のsnapshot。状態・適用値・有効フラグ・CLK_VALIDを同じ更新で記録する。
void audio_diagnostics_update_rate_snapshot(uint32_t state,
                                            uint32_t applied_hz,
                                            uint32_t applied_valid,
                                            uint32_t clock_valid);

void audio_diagnostics_record_rate_switch_attempt(uint32_t target_hz, uint32_t sequence);
void audio_diagnostics_record_rate_switch_noop(uint32_t target_hz, uint32_t sequence);
void audio_diagnostics_record_rate_switch_success(uint32_t applied_hz);
// failed_resultは失敗したAPIの結果コード（sigma_spi_result_tやADAU1466_RATE_FAIL_*等）。
void audio_diagnostics_record_rate_switch_failure(uint32_t failed_result,
                                                  uint32_t failed_stage,
                                                  uint32_t hal_status,
                                                  uint32_t aux_status);
void audio_diagnostics_record_rate_switch_cleanup(uint32_t cleanup_status);

// DMA callback kinds. The values must match the transport's internal
// DMA event encoding; audio_transport.c asserts this at compile time.
enum
{
    AUDIO_DIAG_DMA_EVENT_NONE     = 0u,
    AUDIO_DIAG_DMA_EVENT_HALF     = 1u,
    AUDIO_DIAG_DMA_EVENT_COMPLETE = 2u,
};

// g_audio_tx_diagnostics のリセット。reset_tx_locked() は呼出側がクリティカル
// セクションを保持していること。transport はTX DMAイベントのリセットと同一
// セクションで実行する。
void audio_diagnostics_reset_tx_locked(void);
void audio_diagnostics_reset_interval(void);
void audio_diagnostics_reset_session(void);

// TXリング水位。streaming は transport の s_streaming_out を渡す。
void audio_diagnostics_record_tx_level(bool streaming, int32_t used);
void audio_diagnostics_record_tx_interval_level(int32_t used);

// USB再生priming。待機half数と完了回数を区別して記録する。
void audio_diagnostics_record_tx_priming_wait(void);
void audio_diagnostics_record_tx_priming_complete(void);

// ドリフト補正の閾値逸脱と、最小間隔による補正見送り。
// upward=true は上側（水位過多）、false は下側（水位不足傾向）。
void audio_diagnostics_record_tx_drift_threshold(bool upward);
void audio_diagnostics_record_tx_drift_suppressed(bool upward);

// TXリングイベント。flags は AUDIO_TX_DIAG_EVENT_* のビット和。
void audio_diagnostics_record_tx_event(bool streaming, uint32_t flags, int32_t used);

// DMA callback / overwrite / drop / error
void audio_diagnostics_record_tx_dma_callback(uint32_t callback_event,
                                              uint32_t overwritten_event,
                                              bool streaming,
                                              int32_t used);
void audio_diagnostics_record_rx_dma_callback(uint32_t callback_event,
                                              uint32_t overwritten_event);
void audio_diagnostics_record_tx_dma_service(uint32_t callback_event,
                                             uint32_t service_cycles,
                                             bool deadline_overrun);
void audio_diagnostics_record_rx_dma_service(uint32_t callback_event,
                                             uint32_t service_cycles,
                                             bool deadline_overrun);
void audio_diagnostics_record_tx_events_dropped(uint32_t last_event,
                                                uint32_t dropped_events,
                                                int32_t used);
void audio_diagnostics_record_rx_events_dropped(uint32_t last_event,
                                                uint32_t dropped_events);
// RXリング破棄。full_discardは負値・capacity超過による異常水位の全破棄を表す。
void audio_diagnostics_record_rx_ring_discard(uint32_t dropped_words, bool full_discard);
void audio_diagnostics_record_dma_error(uint32_t error_code,
                                        bool tx_route,
                                        bool streaming,
                                        int32_t used);
// TX/RXへ対応付けられないDMAハンドルのエラー。復旧要求には使用しない。
void audio_diagnostics_record_unknown_dma_error(uint32_t error_code);

// 復旧要求・試行・結果。復旧のバッファ消去では消去しない。
void audio_diagnostics_record_recovery_request(uint32_t cause_mask,
                                               uint32_t dma_error_code,
                                               uint32_t sai_error_code,
                                               uint32_t sai_status_flags);
void audio_diagnostics_record_recovery_attempt(void);
void audio_diagnostics_record_recovery_success(void);
void audio_diagnostics_record_recovery_failure(uint32_t failed_stage);
// latched=falseでは連続失敗数もクリアする。
void audio_diagnostics_set_recovery_latched(bool latched);

// SAI error
void audio_diagnostics_record_sai_tx_error(uint32_t error_code,
                                           uint32_t status_flags,
                                           bool streaming,
                                           int32_t used);
void audio_diagnostics_record_sai_rx_error(uint32_t error_code,
                                           uint32_t status_flags);

// USB OUT packet / FIFO / service time
void audio_diagnostics_record_usb_out_packet(uint16_t bytes,
                                             uint32_t rx_cycle,
                                             bool pending);
void audio_diagnostics_record_usb_out_fifo(uint16_t fifo_count);
void audio_diagnostics_reset_usb_out_gap(void);
uint32_t audio_diagnostics_usb_out_pending_cycle(void);
void audio_diagnostics_record_usb_out_service(uint32_t pending_cycle);
void audio_diagnostics_record_usb_out_read(bool streaming, uint16_t bytes);

// USB IN packet / FIFO / write result
void audio_diagnostics_record_usb_in_packet(uint16_t bytes);
void audio_diagnostics_record_usb_in_notify(void);
void audio_diagnostics_record_usb_in_source_wait(void);
void audio_diagnostics_record_usb_in_write(uint16_t written, uint16_t requested);
void audio_diagnostics_record_usb_in_fifo(uint16_t fifo_count);

// 1秒周期のRTTサマリー。Audio Taskから一度だけ呼び出す。
void audio_diagnostics_log_periodic(uint32_t sample_rate_hz,
                                    uint32_t task_frequency_hz,
                                    bool streaming_out,
                                    int32_t tx_used_words);

#endif /* AUDIO_DIAGNOSTICS_INTERNAL_H_ */
