/*
 * audio_diagnostics_internal.h
 *
 * Private recording API for the audio diagnostics counters. The functions are
 * called from USB/DMA ISR context (transport) and from the Audio Task; they
 * only update fixed-size counters, min/max values and timestamps.
 *
 * AUDIO_DIAG_LOG selects the RTT interval summary. AUDIO_DIAG_DMA_TIME_REPORT
 * (measurement builds only) prints the DMA timing once when both streams stop.
 * The defaults live here so every audio module sees the same value from the
 * build flags.
 */

#ifndef AUDIO_DIAGNOSTICS_INTERNAL_H_
#define AUDIO_DIAGNOSTICS_INTERNAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef AUDIO_DIAG_LOG
#define AUDIO_DIAG_LOG 0
#endif

// 1の場合、OUT/INの両方が停止した遷移時に一度だけ、DMA処理時間とOUT診断をRTTへ
// 出力する。周期ログ（AUDIO_DIAG_LOG）を出さずに、CPUから読んだ値で測定するための
// 確認用ビルドで使う。
#ifndef AUDIO_DIAG_DMA_TIME_REPORT
#define AUDIO_DIAG_DMA_TIME_REPORT 0
#endif

// 1の場合、DMA half所有権確認（#251）の試験用遅延注入とトレースを有効にする。
// 試験ビルド専用で、製品ビルドでは0のまま（注入のコードと変数を生成しない）。
#ifndef AUDIO_DMA_OWNERSHIP_TEST
#define AUDIO_DMA_OWNERSHIP_TEST 0
#endif

// DMA half所有権確認（GPDMA一時停止中のcommit/snapshot）の結果。transportと共通の値。
enum
{
    AUDIO_DIAG_DMA_OWN_OK = 0u,           // 所有権を確認してアクセスした
    AUDIO_DIAG_DMA_OWN_BUSY,              // TX: DMAがhalfを読んでいる途中（skip）、RX: 書いている途中（reject）
    AUDIO_DIAG_DMA_OWN_NOT_READY,         // RXのみ: halfの末尾がDMA FIFOに残っている可能性がある
    AUDIO_DIAG_DMA_OWN_UNKNOWN,           // チャネル無効・エラー・不正なBNDT・停止後の状態不一致
    AUDIO_DIAG_DMA_OWN_SUSPEND_TIMEOUT,   // 期限内にSUSPFが立たなかった
    AUDIO_DIAG_DMA_OWN_SUSPEND_FAULT,     // 停止状態異常（ラッチ中、または解除を確かめられない）
    AUDIO_DIAG_DMA_OWN_STREAM_BOUNDARY,   // TXのみ: 未適用のstream要求があり、旧streamの音声を書かなかった
};

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
    // tx_used_*/last_event_used_words: USB OUT FIFO水位（word、half処理開始時＝読出し直前）。
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
    // DMA処理完了時間（DWT->CYCCNT基準、割り込み／Taskプリエンプト時間を含む）。
    // process=処理開始→完了、complete=callback→完了。deadline=DMA half期間
    // （half frame数/実サンプルレート）。complete最大値の更新時はその時点のdeadlineも
    // 同時に保存する。UINT32_MAXは期限なし（レート未確定）。
    uint32_t half_process_cycles_max;
    uint32_t cplt_process_cycles_max;
    uint32_t half_complete_cycles_max;
    uint32_t cplt_complete_cycles_max;
    uint32_t half_complete_deadline_cycles_at_max;
    uint32_t cplt_complete_deadline_cycles_at_max;
    uint32_t half_complete_deadline_overruns;
    uint32_t cplt_complete_deadline_overruns;
    uint32_t last_process_cycles;
    uint32_t last_complete_cycles;
    uint32_t last_complete_deadline_cycles;
    // USB OUT FIFOの直接読出し（#260）。
    uint32_t out_stale_request_skips;          // 未適用のstream要求があり読まなかったhalf数
    uint32_t out_fifo_full_events;             // packet書込み後にFIFOが満杯（上書きの可能性。USB ISRで検出）
    uint32_t out_fifo_overflow_recoveries;     // 満杯・overflowを検出してFIFOをclearした回数
    uint32_t out_fifo_overflow_discard_bytes;  // そのclearでアプリが破棄した有効データ(byte)の累計
    // raw count > depthで観測できた上書き量(byte)の累計。二重overflowで失われた量は
    // 観測できないため、上書きによる累積喪失量の下限として扱う。
    uint32_t out_fifo_overflow_lost_bytes;
    // raw count == depthの満杯でclearした回数（二重overflowの可能性があり喪失量不明）。
    // 二重overflow後にさらに書き込まれるとraw count > depthに戻るため、二重overflowの
    // 全件を数えるものではない。
    uint32_t out_fifo_overflow_unknown_events;
    uint32_t out_reprime_events;               // FIFOが空になりprimingへ戻した回数
    uint32_t out_take_section_cycles_max;      // FIFO読出しPRIMASK区間の最大サイクル数
    // DMA half所有権確認（#251）。作業バッファ→DMA halfのcommitをGPDMA一時停止中に行う。
    uint32_t dma_commit_skips;                   // DMAがhalfを読んでいる途中で書かなかった回数
    uint32_t dma_commit_stream_boundary_events;  // 旧streamの音声を無音へ置き換えた回数
    uint32_t dma_commit_unknown_position_events; // 状態・位置を確かめられず書かなかった回数
    uint32_t dma_commit_entry_q_max;             // committed時の停止位置q（相手halfの先頭から、word）の最大
    uint32_t dma_suspend_timeouts;               // 期限内にSUSPFが立たなかった回数
    uint32_t dma_suspend_stale_flag_clears;      // 開始前に古いSUSPFをクリアした回数
    uint32_t dma_suspend_faults;                 // 停止状態異常（以後アクセスせず復旧を要求）
    uint32_t dma_suspend_cycles_max;             // SUSP書込み→解除完了の最大サイクル数
    uint32_t dma_suspend_over_budget_events;     // 停止期間がframe周期の半分を超えた回数（性能目標の超過）
} audio_tx_diagnostics_t;

extern volatile audio_tx_diagnostics_t g_audio_tx_diagnostics;

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
    uint32_t dma_error_events;
    uint32_t sai_error_events;
    uint32_t last_dma_error_code;
    uint32_t last_sai_error_code;
    uint32_t last_sai_status_flags;
    // DMA処理完了時間。TXと同じ定義（DWT->CYCCNT基準）。RX診断はレート変更をまたいで
    // 永続するため、complete最大値に対応するdeadlineを同時に保存する。
    uint32_t rx_half_process_cycles_max;
    uint32_t rx_cplt_process_cycles_max;
    uint32_t rx_half_complete_cycles_max;
    uint32_t rx_cplt_complete_cycles_max;
    uint32_t rx_half_complete_deadline_cycles_at_max;
    uint32_t rx_cplt_complete_deadline_cycles_at_max;
    uint32_t rx_half_complete_deadline_overruns;
    uint32_t rx_cplt_complete_deadline_overruns;
    uint32_t rx_last_process_cycles;
    uint32_t rx_last_complete_cycles;
    uint32_t rx_last_complete_deadline_cycles;
    // USB IN stream境界とFIFO書込み（#250/#253/#259）。Audio Taskだけが更新する。
    uint32_t usb_in_start_boundaries;        // IN開始境界の適用回数
    uint32_t usb_in_start_discard_words;     // 開始境界で破棄したIN FIFO残量(word)の累計
    uint32_t usb_in_fifo_judged_count_max;   // 最終判定時に観測したIN FIFO水位の最大値(byte)
    uint32_t usb_in_fifo_post_write_max;     // 判定時水位−trim＋書込みbyte数の最大値(byte)。write直後の水位の上界
    uint32_t usb_in_stale_request_skips;     // 未適用のstream要求があり書込みを見送った回数
    uint32_t usb_in_write_zero_events;       // tud_audio_n_write()が0を返した回数（未構成時のみ）
    uint32_t usb_in_write_partial_events;    // 要求量と異なる非0値を返した回数（現行TinyUSBでは0）
    uint32_t usb_in_write_section_cycles_max;  // scheduler停止区間の最大サイクル数(DWT->CYCCNT)
    uint32_t usb_in_fifo_trim_events;        // FIFO上限超過で古いデータを破棄した回数
    uint32_t usb_in_fifo_trim_bytes;         // FIFO上限超過で破棄した古いデータのbyte数累計
    uint32_t usb_in_fifo_full_drops;         // trim後も空き不足で書かなかった回数（防御処理、通常0）
    uint32_t usb_in_chunk_drop_events;       // streaming中に書かなかったチャンクの回数（全原因の合計）
    uint32_t usb_in_chunk_drop_bytes;        // streaming中に書かなかったチャンクのbyte数累計
    // DMA half所有権確認（#251）。DMA half→作業バッファのsnapshotをGPDMA一時停止中に行う。
    uint32_t rx_dma_snapshot_rejects;                // DMAがhalfを書いている途中で読まなかった回数
    uint32_t rx_dma_snapshot_not_ready_events;       // 宛先書込み未完了の可能性で待ってやり直した回数
    uint32_t rx_dma_snapshot_wait_cycles_max;        // そのやり直し前の待ちの最大サイクル数
    uint32_t rx_dma_snapshot_unknown_position_events; // 状態・位置を確かめられず読まなかった回数
    uint32_t rx_dma_snapshot_entry_q_max;            // 読んだ時の停止位置qの最大
    uint32_t rx_dma_suspend_timeouts;
    uint32_t rx_dma_suspend_stale_flag_clears;
    uint32_t rx_dma_suspend_faults;
    uint32_t rx_dma_suspend_cycles_max;
    uint32_t rx_dma_suspend_over_budget_events;
} audio_rx_diagnostics_t;

extern volatile audio_rx_diagnostics_t g_audio_rx_diagnostics;

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
} audio_recovery_diagnostics_t;

extern volatile audio_recovery_diagnostics_t g_audio_recovery_diagnostics;

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
} audio_rate_switch_diagnostics_t;

extern volatile audio_rate_switch_diagnostics_t g_audio_rate_switch_diagnostics;

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
} audio_transport_failure_t;

// 最初の失敗だけを保持する。failureがNULLなら何もしない。
static inline void audio_transport_failure_record(audio_transport_failure_t* failure,
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

// USB OUT FIFO水位（word）。streaming は transport の s_streaming_out を渡す。
void audio_diagnostics_record_tx_level(bool streaming, int32_t used);
void audio_diagnostics_record_tx_interval_level(int32_t used);

// USB OUT FIFOの直接読出し（#260）。full_eventはUSB ISR、それ以外はAudio Task context。
// overflow_recoveryの値はFIFO読出しPRIMASK区間内で取得したものを区間外で渡す。
void audio_diagnostics_record_out_stale_request_skip(void);
void audio_diagnostics_record_out_fifo_full_event(void);
void audio_diagnostics_record_out_fifo_overflow_recovery(uint32_t raw_count_bytes, uint32_t depth_bytes);
void audio_diagnostics_record_out_reprime(void);
void audio_diagnostics_record_out_take_section(uint32_t cycles);

// USB再生priming。待機half数と完了回数を区別して記録する。
void audio_diagnostics_record_tx_priming_wait(void);
void audio_diagnostics_record_tx_priming_complete(void);

// ドリフト補正の閾値逸脱と、最小間隔による補正見送り。
// upward=true は上側（水位過多）、false は下側（水位不足傾向）。
void audio_diagnostics_record_tx_drift_threshold(bool upward);
void audio_diagnostics_record_tx_drift_suppressed(bool upward);

// TXイベント。flags は AUDIO_TX_DIAG_EVENT_* のビット和。used はUSB OUT FIFO水位（word）。
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
// DMA処理完了時間。process_cycles=処理開始→完了、complete_cycles=callback→完了、
// deadline_cycles=そのイベント時点のDMA half期限。complete_cycles > deadline_cyclesで
// 完了期限超過数を加算し、complete最大値の更新時はdeadlineも同時に保存する。
void audio_diagnostics_record_tx_dma_complete(uint32_t callback_event,
                                              uint32_t process_cycles,
                                              uint32_t complete_cycles,
                                              uint32_t deadline_cycles);
void audio_diagnostics_record_rx_dma_complete(uint32_t callback_event,
                                              uint32_t process_cycles,
                                              uint32_t complete_cycles,
                                              uint32_t deadline_cycles);
void audio_diagnostics_record_tx_events_dropped(uint32_t last_event,
                                                uint32_t dropped_events,
                                                int32_t used);
void audio_diagnostics_record_rx_events_dropped(uint32_t last_event,
                                                uint32_t dropped_events);

// USB IN stream境界とFIFO書込み（#250/#253/#259）。Audio Task context only。
// 値はscheduler停止区間内で取得したものを、区間外で渡すこと。
void audio_diagnostics_record_usb_in_start_boundary(uint32_t discarded_words);
// judged_count: 最終判定時のFIFO水位。post_write_bound: 判定時水位−trim＋書込みbyte数（書込みなしは0）。
void audio_diagnostics_record_usb_in_fifo_level(uint32_t judged_count, uint32_t post_write_bound);
void audio_diagnostics_record_usb_in_fifo_trim(uint32_t trimmed_bytes);
void audio_diagnostics_record_usb_in_fifo_full_drop(void);
void audio_diagnostics_record_usb_in_stale_request_skip(void);
// streaming中に書かなかったチャンク（原因別カウンタとは別に合計を数える）。
void audio_diagnostics_record_usb_in_chunk_drop(uint32_t chunk_bytes);
// partial=false: 0を返した。partial=true: 要求量と異なる非0値を返した。
void audio_diagnostics_record_usb_in_write_error(bool partial);
void audio_diagnostics_record_usb_in_write_section(uint32_t cycles);

// DMA half所有権確認（#251）。Audio Task context only。PRIMASK区間の外で、区間内に
// 取得した値を渡す。result: AUDIO_DIAG_DMA_OWN_*。q: 停止位置（OKのときだけ有効）。
// suspend_cycles: SUSP書込み→解除完了（停止を要求しなかった経路は0）。
// over_budget: suspend_cyclesがframe周期の半分を超えた。fault: 停止状態異常を新たに検出した。
void audio_diagnostics_record_tx_dma_ownership(uint32_t result,
                                               uint32_t q,
                                               uint32_t suspend_cycles,
                                               bool over_budget,
                                               bool stale_flag_cleared,
                                               bool fault);
void audio_diagnostics_record_rx_dma_ownership(uint32_t result,
                                               uint32_t q,
                                               uint32_t suspend_cycles,
                                               bool over_budget,
                                               bool stale_flag_cleared,
                                               bool fault);
// TX: 旧streamの音声を無音へ置き換えた。RX: not_readyで待ってやり直した（wait_cycles: 待ち時間）。
void audio_diagnostics_record_tx_dma_stream_boundary(void);
void audio_diagnostics_record_rx_dma_not_ready(uint32_t wait_cycles);

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

// USB OUT packet / FIFO / feedback
void audio_diagnostics_record_usb_out_packet(uint16_t bytes, uint32_t rx_cycle);
void audio_diagnostics_record_usb_out_fifo(uint16_t fifo_count);
void audio_diagnostics_reset_usb_out_gap(void);
// アプリ側feedback値（16.16）。USB ISR context。
void audio_diagnostics_record_usb_out_feedback(uint32_t feedback);

// USB IN packet / FIFO / write result
void audio_diagnostics_record_usb_in_packet(uint16_t bytes);
void audio_diagnostics_record_usb_in_write(uint16_t written, uint16_t requested);
void audio_diagnostics_record_usb_in_fifo(uint16_t fifo_count);

// AUDIO_DIAG_DMA_TIME_REPORT=1の場合だけ有効。OUT/INの両方が停止した遷移時に
// Audio Taskから一度だけ呼び出し、DMA処理時間とOUT診断を出力する。
void audio_diagnostics_report_dma_time(void);

// 1秒周期のRTTサマリー。Audio Taskから一度だけ呼び出す。
// streaming_out/streaming_inが両方falseの場合は出力しない。
void audio_diagnostics_log_periodic(uint32_t sample_rate_hz,
                                    uint32_t task_frequency_hz,
                                    bool streaming_out,
                                    bool streaming_in,
                                    int32_t tx_used_words);

#endif /* AUDIO_DIAGNOSTICS_INTERNAL_H_ */
