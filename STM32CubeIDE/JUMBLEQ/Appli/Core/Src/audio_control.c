/*
 * audio_control.c
 *
 * Audio subsystem facade and Audio Task arbitration.
 *
 * Owns the applied sample-rate state and the top-level Audio Task that applies
 * USB requests. The UAC2 control plane lives in audio_usb_control.c, realtime
 * data movement in audio_transport.c, EEPROM settings in eeprom_config.c.
 */

#include "audio_control.h"
#include "audio_control_internal.h"
#include "audio_diagnostics_internal.h"
#include "audio_transport_internal.h"
#include "audio_usb_control_internal.h"
#include "eeprom_config_internal.h"
#include "ui_control_internal.h"
#include "ui_routing_control_internal.h"
#include "timecode_synth.h"

#include "adc.h"
#include "hpdma.h"
#include "i2c.h"
#include "linked_list.h"
#include "eeprom.h"

#include "adau1466.h"
#include "SEGGER_RTT.h"

#include "cmsis_os2.h"

enum
{
    AUDIO_TASK_STATS_PERIOD_MS = 1000u,
    // DMA復旧: TX開始からRX開始までの同期待ち（既存レート変更と同じ10ms）
    AUDIO_RECOVERY_TX_SYNC_DELAY_MS = 10u,
    // 連続失敗時の再試行間隔
    AUDIO_RECOVERY_BACKOFF_MS = 100u,
    // この回数だけ連続失敗したらFAILEDへラッチし、両経路停止・無音を維持する
    AUDIO_RECOVERY_MAX_CONSECUTIVE_FAILURES = 3u,
    // commit区間mutexの取得待ち上限。commit自体は短く、通常は即時取得できる
    AUDIO_COMMIT_LOCK_TIMEOUT_MS = 20u,
};

extern DMA_QListTypeDef List_HPDMA1_Channel0;

// DMA/SAIエラーからの復旧状態。ISR共有状態とは分離し、状態遷移はAudio Taskだけが行う。
typedef enum
{
    AUDIO_RECOVERY_STATE_IDLE = 0,
    AUDIO_RECOVERY_STATE_PREPARE,
    AUDIO_RECOVERY_STATE_WAIT_TX_SYNC,
    AUDIO_RECOVERY_STATE_BACKOFF,
    AUDIO_RECOVERY_STATE_FAILED,
} audio_recovery_state_t;

static audio_recovery_state_t s_recovery_state = AUDIO_RECOVERY_STATE_IDLE;
static uint32_t s_recovery_request_sequence = 0u;
static bool s_recovery_request_in_flight = false;
static uint32_t s_recovery_wait_start_tick = 0u;
static uint32_t s_recovery_backoff_start_tick = 0u;
static uint32_t s_recovery_consecutive_failures = 0u;

// サンプルレートの要求状態(USB Taskが更新)と適用状態(Audio Taskが更新)。
// 要求はレートとsequenceを一組でpublishし、Audio Taskは同じcritical sectionで
// 両方をsnapshotして処理対象を固定する。適用済みレートは「最後に切り替え全体が
// 成功した値」、有効フラグは「現在もその設定を信用できるか」を表す。
typedef struct
{
    volatile uint32_t requested_hz;        // 最後に受理した要求値。適用完了前でも更新される
    volatile uint32_t requested_sequence;  // 受理ごとに増加する要求sequence
    volatile bool change_pending;          // Audio Taskが処理すべき変更要求がある
    uint32_t applied_hz;                   // 最後に切り替え全体が成功した値
    uint32_t applied_sequence;             // 適用が確定した要求sequence
    bool applied_hz_valid;                 // 現在もその設定を信用できるか
    volatile uint32_t state;               // AudioRateState_t
} audio_sample_rate_state_t;

static audio_sample_rate_state_t s_sample_rate = {
    .requested_hz       = 48000U,
    .requested_sequence = 0U,
    .change_pending     = false,
    .applied_hz         = 48000U,
    .applied_sequence   = 0U,
    .applied_hz_valid   = false,  // 初期適用確認まで無効
    .state              = AUDIO_RATE_STATE_SWITCHING,
};

// ISR/USB callbackから参照する物理搬送レートと搬送許可。READY時は適用値、
// 切り替え中は固定targetを保持する（複合状態をatomicに読める単純値として公開）。
static volatile uint32_t s_transport_rate_hz = 48000U;
static volatile bool s_transport_rate_ready = false;
// 要求受理から処理開始までの未処理期間gate。受理直後から通常搬送を禁止し、
// Audio Taskが処理対象を固定した時点で解除する。
static volatile bool s_rate_change_unprocessed = false;
// DMA/SAIが停止確認を経て安全な状態にあるか。停止未確認のまま復旧がBACKOFF/FAILEDへ
// 入った場合、DMA参照バッファを通常搬送で再利用しないためのgate。
static volatile bool s_transport_paths_safe = true;

// 搬送のcommit区間とレート要求publishを直列化するmutex。時間のかかるTinyUSB APIを
// 割り込み禁止区間へ入れずに排他する。Audio Task起動時に生成する。
static osMutexId_t s_commit_mutex = NULL;
static const osMutexAttr_t s_commit_mutex_attributes = {
    .name = "audioCommit",
};

static void audio_rate_switch_process(uint32_t target_hz, uint32_t target_sequence);
static void audio_recovery_process(void);
static void audio_recovery_release_latch(void);
static void audio_recovery_reset_after_rate_change(void);

#if RESET_FROM_FW
// DSPレート適用の失敗ステップを診断の操作IDへ変換する。
static uint32_t audio_rate_dsp_step_op(adau1466_rate_step_t step)
{
    switch (step)
    {
        case ADAU1466_RATE_STEP_MUX:         return AUDIO_RATE_OP_DSP_MUX;
        case ADAU1466_RATE_STEP_SOUT_SOURCE: return AUDIO_RATE_OP_DSP_SOUT_SOURCE;
        case ADAU1466_RATE_STEP_CLK_GEN:     return AUDIO_RATE_OP_DSP_CLK_GEN;
        case ADAU1466_RATE_STEP_PLL_READ:    return AUDIO_RATE_OP_DSP_PLL_LOCK;
        default:                             return AUDIO_RATE_OP_NONE;
    }
}
#endif

void audio_control_request_sample_rate(uint32_t sample_rate_hz)
{
#if AUDIO_DIAG_LOG
    const uint32_t previous_requested_hz = s_sample_rate.requested_hz;
    const bool same_as_applied = s_sample_rate.applied_hz_valid &&
                                 (sample_rate_hz == s_sample_rate.applied_hz);
    SEGGER_RTT_printf(0,
                      "[AUD][RATE-REQ] tick=%lu requested=%lu previous_requested=%lu applied=%lu applied_valid=%u same_as_applied=%u\r\n",
                      (unsigned long) HAL_GetTick(),
                      (unsigned long) sample_rate_hz,
                      (unsigned long) previous_requested_hz,
                      (unsigned long) s_sample_rate.applied_hz,
                      s_sample_rate.applied_hz_valid ? 1u : 0u,
                      same_as_applied ? 1u : 0u);
#endif

    // commit区間と排他に受理する。commit中の場合はmutexを取得できるまで待ってから
    // sequenceを進めるため、「受理後に旧commitが残る」状態を作らない。
    const bool commit_locked = audio_control_commit_lock_wait();

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    s_sample_rate.requested_hz = sample_rate_hz;
    s_sample_rate.requested_sequence++;
    // 受理直後から通常搬送を禁止する。no-op処理または切り替え完了で再許可する。
    s_rate_change_unprocessed = true;
    __DMB();
    s_sample_rate.change_pending = true;

    __set_PRIMASK(primask);

    if (commit_locked)
    {
        audio_control_commit_unlock();
    }

    audio_transport_notify_task();
}

// 処理対象のレートとsequenceを同じcritical sectionで固定する。処理中に届いた
// 新しい要求は別sequenceとしてpendingに残り、次の呼出しで最新要求へ集約される。
static bool audio_rate_change_take_pending(uint32_t* target_hz, uint32_t* target_sequence)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const bool pending = s_sample_rate.change_pending;
    if (pending)
    {
        *target_hz       = s_sample_rate.requested_hz;
        *target_sequence = s_sample_rate.requested_sequence;
        s_sample_rate.change_pending = false;
        // 処理対象を固定したので未処理期間gateを解除する。処理中に新要求が届いた
        // 場合は受理側が再びgateを立て、次回処理まで通常搬送を止める。
        s_rate_change_unprocessed = false;
    }

    __set_PRIMASK(primask);
    return pending;
}

void audio_control_get_rate_snapshot(AudioRateSnapshot_t* snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    snapshot->requested_hz       = s_sample_rate.requested_hz;
    snapshot->requested_sequence = s_sample_rate.requested_sequence;
    snapshot->applied_hz         = s_sample_rate.applied_hz;
    snapshot->applied_sequence   = s_sample_rate.applied_sequence;
    snapshot->applied_hz_valid   = s_sample_rate.applied_hz_valid;
    snapshot->state              = (AudioRateState_t) s_sample_rate.state;

    __set_PRIMASK(primask);
}

bool audio_control_clock_valid(void)
{
    AudioRateSnapshot_t snapshot;
    audio_control_get_rate_snapshot(&snapshot);

    return (snapshot.state == AUDIO_RATE_STATE_READY) &&
           snapshot.applied_hz_valid &&
           (snapshot.applied_sequence == snapshot.requested_sequence) &&
           (snapshot.applied_hz == snapshot.requested_hz);
}

uint32_t audio_control_transport_sample_rate_hz(void)
{
    return s_transport_rate_hz;
}

bool audio_control_transport_ready(void)
{
    // レート許可に加え、受理済みで未処理の要求がある間は通常搬送を許可しない。
    return s_transport_rate_ready && !s_rate_change_unprocessed;
}

bool audio_control_transport_paths_safe(void)
{
    return s_transport_paths_safe;
}

uint32_t audio_control_rate_request_sequence(void)
{
    return s_sample_rate.requested_sequence;
}

bool audio_control_transport_commit_allowed(uint32_t request_sequence)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // 要求publishと同じ排他で評価し、受理済みの新要求があればcommitを許可しない。
    const bool allowed = s_transport_rate_ready &&
                         !s_rate_change_unprocessed &&
                         s_transport_paths_safe &&
                         (s_sample_rate.requested_sequence == request_sequence);

    __set_PRIMASK(primask);
    return allowed;
}

bool audio_control_commit_lock(void)
{
    if (s_commit_mutex == NULL)
    {
        // Audio Task起動前はcommit自体が走らないため、排他なしでも安全。
        return false;
    }

    return osMutexAcquire(s_commit_mutex, pdMS_TO_TICKS(AUDIO_COMMIT_LOCK_TIMEOUT_MS)) == osOK;
}

bool audio_control_commit_lock_wait(void)
{
    if (s_commit_mutex == NULL)
    {
        // Audio Task起動前はcommit自体が走らないため、排他なしでも安全。
        return false;
    }

    // 取得失敗で受理すると旧commitが残るため、取得できるまで待つ。
    // commit区間は短く、待機中の優先度継承でAudio Taskが完了させる。
    return osMutexAcquire(s_commit_mutex, osWaitForever) == osOK;
}

void audio_control_commit_unlock(void)
{
    if (s_commit_mutex != NULL)
    {
        (void) osMutexRelease(s_commit_mutex);
    }
}

void audio_control_publish_initial_rate_result(uint32_t applied_hz, bool success)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (success)
    {
        s_sample_rate.state            = AUDIO_RATE_STATE_READY;
        s_sample_rate.applied_hz       = applied_hz;
        // 最新要求が初期レートと一致する場合だけ、そのsequenceを適用済みとして記録する。
        s_sample_rate.applied_sequence = (s_sample_rate.requested_hz == applied_hz) ?
                                             s_sample_rate.requested_sequence : 0U;
        s_sample_rate.applied_hz_valid = true;
        s_transport_rate_hz            = applied_hz;
        s_transport_rate_ready         = true;
    }
    else
    {
        // 初期適用失敗: 最後の適用値は履歴として残すが信用せず、搬送を許可しない。
        s_sample_rate.state            = AUDIO_RATE_STATE_FAILED;
        s_sample_rate.applied_hz_valid = false;
        s_transport_rate_hz            = s_sample_rate.applied_hz;
        s_transport_rate_ready         = false;
    }

    __set_PRIMASK(primask);

    audio_diagnostics_update_rate_snapshot(s_sample_rate.state,
                                           s_sample_rate.applied_hz,
                                           s_sample_rate.applied_hz_valid,
                                           audio_control_clock_valid());

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
    if (success)
    {
        // 初回送信前に適用レートの公称feedback値を設定しておく。
        audio_usb_control_feedback_reset(applied_hz);
    }
#endif
}

// EEPROMからデバイス設定全体を読み込み、DSP/UIへ適用する。読込失敗または設定不正時は
// デフォルト設定を適用し、EEPROMへ保存して次回起動に備える。
void audio_control_load_config_or_restore_defaults(void)
{
    EEPROM_DeviceConfig_t cfg;

    if (EEPROM_LoadConfig(&hi2c2, &cfg) == HAL_OK)
    {
        if (eeprom_config_apply(&cfg))
        {
            SEGGER_RTT_printf(0,
                              "EEPROM routing applied: CH1=%u CH2=%u CH_FADER_A=%u CH_FADER_B=%u CH_FADER_POST=%u RTN=%u HP=%u MODE1=%u MODE2=%u DVS_DELAY_MS=%u AUX2=%u AUX3=%u REVERSE_A=%u REVERSE_B=%u CURVE_WIDTH_A=%.4f CURVE_WIDTH_B=%.4f RATIO=%u WARP=%u\r\n",
                              (unsigned)cfg.current_ch1_input_type,
                              (unsigned)cfg.current_ch2_input_type,
                              (unsigned)cfg.current_ch_fader_a_assign,
                              (unsigned)cfg.current_ch_fader_b_assign,
                              (unsigned)cfg.current_ch_fader_post_assign,
                              (unsigned)cfg.current_return_assign,
                              (unsigned)cfg.current_hp_out_source,
                              (unsigned)cfg.current_ch1_input_mode,
                              (unsigned)cfg.current_ch2_input_mode,
                              (unsigned)cfg.ch_fader_dvs_delay_ms,
                              (unsigned)cfg.sensor2_aux_fade_down_assign,
                              (unsigned)cfg.sensor3_aux_fade_down_assign,
                              (unsigned)eeprom_config_is_ch_fader_reverse_a_enabled(&cfg),
                              (unsigned)eeprom_config_is_ch_fader_reverse_b_enabled(&cfg),
                              (double)cfg.current_ch_fader_curve_width_a,
                              (double)cfg.current_ch_fader_curve_width_b,
                              (unsigned)cfg.timecode_synth_ratio_set,
                              (unsigned)cfg.timecode_synth_warp_algorithm);
            return;
        }

        SEGGER_RTT_printf(0, "EEPROM config invalid; restoring defaults\r\n");
    }
    else
    {
        SEGGER_RTT_printf(0, "EEPROM config load failed; restoring defaults\r\n");
    }

    EEPROM_ConfigSetDefaults(&cfg);
    if (!eeprom_config_apply(&cfg))
    {
        SEGGER_RTT_printf(0, "EEPROM default routing apply failed\r\n");
        return;
    }

    if (EEPROM_SaveConfig(&hi2c2, &cfg) == HAL_OK)
    {
        SEGGER_RTT_printf(0, "EEPROM config initialized with defaults\r\n");
    }
    else
    {
        SEGGER_RTT_printf(0, "EEPROM default config save failed\r\n");
    }
}

// 最後に受理した設定要求値（要求レート）を返す。物理的な適用成功の通知ではない。
// USB GET_CUR、feedback初期設定、UI表示はこの要求値を共用する。実搬送のレートには
// audio_control_transport_sample_rate_hz()（適用値／切替中の固定target）を使う。
uint32_t audio_control_requested_sample_rate_hz(void)
{
    return s_sample_rate.requested_hz;
}

// UI状態、タイムコード合成状態、トランスポートバッファを含む実行時状態を初期化する。
// バッファ消去のみを行う audio_transport_reset_buffers() とは範囲が異なる。
void audio_control_reset_runtime_state(void)
{
    ui_control_reset_state();

    timecode_synth_init(s_sample_rate.requested_hz);

    audio_transport_reset_buffers();
}

void audio_control_register_task(void)
{
    if (s_commit_mutex == NULL)
    {
        s_commit_mutex = osMutexNew(&s_commit_mutex_attributes);
    }

    audio_transport_register_current_task();
}

void start_sai(void)
{
    audio_transport_start();
}

// audio_task()呼び出し頻度計測用
static volatile uint32_t audio_task_call_count = 0;
static volatile uint32_t audio_task_last_tick  = 0;
static volatile uint32_t audio_task_frequency  = 0;  // 呼び出し回数/秒
// 復旧ラッチを解除して再試行可能にする。stream再開要求から呼ぶ。
static void audio_recovery_release_latch(void)
{
    if (s_recovery_state == AUDIO_RECOVERY_STATE_FAILED)
    {
        s_recovery_consecutive_failures = 0u;
        audio_diagnostics_set_recovery_latched(false);

        if (s_recovery_request_in_flight)
        {
            // 保持中のin-flight要求をそのまま再試行する。takeし直すと同じ
            // sequenceを空のpayloadで再記録してしまう。
            s_recovery_state = AUDIO_RECOVERY_STATE_PREPARE;
        }
        else
        {
            s_recovery_state = AUDIO_RECOVERY_STATE_IDLE;
        }
    }
}

// レート変更がSAI/GPDMAを再構築・再始動した後に状態機械を通常へ戻す。
static void audio_recovery_reset_after_rate_change(void)
{
    s_recovery_state = AUDIO_RECOVERY_STATE_IDLE;
    // レート変更がpending要求をtake+ackして再構築済みのためin-flightは解消する。
    s_recovery_request_in_flight = false;
    s_recovery_consecutive_failures = 0u;
    audio_diagnostics_set_recovery_latched(false);
    s_transport_paths_safe = true;
}

static void audio_recovery_on_failure(uint32_t failed_stage)
{
    audio_diagnostics_record_recovery_failure(failed_stage);
    // 停止確認に失敗した経路はDMAが動作中の可能性があるため、通常搬送で
    // 参照バッファへ触れない。
    s_transport_paths_safe = false;
    s_recovery_consecutive_failures++;

    if (s_recovery_consecutive_failures >= AUDIO_RECOVERY_MAX_CONSECUTIVE_FAILURES)
    {
        // transport側プリミティブが両経路を停止済み。無音のままラッチする。
        audio_diagnostics_set_recovery_latched(true);
        s_recovery_state = AUDIO_RECOVERY_STATE_FAILED;
        SEGGER_RTT_printf(0,
                          "[AUD] DMA recovery failed at stage %lu; latched after %lu failures\n",
                          (unsigned long) failed_stage,
                          (unsigned long) s_recovery_consecutive_failures);
    }
    else
    {
        s_recovery_backoff_start_tick = HAL_GetTick();
        s_recovery_state = AUDIO_RECOVERY_STATE_BACKOFF;
    }
}

// PREPARE: 停止・初期化・DMA/SAI再構築・TX開始を実行して次の状態へ進む。
// 復旧はSAIのMSP資源と設定を維持し、生成MspInit（fatal要因）を通らない。
static void audio_recovery_begin_attempt(void)
{
    s_recovery_state = AUDIO_RECOVERY_STATE_PREPARE;
    // 停止確認が完了するまでDMA参照バッファを通常搬送で再利用しない。
    s_transport_paths_safe = false;

    const bool stopped = audio_transport_stop_and_clear_paths(NULL);
    audio_diagnostics_record_recovery_attempt();

    if (!stopped)
    {
        // 停止完了を確認できない場合は再構築せず、backoff後に再試行する。
        // 参照バッファは消去されていない。
        audio_recovery_on_failure(AUDIO_RECOVERY_STAGE_PREPARE);
        return;
    }

    if (audio_transport_rebuild_and_start_tx(NULL))
    {
        s_recovery_wait_start_tick = HAL_GetTick();
        s_recovery_state = AUDIO_RECOVERY_STATE_WAIT_TX_SYNC;
    }
    else
    {
        audio_recovery_on_failure(AUDIO_RECOVERY_STAGE_TX_START);
    }
}

// Audio Taskの復旧状態機械。要求の受理、10ms同期待ち、backoff、ラッチを進める。
static void audio_recovery_process(void)
{
    switch (s_recovery_state)
    {
        case AUDIO_RECOVERY_STATE_IDLE:
        {
            AudioRecoveryRequest_t request;
            if (!audio_transport_take_recovery_request(&request))
            {
                break;
            }

            s_recovery_request_sequence = request.sequence;
            s_recovery_request_in_flight = true;
            audio_diagnostics_record_recovery_request(request.cause_mask,
                                                      request.dma_error_code,
                                                      request.sai_error_code,
                                                      request.sai_status_flags);
            SEGGER_RTT_printf(0,
                              "[AUD] DMA recovery requested: cause=0x%02lX dma=0x%08lX sai=0x%08lX sr=0x%08lX\n",
                              (unsigned long) request.cause_mask,
                              (unsigned long) request.dma_error_code,
                              (unsigned long) request.sai_error_code,
                              (unsigned long) request.sai_status_flags);
            s_recovery_state = AUDIO_RECOVERY_STATE_PREPARE;
        }
            /* fall through */
        case AUDIO_RECOVERY_STATE_PREPARE:
            audio_recovery_begin_attempt();
            break;

        case AUDIO_RECOVERY_STATE_WAIT_TX_SYNC:
            if (HAL_GetTick() - s_recovery_wait_start_tick >= AUDIO_RECOVERY_TX_SYNC_DELAY_MS)
            {
                if (audio_transport_start_rx_after_tx_sync(NULL))
                {
                    audio_transport_ack_recovery_request(s_recovery_request_sequence);
                    s_recovery_request_in_flight = false;
                    audio_diagnostics_record_recovery_success();
                    s_recovery_consecutive_failures = 0u;
                    s_recovery_state = AUDIO_RECOVERY_STATE_IDLE;
                    // 停止確認済みの再構築が完了したため通常搬送を再許可する。
                    // 復旧期間中に残ったTinyUSBデータは送信・primingへ持ち越さない。
                    audio_transport_clear_usb_fifos();
#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
                    // 復旧中にfeedbackが最大値へ張り付かないよう、適用レートで再初期化する。
                    audio_usb_control_feedback_reset(audio_control_transport_sample_rate_hz());
#endif
                    s_transport_paths_safe = true;
                    SEGGER_RTT_printf(0, "[AUD] DMA recovery completed\n");
                }
                else
                {
                    audio_recovery_on_failure(AUDIO_RECOVERY_STAGE_RX_START);
                }
            }
            break;

        case AUDIO_RECOVERY_STATE_BACKOFF:
            if (HAL_GetTick() - s_recovery_backoff_start_tick >= AUDIO_RECOVERY_BACKOFF_MS)
            {
                audio_recovery_begin_attempt();
            }
            break;

        case AUDIO_RECOVERY_STATE_FAILED:
        default:
            break;
    }
}

void audio_task(void)
{
    // USBスタック初期化前にtud_* APIへ入らないようにする。
    if (!tud_inited())
    {
        audio_transport_clear_pending_events();
        return;
    }

    // USBコールバックは要求だけを発行し、共有状態の変更はAudio Taskへ集約する。
    if (audio_transport_apply_requested_stream_state())
    {
        audio_usb_control_set_tx_stream_blink(audio_transport_is_output_streaming());
        audio_usb_control_set_rx_stream_blink(audio_transport_is_input_streaming());

        if (audio_transport_is_output_streaming() || audio_transport_is_input_streaming())
        {
            // stream再開要求を契機に復旧ラッチを解除して再試行できるようにする。
            // レートFAILED/未確定中は解除せず、DMAだけの再始動を許可しない。
            if (audio_control_transport_ready())
            {
                audio_recovery_release_latch();
            }
        }
    }

    // サンプルレート変更とDMA復旧は同じ呼出しで並行実行しない。
    uint32_t target_hz = 0U;
    uint32_t target_sequence = 0U;
    if (audio_rate_change_take_pending(&target_hz, &target_sequence))
    {
        // 切り替え中に受理した新しいSET_CURは別sequenceでpendingに残り、次の
        // 呼出しで処理する。成功しても、その要求のCLK_VALIDは0のままとする。
        audio_rate_switch_process(target_hz, target_sequence);
    }
    else
    {
        // レート状態を優先し、FAILED/未確定の間にDMAだけの再始動をしない。
        if (audio_control_transport_ready())
        {
            audio_recovery_process();
        }

        // DMA half処理をTinyUSB FIFO操作より先に行う。レート未確定・FAILED中の
        // gateはtransport側で行い、USB OUTは読み捨てる。
        audio_transport_service(audio_control_transport_sample_rate_hz());

        // timecodeの非緊急更新はDMA搬送後に行う。
        timecode_synth_update();
        timecode_synth_set_channel_enabled(
            0u, get_current_ch1_input_mode() == UI_INPUT_MODE_SYNTH);
        timecode_synth_set_channel_enabled(
            1u, get_current_ch2_input_mode() == UI_INPUT_MODE_SYNTH);
    }

    // 呼び出し頻度計測と周期診断は最も低い優先度で行う。
    audio_task_call_count++;
    uint32_t now = HAL_GetTick();
    if (now - audio_task_last_tick >= AUDIO_TASK_STATS_PERIOD_MS)
    {
        audio_task_frequency  = audio_task_call_count;
        audio_task_call_count = 0;
        audio_task_last_tick  = now;

#if AUDIO_DIAG_LOG
        audio_diagnostics_log_periodic(s_sample_rate.requested_hz,
                                       audio_task_frequency,
                                       audio_transport_is_output_streaming(),
                                       audio_transport_is_input_streaming(),
                                       audio_transport_tx_used_words());
        audio_diagnostics_reset_interval();
#endif
    }
}

// ADC/HPDMAを再構成してUI用ADCを戻す。成功・失敗の共通cleanupでも使う。
// 失敗時はfailureへ失敗した操作とHAL結果を格納する。
static bool audio_rate_switch_restart_adc(AudioTransportFailure_t* failure)
{
    HAL_StatusTypeDef status = MX_List_HPDMA1_Channel0_Config();
    if (status != HAL_OK)
    {
        audio_transport_failure_record(failure, AUDIO_RATE_OP_ADC_RESTART_LIST_CONFIG,
                                       (uint32_t) status, handle_HPDMA1_Channel0.ErrorCode);
        return false;
    }
    status = HAL_DMAEx_List_LinkQ(&handle_HPDMA1_Channel0, &List_HPDMA1_Channel0);
    if (status != HAL_OK)
    {
        audio_transport_failure_record(failure, AUDIO_RATE_OP_ADC_RESTART_LINKQ,
                                       (uint32_t) status, handle_HPDMA1_Channel0.ErrorCode);
        return false;
    }
    handle_HPDMA1_Channel0.XferCpltCallback = ui_control_dma_adc_cplt;
    status = HAL_DMAEx_List_Start_IT(&handle_HPDMA1_Channel0);
    if (status != HAL_OK)
    {
        audio_transport_failure_record(failure, AUDIO_RATE_OP_ADC_RESTART_START,
                                       (uint32_t) status, handle_HPDMA1_Channel0.ErrorCode);
        return false;
    }
    status = HAL_ADC_Start(&hadc1);
    if (status != HAL_OK)
    {
        // HPDMAだけ起動済みの状態を残さない。停止を確認できない場合は
        // 次のレート要求の停止処理で改めてabortする。
        audio_transport_failure_record(failure, AUDIO_RATE_OP_ADC_RESTART_ADC,
                                       (uint32_t) status, handle_HPDMA1_Channel0.ErrorCode);
        (void) audio_transport_dma_abort_confirmed(&handle_HPDMA1_Channel0);
        return false;
    }
    return true;
}

// サンプルレート切り替え本体。Audio Task context only。
// 停止・DSP更新・再構築・再開を順に行い、全手順成功した場合だけ適用値とREADYを
// commitして通常搬送を許可する。FAILEDでは同じレート要求でも全手順をやり直す。
static void audio_rate_switch_process(uint32_t target_hz, uint32_t target_sequence)
{
    // READYかつ有効な同一レート要求だけをno-opにする。no-opでは復旧状態に触れず、
    // DMA復旧要求のackやFAILED解除を偽装しない。
    if ((s_sample_rate.state == AUDIO_RATE_STATE_READY) && s_sample_rate.applied_hz_valid &&
        (target_hz == s_sample_rate.applied_hz))
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_sample_rate.applied_sequence = target_sequence;
        __set_PRIMASK(primask);

        audio_diagnostics_record_rate_switch_noop(target_hz, target_sequence);
        audio_diagnostics_update_rate_snapshot(s_sample_rate.state,
                                               s_sample_rate.applied_hz,
                                               s_sample_rate.applied_hz_valid,
                                               audio_control_clock_valid());
#if AUDIO_DIAG_LOG
        SEGGER_RTT_printf(0,
                          "[AUD][RATE-APPLY] tick=%lu action=no-op requested=%lu applied=%lu\r\n",
                          (unsigned long) HAL_GetTick(),
                          (unsigned long) target_hz,
                          (unsigned long) s_sample_rate.applied_hz);
#endif
        return;
    }

    // 切り替え開始を公開: 通常搬送を禁止し、適用値は「現在も信用できるか」を落とす。
    // 物理レート参照は固定targetへ切り替える。
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_sample_rate.state            = AUDIO_RATE_STATE_SWITCHING;
        s_sample_rate.applied_hz_valid = false;
        s_transport_rate_hz            = target_hz;
        s_transport_rate_ready         = false;
        __set_PRIMASK(primask);
    }

    audio_diagnostics_record_rate_switch_attempt(target_hz, target_sequence);
    audio_diagnostics_update_rate_snapshot(AUDIO_RATE_STATE_SWITCHING,
                                           s_sample_rate.applied_hz, false, false);

#if AUDIO_DIAG_LOG
    SEGGER_RTT_printf(0,
                      "[AUD][RATE-APPLY] tick=%lu action=switch-begin requested=%lu applied=%lu\r\n",
                      (unsigned long) HAL_GetTick(),
                      (unsigned long) target_hz,
                      (unsigned long) s_sample_rate.applied_hz);
#endif

    // 切り替え開始前に受理済みの復旧要求を固定する。成功時だけackし、切り替え中に
    // 届いた新しい要求はpendingのまま残す（古い成功で消さない）。
    const uint32_t recovery_sequence_at_start = audio_transport_recovery_request_sequence();

    uint32_t failed_result     = 0U;
    uint32_t failed_stage      = AUDIO_RATE_STAGE_NONE;
    uint32_t failed_hal_status = 0U;
    uint32_t failed_aux_status = 0U;

    // 1. ADC/HPDMA停止。DSPパラメータ書込みとの競合を避ける。
    const HAL_StatusTypeDef adc_stop_status = HAL_ADC_Stop(&hadc1);
    const bool hpdma_stopped = audio_transport_dma_abort_confirmed(&handle_HPDMA1_Channel0);
    ui_control_set_adc_complete(false);
    __DSB();

    if ((adc_stop_status != HAL_OK) || !hpdma_stopped)
    {
        failed_stage = AUDIO_RATE_STAGE_ADC_STOP;
        if (adc_stop_status != HAL_OK)
        {
            failed_hal_status = (uint32_t) adc_stop_status;
            failed_aux_status = audio_rate_aux_pack(AUDIO_RATE_OP_ADC_STOP, 0u);
        }
        else
        {
            failed_hal_status = (uint32_t) HAL_ERROR;
            failed_aux_status = audio_rate_aux_pack(AUDIO_RATE_OP_HPDMA_STOP,
                                                    handle_HPDMA1_Channel0.ErrorCode);
        }
    }

    // 2. SAI/GPDMA停止とバッファ初期化（停止確認できた場合のみ初期化する）。
    if (failed_stage == AUDIO_RATE_STAGE_NONE)
    {
        AudioTransportFailure_t transport_failure = {0};
        if (!audio_transport_reset_for_sample_rate(target_hz, &transport_failure))
        {
            failed_stage      = AUDIO_RATE_STAGE_TRANSPORT_STOP;
            failed_hal_status = transport_failure.hal_status;
            failed_aux_status = audio_rate_aux_pack(transport_failure.operation,
                                                    transport_failure.error_code);
        }
    }

#if RESET_FROM_FW
    // 3. DSPのASRC/direct経路切替。DSP coreとAK4619は96 kHzのまま維持する。
    if (failed_stage == AUDIO_RATE_STAGE_NONE)
    {
        adau1466_rate_failure_t dsp_failure = {0};
        if (!AUDIO_Update_ADAU1466_SampleRate_Checked(target_hz, &dsp_failure))
        {
            failed_stage = AUDIO_RATE_STAGE_DSP_UPDATE;
            failed_result = (uint32_t) dsp_failure.reason;
            if (dsp_failure.reason == ADAU1466_RATE_FAIL_PLL_TIMEOUT)
            {
                // SPI読み出し自体は成功していてもPLLがロックしない場合があるため、
                // SPI成功として記録せず、理由とHAL結果を分けて残す。
                failed_hal_status = (uint32_t) HAL_TIMEOUT;
                failed_aux_status = audio_rate_aux_pack(AUDIO_RATE_OP_DSP_PLL_LOCK,
                                                        dsp_failure.spi_hal_error);
            }
            else if (dsp_failure.reason == ADAU1466_RATE_FAIL_UNSUPPORTED)
            {
                failed_hal_status = (uint32_t) HAL_ERROR;
                failed_aux_status = audio_rate_aux_pack(AUDIO_RATE_OP_DSP_UNSUPPORTED, 0u);
            }
            else
            {
                // SPI失敗・読み出し失敗は、その呼出し固有の結果／HAL status／ErrorCode。
                failed_result     = dsp_failure.spi_result;
                failed_hal_status = dsp_failure.spi_hal_status;
                failed_aux_status = audio_rate_aux_pack(audio_rate_dsp_step_op(dsp_failure.step),
                                                        dsp_failure.spi_hal_error);
            }
        }
    }
#else
    // USBiデバッグ経路(RESET_FROM_FW=0)はDSPを切り替えないため、新レートの適用成功と
    // 扱わない（DSP更新の未実行を0xFFFFFFFFで記録する）。
    if (failed_stage == AUDIO_RATE_STAGE_NONE)
    {
        failed_stage      = AUDIO_RATE_STAGE_DSP_UPDATE;
        failed_hal_status = 0xFFFFFFFFU;
        failed_aux_status = audio_rate_aux_pack(AUDIO_RATE_OP_DSP_SKIPPED, 0u);
    }
#endif

    // 4. DMA/SAI再構築とTX開始（MSP資源は維持し、生成MspInitを通らない）。
    if (failed_stage == AUDIO_RATE_STAGE_NONE)
    {
        AudioTransportFailure_t transport_failure = {0};
        if (!audio_transport_rebuild_and_start_tx(&transport_failure))
        {
            failed_stage      = AUDIO_RATE_STAGE_TX_START;
            failed_hal_status = transport_failure.hal_status;
            failed_aux_status = audio_rate_aux_pack(transport_failure.operation,
                                                    transport_failure.error_code);
        }
    }

    // 5. TX同期待ち（既存の10 ms）。
    if (failed_stage == AUDIO_RATE_STAGE_NONE)
    {
        osDelay(AUDIO_RECOVERY_TX_SYNC_DELAY_MS);
    }

    // 6. RX開始。
    if (failed_stage == AUDIO_RATE_STAGE_NONE)
    {
        AudioTransportFailure_t transport_failure = {0};
        if (!audio_transport_start_rx_after_tx_sync(&transport_failure))
        {
            failed_stage      = AUDIO_RATE_STAGE_RX_START;
            failed_hal_status = transport_failure.hal_status;
            failed_aux_status = audio_rate_aux_pack(transport_failure.operation,
                                                    transport_failure.error_code);
        }
    }

    // 7. ADC/HPDMA再開。成功・失敗とも共通cleanupとしてUI用ADCを戻す。
    // HPDMAの停止が確認できない場合(ADC停止段階の失敗)は、動作中チャネルの再構成を
    // 避けるため触れない。
    bool adc_restarted = false;
    if (hpdma_stopped)
    {
        AudioTransportFailure_t transport_failure = {0};
        adc_restarted = audio_rate_switch_restart_adc(&transport_failure);
        if ((failed_stage == AUDIO_RATE_STAGE_NONE) && !adc_restarted)
        {
            failed_stage      = AUDIO_RATE_STAGE_ADC_RESTART;
            failed_hal_status = transport_failure.hal_status;
            failed_aux_status = audio_rate_aux_pack(transport_failure.operation,
                                                    transport_failure.error_code);
        }
    }

    if (failed_stage == AUDIO_RATE_STAGE_NONE)
    {
        // 全手順成功: 適用値・sequence・READYをcommitし、通常搬送を許可する。
        const uint32_t previous_applied_hz = s_sample_rate.applied_hz;

        // 切り替え期間中にTinyUSBへ残ったデータを破棄し、primingやUSB IN送信へ
        // 持ち越さない。転送中のバッファには触れず公開APIだけを使う。
        audio_transport_clear_usb_fifos();

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
        // 成功commit前に固定target_hzでfeedbackの定数と平滑化履歴を初期化する。
        // 切り替え中に更新された最新要求値は混ぜない。
        audio_usb_control_feedback_reset(target_hz);
#endif

        {
            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            s_sample_rate.state            = AUDIO_RATE_STATE_READY;
            s_sample_rate.applied_hz       = target_hz;
            s_sample_rate.applied_sequence = target_sequence;
            s_sample_rate.applied_hz_valid = true;
            s_transport_rate_hz            = target_hz;
            s_transport_rate_ready         = true;
            __set_PRIMASK(primask);
        }

        audio_diagnostics_record_rate_switch_success(target_hz);
        audio_diagnostics_record_rate_switch_cleanup(AUDIO_RATE_CLEANUP_TRANSPORT_STOPPED |
                                                     AUDIO_RATE_CLEANUP_ADC_RESTARTED);
        audio_diagnostics_update_rate_snapshot(s_sample_rate.state,
                                               s_sample_rate.applied_hz,
                                               s_sample_rate.applied_hz_valid,
                                               audio_control_clock_valid());

        // 開始前に受理済みだった復旧要求は再構築で完了扱いにする。
        audio_transport_ack_recovery_request(recovery_sequence_at_start);
        audio_recovery_reset_after_rate_change();

        SEGGER_RTT_printf(0,
                          "[SAI] reset for %lu Hz (prev=%lu) tick=%lu\n",
                          (unsigned long) target_hz,
                          (unsigned long) previous_applied_hz,
                          (unsigned long) HAL_GetTick());
        return;
    }

    // 失敗: 最後の適用成功値は履歴として残すが信用せず、FAILEDで搬送禁止を維持する。
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_sample_rate.state            = AUDIO_RATE_STATE_FAILED;
        s_sample_rate.applied_hz_valid = false;
        s_transport_rate_hz            = s_sample_rate.applied_hz;
        s_transport_rate_ready         = false;
        __set_PRIMASK(primask);
    }

    audio_diagnostics_record_rate_switch_failure(failed_result, failed_stage,
                                                 failed_hal_status, failed_aux_status);

    // 再構築済みの経路も含めて停止を確認する。停止未確認ならバッファを消去しない。
    // cleanupの成否は元の失敗情報を上書きしない。
    uint32_t cleanup_status = 0U;
    if (audio_transport_stop_and_clear_paths(NULL))
    {
        cleanup_status |= AUDIO_RATE_CLEANUP_TRANSPORT_STOPPED;
    }
    if (adc_restarted)
    {
        cleanup_status |= AUDIO_RATE_CLEANUP_ADC_RESTARTED;
    }
    // 失敗期間中にTinyUSBへ残ったデータを、搬送再許可時のpriming/送信へ持ち越さない。
    audio_transport_clear_usb_fifos();
    audio_diagnostics_record_rate_switch_cleanup(cleanup_status);
    audio_diagnostics_update_rate_snapshot(s_sample_rate.state,
                                           s_sample_rate.applied_hz,
                                           s_sample_rate.applied_hz_valid,
                                           audio_control_clock_valid());

    SEGGER_RTT_printf(0,
                      "[SAI] rate switch failed: requested=%lu last_applied=%lu stage=%lu hal=0x%08lX aux=0x%08lX cleanup=0x%02lX\n",
                      (unsigned long) target_hz,
                      (unsigned long) s_sample_rate.applied_hz,
                      (unsigned long) failed_stage,
                      (unsigned long) failed_hal_status,
                      (unsigned long) failed_aux_status,
                      (unsigned long) cleanup_status);
}
