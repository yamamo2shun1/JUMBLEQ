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

enum
{
    AUDIO_TASK_STATS_PERIOD_MS = 1000u,
    // DMA復旧: TX開始からRX開始までの同期待ち（既存レート変更と同じ10ms）
    AUDIO_RECOVERY_TX_SYNC_DELAY_MS = 10u,
    // 連続失敗時の再試行間隔
    AUDIO_RECOVERY_BACKOFF_MS = 100u,
    // この回数だけ連続失敗したらFAILEDへラッチし、両経路停止・無音を維持する
    AUDIO_RECOVERY_MAX_CONSECUTIVE_FAILURES = 3u,
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
typedef struct
{
    volatile uint32_t requested_hz;   // 最後に受理した要求値。適用完了前でも更新される
    volatile bool change_pending;     // Audio Taskが処理すべき変更要求がある
    uint32_t applied_hz;              // 最後にADAU1466切替が成功した値
    bool applied_hz_valid;            // applied_hzをearly-return判定に使えるか
} audio_sample_rate_state_t;

static audio_sample_rate_state_t s_sample_rate = {
    .requested_hz     = 48000U,
    .change_pending   = false,
    .applied_hz       = 48000U,
    .applied_hz_valid = true,
};

static bool AUDIO_SAI_Reset_ForNewRate(void);
static void audio_recovery_process(void);
static void audio_recovery_release_latch(void);
static void audio_recovery_reset_after_rate_change(void);

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

    s_sample_rate.requested_hz = sample_rate_hz;
    __DMB();
    s_sample_rate.change_pending = true;
    audio_transport_notify_task();
}

static bool audio_sample_rate_change_take_pending(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const bool pending = s_sample_rate.change_pending;
    s_sample_rate.change_pending = false;

    __set_PRIMASK(primask);
    return pending;
}

void AUDIO_LoadAndApplyRoutingFromEEPROM(void)
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

uint32_t get_current_sample_rate_hz(void)
{
    return s_sample_rate.requested_hz;
}

void reset_audio_buffer(void)
{
    ui_control_reset_state();

    timecode_synth_init(s_sample_rate.requested_hz);

    audio_transport_reset_buffers();
}

void audio_control_register_task(void)
{
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
}

static void audio_recovery_on_failure(uint32_t failed_stage)
{
    audio_diagnostics_record_recovery_failure(failed_stage);
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

    const bool stopped = audio_transport_stop_and_clear_paths(false);
    audio_diagnostics_record_recovery_attempt();

    if (!stopped)
    {
        // 停止完了を確認できない場合は再構築せず、backoff後に再試行する。
        audio_recovery_on_failure(AUDIO_RECOVERY_STAGE_PREPARE);
        return;
    }

    if (audio_transport_rebuild_and_start_tx(false))
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
                if (audio_transport_start_rx_after_tx_sync())
                {
                    audio_transport_ack_recovery_request(s_recovery_request_sequence);
                    s_recovery_request_in_flight = false;
                    audio_diagnostics_record_recovery_success();
                    s_recovery_consecutive_failures = 0u;
                    s_recovery_state = AUDIO_RECOVERY_STATE_IDLE;
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
            audio_recovery_release_latch();
        }
    }

    // サンプルレート変更とDMA復旧は同じ呼出しで並行実行しない。
    if (audio_sample_rate_change_take_pending())
    {
        // A new SET_CUR received during the switch remains pending for the next call.
#if RESET_FROM_FW
        if (AUDIO_SAI_Reset_ForNewRate())
        {
            // レート変更がSAI/GPDMAを再構築・再始動した。
            audio_recovery_reset_after_rate_change();
        }
        else
        {
            // 同一レート要求で再構築しなかった場合も、ラッチ解除の契機とする。
            audio_recovery_release_latch();
        }
#endif
    }
    else
    {
        audio_recovery_process();

        // DMA half処理をTinyUSB FIFO操作より先に行う。
        audio_transport_service(s_sample_rate.requested_hz);

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
                                       audio_transport_tx_used_words());
        audio_diagnostics_reset_interval();
#endif
    }
}

static bool AUDIO_SAI_Reset_ForNewRate(void)
{
    const uint32_t new_hz = s_sample_rate.requested_hz;
    bool rate_switch_succeeded = true;

    if (s_sample_rate.applied_hz_valid && (new_hz == s_sample_rate.applied_hz))
    {
#if AUDIO_DIAG_LOG
        SEGGER_RTT_printf(0,
                          "[AUD][RATE-APPLY] tick=%lu action=no-op requested=%lu applied=%lu\r\n",
                          (unsigned long) HAL_GetTick(),
                          (unsigned long) new_hz,
                          (unsigned long) s_sample_rate.applied_hz);
#endif
        return false;
    }

    // この開始前までに受理済みだった復旧要求は、SAI/GPDMA再構築で完了扱いにする。
    // 再構築中・再構築後に届いた新しい要求はsequence比較でpendingのまま残る。
    AudioRecoveryRequest_t pending_recovery;
    const bool has_pending_recovery = audio_transport_take_recovery_request(&pending_recovery);

#if AUDIO_DIAG_LOG
    SEGGER_RTT_printf(0,
                      "[AUD][RATE-APPLY] tick=%lu action=switch-begin requested=%lu applied=%lu applied_valid=%u\r\n",
                      (unsigned long) HAL_GetTick(),
                      (unsigned long) new_hz,
                      (unsigned long) s_sample_rate.applied_hz,
                      s_sample_rate.applied_hz_valid ? 1u : 0u);
#endif

    /* Stop ADC DMA to prevent concurrent DSP parameter writes during the path switch. */
    (void) HAL_ADC_Stop(&hadc1);
    (void) HAL_DMA_Abort(&handle_HPDMA1_Channel0);
    ui_control_set_adc_complete(false);
    __DSB();

    audio_transport_reset_for_sample_rate(new_hz);

#if RESET_FROM_FW
    // The DSP core and AK4619 link stay at 96 kHz. Only the USB-side clock and
    // ADAU1466 ASRC/direct routing change, so program and parameter RAM remain
    // intact and no routing/UI state restoration is required.
    if (!AUDIO_Update_ADAU1466_SampleRate(new_hz))
    {
        SEGGER_RTT_printf(0, "[AUD] ADAU1466 rate switch failed: %lu Hz\n", (unsigned long) new_hz);
        rate_switch_succeeded = false;
    }
#endif

    audio_transport_restart_after_rate_change();

    if (has_pending_recovery)
    {
        // 開始前に受理済みだった復旧要求は再構築で完了扱いにする。
        audio_transport_ack_recovery_request(pending_recovery.sequence);
    }

    /* Restart ADC DMA after sample rate change is complete */
    if (MX_List_HPDMA1_Channel0_Config() != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMAEx_List_LinkQ(&handle_HPDMA1_Channel0, &List_HPDMA1_Channel0) != HAL_OK)
    {
        Error_Handler();
    }
    handle_HPDMA1_Channel0.XferCpltCallback = ui_control_dma_adc_cplt;
    if (HAL_DMAEx_List_Start_IT(&handle_HPDMA1_Channel0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    if (rate_switch_succeeded)
    {
        SEGGER_RTT_printf(0,
                          "[SAI] reset for %lu Hz (prev=%lu) tick=%lu\n",
                          (unsigned long) new_hz,
                          (unsigned long) s_sample_rate.applied_hz,
                          (unsigned long) HAL_GetTick());
        s_sample_rate.applied_hz = new_hz;
        s_sample_rate.applied_hz_valid = true;
    }
    else
    {
        s_sample_rate.applied_hz_valid = false;
        SEGGER_RTT_printf(0,
                          "[SAI] rate switch not finalized: requested=%lu last_applied=%lu\n",
                          (unsigned long) new_hz,
                          (unsigned long) s_sample_rate.applied_hz);
    }

    return true;
}
