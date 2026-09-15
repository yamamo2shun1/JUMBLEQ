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
#include "timecode_synth.h"

#include "adc.h"
#include "hpdma.h"
#include "i2c.h"
#include "linked_list.h"
#include "eeprom.h"

#include "adau1466.h"

enum
{
    AUDIO_TASK_STATS_PERIOD_MS = 1000u,
};

extern DMA_QListTypeDef List_HPDMA1_Channel0;

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

void audio_control_request_sample_rate(uint32_t sample_rate_hz)
{
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
    }

    // 呼び出し頻度計測
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

    if (audio_sample_rate_change_take_pending())
    {
        // A new SET_CUR received during the switch remains pending for the next call.
#if RESET_FROM_FW
        AUDIO_SAI_Reset_ForNewRate();
#endif
    }
    else
    {
        timecode_synth_update();
        timecode_synth_set_channel_enabled(
            0u, get_current_ch1_input_mode() == UI_INPUT_MODE_SYNTH);
        timecode_synth_set_channel_enabled(
            1u, get_current_ch2_input_mode() == UI_INPUT_MODE_SYNTH);

        audio_transport_service(s_sample_rate.requested_hz);
    }
}

void AUDIO_SAI_Reset_ForNewRate(void)
{
    const uint32_t new_hz = s_sample_rate.requested_hz;
    bool rate_switch_succeeded = true;

    if (s_sample_rate.applied_hz_valid && (new_hz == s_sample_rate.applied_hz))
    {
        return;
    }

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
                          "[SAI] reset for %lu Hz (prev=%lu)\n",
                          (unsigned long) new_hz,
                          (unsigned long) s_sample_rate.applied_hz);
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
}
