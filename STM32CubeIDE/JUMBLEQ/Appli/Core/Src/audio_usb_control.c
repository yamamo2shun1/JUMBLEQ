/*
 * audio_usb_control.c
 *
 * TinyUSB UAC2 control-plane adapter. Owns the device callbacks, the
 * host-visible Mute/Volume state and the supported sample-rate list.
 *
 * Sample-rate and stream changes are published as requests and applied by the
 * Audio Task. Callbacks never reinitialize SAI/GPDMA or the DSP path.
 */

#include "audio_control.h"
#include "audio_control_internal.h"
#include "audio_diagnostics_internal.h"
#include "audio_transport_internal.h"
#include "audio_usb_control_internal.h"
#include "adau1466.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "SEGGER_RTT.h"
#if AUDIO_DIAG_LOG
#include "device/dcd.h"
#endif
#include "tusb.h"

#define N_SAMPLE_RATES TU_ARRAY_SIZE(sample_rates)

enum
{
    BLINK_STREAMING   = 25,
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED     = 1000,
    BLINK_SUSPENDED   = 2500,
};

enum
{
    VOLUME_CTRL_0_DB          = 0,
    VOLUME_CTRL_50_DB         = 12800,
    VOLUME_CTRL_RESOLUTION_DB = 256,  // Q8.8で1dB。GET_RANGEのbResと一致させる
};

// 広告範囲の境界が刻みと整合していることを保証する。
_Static_assert((VOLUME_CTRL_0_DB % VOLUME_CTRL_RESOLUTION_DB) == 0,
               "volume control max must align with the resolution");
_Static_assert((VOLUME_CTRL_50_DB % VOLUME_CTRL_RESOLUTION_DB) == 0,
               "volume control min must align with the resolution");

// Audio controls
static volatile uint32_t tx_blink_interval_ms = BLINK_NOT_MOUNTED;
static volatile uint32_t rx_blink_interval_ms = BLINK_NOT_MOUNTED;

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
// アプリ側feedback計算の状態。TinyUSBのFIFO_COUNT方式と同じ平滑化・目標水位・
// 上下限・補正量を手書きコードで維持する。feedbackは16.16形式。
typedef struct
{
    bool     valid;            // 定数が初期化済み
    uint16_t fifo_threshold;   // 目標FIFO水位(byte)
    uint16_t rate_const[2];    // 目標からの水位偏差1byte当たりの補正量
    uint32_t nominal;          // 公称feedback値(16.16)
    uint32_t min_value;        // 許容下限(16.16)
    uint32_t max_value;        // 許容上限(16.16)
    uint32_t fifo_lvl_avg;     // 平滑化したFIFO水位(16.16)
} audio_usb_feedback_state_t;

static audio_usb_feedback_state_t s_feedback = {0};
#endif

const uint32_t sample_rates[] = {48000, 96000};

// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];     // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];  // +1 for master channel 0

// ---- Feature Unit（音量・ミュート）の非同期適用 ----
// USB callbackは要求値とdirty bitの記録だけを行い、DSP適用は専用Taskが行う。
enum
{
    AUDIO_USB_FEATURE_SERVICE_PERIOD_MS = 10U,
    AUDIO_USB_FEATURE_RETRY_INTERVAL_MS = 50U,
    AUDIO_USB_FEATURE_ALL_CHANNEL_MASK  = (1U << CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX) - 1U,
};

static volatile uint8_t s_feature_dirty_mask = 0U;
static bool s_feature_backoff_active = false;
static uint32_t s_feature_retry_started_tick = 0U;
static osThreadId_t s_feature_task_handle = NULL;
// ヒープ不足時もvApplicationMallocFailedHookへ入らず生成できるよう、スタックとTCBは
// 静的確保で提供する（CMSIS-RTOS2のcb_mem/stack_mem指定でxTaskCreateStatic経路）。
static StackType_t s_feature_task_stack[(256 * 4) / sizeof(StackType_t)]
    __attribute__((aligned(portBYTE_ALIGNMENT)));
static StaticTask_t s_feature_task_cb;
static const osThreadAttr_t s_feature_task_attributes = {
    .name       = "usbFeatureTask",
    .stack_mem  = s_feature_task_stack,
    .stack_size = sizeof(s_feature_task_stack),
    .cb_mem     = &s_feature_task_cb,
    .cb_size    = sizeof(s_feature_task_cb),
    .priority   = (osPriority_t) osPriorityNormal,
};

volatile AudioUsbFeatureDiagnostics_t g_audio_usb_feature_diagnostics = {0};

// チャンネルのdirty bit。Master(ch0)は全チャンネルを対象にする。
static uint8_t audio_usb_feature_channel_bit(uint8_t channel)
{
    if (channel == 0U)
    {
        return (uint8_t) AUDIO_USB_FEATURE_ALL_CHANNEL_MASK;
    }

    return (uint8_t) (1U << (channel - 1U));
}

// 音量要求の検証。GET_RANGEで広告する範囲(-50..0dB)と1dB刻みに一致する場合だけ
// 受理する。受信値はUSB little-endianのsigned Q8.8として復号済みの値。
static bool audio20_feature_unit_volume_is_valid(int16_t volume_q8_8)
{
    if ((volume_q8_8 < -VOLUME_CTRL_50_DB) || (volume_q8_8 > VOLUME_CTRL_0_DB))
    {
        return false;
    }

    return (volume_q8_8 % VOLUME_CTRL_RESOLUTION_DB) == 0;
}

// SET_CUR受理。要求値の更新とdirty bit設定を同じPRIMASK区間で行う。
// SPI呼出し・mutex/セマフォ待ち・printfは行わない。
static void audio_usb_feature_store_mute(uint8_t channel, int8_t value)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    mute[channel] = value;
    const uint8_t dirty_bit = audio_usb_feature_channel_bit(channel);
    if ((s_feature_dirty_mask & dirty_bit) != 0U)
    {
        g_audio_usb_feature_diagnostics.coalesced_request_count++;
    }
    s_feature_dirty_mask |= dirty_bit;

    __set_PRIMASK(primask);
}

static void audio_usb_feature_store_volume(uint8_t channel, int16_t value)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    volume[channel] = value;
    const uint8_t dirty_bit = audio_usb_feature_channel_bit(channel);
    if ((s_feature_dirty_mask & dirty_bit) != 0U)
    {
        g_audio_usb_feature_diagnostics.coalesced_request_count++;
    }
    s_feature_dirty_mask |= dirty_bit;

    __set_PRIMASK(primask);
}

// 適用失敗。当該チャンネルをpendingへ戻し、失敗時刻からのバックオフを開始する。
static void audio_usb_feature_note_failure(uint8_t channel, uint32_t operation,
                                           sigma_spi_result_t result)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_feature_dirty_mask |= audio_usb_feature_channel_bit(channel);
    __set_PRIMASK(primask);

    s_feature_retry_started_tick = HAL_GetTick();
    s_feature_backoff_active     = true;

    g_audio_usb_feature_diagnostics.failed_channel_count++;
    g_audio_usb_feature_diagnostics.last_result           = (uint32_t) result;
    g_audio_usb_feature_diagnostics.last_failed_result    = (uint32_t) result;
    g_audio_usb_feature_diagnostics.last_failed_channel   = channel;
    g_audio_usb_feature_diagnostics.last_failed_operation = operation;
    g_audio_usb_feature_diagnostics.backoff_active        = 1U;
}

// 1チャンネル分の適用。ミュート要求はMuteを先に書き、Gain失敗を理由に
// ミュートを見送らない。ミュート解除は同一snapshotのGain成功時だけ行う。
static void audio_usb_feature_apply_channel(uint8_t channel,
                                            const int8_t* mute_snapshot,
                                            const int16_t* volume_snapshot)
{
    const int32_t effective_volume_q8_8 = (int32_t) volume_snapshot[0] + volume_snapshot[channel];
    const int16_t effective_volume_db   = (int16_t) (effective_volume_q8_8 / VOLUME_CTRL_RESOLUTION_DB);
    const bool effective_mute           = (mute_snapshot[0] != 0) || (mute_snapshot[channel] != 0);

    sigma_spi_result_t result = SIGMA_SPI_RESULT_OK;
    uint32_t failed_operation = AUDIO_USB_FEATURE_OP_NONE;

    g_audio_usb_feature_diagnostics.last_apply_tick_ms = HAL_GetTick();

    if (effective_mute)
    {
        result = control_input_from_usb_mute(channel, true);
        if (result == SIGMA_SPI_RESULT_OK)
        {
            result = control_input_from_usb_gain(channel, effective_volume_db);
            if (result != SIGMA_SPI_RESULT_OK)
            {
                failed_operation = AUDIO_USB_FEATURE_OP_GAIN;
            }
        }
        else
        {
            failed_operation = AUDIO_USB_FEATURE_OP_MUTE;
        }
    }
    else
    {
        result = control_input_from_usb_gain(channel, effective_volume_db);
        if (result == SIGMA_SPI_RESULT_OK)
        {
            result = control_input_from_usb_mute(channel, false);
            if (result != SIGMA_SPI_RESULT_OK)
            {
                failed_operation = AUDIO_USB_FEATURE_OP_MUTE;
            }
        }
        else
        {
            failed_operation = AUDIO_USB_FEATURE_OP_GAIN;
        }
    }

    if (result != SIGMA_SPI_RESULT_OK)
    {
        // Gain成功・Mute失敗もチャンネル全体の適用成功としては記録しない。
        audio_usb_feature_note_failure(channel, failed_operation, result);
        return;
    }

    g_audio_usb_feature_diagnostics.applied_channel_count++;
    g_audio_usb_feature_diagnostics.last_result = (uint32_t) SIGMA_SPI_RESULT_OK;
}

// 専用Taskから周期呼出しする適用service。dirtyチャンネルを最新値へ集約して適用する。
static void audio_usb_control_feature_service(void)
{
    if (s_feature_backoff_active)
    {
        // 失敗時刻からの経過で再試行可否を判定する（期限後も判定が崩れない）。
        if ((uint32_t) (HAL_GetTick() - s_feature_retry_started_tick) < AUDIO_USB_FEATURE_RETRY_INTERVAL_MS)
        {
            return;
        }

        s_feature_backoff_active = false;
        g_audio_usb_feature_diagnostics.backoff_active = 0U;
        g_audio_usb_feature_diagnostics.retry_count++;
    }

    uint8_t dirty_mask = 0U;
    int8_t mute_snapshot[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];
    int16_t volume_snapshot[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    dirty_mask = s_feature_dirty_mask;
    if (dirty_mask != 0U)
    {
        // 適用前にクリアし、適用中に届いた新要求を次回へ引き継ぐ。
        s_feature_dirty_mask = 0U;
        for (uint32_t i = 0U; i <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX; i++)
        {
            mute_snapshot[i]   = mute[i];
            volume_snapshot[i] = volume[i];
        }
    }

    __set_PRIMASK(primask);

    for (uint32_t channel = 1U; channel <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX; channel++)
    {
        if ((dirty_mask & (uint8_t) (1U << (channel - 1U))) == 0U)
        {
            continue;
        }

        // 失敗したチャンネルはpendingへ戻り、他チャンネルの適用は継続する。
        // （snapshotでmaskをクリア済みのため、残りを止めると要求を失う）
        audio_usb_feature_apply_channel((uint8_t) channel, mute_snapshot, volume_snapshot);
    }
}

static void audio_usb_feature_task(void* argument)
{
    (void) argument;

    for (;;)
    {
        audio_usb_control_feature_service();
        osDelay(AUDIO_USB_FEATURE_SERVICE_PERIOD_MS);
    }
}

void audio_usb_control_feature_task_start(void)
{
    if (s_feature_task_handle != NULL)
    {
        return;
    }

    s_feature_task_handle = osThreadNew(audio_usb_feature_task, NULL, &s_feature_task_attributes);
    if (s_feature_task_handle == NULL)
    {
        g_audio_usb_feature_diagnostics.task_create_failed = 1U;
    }
}

void audio_usb_control_set_tx_stream_blink(bool streaming)
{
    tx_blink_interval_ms = streaming ? BLINK_STREAMING :
                                        (tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED);
}

void audio_usb_control_set_rx_stream_blink(bool streaming)
{
    rx_blink_interval_ms = streaming ? BLINK_STREAMING :
                                        (tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED);
}

uint32_t get_tx_blink_interval_ms(void)
{
    return tx_blink_interval_ms;
}

uint32_t get_rx_blink_interval_ms(void)
{
    return rx_blink_interval_ms;
}

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
// マイクロフレーム当たりの公称フレーム数(16.16)をレートから求める。
// High-Speedは8 kHzマイクロフレーム、Full-Speedは1 kHzフレーム基準。
static uint32_t audio_usb_feedback_nominal(uint32_t sample_rate_hz, uint32_t frame_div)
{
    return ((sample_rate_hz / 100u) << 16) / (frame_div / 100u);
}

void audio_usb_control_feedback_reset(uint32_t sample_rate_hz)
{
    const uint32_t frame_div = (TUSB_SPEED_FULL == tud_speed_get()) ? 1000u : 8000u;
    const uint32_t nominal = audio_usb_feedback_nominal(sample_rate_hz, frame_div);

    uint32_t threshold = audio_transport_usb_out_fifo_target_bytes(sample_rate_hz);
    if (threshold == 0u)
    {
        threshold = 1u;
    }

    const uint32_t max_value = ((sample_rate_hz / frame_div) + 1u) << 16;
    const uint32_t min_value = ((sample_rate_hz - 1u) / frame_div) << 16;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    s_feedback.valid          = true;
    s_feedback.nominal        = nominal;
    s_feedback.min_value      = min_value;
    s_feedback.max_value      = max_value;
    s_feedback.fifo_threshold = (uint16_t) threshold;
    s_feedback.rate_const[0]  = (uint16_t) ((max_value - nominal) / threshold);
    s_feedback.rate_const[1]  = (uint16_t) ((nominal - min_value) / threshold);
    if (frame_div == 8000u)
    {
        // High-SpeedはパケットサイズがMSOごとに変動し得るため感度を下げる。
        s_feedback.rate_const[0] /= 8u;
        s_feedback.rate_const[1] /= 8u;
    }
    // 平滑化履歴は目標水位から開始し、切り替え後の初回補正を滑らかにする。
    s_feedback.fifo_lvl_avg = threshold << 16;

    __set_PRIMASK(primask);

    // 初回送信が設定callbackより前に予約される場合があるため、公称値を先に設定する。
    (void) tud_audio_n_fb_set(AUDIO_FUNC_ID, nominal);
}

void audio_usb_control_feedback_update(void)
{
    // 搬送禁止中は水位不足による補正をしない。FIFO読み捨てで水位が下がり、
    // 補正が最大値へ張り付くのを防ぐ。
    if (!audio_control_transport_ready() || !audio_control_transport_paths_safe())
    {
        return;
    }

    tu_fifo_t* ep_out_ff = tud_audio_n_get_ep_out_ff(AUDIO_FUNC_ID);
    if (ep_out_ff == NULL)
    {
        return;
    }

    const uint16_t lvl_new = (uint16_t) tu_fifo_count(ep_out_ff);

    uint32_t feedback = 0u;
    bool updated = false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_feedback.valid)
    {
        // TinyUSBのFIFO_COUNT方式と同じ1/64低域通過フィルタで水位を平滑化する。
        uint32_t lvl = (uint32_t) (((uint64_t) s_feedback.fifo_lvl_avg * 63u +
                                    ((uint32_t) lvl_new << 16)) >> 6);
        s_feedback.fifo_lvl_avg = lvl;

        const uint32_t ff_lvl = lvl >> 16;
        const uint32_t ff_thr = s_feedback.fifo_threshold;

        if (ff_lvl < ff_thr)
        {
            feedback = s_feedback.nominal + (ff_thr - ff_lvl) * s_feedback.rate_const[0];
        }
        else
        {
            feedback = s_feedback.nominal - (ff_lvl - ff_thr) * s_feedback.rate_const[1];
        }

        if (feedback > s_feedback.max_value)
        {
            feedback = s_feedback.max_value;
        }
        if (feedback < s_feedback.min_value)
        {
            feedback = s_feedback.min_value;
        }
        updated = true;
    }

    __set_PRIMASK(primask);

    if (updated)
    {
        (void) tud_audio_n_fb_set(AUDIO_FUNC_ID, feedback);
    }
}
#endif

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
#if AUDIO_DIAG_LOG
    SEGGER_RTT_printf(0,
                      "[AUD][USB-EVENT] tick=%lu event=mount\r\n",
                      (unsigned long) HAL_GetTick());
#endif
    tx_blink_interval_ms = BLINK_MOUNTED;
    rx_blink_interval_ms = BLINK_MOUNTED;

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
    // 初回のSET_INTERFACEでfeedback送信が予約されるため、その前に公称値を設定する。
    AudioRateSnapshot_t rate_snapshot;
    audio_control_get_rate_snapshot(&rate_snapshot);
    audio_usb_control_feedback_reset(rate_snapshot.requested_hz);
#endif
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
#if AUDIO_DIAG_LOG
    SEGGER_RTT_printf(0,
                      "[AUD][USB-EVENT] tick=%lu event=unmount\r\n",
                      (unsigned long) HAL_GetTick());
#endif
    tx_blink_interval_ms = BLINK_NOT_MOUNTED;
    rx_blink_interval_ms = BLINK_NOT_MOUNTED;
    audio_transport_request_stream(AUDIO_TRANSPORT_STREAM_OUT, false);
    audio_transport_request_stream(AUDIO_TRANSPORT_STREAM_IN, false);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
#if AUDIO_DIAG_LOG
    SEGGER_RTT_printf(0,
                      "[AUD][USB-EVENT] tick=%lu event=suspend remote_wakeup=%u\r\n",
                      (unsigned long) HAL_GetTick(),
                      remote_wakeup_en ? 1u : 0u);
#else
    (void) remote_wakeup_en;
#endif
    tx_blink_interval_ms = BLINK_SUSPENDED;
    rx_blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
#if AUDIO_DIAG_LOG
    SEGGER_RTT_printf(0,
                      "[AUD][USB-EVENT] tick=%lu event=resume mounted=%u\r\n",
                      (unsigned long) HAL_GetTick(),
                      tud_mounted() ? 1u : 0u);
#endif
    tx_blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
    rx_blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

// TinyUSB has no task-context bus-reset callback. This hook runs when the DCD
// event is queued; only rare reset events are printed to avoid ISR log load.
#if AUDIO_DIAG_LOG
void tud_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr)
{
    if (eventid == DCD_EVENT_BUS_RESET)
    {
        SEGGER_RTT_printf(0,
                          "[AUD][USB-EVENT] tick=%lu event=bus-reset rhport=%u in_isr=%u\r\n",
                          (unsigned long) HAL_GetTick(),
                          (unsigned int) rhport,
                          in_isr ? 1u : 0u);
    }
}
#endif

//--------------------------------------------------------------------+
// Audio Callback Functions
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// UAC2 Helper Functions
//--------------------------------------------------------------------+

// Helper for clock get requests
static bool audio20_clock_get_request(uint8_t rhport, tusb_control_request_t const* request)
{
    TU_ASSERT(TU_U16_HIGH(request->wIndex) == UAC2_ENTITY_CLOCK);

    if (TU_U16_HIGH(request->wValue) == AUDIO20_CS_CTRL_SAM_FREQ)
    {
        if (request->bRequest == AUDIO20_CS_REQ_CUR)
        {
            const uint32_t current_sample_rate = audio_control_requested_sample_rate_hz();
            TU_LOG1("Clock get current freq %" PRIu32 "\r\n", current_sample_rate);

            audio20_control_cur_4_t curf = {(int32_t) tu_htole32(current_sample_rate)};
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &curf, sizeof(curf));
        }
        else if (request->bRequest == AUDIO20_CS_REQ_RANGE)
        {
            audio20_control_range_4_n_t(N_SAMPLE_RATES) rangef =
                {
                    .wNumSubRanges = tu_htole16(N_SAMPLE_RATES)};
            SEGGER_RTT_printf(0,
                              "[USB] GET_RANGE sample-rate: count=%u request_len=%u response_len=%u\n",
                              (unsigned) N_SAMPLE_RATES,
                              (unsigned) tu_le16toh(request->wLength),
                              (unsigned) sizeof(rangef));
            TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
            for (uint8_t i = 0; i < N_SAMPLE_RATES; i++)
            {
                rangef.subrange[i].bMin = (int32_t) sample_rates[i];
                rangef.subrange[i].bMax = (int32_t) sample_rates[i];
                rangef.subrange[i].bRes = 0;
                SEGGER_RTT_printf(0,
                                  "[USB] RANGE[%u]: min=%lu max=%lu res=%lu\n",
                                  (unsigned) i,
                                  (unsigned long) rangef.subrange[i].bMin,
                                  (unsigned long) rangef.subrange[i].bMax,
                                  (unsigned long) rangef.subrange[i].bRes);
                TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int) rangef.subrange[i].bMin, (int) rangef.subrange[i].bMax, (int) rangef.subrange[i].bRes);
            }

            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &rangef, sizeof(rangef));
        }
    }
    else if (TU_U16_HIGH(request->wValue) == AUDIO20_CS_CTRL_CLK_VALID && request->bRequest == AUDIO20_CS_REQ_CUR)
    {
        // 最新要求と適用状態が整合し、切り替え全体が成功した場合だけ有効を返す。
        // 要求受理後の未処理期間・SWITCHING・FAILED・初期適用未確認では0。
        audio20_control_cur_1_t cur_valid = {.bCur = audio_control_clock_valid() ? 1U : 0U};
        TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &cur_valid, sizeof(cur_valid));
    }
    TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n", TU_U16_HIGH(request->wIndex), TU_U16_HIGH(request->wValue), request->bRequest);
    return false;
}

// Helper for clock set requests
static bool audio20_clock_set_request(uint8_t rhport, tusb_control_request_t const* request, uint8_t const* buf)
{
    (void) rhport;

    TU_ASSERT(TU_U16_HIGH(request->wIndex) == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    if (TU_U16_HIGH(request->wValue) == AUDIO20_CS_CTRL_SAM_FREQ)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_4_t));

        uint32_t requested_sample_rate =
            tu_le32toh((uint32_t) ((audio20_control_cur_4_t const*) buf)->bCur);
        bool supported = false;
        for (uint8_t i = 0U; i < N_SAMPLE_RATES; i++)
        {
            if (requested_sample_rate == sample_rates[i])
            {
                supported = true;
                break;
            }
        }

        if (!supported)
        {
            SEGGER_RTT_printf(0,
                              "[USB] unsupported sample-rate request: %lu Hz tick=%lu\n",
                              (unsigned long) requested_sample_rate,
                              (unsigned long) HAL_GetTick());
            return false;
        }

        audio_control_request_sample_rate(requested_sample_rate);

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
        // 受理した要求値で公称feedback値を先行設定する（切り替え成功時は
        // 固定targetで再初期化し、途中で更新された要求値は混ぜない）。
        audio_usb_control_feedback_reset(requested_sample_rate);
#endif

        SEGGER_RTT_printf(0,
                          "[USB] sample-rate request: %lu Hz tick=%lu\n",
                          (unsigned long) requested_sample_rate,
                          (unsigned long) HAL_GetTick());
        TU_LOG1("Clock set current freq: %" PRIu32 "\r\n", requested_sample_rate);

        return true;
    }
    else
    {
        TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n", TU_U16_HIGH(request->wIndex), TU_U16_HIGH(request->wValue), request->bRequest);
        return false;
    }
}

static bool audio20_feature_unit_channel_is_valid(uint8_t channel)
{
    return channel <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX;
}

// Helper for feature unit get requests
static bool audio20_feature_unit_get_request(uint8_t rhport, tusb_control_request_t const* request)
{
    TU_ASSERT(TU_U16_HIGH(request->wIndex) == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT);

    const uint8_t channel = TU_U16_LOW(request->wValue);
    if (!audio20_feature_unit_channel_is_valid(channel))
    {
        TU_LOG1("Feature unit get request has invalid channel %u\r\n", channel);
        return false;
    }

    if (TU_U16_HIGH(request->wValue) == AUDIO20_FU_CTRL_MUTE && request->bRequest == AUDIO20_CS_REQ_CUR)
    {
        audio20_control_cur_1_t mute1 = {.bCur = mute[channel]};
        TU_LOG1("Get channel %u mute %d\r\n", channel, mute1.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &mute1, sizeof(mute1));
    }
    else if (TU_U16_HIGH(request->wValue) == AUDIO20_FU_CTRL_VOLUME)
    {
        if (request->bRequest == AUDIO20_CS_REQ_RANGE)
        {
            audio20_control_range_2_n_t(1) range_vol = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0]   = {.bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(VOLUME_CTRL_RESOLUTION_DB)}
            };
            TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", TU_U16_LOW(request->wValue), range_vol.subrange[0].bMin / VOLUME_CTRL_RESOLUTION_DB, range_vol.subrange[0].bMax / VOLUME_CTRL_RESOLUTION_DB, range_vol.subrange[0].bRes / VOLUME_CTRL_RESOLUTION_DB);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &range_vol, sizeof(range_vol));
        }
        else if (request->bRequest == AUDIO20_CS_REQ_CUR)
        {
            audio20_control_cur_2_t cur_vol = {.bCur = tu_htole16(volume[channel])};
            TU_LOG1("Get channel %u volume %d dB\r\n", channel, cur_vol.bCur / VOLUME_CTRL_RESOLUTION_DB);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &cur_vol, sizeof(cur_vol));
        }
    }
    TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n", TU_U16_HIGH(request->wIndex), TU_U16_HIGH(request->wValue), request->bRequest);

    return false;
}

// Helper for feature unit set requests
static bool audio20_feature_unit_set_request(uint8_t rhport, tusb_control_request_t const* request, uint8_t const* buf)
{
    (void) rhport;

    TU_ASSERT(TU_U16_HIGH(request->wIndex) == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    const uint8_t channel = TU_U16_LOW(request->wValue);
    if (!audio20_feature_unit_channel_is_valid(channel))
    {
        TU_LOG1("Feature unit set request has invalid channel %u\r\n", channel);
        return false;
    }

    if (TU_U16_HIGH(request->wValue) == AUDIO20_FU_CTRL_MUTE)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_1_t));

        const int8_t requested_mute = ((audio20_control_cur_1_t const*) buf)->bCur;

        // 要求値を記録してdirty bitを立てるだけ。DSP適用はusbFeatureTaskが行う。
        audio_usb_feature_store_mute(channel, requested_mute);

        g_audio_usb_feature_diagnostics.request_count++;
        if (channel == 0U)
        {
            g_audio_usb_feature_diagnostics.master_request_count++;
        }

        TU_LOG1("Set channel %d Mute: %d\r\n", channel, requested_mute);

        return true;
    }
    else if (TU_U16_HIGH(request->wValue) == AUDIO20_FU_CTRL_VOLUME)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_2_t));

        const int16_t requested_volume = ((audio20_control_cur_2_t const*) buf)->bCur;

        if (!audio20_feature_unit_volume_is_valid(requested_volume))
        {
            // 広告範囲(-50..0dB, 1dB刻み)外・端数はSTALLで拒否し、保存値・
            // dirty bit・適用要求を変更しない（既存のpending要求も保持する）。
            g_audio_usb_feature_diagnostics.rejected_request_count++;
            g_audio_usb_feature_diagnostics.last_rejected_channel = channel;
            g_audio_usb_feature_diagnostics.last_rejected_volume  = requested_volume;
            TU_LOG1("Reject channel %d volume: %d (Q8.8)\r\n", channel, requested_volume);
            return false;
        }

        // 要求値を記録してdirty bitを立てるだけ。DSP適用はusbFeatureTaskが行う。
        audio_usb_feature_store_volume(channel, requested_volume);

        g_audio_usb_feature_diagnostics.request_count++;
        if (channel == 0U)
        {
            g_audio_usb_feature_diagnostics.master_request_count++;
        }

        TU_LOG1("Set channel %d volume: %d dB\r\n", channel, requested_volume / VOLUME_CTRL_RESOLUTION_DB);

        return true;
    }
    else
    {
        TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n", TU_U16_HIGH(request->wIndex), TU_U16_HIGH(request->wValue), request->bRequest);
        return false;
    }
}

static bool audio20_get_req_entity(uint8_t rhport, tusb_control_request_t const* p_request)
{
    tusb_control_request_t const* request = p_request;

    if (TU_U16_HIGH(request->wIndex) == UAC2_ENTITY_CLOCK)
        return audio20_clock_get_request(rhport, request);
    if (TU_U16_HIGH(request->wIndex) == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT)
        return audio20_feature_unit_get_request(rhport, request);
    else
    {
        TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n", TU_U16_HIGH(request->wIndex), TU_U16_HIGH(request->wValue), request->bRequest);
    }
    return false;
}

static bool audio20_set_req_entity(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* buf)
{
    tusb_control_request_t const* request = p_request;

    if (TU_U16_HIGH(request->wIndex) == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT)
        return audio20_feature_unit_set_request(rhport, request, buf);
    if (TU_U16_HIGH(request->wIndex) == UAC2_ENTITY_CLOCK)
        return audio20_clock_set_request(rhport, request, buf);
    TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n", TU_U16_HIGH(request->wIndex), TU_U16_HIGH(request->wValue), request->bRequest);

    return false;
}

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* pBuff)
{
    (void) rhport;
    (void) pBuff;
    (void) p_request;
    return false;  // EP-specific requests are not used for UAC2 in this project.
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;
    (void) p_request;
    return false;  // EP-specific requests are not used for UAC2 in this project.
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;
    return audio20_get_req_entity(rhport, p_request);
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* buf)
{
    (void) rhport;
    return audio20_set_req_entity(rhport, p_request, buf);
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

#if AUDIO_DIAG_LOG
    SEGGER_RTT_printf(0,
                      "[AUD][USB-ITF] tick=%lu phase=close itf=%u alt=%u out_itf=%u in_itf=%u\r\n",
                      (unsigned long) HAL_GetTick(),
                      (unsigned int) itf,
                      (unsigned int) alt,
                      (itf == ITF_NUM_AUDIO_STREAMING_STEREO_OUT) ? 1u : 0u,
                      (itf == ITF_NUM_AUDIO_STREAMING_STEREO_IN) ? 1u : 0u);
#endif

    if (ITF_NUM_AUDIO_STREAMING_STEREO_OUT == itf && alt == 0)
    {
        audio_transport_request_stream(AUDIO_TRANSPORT_STREAM_OUT, false);
    }

    if (ITF_NUM_AUDIO_STREAMING_STEREO_IN == itf && alt == 0)
    {
        audio_transport_request_stream(AUDIO_TRANSPORT_STREAM_IN, false);
    }

    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

#if AUDIO_DIAG_LOG
    SEGGER_RTT_printf(0,
                      "[AUD][USB-ITF] tick=%lu phase=set itf=%u alt=%u out_itf=%u in_itf=%u\r\n",
                      (unsigned long) HAL_GetTick(),
                      (unsigned int) itf,
                      (unsigned int) alt,
                      (itf == ITF_NUM_AUDIO_STREAMING_STEREO_OUT) ? 1u : 0u,
                      (itf == ITF_NUM_AUDIO_STREAMING_STEREO_IN) ? 1u : 0u);
#endif

    TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
    if (ITF_NUM_AUDIO_STREAMING_STEREO_OUT == itf && alt != 0)
    {
        audio_transport_request_stream(AUDIO_TRANSPORT_STREAM_OUT, true);
    }

    if (ITF_NUM_AUDIO_STREAMING_STEREO_IN == itf && alt != 0)
    {
        audio_transport_request_stream(AUDIO_TRANSPORT_STREAM_IN, true);
    }

    return true;
}

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t* feedback_param)
{
    (void) alt_itf;
    if (func_id != AUDIO_FUNC_ID)
    {
        return;
    }

    // TinyUSB内部の自動計算（旧レートの定数のままfeedbackを上書きし続ける）を止める。
    // feedback値はアプリ側で計算し、tud_audio_n_fb_set()で更新する。
    AudioRateSnapshot_t rate_snapshot;
    audio_control_get_rate_snapshot(&rate_snapshot);

    feedback_param->method      = AUDIO_FEEDBACK_METHOD_DISABLED;
    feedback_param->sample_freq = rate_snapshot.requested_hz;
}
#endif
