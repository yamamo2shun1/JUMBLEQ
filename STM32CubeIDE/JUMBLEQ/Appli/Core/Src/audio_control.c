/*
 * audio_control.c
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#include "audio_control.h"
#include "ui_control_internal.h"
#include "timecode_oscillator.h"

#include "adc.h"
#include "gpdma.h"
#include "hpdma.h"
#include "i2c.h"
#include "linked_list.h"
#include "sai.h"
#include "sai.h"
#include "eeprom.h"

#include "FreeRTOS.h"  // for xPortGetFreeHeapSize
#include "cmsis_os2.h"
#include "task.h"

#include "ak4619.h"
#include "adau1466.h"
#include "SigmaStudioFW.h"

#define N_SAMPLE_RATES TU_ARRAY_SIZE(sample_rates)
#define AUDIO_DIAG_LOG 0
#define DBG_MIN_U32_INIT UINT32_MAX

enum
{
    AUDIO_TASK_STATS_PERIOD_MS = 1000u,
    AUDIO_BYTES_PER_SAMPLE_32  = 4u,
    AUDIO_USB_FRAME_CHANNELS   = 4u,
    AUDIO_RING_FRAME_WORDS     = 4u,
    DBG_MIN_U16_INIT           = 0xFFFFu,
    AUDIO_USB_HS_MICROFRAMES_PER_SECOND = 8000u,
    AUDIO_FUNC_ID              = 0u,
};

extern DMA_QListTypeDef List_GPDMA1_Channel2;
extern DMA_QListTypeDef List_GPDMA1_Channel3;
extern DMA_QListTypeDef List_HPDMA1_Channel0;

enum
{
    BLINK_STREAMING   = 25,
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED     = 1000,
    BLINK_SUSPENDED   = 2500,
};

enum
{
    VOLUME_CTRL_0_DB    = 0,
    VOLUME_CTRL_50_DB   = 12800,
};

// Audio controls
static uint32_t tx_blink_interval_ms = BLINK_NOT_MOUNTED;
static uint32_t rx_blink_interval_ms = BLINK_NOT_MOUNTED;

volatile uint32_t sai_tx_rng_buf_index = 0;
volatile uint32_t sai_rx_rng_buf_index = 0;
volatile uint32_t sai_transmit_index   = 0;
volatile uint32_t sai_receive_index    = 0;

static volatile uint8_t tx_pending_mask = 0;      // bit0: first-half, bit1: second-half
static volatile uint8_t rx_pending_mask = 0;      // bit0: first-half, bit1: second-half
static volatile bool usb_tx_pending     = false;  // USB TX送信要求フラグ (ISR→Task通知用)
static volatile bool usb_rx_pending     = false;  // USB RX受信通知フラグ (ISR→Task通知用)
static TaskHandle_t s_audio_task_handle = NULL;

bool s_streaming_out = false;
bool s_streaming_in  = false;

volatile AudioTxDiagnostics_t g_audio_tx_diagnostics = {
    .tx_used_min_words = UINT32_MAX,
};

static volatile uint32_t s_tx_half_pending_cycle = 0u;
static volatile uint32_t s_tx_cplt_pending_cycle = 0u;
static volatile uint8_t s_tx_last_pending_callback = 0u;

static void audio_tx_diagnostics_reset(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t reset_count = g_audio_tx_diagnostics.reset_count + 1u;
    AudioTxDiagnostics_t reset = {0};
    reset.reset_count          = reset_count;
    reset.tx_used_min_words    = UINT32_MAX;
    g_audio_tx_diagnostics     = reset;
    const uint32_t now_cycles  = DWT->CYCCNT;
    s_tx_half_pending_cycle    = ((tx_pending_mask & 0x01u) != 0u) ? now_cycles : 0u;
    s_tx_cplt_pending_cycle    = ((tx_pending_mask & 0x02u) != 0u) ? now_cycles : 0u;
    s_tx_last_pending_callback = 0u;
    __set_PRIMASK(primask);
}

static inline int32_t audio_tx_used_words(void)
{
    return (int32_t) (sai_tx_rng_buf_index - sai_transmit_index);
}

static inline void audio_tx_diagnostics_record_level(int32_t used)
{
    if (!s_streaming_out || used < 0)
    {
        return;
    }

    const uint32_t level = (uint32_t) used;
    if (level < g_audio_tx_diagnostics.tx_used_min_words)
    {
        g_audio_tx_diagnostics.tx_used_min_words = level;
    }
    if (level > g_audio_tx_diagnostics.tx_used_max_words)
    {
        g_audio_tx_diagnostics.tx_used_max_words = level;
    }
}

static inline void audio_tx_diagnostics_record_event(uint32_t flags, int32_t used)
{
    if (!s_streaming_out)
    {
        return;
    }

    g_audio_tx_diagnostics.last_event_used_words = used;
    g_audio_tx_diagnostics.last_event_flags      = flags;
    g_audio_tx_diagnostics.last_event_tick_ms    = HAL_GetTick();
    g_audio_tx_diagnostics.last_event_cycle      = DWT->CYCCNT;
}

void audio_control_register_task(void)
{
    s_audio_task_handle = xTaskGetCurrentTaskHandle();
}

static inline void audio_task_notify(void)
{
    if (s_audio_task_handle == NULL)
    {
        return;
    }

    xTaskNotifyGive(s_audio_task_handle);
}

static inline void audio_task_notify_from_isr(void)
{
    if (s_audio_task_handle == NULL)
    {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(s_audio_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#if AUDIO_DIAG_LOG
static volatile uint32_t dbg_tx_used_min            = 0xFFFFFFFFu;
static volatile uint32_t dbg_tx_used_max            = 0u;
static volatile uint32_t dbg_tx_underrun_events     = 0u;
static volatile uint32_t dbg_tx_partial_fill_events = 0u;
static volatile uint32_t dbg_tx_drift_up_events     = 0u;
static volatile uint32_t dbg_tx_drift_dn_events     = 0u;
static volatile uint32_t dbg_usb_read_zero_events   = 0u;
static volatile uint32_t dbg_usb_read_bytes         = 0u;
static volatile uint32_t dbg_dma_err_events         = 0u;
static volatile uint32_t dbg_sai_tx_err_events      = 0u;
static volatile uint32_t dbg_sai_rx_err_events      = 0u;
static volatile uint32_t dbg_sai_tx_last_err        = 0u;
static volatile uint32_t dbg_sai_rx_last_err        = 0u;
static volatile uint32_t dbg_sai_tx_sr_flags        = 0u;
static volatile uint32_t dbg_sai_rx_sr_flags        = 0u;
static uint32_t dbg_sigma_calls_prev                = 0u;
static uint32_t dbg_sigma_err_prev                  = 0u;
static uint32_t dbg_sigma_to_prev                   = 0u;
static uint32_t dbg_sigma_mto_prev                  = 0u;
static volatile uint32_t dbg_tx_half_rewrite_events = 0u;
static volatile uint32_t dbg_tx_cplt_rewrite_events = 0u;
static volatile uint32_t dbg_rx_half_rewrite_events = 0u;
static volatile uint32_t dbg_rx_cplt_rewrite_events = 0u;
static volatile uint16_t dbg_usb_read_size_min      = DBG_MIN_U16_INIT;
static volatile uint16_t dbg_usb_read_size_max      = 0u;
static volatile uint32_t dbg_usb_out_packet_events  = 0u;
static volatile uint32_t dbg_usb_out_packet_bytes   = 0u;
static volatile uint16_t dbg_usb_out_packet_size_min = DBG_MIN_U16_INIT;
static volatile uint16_t dbg_usb_out_packet_size_max = 0u;
static volatile uint32_t dbg_usb_out_gap_cycles_min = DBG_MIN_U32_INIT;
static volatile uint32_t dbg_usb_out_gap_cycles_max = 0u;
static volatile uint32_t dbg_usb_out_gap_late_events = 0u;
static volatile uint32_t dbg_usb_out_prev_cycle     = 0u;
static volatile bool dbg_usb_out_prev_cycle_valid   = false;
static volatile uint32_t dbg_usb_out_coalesced_events = 0u;
static volatile uint32_t dbg_usb_out_pending_cycle  = 0u;
static volatile uint32_t dbg_usb_out_service_cycles_min = DBG_MIN_U32_INIT;
static volatile uint32_t dbg_usb_out_service_cycles_max = 0u;
static volatile uint16_t dbg_usb_out_fifo_min       = DBG_MIN_U16_INIT;
static volatile uint16_t dbg_usb_out_fifo_max       = 0u;
static volatile uint32_t dbg_usb_in_packet_events   = 0u;
static volatile uint32_t dbg_usb_in_packet_zero_events = 0u;
static volatile uint32_t dbg_usb_in_packet_bytes    = 0u;
static volatile uint16_t dbg_usb_in_packet_size_min = DBG_MIN_U16_INIT;
static volatile uint16_t dbg_usb_in_packet_size_max = 0u;
static volatile uint32_t dbg_usb_in_notify_events   = 0u;
static volatile uint32_t dbg_usb_in_source_wait_events = 0u;
static volatile uint32_t dbg_usb_in_write_zero_events = 0u;
static volatile uint32_t dbg_usb_in_write_partial_events = 0u;
static volatile uint32_t dbg_usb_in_write_bytes     = 0u;
static volatile uint16_t dbg_usb_in_fifo_min        = DBG_MIN_U16_INIT;
static volatile uint16_t dbg_usb_in_fifo_max        = 0u;

static uint32_t audio_diag_cycles_to_us(uint32_t cycles)
{
    if (SystemCoreClock == 0u)
    {
        return 0u;
    }

    return (uint32_t) ((((uint64_t) cycles * 1000000u) + (SystemCoreClock / 2u)) / SystemCoreClock);
}
#endif

static volatile bool is_sr_changed = false;

const uint32_t sample_rates[] = {48000, 96000};
static volatile uint32_t current_sample_rate = 48000U;

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t usb_out_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t usb_in_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4] = {0};

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t sai_tx_rng_buf[SAI_RNG_BUF_SIZE] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t sai_rx_rng_buf[SAI_RNG_BUF_SIZE] = {0};

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t stereo_out_buf[SAI_TX_BUF_SIZE] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t stereo_in_buf[SAI_RX_BUF_SIZE]  = {0};

#if ENABLE_TIMECODE_OSCILLATOR
enum
{
    TIMECODE_SYNTH_CHANNEL_COUNT = 2u,
    TIMECODE_SYNTH_FIFO_FRAMES   = 256u,
    TIMECODE_SYNTH_BLOCK_FRAMES  = (SAI_RX_BUF_SIZE / 2u) / AUDIO_RING_FRAME_WORDS,
};

_Static_assert((TIMECODE_SYNTH_FIFO_FRAMES & (TIMECODE_SYNTH_FIFO_FRAMES - 1u)) == 0u,
               "Timecode synth FIFO size must be a power of two");
_Static_assert(TIMECODE_SYNTH_BLOCK_FRAMES <= TIMECODE_OSCILLATOR_MAX_BLOCK_FRAMES,
               "Timecode oscillator block buffer is too small");

typedef struct
{
    int32_t samples[TIMECODE_SYNTH_FIFO_FRAMES];
    uint32_t write_index;
    uint32_t read_index;
    volatile uint32_t overflow_frames;
    volatile uint32_t underrun_frames;
} TimecodeSynthFifo_t;

static TimecodeOscillator_t s_timecode_oscillator[TIMECODE_SYNTH_CHANNEL_COUNT];
static TimecodeSynthFifo_t s_timecode_synth_fifo[TIMECODE_SYNTH_CHANNEL_COUNT];
static int32_t s_timecode_synth_block[TIMECODE_SYNTH_CHANNEL_COUNT][TIMECODE_SYNTH_BLOCK_FRAMES];
static bool s_timecode_synth_active[TIMECODE_SYNTH_CHANNEL_COUNT] = {false, false};
static volatile uint8_t s_timecode_synth_control[TIMECODE_SYNTH_CONTROL_COUNT] = {
    [TIMECODE_SYNTH_CONTROL_ROOT]        = 64u,
    [TIMECODE_SYNTH_CONTROL_MORPH]       = 0u,
    [TIMECODE_SYNTH_CONTROL_SLOPE]       = 64u,
    [TIMECODE_SYNTH_CONTROL_SMOOTH_FOLD] = 64u,
    [TIMECODE_SYNTH_CONTROL_WARP_AMOUNT] = 0u,
};
static volatile uint32_t s_timecode_synth_control_revision = 1u;
static uint32_t s_timecode_synth_applied_revision = 0u;

static float timecode_synth_normalize_control(uint8_t value)
{
    return (float) value * (1.0f / 127.0f);
}

static float timecode_synth_smooth_fold_from_control(uint8_t value)
{
    enum
    {
        SMOOTH_FOLD_DEAD_ZONE_LOW  = 60u,
        SMOOTH_FOLD_DEAD_ZONE_HIGH = 67u,
    };

    if (value < SMOOTH_FOLD_DEAD_ZONE_LOW)
    {
        return -1.0f + ((float) value / (float) SMOOTH_FOLD_DEAD_ZONE_LOW);
    }
    if (value > SMOOTH_FOLD_DEAD_ZONE_HIGH)
    {
        return (float) (value - SMOOTH_FOLD_DEAD_ZONE_HIGH) /
               (float) (127u - SMOOTH_FOLD_DEAD_ZONE_HIGH);
    }
    return 0.0f;
}

static void timecode_synth_apply_pending_controls(void)
{
    uint8_t controls[TIMECODE_SYNTH_CONTROL_COUNT];
    uint32_t revision_before;
    uint32_t revision_after;

    do
    {
        revision_before = s_timecode_synth_control_revision;
        __DMB();
        for (uint32_t i = 0u; i < TIMECODE_SYNTH_CONTROL_COUNT; i++)
        {
            controls[i] = s_timecode_synth_control[i];
        }
        __DMB();
        revision_after = s_timecode_synth_control_revision;
    } while (revision_before != revision_after);

    if (revision_after == s_timecode_synth_applied_revision)
    {
        return;
    }

    const float root_normalized =
        timecode_synth_normalize_control(controls[TIMECODE_SYNTH_CONTROL_ROOT]);
    const float root_hz = 27.5f * powf(2.0f, 4.0f * root_normalized);
    const float morph =
        timecode_synth_normalize_control(controls[TIMECODE_SYNTH_CONTROL_MORPH]);
    const float slope = 0.1f + 0.8f *
        timecode_synth_normalize_control(controls[TIMECODE_SYNTH_CONTROL_SLOPE]);
    const float smooth_fold = timecode_synth_smooth_fold_from_control(
        controls[TIMECODE_SYNTH_CONTROL_SMOOTH_FOLD]);
    const float warp_amount =
        timecode_synth_normalize_control(controls[TIMECODE_SYNTH_CONTROL_WARP_AMOUNT]);

    for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
    {
        TimecodeOscillatorParameters_t parameters =
            *timecode_oscillator_get_parameters(&s_timecode_oscillator[channel]);
        parameters.root_hz        = root_hz;
        parameters.morph          = morph;
        parameters.slope          = slope;
        parameters.smooth_fold    = smooth_fold;
        parameters.warp_algorithm = TIMECODE_WARP_CROSSFOLD;
        parameters.warp_amount    = warp_amount;
        timecode_oscillator_set_parameters(&s_timecode_oscillator[channel], &parameters);
    }

    s_timecode_synth_applied_revision = revision_after;
}

static void timecode_synth_fifo_reset(TimecodeSynthFifo_t* fifo)
{
    fifo->write_index = 0u;
    fifo->read_index  = 0u;
    memset(fifo->samples, 0, sizeof(fifo->samples));
}

static uint32_t timecode_synth_fifo_used(TimecodeSynthFifo_t* fifo)
{
    uint32_t used = fifo->write_index - fifo->read_index;
    if (used > TIMECODE_SYNTH_FIFO_FRAMES)
    {
        fifo->read_index = fifo->write_index;
        used = 0u;
    }
    return used;
}

static void timecode_synth_fifo_push(TimecodeSynthFifo_t* fifo,
                                     const int32_t* samples,
                                     uint32_t frame_count)
{
    if (frame_count > TIMECODE_SYNTH_FIFO_FRAMES)
    {
        samples += frame_count - TIMECODE_SYNTH_FIFO_FRAMES;
        frame_count = TIMECODE_SYNTH_FIFO_FRAMES;
    }

    const uint32_t used = timecode_synth_fifo_used(fifo);
    const uint32_t free = TIMECODE_SYNTH_FIFO_FRAMES - used;
    if (frame_count > free)
    {
        const uint32_t drop_frames = frame_count - free;
        fifo->read_index += drop_frames;
        fifo->overflow_frames += drop_frames;
    }

    for (uint32_t frame = 0u; frame < frame_count; frame++)
    {
        fifo->samples[fifo->write_index & (TIMECODE_SYNTH_FIFO_FRAMES - 1u)] = samples[frame];
        fifo->write_index++;
    }
}

static int32_t timecode_synth_fifo_pop(TimecodeSynthFifo_t* fifo)
{
    if (timecode_synth_fifo_used(fifo) == 0u)
    {
        fifo->underrun_frames++;
        return 0;
    }

    const int32_t sample = fifo->samples[fifo->read_index & (TIMECODE_SYNTH_FIFO_FRAMES - 1u)];
    fifo->read_index++;
    return sample;
}

static void timecode_synth_reset_for_sample_rate(uint32_t sample_rate_hz)
{
    for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
    {
        timecode_oscillator_init(&s_timecode_oscillator[channel], sample_rate_hz);
        timecode_oscillator_set_enabled(&s_timecode_oscillator[channel],
                                        s_timecode_synth_active[channel]);
        timecode_synth_fifo_reset(&s_timecode_synth_fifo[channel]);
    }
    s_timecode_synth_applied_revision = 0u;
}

static void timecode_synth_initialize(uint32_t sample_rate_hz)
{
    for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
    {
        s_timecode_synth_active[channel] = false;
        s_timecode_synth_fifo[channel].overflow_frames = 0u;
        s_timecode_synth_fifo[channel].underrun_frames = 0u;
    }
    timecode_synth_reset_for_sample_rate(sample_rate_hz);
}

static void timecode_synth_update_modes(void)
{
    timecode_synth_apply_pending_controls();

    const bool requested[TIMECODE_SYNTH_CHANNEL_COUNT] = {
        get_current_ch1_input_mode() == UI_INPUT_MODE_SYNTH,
        get_current_ch2_input_mode() == UI_INPUT_MODE_SYNTH,
    };

    for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
    {
        if (requested[channel] == s_timecode_synth_active[channel])
        {
            continue;
        }

        s_timecode_synth_active[channel] = requested[channel];
        timecode_synth_fifo_reset(&s_timecode_synth_fifo[channel]);
        timecode_oscillator_set_enabled(&s_timecode_oscillator[channel], requested[channel]);
    }
}

static void timecode_synth_process_rx_half(uint32_t index0)
{
    const int32_t* input = stereo_in_buf + index0;
    for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
    {
        if (!s_timecode_synth_active[channel])
        {
            continue;
        }

        const uint32_t channel_offset = channel * 2u;
        timecode_oscillator_process_interleaved(&s_timecode_oscillator[channel],
                                                input + channel_offset,
                                                input + channel_offset + 1u,
                                                AUDIO_RING_FRAME_WORDS,
                                                s_timecode_synth_block[channel],
                                                TIMECODE_SYNTH_BLOCK_FRAMES);
        timecode_synth_fifo_push(&s_timecode_synth_fifo[channel],
                                 s_timecode_synth_block[channel],
                                 TIMECODE_SYNTH_BLOCK_FRAMES);
    }
}

static void timecode_synth_overlay_tx_half(uint32_t index0)
{
    int32_t* output = stereo_out_buf + index0;
    const uint32_t frame_count = (SAI_TX_BUF_SIZE / 2u) / AUDIO_RING_FRAME_WORDS;

    for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
    {
        if (!s_timecode_synth_active[channel])
        {
            continue;
        }

        const uint32_t channel_offset = channel * 2u;
        for (uint32_t frame = 0u; frame < frame_count; frame++)
        {
            const int32_t sample = timecode_synth_fifo_pop(&s_timecode_synth_fifo[channel]);
            output[frame * AUDIO_RING_FRAME_WORDS + channel_offset] = sample;
            output[frame * AUDIO_RING_FRAME_WORDS + channel_offset + 1u] = sample;
        }
    }
}
#endif

void audio_control_set_timecode_synth_control(TimecodeSynthControl_t control, uint8_t value)
{
#if ENABLE_TIMECODE_OSCILLATOR
    if ((uint32_t) control >= TIMECODE_SYNTH_CONTROL_COUNT)
    {
        return;
    }

    if (value > 127u)
    {
        value = 127u;
    }

    if (s_timecode_synth_control[control] == value)
    {
        return;
    }

    s_timecode_synth_control[control] = value;
    __DMB();
    s_timecode_synth_control_revision++;
#else
    (void) control;
    (void) value;
#endif
}

// Speaker data size received in the last frame
uint16_t spk_data_size;

// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];     // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];  // +1 for master channel 0

void control_input_from_usb_gain(uint8_t ch, int16_t db);

void reset_audio_buffer(void)
{
    ui_control_reset_state();

#if ENABLE_TIMECODE_OSCILLATOR
    timecode_synth_initialize(current_sample_rate);
#endif

    for (uint16_t i = 0; i < CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4; i++)
    {
        usb_out_buf[i] = 0;
    }

    for (uint16_t i = 0; i < CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4; i++)
    {
        usb_in_buf[i] = 0;
    }

    for (uint16_t i = 0; i < SAI_RNG_BUF_SIZE; i++)
    {
        sai_tx_rng_buf[i] = 0;
        sai_rx_rng_buf[i] = 0;
    }

    for (uint16_t i = 0; i < SAI_TX_BUF_SIZE; i++)
    {
        stereo_out_buf[i] = 0;
    }

    for (uint16_t i = 0; i < SAI_RX_BUF_SIZE; i++)
    {
        stereo_in_buf[i] = 0;
    }

    __DSB();
}

void AUDIO_LoadAndApplyRoutingFromEEPROM(void)
{
    EEPROM_DeviceConfig_t cfg;
    UI_ControlPersistState_t ui_state;

    if (EEPROM_LoadConfig(&hi2c2, &cfg) == HAL_OK)
    {
        ui_state.current_ch1_input_type = cfg.current_ch1_input_type;
        ui_state.current_ch2_input_type = cfg.current_ch2_input_type;
        ui_state.current_ch_fader_a_assign     = cfg.current_ch_fader_a_assign;
        ui_state.current_ch_fader_b_assign     = cfg.current_ch_fader_b_assign;
        ui_state.current_ch_fader_post_assign  = cfg.current_ch_fader_post_assign;
        ui_state.current_return_assign  = cfg.current_return_assign;
        ui_state.current_hp_out_source  = cfg.current_hp_out_source;
        ui_state.current_ch1_input_mode = cfg.current_ch1_input_mode;
        ui_state.current_ch2_input_mode = cfg.current_ch2_input_mode;
        ui_state.ch_fader_dvs_delay_ms = cfg.ch_fader_dvs_delay_ms;
        ui_state.sensor2_aux_fade_down_assign = cfg.sensor2_aux_fade_down_assign;
        ui_state.sensor3_aux_fade_down_assign = cfg.sensor3_aux_fade_down_assign;
        ui_state.ch_fader_reverse_a = (cfg.ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_A) != 0U;
        ui_state.ch_fader_reverse_b = (cfg.ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_B) != 0U;
        ui_state.mag_out_as_note = (cfg.mag_output_mode_flags & EEPROM_CFG_FLAG_MAG_OUT_AS_NOTE) != 0U;
        ui_state.current_ch_fader_curve_width_a = cfg.current_ch_fader_curve_width_a;
        ui_state.current_ch_fader_curve_width_b = cfg.current_ch_fader_curve_width_b;

        if (ui_control_apply_persist_state(&ui_state))
        {
            SEGGER_RTT_printf(0,
                              "EEPROM routing applied: CH1=%u CH2=%u CH_FADER_A=%u CH_FADER_B=%u CH_FADER_POST=%u RTN=%u HP=%u MODE1=%u MODE2=%u DVS_DELAY_MS=%u AUX2=%u AUX3=%u REVERSE_A=%u REVERSE_B=%u CURVE_WIDTH_A=%.4f CURVE_WIDTH_B=%.4f\r\n",
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
                              (unsigned)ui_state.ch_fader_reverse_a,
                              (unsigned)ui_state.ch_fader_reverse_b,
                              (double)cfg.current_ch_fader_curve_width_a,
                              (double)cfg.current_ch_fader_curve_width_b);
        }
        else
        {
            SEGGER_RTT_printf(0, "EEPROM routing apply failed\r\n");
        }
    }
    else
    {
        EEPROM_ConfigSetDefaults(&cfg);
        ui_state.current_ch1_input_type = cfg.current_ch1_input_type;
        ui_state.current_ch2_input_type = cfg.current_ch2_input_type;
        ui_state.current_ch_fader_a_assign     = cfg.current_ch_fader_a_assign;
        ui_state.current_ch_fader_b_assign     = cfg.current_ch_fader_b_assign;
        ui_state.current_ch_fader_post_assign  = cfg.current_ch_fader_post_assign;
        ui_state.current_return_assign  = cfg.current_return_assign;
        ui_state.current_hp_out_source  = cfg.current_hp_out_source;
        ui_state.current_ch1_input_mode = cfg.current_ch1_input_mode;
        ui_state.current_ch2_input_mode = cfg.current_ch2_input_mode;
        ui_state.ch_fader_dvs_delay_ms = cfg.ch_fader_dvs_delay_ms;
        ui_state.sensor2_aux_fade_down_assign = cfg.sensor2_aux_fade_down_assign;
        ui_state.sensor3_aux_fade_down_assign = cfg.sensor3_aux_fade_down_assign;
        ui_state.ch_fader_reverse_a = (cfg.ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_A) != 0U;
        ui_state.ch_fader_reverse_b = (cfg.ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_B) != 0U;
        ui_state.mag_out_as_note = (cfg.mag_output_mode_flags & EEPROM_CFG_FLAG_MAG_OUT_AS_NOTE) != 0U;
        ui_state.current_ch_fader_curve_width_a = cfg.current_ch_fader_curve_width_a;
        ui_state.current_ch_fader_curve_width_b = cfg.current_ch_fader_curve_width_b;
        (void)ui_control_apply_persist_state(&ui_state);

        if (EEPROM_SaveConfig(&hi2c2, &cfg) == HAL_OK)
        {
            SEGGER_RTT_printf(0, "EEPROM config initialized with defaults\r\n");
        }
        else
        {
            SEGGER_RTT_printf(0, "EEPROM default config save failed\r\n");
        }
    }
}

uint32_t get_tx_blink_interval_ms(void)
{
    return tx_blink_interval_ms;
}

uint32_t get_rx_blink_interval_ms(void)
{
    return rx_blink_interval_ms;
}

uint32_t get_current_sample_rate_hz(void)
{
    return current_sample_rate;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
    tx_blink_interval_ms = BLINK_MOUNTED;
    rx_blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    tx_blink_interval_ms = BLINK_NOT_MOUNTED;
    rx_blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    tx_blink_interval_ms = BLINK_SUSPENDED;
    rx_blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    tx_blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
    rx_blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

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
        audio20_control_cur_1_t cur_valid = {.bCur = 1};
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
                              "[USB] unsupported sample-rate request: %lu Hz\n",
                              (unsigned long) requested_sample_rate);
            return false;
        }

        current_sample_rate = requested_sample_rate;
        __DMB();
        is_sr_changed = true;
        audio_task_notify();

        SEGGER_RTT_printf(0,
                          "[USB] sample-rate request: %lu Hz\n",
                          (unsigned long) requested_sample_rate);
        TU_LOG1("Clock set current freq: %" PRIu32 "\r\n", requested_sample_rate);

        return true;
    }
    else
    {
        TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n", TU_U16_HIGH(request->wIndex), TU_U16_HIGH(request->wValue), request->bRequest);
        return false;
    }
}

// Helper for feature unit get requests
static bool audio20_feature_unit_get_request(uint8_t rhport, tusb_control_request_t const* request)
{
    TU_ASSERT(TU_U16_HIGH(request->wIndex) == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT);

    if (TU_U16_HIGH(request->wValue) == AUDIO20_FU_CTRL_MUTE && request->bRequest == AUDIO20_CS_REQ_CUR)
    {
        audio20_control_cur_1_t mute1 = {.bCur = mute[TU_U16_LOW(request->wValue)]};
        TU_LOG1("Get channel %u mute %d\r\n", TU_U16_LOW(request->wValue), mute1.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &mute1, sizeof(mute1));
    }
    else if (TU_U16_HIGH(request->wValue) == AUDIO20_FU_CTRL_VOLUME)
    {
        if (request->bRequest == AUDIO20_CS_REQ_RANGE)
        {
            audio20_control_range_2_n_t(1) range_vol = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0]   = {.bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256)}
            };
            TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", TU_U16_LOW(request->wValue), range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &range_vol, sizeof(range_vol));
        }
        else if (request->bRequest == AUDIO20_CS_REQ_CUR)
        {
            audio20_control_cur_2_t cur_vol = {.bCur = tu_htole16(volume[TU_U16_LOW(request->wValue)])};
            TU_LOG1("Get channel %u volume %d dB\r\n", TU_U16_LOW(request->wValue), cur_vol.bCur / 256);
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

    if (TU_U16_HIGH(request->wValue) == AUDIO20_FU_CTRL_MUTE)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_1_t));

        mute[TU_U16_LOW(request->wValue)] = ((audio20_control_cur_1_t const*) buf)->bCur;

        TU_LOG1("Set channel %d Mute: %d\r\n", TU_U16_LOW(request->wValue), mute[TU_U16_LOW(request->wValue)]);

        return true;
    }
    else if (TU_U16_HIGH(request->wValue) == AUDIO20_FU_CTRL_VOLUME)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_2_t));

        volume[TU_U16_LOW(request->wValue)] = ((audio20_control_cur_2_t const*) buf)->bCur;

        TU_LOG1("Set channel %d volume: %d dB\r\n", TU_U16_LOW(request->wValue), volume[TU_U16_LOW(request->wValue)] / 256);

        control_input_from_usb_gain(TU_U16_LOW(request->wValue), volume[TU_U16_LOW(request->wValue)] / 256);

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

    if (ITF_NUM_AUDIO_STREAMING_STEREO_OUT == itf && alt == 0)
    {
        tx_blink_interval_ms = BLINK_MOUNTED;
        s_streaming_out      = false;
        spk_data_size        = 0;
        usb_rx_pending       = false;
        sai_tx_rng_buf_index = 0;
        sai_transmit_index   = 0;
        tx_pending_mask      = 0;  // DMAフラグをクリア
#if AUDIO_DIAG_LOG
        dbg_usb_out_prev_cycle_valid = false;
#endif
    }

    if (ITF_NUM_AUDIO_STREAMING_STEREO_IN == itf && alt == 0)
    {
        rx_blink_interval_ms = BLINK_MOUNTED;
        s_streaming_in       = false;
        usb_tx_pending       = false;
        sai_rx_rng_buf_index = 0;
        sai_receive_index    = 0;
        rx_pending_mask      = 0;  // DMAフラグをクリア
    }

    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
    if (ITF_NUM_AUDIO_STREAMING_STEREO_OUT == itf && alt != 0)
    {
        tx_blink_interval_ms = BLINK_STREAMING;

        s_streaming_out = true;
        spk_data_size   = 0;
        usb_rx_pending  = false;
        audio_tx_diagnostics_reset();
#if AUDIO_DIAG_LOG
        dbg_usb_out_prev_cycle_valid = false;
#endif
    }

    if (ITF_NUM_AUDIO_STREAMING_STEREO_IN == itf && alt != 0)
    {
        rx_blink_interval_ms = BLINK_STREAMING;

        s_streaming_in  = true;
        usb_tx_pending  = true;
#if AUDIO_DIAG_LOG
        dbg_usb_in_notify_events++;
#endif
        audio_task_notify();
    }

    // Clear buffer when streaming is changed
    spk_data_size = 0;

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

    // Use TinyUSB's FIFO-count based feedback so host OUT packet rate follows
    // this device's effective consume rate and suppresses long-term drift.
    feedback_param->method      = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = current_sample_rate;

    // Keep FIFO around the middle to balance jitter tolerance and latency.
    feedback_param->fifo_count.fifo_threshold = (uint16_t) (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 2U);
}
#endif

static void dma_sai2_tx_half(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    s_tx_half_pending_cycle    = DWT->CYCCNT;
    s_tx_last_pending_callback = 1u;
    if (s_streaming_out)
    {
        g_audio_tx_diagnostics.tx_half_callbacks++;
        if ((tx_pending_mask & 0x01U) != 0U)
        {
            g_audio_tx_diagnostics.half_rewrite_events++;
            audio_tx_diagnostics_record_event(AUDIO_TX_DIAG_EVENT_HALF_REWRITE,
                                              audio_tx_used_words());
        }
        g_audio_tx_diagnostics.last_pending_callback = 1u;
    }
#if AUDIO_DIAG_LOG
    if ((tx_pending_mask & 0x01U) != 0U)
    {
        dbg_tx_half_rewrite_events++;
    }
#endif
    tx_pending_mask |= 0x01;
    __DMB();
    audio_task_notify_from_isr();
}
static void dma_sai2_tx_cplt(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    s_tx_cplt_pending_cycle    = DWT->CYCCNT;
    s_tx_last_pending_callback = 2u;
    if (s_streaming_out)
    {
        g_audio_tx_diagnostics.tx_cplt_callbacks++;
        if ((tx_pending_mask & 0x02U) != 0U)
        {
            g_audio_tx_diagnostics.cplt_rewrite_events++;
            audio_tx_diagnostics_record_event(AUDIO_TX_DIAG_EVENT_CPLT_REWRITE,
                                              audio_tx_used_words());
        }
        g_audio_tx_diagnostics.last_pending_callback = 2u;
    }
#if AUDIO_DIAG_LOG
    if ((tx_pending_mask & 0x02U) != 0U)
    {
        dbg_tx_cplt_rewrite_events++;
    }
#endif
    tx_pending_mask |= 0x02;
    __DMB();
    audio_task_notify_from_isr();
}

static void dma_sai1_rx_half(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
#if AUDIO_DIAG_LOG
    if ((rx_pending_mask & 0x01U) != 0U)
    {
        dbg_rx_half_rewrite_events++;
    }
#endif
    rx_pending_mask |= 0x01;
    __DMB();
    audio_task_notify_from_isr();
}
static void dma_sai1_rx_cplt(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
#if AUDIO_DIAG_LOG
    if ((rx_pending_mask & 0x02U) != 0U)
    {
        dbg_rx_cplt_rewrite_events++;
    }
#endif
    rx_pending_mask |= 0x02;
    __DMB();
    audio_task_notify_from_isr();
}

static void dma_sai_error(DMA_HandleTypeDef* hdma)
{
    SEGGER_RTT_printf(0, "DMA ERR! code=%08X\n", hdma->ErrorCode);
    if (s_streaming_out && hdma == &handle_GPDMA1_Channel2)
    {
        g_audio_tx_diagnostics.dma_error_events++;
        g_audio_tx_diagnostics.last_dma_error_code = hdma->ErrorCode;
        audio_tx_diagnostics_record_event(AUDIO_TX_DIAG_EVENT_DMA_ERROR,
                                          audio_tx_used_words());
    }
#if AUDIO_DIAG_LOG
    (void) hdma;
    dbg_dma_err_events++;
#endif
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef* hsai)
{
    uint32_t sr = hsai->Instance->SR;
    if (hsai == &hsai_BlockA2)
    {
        if (s_streaming_out)
        {
            g_audio_tx_diagnostics.sai_error_events++;
            g_audio_tx_diagnostics.last_sai_error_code  = HAL_SAI_GetError(hsai);
            g_audio_tx_diagnostics.last_sai_status_flags =
                sr & (SAI_xSR_OVRUDR | SAI_xSR_WCKCFG | SAI_xSR_CNRDY | SAI_xSR_AFSDET | SAI_xSR_LFSDET);
            audio_tx_diagnostics_record_event(AUDIO_TX_DIAG_EVENT_SAI_ERROR,
                                              audio_tx_used_words());
        }
#if AUDIO_DIAG_LOG
        dbg_sai_tx_err_events++;
        dbg_sai_tx_last_err = HAL_SAI_GetError(hsai);
        dbg_sai_tx_sr_flags |= (sr & (SAI_xSR_OVRUDR | SAI_xSR_WCKCFG | SAI_xSR_CNRDY | SAI_xSR_AFSDET | SAI_xSR_LFSDET));
#endif
    }
    else if (hsai == &hsai_BlockA1)
    {
#if AUDIO_DIAG_LOG
        dbg_sai_rx_err_events++;
        dbg_sai_rx_last_err = HAL_SAI_GetError(hsai);
        dbg_sai_rx_sr_flags |= (sr & (SAI_xSR_OVRUDR | SAI_xSR_WCKCFG | SAI_xSR_CNRDY | SAI_xSR_AFSDET | SAI_xSR_LFSDET));
#endif
    }
}

void start_sai(void)
{
    // ========================================
    // リングバッファをプリフィル（無音で初期化）
    // SAI DMAが開始直後にHalf割り込みを発生させた時、E
    // リングバッファにデータがないとアンダーランになるため、
    // 無音データを事前に投入しておく
    // 96kHzではデータレートが高いため、十分な量をプリフィルする
    // ========================================
    uint32_t prefill_size = SAI_TX_BUF_SIZE;
    memset(sai_tx_rng_buf, 0, prefill_size * sizeof(int32_t));
    sai_tx_rng_buf_index = prefill_size;
    sai_transmit_index   = 0;
    tx_pending_mask      = 0;
    audio_tx_diagnostics_reset();

#if AUDIO_DIAG_LOG
    dbg_tx_used_min            = 0xFFFFFFFFu;
    dbg_tx_used_max            = 0u;
    dbg_tx_underrun_events     = 0u;
    dbg_tx_partial_fill_events = 0u;
    dbg_tx_drift_up_events     = 0u;
    dbg_tx_drift_dn_events     = 0u;
    dbg_usb_read_zero_events   = 0u;
    dbg_usb_read_bytes         = 0u;
    dbg_dma_err_events         = 0u;
    dbg_sai_tx_err_events      = 0u;
    dbg_sai_rx_err_events      = 0u;
    dbg_sai_tx_last_err        = 0u;
    dbg_sai_rx_last_err        = 0u;
    dbg_sai_tx_sr_flags        = 0u;
    dbg_sai_rx_sr_flags        = 0u;
    dbg_tx_half_rewrite_events = 0u;
    dbg_tx_cplt_rewrite_events = 0u;
    dbg_rx_half_rewrite_events = 0u;
    dbg_rx_cplt_rewrite_events = 0u;
    dbg_usb_read_size_min      = DBG_MIN_U16_INIT;
    dbg_usb_read_size_max      = 0u;
    dbg_usb_out_packet_events  = 0u;
    dbg_usb_out_packet_bytes   = 0u;
    dbg_usb_out_packet_size_min = DBG_MIN_U16_INIT;
    dbg_usb_out_packet_size_max = 0u;
    dbg_usb_out_gap_cycles_min = DBG_MIN_U32_INIT;
    dbg_usb_out_gap_cycles_max = 0u;
    dbg_usb_out_gap_late_events = 0u;
    dbg_usb_out_prev_cycle     = 0u;
    dbg_usb_out_prev_cycle_valid = false;
    dbg_usb_out_coalesced_events = 0u;
    dbg_usb_out_pending_cycle  = 0u;
    dbg_usb_out_service_cycles_min = DBG_MIN_U32_INIT;
    dbg_usb_out_service_cycles_max = 0u;
    dbg_usb_out_fifo_min       = DBG_MIN_U16_INIT;
    dbg_usb_out_fifo_max       = 0u;
    dbg_usb_in_packet_events   = 0u;
    dbg_usb_in_packet_zero_events = 0u;
    dbg_usb_in_packet_bytes    = 0u;
    dbg_usb_in_packet_size_min = DBG_MIN_U16_INIT;
    dbg_usb_in_packet_size_max = 0u;
    dbg_usb_in_notify_events   = 0u;
    dbg_usb_in_source_wait_events = 0u;
    dbg_usb_in_write_zero_events = 0u;
    dbg_usb_in_write_partial_events = 0u;
    dbg_usb_in_write_bytes     = 0u;
    dbg_usb_in_fifo_min        = DBG_MIN_U16_INIT;
    dbg_usb_in_fifo_max        = 0u;
#endif

    // SAI2 -> Slave Transmit
    // USB -> STM32 -(SAI)-> ADAU1466
    if (MX_List_GPDMA1_Channel2_Config() != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel2, &List_GPDMA1_Channel2) != HAL_OK)
    {
        /* DMA link list error */
        Error_Handler();
    }
    handle_GPDMA1_Channel2.XferHalfCpltCallback = dma_sai2_tx_half;
    handle_GPDMA1_Channel2.XferCpltCallback     = dma_sai2_tx_cplt;
    handle_GPDMA1_Channel2.XferErrorCallback    = dma_sai_error;
    if (HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel2) != HAL_OK)
    {
        /* DMA start error */
        Error_Handler();
    }
#if 0
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*) stereo_out_buf, SAI_TX_BUF_SIZE) != HAL_OK)
    {
        /* SAI transmit start error */
        Error_Handler();
    }
#endif
    hsai_BlockA2.Instance->CR1 |= SAI_xCR1_DMAEN;  // ここでDMAリクエストを有効化
    __HAL_SAI_ENABLE(&hsai_BlockA2);

    osDelay(500);
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    // SAI1 -> Slave Receize
    // ADAU1466 -(SAI)-> STM32 -> USB
    if (MX_List_GPDMA1_Channel3_Config() != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel3, &List_GPDMA1_Channel3) != HAL_OK)
    {
        /* DMA link list error */
        Error_Handler();
    }
    handle_GPDMA1_Channel3.XferHalfCpltCallback = dma_sai1_rx_half;
    handle_GPDMA1_Channel3.XferCpltCallback     = dma_sai1_rx_cplt;
    handle_GPDMA1_Channel3.XferErrorCallback    = dma_sai_error;
    if (HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel3) != HAL_OK)
    {
        /* DMA start error */
        Error_Handler();
    }
#if 0
    if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) stereo_in_buf, SAI_RX_BUF_SIZE) != HAL_OK)
    {
        /* SAI receive start error */
        Error_Handler();
    }
#endif
    hsai_BlockA1.Instance->CR1 |= SAI_xCR1_DMAEN;  // ここでDMAリクエストを有効化
    __HAL_SAI_ENABLE(&hsai_BlockA1);
}

// ==============================
// USB(OUT) path -> Ring -> SAI(TX)
// ==============================

void copybuf_usb2ring(void)
{
    // SEGGER_RTT_printf(0, "st = %d, sb_index = %d -> ", sai_transmit_index, sai_tx_rng_buf_index);

    int32_t used = (int32_t) (sai_tx_rng_buf_index - sai_transmit_index);

    if (used < 0)
    {
        sai_transmit_index = sai_tx_rng_buf_index;
        used               = 0;
    }
    int32_t free = (int32_t) (SAI_RNG_BUF_SIZE - 1) - used;
    if (free <= 0)
    {
        return;
    }

    // USBは4ch、SAIも4chのためそのままコピー
    // USB: [L1][R1][L2][R2][L1][R1][L2][R2]...
    // SAI: [L1][R1][L2][R2][L1][R1][L2][R2]...

    // 24bit in 32bit slot: 4ch全てそのままコピー
    uint32_t sai_words = spk_data_size / sizeof(int32_t);  // SAIに書くword数(4ch分)

    if ((int32_t) sai_words > free)
    {
        sai_words = (uint32_t) free;
    }

    for (uint32_t i = 0; i < sai_words; i++)
    {
        sai_tx_rng_buf[sai_tx_rng_buf_index & (SAI_RNG_BUF_SIZE - 1)] = usb_in_buf[i];
        sai_tx_rng_buf_index++;
    }

    // SEGGER_RTT_printf(0, " %d\n", sai_tx_rng_buf_index);
}

static inline void fill_tx_half(uint32_t index0)
{
    const uint32_t n           = (SAI_TX_BUF_SIZE / 2);
    const uint32_t frame_words = 4;  // 4ch x 32bit = 1 frame
    uint32_t consume_words     = n;
    uint32_t source_skip_words = 0;
    uint32_t diagnostic_event_flags = 0u;

    // index0の範囲チェック
    if (index0 > (SAI_TX_BUF_SIZE - n))
    {
        // 不正な値は無音で埋める
        return;
    }

    int32_t used = (int32_t) (sai_tx_rng_buf_index - sai_transmit_index);
    audio_tx_diagnostics_record_level(used);
#if AUDIO_DIAG_LOG
    if (used >= 0)
    {
        uint32_t u = (uint32_t) used;
        if (u < dbg_tx_used_min)
            dbg_tx_used_min = u;
        if (u > dbg_tx_used_max)
            dbg_tx_used_max = u;
    }
#endif
    if (used < 0)
    {
        // 同期ズレは破棄して合わせ直す
        sai_transmit_index = sai_tx_rng_buf_index;
        used               = 0;
    }

    // データ不足時は可能な分だけ再生し、残りは末尾フレーム保持で埋める
    // いきなり無音にせず、クリック感を抑える
    if (used < (int32_t) n)
    {
        if (s_streaming_out)
        {
            g_audio_tx_diagnostics.underrun_events++;
            diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_UNDERRUN;
        }
#if AUDIO_DIAG_LOG
        dbg_tx_underrun_events++;
#endif
        if (used <= 0)
        {
            memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
            audio_tx_diagnostics_record_event(diagnostic_event_flags, used);
#if ENABLE_TIMECODE_OSCILLATOR
            timecode_synth_overlay_tx_half(index0);
#endif
            return;
        }
        consume_words = ((uint32_t) used / frame_words) * frame_words;
        if (consume_words == 0)
        {
            memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
            audio_tx_diagnostics_record_event(diagnostic_event_flags, used);
#if ENABLE_TIMECODE_OSCILLATOR
            timecode_synth_overlay_tx_half(index0);
#endif
            return;
        }
    }

    // usedが大きすぎる場合も異常（オーバーフロー等）
    if (used > (int32_t) SAI_RNG_BUF_SIZE)
    {
        // リセットして無音で埋める
        sai_transmit_index = sai_tx_rng_buf_index;
        memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
#if ENABLE_TIMECODE_OSCILLATOR
        timecode_synth_overlay_tx_half(index0);
#endif
        return;
    }

    // 長時間再生時のUSB/SAIクロック差を吸収するため、
    // リング水位に応じて 1 frame だけ消費量を増減する
    const int32_t target_level = (int32_t) SAI_TX_TARGET_LEVEL_WORDS;
    const int32_t high_thr     = target_level + (int32_t) (SAI_TX_BUF_SIZE / 2);
    const int32_t low_thr      = target_level - (int32_t) (SAI_TX_BUF_SIZE / 2);

    if (used >= (int32_t) n && used > high_thr && used >= (int32_t) (n + frame_words))
    {
        // バッファ過多: 最古の1 frameを捨て、DMA halfには通常量だけ書き込む
        consume_words     = n + frame_words;
        source_skip_words = frame_words;
        if (s_streaming_out)
        {
            g_audio_tx_diagnostics.drift_up_events++;
            diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_DRIFT_UP;
        }
#if AUDIO_DIAG_LOG
        dbg_tx_drift_up_events++;
#endif
    }
    else if (used >= (int32_t) n && used < low_thr && n > frame_words)
    {
        // バッファ不足傾向: 1 frame 少なく消費して追従
        consume_words = n - frame_words;
        if (s_streaming_out)
        {
            g_audio_tx_diagnostics.drift_down_events++;
            diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_DRIFT_DOWN;
        }
#if AUDIO_DIAG_LOG
        dbg_tx_drift_dn_events++;
#endif
    }

    // 安全ガード
    if ((int32_t) consume_words > used)
    {
        consume_words = (uint32_t) used;
    }
    consume_words = (consume_words / frame_words) * frame_words;

    if (consume_words == 0)
    {
        memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
        audio_tx_diagnostics_record_event(diagnostic_event_flags, used);
#if ENABLE_TIMECODE_OSCILLATOR
        timecode_synth_overlay_tx_half(index0);
#endif
        return;
    }

    if (source_skip_words > consume_words)
    {
        source_skip_words = 0;
    }
    uint32_t copy_words = consume_words - source_skip_words;
    if (copy_words > n)
    {
        copy_words = n;
    }

    const uint32_t index1 = (sai_transmit_index + source_skip_words) & (SAI_RNG_BUF_SIZE - 1);
    uint32_t first        = SAI_RNG_BUF_SIZE - index1;
    if (first > copy_words)
        first = copy_words;

    memcpy(stereo_out_buf + index0, sai_tx_rng_buf + index1, first * sizeof(int32_t));
    if (first < copy_words)
        memcpy(stereo_out_buf + index0 + first, sai_tx_rng_buf, (copy_words - first) * sizeof(int32_t));

    if (copy_words < n)
    {
        if (s_streaming_out)
        {
            g_audio_tx_diagnostics.partial_fill_events++;
            diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_PARTIAL_FILL;
        }
#if AUDIO_DIAG_LOG
        dbg_tx_partial_fill_events++;
#endif
        // 不足分は最後の1frameを繰り返し、クリックノイズを抑える
        uint32_t* dst = (uint32_t*) (stereo_out_buf + index0 + copy_words);
        uint32_t* src = (uint32_t*) (stereo_out_buf + index0 + copy_words - frame_words);
        for (uint32_t i = copy_words; i < n; i += frame_words)
        {
            dst[0] = src[0];
            dst[1] = src[1];
            dst[2] = src[2];
            dst[3] = src[3];
            dst += frame_words;
        }
    }

    sai_transmit_index += consume_words;
    audio_tx_diagnostics_record_level(used - (int32_t) consume_words);
    if (diagnostic_event_flags != 0u)
    {
        audio_tx_diagnostics_record_event(diagnostic_event_flags, used);
    }
#if ENABLE_TIMECODE_OSCILLATOR
    timecode_synth_overlay_tx_half(index0);
#endif
}

void copybuf_ring2sai(void)
{
    // ISRからの更新要求を取り出し、該当halfを更新する
    uint8_t mask;
    uint8_t last_pending_callback;
    uint32_t half_pending_cycle;
    uint32_t cplt_pending_cycle;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    mask                       = tx_pending_mask;
    last_pending_callback      = s_tx_last_pending_callback;
    half_pending_cycle         = s_tx_half_pending_cycle;
    cplt_pending_cycle         = s_tx_cplt_pending_cycle;
    tx_pending_mask            = 0;
    s_tx_last_pending_callback = 0u;
    if ((mask & 0x01u) != 0u)
    {
        s_tx_half_pending_cycle = 0u;
    }
    if ((mask & 0x02u) != 0u)
    {
        s_tx_cplt_pending_cycle = 0u;
    }
    __set_PRIMASK(primask);

    if (s_streaming_out && mask != 0u)
    {
        const uint32_t now_cycles = DWT->CYCCNT;
        if ((mask & 0x01u) != 0u)
        {
            const uint32_t service_cycles = now_cycles - half_pending_cycle;
            if (service_cycles > g_audio_tx_diagnostics.half_service_cycles_max)
            {
                g_audio_tx_diagnostics.half_service_cycles_max = service_cycles;
            }
        }
        if ((mask & 0x02u) != 0u)
        {
            const uint32_t service_cycles = now_cycles - cplt_pending_cycle;
            if (service_cycles > g_audio_tx_diagnostics.cplt_service_cycles_max)
            {
                g_audio_tx_diagnostics.cplt_service_cycles_max = service_cycles;
            }
        }
        if ((mask & 0x03u) == 0x03u)
        {
            g_audio_tx_diagnostics.both_pending_events++;
            g_audio_tx_diagnostics.both_pending_last_callback = last_pending_callback;
            audio_tx_diagnostics_record_event(AUDIO_TX_DIAG_EVENT_BOTH_PENDING,
                                              audio_tx_used_words());
        }
    }

    if (mask & 0x01)
        fill_tx_half(0);
    if (mask & 0x02)
        fill_tx_half(SAI_TX_BUF_SIZE / 2);
}

// ==============================
// SAI(RX) -> Ring -> USB(IN) path
// ==============================
static inline void fill_rx_half(uint32_t index0)
{
    const uint32_t n = (SAI_RX_BUF_SIZE / 2);  // 半分のword数

    // index0の範囲チェック
    if (index0 >= SAI_RX_BUF_SIZE)
    {
        return;
    }

#if ENABLE_TIMECODE_OSCILLATOR
    timecode_synth_process_rx_half(index0);
#endif

    int32_t used = (int32_t) (sai_rx_rng_buf_index - sai_receive_index);
    if (used < 0)
    {
        sai_receive_index = sai_rx_rng_buf_index;
        used              = 0;
    }

    // usedが大きすぎる場合も異常（オーバーフロー等）
    if (used > (int32_t) SAI_RNG_BUF_SIZE)
    {
        sai_receive_index = sai_rx_rng_buf_index;
        used              = 0;
    }

    int32_t free = (int32_t) (SAI_RNG_BUF_SIZE - 1) - used;
    if (free < (int32_t) n)
    {
        // 追いつけない時は古いデータを捨てるが、必ず4chフレーム境界で進める。
        int32_t drop_words = (int32_t) n - free;
        const int32_t frame_words = (int32_t) AUDIO_RING_FRAME_WORDS;
        drop_words = ((drop_words + frame_words - 1) / frame_words) * frame_words;
        if (drop_words > used)
        {
            drop_words = (used / frame_words) * frame_words;
        }
        sai_receive_index += (uint32_t) drop_words;
    }

    uint32_t w     = sai_rx_rng_buf_index & (SAI_RNG_BUF_SIZE - 1);
    uint32_t first = SAI_RNG_BUF_SIZE - w;
    if (first > n)
        first = n;

    memcpy(sai_rx_rng_buf + w, stereo_in_buf + index0, first * sizeof(int32_t));
    if (first < n)
        memcpy(sai_rx_rng_buf, stereo_in_buf + index0 + first, (n - first) * sizeof(int32_t));

    sai_rx_rng_buf_index += n;
}

static void copybuf_sai2ring(void)
{
    uint8_t mask;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    mask            = rx_pending_mask;
    rx_pending_mask = 0;
    __set_PRIMASK(primask);

    // 注意: half->cplt の順で両方の更新要求が溜まる場合がある
    if (mask & 0x01)
        fill_rx_half(0);
    if (mask & 0x02)
        fill_rx_half(SAI_RX_BUF_SIZE / 2);
}

// USB INエンドポイントの1転送間隔あたりのフレーム数
static uint32_t audio_frames_per_usb_in_interval(void)
{
    // 現在対応している48/96kHzはいずれも整数フレームになる。
    return (current_sample_rate * CFG_TUD_AUDIO_FUNC_1_EP_IN_INTERVAL_UFRAMES) /
           AUDIO_USB_HS_MICROFRAMES_PER_SECOND;
}

static bool audio_usb_in_source_ready(void)
{
    const uint32_t required_words = audio_frames_per_usb_in_interval() * AUDIO_RING_FRAME_WORDS;
    const int32_t available_words = (int32_t) (sai_rx_rng_buf_index - sai_receive_index);

    return available_words >= (int32_t) required_words;
}

static uint16_t audio_out_read_budget_bytes(void)
{
    int32_t used = (int32_t) (sai_tx_rng_buf_index - sai_transmit_index);
    audio_tx_diagnostics_record_level(used);
    if (used < 0)
    {
        used = 0;
    }
    if (used > (int32_t) SAI_RNG_BUF_SIZE)
    {
        used = (int32_t) SAI_RNG_BUF_SIZE;
    }

    // Keep the TX ring around target + one DMA half-buffer.
    int32_t budget_words = (int32_t) SAI_TX_TARGET_LEVEL_WORDS + (int32_t) (SAI_TX_BUF_SIZE / 2) - used;
    if (budget_words <= 0)
    {
        return 0;
    }

    budget_words = (budget_words / (int32_t) AUDIO_RING_FRAME_WORDS) * (int32_t) AUDIO_RING_FRAME_WORDS;

    uint32_t bytes = (uint32_t) budget_words * sizeof(int32_t);
    if (bytes > sizeof(usb_in_buf))
    {
        bytes = sizeof(usb_in_buf);
    }
    return (uint16_t) bytes;
}

static void copybuf_ring2usb_and_send(void)
{
    if (!tud_audio_n_mounted(AUDIO_FUNC_ID))
    {
        return;
    }

    // IN(録音)側がstreamingしていないなら送らない
    if (!s_streaming_in)
    {
        return;
    }

    tu_fifo_t* ep_in_ff = tud_audio_n_get_ep_in_ff(AUDIO_FUNC_ID);
    if (ep_in_ff == NULL)
    {
        return;
    }

#if AUDIO_DIAG_LOG
    uint16_t fifo_count = tu_fifo_count(ep_in_ff);
    if (fifo_count < dbg_usb_in_fifo_min)
    {
        dbg_usb_in_fifo_min = fifo_count;
    }
    if (fifo_count > dbg_usb_in_fifo_max)
    {
        dbg_usb_in_fifo_max = fifo_count;
    }
#endif

    const uint32_t frames    = audio_frames_per_usb_in_interval();  // 24 or 48 frames/0.5ms
    const uint32_t sai_words = frames * AUDIO_RING_FRAME_WORDS;  // 4ch(4word/frame)

    int32_t used = (int32_t) (sai_rx_rng_buf_index - sai_receive_index);
    if (used < 0)
    {
        sai_receive_index = sai_rx_rng_buf_index;
        return;
    }
    if (used < (int32_t) sai_words)
    {
#if AUDIO_DIAG_LOG
        dbg_usb_in_source_wait_events++;
#endif
        return;  // 足りないなら今回は送らない
    }

    // USBは4ch、SAIも4ch
    // SAI: [L1][R1][L2][R2][L1][R1][L2][R2]...
    // USB: [L1][R1][L2][R2][L1][R1][L2][R2]...

    // 24bit in 32bit slot: SAI(2ch) -> USB(4ch) 変換
    const uint32_t usb_bytes = frames * AUDIO_USB_FRAME_CHANNELS * sizeof(int32_t);  // 4ch分

    // 安全: usb_out_buf が足りない想定なら絶対に書かない
    if (usb_bytes > sizeof(usb_out_buf))
        return;

    bool send_ch1_to_usb = true;
    bool send_ch2_to_usb = true;
#if ENABLE_TIMECODE_OSCILLATOR
    send_ch1_to_usb = !s_timecode_synth_active[0];
    send_ch2_to_usb = !s_timecode_synth_active[1];
#endif

    for (uint32_t f = 0; f < frames; f++)
    {
        uint32_t r_L1          = (sai_receive_index + f * AUDIO_RING_FRAME_WORDS + 0) & (SAI_RNG_BUF_SIZE - 1);
        uint32_t r_R1          = (sai_receive_index + f * AUDIO_RING_FRAME_WORDS + 1) & (SAI_RNG_BUF_SIZE - 1);
        uint32_t r_L2          = (sai_receive_index + f * AUDIO_RING_FRAME_WORDS + 2) & (SAI_RNG_BUF_SIZE - 1);
        uint32_t r_R2          = (sai_receive_index + f * AUDIO_RING_FRAME_WORDS + 3) & (SAI_RNG_BUF_SIZE - 1);
        usb_out_buf[f * AUDIO_USB_FRAME_CHANNELS + 0] = send_ch1_to_usb ? sai_rx_rng_buf[r_L1] : 0;  // L1
        usb_out_buf[f * AUDIO_USB_FRAME_CHANNELS + 1] = send_ch1_to_usb ? sai_rx_rng_buf[r_R1] : 0;  // R1
        usb_out_buf[f * AUDIO_USB_FRAME_CHANNELS + 2] = send_ch2_to_usb ? sai_rx_rng_buf[r_L2] : 0;  // L2
        usb_out_buf[f * AUDIO_USB_FRAME_CHANNELS + 3] = send_ch2_to_usb ? sai_rx_rng_buf[r_R2] : 0;  // R2
    }

    // ISRコンテキストから呼ばれるので通常版を使用
    uint16_t written = tud_audio_n_write(AUDIO_FUNC_ID, usb_out_buf, (uint16_t) usb_bytes);

#if AUDIO_DIAG_LOG
    dbg_usb_in_write_bytes += written;
    if (written == 0U)
    {
        dbg_usb_in_write_zero_events++;
    }
    else if (written < usb_bytes)
    {
        dbg_usb_in_write_partial_events++;
    }

    fifo_count = tu_fifo_count(ep_in_ff);
    if (fifo_count < dbg_usb_in_fifo_min)
    {
        dbg_usb_in_fifo_min = fifo_count;
    }
    if (fifo_count > dbg_usb_in_fifo_max)
    {
        dbg_usb_in_fifo_max = fifo_count;
    }
#endif

    if (written == 0)
    {
        return;
    }

    // 書けた分だけ読みポインタを進める
    uint32_t written_frames = ((uint32_t) written) / (AUDIO_USB_FRAME_CHANNELS * sizeof(int32_t));
    if (written_frames > frames)
        written_frames = frames;
    if (written_frames == 0)
        return;
    sai_receive_index += written_frames * AUDIO_RING_FRAME_WORDS;  // SAIは4ch分
}

// TinyUSB TX完了コールバック - USB ISRコンテキストで呼ばれる
// ISR内でFIFO操作を行うとRX処理と競合するため、フラグのみ設定
bool tud_audio_tx_done_isr(uint8_t rhport, uint16_t n_bytes_sent, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) ep_in;
    (void) cur_alt_setting;
    if (func_id != AUDIO_FUNC_ID)
    {
        return true;
    }

#if AUDIO_DIAG_LOG
    dbg_usb_in_packet_events++;
    dbg_usb_in_packet_bytes += n_bytes_sent;
    if (n_bytes_sent == 0U)
    {
        dbg_usb_in_packet_zero_events++;
    }
    if (n_bytes_sent < dbg_usb_in_packet_size_min)
    {
        dbg_usb_in_packet_size_min = n_bytes_sent;
    }
    if (n_bytes_sent > dbg_usb_in_packet_size_max)
    {
        dbg_usb_in_packet_size_max = n_bytes_sent;
    }
#else
    (void) n_bytes_sent;
#endif

    // USB INの転送完了後、次の0.5ms分が揃っている時だけTaskを起こす。
    if (s_streaming_in && !usb_tx_pending && audio_usb_in_source_ready())
    {
        usb_tx_pending = true;
#if AUDIO_DIAG_LOG
        dbg_usb_in_notify_events++;
#endif
        audio_task_notify_from_isr();
    }
    return true;
}

bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) ep_out;
    (void) cur_alt_setting;
    if (func_id != AUDIO_FUNC_ID)
    {
        return true;
    }

#if AUDIO_DIAG_LOG
    const uint32_t rx_cycle = DWT->CYCCNT;

    dbg_usb_out_packet_events++;
    dbg_usb_out_packet_bytes += n_bytes_received;
    if (n_bytes_received < dbg_usb_out_packet_size_min)
    {
        dbg_usb_out_packet_size_min = n_bytes_received;
    }
    if (n_bytes_received > dbg_usb_out_packet_size_max)
    {
        dbg_usb_out_packet_size_max = n_bytes_received;
    }

    if (dbg_usb_out_prev_cycle_valid)
    {
        const uint32_t gap_cycles = rx_cycle - dbg_usb_out_prev_cycle;
        const uint32_t expected_cycles = SystemCoreClock / AUDIO_USB_HS_MICROFRAMES_PER_SECOND;
        const uint32_t late_threshold_cycles = expected_cycles + (expected_cycles / 2u);

        if (gap_cycles < dbg_usb_out_gap_cycles_min)
        {
            dbg_usb_out_gap_cycles_min = gap_cycles;
        }
        if (gap_cycles > dbg_usb_out_gap_cycles_max)
        {
            dbg_usb_out_gap_cycles_max = gap_cycles;
        }
        if (gap_cycles > late_threshold_cycles)
        {
            dbg_usb_out_gap_late_events++;
        }
    }
    dbg_usb_out_prev_cycle       = rx_cycle;
    dbg_usb_out_prev_cycle_valid = true;

    tu_fifo_t* ep_out_ff = tud_audio_n_get_ep_out_ff(AUDIO_FUNC_ID);
    if (ep_out_ff != NULL)
    {
        const uint16_t fifo_count = tu_fifo_count(ep_out_ff);
        if (fifo_count < dbg_usb_out_fifo_min)
        {
            dbg_usb_out_fifo_min = fifo_count;
        }
        if (fifo_count > dbg_usb_out_fifo_max)
        {
            dbg_usb_out_fifo_max = fifo_count;
        }
    }

    if (usb_rx_pending)
    {
        dbg_usb_out_coalesced_events++;
    }
    else
    {
        dbg_usb_out_pending_cycle = rx_cycle;
    }
#else
    (void) n_bytes_received;
#endif

    usb_rx_pending = true;
    audio_task_notify_from_isr();
    return true;
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
        spk_data_size  = 0;
        usb_tx_pending = false;
        usb_rx_pending = false;
        return;
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
        if (s_streaming_out)
        {
            int32_t tx_used_now  = (int32_t) (sai_tx_rng_buf_index - sai_transmit_index);
            uint32_t sigma_calls = sigma_spi_it_write_calls;
            uint32_t sigma_err   = sigma_spi_it_write_errors;
            uint32_t sigma_to    = sigma_spi_it_write_timeouts;
            uint32_t sigma_mto   = sigma_spi_it_mutex_timeouts;
            SEGGER_RTT_printf(0,
                               "[AUD][SUMMARY] sample_rate=%lu task_hz=%lu\r\n",
                               (unsigned long) current_sample_rate,
                               (unsigned long) audio_task_frequency);
            SEGGER_RTT_printf(0,
                               "[AUD][TX-BUF] used=%ld min/max=%lu/%lu\r\n",
                               (long) tx_used_now,
                               (unsigned long) ((dbg_tx_used_min == 0xFFFFFFFFu) ? 0u : dbg_tx_used_min),
                               (unsigned long) dbg_tx_used_max);
            SEGGER_RTT_printf(0,
                               "[AUD][TX-EVENT] underrun=%lu partial=%lu drift_up/down=%lu/%lu\r\n",
                               (unsigned long) dbg_tx_underrun_events,
                               (unsigned long) dbg_tx_partial_fill_events,
                               (unsigned long) dbg_tx_drift_up_events,
                               (unsigned long) dbg_tx_drift_dn_events);
            SEGGER_RTT_printf(0,
                               "[AUD][USB-READ] zero=%lu bytes=%lu size_min/max=%u/%u\r\n",
                               (unsigned long) dbg_usb_read_zero_events,
                               (unsigned long) dbg_usb_read_bytes,
                               (unsigned int) ((dbg_usb_read_size_min == DBG_MIN_U16_INIT) ? 0u : dbg_usb_read_size_min),
                               (unsigned int) dbg_usb_read_size_max);
            SEGGER_RTT_printf(0,
                               "[AUD][USB-IN-PKT] count=%lu zero=%lu bytes=%lu size_min/max=%u/%u\r\n",
                               (unsigned long) dbg_usb_in_packet_events,
                               (unsigned long) dbg_usb_in_packet_zero_events,
                               (unsigned long) dbg_usb_in_packet_bytes,
                               (unsigned int) ((dbg_usb_in_packet_size_min == DBG_MIN_U16_INIT) ? 0u : dbg_usb_in_packet_size_min),
                               (unsigned int) dbg_usb_in_packet_size_max);
            SEGGER_RTT_printf(0,
                               "[AUD][USB-IN-WR] notify=%lu wait=%lu zero=%lu partial=%lu\r\n",
                               (unsigned long) dbg_usb_in_notify_events,
                               (unsigned long) dbg_usb_in_source_wait_events,
                               (unsigned long) dbg_usb_in_write_zero_events,
                               (unsigned long) dbg_usb_in_write_partial_events);
            SEGGER_RTT_printf(0,
                               "[AUD][USB-IN-FIFO] bytes=%lu min/max=%u/%u\r\n",
                               (unsigned long) dbg_usb_in_write_bytes,
                               (unsigned int) ((dbg_usb_in_fifo_min == DBG_MIN_U16_INIT) ? 0u : dbg_usb_in_fifo_min),
                               (unsigned int) dbg_usb_in_fifo_max);
            SEGGER_RTT_printf(0,
                               "[AUD][REWRITE] tx_half/cplt=%lu/%lu rx_half/cplt=%lu/%lu\r\n",
                               (unsigned long) dbg_tx_half_rewrite_events,
                               (unsigned long) dbg_tx_cplt_rewrite_events,
                               (unsigned long) dbg_rx_half_rewrite_events,
                               (unsigned long) dbg_rx_cplt_rewrite_events);
            SEGGER_RTT_printf(0,
                               "[AUD][SAI] dma_err=%lu tx_err=%lu rx_err=%lu\r\n",
                               (unsigned long) dbg_dma_err_events,
                               (unsigned long) dbg_sai_tx_err_events,
                               (unsigned long) dbg_sai_rx_err_events);
            SEGGER_RTT_printf(0,
                               "[AUD][SAI-REG] tx_err=0x%08lX rx_err=0x%08lX tx_sr=0x%08lX rx_sr=0x%08lX\r\n",
                               (unsigned long) dbg_sai_tx_last_err,
                               (unsigned long) dbg_sai_rx_last_err,
                               (unsigned long) dbg_sai_tx_sr_flags,
                               (unsigned long) dbg_sai_rx_sr_flags);
            SEGGER_RTT_printf(0,
                               "[AUD][SPI] calls=%lu errors=%lu timeouts=%lu mutex_timeouts=%lu\r\n",
                               (unsigned long) (sigma_calls - dbg_sigma_calls_prev),
                               (unsigned long) (sigma_err - dbg_sigma_err_prev),
                               (unsigned long) (sigma_to - dbg_sigma_to_prev),
                               (unsigned long) (sigma_mto - dbg_sigma_mto_prev));
            SEGGER_RTT_printf(0,
                               "[AUD][USB-OUT-PKT] packets=%lu bytes=%lu size_min/max=%u/%u\r\n",
                               (unsigned long) dbg_usb_out_packet_events,
                               (unsigned long) dbg_usb_out_packet_bytes,
                               (unsigned int) ((dbg_usb_out_packet_size_min == DBG_MIN_U16_INIT) ? 0u : dbg_usb_out_packet_size_min),
                               (unsigned int) dbg_usb_out_packet_size_max);
            SEGGER_RTT_printf(0,
                               "[AUD][USB-OUT-FIFO] min/max=%u/%u\r\n",
                               (unsigned int) ((dbg_usb_out_fifo_min == DBG_MIN_U16_INIT) ? 0u : dbg_usb_out_fifo_min),
                               (unsigned int) dbg_usb_out_fifo_max);
            SEGGER_RTT_printf(0,
                               "[AUD][USB-OUT-GAP] us_min/max=%lu/%lu late=%lu coalesced=%lu\r\n",
                               (unsigned long) ((dbg_usb_out_gap_cycles_min == DBG_MIN_U32_INIT) ? 0u : audio_diag_cycles_to_us(dbg_usb_out_gap_cycles_min)),
                               (unsigned long) audio_diag_cycles_to_us(dbg_usb_out_gap_cycles_max),
                               (unsigned long) dbg_usb_out_gap_late_events,
                               (unsigned long) dbg_usb_out_coalesced_events);
            SEGGER_RTT_printf(0,
                               "[AUD][USB-OUT-SERVICE] us_min/max=%lu/%lu\r\n",
                               (unsigned long) ((dbg_usb_out_service_cycles_min == DBG_MIN_U32_INIT) ? 0u : audio_diag_cycles_to_us(dbg_usb_out_service_cycles_min)),
                               (unsigned long) audio_diag_cycles_to_us(dbg_usb_out_service_cycles_max));
            dbg_sigma_calls_prev = sigma_calls;
            dbg_sigma_err_prev   = sigma_err;
            dbg_sigma_to_prev    = sigma_to;
            dbg_sigma_mto_prev   = sigma_mto;
        }
        dbg_tx_used_min            = 0xFFFFFFFFu;
        dbg_tx_used_max            = 0u;
        dbg_tx_underrun_events     = 0u;
        dbg_tx_partial_fill_events = 0u;
        dbg_tx_drift_up_events     = 0u;
        dbg_tx_drift_dn_events     = 0u;
        dbg_usb_read_zero_events   = 0u;
        dbg_usb_read_bytes         = 0u;
        dbg_dma_err_events         = 0u;
        dbg_sai_tx_err_events      = 0u;
        dbg_sai_rx_err_events      = 0u;
        dbg_sai_tx_last_err        = 0u;
        dbg_sai_rx_last_err        = 0u;
        dbg_sai_tx_sr_flags        = 0u;
        dbg_sai_rx_sr_flags        = 0u;
        dbg_tx_half_rewrite_events = 0u;
        dbg_tx_cplt_rewrite_events = 0u;
        dbg_rx_half_rewrite_events = 0u;
        dbg_rx_cplt_rewrite_events = 0u;
        dbg_usb_read_size_min      = DBG_MIN_U16_INIT;
        dbg_usb_read_size_max      = 0u;
        dbg_usb_out_packet_events  = 0u;
        dbg_usb_out_packet_bytes   = 0u;
        dbg_usb_out_packet_size_min = DBG_MIN_U16_INIT;
        dbg_usb_out_packet_size_max = 0u;
        dbg_usb_out_gap_cycles_min = DBG_MIN_U32_INIT;
        dbg_usb_out_gap_cycles_max = 0u;
        dbg_usb_out_gap_late_events = 0u;
        dbg_usb_out_coalesced_events = 0u;
        dbg_usb_out_service_cycles_min = DBG_MIN_U32_INIT;
        dbg_usb_out_service_cycles_max = 0u;
        dbg_usb_out_fifo_min       = DBG_MIN_U16_INIT;
        dbg_usb_out_fifo_max       = 0u;
        dbg_usb_in_packet_events   = 0u;
        dbg_usb_in_packet_zero_events = 0u;
        dbg_usb_in_packet_bytes    = 0u;
        dbg_usb_in_packet_size_min = DBG_MIN_U16_INIT;
        dbg_usb_in_packet_size_max = 0u;
        dbg_usb_in_notify_events   = 0u;
        dbg_usb_in_source_wait_events = 0u;
        dbg_usb_in_write_zero_events = 0u;
        dbg_usb_in_write_partial_events = 0u;
        dbg_usb_in_write_bytes     = 0u;
        dbg_usb_in_fifo_min        = DBG_MIN_U16_INIT;
        dbg_usb_in_fifo_max        = 0u;
#endif
    }

    if (is_sr_changed)
    {
        // Consume the current request before doing the blocking reconfiguration.
        // A new SET_CUR received during the switch will set the flag again.
        is_sr_changed = false;
        __DMB();
#if RESET_FROM_FW
        AUDIO_SAI_Reset_ForNewRate();
#endif
    }
    else
    {
#if ENABLE_TIMECODE_OSCILLATOR
        timecode_synth_update_modes();
#endif

        bool usb_rx_event = false;
        bool usb_tx_event = false;
#if AUDIO_DIAG_LOG
        uint32_t usb_rx_event_cycle = 0u;
#endif

        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (usb_rx_pending)
        {
            usb_rx_pending = false;
            usb_rx_event   = true;
#if AUDIO_DIAG_LOG
            usb_rx_event_cycle = dbg_usb_out_pending_cycle;
#endif
        }
        if (usb_tx_pending)
        {
            usb_tx_pending = false;
            usb_tx_event   = true;
        }
        __set_PRIMASK(primask);

#if AUDIO_DIAG_LOG
        if (usb_rx_event)
        {
            const uint32_t service_cycles = DWT->CYCCNT - usb_rx_event_cycle;
            if (service_cycles < dbg_usb_out_service_cycles_min)
            {
                dbg_usb_out_service_cycles_min = service_cycles;
            }
            if (service_cycles > dbg_usb_out_service_cycles_max)
            {
                dbg_usb_out_service_cycles_max = service_cycles;
            }
        }
#endif

        // USB OUTは受信通知とFIFO残量に追従して即時に吸い出す。
        if (usb_rx_event || tud_audio_n_available(AUDIO_FUNC_ID) > 0U)
        {
            uint16_t budget = audio_out_read_budget_bytes();
            uint16_t avail = tud_audio_n_available(AUDIO_FUNC_ID);
            uint16_t to_read = (avail < budget) ? avail : budget;
            if (to_read > sizeof(usb_in_buf))
            {
                to_read = (uint16_t) sizeof(usb_in_buf);
            }

            if (to_read > 0U)
            {
                spk_data_size = tud_audio_n_read(AUDIO_FUNC_ID, usb_in_buf, to_read);
            }
            else
            {
                spk_data_size = 0;
            }
        }
        else
        {
            spk_data_size = 0;
        }
#if AUDIO_DIAG_LOG
        dbg_usb_read_bytes += spk_data_size;
        if (s_streaming_out && spk_data_size == 0)
        {
            dbg_usb_read_zero_events++;
        }
        if (spk_data_size < dbg_usb_read_size_min)
        {
            dbg_usb_read_size_min = spk_data_size;
        }
        if (spk_data_size > dbg_usb_read_size_max)
        {
            dbg_usb_read_size_max = spk_data_size;
        }
#endif

        // USB -> SAI
        if (spk_data_size > 0)
        {
            copybuf_usb2ring();
        }
        copybuf_ring2sai();

        // SAI -> USB
        copybuf_sai2ring();

        // USB INはエンドポイントの1転送間隔分（現在0.5ms）が揃ったら次の塊を積む。
        // SAI RX DMA通知でも補充することで、IN FIFOが空になった場合の停止を防ぐ。
        if (s_streaming_in && (usb_tx_event || audio_usb_in_source_ready()))
        {
            copybuf_ring2usb_and_send();
        }

    }
}

void AUDIO_SAI_Reset_ForNewRate(void)
{
    static uint32_t prev_hz = 48000;
    const uint32_t new_hz   = current_sample_rate;

    if (new_hz == prev_hz)
    {
        return;
    }

    /* Stop ADC DMA to prevent concurrent DSP parameter writes during the path switch. */
    (void) HAL_ADC_Stop(&hadc1);
    (void) HAL_DMA_Abort(&handle_HPDMA1_Channel0);
    ui_control_set_adc_complete(false);
    __DSB();

    /* Disable interrupts during critical DMA/SAI stop sequence */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* Disable SAI DMA requests first */
    hsai_BlockA2.Instance->CR1 &= ~SAI_xCR1_DMAEN;
    hsai_BlockA1.Instance->CR1 &= ~SAI_xCR1_DMAEN;

    /* Disable SAI blocks */
    __HAL_SAI_DISABLE(&hsai_BlockA2);
    __HAL_SAI_DISABLE(&hsai_BlockA1);
    __DSB();

    __set_PRIMASK(primask);

    /* Abort DMA transfers */
    (void) HAL_DMA_Abort(&handle_GPDMA1_Channel2);
    (void) HAL_DMA_Abort(&handle_GPDMA1_Channel3);
    __DSB();

    /* Fully re-init SAI blocks so FIFOs/flags are reset as well */
    (void) HAL_SAI_DeInit(&hsai_BlockA2);
    (void) HAL_SAI_DeInit(&hsai_BlockA1);

    sai_tx_rng_buf_index = 0;
    sai_rx_rng_buf_index = 0;
    sai_transmit_index   = 0;
    sai_receive_index    = 0;
    tx_pending_mask      = 0;
    rx_pending_mask      = 0;
    audio_tx_diagnostics_reset();

#if AUDIO_DIAG_LOG
    dbg_tx_used_min            = 0xFFFFFFFFu;
    dbg_tx_used_max            = 0u;
    dbg_tx_underrun_events     = 0u;
    dbg_tx_partial_fill_events = 0u;
    dbg_tx_drift_up_events     = 0u;
    dbg_tx_drift_dn_events     = 0u;
    dbg_usb_read_zero_events   = 0u;
    dbg_usb_read_bytes         = 0u;
    dbg_dma_err_events         = 0u;
    dbg_sai_tx_err_events      = 0u;
    dbg_sai_rx_err_events      = 0u;
    dbg_sai_tx_last_err        = 0u;
    dbg_sai_rx_last_err        = 0u;
    dbg_sai_tx_sr_flags        = 0u;
    dbg_sai_rx_sr_flags        = 0u;
    dbg_tx_half_rewrite_events = 0u;
    dbg_tx_cplt_rewrite_events = 0u;
    dbg_rx_half_rewrite_events = 0u;
    dbg_rx_cplt_rewrite_events = 0u;
    dbg_usb_read_size_min      = DBG_MIN_U16_INIT;
    dbg_usb_read_size_max      = 0u;
    dbg_usb_out_packet_events  = 0u;
    dbg_usb_out_packet_bytes   = 0u;
    dbg_usb_out_packet_size_min = DBG_MIN_U16_INIT;
    dbg_usb_out_packet_size_max = 0u;
    dbg_usb_out_gap_cycles_min = DBG_MIN_U32_INIT;
    dbg_usb_out_gap_cycles_max = 0u;
    dbg_usb_out_gap_late_events = 0u;
    dbg_usb_out_prev_cycle     = 0u;
    dbg_usb_out_prev_cycle_valid = false;
    dbg_usb_out_coalesced_events = 0u;
    dbg_usb_out_pending_cycle  = 0u;
    dbg_usb_out_service_cycles_min = DBG_MIN_U32_INIT;
    dbg_usb_out_service_cycles_max = 0u;
    dbg_usb_out_fifo_min       = DBG_MIN_U16_INIT;
    dbg_usb_out_fifo_max       = 0u;
    dbg_usb_in_packet_events   = 0u;
    dbg_usb_in_packet_zero_events = 0u;
    dbg_usb_in_packet_bytes    = 0u;
    dbg_usb_in_packet_size_min = DBG_MIN_U16_INIT;
    dbg_usb_in_packet_size_max = 0u;
    dbg_usb_in_source_wait_events = 0u;
    dbg_usb_in_write_zero_events = 0u;
    dbg_usb_in_write_partial_events = 0u;
    dbg_usb_in_write_bytes     = 0u;
    dbg_usb_in_fifo_min        = DBG_MIN_U16_INIT;
    dbg_usb_in_fifo_max        = 0u;
#endif

    /* Clear all audio buffers to avoid noise from stale data */
    memset(sai_tx_rng_buf, 0, sizeof(sai_tx_rng_buf));
    memset(sai_rx_rng_buf, 0, sizeof(sai_rx_rng_buf));
    memset(stereo_out_buf, 0, sizeof(stereo_out_buf));
    memset(stereo_in_buf, 0, sizeof(stereo_in_buf));
    memset(usb_in_buf, 0, sizeof(usb_in_buf));
    memset(usb_out_buf, 0, sizeof(usb_out_buf));
#if ENABLE_TIMECODE_OSCILLATOR
    timecode_synth_reset_for_sample_rate(new_hz);
#endif
    __DSB();

#if RESET_FROM_FW
    // The DSP core and AK4619 link stay at 96 kHz. Only the USB-side clock and
    // ADAU1466 ASRC/direct routing change, so program and parameter RAM remain
    // intact and no routing/UI state restoration is required.
    if (!AUDIO_Update_ADAU1466_SampleRate(new_hz))
    {
        SEGGER_RTT_printf(0, "[AUD] ADAU1466 rate switch failed: %lu Hz\n", (unsigned long) new_hz);
    }
#endif

    /* Re-init DMA channels (linked-list mode) */
    (void) HAL_DMA_DeInit(&handle_GPDMA1_Channel2);
    (void) HAL_DMA_DeInit(&handle_GPDMA1_Channel3);

    handle_GPDMA1_Channel2.Instance                         = GPDMA1_Channel2;
    handle_GPDMA1_Channel2.InitLinkedList.Priority          = DMA_LOW_PRIORITY_HIGH_WEIGHT;
    handle_GPDMA1_Channel2.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
    handle_GPDMA1_Channel2.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
    handle_GPDMA1_Channel2.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
    handle_GPDMA1_Channel2.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
    if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel2, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
        Error_Handler();
    }

    handle_GPDMA1_Channel3.Instance                         = GPDMA1_Channel3;
    handle_GPDMA1_Channel3.InitLinkedList.Priority          = DMA_LOW_PRIORITY_HIGH_WEIGHT;
    handle_GPDMA1_Channel3.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
    handle_GPDMA1_Channel3.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
    handle_GPDMA1_Channel3.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
    handle_GPDMA1_Channel3.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
    if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel3, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
        Error_Handler();
    }

    /* Reconfigure peripherals (SAI) */
    MX_SAI1_Init();
    MX_SAI2_Init();

    /* Prefill TX ring buffer with silence (already zeroed above) */
    /* Set write index ahead to provide initial data for DMA */
    /* 96kHz needs larger prefill due to higher data rate */
    sai_tx_rng_buf_index = SAI_TX_BUF_SIZE;
    sai_transmit_index   = 0;

    /* Configure and link DMA for SAI2 TX */
    if (MX_List_GPDMA1_Channel2_Config() != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel2, &List_GPDMA1_Channel2) != HAL_OK)
    {
        Error_Handler();
    }
    handle_GPDMA1_Channel2.XferHalfCpltCallback = dma_sai2_tx_half;
    handle_GPDMA1_Channel2.XferCpltCallback     = dma_sai2_tx_cplt;
    handle_GPDMA1_Channel2.XferErrorCallback    = dma_sai_error;
    if (HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel2) != HAL_OK)
    {
        Error_Handler();
    }
    hsai_BlockA2.Instance->CR1 |= SAI_xCR1_DMAEN;
    __HAL_SAI_ENABLE(&hsai_BlockA2);

    /* Wait for SAI TX to synchronize with external clock before starting RX */
    osDelay(10);

    /* Configure and link DMA for SAI1 RX */
    if (MX_List_GPDMA1_Channel3_Config() != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel3, &List_GPDMA1_Channel3) != HAL_OK)
    {
        Error_Handler();
    }
    handle_GPDMA1_Channel3.XferHalfCpltCallback = dma_sai1_rx_half;
    handle_GPDMA1_Channel3.XferCpltCallback     = dma_sai1_rx_cplt;
    handle_GPDMA1_Channel3.XferErrorCallback    = dma_sai_error;
    if (HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel3) != HAL_OK)
    {
        Error_Handler();
    }
    hsai_BlockA1.Instance->CR1 |= SAI_xCR1_DMAEN;
    __HAL_SAI_ENABLE(&hsai_BlockA1);

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

    SEGGER_RTT_printf(0, "[SAI] reset for %lu Hz (prev=%lu)\n", (unsigned long) new_hz, (unsigned long) prev_hz);

    prev_hz = new_hz;
}
