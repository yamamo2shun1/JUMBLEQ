/*
 * audio_diagnostics.c
 *
 * Audio diagnostics state, recording helpers and the periodic RTT summary.
 *
 * All record functions are called from ISR context and only update fixed-size
 * counters/min-max values. RTT output happens exclusively in the Audio Task
 * through audio_diagnostics_log_periodic().
 */

#include "audio_diagnostics_internal.h"

#include "SEGGER_RTT.h"
#include "stm32h7rsxx_hal.h"

#include "SigmaStudioFW.h"

#define DBG_MIN_U32_INIT UINT32_MAX

enum
{
    DBG_MIN_U16_INIT = 0xFFFFu,
    AUDIO_USB_HS_MICROFRAMES_PER_SECOND = 8000u,
};

volatile AudioTxDiagnostics_t g_audio_tx_diagnostics = {
    .tx_used_min_words = UINT32_MAX,
};

#if AUDIO_DIAG_LOG
static volatile uint32_t dbg_tx_used_min            = DBG_MIN_U32_INIT;
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

static void audio_diag_reset_interval_locked(void)
{
    dbg_tx_used_min            = DBG_MIN_U32_INIT;
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
}

static uint32_t audio_diag_cycles_to_us(uint32_t cycles)
{
    if (SystemCoreClock == 0u)
    {
        return 0u;
    }

    return (uint32_t) ((((uint64_t) cycles * 1000000u) + (SystemCoreClock / 2u)) / SystemCoreClock);
}
#endif

void audio_diagnostics_reset_tx_locked(void)
{
    const uint32_t reset_count = g_audio_tx_diagnostics.reset_count + 1u;
    AudioTxDiagnostics_t reset = {0};
    reset.reset_count          = reset_count;
    reset.tx_used_min_words    = UINT32_MAX;
    g_audio_tx_diagnostics     = reset;
}

#if AUDIO_DIAG_LOG
void audio_diagnostics_reset_interval(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    audio_diag_reset_interval_locked();

    __set_PRIMASK(primask);
}

void audio_diagnostics_reset_session(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    audio_diag_reset_interval_locked();
    dbg_usb_out_prev_cycle       = 0u;
    dbg_usb_out_prev_cycle_valid = false;
    dbg_usb_out_pending_cycle    = 0u;

    __set_PRIMASK(primask);
}

void audio_diagnostics_reset_usb_out_gap(void)
{
    dbg_usb_out_prev_cycle_valid = false;
}
#endif

void audio_diagnostics_record_tx_level(bool streaming, int32_t used)
{
    if (!streaming || used < 0)
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

#if AUDIO_DIAG_LOG
void audio_diagnostics_record_tx_interval_level(int32_t used)
{
    if (used < 0)
    {
        return;
    }

    const uint32_t level = (uint32_t) used;
    if (level < dbg_tx_used_min)
    {
        dbg_tx_used_min = level;
    }
    if (level > dbg_tx_used_max)
    {
        dbg_tx_used_max = level;
    }
}
#endif

void audio_diagnostics_record_tx_event(bool streaming, uint32_t flags, int32_t used)
{
    if (streaming)
    {
        if ((flags & AUDIO_TX_DIAG_EVENT_UNDERRUN) != 0u)
        {
            g_audio_tx_diagnostics.underrun_events++;
        }
        if ((flags & AUDIO_TX_DIAG_EVENT_PARTIAL_FILL) != 0u)
        {
            g_audio_tx_diagnostics.partial_fill_events++;
        }
        if ((flags & AUDIO_TX_DIAG_EVENT_DRIFT_UP) != 0u)
        {
            g_audio_tx_diagnostics.drift_up_events++;
        }
        if ((flags & AUDIO_TX_DIAG_EVENT_DRIFT_DOWN) != 0u)
        {
            g_audio_tx_diagnostics.drift_down_events++;
        }

        g_audio_tx_diagnostics.last_event_used_words = used;
        g_audio_tx_diagnostics.last_event_flags      = flags;
        g_audio_tx_diagnostics.last_event_tick_ms    = HAL_GetTick();
        g_audio_tx_diagnostics.last_event_cycle      = DWT->CYCCNT;
    }

#if AUDIO_DIAG_LOG
    if ((flags & AUDIO_TX_DIAG_EVENT_UNDERRUN) != 0u)
    {
        dbg_tx_underrun_events++;
    }
    if ((flags & AUDIO_TX_DIAG_EVENT_PARTIAL_FILL) != 0u)
    {
        dbg_tx_partial_fill_events++;
    }
    if ((flags & AUDIO_TX_DIAG_EVENT_DRIFT_UP) != 0u)
    {
        dbg_tx_drift_up_events++;
    }
    if ((flags & AUDIO_TX_DIAG_EVENT_DRIFT_DOWN) != 0u)
    {
        dbg_tx_drift_dn_events++;
    }
#endif
}

void audio_diagnostics_record_tx_dma_callback(uint32_t callback_event,
                                              uint32_t overwritten_event,
                                              bool streaming,
                                              int32_t used)
{
    if (streaming)
    {
        if (callback_event == AUDIO_DIAG_DMA_EVENT_HALF)
        {
            g_audio_tx_diagnostics.tx_half_callbacks++;
        }
        else
        {
            g_audio_tx_diagnostics.tx_cplt_callbacks++;
        }

        if (overwritten_event == AUDIO_DIAG_DMA_EVENT_HALF)
        {
            g_audio_tx_diagnostics.half_rewrite_events++;
            audio_diagnostics_record_tx_event(true, AUDIO_TX_DIAG_EVENT_HALF_REWRITE, used);
        }
        else if (overwritten_event == AUDIO_DIAG_DMA_EVENT_COMPLETE)
        {
            g_audio_tx_diagnostics.cplt_rewrite_events++;
            audio_diagnostics_record_tx_event(true, AUDIO_TX_DIAG_EVENT_CPLT_REWRITE, used);
        }
        g_audio_tx_diagnostics.last_pending_callback = callback_event;
    }

#if AUDIO_DIAG_LOG
    if (overwritten_event == AUDIO_DIAG_DMA_EVENT_HALF)
    {
        dbg_tx_half_rewrite_events++;
    }
    else if (overwritten_event == AUDIO_DIAG_DMA_EVENT_COMPLETE)
    {
        dbg_tx_cplt_rewrite_events++;
    }
#endif
}

void audio_diagnostics_record_rx_dma_callback(uint32_t callback_event,
                                              uint32_t overwritten_event)
{
    (void) callback_event;

#if AUDIO_DIAG_LOG
    if (overwritten_event == AUDIO_DIAG_DMA_EVENT_HALF)
    {
        dbg_rx_half_rewrite_events++;
    }
    else if (overwritten_event == AUDIO_DIAG_DMA_EVENT_COMPLETE)
    {
        dbg_rx_cplt_rewrite_events++;
    }
#else
    (void) overwritten_event;
#endif
}

void audio_diagnostics_record_tx_dma_service(uint32_t callback_event,
                                             uint32_t service_cycles)
{
    if (callback_event == AUDIO_DIAG_DMA_EVENT_HALF)
    {
        if (service_cycles > g_audio_tx_diagnostics.half_service_cycles_max)
        {
            g_audio_tx_diagnostics.half_service_cycles_max = service_cycles;
        }
    }
    else if (callback_event == AUDIO_DIAG_DMA_EVENT_COMPLETE)
    {
        if (service_cycles > g_audio_tx_diagnostics.cplt_service_cycles_max)
        {
            g_audio_tx_diagnostics.cplt_service_cycles_max = service_cycles;
        }
    }
}

void audio_diagnostics_record_tx_events_dropped(uint32_t last_event,
                                                uint32_t dropped_events,
                                                int32_t used)
{
    g_audio_tx_diagnostics.both_pending_events++;
    g_audio_tx_diagnostics.dma_events_dropped += dropped_events;
    g_audio_tx_diagnostics.both_pending_last_callback = last_event;
    audio_diagnostics_record_tx_event(true, AUDIO_TX_DIAG_EVENT_BOTH_PENDING, used);
}

void audio_diagnostics_record_dma_error(uint32_t error_code,
                                        bool tx_streaming,
                                        int32_t used)
{
    if (tx_streaming)
    {
        g_audio_tx_diagnostics.dma_error_events++;
        g_audio_tx_diagnostics.last_dma_error_code = error_code;
        audio_diagnostics_record_tx_event(true, AUDIO_TX_DIAG_EVENT_DMA_ERROR, used);
    }
#if AUDIO_DIAG_LOG
    dbg_dma_err_events++;
#endif
}

void audio_diagnostics_record_sai_tx_error(uint32_t error_code,
                                           uint32_t status_flags,
                                           bool streaming,
                                           int32_t used)
{
    if (streaming)
    {
        g_audio_tx_diagnostics.sai_error_events++;
        g_audio_tx_diagnostics.last_sai_error_code   = error_code;
        g_audio_tx_diagnostics.last_sai_status_flags = status_flags;
        audio_diagnostics_record_tx_event(true, AUDIO_TX_DIAG_EVENT_SAI_ERROR, used);
    }
#if AUDIO_DIAG_LOG
    dbg_sai_tx_err_events++;
    dbg_sai_tx_last_err = error_code;
    dbg_sai_tx_sr_flags |= status_flags;
#endif
}

void audio_diagnostics_record_sai_rx_error(uint32_t error_code,
                                           uint32_t status_flags)
{
#if AUDIO_DIAG_LOG
    dbg_sai_rx_err_events++;
    dbg_sai_rx_last_err = error_code;
    dbg_sai_rx_sr_flags |= status_flags;
#else
    (void) error_code;
    (void) status_flags;
#endif
}

#if AUDIO_DIAG_LOG
void audio_diagnostics_record_usb_out_packet(uint16_t bytes,
                                             uint32_t rx_cycle,
                                             bool pending)
{
    dbg_usb_out_packet_events++;
    dbg_usb_out_packet_bytes += bytes;
    if (bytes < dbg_usb_out_packet_size_min)
    {
        dbg_usb_out_packet_size_min = bytes;
    }
    if (bytes > dbg_usb_out_packet_size_max)
    {
        dbg_usb_out_packet_size_max = bytes;
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

    if (pending)
    {
        dbg_usb_out_coalesced_events++;
    }
    else
    {
        dbg_usb_out_pending_cycle = rx_cycle;
    }
}

void audio_diagnostics_record_usb_out_fifo(uint16_t fifo_count)
{
    if (fifo_count < dbg_usb_out_fifo_min)
    {
        dbg_usb_out_fifo_min = fifo_count;
    }
    if (fifo_count > dbg_usb_out_fifo_max)
    {
        dbg_usb_out_fifo_max = fifo_count;
    }
}

uint32_t audio_diagnostics_usb_out_pending_cycle(void)
{
    return dbg_usb_out_pending_cycle;
}

void audio_diagnostics_record_usb_out_service(uint32_t pending_cycle)
{
    const uint32_t service_cycles = DWT->CYCCNT - pending_cycle;
    if (service_cycles < dbg_usb_out_service_cycles_min)
    {
        dbg_usb_out_service_cycles_min = service_cycles;
    }
    if (service_cycles > dbg_usb_out_service_cycles_max)
    {
        dbg_usb_out_service_cycles_max = service_cycles;
    }
}

void audio_diagnostics_record_usb_out_read(bool streaming, uint16_t bytes)
{
    dbg_usb_read_bytes += bytes;
    if (streaming && bytes == 0u)
    {
        dbg_usb_read_zero_events++;
    }
    if (bytes < dbg_usb_read_size_min)
    {
        dbg_usb_read_size_min = bytes;
    }
    if (bytes > dbg_usb_read_size_max)
    {
        dbg_usb_read_size_max = bytes;
    }
}

void audio_diagnostics_record_usb_in_packet(uint16_t bytes)
{
    dbg_usb_in_packet_events++;
    dbg_usb_in_packet_bytes += bytes;
    if (bytes == 0u)
    {
        dbg_usb_in_packet_zero_events++;
    }
    if (bytes < dbg_usb_in_packet_size_min)
    {
        dbg_usb_in_packet_size_min = bytes;
    }
    if (bytes > dbg_usb_in_packet_size_max)
    {
        dbg_usb_in_packet_size_max = bytes;
    }
}

void audio_diagnostics_record_usb_in_notify(void)
{
    dbg_usb_in_notify_events++;
}

void audio_diagnostics_record_usb_in_source_wait(void)
{
    dbg_usb_in_source_wait_events++;
}

void audio_diagnostics_record_usb_in_write(uint16_t written, uint16_t requested)
{
    dbg_usb_in_write_bytes += written;
    if (written == 0U)
    {
        dbg_usb_in_write_zero_events++;
    }
    else if (written < requested)
    {
        dbg_usb_in_write_partial_events++;
    }
}

void audio_diagnostics_record_usb_in_fifo(uint16_t fifo_count)
{
    if (fifo_count < dbg_usb_in_fifo_min)
    {
        dbg_usb_in_fifo_min = fifo_count;
    }
    if (fifo_count > dbg_usb_in_fifo_max)
    {
        dbg_usb_in_fifo_max = fifo_count;
    }
}
#endif

void audio_diagnostics_log_periodic(uint32_t sample_rate_hz,
                                    uint32_t task_frequency_hz,
                                    bool streaming_out,
                                    int32_t tx_used_words)
{
#if AUDIO_DIAG_LOG
    if (!streaming_out)
    {
        return;
    }

    uint32_t sigma_calls = sigma_spi_it_write_calls;
    uint32_t sigma_err   = sigma_spi_it_write_errors;
    uint32_t sigma_to    = sigma_spi_it_write_timeouts;
    uint32_t sigma_mto   = sigma_spi_it_mutex_timeouts;
    SEGGER_RTT_printf(0,
                      "[AUD][SUMMARY] sample_rate=%lu task_hz=%lu\r\n",
                      (unsigned long) sample_rate_hz,
                      (unsigned long) task_frequency_hz);
    SEGGER_RTT_printf(0,
                      "[AUD][TX-BUF] used=%ld min/max=%lu/%lu\r\n",
                      (long) tx_used_words,
                      (unsigned long) ((dbg_tx_used_min == DBG_MIN_U32_INIT) ? 0u : dbg_tx_used_min),
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
#else
    (void) sample_rate_hz;
    (void) task_frequency_hz;
    (void) streaming_out;
    (void) tx_used_words;
#endif
}
