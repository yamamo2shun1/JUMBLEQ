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

#include "main.h"

#ifndef AUDIO_DIAG_LOG
#define AUDIO_DIAG_LOG 0
#endif

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
                                             uint32_t service_cycles);
void audio_diagnostics_record_tx_events_dropped(uint32_t last_event,
                                                uint32_t dropped_events,
                                                int32_t used);
void audio_diagnostics_record_dma_error(uint32_t error_code,
                                        bool tx_streaming,
                                        int32_t used);

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
