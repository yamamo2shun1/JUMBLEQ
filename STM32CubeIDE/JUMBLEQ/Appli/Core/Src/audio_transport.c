/*
 * audio_transport.c
 *
 * Realtime audio transport between TinyUSB UAC2 endpoints, the TX/RX ring
 * buffers and SAI TX/RX over GPDMA.
 *
 * ISR callbacks only publish the latest DMA event and pending flags; the
 * Audio Task applies stream requests and performs all buffer copies.
 */

#include "audio_control.h"
#include "audio_diagnostics_internal.h"
#include "audio_transport_internal.h"
#include "timecode_synth.h"

#include "gpdma.h"
#include "linked_list.h"
#include "sai.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "SEGGER_RTT.h"

enum
{
    AUDIO_USB_FRAME_CHANNELS = 4u,
    AUDIO_RING_FRAME_WORDS   = 4u,
    AUDIO_USB_HS_MICROFRAMES_PER_SECOND = 8000u,
    AUDIO_USB_FRAME_BYTES = AUDIO_USB_FRAME_CHANNELS * sizeof(int32_t),
    AUDIO_USB_OUT_TARGET_MICROFRAMES = 4u,  // 0.5 ms (HS OUT転送 0.125 ms x 4)
    AUDIO_USB_IN_TARGET_INTERVALS    = 2u,  // 1.0 ms (EP IN interval 0.5 ms x 2)
    DMA_AUDIO_EVENT_NONE       = 0u,
    DMA_AUDIO_EVENT_HALF       = 1u,
    DMA_AUDIO_EVENT_COMPLETE   = 2u,
    AUDIO_STREAM_OUT_BIT       = (1u << 0),
    AUDIO_STREAM_IN_BIT        = (1u << 1),
};

// 最大サンプルレート時のFIFO目標byte数。実行時helperと同じ切り上げ計算で、
// 設定矛盾（目標 + 最大packetがFIFO容量を超える）をビルド時に検出する。
#define AUDIO_USB_OUT_TARGET_BYTES_MAX \
    (((((uint32_t) CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE * AUDIO_USB_OUT_TARGET_MICROFRAMES) + \
       (AUDIO_USB_HS_MICROFRAMES_PER_SECOND - 1u)) / \
      AUDIO_USB_HS_MICROFRAMES_PER_SECOND) * \
     (uint32_t) AUDIO_USB_FRAME_BYTES)
#define AUDIO_USB_IN_TARGET_BYTES_MAX \
    (((((uint32_t) CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE * CFG_TUD_AUDIO_FUNC_1_EP_IN_INTERVAL_UFRAMES) + \
       (AUDIO_USB_HS_MICROFRAMES_PER_SECOND - 1u)) / \
      AUDIO_USB_HS_MICROFRAMES_PER_SECOND) * \
     AUDIO_USB_IN_TARGET_INTERVALS * (uint32_t) AUDIO_USB_FRAME_BYTES)

#define SAI_ERROR_STATUS_MASK \
    (SAI_xSR_OVRUDR | SAI_xSR_WCKCFG | SAI_xSR_CNRDY | SAI_xSR_AFSDET | SAI_xSR_LFSDET)

// 有効化するSAIエラー割り込み。SRフラグのマスクと対応させ、HAL_SAI_ErrorCallback
// （＝復旧要求publish）へ到達させる。
#define SAI_ERROR_INTERRUPT_MASK \
    (SAI_IT_OVRUDR | SAI_IT_WCKCFG | SAI_IT_CNRDY | SAI_IT_AFSDET | SAI_IT_LFSDET)

extern DMA_QListTypeDef List_GPDMA1_Channel2;
extern DMA_QListTypeDef List_GPDMA1_Channel3;

typedef struct
{
    int32_t* data;
    uint32_t capacity_words;
    volatile uint32_t write_index;
    volatile uint32_t read_index;
} AudioRingBuffer_t;

_Static_assert((SAI_RNG_BUF_SIZE & (SAI_RNG_BUF_SIZE - 1U)) == 0U,
               "Audio ring buffer size must be a power of two");
_Static_assert((SAI_RNG_BUF_SIZE % AUDIO_RING_FRAME_WORDS) == 0U,
               "Audio ring buffer size must preserve frame alignment");
_Static_assert((uint32_t) DMA_AUDIO_EVENT_NONE == (uint32_t) AUDIO_DIAG_DMA_EVENT_NONE,
               "DMA event encoding must match the diagnostics API");
_Static_assert((uint32_t) DMA_AUDIO_EVENT_HALF == (uint32_t) AUDIO_DIAG_DMA_EVENT_HALF,
               "DMA event encoding must match the diagnostics API");
_Static_assert((uint32_t) DMA_AUDIO_EVENT_COMPLETE == (uint32_t) AUDIO_DIAG_DMA_EVENT_COMPLETE,
               "DMA event encoding must match the diagnostics API");
_Static_assert(AUDIO_USB_FRAME_BYTES == AUDIO_RING_FRAME_WORDS * sizeof(int32_t),
               "USB and ring buffer frame sizes must match");
// FIFO目標は最大packetを追加で格納できる余白を残し、uint16_tに収まること。
_Static_assert(AUDIO_USB_OUT_TARGET_BYTES_MAX + CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX <=
                   CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ,
               "USB OUT FIFO target must leave room for one maximum packet");
_Static_assert(AUDIO_USB_IN_TARGET_BYTES_MAX + CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX <=
                   CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ,
               "USB IN FIFO target must leave room for one maximum packet");

static __attribute__((section("noncacheable_buffer"), aligned(32)))
int32_t s_tx_ring_storage[SAI_RNG_BUF_SIZE] = {0};
static __attribute__((section("noncacheable_buffer"), aligned(32)))
int32_t s_rx_ring_storage[SAI_RNG_BUF_SIZE] = {0};

static AudioRingBuffer_t s_tx_ring = {
    .data           = s_tx_ring_storage,
    .capacity_words = SAI_RNG_BUF_SIZE,
};
static AudioRingBuffer_t s_rx_ring = {
    .data           = s_rx_ring_storage,
    .capacity_words = SAI_RNG_BUF_SIZE,
};

static inline int32_t audio_ring_used_words(const AudioRingBuffer_t* ring)
{
    return (int32_t) (ring->write_index - ring->read_index);
}

static inline uint32_t audio_ring_offset(const AudioRingBuffer_t* ring,
                                         uint32_t absolute_index)
{
    return absolute_index & (ring->capacity_words - 1U);
}

static inline void audio_ring_discard_all(AudioRingBuffer_t* ring)
{
    ring->read_index = ring->write_index;
}

static inline void audio_ring_reset_indices(AudioRingBuffer_t* ring,
                                            uint32_t prefill_words)
{
    ring->read_index  = 0U;
    ring->write_index = prefill_words;
}

static void audio_ring_clear_storage(AudioRingBuffer_t* ring)
{
    memset(ring->data, 0, ring->capacity_words * sizeof(ring->data[0]));
}

typedef struct
{
    // ISRはコールバックごとに増加させ、Taskは差分から滞留数を求める。
    // uint32_tのラップ後も符号なし減算で差分を維持できる。
    uint32_t produced_sequence;
    uint32_t consumed_sequence;
    // 最新コールバックが示す、現在DMAがアクセスしていないhalf。
    uint32_t latest_event;
    uint32_t latest_cycle;
    uint32_t dropped_events;
    // 復旧・レート変更で経路を再構築した世代。古い世代の遅延イベントを
    // 通常搬送へ混入させないために使用する。
    uint32_t latest_generation;
} DmaAudioEventState_t;

typedef struct
{
    uint32_t event;
    uint32_t cycle;
    uint32_t dropped_events;
    uint32_t generation;
} DmaAudioEventSnapshot_t;

static volatile DmaAudioEventState_t s_tx_dma_event = {0};
static volatile DmaAudioEventState_t s_rx_dma_event = {0};
static volatile uint32_t s_dma_event_generation = 0u;

static inline uint32_t dma_audio_event_publish_from_isr(volatile DmaAudioEventState_t* state,
                                                        uint32_t event)
{
    const bool pending = state->produced_sequence != state->consumed_sequence;
    const uint32_t overwritten_event = pending ? state->latest_event : DMA_AUDIO_EVENT_NONE;

    state->latest_event = event;
    state->latest_cycle = DWT->CYCCNT;
    state->latest_generation = s_dma_event_generation;
    __DMB();
    state->produced_sequence++;

    return overwritten_event;
}

static bool dma_audio_event_take_latest(volatile DmaAudioEventState_t* state,
                                        DmaAudioEventSnapshot_t* snapshot)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t produced_sequence = state->produced_sequence;
    const uint32_t consumed_sequence = state->consumed_sequence;
    const uint32_t pending_count = produced_sequence - consumed_sequence;
    if (pending_count == 0u)
    {
        __set_PRIMASK(primask);
        return false;
    }

    snapshot->event          = state->latest_event;
    snapshot->cycle          = state->latest_cycle;
    snapshot->dropped_events = pending_count - 1u;
    snapshot->generation     = state->latest_generation;

    state->consumed_sequence = produced_sequence;
    state->latest_event      = DMA_AUDIO_EVENT_NONE;
    state->latest_cycle      = 0u;
    state->dropped_events   += snapshot->dropped_events;

    __set_PRIMASK(primask);
    return true;
}

static inline void dma_audio_event_reset_locked(volatile DmaAudioEventState_t* state)
{
    state->consumed_sequence = state->produced_sequence;
    state->latest_event      = DMA_AUDIO_EVENT_NONE;
    state->latest_cycle      = 0u;
    state->dropped_events    = 0u;
}

static void dma_audio_event_reset(volatile DmaAudioEventState_t* state)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    dma_audio_event_reset_locked(state);

    __set_PRIMASK(primask);
}

static TaskHandle_t s_audio_task_handle = NULL;

void audio_transport_register_current_task(void)
{
    s_audio_task_handle = xTaskGetCurrentTaskHandle();
}

void audio_transport_notify_task(void)
{
    if (s_audio_task_handle == NULL)
    {
        return;
    }

    xTaskNotifyGive(s_audio_task_handle);
}

static inline void audio_transport_notify_from_isr(void)
{
    if (s_audio_task_handle == NULL)
    {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(s_audio_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ==============================
// DMA/SAIエラー復旧要求 (ISR publish / Audio Task take+ack)
// ==============================

typedef struct
{
    volatile uint32_t published_sequence;
    uint32_t acknowledged_sequence;
    uint32_t cause_mask;
    uint32_t dma_error_code;
    uint32_t sai_error_code;
    uint32_t sai_status_flags;
} RecoveryRequestState_t;

static volatile RecoveryRequestState_t s_recovery_request = {0};

// ISR context. 診断・要求の記録とTask通知だけを行い、停止・再初期化はしない。
static void recovery_request_publish_from_isr(uint32_t cause_bit,
                                              uint32_t dma_error_code,
                                              uint32_t sai_error_code,
                                              uint32_t sai_status_flags)
{
    s_recovery_request.cause_mask |= cause_bit;
    if (dma_error_code != 0u)
    {
        s_recovery_request.dma_error_code = dma_error_code;
    }
    if (sai_error_code != 0u)
    {
        s_recovery_request.sai_error_code = sai_error_code;
    }
    if (sai_status_flags != 0u)
    {
        s_recovery_request.sai_status_flags |= sai_status_flags;
    }
    __DMB();
    s_recovery_request.published_sequence++;
    audio_transport_notify_from_isr();
}

bool audio_transport_take_recovery_request(AudioRecoveryRequest_t* request)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t published = s_recovery_request.published_sequence;
    const bool pending = (published != s_recovery_request.acknowledged_sequence);
    if (pending)
    {
        request->sequence         = published;
        request->cause_mask       = s_recovery_request.cause_mask;
        request->dma_error_code   = s_recovery_request.dma_error_code;
        request->sai_error_code   = s_recovery_request.sai_error_code;
        request->sai_status_flags = s_recovery_request.sai_status_flags;

        // 今回のpayloadをin-flightとして切り離し、以後のエラーを新しいpayloadへ
        // 集約する。ack後に古い原因が残り、後発要求へ混ざることを防ぐ。
        s_recovery_request.cause_mask       = 0u;
        s_recovery_request.dma_error_code   = 0u;
        s_recovery_request.sai_error_code   = 0u;
        s_recovery_request.sai_status_flags = 0u;
    }

    __set_PRIMASK(primask);
    return pending;
}

void audio_transport_ack_recovery_request(uint32_t sequence)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if ((int32_t) (sequence - s_recovery_request.acknowledged_sequence) > 0)
    {
        s_recovery_request.acknowledged_sequence = sequence;
    }

    __set_PRIMASK(primask);
}

static volatile bool s_streaming_out = false;
static volatile bool s_streaming_in  = false;
static volatile uint32_t s_stream_requested_mask     = 0u;
static volatile uint32_t s_stream_request_sequence   = 0u;
static uint32_t s_stream_applied_request_sequence    = 0u;

static volatile bool usb_tx_pending = false;  // USB TX送信要求フラグ (ISR→Task通知用)
static volatile bool usb_rx_pending = false;  // USB RX受信通知フラグ (ISR→Task通知用)

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t usb_capture_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t usb_playback_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4] = {0};

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t stereo_out_buf[SAI_TX_BUF_SIZE] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t stereo_in_buf[SAI_RX_BUF_SIZE]  = {0};

// Speaker data size received in the last frame
uint16_t spk_data_size;

static void fill_tx_half(uint32_t index0);
static void fill_rx_half(uint32_t index0, bool streaming);
static uint32_t audio_frames_per_usb_in_interval(uint32_t sample_rate_hz);
static bool audio_usb_in_source_ready(uint32_t sample_rate_hz);
static void copybuf_ring2usb_and_send(uint32_t sample_rate_hz);
static void audio_transport_apply_usb_in_fifo_target(uint32_t sample_rate_hz);

static inline int32_t audio_tx_used_words(void)
{
    return audio_ring_used_words(&s_tx_ring);
}

static void audio_transport_reset_tx_diagnostics(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    audio_diagnostics_reset_tx_locked();
    dma_audio_event_reset_locked(&s_tx_dma_event);

    __set_PRIMASK(primask);
}

// ==============================
// DMA / SAI callbacks
// ==============================

static void dma_sai2_tx_half(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    const uint32_t overwritten_event =
        dma_audio_event_publish_from_isr(&s_tx_dma_event, DMA_AUDIO_EVENT_HALF);
    const bool streaming = s_streaming_out;
    audio_diagnostics_record_tx_dma_callback(DMA_AUDIO_EVENT_HALF, overwritten_event, streaming,
                                             streaming ? audio_tx_used_words() : 0);
    audio_transport_notify_from_isr();
}
static void dma_sai2_tx_cplt(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    const uint32_t overwritten_event =
        dma_audio_event_publish_from_isr(&s_tx_dma_event, DMA_AUDIO_EVENT_COMPLETE);
    const bool streaming = s_streaming_out;
    audio_diagnostics_record_tx_dma_callback(DMA_AUDIO_EVENT_COMPLETE, overwritten_event, streaming,
                                             streaming ? audio_tx_used_words() : 0);
    audio_transport_notify_from_isr();
}

static void dma_sai1_rx_half(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    const uint32_t overwritten_event =
        dma_audio_event_publish_from_isr(&s_rx_dma_event, DMA_AUDIO_EVENT_HALF);
    audio_diagnostics_record_rx_dma_callback(DMA_AUDIO_EVENT_HALF, overwritten_event);
    audio_transport_notify_from_isr();
}
static void dma_sai1_rx_cplt(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    const uint32_t overwritten_event =
        dma_audio_event_publish_from_isr(&s_rx_dma_event, DMA_AUDIO_EVENT_COMPLETE);
    audio_diagnostics_record_rx_dma_callback(DMA_AUDIO_EVENT_COMPLETE, overwritten_event);
    audio_transport_notify_from_isr();
}

static void dma_sai_error(DMA_HandleTypeDef* hdma)
{
    if (hdma == &handle_GPDMA1_Channel2)
    {
        const bool streaming = s_streaming_out;
        audio_diagnostics_record_dma_error(hdma->ErrorCode, true, streaming,
                                           streaming ? audio_tx_used_words() : 0);
        recovery_request_publish_from_isr(AUDIO_RECOVERY_CAUSE_TX_DMA, hdma->ErrorCode, 0u, 0u);
    }
    else if (hdma == &handle_GPDMA1_Channel3)
    {
        audio_diagnostics_record_dma_error(hdma->ErrorCode, false, false, 0);
        recovery_request_publish_from_isr(AUDIO_RECOVERY_CAUSE_RX_DMA, hdma->ErrorCode, 0u, 0u);
    }
    else
    {
        // 未知のハンドルは診断へ残すが、誤った経路情報で復旧しない。
        audio_diagnostics_record_unknown_dma_error(hdma->ErrorCode);
    }
}

// SAIエラー割り込みのISR処理。フラグの保存・クリア、対象割り込みのマスク、診断記録、
// 復旧要求のpublishのみを行い、HAL標準ハンドラの停止待ち（SAI_DMAAbort→SAI_Disable）へ
// 渡さない。停止・再初期化はAudio Taskの復旧処理へ集約する。
// 処理した場合trueを返し、呼出側はHAL_SAI_IRQHandlerをスキップする。
bool audio_transport_sai_error_isr(SAI_HandleTypeDef* hsai)
{
    const uint32_t pending = hsai->Instance->SR & SAI_ERROR_STATUS_MASK;
    if (pending == 0u)
    {
        return false;
    }

    const bool tx_handle = (hsai == &hsai_BlockA2);
    const bool rx_handle = (hsai == &hsai_BlockA1);
    if (!tx_handle && !rx_handle)
    {
        return false;
    }

    // 保存したフラグをクリアし、再入を防ぐため対象割り込みをマスクする。
    hsai->Instance->CLRFR = pending;
    hsai->Instance->IMR &= ~SAI_ERROR_INTERRUPT_MASK;

    uint32_t error = 0u;
    if ((pending & SAI_xSR_OVRUDR) != 0u)
    {
        error |= (hsai->State == HAL_SAI_STATE_BUSY_RX) ? HAL_SAI_ERROR_OVR : HAL_SAI_ERROR_UDR;
    }
    if ((pending & SAI_xSR_WCKCFG) != 0u)
    {
        error |= HAL_SAI_ERROR_WCKCFG;
    }
    if ((pending & SAI_xSR_CNRDY) != 0u)
    {
        error |= HAL_SAI_ERROR_CNREADY;
    }
    if ((pending & SAI_xSR_AFSDET) != 0u)
    {
        error |= HAL_SAI_ERROR_AFSDET;
    }
    if ((pending & SAI_xSR_LFSDET) != 0u)
    {
        error |= HAL_SAI_ERROR_LFSDET;
    }

    if (tx_handle)
    {
        const bool streaming = s_streaming_out;
        audio_diagnostics_record_sai_tx_error(error, pending, streaming,
                                              streaming ? audio_tx_used_words() : 0);
        recovery_request_publish_from_isr(AUDIO_RECOVERY_CAUSE_TX_SAI, 0u, error, pending);
    }
    else
    {
        audio_diagnostics_record_sai_rx_error(error, pending);
        recovery_request_publish_from_isr(AUDIO_RECOVERY_CAUSE_RX_SAI, 0u, error, pending);
    }

    return true;
}

// HAL標準ハンドラ経由のフォールバック。通常はaudio_transport_sai_error_isr()が
// 先に処理してHAL_SAI_IRQHandlerへ渡さないため、ここには到達しない。
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef* hsai)
{
    const uint32_t sr = hsai->Instance->SR;
    if (hsai == &hsai_BlockA2)
    {
        const bool streaming = s_streaming_out;
        const uint32_t error = HAL_SAI_GetError(hsai);
        audio_diagnostics_record_sai_tx_error(error,
                                              sr & SAI_ERROR_STATUS_MASK,
                                              streaming,
                                              streaming ? audio_tx_used_words() : 0);
        recovery_request_publish_from_isr(AUDIO_RECOVERY_CAUSE_TX_SAI, 0u, error,
                                          sr & SAI_ERROR_STATUS_MASK);
    }
    else if (hsai == &hsai_BlockA1)
    {
        const uint32_t error = HAL_SAI_GetError(hsai);
        audio_diagnostics_record_sai_rx_error(error, sr & SAI_ERROR_STATUS_MASK);
        recovery_request_publish_from_isr(AUDIO_RECOVERY_CAUSE_RX_SAI, 0u, error,
                                          sr & SAI_ERROR_STATUS_MASK);
    }
}

// ==============================
// SAI / GPDMA lifecycle
// ==============================

static bool audio_transport_start_tx_path(void)
{
    if (MX_List_GPDMA1_Channel2_Config() != HAL_OK)
    {
        return false;
    }
    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel2, &List_GPDMA1_Channel2) != HAL_OK)
    {
        return false;
    }

    handle_GPDMA1_Channel2.XferHalfCpltCallback = dma_sai2_tx_half;
    handle_GPDMA1_Channel2.XferCpltCallback     = dma_sai2_tx_cplt;
    handle_GPDMA1_Channel2.XferErrorCallback    = dma_sai_error;
    if (HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel2) != HAL_OK)
    {
        return false;
    }

    // 手動DMA開始方式に合わせてHAL状態をBUSY_TXへ設定する。HAL IRQ handlerの
    // エラー分類（OVRUDR時のUDR/OVR判定）がこのStateを参照する。
    hsai_BlockA2.State = HAL_SAI_STATE_BUSY_TX;
    // SAIエラー割り込みを有効化してHAL_SAI_ErrorCallback（復旧要求）へ到達させる。
    __HAL_SAI_ENABLE_IT(&hsai_BlockA2, SAI_ERROR_INTERRUPT_MASK);

    hsai_BlockA2.Instance->CR1 |= SAI_xCR1_DMAEN;
    __HAL_SAI_ENABLE(&hsai_BlockA2);
    return true;
}

static bool audio_transport_start_rx_path(void)
{
    if (MX_List_GPDMA1_Channel3_Config() != HAL_OK)
    {
        return false;
    }
    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel3, &List_GPDMA1_Channel3) != HAL_OK)
    {
        return false;
    }

    handle_GPDMA1_Channel3.XferHalfCpltCallback = dma_sai1_rx_half;
    handle_GPDMA1_Channel3.XferCpltCallback     = dma_sai1_rx_cplt;
    handle_GPDMA1_Channel3.XferErrorCallback    = dma_sai_error;
    if (HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel3) != HAL_OK)
    {
        return false;
    }

    // 手動DMA開始方式に合わせてHAL状態をBUSY_RXへ設定する。HAL IRQ handlerの
    // エラー分類（OVRUDR時のUDR/OVR判定）がこのStateを参照する。
    hsai_BlockA1.State = HAL_SAI_STATE_BUSY_RX;
    // SAIエラー割り込みを有効化してHAL_SAI_ErrorCallback（復旧要求）へ到達させる。
    __HAL_SAI_ENABLE_IT(&hsai_BlockA1, SAI_ERROR_INTERRUPT_MASK);

    hsai_BlockA1.Instance->CR1 |= SAI_xCR1_DMAEN;
    __HAL_SAI_ENABLE(&hsai_BlockA1);
    return true;
}

// SAI停止。HAL_SAI_Abortで停止完了確認・DMA abort・IMR/フラグ初期化・FIFO flushを
// 行い、StateをREADYへ戻す（次のHAL_SAI_Initが生成MspInitをスキップできる状態）。
// deinit_sai=trueはさらにSAIをDeInitする（レート変更用）。復旧はfalseでMSPを維持する。
// 戻り値は停止完了確認の成否。falseでもDMAは明示的にabortする。
static bool audio_transport_stop_sai_paths(bool deinit_sai)
{
    bool stopped = true;

    if (HAL_SAI_Abort(&hsai_BlockA2) != HAL_OK)
    {
        stopped = false;
    }
    if (HAL_SAI_Abort(&hsai_BlockA1) != HAL_OK)
    {
        stopped = false;
    }

    (void) HAL_DMA_Abort(&handle_GPDMA1_Channel2);
    (void) HAL_DMA_Abort(&handle_GPDMA1_Channel3);
    __DSB();

    if (deinit_sai)
    {
        (void) HAL_SAI_DeInit(&hsai_BlockA2);
        (void) HAL_SAI_DeInit(&hsai_BlockA1);
    }

    return stopped;
}

static bool audio_transport_init_dma_channel(DMA_HandleTypeDef* hdma,
                                             DMA_Channel_TypeDef* instance)
{
    hdma->Instance                         = instance;
    hdma->InitLinkedList.Priority          = DMA_LOW_PRIORITY_HIGH_WEIGHT;
    hdma->InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
    hdma->InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
    hdma->InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
    hdma->InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;

    return (HAL_DMAEx_List_Init(hdma) == HAL_OK) &&
           (HAL_DMA_ConfigChannelAttributes(hdma, DMA_CHANNEL_NPRIV) == HAL_OK);
}

static bool audio_transport_reinit_dma_channels(void)
{
    (void) HAL_DMA_DeInit(&handle_GPDMA1_Channel2);
    (void) HAL_DMA_DeInit(&handle_GPDMA1_Channel3);

    return audio_transport_init_dma_channel(&handle_GPDMA1_Channel2, GPDMA1_Channel2) &&
           audio_transport_init_dma_channel(&handle_GPDMA1_Channel3, GPDMA1_Channel3);
}

// ==============================
// Reset / start / stream state
// ==============================

void audio_transport_reset_buffers(void)
{
    for (uint16_t i = 0; i < CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4; i++)
    {
        usb_capture_buf[i] = 0;
    }

    for (uint16_t i = 0; i < CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4; i++)
    {
        usb_playback_buf[i] = 0;
    }

    audio_ring_clear_storage(&s_tx_ring);
    audio_ring_clear_storage(&s_rx_ring);

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

void audio_transport_start(void)
{
    // ========================================
    // リングバッファをプリフィル（無音で初期化）
    // SAI DMAが開始直後にHalf割り込みを発生させた時、
    // リングバッファにデータがないとアンダーランになるため、
    // 無音データを事前に投入しておく
    // 96kHzではデータレートが高いため、十分な量をプリフィルする
    // ========================================
    uint32_t prefill_size = SAI_TX_BUF_SIZE;
    memset(s_tx_ring.data, 0, prefill_size * sizeof(s_tx_ring.data[0]));
    audio_ring_reset_indices(&s_tx_ring, prefill_size);
    audio_transport_reset_tx_diagnostics();
    dma_audio_event_reset(&s_rx_dma_event);

#if AUDIO_DIAG_LOG
    audio_diagnostics_reset_session();
#endif

    // SAI2 -> Slave Transmit
    // USB -> STM32 -(SAI)-> ADAU1466
    if (!audio_transport_start_tx_path())
    {
        Error_Handler();
    }

    osDelay(500);
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    // SAI1 -> Slave Receize
    // ADAU1466 -(SAI)-> STM32 -> USB
    if (!audio_transport_start_rx_path())
    {
        Error_Handler();
    }
}

// 復旧・レート変更共通。SAI/GPDMA停止 → DMAイベント/リングindexリセット →
// DMA/リング/USBバッファ消去。timecode設定には触れない。Audio Task context only。
// deinit_sai=trueはSAIをDeInitする（レート変更用）。復旧はfalseにしてSAIのMSP資源と
// 設定を維持し、HAL_SAI_MspInit（Error_Handlerを含む）を再実行させない。
// g_audio_tx_diagnosticsは原因調査のため保持する（明示リセットは呼出側で行う）。
// 戻り値はSAI停止完了確認の成否。バッファ消去は常に行う。
bool audio_transport_stop_and_clear_paths(bool deinit_sai)
{
    const bool stopped = audio_transport_stop_sai_paths(deinit_sai);

    audio_ring_reset_indices(&s_tx_ring, 0U);
    audio_ring_reset_indices(&s_rx_ring, 0U);

    // 停止中に遅延して届くイベントを通常搬送へ混入させないよう世代を進める。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    dma_audio_event_reset_locked(&s_tx_dma_event);
    dma_audio_event_reset_locked(&s_rx_dma_event);
    s_dma_event_generation++;
    __set_PRIMASK(primask);

#if AUDIO_DIAG_LOG
    audio_diagnostics_reset_session();
#endif

    /* Clear all audio buffers to avoid noise from stale data */
    audio_ring_clear_storage(&s_tx_ring);
    audio_ring_clear_storage(&s_rx_ring);
    memset(stereo_out_buf, 0, sizeof(stereo_out_buf));
    memset(stereo_in_buf, 0, sizeof(stereo_in_buf));
    memset(usb_playback_buf, 0, sizeof(usb_playback_buf));
    memset(usb_capture_buf, 0, sizeof(usb_capture_buf));
    __DSB();

    return stopped;
}

void audio_transport_reset_for_sample_rate(uint32_t sample_rate_hz)
{
    (void) audio_transport_stop_and_clear_paths(true);

    // レート変更はstream開始と同様、TX診断を明示的にリセットする。
    audio_transport_reset_tx_diagnostics();

    timecode_synth_reset_for_sample_rate(sample_rate_hz);

    // alt settingが維持されたままレートだけ変わる場合に備え、新レートのIN FIFO目標も適用する。
    audio_transport_apply_usb_in_fifo_target(sample_rate_hz);

    __DSB();
}

// DMA channel再構築とTXリングprefill・TX開始。失敗時は両経路を停止してfalse。
// init_sai=trueはレート変更用で、CubeMX生成のMX_SAIx_Init()でSAIを再構築する。
// init_sai=falseは復旧用で、READY状態からのHAL_SAI_Init（MspInitをスキップ）により
// MSP資源を維持したままSAI設定とErrorCodeを再初期化して再始動する。
bool audio_transport_rebuild_and_start_tx(bool init_sai)
{
    /* Re-init DMA channels (linked-list mode) */
    if (!audio_transport_reinit_dma_channels())
    {
        (void) audio_transport_stop_sai_paths(init_sai);
        return false;
    }

    if (init_sai)
    {
        /* Reconfigure peripherals (SAI). 失敗時は既存どおりfatal。 */
        MX_SAI1_Init();
        MX_SAI2_Init();
    }
    else
    {
        /* 復旧: 停止時にStateがREADYへ戻っているためMspInitは再実行されない。 */
        if ((HAL_SAI_Init(&hsai_BlockA1) != HAL_OK) || (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK))
        {
            (void) audio_transport_stop_sai_paths(false);
            return false;
        }

        // HAL_SAI_Initで設定が入り直すためDMAリンクを再実行する。
        __HAL_LINKDMA(&hsai_BlockA1, hdmarx, handle_GPDMA1_Channel3);
        __HAL_LINKDMA(&hsai_BlockA2, hdmatx, handle_GPDMA1_Channel2);
    }

    /* Prefill TX ring buffer with silence (already zeroed above) */
    /* Set write index ahead to provide initial data for DMA */
    /* 96kHz needs larger prefill due to higher data rate */
    audio_ring_reset_indices(&s_tx_ring, SAI_TX_BUF_SIZE);

    /* Configure and link DMA for SAI2 TX */
    if (!audio_transport_start_tx_path())
    {
        (void) audio_transport_stop_sai_paths(init_sai);
        return false;
    }

    return true;
}

// TX同期待ち後のRX開始。失敗時は両経路を停止（MSP維持）してfalse。
bool audio_transport_start_rx_after_tx_sync(void)
{
    /* Configure and link DMA for SAI1 RX */
    if (!audio_transport_start_rx_path())
    {
        (void) audio_transport_stop_sai_paths(false);
        return false;
    }

    // 復旧後もalt settingが維持される場合があるため、IN FIFO目標を再適用する。
    audio_transport_apply_usb_in_fifo_target(get_current_sample_rate_hz());
    return true;
}

void audio_transport_restart_after_rate_change(void)
{
    if (!audio_transport_rebuild_and_start_tx(true))
    {
        Error_Handler();
    }

    /* Wait for SAI TX to synchronize with external clock before starting RX */
    osDelay(10);

    if (!audio_transport_start_rx_after_tx_sync())
    {
        Error_Handler();
    }
}

void audio_transport_request_stream(AudioTransportStream_t stream, bool enabled)
{
    const uint32_t stream_bit = (stream == AUDIO_TRANSPORT_STREAM_OUT) ?
                                    AUDIO_STREAM_OUT_BIT :
                                    AUDIO_STREAM_IN_BIT;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

#if AUDIO_DIAG_LOG
    const uint32_t previous_mask = s_stream_requested_mask;
#endif

    if (enabled)
    {
        s_stream_requested_mask |= stream_bit;
    }
    else
    {
        s_stream_requested_mask &= ~stream_bit;
    }
    __DMB();
    s_stream_request_sequence++;
#if AUDIO_DIAG_LOG
    const uint32_t requested_mask = s_stream_requested_mask;
    const uint32_t request_sequence = s_stream_request_sequence;
#endif

    __set_PRIMASK(primask);

#if AUDIO_DIAG_LOG
    SEGGER_RTT_printf(0,
                      "[AUD][STREAM-REQ] tick=%lu stream=%s enabled=%u mask=0x%02lx->0x%02lx sequence=%lu\r\n",
                      (unsigned long) HAL_GetTick(),
                      (stream == AUDIO_TRANSPORT_STREAM_OUT) ? "OUT" : "IN",
                      enabled ? 1u : 0u,
                      (unsigned long) previous_mask,
                      (unsigned long) requested_mask,
                      (unsigned long) request_sequence);
#endif
    audio_transport_notify_task();
}

static bool audio_stream_take_requested_state(uint32_t* requested_mask)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t request_sequence = s_stream_request_sequence;
    if (request_sequence == s_stream_applied_request_sequence)
    {
        __set_PRIMASK(primask);
        return false;
    }

    *requested_mask = s_stream_requested_mask;
    s_stream_applied_request_sequence = request_sequence;

    __set_PRIMASK(primask);
    return true;
}

static void audio_stream_apply_out_state(bool enabled)
{
    if (enabled == s_streaming_out)
    {
        return;
    }

    if (enabled)
    {
        audio_transport_reset_tx_diagnostics();

        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        spk_data_size   = 0u;
        usb_rx_pending  = false;
        s_streaming_out = true;
        __set_PRIMASK(primask);
    }
    else
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_streaming_out      = false;
        spk_data_size        = 0u;
        usb_rx_pending       = false;
        audio_ring_reset_indices(&s_tx_ring, 0U);
        dma_audio_event_reset_locked(&s_tx_dma_event);
        __set_PRIMASK(primask);
    }

#if AUDIO_DIAG_LOG
    audio_diagnostics_reset_usb_out_gap();
#endif
}

static void audio_stream_apply_in_state(bool enabled)
{
    if (enabled != s_streaming_in)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        if (enabled)
        {
            usb_tx_pending = true;
            dma_audio_event_reset_locked(&s_rx_dma_event);
            s_streaming_in = true;
        }
        else
        {
            s_streaming_in       = false;
            usb_tx_pending       = false;
            audio_ring_reset_indices(&s_rx_ring, 0U);
            dma_audio_event_reset_locked(&s_rx_dma_event);
        }

        __set_PRIMASK(primask);

#if AUDIO_DIAG_LOG
        if (enabled)
        {
            audio_diagnostics_record_usb_in_notify();
        }
#endif
    }

    if (enabled)
    {
        // TinyUSBはSET_INTERFACEのたびにIN FIFO thresholdをFIFO半分へ戻すため、
        // 有効要求を適用するたびに現在レートの目標で上書きする。
        audio_transport_apply_usb_in_fifo_target(get_current_sample_rate_hz());
    }
}

bool audio_transport_apply_requested_stream_state(void)
{
    uint32_t requested_mask;
    if (!audio_stream_take_requested_state(&requested_mask))
    {
        return false;
    }

#if AUDIO_DIAG_LOG
    const bool previous_out = s_streaming_out;
    const bool previous_in  = s_streaming_in;
#endif
    audio_stream_apply_out_state((requested_mask & AUDIO_STREAM_OUT_BIT) != 0u);
    audio_stream_apply_in_state((requested_mask & AUDIO_STREAM_IN_BIT) != 0u);

#if AUDIO_DIAG_LOG
    SEGGER_RTT_printf(0,
                      "[AUD][STREAM-APPLY] tick=%lu mask=0x%02lx out=%u->%u in=%u->%u\r\n",
                      (unsigned long) HAL_GetTick(),
                      (unsigned long) requested_mask,
                      previous_out ? 1u : 0u,
                      s_streaming_out ? 1u : 0u,
                      previous_in ? 1u : 0u,
                      s_streaming_in ? 1u : 0u);
#endif
    return true;
}

void audio_transport_clear_pending_events(void)
{
    spk_data_size  = 0;
    usb_tx_pending = false;
    usb_rx_pending = false;
}

// ==============================
// USB(OUT) path -> Ring -> SAI(TX)
// ==============================

static void copybuf_usb2ring(void)
{
    int32_t used = audio_ring_used_words(&s_tx_ring);

    if (used < 0)
    {
        audio_ring_discard_all(&s_tx_ring);
        used = 0;
    }
    int32_t free = (int32_t) (s_tx_ring.capacity_words - 1U) - used;
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
        s_tx_ring.data[audio_ring_offset(&s_tx_ring, s_tx_ring.write_index)] = usb_playback_buf[i];
        s_tx_ring.write_index++;
    }
}

static void fill_tx_half(uint32_t index0)
{
    const uint32_t n           = (SAI_TX_BUF_SIZE / 2);
    const uint32_t frame_words = 4;  // 4ch x 32bit = 1 frame
    uint32_t consume_words     = n;
    uint32_t source_skip_words = 0;
    uint32_t diagnostic_event_flags = 0u;
    const bool streaming       = s_streaming_out;

    // index0の範囲チェック
    if (index0 > (SAI_TX_BUF_SIZE - n))
    {
        // 不正な値は無音で埋める
        return;
    }

    int32_t used = audio_ring_used_words(&s_tx_ring);
    audio_diagnostics_record_tx_level(streaming, used);
#if AUDIO_DIAG_LOG
    audio_diagnostics_record_tx_interval_level(used);
#endif
    if (used < 0)
    {
        // 同期ズレは破棄して合わせ直す
        audio_ring_discard_all(&s_tx_ring);
        used = 0;
    }

    // データ不足時は可能な分だけ再生し、残りは末尾フレーム保持で埋める
    // いきなり無音にせず、クリック感を抑える
    if (used < (int32_t) n)
    {
        diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_UNDERRUN;
        if (used <= 0)
        {
            memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
            audio_diagnostics_record_tx_event(streaming, diagnostic_event_flags, used);
            timecode_synth_render_output(stereo_out_buf + index0,
                                         AUDIO_RING_FRAME_WORDS,
                                         (SAI_TX_BUF_SIZE / 2u) / AUDIO_RING_FRAME_WORDS);
            return;
        }
        consume_words = ((uint32_t) used / frame_words) * frame_words;
        if (consume_words == 0)
        {
            memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
            audio_diagnostics_record_tx_event(streaming, diagnostic_event_flags, used);
            timecode_synth_render_output(stereo_out_buf + index0,
                                         AUDIO_RING_FRAME_WORDS,
                                         (SAI_TX_BUF_SIZE / 2u) / AUDIO_RING_FRAME_WORDS);
            return;
        }
    }

    // usedが大きすぎる場合も異常（オーバーフロー等）
    if (used > (int32_t) s_tx_ring.capacity_words)
    {
        // リセットして無音で埋める
        audio_ring_discard_all(&s_tx_ring);
        memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
        timecode_synth_render_output(stereo_out_buf + index0,
                                     AUDIO_RING_FRAME_WORDS,
                                     (SAI_TX_BUF_SIZE / 2u) / AUDIO_RING_FRAME_WORDS);
        return;
    }

    // 長時間再生時のUSB/SAIクロック差を吸収するため、リング水位に応じて
    // 1 frameだけ消費量を増減する。usedはDMA half消費前の水位なので、
    // 消費後の目標水位に今回の通常消費量を加えた値を判定基準にする。
    const int32_t target_before_consume =
        (int32_t) SAI_TX_TARGET_LEVEL_WORDS + (int32_t) n;

    if (used > target_before_consume && used >= (int32_t) (n + frame_words))
    {
        // バッファ過多: 最古の1 frameを捨て、DMA halfには通常量だけ書き込む
        consume_words     = n + frame_words;
        source_skip_words = frame_words;
        diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_DRIFT_UP;
    }
    else if (used >= (int32_t) n && used < target_before_consume && n > frame_words)
    {
        // バッファ不足傾向: 1 frame 少なく消費して追従
        consume_words = n - frame_words;
        diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_DRIFT_DOWN;
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
        audio_diagnostics_record_tx_event(streaming, diagnostic_event_flags, used);
        timecode_synth_render_output(stereo_out_buf + index0,
                                     AUDIO_RING_FRAME_WORDS,
                                     (SAI_TX_BUF_SIZE / 2u) / AUDIO_RING_FRAME_WORDS);
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

    const uint32_t index1 =
        audio_ring_offset(&s_tx_ring, s_tx_ring.read_index + source_skip_words);
    uint32_t first = s_tx_ring.capacity_words - index1;
    if (first > copy_words)
        first = copy_words;

    memcpy(stereo_out_buf + index0, s_tx_ring.data + index1, first * sizeof(int32_t));
    if (first < copy_words)
        memcpy(stereo_out_buf + index0 + first,
               s_tx_ring.data,
               (copy_words - first) * sizeof(int32_t));

    if (copy_words < n)
    {
        diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_PARTIAL_FILL;
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

    s_tx_ring.read_index += consume_words;
    audio_diagnostics_record_tx_level(streaming, used - (int32_t) consume_words);
    if (diagnostic_event_flags != 0u)
    {
        audio_diagnostics_record_tx_event(streaming, diagnostic_event_flags, used);
    }
    timecode_synth_render_output(stereo_out_buf + index0,
                                 AUDIO_RING_FRAME_WORDS,
                                 (SAI_TX_BUF_SIZE / 2u) / AUDIO_RING_FRAME_WORDS);
}

// DMA half期間（4ch frame単位）をSystemCoreClockサイクルへ換算する。
// 96kHzで約0.333ms、48kHzで約0.667ms。sample_rate_hzが不正なら期限なし。
static uint32_t audio_dma_half_deadline_cycles(uint32_t half_words, uint32_t sample_rate_hz)
{
    if ((sample_rate_hz == 0u) || (SystemCoreClock == 0u))
    {
        return UINT32_MAX;
    }

    const uint32_t half_frames = half_words / AUDIO_RING_FRAME_WORDS;
    return (uint32_t) (((uint64_t) half_frames * SystemCoreClock) / sample_rate_hz);
}

static void copybuf_ring2sai(uint32_t sample_rate_hz)
{
    DmaAudioEventSnapshot_t event;
    if (!dma_audio_event_take_latest(&s_tx_dma_event, &event))
    {
        return;
    }

    if (event.generation != s_dma_event_generation)
    {
        // 復旧・レート変更前の遅延イベントは通常搬送へ混入させない。
        return;
    }

    if (s_streaming_out)
    {
        const uint32_t service_cycles = DWT->CYCCNT - event.cycle;
        const uint32_t deadline_cycles =
            audio_dma_half_deadline_cycles(SAI_TX_BUF_SIZE / 2u, sample_rate_hz);
        audio_diagnostics_record_tx_dma_service(event.event, service_cycles,
                                                service_cycles > deadline_cycles);

        if (event.dropped_events != 0u)
        {
            audio_diagnostics_record_tx_events_dropped(event.event,
                                                       event.dropped_events,
                                                       audio_tx_used_words());
        }
    }

    // 遅延時は古い要求を処理しない。最新コールバックが示す現在安全なhalfだけを更新する。
    if (event.event == DMA_AUDIO_EVENT_HALF)
    {
        fill_tx_half(0);
    }
    else if (event.event == DMA_AUDIO_EVENT_COMPLETE)
    {
        fill_tx_half(SAI_TX_BUF_SIZE / 2);
    }
}

// ==============================
// SAI(RX) -> Ring -> USB(IN) path
// ==============================
static void fill_rx_half(uint32_t index0, bool streaming)
{
    const uint32_t n = (SAI_RX_BUF_SIZE / 2);  // 半分のword数

    // index0の範囲チェック
    if (index0 >= SAI_RX_BUF_SIZE)
    {
        return;
    }

    timecode_synth_process_input(stereo_in_buf + index0,
                                 AUDIO_RING_FRAME_WORDS,
                                 (SAI_RX_BUF_SIZE / 2u) / AUDIO_RING_FRAME_WORDS);

    int32_t used = audio_ring_used_words(&s_rx_ring);
    if (used < 0)
    {
        audio_ring_discard_all(&s_rx_ring);
        audio_diagnostics_record_rx_ring_discard(0u, true);
        used = 0;
    }

    // usedが大きすぎる場合も異常（オーバーフロー等）
    if (used > (int32_t) s_rx_ring.capacity_words)
    {
        audio_ring_discard_all(&s_rx_ring);
        audio_diagnostics_record_rx_ring_discard(0u, true);
        used = 0;
    }

    int32_t free = (int32_t) (s_rx_ring.capacity_words - 1U) - used;
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
        if (drop_words > 0)
        {
            s_rx_ring.read_index += (uint32_t) drop_words;
            if (streaming)
            {
                // USB IN停止中は排出されないため、正常動作としての破棄は記録しない。
                audio_diagnostics_record_rx_ring_discard((uint32_t) drop_words, false);
            }
        }
    }

    uint32_t w     = audio_ring_offset(&s_rx_ring, s_rx_ring.write_index);
    uint32_t first = s_rx_ring.capacity_words - w;
    if (first > n)
        first = n;

    memcpy(s_rx_ring.data + w, stereo_in_buf + index0, first * sizeof(int32_t));
    if (first < n)
        memcpy(s_rx_ring.data, stereo_in_buf + index0 + first, (n - first) * sizeof(int32_t));

    s_rx_ring.write_index += n;
}

static void copybuf_sai2ring(uint32_t sample_rate_hz)
{
    DmaAudioEventSnapshot_t event;
    if (!dma_audio_event_take_latest(&s_rx_dma_event, &event))
    {
        return;
    }

    if (event.generation != s_dma_event_generation)
    {
        // 復旧・レート変更前の遅延イベントは通常搬送へ混入させない。
        return;
    }

    if (s_streaming_in)
    {
        const uint32_t service_cycles = DWT->CYCCNT - event.cycle;
        const uint32_t deadline_cycles =
            audio_dma_half_deadline_cycles(SAI_RX_BUF_SIZE / 2u, sample_rate_hz);
        audio_diagnostics_record_rx_dma_service(event.event, service_cycles,
                                                service_cycles > deadline_cycles);

        if (event.dropped_events != 0u)
        {
            audio_diagnostics_record_rx_events_dropped(event.event, event.dropped_events);
        }
    }

    // TXと同様に、最新コールバックが示す現在安全なhalfだけを取り込む。
    if (event.event == DMA_AUDIO_EVENT_HALF)
    {
        fill_rx_half(0, s_streaming_in);
    }
    else if (event.event == DMA_AUDIO_EVENT_COMPLETE)
    {
        fill_rx_half(SAI_RX_BUF_SIZE / 2, s_streaming_in);
    }
}

// USB INエンドポイントの1転送間隔あたりのフレーム数
static uint32_t audio_frames_per_usb_in_interval(uint32_t sample_rate_hz)
{
    // 現在対応している48/96kHzはいずれも整数フレームになる。
    return (uint32_t) (((uint64_t) sample_rate_hz * CFG_TUD_AUDIO_FUNC_1_EP_IN_INTERVAL_UFRAMES) /
                       AUDIO_USB_HS_MICROFRAMES_PER_SECOND);
}

// 目標byte数を、1 frame以上・FIFO容量内で最大packet分の余白を残す上限・
// frame境界・uint16_t範囲へクランプする。
static uint16_t audio_usb_fifo_target_bytes_clamp(uint64_t target_bytes,
                                                  uint32_t fifo_capacity_bytes,
                                                  uint32_t max_packet_bytes)
{
    uint32_t max_target_bytes = (fifo_capacity_bytes > max_packet_bytes) ?
                                    (fifo_capacity_bytes - max_packet_bytes) :
                                    0u;
    if (max_target_bytes > UINT16_MAX)
    {
        max_target_bytes = UINT16_MAX;
    }
    max_target_bytes = (max_target_bytes / AUDIO_USB_FRAME_BYTES) * AUDIO_USB_FRAME_BYTES;

    if (target_bytes > max_target_bytes)
    {
        target_bytes = max_target_bytes;
    }
    if (target_bytes < AUDIO_USB_FRAME_BYTES)
    {
        target_bytes = AUDIO_USB_FRAME_BYTES;
    }

    return (uint16_t) target_bytes;
}

// USB OUT feedbackの目標FIFO水位。High-Speed microframe 4回分 = 0.5 ms相当。
uint16_t audio_transport_usb_out_fifo_target_bytes(uint32_t sample_rate_hz)
{
    const uint64_t target_frames =
        (((uint64_t) sample_rate_hz * AUDIO_USB_OUT_TARGET_MICROFRAMES) +
         (AUDIO_USB_HS_MICROFRAMES_PER_SECOND - 1u)) /
        AUDIO_USB_HS_MICROFRAMES_PER_SECOND;

    return audio_usb_fifo_target_bytes_clamp(target_frames * AUDIO_USB_FRAME_BYTES,
                                             CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ,
                                             CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX);
}

// USB IN flow controlの目標FIFO水位。EP IN interval 2回分 = 1.0 ms相当。
static uint16_t audio_transport_usb_in_fifo_target_bytes(uint32_t sample_rate_hz)
{
    const uint64_t target_frames =
        (uint64_t) audio_frames_per_usb_in_interval(sample_rate_hz) *
        AUDIO_USB_IN_TARGET_INTERVALS;

    return audio_usb_fifo_target_bytes_clamp(target_frames * AUDIO_USB_FRAME_BYTES,
                                             CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ,
                                             CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX);
}

// TinyUSBのIN FIFO目標を現在レートの1.0 ms相当へ適用する。Audio Task contextのみ。
static void audio_transport_apply_usb_in_fifo_target(uint32_t sample_rate_hz)
{
    if (!tud_inited())
    {
        return;
    }

    tud_audio_n_set_ep_in_fifo_threshold(AUDIO_FUNC_ID,
                                         audio_transport_usb_in_fifo_target_bytes(sample_rate_hz));
}

static bool audio_usb_in_source_ready(uint32_t sample_rate_hz)
{
    const uint32_t required_words = audio_frames_per_usb_in_interval(sample_rate_hz) * AUDIO_RING_FRAME_WORDS;
    const int32_t available_words = audio_ring_used_words(&s_rx_ring);

    return available_words >= (int32_t) required_words;
}

static uint16_t audio_out_read_budget_bytes(void)
{
    int32_t used = audio_ring_used_words(&s_tx_ring);
    audio_diagnostics_record_tx_level(s_streaming_out, used);
    if (used < 0)
    {
        used = 0;
    }
    if (used > (int32_t) s_tx_ring.capacity_words)
    {
        used = (int32_t) s_tx_ring.capacity_words;
    }

    // Keep the TX ring around target + one DMA half-buffer.
    int32_t budget_words = (int32_t) SAI_TX_TARGET_LEVEL_WORDS + (int32_t) (SAI_TX_BUF_SIZE / 2) - used;
    if (budget_words <= 0)
    {
        return 0;
    }

    budget_words = (budget_words / (int32_t) AUDIO_RING_FRAME_WORDS) * (int32_t) AUDIO_RING_FRAME_WORDS;

    uint32_t bytes = (uint32_t) budget_words * sizeof(int32_t);
    if (bytes > sizeof(usb_playback_buf))
    {
        bytes = sizeof(usb_playback_buf);
    }
    return (uint16_t) bytes;
}

static void copybuf_ring2usb_and_send(uint32_t sample_rate_hz)
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
    audio_diagnostics_record_usb_in_fifo(tu_fifo_count(ep_in_ff));
#endif

    const uint32_t frames    = audio_frames_per_usb_in_interval(sample_rate_hz);  // 24 or 48 frames/0.5ms
    const uint32_t sai_words = frames * AUDIO_RING_FRAME_WORDS;  // 4ch(4word/frame)

    int32_t used = audio_ring_used_words(&s_rx_ring);
    if (used < 0)
    {
        audio_ring_discard_all(&s_rx_ring);
        return;
    }
    if (used < (int32_t) sai_words)
    {
#if AUDIO_DIAG_LOG
        audio_diagnostics_record_usb_in_source_wait();
#endif
        return;  // 足りないなら今回は送らない
    }

    // USBは4ch、SAIも4ch
    // SAI: [L1][R1][L2][R2][L1][R1][L2][R2]...
    // USB: [L1][R1][L2][R2][L1][R1][L2][R2]...

    // 24bit in 32bit slot: SAI(2ch) -> USB(4ch) 変換
    const uint32_t usb_bytes = frames * AUDIO_USB_FRAME_CHANNELS * sizeof(int32_t);  // 4ch分

    // 安全: usb_capture_buf が足りない想定なら絶対に書かない
    if (usb_bytes > sizeof(usb_capture_buf))
        return;

    const bool send_ch1_to_usb = !timecode_synth_is_channel_enabled(0u);
    const bool send_ch2_to_usb = !timecode_synth_is_channel_enabled(1u);

    for (uint32_t f = 0; f < frames; f++)
    {
        const uint32_t frame_index = s_rx_ring.read_index + f * AUDIO_RING_FRAME_WORDS;
        uint32_t r_L1 = audio_ring_offset(&s_rx_ring, frame_index + 0U);
        uint32_t r_R1 = audio_ring_offset(&s_rx_ring, frame_index + 1U);
        uint32_t r_L2 = audio_ring_offset(&s_rx_ring, frame_index + 2U);
        uint32_t r_R2 = audio_ring_offset(&s_rx_ring, frame_index + 3U);
        usb_capture_buf[f * AUDIO_USB_FRAME_CHANNELS + 0] = send_ch1_to_usb ? s_rx_ring.data[r_L1] : 0;  // L1
        usb_capture_buf[f * AUDIO_USB_FRAME_CHANNELS + 1] = send_ch1_to_usb ? s_rx_ring.data[r_R1] : 0;  // R1
        usb_capture_buf[f * AUDIO_USB_FRAME_CHANNELS + 2] = send_ch2_to_usb ? s_rx_ring.data[r_L2] : 0;  // L2
        usb_capture_buf[f * AUDIO_USB_FRAME_CHANNELS + 3] = send_ch2_to_usb ? s_rx_ring.data[r_R2] : 0;  // R2
    }

    // ISRコンテキストから呼ばれるので通常版を使用
    uint16_t written = tud_audio_n_write(AUDIO_FUNC_ID, usb_capture_buf, (uint16_t) usb_bytes);

#if AUDIO_DIAG_LOG
    audio_diagnostics_record_usb_in_write(written, (uint16_t) usb_bytes);
    audio_diagnostics_record_usb_in_fifo(tu_fifo_count(ep_in_ff));
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
    s_rx_ring.read_index += written_frames * AUDIO_RING_FRAME_WORDS;  // SAIは4ch分
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
    audio_diagnostics_record_usb_in_packet(n_bytes_sent);
#else
    (void) n_bytes_sent;
#endif

    // USB INの転送完了後、次の0.5ms分が揃っている時だけTaskを起こす。
    if (s_streaming_in && !usb_tx_pending && audio_usb_in_source_ready(get_current_sample_rate_hz()))
    {
        usb_tx_pending = true;
#if AUDIO_DIAG_LOG
        audio_diagnostics_record_usb_in_notify();
#endif
        audio_transport_notify_from_isr();
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

    audio_diagnostics_record_usb_out_packet(n_bytes_received, rx_cycle, usb_rx_pending);

    tu_fifo_t* ep_out_ff = tud_audio_n_get_ep_out_ff(AUDIO_FUNC_ID);
    if (ep_out_ff != NULL)
    {
        audio_diagnostics_record_usb_out_fifo(tu_fifo_count(ep_out_ff));
    }
#else
    (void) n_bytes_received;
#endif

    usb_rx_pending = true;
    audio_transport_notify_from_isr();
    return true;
}

void audio_transport_service(uint32_t sample_rate_hz)
{
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
        usb_rx_event_cycle = audio_diagnostics_usb_out_pending_cycle();
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
        audio_diagnostics_record_usb_out_service(usb_rx_event_cycle);
    }
#endif

    // DMA halfをTinyUSB FIFO操作より先に処理する。
    // USB -> SAI (TX側の安全なhalfを書き換える)
    copybuf_ring2sai(sample_rate_hz);

    // SAI -> USB (RX側の安全なhalfをリングへ退避する)
    copybuf_sai2ring(sample_rate_hz);

    // USB OUTは受信通知とFIFO残量に追従して即時に吸い出す。
    if (usb_rx_event || tud_audio_n_available(AUDIO_FUNC_ID) > 0U)
    {
        uint16_t budget = audio_out_read_budget_bytes();
        uint16_t avail = tud_audio_n_available(AUDIO_FUNC_ID);
        uint16_t to_read = (avail < budget) ? avail : budget;
        if (to_read > sizeof(usb_playback_buf))
        {
            to_read = (uint16_t) sizeof(usb_playback_buf);
        }

        if (to_read > 0U)
        {
            spk_data_size = tud_audio_n_read(AUDIO_FUNC_ID, usb_playback_buf, to_read);
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
    audio_diagnostics_record_usb_out_read(s_streaming_out, spk_data_size);
#endif

    // USB -> SAI
    if (spk_data_size > 0)
    {
        copybuf_usb2ring();
    }

    // USB INはエンドポイントの1転送間隔分（現在0.5ms）が揃ったら次の塊を積む。
    // SAI RX DMA通知でも補充することで、IN FIFOが空になった場合の停止を防ぐ。
    if (s_streaming_in && (usb_tx_event || audio_usb_in_source_ready(sample_rate_hz)))
    {
        copybuf_ring2usb_and_send(sample_rate_hz);
    }
}

bool audio_transport_is_output_streaming(void)
{
    return s_streaming_out;
}

bool audio_transport_is_input_streaming(void)
{
    return s_streaming_in;
}

int32_t audio_transport_tx_used_words(void)
{
    return audio_ring_used_words(&s_tx_ring);
}
