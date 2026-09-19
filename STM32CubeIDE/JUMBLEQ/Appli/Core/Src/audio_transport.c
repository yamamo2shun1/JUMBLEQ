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
#include "audio_control_internal.h"
#include "audio_diagnostics_internal.h"
#include "audio_transport_internal.h"
#include "audio_usb_control_internal.h"
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
    // USB再生(OUT)primingの目標水位（消費前、word単位）。通常の消費前目標と同じ基準。
    // 224 word = 56 frame = 48kHzで約1.167ms、96kHzで約0.583ms。
    AUDIO_TX_PRIME_LEVEL_WORDS = SAI_TX_TARGET_LEVEL_WORDS + (SAI_TX_BUF_SIZE / 2),
    // ドリフト補正の開始／解除閾値（消費前水位、word単位）。
    // 開始は目標±2 frame、解除は±1 frame。USB OUTパケットは48kHzで約13 frame相当が
    // 0.125msごとに届き、水位は通常±1フレーム強揺れるため、開始だけでは補正せず、
    // 逸脱の継続時間と補正間隔の条件を満たした場合だけ補正する。
    AUDIO_TX_DRIFT_UP_START_WORDS     = AUDIO_TX_PRIME_LEVEL_WORDS + 8,
    AUDIO_TX_DRIFT_UP_RELEASE_WORDS   = AUDIO_TX_PRIME_LEVEL_WORDS + 4,
    AUDIO_TX_DRIFT_DOWN_START_WORDS   = AUDIO_TX_PRIME_LEVEL_WORDS - 8,
    AUDIO_TX_DRIFT_DOWN_RELEASE_WORDS = AUDIO_TX_PRIME_LEVEL_WORDS - 4,
    // 逸脱がこの時間継続した場合だけ補正する。通常パケット周期（0.125ms）の
    // 揺れは継続しないため、48/96kHzで共通の時間基準として20msとする。
    AUDIO_TX_DRIFT_HOLD_MS = 20u,
    // 補正の最小間隔。最大10 frame/s（1 frame補正）に相当し、20ms毎の判断でも
    // 補正が連続しないようにする。実測で不足する場合はこの値で調整する。
    AUDIO_TX_DRIFT_MIN_INTERVAL_MS = 100u,
    // 補正時のクロスフェード長（frame）。1 frameの位相移動を8 frameへ分散し、
    // 4chで同じ位置・同じ係数を使う。DMA half（32 frame）より十分短くする。
    AUDIO_TX_DRIFT_BLEND_FRAMES = 8u,
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
_Static_assert((AUDIO_TX_PRIME_LEVEL_WORDS % AUDIO_RING_FRAME_WORDS) == 0u,
               "USB playback priming level must preserve frame alignment");
_Static_assert((AUDIO_TX_PRIME_LEVEL_WORDS * (uint32_t) sizeof(int32_t)) <=
                   CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ,
               "USB playback priming level must fit one USB read buffer");
_Static_assert(AUDIO_TX_PRIME_LEVEL_WORDS < SAI_RNG_BUF_SIZE,
               "USB playback priming level must fit the TX ring");
_Static_assert(AUDIO_TX_DRIFT_BLEND_FRAMES < ((SAI_TX_BUF_SIZE / 2u) / AUDIO_RING_FRAME_WORDS),
               "drift blend must be shorter than one DMA half");
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

// リング折り返しを考慮して1 frame（4 word）を読み出す。
// absolute_word_indexはframe境界（4の倍数）を指すこと。
static inline void audio_ring_load_frame(const AudioRingBuffer_t* ring,
                                         uint32_t absolute_word_index,
                                         int32_t* frame)
{
    for (uint32_t i = 0; i < AUDIO_RING_FRAME_WORDS; i++)
    {
        frame[i] = ring->data[audio_ring_offset(ring, absolute_word_index + i)];
    }
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

        // 未takeのpayloadをackで完了扱いにした場合、古い原因・HALコードが次の
        // publishへマージされないよう消去する。take済みなら既に空。ack以後の
        // 新しい要求がpendingならpayloadはその要求のものなので保持する。
        if (s_recovery_request.acknowledged_sequence == s_recovery_request.published_sequence)
        {
            s_recovery_request.cause_mask       = 0u;
            s_recovery_request.dma_error_code   = 0u;
            s_recovery_request.sai_error_code   = 0u;
            s_recovery_request.sai_status_flags = 0u;
        }
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

// USB再生(OUT)のprimingとドリフト補正の状態。Audio Taskだけが更新する。
// primedは「USB実データがpriming水位へ到達した」ことだけを示し、無音や停止前の残存データでは成立しない。
static bool s_tx_primed = false;
static uint32_t s_tx_usb_fill_words = 0u;
static int8_t s_tx_drift_direction = 0;  // +1: 上側（水位過多）, -1: 下側（不足傾向）, 0: 逸脱なし
static bool s_tx_drift_since_valid = false;
static uint32_t s_tx_drift_since_tick = 0u;
static bool s_tx_drift_suppressed_recorded = false;
static bool s_tx_last_correction_valid = false;
static uint32_t s_tx_last_correction_tick = 0u;

// OUT再生開始境界。primingと補正履歴を初期化する。Audio Task context only。
static void audio_tx_playback_state_reset(void)
{
    // 開始境界より前の残存データ（停止前のUSB音声や無音データ）を再生対象から除外し、
    // 次のUSB実データがリング先頭から再生されるようにする。
    audio_ring_discard_all(&s_tx_ring);

    s_tx_primed = false;
    s_tx_usb_fill_words = 0u;
    s_tx_drift_direction = 0;
    s_tx_drift_since_valid = false;
    s_tx_drift_since_tick = 0u;
    s_tx_drift_suppressed_recorded = false;
    s_tx_last_correction_valid = false;
    s_tx_last_correction_tick = 0u;
}

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

static HAL_StatusTypeDef audio_transport_start_tx_path(void)
{
    HAL_StatusTypeDef status = MX_List_GPDMA1_Channel2_Config();
    if (status != HAL_OK)
    {
        return status;
    }
    status = HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel2, &List_GPDMA1_Channel2);
    if (status != HAL_OK)
    {
        return status;
    }

    handle_GPDMA1_Channel2.XferHalfCpltCallback = dma_sai2_tx_half;
    handle_GPDMA1_Channel2.XferCpltCallback     = dma_sai2_tx_cplt;
    handle_GPDMA1_Channel2.XferErrorCallback    = dma_sai_error;
    status = HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel2);
    if (status != HAL_OK)
    {
        return status;
    }

    // 手動DMA開始方式に合わせてHAL状態をBUSY_TXへ設定する。HAL IRQ handlerの
    // エラー分類（OVRUDR時のUDR/OVR判定）がこのStateを参照する。
    hsai_BlockA2.State = HAL_SAI_STATE_BUSY_TX;
    // SAIエラー割り込みを有効化してHAL_SAI_ErrorCallback（復旧要求）へ到達させる。
    __HAL_SAI_ENABLE_IT(&hsai_BlockA2, SAI_ERROR_INTERRUPT_MASK);

    hsai_BlockA2.Instance->CR1 |= SAI_xCR1_DMAEN;
    __HAL_SAI_ENABLE(&hsai_BlockA2);
    return HAL_OK;
}

static HAL_StatusTypeDef audio_transport_start_rx_path(void)
{
    HAL_StatusTypeDef status = MX_List_GPDMA1_Channel3_Config();
    if (status != HAL_OK)
    {
        return status;
    }
    status = HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel3, &List_GPDMA1_Channel3);
    if (status != HAL_OK)
    {
        return status;
    }

    handle_GPDMA1_Channel3.XferHalfCpltCallback = dma_sai1_rx_half;
    handle_GPDMA1_Channel3.XferCpltCallback     = dma_sai1_rx_cplt;
    handle_GPDMA1_Channel3.XferErrorCallback    = dma_sai_error;
    status = HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel3);
    if (status != HAL_OK)
    {
        return status;
    }

    // 手動DMA開始方式に合わせてHAL状態をBUSY_RXへ設定する。HAL IRQ handlerの
    // エラー分類（OVRUDR時のUDR/OVR判定）がこのStateを参照する。
    hsai_BlockA1.State = HAL_SAI_STATE_BUSY_RX;
    // SAIエラー割り込みを有効化してHAL_SAI_ErrorCallback（復旧要求）へ到達させる。
    __HAL_SAI_ENABLE_IT(&hsai_BlockA1, SAI_ERROR_INTERRUPT_MASK);

    hsai_BlockA1.Instance->CR1 |= SAI_xCR1_DMAEN;
    __HAL_SAI_ENABLE(&hsai_BlockA1);
    return HAL_OK;
}

bool audio_transport_dma_abort_confirmed(DMA_HandleTypeDef* hdma)
{
    if (hdma == NULL)
    {
        return false;
    }

    if (hdma->State == HAL_DMA_STATE_BUSY)
    {
        return (HAL_DMA_Abort(hdma) == HAL_OK);
    }

    // BUSY以外はabortを試みない。HAL_DMA_Abortは非BUSY時にErrorCodeを
    // HAL_DMA_ERROR_NO_XFERへ上書きするため、ErrorCodeだけではabort timeout
    // （State=ERROR、チャネルが動作中の可能性がある）と区別できない。
    // READY(停止済み・転送エラー後のHALリセット済み)とRESET(未初期化)だけを
    // 動作していない状態として扱い、ERROR/SUSPEND/ABORTは停止完了と見なさない。
    return (hdma->State == HAL_DMA_STATE_READY) || (hdma->State == HAL_DMA_STATE_RESET);
}

uint32_t audio_transport_recovery_request_sequence(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t sequence = s_recovery_request.published_sequence;

    __set_PRIMASK(primask);
    return sequence;
}

// SAI停止。HAL_SAI_Abortで停止完了確認・DMA abort・IMR/フラグ初期化・FIFO flushを行い、
// StateをREADYへ戻す（次のHAL_SAI_Initが生成MspInitをスキップできる状態）。
// MSP資源とSAI設定は維持する（DeInitしない）。
// 戻り値はSAI/GPDMA停止完了確認の成否。falseの場合は呼出側が再構築を行わない。
static bool audio_transport_stop_sai_paths(AudioTransportFailure_t* failure)
{
    bool stopped = true;

    HAL_StatusTypeDef sai_status = HAL_SAI_Abort(&hsai_BlockA2);
    if (sai_status != HAL_OK)
    {
        stopped = false;
        audio_transport_failure_record(failure, AUDIO_RATE_OP_SAI_ABORT_TX,
                                       (uint32_t) sai_status, hsai_BlockA2.ErrorCode);
    }
    sai_status = HAL_SAI_Abort(&hsai_BlockA1);
    if (sai_status != HAL_OK)
    {
        stopped = false;
        audio_transport_failure_record(failure, AUDIO_RATE_OP_SAI_ABORT_RX,
                                       (uint32_t) sai_status, hsai_BlockA1.ErrorCode);
    }

    // SAI経由のabortに加え、各GPDMAチャネルが停止したことを個別に確認する。
    if (!audio_transport_dma_abort_confirmed(&handle_GPDMA1_Channel2))
    {
        stopped = false;
        audio_transport_failure_record(failure, AUDIO_RATE_OP_GPDMA_ABORT_TX,
                                       (uint32_t) HAL_ERROR, handle_GPDMA1_Channel2.ErrorCode);
    }
    if (!audio_transport_dma_abort_confirmed(&handle_GPDMA1_Channel3))
    {
        stopped = false;
        audio_transport_failure_record(failure, AUDIO_RATE_OP_GPDMA_ABORT_RX,
                                       (uint32_t) HAL_ERROR, handle_GPDMA1_Channel3.ErrorCode);
    }
    __DSB();

    return stopped;
}

static HAL_StatusTypeDef audio_transport_init_dma_channel(DMA_HandleTypeDef* hdma,
                                                          DMA_Channel_TypeDef* instance)
{
    hdma->Instance                         = instance;
    hdma->InitLinkedList.Priority          = DMA_LOW_PRIORITY_HIGH_WEIGHT;
    hdma->InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
    hdma->InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
    hdma->InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
    hdma->InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;

    HAL_StatusTypeDef status = HAL_DMAEx_List_Init(hdma);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_DMA_ConfigChannelAttributes(hdma, DMA_CHANNEL_NPRIV);
}

static bool audio_transport_reinit_dma_channels(AudioTransportFailure_t* failure)
{
    (void) HAL_DMA_DeInit(&handle_GPDMA1_Channel2);
    (void) HAL_DMA_DeInit(&handle_GPDMA1_Channel3);

    HAL_StatusTypeDef status = audio_transport_init_dma_channel(&handle_GPDMA1_Channel2,
                                                                GPDMA1_Channel2);
    if (status != HAL_OK)
    {
        audio_transport_failure_record(failure, AUDIO_RATE_OP_GPDMA_INIT_TX,
                                       (uint32_t) status, handle_GPDMA1_Channel2.ErrorCode);
        return false;
    }

    status = audio_transport_init_dma_channel(&handle_GPDMA1_Channel3, GPDMA1_Channel3);
    if (status != HAL_OK)
    {
        audio_transport_failure_record(failure, AUDIO_RATE_OP_GPDMA_INIT_RX,
                                       (uint32_t) status, handle_GPDMA1_Channel3.ErrorCode);
        return false;
    }

    return true;
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

    audio_tx_playback_state_reset();

    __DSB();
}

void audio_transport_start(void)
{
    // ========================================
    // リングバッファは空のまま開始する
    // SAI DMAが開始直後にHalf割り込みを発生させても、USB再生priming中の
    // fill_tx_half()がDMA halfへ無音を書き込むためアンダーランにならない。
    // 無音prefillをリングへ入れると、priming完了時にUSB音声の前に
    // prefill分の無音が残って再生されるため、リングは空にしておく。
    // ========================================
    memset(s_tx_ring.data, 0, sizeof(s_tx_ring_storage));
    audio_ring_reset_indices(&s_tx_ring, 0U);
    audio_transport_reset_tx_diagnostics();
    dma_audio_event_reset(&s_rx_dma_event);

#if AUDIO_DIAG_LOG
    audio_diagnostics_reset_session();
#endif

    // SAI2 -> Slave Transmit
    // USB -> STM32 -(SAI)-> ADAU1466
    if (audio_transport_start_tx_path() != HAL_OK)
    {
        Error_Handler();
    }

    osDelay(500);
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    // SAI1 -> Slave Receize
    // ADAU1466 -(SAI)-> STM32 -> USB
    if (audio_transport_start_rx_path() != HAL_OK)
    {
        Error_Handler();
    }
}

// 復旧・レート変更共通。SAI/GPDMA停止 → DMAイベント/リングindexリセット →
// DMA/リング/USBバッファ消去。timecode設定には触れない。Audio Task context only。
// SAIのMSP資源と設定を維持し、HAL_SAI_MspInit（Error_Handlerを含む）を再実行させない。
// g_audio_tx_diagnosticsは原因調査のため保持する（明示リセットは呼出側で行う）。
// 戻り値はSAI/GPDMA停止完了確認の成否。falseの場合は転送停止を確認できていないため、
// 参照バッファの消去・再利用を行わない。failureへ失敗した操作とHAL結果を格納する。
bool audio_transport_stop_and_clear_paths(AudioTransportFailure_t* failure)
{
    if (!audio_transport_stop_sai_paths(failure))
    {
        return false;
    }

    audio_ring_reset_indices(&s_tx_ring, 0U);
    audio_ring_reset_indices(&s_rx_ring, 0U);

    // レート変更・復旧でも再生開始境界をやり直す（OUT有効のまま再構築する場合を含む）。
    audio_tx_playback_state_reset();

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

    return true;
}

bool audio_transport_reset_for_sample_rate(uint32_t sample_rate_hz,
                                           AudioTransportFailure_t* failure)
{
    if (!audio_transport_stop_and_clear_paths(failure))
    {
        return false;
    }

    // レート変更はstream開始と同様、TX診断を明示的にリセットする。
    audio_transport_reset_tx_diagnostics();

    timecode_synth_reset_for_sample_rate(sample_rate_hz);

    // alt settingが維持されたままレートだけ変わる場合に備え、新レートのIN FIFO目標も適用する。
    audio_transport_apply_usb_in_fifo_target(sample_rate_hz);

    // 切り替え期間中のTinyUSB残存データをここで破棄し、再開後のprimingや
    // USB IN送信へ持ち越さない。
    audio_transport_clear_usb_fifos();

    __DSB();

    return true;
}

// DMA channel再構築とTX開始。失敗時は両経路を停止してfalse。
// READY状態からのHAL_SAI_Init（MspInitをスキップ）によりMSP資源を維持したまま
// SAI設定とErrorCodeを再初期化し、DMAリンクを再実行して再始動する。
bool audio_transport_rebuild_and_start_tx(AudioTransportFailure_t* failure)
{
    /* Re-init DMA channels (linked-list mode) */
    if (!audio_transport_reinit_dma_channels(failure))
    {
        (void) audio_transport_stop_sai_paths(failure);
        return false;
    }

    /* 停止時にStateがREADYへ戻っているためMspInitは再実行されない。 */
    HAL_StatusTypeDef sai_status = HAL_SAI_Init(&hsai_BlockA1);
    if (sai_status != HAL_OK)
    {
        audio_transport_failure_record(failure, AUDIO_RATE_OP_SAI_INIT_RX,
                                       (uint32_t) sai_status, hsai_BlockA1.ErrorCode);
        (void) audio_transport_stop_sai_paths(failure);
        return false;
    }
    sai_status = HAL_SAI_Init(&hsai_BlockA2);
    if (sai_status != HAL_OK)
    {
        audio_transport_failure_record(failure, AUDIO_RATE_OP_SAI_INIT_TX,
                                       (uint32_t) sai_status, hsai_BlockA2.ErrorCode);
        (void) audio_transport_stop_sai_paths(failure);
        return false;
    }

    // HAL_SAI_Initで設定が入り直すためDMAリンクを再実行する。
    __HAL_LINKDMA(&hsai_BlockA1, hdmarx, handle_GPDMA1_Channel3);
    __HAL_LINKDMA(&hsai_BlockA2, hdmatx, handle_GPDMA1_Channel2);

    /* TX ringは空のまま再始動する。priming完了まではfill_tx_half()が無音を書き、
     * prefillを入れるとUSB音声の前に無音が残って再生されるため。 */
    audio_ring_reset_indices(&s_tx_ring, 0U);

    /* Configure and link DMA for SAI2 TX */
    sai_status = audio_transport_start_tx_path();
    if (sai_status != HAL_OK)
    {
        audio_transport_failure_record(failure, AUDIO_RATE_OP_TX_PATH_START,
                                       (uint32_t) sai_status, handle_GPDMA1_Channel2.ErrorCode);
        (void) audio_transport_stop_sai_paths(failure);
        return false;
    }

    return true;
}

// TX同期待ち後のRX開始。失敗時は両経路を停止（MSP維持）してfalse。
bool audio_transport_start_rx_after_tx_sync(AudioTransportFailure_t* failure)
{
    /* Configure and link DMA for SAI1 RX */
    HAL_StatusTypeDef rx_status = audio_transport_start_rx_path();
    if (rx_status != HAL_OK)
    {
        audio_transport_failure_record(failure, AUDIO_RATE_OP_RX_PATH_START,
                                       (uint32_t) rx_status, handle_GPDMA1_Channel3.ErrorCode);
        (void) audio_transport_stop_sai_paths(failure);
        return false;
    }

    // 復旧後もalt settingが維持される場合があるため、IN FIFO目標を再適用する。
    audio_transport_apply_usb_in_fifo_target(audio_control_transport_sample_rate_hz());
    return true;
}

// TinyUSBのIN/OUT FIFOに残る切り替え期間中のデータを公開APIで破棄する。
// TinyUSBが内部で使用しているバッファには触れない。Audio Task context only。
// クリアはFIFOのrd_idx/wr_idxを更新するため、USB ISRとUSB TaskのFIFO操作の
// どちらとも直列化する必要がある。PRIMASKはISRと（PendSV/SysTick経由の）
// Task切替の両方を止めるため、この短区間で両方と排他できる。
void audio_transport_clear_usb_fifos(void)
{
    if (!tud_inited())
    {
        return;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    (void) tud_audio_n_clear_ep_out_ff(AUDIO_FUNC_ID);
    (void) tud_audio_n_clear_ep_in_ff(AUDIO_FUNC_ID);

    __set_PRIMASK(primask);

    spk_data_size = 0u;
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
        audio_tx_playback_state_reset();

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

        audio_tx_playback_state_reset();
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
        audio_transport_apply_usb_in_fifo_target(audio_control_transport_sample_rate_hz());
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

    // primingは「リングにある実データ」ではなく、USBから補充した語数で判定する。
    // 無音や停止前の残存データをpriming完了の根拠にしないため。
    if (!s_tx_primed)
    {
        s_tx_usb_fill_words += sai_words;
        if (s_tx_usb_fill_words > AUDIO_TX_PRIME_LEVEL_WORDS)
        {
            s_tx_usb_fill_words = AUDIO_TX_PRIME_LEVEL_WORDS;
        }
    }
}

// ドリフト補正の継続履歴（逸脱方向と計時）だけを解除する。
// primed状態と実施済み補正の最終時刻は維持する。Audio Task context only。
static void audio_tx_drift_history_clear(void)
{
    s_tx_drift_direction = 0;
    s_tx_drift_since_valid = false;
    s_tx_drift_suppressed_recorded = false;
}

// 消費前水位からドリフト補正の方向を決める。開始／解除閾値のヒステリシス、
// 逸脱の継続時間、補正の最小間隔を満たした場合だけ補正方向を返す。
// 実際に補正を実施した側がaudio_tx_drift_commit()を呼び、頻度制限の時刻を更新する。
// Audio Task context only。
static int8_t audio_tx_drift_update(int32_t used, uint32_t now_ms)
{
    if (!s_streaming_out || !s_tx_primed || (used < 0) ||
        (used > (int32_t) s_tx_ring.capacity_words))
    {
        return 0;
    }

    // 実underrun相当（halfを満たない水位）では補正できず、保持／無音で回復を待つ。
    // 未実施の補正で頻度制限を開始しないよう、逸脱の継続履歴をここで解除する。
    if (used < (int32_t) (SAI_TX_BUF_SIZE / 2))
    {
        audio_tx_drift_history_clear();
        return 0;
    }

    // ヒステリシス: 逸脱中は解除閾値へ戻るまで同じ方向を維持する。
    if (s_tx_drift_direction > 0)
    {
        if (used <= (int32_t) AUDIO_TX_DRIFT_UP_RELEASE_WORDS)
        {
            s_tx_drift_direction = 0;
            s_tx_drift_since_valid = false;
            s_tx_drift_suppressed_recorded = false;
        }
    }
    else if (s_tx_drift_direction < 0)
    {
        if (used >= (int32_t) AUDIO_TX_DRIFT_DOWN_RELEASE_WORDS)
        {
            s_tx_drift_direction = 0;
            s_tx_drift_since_valid = false;
            s_tx_drift_suppressed_recorded = false;
        }
    }
    else if (used >= (int32_t) AUDIO_TX_DRIFT_UP_START_WORDS)
    {
        // 方向反転時は前方向の継続履歴を引き継がず、ここから計時する。
        s_tx_drift_direction = 1;
        s_tx_drift_since_valid = true;
        s_tx_drift_since_tick = now_ms;
        s_tx_drift_suppressed_recorded = false;
        audio_diagnostics_record_tx_drift_threshold(true);
    }
    else if (used <= (int32_t) AUDIO_TX_DRIFT_DOWN_START_WORDS)
    {
        s_tx_drift_direction = -1;
        s_tx_drift_since_valid = true;
        s_tx_drift_since_tick = now_ms;
        s_tx_drift_suppressed_recorded = false;
        audio_diagnostics_record_tx_drift_threshold(false);
    }
    else
    {
        // 通常の水位変動域。逸脱なし。
    }

    if ((s_tx_drift_direction == 0) || !s_tx_drift_since_valid)
    {
        return 0;
    }

    if ((uint32_t) (now_ms - s_tx_drift_since_tick) < AUDIO_TX_DRIFT_HOLD_MS)
    {
        return 0;
    }

    if (s_tx_last_correction_valid &&
        ((uint32_t) (now_ms - s_tx_last_correction_tick) < AUDIO_TX_DRIFT_MIN_INTERVAL_MS))
    {
        if (!s_tx_drift_suppressed_recorded)
        {
            audio_diagnostics_record_tx_drift_suppressed(s_tx_drift_direction > 0);
            s_tx_drift_suppressed_recorded = true;
        }
        return 0;
    }

    return s_tx_drift_direction;
}

// 補正（消費量変更と補間）を実際に実施した後に呼ぶ。頻度制限の時刻を更新し、
// 継続時間を再計時する。Audio Task context only。
static void audio_tx_drift_commit(uint32_t now_ms)
{
    s_tx_last_correction_valid = true;
    s_tx_last_correction_tick = now_ms;
    // 補正後も逸脱が続く場合に備え、継続時間を再計時する。
    s_tx_drift_since_tick = now_ms;
}

static void fill_tx_half(uint32_t index0)
{
    const uint32_t n           = (SAI_TX_BUF_SIZE / 2);
    const uint32_t frame_words = AUDIO_RING_FRAME_WORDS;
    uint32_t consume_words     = n;
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
        // 同期ズレは破棄して合わせ直す。新しい再生開始境界として再primingする。
        audio_ring_discard_all(&s_tx_ring);
        audio_tx_playback_state_reset();
        used = 0;
    }

    // USB実データの充填待ち（priming）。リングは消費せず無音を出し、
    // USB受信とリング補充はaudio_transport_service()側で継続する。
    if (!s_tx_primed)
    {
        if (s_tx_usb_fill_words >= AUDIO_TX_PRIME_LEVEL_WORDS)
        {
            s_tx_primed = true;
            audio_diagnostics_record_tx_priming_complete();
        }
        else
        {
            // OUT停止中の待機は通常運用のsilenceなので診断へ計上しない。
            if (streaming)
            {
                audio_diagnostics_record_tx_priming_wait();
            }
            memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
            timecode_synth_render_output(stereo_out_buf + index0,
                                         AUDIO_RING_FRAME_WORDS,
                                         (SAI_TX_BUF_SIZE / 2u) / AUDIO_RING_FRAME_WORDS);
            return;
        }
    }

    // データ不足時は可能な分だけ再生し、残りは末尾フレーム保持で埋める
    // いきなり無音にせず、クリック感を抑える
    if (used < (int32_t) n)
    {
        diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_UNDERRUN;
        // 空リング等で早期returnする場合も未実施の補正履歴を残さないよう先に解除する。
        // primed状態と実施済み補正の最終時刻は維持する。
        audio_tx_drift_history_clear();
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
        // リセットして無音で埋める。新しい再生開始境界として再primingする。
        audio_ring_discard_all(&s_tx_ring);
        audio_tx_playback_state_reset();
        memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
        timecode_synth_render_output(stereo_out_buf + index0,
                                     AUDIO_RING_FRAME_WORDS,
                                     (SAI_TX_BUF_SIZE / 2u) / AUDIO_RING_FRAME_WORDS);
        return;
    }

    // 長時間再生時のUSB/SAIクロック差を吸収するため、水位の継続的な逸脱に対して
    // 1 frameだけ消費量を増減する。priming中・実underrun・異常水位では補正しない。
    const uint32_t now_ms = HAL_GetTick();
    const int8_t drift_direction = audio_tx_drift_update(used, now_ms);
    bool drift_applied = false;
    if ((drift_direction > 0) && (used >= (int32_t) (n + frame_words)))
    {
        // バッファ過多: 1 frame余分に消費し、half内のクロスフェードでつなぐ。
        consume_words = n + frame_words;
        drift_applied = true;
        diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_DRIFT_UP;
    }
    else if ((drift_direction < 0) && (used >= (int32_t) n))
    {
        // バッファ不足傾向: 1 frame 少なく消費し、クロスフェードで引き伸ばす。
        consume_words = n - frame_words;
        drift_applied = true;
        diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_DRIFT_DOWN;
    }

    if (drift_applied)
    {
        // 実際に補正を実施した場合だけ頻度制限の時刻を更新する。
        audio_tx_drift_commit(now_ms);
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

    if (drift_applied)
    {
        // 補正half: 先頭の非補間部は通常コピーし、末尾BLEND_FRAMESだけ
        // 1 frame分ずらした2ソースを線形クロスフェードする。出力は常にn word。
        const uint32_t copy_frames  = n / frame_words;
        const uint32_t blend_frames = AUDIO_TX_DRIFT_BLEND_FRAMES;
        const uint32_t plain_words  = (copy_frames - blend_frames) * frame_words;

        const uint32_t index1 = audio_ring_offset(&s_tx_ring, s_tx_ring.read_index);
        uint32_t first = s_tx_ring.capacity_words - index1;
        if (first > plain_words)
        {
            first = plain_words;
        }

        memcpy(stereo_out_buf + index0, s_tx_ring.data + index1, first * sizeof(int32_t));
        if (first < plain_words)
        {
            memcpy(stereo_out_buf + index0 + first,
                   s_tx_ring.data,
                   (plain_words - first) * sizeof(int32_t));
        }

        for (uint32_t j = 0; j < blend_frames; j++)
        {
            const uint32_t frame_index = (plain_words / frame_words) + j;
            int32_t frame_a[4];
            int32_t frame_b[4];
            audio_ring_load_frame(&s_tx_ring,
                                  s_tx_ring.read_index + frame_index * frame_words,
                                  frame_a);
            const uint32_t other_offset = (drift_direction > 0) ?
                                              ((frame_index + 1u) * frame_words) :
                                              ((frame_index - 1u) * frame_words);
            audio_ring_load_frame(&s_tx_ring, s_tx_ring.read_index + other_offset, frame_b);

            // 4chすべて同じ位置・同じ係数でクロスフェードする（ch間を混ぜない）。
            // 中間値はint64で計算し、丸めは0から遠い側へ寄せる。24bit-in-32bitの
            // 表現に依存せず、係数が0..blend_framesの凸結合なので結果は必ず
            // 2入力の範囲内に収まり、飽和は不要。
            const uint32_t w = j + 1u;
            int32_t* dst = stereo_out_buf + index0 + plain_words + (j * frame_words);
            for (uint32_t c = 0; c < frame_words; c++)
            {
                int64_t blended =
                    ((int64_t) frame_a[c] * (int64_t) (blend_frames - w)) +
                    ((int64_t) frame_b[c] * (int64_t) w);
                if (blended >= 0)
                {
                    blended += (int64_t) (blend_frames / 2u);
                }
                else
                {
                    blended -= (int64_t) (blend_frames / 2u);
                }
                dst[c] = (int32_t) (blended / (int64_t) blend_frames);
            }
        }
    }
    else
    {
        uint32_t copy_words = consume_words;
        if (copy_words > n)
        {
            copy_words = n;
        }

        const uint32_t index1 = audio_ring_offset(&s_tx_ring, s_tx_ring.read_index);
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

    const bool streaming = s_streaming_out;
    // 既存service計測と同じ位置で処理開始時刻を取得する。期限計算はこの後に行うため、
    // 既存service_cyclesへ期限計算時間は混入しない。
    const uint32_t process_start_cycle = DWT->CYCCNT;

    uint32_t deadline_cycles;
    if (streaming)
    {
        const uint32_t service_cycles = process_start_cycle - event.cycle;
        deadline_cycles =
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
    else
    {
        // stream停止中もDMAとfill/synthは動作するため、完了計測用の期限を算出する。
        deadline_cycles =
            audio_dma_half_deadline_cycles(SAI_TX_BUF_SIZE / 2u, sample_rate_hz);
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

    // half更新完了時点を境界とし、コピー・補間・合成を含む完了時間を計測する。
    const uint32_t end_cycle = DWT->CYCCNT;
    audio_diagnostics_record_tx_dma_complete(event.event,
                                             end_cycle - process_start_cycle,
                                             end_cycle - event.cycle,
                                             deadline_cycles);
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

    const bool streaming = s_streaming_in;
    // 既存service計測と同じ位置で処理開始時刻を取得する。
    const uint32_t process_start_cycle = DWT->CYCCNT;

    uint32_t deadline_cycles;
    if (streaming)
    {
        const uint32_t service_cycles = process_start_cycle - event.cycle;
        deadline_cycles =
            audio_dma_half_deadline_cycles(SAI_RX_BUF_SIZE / 2u, sample_rate_hz);
        audio_diagnostics_record_rx_dma_service(event.event, service_cycles,
                                                service_cycles > deadline_cycles);

        if (event.dropped_events != 0u)
        {
            audio_diagnostics_record_rx_events_dropped(event.event, event.dropped_events);
        }
    }
    else
    {
        // stream停止中もDMAとfill/synthは動作するため、完了計測用の期限を算出する。
        deadline_cycles =
            audio_dma_half_deadline_cycles(SAI_RX_BUF_SIZE / 2u, sample_rate_hz);
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

    // half取り込み完了時点を境界とし、synth処理を含む完了時間を計測する。
    const uint32_t end_cycle = DWT->CYCCNT;
    audio_diagnostics_record_rx_dma_complete(event.event,
                                             end_cycle - process_start_cycle,
                                             end_cycle - event.cycle,
                                             deadline_cycles);
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

    // priming中はUSBから補充した実データ量を基準に読み出す。
    // primed後は従来どおり target + one DMA half-buffer を基準にする。
    int32_t budget_words;
    if (!s_tx_primed)
    {
        budget_words = (int32_t) AUDIO_TX_PRIME_LEVEL_WORDS - (int32_t) s_tx_usb_fill_words;
    }
    else
    {
        budget_words = (int32_t) SAI_TX_TARGET_LEVEL_WORDS +
                       (int32_t) (SAI_TX_BUF_SIZE / 2) - used;
    }
    if (budget_words <= 0)
    {
        return 0;
    }

    // リングの空きを超えて読むとcopybuf_usb2ring()で捨てられるため、空きで制限する。
    const int32_t free_words = (int32_t) (s_tx_ring.capacity_words - 1U) - used;
    if (free_words <= 0)
    {
        return 0;
    }
    if (budget_words > free_words)
    {
        budget_words = free_words;
    }

    budget_words = (budget_words / (int32_t) AUDIO_RING_FRAME_WORDS) * (int32_t) AUDIO_RING_FRAME_WORDS;
    if (budget_words <= 0)
    {
        return 0;
    }

    uint32_t bytes = (uint32_t) budget_words * sizeof(int32_t);
    if (bytes > sizeof(usb_playback_buf))
    {
        bytes = sizeof(usb_playback_buf);
    }
    return (uint16_t) bytes;
}

// レート未確定・失敗期間のUSB OUT残存FIFOを読み捨てる。リングへは積まず、
// usb_playback_bufは転送用の一時領域としてのみ使用する。Audio Task context only。
static void audio_transport_discard_usb_out(void)
{
    spk_data_size = 0u;

    if (!tud_inited() || !tud_audio_n_mounted(AUDIO_FUNC_ID))
    {
        return;
    }

    uint16_t avail = tud_audio_n_available(AUDIO_FUNC_ID);
    while (avail > 0u)
    {
        const uint16_t chunk = (avail > (uint16_t) sizeof(usb_playback_buf)) ?
                                   (uint16_t) sizeof(usb_playback_buf) :
                                   avail;
        const uint16_t read = tud_audio_n_read(AUDIO_FUNC_ID, usb_playback_buf, chunk);
        if (read == 0u)
        {
            break;
        }
        avail = tud_audio_n_available(AUDIO_FUNC_ID);
    }
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
    // レート未確定・失敗中・停止未確認の復旧中は搬送を許可しない。
    if (s_streaming_in && !usb_tx_pending &&
        audio_control_transport_ready() && audio_control_transport_paths_safe() &&
        audio_usb_in_source_ready(audio_control_transport_sample_rate_hz()))
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

    // アプリ側feedback更新。搬送禁止中は水位不足による補正をしない。
    audio_usb_control_feedback_update();

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

    // レート未確定（初期適用失敗・切替中・FAILED）の間、およびDMA復旧が停止確認を
    // 経ていない間は通常搬送を許可しない。USB OUTは読み捨て、SAI/RXリング・USB IN
    // へは積まず、参照バッファにも触れない。
    const uint32_t request_sequence = audio_control_rate_request_sequence();
    if (!audio_control_transport_commit_allowed(request_sequence))
    {
        audio_transport_discard_usb_out();
        return;
    }

    // 以降のcommit区間は要求publishとmutexで排他にする。TinyUSB APIを割り込み禁止
    // 区間へ入れずに、受理済み要求がcommitの途中へ割り込まないことを保証する。
    if (!audio_control_commit_lock())
    {
        // ロックできない場合は安全側でこの周期の搬送を見送る。
        audio_transport_discard_usb_out();
        return;
    }

    if (!audio_control_transport_commit_allowed(request_sequence))
    {
        audio_control_commit_unlock();
        audio_transport_discard_usb_out();
        return;
    }

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

    // USB -> SAI。commit区間内なので要求sequenceは変化しない。
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

    audio_control_commit_unlock();
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
