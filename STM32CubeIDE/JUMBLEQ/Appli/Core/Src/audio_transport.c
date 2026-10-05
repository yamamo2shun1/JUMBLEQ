/*
 * audio_transport.c
 *
 * Realtime audio transport between TinyUSB UAC2 endpoints and SAI TX/RX over
 * GPDMA. SAI TX DMA halves are read directly from the USB OUT FIFO and SAI RX
 * DMA halves are written directly into the USB IN FIFO.
 *
 * ISR callbacks only publish the latest DMA event and pending flags; the
 * Audio Task applies stream requests and performs all buffer copies.
 * The DMA halves are accessed only by one copy each (TX commit / RX snapshot)
 * while the GPDMA channel is suspended (#251).
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
    // 1.0 ms (EP IN interval 0.5 ms x 2)。DMA half単位（約0.333ms）で書き0.5msごとに
    // 送るため、送信直前の水位は目標から約DMA half/4下振れし、Task遅延で1チャンクが
    // 送信をまたぐとさらに1 DMA half下がる。長さ0のpacketを避ける下限
    // (interval−1 frame)＋DMA half＋DMA half/4 は48kHz 43 frame・96kHz 87 frameで、
    // 目標（48 / 96 frame）はこれを満たす。
    AUDIO_USB_IN_TARGET_INTERVALS    = 2u,
    // USB IN FIFOへの書込み上限（暫定値、実測で確定する）。flow control目標(2 interval)
    // ＋書込み1チャンク（DMA half）＋余白 = 2.0 ms相当のframe数。保持データ量の上限であり、
    // ホスト停滞中の経過時間の上限ではない。超過時はFIFOの古いデータを目標まで破棄する。
    AUDIO_USB_IN_FIFO_LIMIT_INTERVALS = 4u,
    // SAI DMA half期間をサンプルレートに依らず約0.333ms（1/3000秒）にする。
    // 48kHzで16 frame（64 word）、96kHzで32 frame（128 word）。
    AUDIO_SAI_DMA_HALVES_PER_SECOND = 3000u,
    AUDIO_SAI_DMA_HALF_WORDS_MIN    = 64u,
    AUDIO_SAI_DMA_HALF_WORDS_MAX    = SAI_TX_DMA_BUF_WORDS / 2u,
    // ドリフト補正（最終手段）の閾値はUSB OUT FIFO水位で判定する（audio_tx_out_levels()）。
    // 逸脱がこの時間継続した場合だけ補正を始める。通常パケット周期（0.125ms）の
    // 揺れは継続しないため、48/96kHzで共通の時間基準として20msとする。
    AUDIO_TX_DRIFT_HOLD_MS = 20u,
    // 補正開始後、同じ方向の逸脱が続く間の補正の最小間隔。最大100 frame/s。
    // feedbackに従わないホストとSAIのクロック差±500ppm（48kHzで24 frame/s、
    // 96kHzで48 frame/s）を保証範囲とし、96kHzでも約2倍の余裕を持たせる。
    AUDIO_TX_DRIFT_MIN_INTERVAL_MS = 10u,
    // 上側補正の開始／解除: 通常の読出し直前水位（目標＋half/2）からの時間（µs）。
    AUDIO_TX_DRIFT_UP_START_US   = 1000u,
    AUDIO_TX_DRIFT_UP_RELEASE_US = 500u,
    // 補正時のクロスフェード長（frame）。1 frameの位相移動を8 frameへ分散し、
    // 4chで同じ位置・同じ係数を使う。最短のDMA half（16 frame）より短くする。
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
#define AUDIO_USB_IN_LIMIT_BYTES_MAX \
    ((AUDIO_USB_IN_TARGET_BYTES_MAX / AUDIO_USB_IN_TARGET_INTERVALS) * AUDIO_USB_IN_FIFO_LIMIT_INTERVALS)
// 最大サンプルレート時の上側ドリフト補正の開始水位（byte）。目標＋half/2＋1.0ms。
#define AUDIO_TX_DRIFT_UP_START_BYTES_MAX \
    (AUDIO_USB_OUT_TARGET_BYTES_MAX + ((AUDIO_SAI_DMA_HALF_WORDS_MAX / 2u) * (uint32_t) sizeof(int32_t)) + \
     ((((uint32_t) CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE * AUDIO_TX_DRIFT_UP_START_US) / 1000000u) * \
      (uint32_t) AUDIO_USB_FRAME_BYTES))

#define SAI_ERROR_STATUS_MASK \
    (SAI_xSR_OVRUDR | SAI_xSR_WCKCFG | SAI_xSR_CNRDY | SAI_xSR_AFSDET | SAI_xSR_LFSDET)

// 有効化するSAIエラー割り込み。SRフラグのマスクと対応させ、HAL_SAI_ErrorCallback
// （＝復旧要求publish）へ到達させる。
#define SAI_ERROR_INTERRUPT_MASK \
    (SAI_IT_OVRUDR | SAI_IT_WCKCFG | SAI_IT_CNRDY | SAI_IT_AFSDET | SAI_IT_LFSDET)

extern DMA_QListTypeDef List_GPDMA1_Channel2;
extern DMA_QListTypeDef List_GPDMA1_Channel3;

_Static_assert((uint32_t) DMA_AUDIO_EVENT_NONE == (uint32_t) AUDIO_DIAG_DMA_EVENT_NONE,
               "DMA event encoding must match the diagnostics API");
_Static_assert((uint32_t) DMA_AUDIO_EVENT_HALF == (uint32_t) AUDIO_DIAG_DMA_EVENT_HALF,
               "DMA event encoding must match the diagnostics API");
_Static_assert((uint32_t) DMA_AUDIO_EVENT_COMPLETE == (uint32_t) AUDIO_DIAG_DMA_EVENT_COMPLETE,
               "DMA event encoding must match the diagnostics API");
_Static_assert(AUDIO_USB_FRAME_BYTES == AUDIO_RING_FRAME_WORDS * sizeof(int32_t),
               "USB and ring buffer frame sizes must match");
_Static_assert((AUDIO_SAI_DMA_HALF_WORDS_MIN % AUDIO_RING_FRAME_WORDS) == 0u,
               "minimum DMA half must preserve frame alignment");
_Static_assert((AUDIO_SAI_DMA_HALF_WORDS_MAX % AUDIO_RING_FRAME_WORDS) == 0u,
               "maximum DMA half must preserve frame alignment");
_Static_assert(AUDIO_SAI_DMA_HALF_WORDS_MIN <= AUDIO_SAI_DMA_HALF_WORDS_MAX,
               "DMA half range must be ordered");
_Static_assert((AUDIO_SAI_DMA_HALF_WORDS_MAX * 2u) <= SAI_TX_DMA_BUF_WORDS,
               "maximum DMA transfer must fit the TX DMA buffer");
_Static_assert((AUDIO_SAI_DMA_HALF_WORDS_MAX * 2u) <= SAI_RX_DMA_BUF_WORDS,
               "maximum DMA transfer must fit the RX DMA buffer");
// USB OUT FIFOは唯一の再生バッファ。上側補正の開始水位の上に、最大packetと
// 読出し1 half分の余裕を残してFIFO容量に収まること（満杯は保証範囲外でだけ起きる）。
_Static_assert(AUDIO_TX_DRIFT_UP_START_BYTES_MAX + CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX +
                   (AUDIO_SAI_DMA_HALF_WORDS_MAX * sizeof(int32_t)) <=
                   CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ,
               "USB OUT FIFO must hold the upper drift threshold, one packet and one DMA half");
_Static_assert((CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ % AUDIO_USB_FRAME_BYTES) == 0u,
               "USB OUT FIFO depth must preserve frame alignment");
_Static_assert(CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ <= 0x7FFFu,
               "USB OUT FIFO depth must allow overflow detection (tu_fifo index range)");
_Static_assert(AUDIO_TX_DRIFT_BLEND_FRAMES < (AUDIO_SAI_DMA_HALF_WORDS_MIN / AUDIO_RING_FRAME_WORDS),
               "drift blend must be shorter than one DMA half");
// FIFO目標は最大packetを追加で格納できる余白を残し、uint16_tに収まること。
_Static_assert(AUDIO_USB_OUT_TARGET_BYTES_MAX + CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX <=
                   CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ,
               "USB OUT FIFO target must leave room for one maximum packet");
_Static_assert(AUDIO_USB_IN_TARGET_BYTES_MAX + CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX <=
                   CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ,
               "USB IN FIFO target must leave room for one maximum packet");
// IN FIFO書込み上限は、目標の上に書込み1チャンク（DMA half）と余白を残し、
// 最大packet分の余白を残してFIFO容量に収まること。
_Static_assert(AUDIO_USB_IN_FIFO_LIMIT_INTERVALS >= AUDIO_USB_IN_TARGET_INTERVALS + 2u,
               "IN FIFO limit must leave one chunk and one interval of headroom above the target");
_Static_assert((AUDIO_SAI_DMA_HALF_WORDS_MAX * sizeof(int32_t)) <=
                   (AUDIO_USB_IN_LIMIT_BYTES_MAX - AUDIO_USB_IN_TARGET_BYTES_MAX),
               "IN FIFO limit must leave room for one DMA half chunk above the target");
_Static_assert(AUDIO_USB_IN_LIMIT_BYTES_MAX + CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX <=
                   CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ,
               "IN FIFO limit must leave room for one maximum packet");

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
} audio_transport_dma_event_state_t;

typedef struct
{
    uint32_t event;
    uint32_t cycle;
    uint32_t dropped_events;
    uint32_t generation;
} audio_transport_dma_event_snapshot_t;

static volatile audio_transport_dma_event_state_t s_tx_dma_event = {0};
static volatile audio_transport_dma_event_state_t s_rx_dma_event = {0};
static volatile uint32_t s_dma_event_generation = 0u;

static inline uint32_t dma_audio_event_publish_from_isr(volatile audio_transport_dma_event_state_t* state,
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

static bool dma_audio_event_take_latest(volatile audio_transport_dma_event_state_t* state,
                                        audio_transport_dma_event_snapshot_t* snapshot)
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

static inline void dma_audio_event_reset_locked(volatile audio_transport_dma_event_state_t* state)
{
    state->consumed_sequence = state->produced_sequence;
    state->latest_event      = DMA_AUDIO_EVENT_NONE;
    state->latest_cycle      = 0u;
    state->dropped_events    = 0u;
}

static void dma_audio_event_reset(volatile audio_transport_dma_event_state_t* state)
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
} audio_transport_recovery_request_state_t;

static volatile audio_transport_recovery_request_state_t s_recovery_request = {0};

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

// Audio Task context. ISR用と同じpayload・sequenceの更新をPRIMASK区間で行い、
// 通知はTask用APIで行う（ISR用の通知をTaskから呼ばない）。エラーコードは記録しない。
static void recovery_request_publish_from_task(uint32_t cause_bit)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    s_recovery_request.cause_mask |= cause_bit;
    __DMB();
    s_recovery_request.published_sequence++;

    __set_PRIMASK(primask);
    audio_transport_notify_task();
}

bool audio_transport_take_recovery_request(audio_transport_recovery_request_t* request)
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
// IN開始要求（alt≠0のSET_INTERFACE）の世代。要求publishと同じPRIMASK区間で進める。
// 適用前にstop→startが集約された場合や、streaming中の再SET_INTERFACEでも
// 開始境界を作るために、maskとは別に数える。
static volatile uint32_t s_stream_in_start_sequence  = 0u;
static uint32_t s_stream_in_applied_start_sequence   = 0u;  // Audio Task専用
// OUT開始要求の世代。INと同じ理由で、streaming中の再SET_INTERFACEや適用前に集約された
// stop→startでも再生開始境界（priming・補正履歴のリセット）を作るために数える。
static volatile uint32_t s_stream_out_start_sequence  = 0u;
static uint32_t s_stream_out_applied_start_sequence   = 0u;  // Audio Task専用

// USB再生(OUT)のprimingとドリフト補正の状態。Audio Taskだけが更新する。
// primedは「USB OUT FIFOの水位がpriming水位へ到達した」ことを示す。FIFOはOUT開始境界で
// TinyUSBがclearするため、停止前の残存データでは成立しない。
static bool s_tx_primed = false;
static int8_t s_tx_drift_direction = 0;  // +1: 上側（水位過多）, -1: 下側（不足傾向）, 0: 逸脱なし
static bool s_tx_drift_since_valid = false;
static uint32_t s_tx_drift_since_tick = 0u;
static bool s_tx_drift_suppressed_recorded = false;
static bool s_tx_last_correction_valid = false;
static uint32_t s_tx_last_correction_tick = 0u;

// OUT再生開始境界。primingと補正履歴を初期化する。Audio Task context only。
static void audio_tx_playback_state_reset(void)
{
    s_tx_primed = false;
    s_tx_drift_direction = 0;
    s_tx_drift_since_valid = false;
    s_tx_drift_since_tick = 0u;
    s_tx_drift_suppressed_recorded = false;
    s_tx_last_correction_valid = false;
    s_tx_last_correction_tick = 0u;
}

// DMA half所有権確認（#251）の作業バッファ。CPUだけが使い、DMAはアクセスしない
// （キャッシュ保守は不要）。DMA共有領域へのアクセスは、TXはs_tx_stage_buf→halfのcommit、
// RXはhalf→s_rx_stage_bufのsnapshotだけにし、どちらもGPDMA一時停止中に行う。
static __attribute__((aligned(32))) int32_t s_tx_stage_buf[AUDIO_SAI_DMA_HALF_WORDS_MAX] = {0};
static __attribute__((aligned(32))) int32_t s_rx_stage_buf[AUDIO_SAI_DMA_HALF_WORDS_MAX] = {0};
// s_tx_stage_bufの内容。Audio Taskだけが更新する。has_stream_audio: USB OUT FIFOのデータを含む。
// timecode_mask: timecode合成が上書きしたch（bit n = ch n、L/Rの組）。
static bool s_tx_stage_has_stream_audio = false;
static uint32_t s_tx_stage_timecode_mask = 0u;

// USB IN FIFOへ書き込む1チャンク（DMA half 1回分）の組立て領域。
static __attribute__((section("noncacheable_buffer"), aligned(32))) int32_t s_usb_capture_buf[AUDIO_SAI_DMA_HALF_WORDS_MAX] = {0};
// USB OUT FIFOからpeekするドリフト補正用の作業領域（DMA half＋1 frame）。
static __attribute__((section("noncacheable_buffer"), aligned(32))) int32_t s_usb_playback_buf[AUDIO_SAI_DMA_HALF_WORDS_MAX + AUDIO_RING_FRAME_WORDS] = {0};

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t sai_tx_dma_buf[SAI_TX_DMA_BUF_WORDS] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t sai_rx_dma_buf[SAI_RX_DMA_BUF_WORDS]  = {0};

// TX/RX共通のDMA half長（word単位）。SAI/GPDMA停止中にAudio Taskだけが
// audio_transport_configure_dma_period()で更新し、DMA動作中は変更しない。ISRは参照しない。
static uint32_t s_sai_dma_half_words = AUDIO_SAI_DMA_HALF_WORDS_MAX;

// 停止状態異常のラッチ（#251）。Audio Taskだけが更新する。立った経路のDMA共有領域へは
// 以後アクセスせず、復旧・レート変更でDMAを停止確認・再構築できたときにだけ解除する。
static bool s_tx_dma_suspend_fault = false;
static bool s_rx_dma_suspend_fault = false;

// DMA構築前に、サンプルレートからhalf長を約0.333ms相当へ確定する。
// 3000で割り切れないレートや、half長がMIN..MAXの範囲外になるレートは、
// 従来の32 frame（MAX）へフォールバックする。Audio Task context only。
static void audio_transport_configure_dma_period(uint32_t sample_rate_hz)
{
    uint32_t half_words = AUDIO_SAI_DMA_HALF_WORDS_MAX;

    if ((sample_rate_hz % AUDIO_SAI_DMA_HALVES_PER_SECOND) == 0u)
    {
        const uint32_t half_frames = sample_rate_hz / AUDIO_SAI_DMA_HALVES_PER_SECOND;
        if ((half_frames >= (AUDIO_SAI_DMA_HALF_WORDS_MIN / AUDIO_RING_FRAME_WORDS)) &&
            (half_frames <= (AUDIO_SAI_DMA_HALF_WORDS_MAX / AUDIO_RING_FRAME_WORDS)))
        {
            half_words = half_frames * AUDIO_RING_FRAME_WORDS;
        }
    }

    s_sai_dma_half_words = half_words;
}

uint32_t audio_transport_sai_dma_xfer_words(void)
{
    return s_sai_dma_half_words * 2u;
}

static void fill_tx_half(uint32_t dst_half_offset_words, uint32_t sample_rate_hz);
static void fill_rx_half(uint32_t src_half_offset_words, bool streaming, uint32_t sample_rate_hz);
static uint32_t audio_frames_per_usb_in_interval(uint32_t sample_rate_hz);
static void copy_rx_half_to_usb_in(const int32_t* src, uint32_t frames, uint32_t sample_rate_hz);
static void audio_transport_apply_usb_in_fifo_target(uint32_t sample_rate_hz);

// USB OUT FIFOの水位（word）。ISRからも呼ぶため読取りだけを行う。
static inline int32_t audio_tx_used_words(void)
{
    tu_fifo_t* ep_out_ff = tud_audio_n_get_ep_out_ff(AUDIO_FUNC_ID);
    return (ep_out_ff != NULL) ? (int32_t) (tu_fifo_count(ep_out_ff) / sizeof(int32_t)) : 0;
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
static bool audio_transport_stop_sai_paths(audio_transport_failure_t* failure)
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

static bool audio_transport_reinit_dma_channels(audio_transport_failure_t* failure)
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
    for (uint16_t i = 0; i < (sizeof(s_usb_capture_buf) / sizeof(s_usb_capture_buf[0])); i++)
    {
        s_usb_capture_buf[i] = 0;
    }

    for (uint16_t i = 0; i < (sizeof(s_usb_playback_buf) / sizeof(s_usb_playback_buf[0])); i++)
    {
        s_usb_playback_buf[i] = 0;
    }

    for (uint16_t i = 0; i < SAI_TX_DMA_BUF_WORDS; i++)
    {
        sai_tx_dma_buf[i] = 0;
    }

    for (uint16_t i = 0; i < SAI_RX_DMA_BUF_WORDS; i++)
    {
        sai_rx_dma_buf[i] = 0;
    }

    memset(s_tx_stage_buf, 0, sizeof(s_tx_stage_buf));
    memset(s_rx_stage_buf, 0, sizeof(s_rx_stage_buf));
    s_tx_stage_has_stream_audio = false;

    audio_tx_playback_state_reset();

    __DSB();
}

void audio_transport_start(void)
{
    // 起動時のDMA half長を、最初のノード構築より前に初期レートで確定する。
    audio_transport_configure_dma_period(audio_control_transport_sample_rate_hz());

    // SAI DMAが開始直後にHalf割り込みを発生させても、USB再生priming中の
    // fill_tx_half()がDMA halfへ無音を書き込むためアンダーランにならない。
    audio_tx_playback_state_reset();
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

// 復旧・レート変更共通。SAI/GPDMA停止 → DMAイベント・再生状態リセット →
// DMA/USB作業バッファ消去。timecode設定には触れない。Audio Task context only。
// SAIのMSP資源と設定を維持し、HAL_SAI_MspInit（Error_Handlerを含む）を再実行させない。
// g_audio_tx_diagnosticsは原因調査のため保持する（明示リセットは呼出側で行う）。
// 戻り値はSAI/GPDMA停止完了確認の成否。falseの場合は転送停止を確認できていないため、
// 参照バッファの消去・再利用を行わない。failureへ失敗した操作とHAL結果を格納する。
bool audio_transport_stop_and_clear_paths(audio_transport_failure_t* failure)
{
    if (!audio_transport_stop_sai_paths(failure))
    {
        return false;
    }

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
    memset(sai_tx_dma_buf, 0, sizeof(sai_tx_dma_buf));
    memset(sai_rx_dma_buf, 0, sizeof(sai_rx_dma_buf));
    memset(s_usb_playback_buf, 0, sizeof(s_usb_playback_buf));
    memset(s_usb_capture_buf, 0, sizeof(s_usb_capture_buf));
    memset(s_tx_stage_buf, 0, sizeof(s_tx_stage_buf));
    memset(s_rx_stage_buf, 0, sizeof(s_rx_stage_buf));
    s_tx_stage_has_stream_audio = false;
    __DSB();

    return true;
}

bool audio_transport_reset_for_sample_rate(uint32_t sample_rate_hz,
                                           audio_transport_failure_t* failure)
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
bool audio_transport_rebuild_and_start_tx(audio_transport_failure_t* failure)
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

    /* 再生状態（priming）はaudio_transport_stop_and_clear_paths()でリセット済み。
     * priming完了まではfill_tx_half()が無音を書く。 */

    /* SAI/GPDMA停止確認後、TX/RXノード構築前にDMA half長を確定する。
     * 切替中は固定target、復旧時は適用済みレートが返る。後から開始するRXも同じ値を使う。 */
    audio_transport_configure_dma_period(audio_control_transport_sample_rate_hz());

    /* Configure and link DMA for SAI2 TX */
    sai_status = audio_transport_start_tx_path();
    if (sai_status != HAL_OK)
    {
        audio_transport_failure_record(failure, AUDIO_RATE_OP_TX_PATH_START,
                                       (uint32_t) sai_status, handle_GPDMA1_Channel2.ErrorCode);
        (void) audio_transport_stop_sai_paths(failure);
        return false;
    }

    // 停止確認（HAL_DMA_Abort）と再初期化を経てTXチャネルの状態が確定した。
    s_tx_dma_suspend_fault = false;
    return true;
}

// TX同期待ち後のRX開始。失敗時は両経路を停止（MSP維持）してfalse。
bool audio_transport_start_rx_after_tx_sync(audio_transport_failure_t* failure)
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

    // 停止確認と再初期化を経てRXチャネルの状態が確定した。
    s_rx_dma_suspend_fault = false;

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
}

void audio_transport_request_stream(audio_transport_stream_t stream, bool enabled)
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
        if (stream == AUDIO_TRANSPORT_STREAM_IN)
        {
            s_stream_in_start_sequence++;
        }
        else
        {
            s_stream_out_start_sequence++;
        }
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

static bool audio_stream_take_requested_state(uint32_t* requested_mask,
                                              bool* in_start_requested,
                                              bool* out_start_requested)
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

    // 開始世代はmaskと同じ区間で取り出し、未適用の開始要求の有無を返す。
    const uint32_t in_start_sequence = s_stream_in_start_sequence;
    *in_start_requested = (in_start_sequence != s_stream_in_applied_start_sequence);
    s_stream_in_applied_start_sequence = in_start_sequence;
    const uint32_t out_start_sequence = s_stream_out_start_sequence;
    *out_start_requested = (out_start_sequence != s_stream_out_applied_start_sequence);
    s_stream_out_applied_start_sequence = out_start_sequence;

    __set_PRIMASK(primask);
    return true;
}

// start_requested: 未適用のOUT開始要求（alt≠0のSET_INTERFACE）があったか。
// streaming中の開き直しでも、TinyUSBが開始境界でclearしたFIFOをpriming水位から
// 再生し直す（FIFOはclearし直さず、clear後に受信した新しいデータは保持する）。
// 開始要求なしの再適用（IN要求のみ等）ではOUTの再生状態に触れない。
static void audio_stream_apply_out_state(bool enabled, bool start_requested)
{
    if (enabled && (!s_streaming_out || start_requested))
    {
        if (s_streaming_out)
        {
            // 再生中の開き直し。処理待ちのTX DMAイベントを破棄すると、直前に旧streamの
            // 音声を書いたhalfが再度出力されるため、イベントは残してpriming（無音）で埋める。
            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            audio_diagnostics_reset_tx_locked();
            __set_PRIMASK(primask);
        }
        else
        {
            audio_transport_reset_tx_diagnostics();
        }
        audio_tx_playback_state_reset();

        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_streaming_out = true;
        __set_PRIMASK(primask);
    }
    else if (!enabled && s_streaming_out)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_streaming_out      = false;
        dma_audio_event_reset_locked(&s_tx_dma_event);
        __set_PRIMASK(primask);

        audio_tx_playback_state_reset();
    }
    else
    {
        return;
    }

#if AUDIO_DIAG_LOG
    audio_diagnostics_reset_usb_out_gap();
#endif
}

// IN開始境界。境界より前にIN FIFOへ入っていた音声を送らない。
// Audio Task context only。IN FIFOのclearはUSB ISRの読出しとusbTaskのclearの
// 両方と排他するためPRIMASK区間で行う。OUT FIFOには触れない。
static void audio_stream_begin_in(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    tu_fifo_t* ep_in_ff = tud_audio_n_get_ep_in_ff(AUDIO_FUNC_ID);
    const uint32_t stale_bytes = (ep_in_ff != NULL) ? tu_fifo_count(ep_in_ff) : 0u;
    dma_audio_event_reset_locked(&s_rx_dma_event);
    (void) tud_audio_n_clear_ep_in_ff(AUDIO_FUNC_ID);
    s_streaming_in        = true;

    __set_PRIMASK(primask);

    audio_diagnostics_record_usb_in_start_boundary(stale_bytes / sizeof(int32_t));
}

// start_requested: 未適用のIN開始要求（alt≠0のSET_INTERFACE）があったか。
// 開始要求なしの再適用（OUT要求のみ等）ではstreaming中のIN FIFOを破棄しない。
static void audio_stream_apply_in_state(bool enabled, bool start_requested)
{
    if (enabled && (!s_streaming_in || start_requested))
    {
        audio_stream_begin_in();
    }
    else if (!enabled && s_streaming_in)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        s_streaming_in        = false;
        dma_audio_event_reset_locked(&s_rx_dma_event);

        __set_PRIMASK(primask);
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
    bool in_start_requested = false;
    bool out_start_requested = false;
    if (!audio_stream_take_requested_state(&requested_mask, &in_start_requested, &out_start_requested))
    {
        return false;
    }

#if AUDIO_DIAG_LOG
    const bool previous_out = s_streaming_out;
    const bool previous_in  = s_streaming_in;
#endif
    audio_stream_apply_out_state((requested_mask & AUDIO_STREAM_OUT_BIT) != 0u, out_start_requested);
    audio_stream_apply_in_state((requested_mask & AUDIO_STREAM_IN_BIT) != 0u, in_start_requested);

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
    // USB OUT/INのISR→Task通知フラグは持たない（OUT/INともDMAイベントを契機に搬送する）。
}

// ==============================
// DMA half ownership (#251)
// ==============================
//
// TX/RXのDMA共有領域へは、GPDMAチャネルを一時停止（CCR.SUSP）し、停止の完了を
// イベントフラグ（SUSPF）と状態フラグ（IDLEF）で確かめてからアクセスする。停止中は
// チャネルがマスタポートで転送しないため（RM0477 12.4.3）、アクセスにかかる時間や
// プリエンプト・バス競合に関係なく、DMAと同じwordへ同時にアクセスしない。位置は
// 停止中に1回だけ読み、その状態のまま判定・コピーする。
// 停止が長引いた場合に起きるのはSAIのunder/overrun（既存の復旧要求へ進む）で、
// DMA共有領域での競合ではない。

enum
{
    // GPDMA1 Ch0〜11のFIFO容量（RM0477 12.3.1 Table 88）。RXの宛先への書込みは、
    // ソースからの読出し（BNDT）より最大この語数だけ遅れる。
    AUDIO_DMA_FIFO_WORDS          = 2u,
    AUDIO_DMA_SUSPEND_TIMEOUT_US  = 1u,
    // RXのnot_readyでやり直す前に待つ上限（frame数）。
    AUDIO_DMA_RX_NOT_READY_WAIT_FRAMES = 2u,
};

#define AUDIO_DMA_CSR_ERROR_MASK (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF)

#ifndef AUDIO_TRANSPORT_DMA_ACCESS_HOOKS
// GPDMAレジスタとDMA共有領域へのアクセス。ホストテストはDMAの模擬へ差し替える。
static inline uint32_t audio_dma_reg_read(volatile uint32_t* reg)
{
    return *reg;
}

static inline void audio_dma_reg_write(volatile uint32_t* reg, uint32_t value)
{
    *reg = value;
}

// 停止中のコピーは32bit単位で行う。newlib-nanoのmemcpyはbyte単位のため、非キャッシュ領域の
// DMAバッファへのアクセス回数が4倍になり停止期間が延びる。volatileにして、コンパイラが
// ループをmemcpyへ戻したりアクセス幅を変えたりしないようにする。
static inline void audio_dma_copy_words(int32_t* dst, const int32_t* src, uint32_t words)
{
    volatile int32_t* const d       = dst;
    const volatile int32_t* const s = src;
    for (uint32_t i = 0u; i < words; i++)
    {
        d[i] = s[i];
    }
}
#endif

// BNDT（ソースから転送する残りbyte数）から、次にソースから読むword位置を求める。
// 0（再ロード直前）と転送長（再ロード直後）はどちらも先頭。転送長を超える値と
// 4の倍数でない値は不正としてfalseを返す。
static bool audio_dma_position_words(uint32_t bndt_bytes, uint32_t xfer_words, uint32_t* pos_words)
{
    const uint32_t xfer_bytes = xfer_words * (uint32_t) sizeof(int32_t);
    if ((bndt_bytes > xfer_bytes) || ((bndt_bytes % (uint32_t) sizeof(int32_t)) != 0u))
    {
        return false;
    }

    *pos_words = (bndt_bytes == 0u) ? 0u : ((xfer_bytes - bndt_bytes) / (uint32_t) sizeof(int32_t));
    return true;
}

// 相手half（half_indexでない方）の先頭から数えたDMA位置q（word）。
// 0 ≦ q < half_wordsならDMAは相手halfにいる。q == half_wordsは対象halfの先頭。
static uint32_t audio_dma_offset_from_other_half(uint32_t pos_words, uint32_t half_index, uint32_t half_words)
{
    const uint32_t xfer_words  = half_words * 2u;
    const uint32_t other_start = (half_index == 0u) ? half_words : 0u;
    return (pos_words + xfer_words - other_start) % xfer_words;
}

// TX（対象halfへ書く）: 停止中のDMAが対象halfの先頭wordをまだ読んでいなければ、
// 対象half全体を書ける。書いた内容はDMAが次に対象halfを読むときに使われる。
static uint32_t audio_dma_judge_tx(uint32_t q, uint32_t half_words)
{
    return (q <= half_words) ? AUDIO_DIAG_DMA_OWN_OK : AUDIO_DIAG_DMA_OWN_BUSY;
}

// RX（対象halfを読む）: 宛先の書込み位置dは p − FIFO ≦ d ≦ p。q ≧ FIFOなら対象halfの
// 末尾まで宛先へ書き終わっていて、q ≦ half_wordsなら次の周回の書込みはまだ始まっていない。
static uint32_t audio_dma_judge_rx(uint32_t q, uint32_t half_words)
{
    if (q > half_words)
    {
        return AUDIO_DIAG_DMA_OWN_BUSY;
    }
    if (q < AUDIO_DMA_FIFO_WORDS)
    {
        return AUDIO_DIAG_DMA_OWN_NOT_READY;
    }
    return AUDIO_DIAG_DMA_OWN_OK;
}

typedef struct
{
    uint32_t result;          // AUDIO_DIAG_DMA_OWN_*
    uint32_t q;               // 停止位置（停止後の状態確認とBNDTが正しいときだけ有効）
    uint32_t suspend_cycles;  // SUSP書込み→解除完了。停止を要求しなかった経路は0
    bool stale_flag_cleared;  // 開始前に古いSUSPFをクリアした
    bool fault;               // 今回、停止状態異常を新たに検出した
#if AUDIO_DMA_OWNERSHIP_TEST
    uint32_t ccr_before;
    uint32_t csr_before;
    uint32_t bndt;
    uint32_t sai_flvl;
#endif
} audio_dma_own_outcome_t;

#if AUDIO_DMA_OWNERSHIP_TEST
// 試験用の遅延注入とトレース（#251）。デバッガから書き換える。製品ビルドでは生成しない。
typedef struct
{
    uint32_t period_events;    // 何回のcommit/snapshotごとに注入するか（0: 注入しない）
    uint32_t tx_delay_us;      // 区間に入る前のビジーウェイト
    uint32_t rx_delay_us;
    uint32_t tx_target_q_enable;  // 1: 区間に入る前に、停止せずに読んだqがtargetになるまで待つ
    uint32_t tx_target_q;
    uint32_t rx_target_q_enable;
    uint32_t rx_target_q;
    uint32_t suspend_hold_us;  // 停止中（判定の後、解除の前）のビジーウェイト
    uint32_t tx_event_count;
    uint32_t rx_event_count;
    uint32_t injected_count;
    uint32_t target_q_timeouts;
} audio_dma_ownership_test_t;

typedef struct
{
    uint32_t tx;           // 1: TX commit、0: RX snapshot
    uint32_t half_index;
    uint32_t half_words;
    uint32_t ccr_before;
    uint32_t csr_before;
    uint32_t bndt;
    uint32_t q;
    uint32_t sai_flvl;     // 停止時のSAI_xSR.FLVL
    uint32_t result;
    uint32_t suspend_cycles;
} audio_dma_ownership_trace_t;

enum
{
    AUDIO_DMA_OWNERSHIP_TRACE_COUNT = 16u,
};

volatile audio_dma_ownership_test_t g_audio_dma_ownership_test = {0};
volatile audio_dma_ownership_trace_t g_audio_dma_ownership_trace[AUDIO_DMA_OWNERSHIP_TRACE_COUNT] = {0};
volatile uint32_t g_audio_dma_ownership_trace_count = 0u;
static bool s_dma_test_hold_active = false;

static void audio_dma_test_busy_wait_us(uint32_t us)
{
    const uint32_t start  = DWT->CYCCNT;
    const uint32_t cycles = (SystemCoreClock / 1000000u) * us;
    while ((DWT->CYCCNT - start) < cycles)
    {
    }
}

// 区間に入る前の注入。注入した回はtrueを返し、停止中の保持（suspend_hold_us）も有効にする。
static bool audio_dma_test_before(bool tx, DMA_HandleTypeDef* hdma, uint32_t half_index,
                                  uint32_t half_words, uint32_t sample_rate_hz)
{
    volatile audio_dma_ownership_test_t* const t = &g_audio_dma_ownership_test;
    const uint32_t count = tx ? ++t->tx_event_count : ++t->rx_event_count;
    if ((t->period_events == 0u) || ((count % t->period_events) != 0u))
    {
        s_dma_test_hold_active = false;
        return false;
    }

    t->injected_count++;
    audio_dma_test_busy_wait_us(tx ? t->tx_delay_us : t->rx_delay_us);

    if (tx ? (t->tx_target_q_enable != 0u) : (t->rx_target_q_enable != 0u))
    {
        // 停止せずに読んだ位置で待つ（判定させる位相の指定にだけ使う）。上限は4 half期間。
        const uint32_t target = tx ? t->tx_target_q : t->rx_target_q;
        const uint32_t start  = DWT->CYCCNT;
        const uint32_t limit  = (sample_rate_hz != 0u) ?
                                    (uint32_t) (((uint64_t) SystemCoreClock * 4u * (half_words / AUDIO_RING_FRAME_WORDS)) /
                                                sample_rate_hz) :
                                    0u;
        for (;;)
        {
            uint32_t pos;
            if (audio_dma_position_words(hdma->Instance->CBR1 & DMA_CBR1_BNDT, half_words * 2u, &pos) &&
                (audio_dma_offset_from_other_half(pos, half_index, half_words) == target))
            {
                break;
            }
            if ((DWT->CYCCNT - start) > limit)
            {
                t->target_q_timeouts++;
                break;
            }
        }
    }

    s_dma_test_hold_active = true;
    return true;
}

static void audio_dma_test_trace(bool tx, uint32_t half_index, uint32_t half_words,
                                 const audio_dma_own_outcome_t* outcome)
{
    const uint32_t index = g_audio_dma_ownership_trace_count % AUDIO_DMA_OWNERSHIP_TRACE_COUNT;
    volatile audio_dma_ownership_trace_t* const e = &g_audio_dma_ownership_trace[index];
    e->tx             = tx ? 1u : 0u;
    e->half_index     = half_index;
    e->half_words     = half_words;
    e->ccr_before     = outcome->ccr_before;
    e->csr_before     = outcome->csr_before;
    e->bndt           = outcome->bndt;
    e->q              = outcome->q;
    e->sai_flvl       = outcome->sai_flvl;
    e->result         = outcome->result;
    e->suspend_cycles = outcome->suspend_cycles;
    g_audio_dma_ownership_trace_count++;
    s_dma_test_hold_active = false;
}
#endif

// GPDMAチャネルを一時停止し、停止位置で所有権を確かめた場合だけhalfをコピーする。
// 呼出側がPRIMASK区間を保持すること。区間内でHAL・ログ・待機APIを呼ばず、途中で
// returnするのは停止を要求する前の経路だけ。停止を要求した後は、どの経路でも共通の
// 解除（SUSP=0 → 読み戻し → SUSPFクリア → 読み戻し。HAL_DMAEx_Resume()と同じ順）を通る。
// to_dma: true=TX（src→対象half）、false=RX（対象half→dst）。
static void audio_dma_owned_copy(DMA_HandleTypeDef* hdma,
                                 bool* fault_latch,
                                 bool to_dma,
                                 uint32_t half_index,
                                 uint32_t half_words,
                                 int32_t* dst,
                                 const int32_t* src,
                                 audio_dma_own_outcome_t* outcome)
{
    outcome->result             = AUDIO_DIAG_DMA_OWN_UNKNOWN;
    outcome->q                  = 0u;
    outcome->suspend_cycles     = 0u;
    outcome->stale_flag_cleared = false;
    outcome->fault              = false;

    if (*fault_latch)
    {
        outcome->result = AUDIO_DIAG_DMA_OWN_SUSPEND_FAULT;
        return;
    }

    DMA_Channel_TypeDef* const ch = hdma->Instance;

    // 開始前の状態確認。SUSP=0かつSUSPF=0を確かめてから停止を要求する。
    const uint32_t ccr = audio_dma_reg_read(&ch->CCR);
    const uint32_t csr = audio_dma_reg_read(&ch->CSR);
#if AUDIO_DMA_OWNERSHIP_TEST
    outcome->ccr_before = ccr;
    outcome->csr_before = csr;
    outcome->bndt       = 0u;
    outcome->sai_flvl   = 0u;
#endif
    if ((ccr & DMA_CCR_SUSP) != 0u)
    {
        // 前回の解除が終わっていない。チャネルの状態は既存の復旧で確定させる。
        *fault_latch    = true;
        outcome->fault  = true;
        outcome->result = AUDIO_DIAG_DMA_OWN_SUSPEND_FAULT;
        return;
    }
    if (((ccr & DMA_CCR_EN) == 0u) || ((csr & AUDIO_DMA_CSR_ERROR_MASK) != 0u))
    {
        return;
    }
    if ((csr & DMA_CSR_SUSPF) != 0u)
    {
        audio_dma_reg_write(&ch->CFCR, DMA_CFCR_SUSPF);
        __DSB();
        if ((audio_dma_reg_read(&ch->CSR) & DMA_CSR_SUSPF) != 0u)
        {
            *fault_latch    = true;
            outcome->fault  = true;
            outcome->result = AUDIO_DIAG_DMA_OWN_SUSPEND_FAULT;
            return;
        }
        outcome->stale_flag_cleared = true;
    }

    // 一時停止の要求。開始前にSUSPF=0を確かめているので、ここで見るSUSPF=1は今回の
    // 要求に対する停止完了を表す。期限の直後に完了した場合もアクセスしない。
    const uint32_t timeout_cycles = (SystemCoreClock / 1000000u) * AUDIO_DMA_SUSPEND_TIMEOUT_US;
    const uint32_t start_cycle    = DWT->CYCCNT;
    audio_dma_reg_write(&ch->CCR, audio_dma_reg_read(&ch->CCR) | DMA_CCR_SUSP);
    __DSB();

    // CSRを読んだ後に経過時間を確かめ、期限内に観測したSUSPFだけを停止完了として受理する。
    // 読取り自体が遅れて期限を過ぎた場合も、SUSPFの値によらずアクセスしない。
    bool suspended = false;
    for (;;)
    {
        const uint32_t csr_poll = audio_dma_reg_read(&ch->CSR);
        if ((DWT->CYCCNT - start_cycle) > timeout_cycles)
        {
            break;
        }
        if ((csr_poll & DMA_CSR_SUSPF) != 0u)
        {
            suspended = true;
            break;
        }
    }

    if (!suspended)
    {
        outcome->result = AUDIO_DIAG_DMA_OWN_SUSPEND_TIMEOUT;
    }
    else
    {
        // 停止した定常状態（RM0477 12.8.7: SUSPF=1、IDLEF=EN=1）を状態フラグでも確かめる。
        // 古いSUSPFを取り違えても、チャネルが転送中ならIDLEF=0で検出できる。
        const uint32_t ccr_s = audio_dma_reg_read(&ch->CCR);
        const uint32_t csr_s = audio_dma_reg_read(&ch->CSR);
        const bool steady = ((ccr_s & DMA_CCR_EN) != 0u) && ((ccr_s & DMA_CCR_SUSP) != 0u) &&
                            ((csr_s & DMA_CSR_IDLEF) != 0u) && ((csr_s & DMA_CSR_SUSPF) != 0u) &&
                            ((csr_s & AUDIO_DMA_CSR_ERROR_MASK) == 0u);
        if (steady)
        {
            // 停止中はBNDTが変わらない（RM0477 12.4.3）。
            const uint32_t bndt = audio_dma_reg_read(&ch->CBR1) & DMA_CBR1_BNDT;
            uint32_t pos;
#if AUDIO_DMA_OWNERSHIP_TEST
            outcome->bndt = bndt;
            if (hdma->Parent != NULL)
            {
                outcome->sai_flvl = (((SAI_HandleTypeDef*) hdma->Parent)->Instance->SR & SAI_xSR_FLVL) >>
                                    SAI_xSR_FLVL_Pos;
            }
#endif
            if (audio_dma_position_words(bndt, half_words * 2u, &pos))
            {
                outcome->q      = audio_dma_offset_from_other_half(pos, half_index, half_words);
                outcome->result = to_dma ? audio_dma_judge_tx(outcome->q, half_words) :
                                           audio_dma_judge_rx(outcome->q, half_words);
                if (outcome->result == AUDIO_DIAG_DMA_OWN_OK)
                {
                    // コピーを停止確認（SUSPF・IDLEF・BNDTの読取り）より前に行わず、
                    // 解除より前に完了させる。
                    __DMB();
                    audio_dma_copy_words(dst, src, half_words);
                    __DSB();
                }
            }
        }
#if AUDIO_DMA_OWNERSHIP_TEST
        if (s_dma_test_hold_active)
        {
            audio_dma_test_busy_wait_us(g_audio_dma_ownership_test.suspend_hold_us);
        }
#endif
    }

    // 解除（すべての経路で共通）。SUSP=0を確かめてからSUSPFをクリアするので、期限の直後や
    // 解除の途中に遅れて停止が完了しても、そのフラグは次の要求へ残らない。
    audio_dma_reg_write(&ch->CCR, audio_dma_reg_read(&ch->CCR) & ~DMA_CCR_SUSP);
    __DSB();
    bool released = ((audio_dma_reg_read(&ch->CCR) & DMA_CCR_SUSP) == 0u);
    audio_dma_reg_write(&ch->CFCR, DMA_CFCR_SUSPF);
    __DSB();
    released = released && ((audio_dma_reg_read(&ch->CSR) & DMA_CSR_SUSPF) == 0u);
    outcome->suspend_cycles = DWT->CYCCNT - start_cycle;

    if (!released)
    {
        // 停止中に行ったコピーは有効だが、以後はこの経路のDMA共有領域へアクセスしない。
        *fault_latch   = true;
        outcome->fault = true;
    }
}

// 停止期間がframe周期の半分を超えたか（性能目標。音切れしない保証ではない）。
static bool audio_dma_suspend_over_budget(uint32_t suspend_cycles, uint32_t sample_rate_hz)
{
    if (sample_rate_hz == 0u)
    {
        return false;
    }
    return suspend_cycles > ((SystemCoreClock / sample_rate_hz) / 2u);
}

// ==============================
// USB(OUT) FIFO -> SAI(TX) path
// ==============================

// OUT再生の判定に使うUSB OUT FIFO水位（word単位、frame境界）。いずれもhalf処理開始時
// （読出し直前）の水位と比較する。T = feedback目標（0.5ms）、P = T＋half/2 は
// feedbackが平均水位をTに保つときの通常の読出し直前水位。
typedef struct
{
    uint32_t prime_words;         // priming水位: T＋half
    uint32_t up_start_words;      // 上側補正の開始: P＋1.0ms
    uint32_t up_release_words;    // 上側補正の解除: P＋0.5ms
    uint32_t down_start_words;    // 下側補正の開始: half＋1 packet
    uint32_t down_release_words;  // 下側補正の解除: half＋2 packet
} audio_tx_out_levels_t;

static inline uint32_t audio_words_floor_frame(uint32_t words)
{
    return (words / AUDIO_RING_FRAME_WORDS) * AUDIO_RING_FRAME_WORDS;
}

static void audio_tx_out_levels(uint32_t sample_rate_hz, audio_tx_out_levels_t* levels)
{
    const uint32_t half_words   = s_sai_dma_half_words;
    const uint32_t target_words = audio_transport_usb_out_fifo_target_bytes(sample_rate_hz) / sizeof(int32_t);
    const uint32_t nominal_pre_read_words = target_words + (half_words / 2u);
    const uint32_t ms_words     = (sample_rate_hz / 1000u) * AUDIO_RING_FRAME_WORDS;
    const uint32_t packet_words =
        (sample_rate_hz / AUDIO_USB_HS_MICROFRAMES_PER_SECOND) * AUDIO_RING_FRAME_WORDS;

    levels->prime_words        = audio_words_floor_frame(target_words + half_words);
    levels->up_start_words     = audio_words_floor_frame(nominal_pre_read_words + ms_words);
    levels->up_release_words   = audio_words_floor_frame(nominal_pre_read_words + (ms_words / 2u));
    levels->down_start_words   = audio_words_floor_frame(half_words + packet_words);
    levels->down_release_words = audio_words_floor_frame(half_words + (2u * packet_words));
}

// ドリフト補正の継続履歴（逸脱方向と計時）だけを解除する。
// primed状態と実施済み補正の最終時刻は維持する。Audio Task context only。
static void audio_tx_drift_history_clear(void)
{
    s_tx_drift_direction = 0;
    s_tx_drift_since_valid = false;
    s_tx_drift_suppressed_recorded = false;
}

// 読出し直前のFIFO水位からドリフト補正の方向を決める。feedbackに従うホストでは
// 水位は開始閾値に達しない（最終手段）。開始／解除閾値のヒステリシスを持ち、逸脱が
// AUDIO_TX_DRIFT_HOLD_MS継続した後は、同じ方向の逸脱が続く間、最小間隔ごとに補正する。
// 実際に補正を実施した側がaudio_tx_drift_commit()を呼び、頻度制限の時刻を更新する。
// Audio Task context only。
static int8_t audio_tx_drift_update(uint32_t level_words,
                                    const audio_tx_out_levels_t* levels,
                                    uint32_t now_ms)
{
    if (!s_streaming_out || !s_tx_primed)
    {
        return 0;
    }

    // 実underrun相当（halfを満たない水位）では補正できず、保持／無音で回復を待つ。
    // 未実施の補正で頻度制限を開始しないよう、逸脱の継続履歴をここで解除する。
    if (level_words < s_sai_dma_half_words)
    {
        audio_tx_drift_history_clear();
        return 0;
    }

    // ヒステリシス: 逸脱中は解除閾値へ戻るまで同じ方向を維持する。
    if (s_tx_drift_direction > 0)
    {
        if (level_words <= levels->up_release_words)
        {
            audio_tx_drift_history_clear();
        }
    }
    else if (s_tx_drift_direction < 0)
    {
        if (level_words >= levels->down_release_words)
        {
            audio_tx_drift_history_clear();
        }
    }
    else if (level_words >= levels->up_start_words)
    {
        // 方向反転時は前方向の継続履歴を引き継がず、ここから計時する。
        s_tx_drift_direction = 1;
        s_tx_drift_since_valid = true;
        s_tx_drift_since_tick = now_ms;
        s_tx_drift_suppressed_recorded = false;
        audio_diagnostics_record_tx_drift_threshold(true);
    }
    else if (level_words <= levels->down_start_words)
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

    // 継続時間は逸脱の開始時だけに適用する。
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

// 補正（消費量変更と補間）を実際に実施した後に呼ぶ。頻度制限の時刻だけを更新し、
// 同じ方向の逸脱が続く間は最小間隔ごとに補正できるようにする。Audio Task context only。
static void audio_tx_drift_commit(uint32_t now_ms)
{
    s_tx_last_correction_valid = true;
    s_tx_last_correction_tick = now_ms;
}

typedef enum
{
    AUDIO_USB_OUT_TAKE_OK = 0,
    AUDIO_USB_OUT_TAKE_STALE,     // 未適用のstream要求あり（またはclearで水位が減少）。読まない
    AUDIO_USB_OUT_TAKE_OVERFLOW,  // 満杯・overflow。FIFOをclearした
} audio_usb_out_take_result_t;

// PRIMASK区間内で取得した値。区間外ではこの値だけを使う。
typedef struct
{
    audio_usb_out_take_result_t result;
    uint32_t raw_count_bytes;  // 区間内で取得したindex差（overflow時はdepthを超える）
    uint32_t depth_bytes;
    uint32_t section_cycles;   // 区間のサイクル数(DWT->CYCCNT)
} audio_usb_out_take_outcome_t;

// USB OUT FIFOからの読出し・peek→discard・overflowからの復旧。Audio Task context only。
// 上書き可能FIFOでは、満杯付近でread/peekのコピー途中にUSB ISRが書き込むと、読出し元の
// データが上書きされ1 frame内に新旧が混在し得る（tusb_fifo.cのwrite/read順序）。
// vTaskSuspendAll()はUSB ISRを止めないため、PRIMASK区間でUSB ISRとTask切替（usbTaskの
// FIFO clear）の両方を止める。区間内では待機API・ログ・HALを呼ばず、途中returnせず、
// PRIMASKを1回だけ復元する。コピー量は最大half＋1 frame。
// overflow状態のFIFOにはdiscard/peek/readを行わない（read indexが正規化されず、
// 二重overflowではデータ順序も保証されないため）。満杯・overflowならclearする。
static void audio_usb_out_take(tu_fifo_t* ep_out_ff,
                               int32_t* dst,
                               uint16_t peek_bytes,
                               uint16_t consume_bytes,
                               audio_usb_out_take_outcome_t* outcome)
{
    outcome->raw_count_bytes = 0u;
    outcome->depth_bytes     = 0u;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t section_start_cycle = DWT->CYCCNT;

    if ((s_stream_request_sequence != s_stream_applied_request_sequence) || !s_streaming_out)
    {
        outcome->result = AUDIO_USB_OUT_TAKE_STALE;
    }
    else
    {
        const uint16_t depth = ep_out_ff->depth;
        const uint16_t raw_count = tu_ff_overflow_count(depth, ep_out_ff->wr_idx, ep_out_ff->rd_idx);
        outcome->raw_count_bytes = raw_count;
        outcome->depth_bytes     = depth;

        if (raw_count >= depth)
        {
            (void) tud_audio_n_clear_ep_out_ff(AUDIO_FUNC_ID);
            outcome->result = AUDIO_USB_OUT_TAKE_OVERFLOW;
        }
        else if (raw_count < peek_bytes)
        {
            // 区間前の判定より水位が減るのはusbTaskのclearだけで、clearは要求publishと
            // 対で発生する。防御として読まずにSTALE扱いにする。
            outcome->result = AUDIO_USB_OUT_TAKE_STALE;
        }
        else if (peek_bytes == consume_bytes)
        {
            (void) tu_fifo_read_n(ep_out_ff, dst, consume_bytes);
            outcome->result = AUDIO_USB_OUT_TAKE_OK;
        }
        else
        {
            (void) tu_fifo_peek_n(ep_out_ff, dst, peek_bytes);
            (void) tu_fifo_discard_n(ep_out_ff, consume_bytes);
            outcome->result = AUDIO_USB_OUT_TAKE_OK;
        }
    }

    outcome->section_cycles = DWT->CYCCNT - section_start_cycle;
    __set_PRIMASK(primask);
}

// 作業バッファへtimecodeを合成し、上書きしたchを記録する（stream境界での無音化に使う）。
static void audio_tx_render_timecode_stage(uint32_t half_words)
{
    uint32_t mask = 0u;
    for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
    {
        if (timecode_synth_is_channel_enabled(channel))
        {
            mask |= (1u << channel);
        }
    }
    s_tx_stage_timecode_mask = mask;
    timecode_synth_render_output(s_tx_stage_buf, AUDIO_RING_FRAME_WORDS, half_words / AUDIO_RING_FRAME_WORDS);
}

static void audio_tx_fill_silence(uint32_t half_words)
{
    memset(s_tx_stage_buf, 0, half_words * sizeof(int32_t));
    audio_tx_render_timecode_stage(half_words);
}

// 作業バッファのUSB OUT由来の音声を無音にし、timecodeが上書きしたchだけを残す。
// audio_tx_fill_silence()と同じ出力になる（timecodeを再合成しない）。
static void audio_tx_stage_strip_stream_audio(uint32_t half_words)
{
    for (uint32_t i = 0u; i < half_words; i += AUDIO_RING_FRAME_WORDS)
    {
        for (uint32_t channel = 0u; channel < TIMECODE_SYNTH_CHANNEL_COUNT; channel++)
        {
            if ((s_tx_stage_timecode_mask & (1u << channel)) == 0u)
            {
                s_tx_stage_buf[i + (channel * 2u)]      = 0;
                s_tx_stage_buf[i + (channel * 2u) + 1u] = 0;
            }
        }
    }
    s_tx_stage_has_stream_audio = false;
}

// 1 half分の出力をs_tx_stage_bufへ作る。DMA共有領域には触れない（commitは
// audio_tx_commit_half()）。分岐・FIFOの読出し量・診断・timecode合成は#260と同じ。
static void audio_tx_build_stage(uint32_t sample_rate_hz)
{
    const uint32_t half_words  = s_sai_dma_half_words;
    const uint32_t frame_words = AUDIO_RING_FRAME_WORDS;
    uint32_t diagnostic_event_flags = 0u;
    const bool streaming       = s_streaming_out;

    int32_t* const dst = s_tx_stage_buf;
    s_tx_stage_has_stream_audio = false;

    // OUT停止中（またはFIFO未構成）は無音。primingはOUT開始境界でリセット済み。
    tu_fifo_t* ep_out_ff = streaming ? tud_audio_n_get_ep_out_ff(AUDIO_FUNC_ID) : NULL;
    if (ep_out_ff == NULL)
    {
        audio_tx_fill_silence(half_words);
        return;
    }

    // 判定はすべて、ここで取得した読出し直前の水位で行う。区間前に水位が減るのは
    // usbTaskのclearだけで、それは区間内で検出する。
    const uint32_t level_words = audio_words_floor_frame(tu_fifo_count(ep_out_ff) / sizeof(int32_t));
    audio_diagnostics_record_tx_level(streaming, (int32_t) level_words);
#if AUDIO_DIAG_LOG
    audio_diagnostics_record_tx_interval_level((int32_t) level_words);
#endif

    audio_tx_out_levels_t levels;
    audio_tx_out_levels(sample_rate_hz, &levels);

    // USB実データの充填待ち（priming）。FIFOは消費せず無音を出す。
    if (!s_tx_primed)
    {
        if (level_words >= levels.prime_words)
        {
            s_tx_primed = true;
            audio_diagnostics_record_tx_priming_complete();
        }
        else
        {
            audio_diagnostics_record_tx_priming_wait();
            audio_tx_fill_silence(half_words);
            return;
        }
    }

    uint32_t peek_words    = half_words;
    uint32_t consume_words = half_words;
    int8_t drift_direction = 0;
    const uint32_t now_ms  = HAL_GetTick();

    if (level_words < half_words)
    {
        diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_UNDERRUN;
        // 未実施の補正履歴を残さないよう先に解除する。
        audio_tx_drift_history_clear();
        if (level_words == 0u)
        {
            // FIFOが空。無音を出してprimingへ戻し、priming水位から再開する。
            // 空のまま再生を続けると、feedbackが水位を戻すまで毎halfが部分埋めになるため。
            s_tx_primed = false;
            audio_diagnostics_record_out_reprime();
            audio_diagnostics_record_tx_event(streaming, diagnostic_event_flags, 0);
            audio_tx_fill_silence(half_words);
            return;
        }
        // 読める分だけ再生し、残りは末尾フレーム保持で埋める（クリック感を抑える）。
        peek_words    = level_words;
        consume_words = level_words;
    }
    else
    {
        // 長時間再生時のUSB/SAIクロック差のうち、feedbackで吸収されない分（feedbackに
        // 従わないホスト）に対して1 frameだけ消費量を増減する。
        drift_direction = audio_tx_drift_update(level_words, &levels, now_ms);
        if ((drift_direction > 0) && (level_words >= (half_words + frame_words)))
        {
            // FIFO過多: 1 frame余分に消費し、half内のクロスフェードでつなぐ。
            peek_words    = half_words + frame_words;
            consume_words = half_words + frame_words;
            diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_DRIFT_UP;
        }
        else if (drift_direction < 0)
        {
            // FIFO不足傾向: 1 frame少なく消費し、クロスフェードで引き伸ばす。
            // 補間に使う最後のframeはpeekだけで消費しない。
            peek_words    = half_words;
            consume_words = half_words - frame_words;
            diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_DRIFT_DOWN;
        }
        else
        {
            drift_direction = 0;
        }
    }

    // 通常は読出し先をDMA halfにしてコピーを1回にする。補正時は作業バッファへpeekする。
    int32_t* const take_dst = (drift_direction != 0) ? s_usb_playback_buf : dst;
    audio_usb_out_take_outcome_t outcome;
    audio_usb_out_take(ep_out_ff,
                       take_dst,
                       (uint16_t) (peek_words * sizeof(int32_t)),
                       (uint16_t) (consume_words * sizeof(int32_t)),
                       &outcome);

    // ---- 以降はPRIMASK区間の外。FIFOには触れず、区間内で取得した値だけを使う。
    audio_diagnostics_record_out_take_section(outcome.section_cycles);

    if (outcome.result == AUDIO_USB_OUT_TAKE_STALE)
    {
        // 次の要求適用で停止か開始境界（primingからやり直し）になる。開き直しで最終maskが
        // 有効のままでも、OUT開始世代により開始境界として適用される。
        audio_diagnostics_record_out_stale_request_skip();
        audio_tx_fill_silence(half_words);
        return;
    }
    if (outcome.result == AUDIO_USB_OUT_TAKE_OVERFLOW)
    {
        // 保証範囲外の復旧。古いFIFO内容は救済せず、無音を出してprimingへ戻す。
        s_tx_primed = false;
        audio_tx_drift_history_clear();
        audio_diagnostics_record_out_fifo_overflow_recovery(outcome.raw_count_bytes, outcome.depth_bytes);
        audio_tx_fill_silence(half_words);
        return;
    }

    if (drift_direction != 0)
    {
        // 実際に補正を実施した場合だけ頻度制限の時刻を更新する。
        audio_tx_drift_commit(now_ms);

        // 補正half: 先頭の非補間部は通常コピーし、末尾BLEND_FRAMESだけ
        // 1 frame分ずらした2ソースを線形クロスフェードする。出力は常にhalf_words word。
        const uint32_t copy_frames  = half_words / frame_words;
        const uint32_t blend_frames = AUDIO_TX_DRIFT_BLEND_FRAMES;
        const uint32_t plain_words  = (copy_frames - blend_frames) * frame_words;

        memcpy(dst, s_usb_playback_buf, plain_words * sizeof(int32_t));

        for (uint32_t j = 0; j < blend_frames; j++)
        {
            const uint32_t frame_index = (plain_words / frame_words) + j;
            const int32_t* frame_a = s_usb_playback_buf + (frame_index * frame_words);
            const uint32_t other_index = (drift_direction > 0) ? (frame_index + 1u) : (frame_index - 1u);
            const int32_t* frame_b = s_usb_playback_buf + (other_index * frame_words);

            // 4chすべて同じ位置・同じ係数でクロスフェードする（ch間を混ぜない）。
            // 中間値はint64で計算し、丸めは0から遠い側へ寄せる。24bit-in-32bitの
            // 表現に依存せず、係数が0..blend_framesの凸結合なので結果は必ず
            // 2入力の範囲内に収まり、飽和は不要。
            const uint32_t blend_weight = j + 1u;
            int32_t* out = dst + plain_words + (j * frame_words);
            for (uint32_t c = 0; c < frame_words; c++)
            {
                int64_t blended =
                    ((int64_t) frame_a[c] * (int64_t) (blend_frames - blend_weight)) +
                    ((int64_t) frame_b[c] * (int64_t) blend_weight);
                if (blended >= 0)
                {
                    blended += (int64_t) (blend_frames / 2u);
                }
                else
                {
                    blended -= (int64_t) (blend_frames / 2u);
                }
                out[c] = (int32_t) (blended / (int64_t) blend_frames);
            }
        }
    }
    else if (consume_words < half_words)
    {
        diagnostic_event_flags |= AUDIO_TX_DIAG_EVENT_PARTIAL_FILL;
        // 不足分は最後の1frameを繰り返し、クリックノイズを抑える
        uint32_t* fill = (uint32_t*) (dst + consume_words);
        const uint32_t* last = (const uint32_t*) (dst + consume_words - frame_words);
        for (uint32_t i = consume_words; i < half_words; i += frame_words)
        {
            fill[0] = last[0];
            fill[1] = last[1];
            fill[2] = last[2];
            fill[3] = last[3];
            fill += frame_words;
        }
    }

    audio_diagnostics_record_tx_level(streaming, (int32_t) (level_words - consume_words));
    if (diagnostic_event_flags != 0u)
    {
        audio_diagnostics_record_tx_event(streaming, diagnostic_event_flags, (int32_t) level_words);
    }
    audio_tx_render_timecode_stage(half_words);
    s_tx_stage_has_stream_audio = true;
}

// 作業バッファをDMAの対象halfへcommitする。Audio Task context only。
// 同じPRIMASK区間で、作業バッファを作った後にpublishされたstream要求を確認し、
// 旧streamの音声はDMAへ書かない（無音とtimecodeへ置き換えてからcommitし直す）。
// applied側のsequenceはAudio Taskだけが進めるので、作業バッファを作ってから
// ここまでに新しい要求がpublishされれば、必ず不一致として検出できる。
static void audio_tx_commit_half(uint32_t dst_half_offset_words, uint32_t sample_rate_hz)
{
    const uint32_t half_words = s_sai_dma_half_words;
    const uint32_t half_index = (dst_half_offset_words == 0u) ? 0u : 1u;
    audio_dma_own_outcome_t outcome = {0};

    for (uint32_t attempt = 0u; attempt < 2u; attempt++)
    {
#if AUDIO_DMA_OWNERSHIP_TEST
        const bool injected = audio_dma_test_before(true, &handle_GPDMA1_Channel2, half_index, half_words,
                                                    sample_rate_hz);
#endif
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        const bool stream_boundary =
            s_tx_stage_has_stream_audio &&
            ((s_stream_request_sequence != s_stream_applied_request_sequence) || !s_streaming_out);
        if (!stream_boundary)
        {
            audio_dma_owned_copy(&handle_GPDMA1_Channel2, &s_tx_dma_suspend_fault, true,
                                 half_index, half_words,
                                 sai_tx_dma_buf + dst_half_offset_words, s_tx_stage_buf, &outcome);
        }

        __set_PRIMASK(primask);

#if AUDIO_DMA_OWNERSHIP_TEST
        if (injected && !stream_boundary)
        {
            audio_dma_test_trace(true, half_index, half_words, &outcome);
        }
#endif
        if (!stream_boundary)
        {
            break;
        }

        // 次の要求適用で停止か開始境界（primingからやり直し）になる。
        audio_tx_stage_strip_stream_audio(half_words);
        audio_diagnostics_record_tx_dma_stream_boundary();
    }

    audio_diagnostics_record_tx_dma_ownership(outcome.result,
                                              outcome.q,
                                              outcome.suspend_cycles,
                                              audio_dma_suspend_over_budget(outcome.suspend_cycles, sample_rate_hz),
                                              outcome.stale_flag_cleared,
                                              outcome.fault);
    if (outcome.fault)
    {
        recovery_request_publish_from_task(AUDIO_RECOVERY_CAUSE_TX_DMA);
    }
}

// 1 half分を作業バッファへ作り、DMAの対象halfへcommitする。期限逸脱（skip）では
// 対象halfへ書かず、FIFOから消費した分・timecodeで取り出した分は戻さない。
static void fill_tx_half(uint32_t dst_half_offset_words, uint32_t sample_rate_hz)
{
    // dst_half_offset_wordsの範囲チェック（有効なDMA転送長の内側だけを更新する）
    if (dst_half_offset_words > (audio_transport_sai_dma_xfer_words() - s_sai_dma_half_words))
    {
        return;
    }

    audio_tx_build_stage(sample_rate_hz);
    audio_tx_commit_half(dst_half_offset_words, sample_rate_hz);
}

// DMA half期間（4ch frame単位）をSystemCoreClockサイクルへ換算する。
// 48/96kHzとも約0.333ms（実行時のhalf長による）。sample_rate_hzが不正なら期限なし。
static uint32_t audio_dma_half_deadline_cycles(uint32_t half_words, uint32_t sample_rate_hz)
{
    if ((sample_rate_hz == 0u) || (SystemCoreClock == 0u))
    {
        return UINT32_MAX;
    }

    const uint32_t half_frames = half_words / AUDIO_RING_FRAME_WORDS;
    return (uint32_t) (((uint64_t) half_frames * SystemCoreClock) / sample_rate_hz);
}

static void copy_usb_out_to_sai_dma(uint32_t sample_rate_hz)
{
    audio_transport_dma_event_snapshot_t event;
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
            audio_dma_half_deadline_cycles(s_sai_dma_half_words, sample_rate_hz);
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
            audio_dma_half_deadline_cycles(s_sai_dma_half_words, sample_rate_hz);
    }

    // 遅延時は古い要求を処理しない。最新コールバックが示すhalfを対象にし、書込みは
    // GPDMA一時停止中に所有権を確かめてから行う（audio_tx_commit_half()）。
    if (event.event == DMA_AUDIO_EVENT_HALF)
    {
        fill_tx_half(0, sample_rate_hz);
    }
    else if (event.event == DMA_AUDIO_EVENT_COMPLETE)
    {
        fill_tx_half(s_sai_dma_half_words, sample_rate_hz);
    }

    // half更新完了時点を境界とし、FIFO読出し・補間・合成を含む完了時間を計測する。
    const uint32_t end_cycle = DWT->CYCCNT;
    audio_diagnostics_record_tx_dma_complete(event.event,
                                             end_cycle - process_start_cycle,
                                             end_cycle - event.cycle,
                                             deadline_cycles);
}

// ==============================
// SAI(RX) -> USB(IN) path
// ==============================
// not_readyのやり直しの前に、停止せずに読んだ位置で、対象halfの末尾の宛先書込みが
// 終わる位置（q ≧ FIFO）までDMAが進むのを待つ。上限はAUDIO_DMA_RX_NOT_READY_WAIT_FRAMES。
// DMA共有領域には触れない。待ったサイクル数を返す。
static uint32_t audio_rx_wait_not_ready(uint32_t half_index, uint32_t half_words, uint32_t sample_rate_hz)
{
    const uint32_t start_cycle = DWT->CYCCNT;
    const uint32_t limit_cycles =
        (sample_rate_hz != 0u) ? ((SystemCoreClock / sample_rate_hz) * AUDIO_DMA_RX_NOT_READY_WAIT_FRAMES) : 0u;
    DMA_Channel_TypeDef* const ch = handle_GPDMA1_Channel3.Instance;

    for (;;)
    {
        uint32_t pos;
        if (!audio_dma_position_words(audio_dma_reg_read(&ch->CBR1) & DMA_CBR1_BNDT, half_words * 2u, &pos) ||
            (audio_dma_offset_from_other_half(pos, half_index, half_words) >= AUDIO_DMA_FIFO_WORDS))
        {
            break;
        }
        if ((DWT->CYCCNT - start_cycle) > limit_cycles)
        {
            break;
        }
    }

    return DWT->CYCCNT - start_cycle;
}

// snapshot 1回分の結果を記録する。停止状態異常を新たに検出した場合は復旧を要求する。
static void audio_rx_record_ownership(const audio_dma_own_outcome_t* outcome, uint32_t sample_rate_hz)
{
    audio_diagnostics_record_rx_dma_ownership(outcome->result,
                                              outcome->q,
                                              outcome->suspend_cycles,
                                              audio_dma_suspend_over_budget(outcome->suspend_cycles, sample_rate_hz),
                                              outcome->stale_flag_cleared,
                                              outcome->fault);
    if (outcome->fault)
    {
        recovery_request_publish_from_task(AUDIO_RECOVERY_CAUSE_RX_DMA);
    }
}

// DMAの対象halfをs_rx_stage_bufへsnapshotする。Audio Task context only。
// 所有権を確認できなかった場合（reject・位置不明・停止タイムアウト・停止状態異常）は
// 作業バッファを0にし、1チャンクの無音として扱う。
static void audio_rx_snapshot_half(uint32_t src_half_offset_words, uint32_t sample_rate_hz)
{
    const uint32_t half_words = s_sai_dma_half_words;
    const uint32_t half_index = (src_half_offset_words == 0u) ? 0u : 1u;
    audio_dma_own_outcome_t outcome = {0};

    for (uint32_t attempt = 0u; attempt < 2u; attempt++)
    {
#if AUDIO_DMA_OWNERSHIP_TEST
        const bool injected = audio_dma_test_before(false, &handle_GPDMA1_Channel3, half_index, half_words,
                                                    sample_rate_hz);
#endif
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        audio_dma_owned_copy(&handle_GPDMA1_Channel3, &s_rx_dma_suspend_fault, false,
                             half_index, half_words,
                             s_rx_stage_buf, sai_rx_dma_buf + src_half_offset_words, &outcome);

        __set_PRIMASK(primask);

#if AUDIO_DMA_OWNERSHIP_TEST
        if (injected)
        {
            audio_dma_test_trace(false, half_index, half_words, &outcome);
        }
#endif
        if (outcome.result != AUDIO_DIAG_DMA_OWN_NOT_READY)
        {
            break;
        }
        if (attempt != 0u)
        {
            // やり直しても宛先書込みの完了を確かめられない。
            outcome.result = AUDIO_DIAG_DMA_OWN_UNKNOWN;
            break;
        }

        // 1回目のNOT_READYも停止・解除を行っているため、その結果を記録してから待つ。
        audio_rx_record_ownership(&outcome, sample_rate_hz);
        audio_diagnostics_record_rx_dma_not_ready(audio_rx_wait_not_ready(half_index, half_words, sample_rate_hz));
    }

    audio_rx_record_ownership(&outcome, sample_rate_hz);

    if (outcome.result != AUDIO_DIAG_DMA_OWN_OK)
    {
        memset(s_rx_stage_buf, 0, half_words * sizeof(int32_t));
    }
}

static void fill_rx_half(uint32_t src_half_offset_words, bool streaming, uint32_t sample_rate_hz)
{
    const uint32_t half_words = s_sai_dma_half_words;  // 実行時のDMA half長（word数）

    // src_half_offset_wordsの範囲チェック（有効なDMA転送長の内側だけを取り込む）
    if (src_half_offset_words > (audio_transport_sai_dma_xfer_words() - half_words))
    {
        return;
    }

    // DMA共有領域へのアクセスはこのsnapshotだけ。以降は作業バッファを読む。
    audio_rx_snapshot_half(src_half_offset_words, sample_rate_hz);

    timecode_synth_process_input(s_rx_stage_buf,
                                 AUDIO_RING_FRAME_WORDS,
                                 half_words / AUDIO_RING_FRAME_WORDS);

    if (!streaming)
    {
        // USB IN停止中は送信先がないため、IN FIFOへ書かない。
        // timecode入力処理は上で継続する。開始境界はaudio_stream_begin_in()が作る。
        return;
    }

    // snapshotしたhalfをそのまま1チャンクとしてIN FIFOへ書き込む。
    copy_rx_half_to_usb_in(s_rx_stage_buf,
                           half_words / AUDIO_RING_FRAME_WORDS,
                           sample_rate_hz);
}

static void copy_sai_rx_dma_to_usb_in(uint32_t sample_rate_hz)
{
    audio_transport_dma_event_snapshot_t event;
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
            audio_dma_half_deadline_cycles(s_sai_dma_half_words, sample_rate_hz);
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
            audio_dma_half_deadline_cycles(s_sai_dma_half_words, sample_rate_hz);
    }

    // TXと同様に、最新コールバックが示すhalfを対象にし、読出しはGPDMA一時停止中に
    // 所有権を確かめてから行う（audio_rx_snapshot_half()）。
    if (event.event == DMA_AUDIO_EVENT_HALF)
    {
        fill_rx_half(0, s_streaming_in, sample_rate_hz);
    }
    else if (event.event == DMA_AUDIO_EVENT_COMPLETE)
    {
        fill_rx_half(s_sai_dma_half_words, s_streaming_in, sample_rate_hz);
    }

    // half取り込み完了時点を境界とし、synth処理とIN FIFO書込みを含む完了時間を計測する。
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

// USB IN FIFOへの書込み上限。EP IN interval 4回分 = 2.0 ms相当（暫定値）。
static uint16_t audio_transport_usb_in_fifo_limit_bytes(uint32_t sample_rate_hz)
{
    const uint64_t limit_frames =
        (uint64_t) audio_frames_per_usb_in_interval(sample_rate_hz) *
        AUDIO_USB_IN_FIFO_LIMIT_INTERVALS;

    return audio_usb_fifo_target_bytes_clamp(limit_frames * AUDIO_USB_FRAME_BYTES,
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

// レート未確定・失敗期間のUSB OUT残存データを破棄する。OUT FIFOだけをPRIMASK区間で
// clearし、USB ISRの書込みとusbTaskのclearの両方と排他する。IN FIFOには触れない。
// Audio Task context only。
static void audio_transport_discard_usb_out(void)
{
    if (!tud_inited() || !tud_audio_n_mounted(AUDIO_FUNC_ID))
    {
        return;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    (void) tud_audio_n_clear_ep_out_ff(AUDIO_FUNC_ID);
    __set_PRIMASK(primask);
}

typedef enum
{
    AUDIO_USB_IN_WRITE_OK = 0,         // 書込み成功（上限超過時はFIFOの古いデータをtrim済み）
    AUDIO_USB_IN_WRITE_DROPPED,        // trim後も空き不足。上書きさせずにチャンクを破棄（防御処理）
    AUDIO_USB_IN_WRITE_STALE_REQUEST,  // 未適用のstream要求あり。旧世代データを書かない
    AUDIO_USB_IN_WRITE_ZERO,           // tud_audio_n_write()が0（未構成時のみ）
    AUDIO_USB_IN_WRITE_PARTIAL,        // 要求量と異なる非0値（現行TinyUSBでは発生しない）
} audio_usb_in_write_result_t;

// scheduler停止区間内で取得した値。resume後はこの値だけを使う。
typedef struct
{
    audio_usb_in_write_result_t result;
    uint16_t judged_count;    // 最終判定時のFIFO水位(byte、trim前)。STALE_REQUESTでは0
    uint16_t trimmed_bytes;   // 上限超過で破棄したFIFOの古いデータ(byte)。trimなしは0
    uint16_t written_bytes;   // tud_audio_n_write()の戻り値。書込みなしは0
    uint32_t section_cycles;  // 区間のサイクル数(DWT->CYCCNT)
} audio_usb_in_write_outcome_t;

// IN FIFOへの最終判定とwrite。Audio Task context only。
// usbTaskのSET_INTERFACE処理（IN FIFO clearと要求publish）がwrite途中に割り込むと、
// tu_fifoのwriteがclear前のindexを公開し得る（tusb_fifo.c、FIFOにmutexなし）。
// vTaskSuspendAll()でusbTaskだけを止め、USB ISRの読出し（SPSCとして安全）は継続させる。
// usbTaskはclearから要求publishまでブロックしないため、clearが起きていれば区間内で
// 要求sequenceの不一致として必ず検出できる。
// 区間内では待機API・ログ・HALを呼ばず、途中returnせずに全経路でxTaskResumeAll()を
// 1回だけ通る。
//
// 水位＋チャンクが上限を超える場合（ホストが読まない期間）は、新しいチャンクではなく
// FIFO先頭の古いデータを捨て、判定時水位＋チャンクが目標になるようにする。
// 読出し側（rd_idx）はUSB ISRも更新するため、trimはPRIMASK区間で行う。
// DWC2 DMAモード（CFG_TUD_EDPT_DEDICATED_HWFIFO=0）ではIN FIFOの読出しはUSB ISRの
// tu_fifo_read_n()だけであり、PRIMASK区間で排他できる。PRIMASK解除後からwriteまでの
// 間もISRは読み出せるため、write後の実水位は目標未満になり得る（保証するのは上限以下）。
static void audio_usb_in_commit_chunk(tu_fifo_t* ep_in_ff,
                                      uint16_t usb_bytes,
                                      uint16_t target_bytes,
                                      uint16_t limit_bytes,
                                      audio_usb_in_write_outcome_t* outcome)
{
    outcome->judged_count  = 0u;
    outcome->trimmed_bytes = 0u;
    outcome->written_bytes = 0u;

    vTaskSuspendAll();
    const uint32_t section_start_cycle = DWT->CYCCNT;

    if ((s_stream_request_sequence != s_stream_applied_request_sequence) || !s_streaming_in)
    {
        outcome->result = AUDIO_USB_IN_WRITE_STALE_REQUEST;
    }
    else
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        const uint16_t fifo_count = tu_fifo_count(ep_in_ff);
        if (((uint32_t) fifo_count + usb_bytes) > limit_bytes)
        {
            // target < limit のため trim_bytes > 0。frame単位へ切り上げ、水位でクランプする。
            uint32_t trim_bytes = (uint32_t) fifo_count + usb_bytes - target_bytes;
            trim_bytes = ((trim_bytes + AUDIO_USB_FRAME_BYTES - 1u) / AUDIO_USB_FRAME_BYTES) *
                         AUDIO_USB_FRAME_BYTES;
            if (trim_bytes > fifo_count)
            {
                trim_bytes = fifo_count;
            }
            outcome->trimmed_bytes = tu_fifo_discard_n(ep_in_ff, (uint16_t) trim_bytes);
        }

        __set_PRIMASK(primask);

        outcome->judged_count = fifo_count;
        // ISRの読出しは空きを増やす方向にだけ働くため、PRIMASK解除後の値で判定してよい。
        const uint16_t fifo_remaining = tu_fifo_remaining(ep_in_ff);

        if (fifo_remaining < usb_bytes)
        {
            outcome->result = AUDIO_USB_IN_WRITE_DROPPED;
        }
        else
        {
            const uint16_t written_bytes = tud_audio_n_write(AUDIO_FUNC_ID, s_usb_capture_buf, usb_bytes);
            outcome->written_bytes = written_bytes;
            if (written_bytes == usb_bytes)
            {
                outcome->result = AUDIO_USB_IN_WRITE_OK;
            }
            else if (written_bytes == 0u)
            {
                outcome->result = AUDIO_USB_IN_WRITE_ZERO;
            }
            else
            {
                outcome->result = AUDIO_USB_IN_WRITE_PARTIAL;
            }
        }
    }

    outcome->section_cycles = DWT->CYCCNT - section_start_cycle;
    (void) xTaskResumeAll();
}

// DMA half 1回分（frames）を1チャンクとしてIN FIFOへ書き込む。Audio Task context only。
// srcはsnapshot済みの作業バッファ（s_rx_stage_buf）。読むだけで書き換えない。
static void copy_rx_half_to_usb_in(const int32_t* src, uint32_t frames, uint32_t sample_rate_hz)
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

    // USBは4ch、SAIも4ch
    // SAI: [L1][R1][L2][R2][L1][R1][L2][R2]...
    // USB: [L1][R1][L2][R2][L1][R1][L2][R2]...
    const uint32_t usb_bytes = frames * AUDIO_USB_FRAME_BYTES;  // 24bit in 32bit slot、4ch分

    // 安全: s_usb_capture_buf が足りない想定なら絶対に書かない
    if (usb_bytes > sizeof(s_usb_capture_buf))
        return;

    const bool send_ch1_to_usb = !timecode_synth_is_channel_enabled(0u);
    const bool send_ch2_to_usb = !timecode_synth_is_channel_enabled(1u);

    for (uint32_t f = 0; f < frames; f++)
    {
        const int32_t* frame = src + f * AUDIO_RING_FRAME_WORDS;
        s_usb_capture_buf[f * AUDIO_USB_FRAME_CHANNELS + 0] = send_ch1_to_usb ? frame[0] : 0;  // L1
        s_usb_capture_buf[f * AUDIO_USB_FRAME_CHANNELS + 1] = send_ch1_to_usb ? frame[1] : 0;  // R1
        s_usb_capture_buf[f * AUDIO_USB_FRAME_CHANNELS + 2] = send_ch2_to_usb ? frame[2] : 0;  // L2
        s_usb_capture_buf[f * AUDIO_USB_FRAME_CHANNELS + 3] = send_ch2_to_usb ? frame[3] : 0;  // R2
    }

    const uint16_t target_bytes = audio_transport_usb_in_fifo_target_bytes(sample_rate_hz);
    const uint16_t limit_bytes  = audio_transport_usb_in_fifo_limit_bytes(sample_rate_hz);

    audio_usb_in_write_outcome_t outcome;
    audio_usb_in_commit_chunk(ep_in_ff, (uint16_t) usb_bytes, target_bytes, limit_bytes, &outcome);

    // ---- 以降はscheduler停止区間の外。resume直後にusbTaskがFIFOをclearし得るため、
    // 診断には区間内で取得したoutcomeの値だけを使い、FIFO水位を再取得しない。
    audio_diagnostics_record_usb_in_write_section(outcome.section_cycles);

#if AUDIO_DIAG_LOG
    if ((outcome.result != AUDIO_USB_IN_WRITE_STALE_REQUEST) &&
        (outcome.result != AUDIO_USB_IN_WRITE_DROPPED))
    {
        audio_diagnostics_record_usb_in_write(outcome.written_bytes, (uint16_t) usb_bytes);
    }
#endif

    if (outcome.trimmed_bytes != 0u)
    {
        audio_diagnostics_record_usb_in_fifo_trim(outcome.trimmed_bytes);
    }

    // write直後の水位の上界（ISRの読出しは考慮しない）。
    const uint32_t post_trim_count = (uint32_t) outcome.judged_count - outcome.trimmed_bytes;

    switch (outcome.result)
    {
        case AUDIO_USB_IN_WRITE_OK:
            audio_diagnostics_record_usb_in_fifo_level(outcome.judged_count,
                                                       post_trim_count + outcome.written_bytes);
            break;

        case AUDIO_USB_IN_WRITE_DROPPED:
            // 上書きさせない。リングがないため保留せず、チャンクを破棄して計数する。
            audio_diagnostics_record_usb_in_fifo_level(outcome.judged_count, 0u);
            audio_diagnostics_record_usb_in_fifo_full_drop();
            audio_diagnostics_record_usb_in_chunk_drop(usb_bytes);
            break;

        case AUDIO_USB_IN_WRITE_STALE_REQUEST:
            // 組立て中に新しいstream要求がpublishされた。次の適用で停止か開始境界になるため、
            // 旧世代のチャンクは書かずに破棄する。
            audio_diagnostics_record_usb_in_stale_request_skip();
            audio_diagnostics_record_usb_in_chunk_drop(usb_bytes);
            break;

        case AUDIO_USB_IN_WRITE_ZERO:
            // 未構成（p_desc == NULL）時のみ。
            audio_diagnostics_record_usb_in_fifo_level(outcome.judged_count, 0u);
            audio_diagnostics_record_usb_in_write_error(false);
            audio_diagnostics_record_usb_in_chunk_drop(usb_bytes);
            break;

        case AUDIO_USB_IN_WRITE_PARTIAL:
        default:
        {
            // 防御処理。現行TinyUSB（上書き可能FIFO）では 0 < n < depth で常にnを返すため
            // 発生しない。frame途中で途切れるとホスト側でch順序がずれるため成功扱いせず、
            // 公開API経由のIN FIFO clearで再同期する。TinyUSBが転送予約に使用している
            // 領域（lin_buf_in等）には触れない。
            audio_diagnostics_record_usb_in_fifo_level(outcome.judged_count,
                                                       post_trim_count + outcome.written_bytes);
            audio_diagnostics_record_usb_in_write_error(true);
            audio_diagnostics_record_usb_in_chunk_drop(usb_bytes);

            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            (void) tud_audio_n_clear_ep_in_ff(AUDIO_FUNC_ID);
            __set_PRIMASK(primask);
            break;
        }
    }
}

// TinyUSB TX完了コールバック - USB ISRコンテキストで呼ばれる
// USB INへの書込みはRX DMAイベントを契機に行うため、ここではTaskを起こさず診断だけを行う。
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
    audio_diagnostics_record_usb_out_packet(n_bytes_received, DWT->CYCCNT);
#else
    (void) n_bytes_received;
#endif

    tu_fifo_t* ep_out_ff = tud_audio_n_get_ep_out_ff(AUDIO_FUNC_ID);
    if (ep_out_ff != NULL)
    {
#if AUDIO_DIAG_LOG
        audio_diagnostics_record_usb_out_fifo(tu_fifo_count(ep_out_ff));
#endif
        // 書込み後に満杯なら、以後のpacketで上書きが起こり得る（保証範囲外）。
        if (s_streaming_out && tu_fifo_full(ep_out_ff))
        {
            audio_diagnostics_record_out_fifo_full_event();
        }
    }

    // アプリ側feedback更新。搬送禁止中は水位不足による補正をしない。
    // USB OUTの読出しはTX DMAイベントを契機に行うため、Audio Taskは起こさない。
    audio_usb_control_feedback_update();
    return true;
}

void audio_transport_service(uint32_t sample_rate_hz)
{
    // レート未確定（初期適用失敗・切替中・FAILED）の間、およびDMA復旧が停止確認を
    // 経ていない間は通常搬送を許可しない。USB OUTは破棄し、SAI DMA・USB IN
    // へは書かず、参照バッファにも触れない。
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

    // USB OUT FIFO -> SAI (TX側の安全なhalfをFIFOから直接埋める)
    copy_usb_out_to_sai_dma(sample_rate_hz);

    // SAI -> USB (RX側の安全なhalfをIN FIFOへ直接書き込む)
    copy_sai_rx_dma_to_usb_in(sample_rate_hz);

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
    return audio_tx_used_words();
}
