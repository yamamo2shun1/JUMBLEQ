/*
 * audio_control.c
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#include "audio_control.h"
#include "ui_control_internal.h"

#include "adc.h"
#include "gpdma.h"
#include "hpdma.h"
#include "i2c.h"
#include "linked_list.h"
#include "sai.h"
#include "sai.h"

#include "FreeRTOS.h"  // for xPortGetFreeHeapSize
#include "cmsis_os2.h"

#include "ak4619.h"
#include "adau1466.h"
#include "SigmaStudioFW.h"

#define N_SAMPLE_RATES TU_ARRAY_SIZE(sample_rates)
#define AUDIO_DIAG_LOG 0

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
static volatile uint16_t dbg_usb_read_size_min      = 0xFFFFu;
static volatile uint16_t dbg_usb_read_size_max      = 0u;
#endif

bool s_streaming_out = false;
bool s_streaming_in  = false;

bool is_sr_changed            = false;

const uint32_t sample_rates[] = {48000, 96000};
uint32_t current_sample_rate  = sample_rates[0];

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t usb_out_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t usb_in_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4] = {0};

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t sai_tx_rng_buf[SAI_RNG_BUF_SIZE] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t sai_rx_rng_buf[SAI_RNG_BUF_SIZE] = {0};

__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t stereo_out_buf[SAI_TX_BUF_SIZE] = {0};
__attribute__((section("noncacheable_buffer"), aligned(32))) int32_t stereo_in_buf[SAI_RX_BUF_SIZE]  = {0};

// Speaker data size received in the last frame
uint16_t spk_data_size;
// Resolution per format (24bit only)
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution;

// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];     // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];  // +1 for master channel 0

void control_input_from_usb_gain(uint8_t ch, int16_t db);

void reset_audio_buffer(void)
{
    ui_control_reset_state();

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

uint32_t get_tx_blink_interval_ms(void)
{
    return tx_blink_interval_ms;
}

uint32_t get_rx_blink_interval_ms(void)
{
    return rx_blink_interval_ms;
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
// UAC1 Helper Functions
//--------------------------------------------------------------------+

static bool audio10_set_req_ep(tusb_control_request_t const* p_request, uint8_t* pBuff)
{
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

    switch (ctrlSel)
    {
    case AUDIO10_EP_CTRL_SAMPLING_FREQ:
        if (p_request->bRequest == AUDIO10_CS_REQ_SET_CUR)
        {
            // Request uses 3 bytes
            TU_VERIFY(p_request->wLength == 3);

            current_sample_rate = tu_unaligned_read32(pBuff) & 0x00FFFFFF;
            is_sr_changed       = true;

            TU_LOG2("EP set current freq: %" PRIu32 "\r\n", current_sample_rate);

            return true;
        }
        break;

    // Unknown/Unsupported control
    default:
        TU_BREAKPOINT();
        return false;
    }

    return false;
}

static bool audio10_get_req_ep(uint8_t rhport, tusb_control_request_t const* p_request)
{
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

    switch (ctrlSel)
    {
    case AUDIO10_EP_CTRL_SAMPLING_FREQ:
        if (p_request->bRequest == AUDIO10_CS_REQ_GET_CUR)
        {
            TU_LOG2("EP get current freq\r\n");

            uint8_t freq[3];
            freq[0] = (uint8_t) (current_sample_rate & 0xFF);
            freq[1] = (uint8_t) ((current_sample_rate >> 8) & 0xFF);
            freq[2] = (uint8_t) ((current_sample_rate >> 16) & 0xFF);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, freq, sizeof(freq));
        }
        break;

    // Unknown/Unsupported control
    default:
        TU_BREAKPOINT();
        return false;
    }

    return false;
}

static bool audio10_set_req_entity(tusb_control_request_t const* p_request, uint8_t* pBuff)
{
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel    = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID   = TU_U16_HIGH(p_request->wIndex);

    // If request is for our speaker feature unit
    if (entityID == UAC1_ENTITY_STEREO_OUT_FEATURE_UNIT)
    {
        switch (ctrlSel)
        {
        case AUDIO10_FU_CTRL_MUTE:
            switch (p_request->bRequest)
            {
            case AUDIO10_CS_REQ_SET_CUR:
                // Only 1st form is supported
                TU_VERIFY(p_request->wLength == 1);

                mute[channelNum] = pBuff[0];

                TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
                return true;

            default:
                return false;  // not supported
            }

        case AUDIO10_FU_CTRL_VOLUME:
            switch (p_request->bRequest)
            {
            case AUDIO10_CS_REQ_SET_CUR:
                // Only 1st form is supported
                TU_VERIFY(p_request->wLength == 2);

                volume[channelNum] = (int16_t) tu_unaligned_read16(pBuff) / 256;

                TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
                return true;

            default:
                return false;  // not supported
            }

            // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    return false;
}

static bool audio10_get_req_entity(uint8_t rhport, tusb_control_request_t const* p_request)
{
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel    = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID   = TU_U16_HIGH(p_request->wIndex);

    // If request is for our speaker feature unit
    if (entityID == UAC1_ENTITY_STEREO_OUT_FEATURE_UNIT)
    {
        switch (ctrlSel)
        {
        case AUDIO10_FU_CTRL_MUTE:
            // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
            // There does not exist a range parameter block for mute
            TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute[channelNum], 1);

        case AUDIO10_FU_CTRL_VOLUME:
            switch (p_request->bRequest)
            {
            case AUDIO10_CS_REQ_GET_CUR:
                TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
                {
                    int16_t vol = (int16_t) volume[channelNum];
                    vol         = vol * 256;  // convert to 1/256 dB units
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &vol, sizeof(vol));
                }

            case AUDIO10_CS_REQ_GET_MIN:
                TU_LOG2("    Get Volume min of channel: %u\r\n", channelNum);
                {
                    int16_t min = -90;        // -90 dB
                    min         = min * 256;  // convert to 1/256 dB units
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &min, sizeof(min));
                }

            case AUDIO10_CS_REQ_GET_MAX:
                TU_LOG2("    Get Volume max of channel: %u\r\n", channelNum);
                {
                    int16_t max = 30;         // +30 dB
                    max         = max * 256;  // convert to 1/256 dB units
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &max, sizeof(max));
                }

            case AUDIO10_CS_REQ_GET_RES:
                TU_LOG2("    Get Volume res of channel: %u\r\n", channelNum);
                {
                    int16_t res = 1;          // 1 dB
                    res         = res * 256;  // convert to 1/256 dB units
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &res, sizeof(res));
                }
                // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
            }
            break;

            // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    return false;
}

//--------------------------------------------------------------------+
// UAC2 Helper Functions
//--------------------------------------------------------------------+

#if TUD_OPT_HIGH_SPEED

// Helper for clock get requests
static bool audio20_clock_get_request(uint8_t rhport, audio20_control_request_t const* request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

    if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ)
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
            TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
            for (uint8_t i = 0; i < N_SAMPLE_RATES; i++)
            {
                rangef.subrange[i].bMin = (int32_t) sample_rates[i];
                rangef.subrange[i].bMax = (int32_t) sample_rates[i];
                rangef.subrange[i].bRes = 0;
                TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int) rangef.subrange[i].bMin, (int) rangef.subrange[i].bMax, (int) rangef.subrange[i].bRes);
            }

            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &rangef, sizeof(rangef));
        }
    }
    else if (request->bControlSelector == AUDIO20_CS_CTRL_CLK_VALID && request->bRequest == AUDIO20_CS_REQ_CUR)
    {
        audio20_control_cur_1_t cur_valid = {.bCur = 1};
        TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &cur_valid, sizeof(cur_valid));
    }
    TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
}

// Helper for clock set requests
static bool audio20_clock_set_request(uint8_t rhport, audio20_control_request_t const* request, uint8_t const* buf)
{
    (void) rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_4_t));

        current_sample_rate = (uint32_t) ((audio20_control_cur_4_t const*) buf)->bCur;
        is_sr_changed       = true;

        TU_LOG1("Clock set current freq: %" PRIu32 "\r\n", current_sample_rate);

        return true;
    }
    else
    {
        TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

// Helper for feature unit get requests
static bool audio20_feature_unit_get_request(uint8_t rhport, audio20_control_request_t const* request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT);

    if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE && request->bRequest == AUDIO20_CS_REQ_CUR)
    {
        audio20_control_cur_1_t mute1 = {.bCur = mute[request->bChannelNumber]};
        TU_LOG1("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &mute1, sizeof(mute1));
    }
    else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME)
    {
        if (request->bRequest == AUDIO20_CS_REQ_RANGE)
        {
            audio20_control_range_2_n_t(1) range_vol = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0]   = {.bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256)}
            };
            TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber, range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &range_vol, sizeof(range_vol));
        }
        else if (request->bRequest == AUDIO20_CS_REQ_CUR)
        {
            audio20_control_cur_2_t cur_vol = {.bCur = tu_htole16(volume[request->bChannelNumber])};
            TU_LOG1("Get channel %u volume %d dB\r\n", request->bChannelNumber, cur_vol.bCur / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const*) request, &cur_vol, sizeof(cur_vol));
        }
    }
    TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

// Helper for feature unit set requests
static bool audio20_feature_unit_set_request(uint8_t rhport, audio20_control_request_t const* request, uint8_t const* buf)
{
    (void) rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_1_t));

        mute[request->bChannelNumber] = ((audio20_control_cur_1_t const*) buf)->bCur;

        TU_LOG1("Set channel %d Mute: %d\r\n", request->bChannelNumber, mute[request->bChannelNumber]);

        return true;
    }
    else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME)
    {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_2_t));

        volume[request->bChannelNumber] = ((audio20_control_cur_2_t const*) buf)->bCur;

        TU_LOG1("Set channel %d volume: %d dB\r\n", request->bChannelNumber, volume[request->bChannelNumber] / 256);

        control_input_from_usb_gain(request->bChannelNumber, volume[request->bChannelNumber] / 256);

        return true;
    }
    else
    {
        TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

static bool audio20_get_req_entity(uint8_t rhport, tusb_control_request_t const* p_request)
{
    audio20_control_request_t const* request = (audio20_control_request_t const*) p_request;

    if (request->bEntityID == UAC2_ENTITY_CLOCK)
        return audio20_clock_get_request(rhport, request);
    if (request->bEntityID == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT)
        return audio20_feature_unit_get_request(rhport, request);
    else
    {
        TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
    }
    return false;
}

static bool audio20_set_req_entity(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* buf)
{
    audio20_control_request_t const* request = (audio20_control_request_t const*) p_request;

    if (request->bEntityID == UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT)
        return audio20_feature_unit_set_request(rhport, request, buf);
    if (request->bEntityID == UAC2_ENTITY_CLOCK)
        return audio20_clock_set_request(rhport, request, buf);
    TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n", request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

#endif  // TUD_OPT_HIGH_SPEED

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* pBuff)
{
    (void) rhport;
    (void) pBuff;

    if (tud_audio_version() == 1)
    {
        return audio10_set_req_ep(p_request, pBuff);
    }
    else if (tud_audio_version() == 2)
    {
        // We do not support any requests here
    }

    return false;  // Yet not implemented
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;

    if (tud_audio_version() == 1)
    {
        return audio10_get_req_ep(rhport, p_request);
    }
    else if (tud_audio_version() == 2)
    {
        // We do not support any requests here
    }

    return false;  // Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request)
{
    (void) rhport;

    if (tud_audio_version() == 1)
    {
        return audio10_get_req_entity(rhport, p_request);
#if TUD_OPT_HIGH_SPEED
    }
    else if (tud_audio_version() == 2)
    {
        return audio20_get_req_entity(rhport, p_request);
#endif
    }

    return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* buf)
{
    (void) rhport;

    if (tud_audio_version() == 1)
    {
        return audio10_set_req_entity(p_request, buf);
#if TUD_OPT_HIGH_SPEED
    }
    else if (tud_audio_version() == 2)
    {
        return audio20_set_req_entity(rhport, p_request, buf);
#endif
    }

    return false;
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
        sai_tx_rng_buf_index = 0;
        sai_transmit_index   = 0;
        tx_pending_mask      = 0;  // DMAフラグをクリア
    }

    if (ITF_NUM_AUDIO_STREAMING_STEREO_IN == itf && alt == 0)
    {
        rx_blink_interval_ms = BLINK_MOUNTED;
        s_streaming_in       = false;
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
    }

    if (ITF_NUM_AUDIO_STREAMING_STEREO_IN == itf && alt != 0)
    {
        rx_blink_interval_ms = BLINK_STREAMING;

        s_streaming_in = true;
    }

    // Clear buffer when streaming format is changed
    spk_data_size = 0;
    if (alt != 0)
    {
        current_resolution = resolutions_per_format[alt - 1];
    }

    return true;
}

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t* feedback_param)
{
    (void) func_id;
    (void) alt_itf;

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
#if AUDIO_DIAG_LOG
    if ((tx_pending_mask & 0x01U) != 0U)
    {
        dbg_tx_half_rewrite_events++;
    }
#endif
    tx_pending_mask |= 0x01;
    __DMB();
}
static void dma_sai2_tx_cplt(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
#if AUDIO_DIAG_LOG
    if ((tx_pending_mask & 0x02U) != 0U)
    {
        dbg_tx_cplt_rewrite_events++;
    }
#endif
    tx_pending_mask |= 0x02;
    __DMB();
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
}

static void dma_sai_error(DMA_HandleTypeDef* hdma)
{
    SEGGER_RTT_printf(0, "DMA ERR! code=%08X\n", hdma->ErrorCode);
#if AUDIO_DIAG_LOG
    (void) hdma;
    dbg_dma_err_events++;
#endif
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef* hsai)
{
#if AUDIO_DIAG_LOG
    uint32_t sr = hsai->Instance->SR;
    if (hsai == &hsai_BlockA2)
    {
        dbg_sai_tx_err_events++;
        dbg_sai_tx_last_err = HAL_SAI_GetError(hsai);
        dbg_sai_tx_sr_flags |= (sr & (SAI_xSR_OVRUDR | SAI_xSR_WCKCFG | SAI_xSR_CNRDY | SAI_xSR_AFSDET | SAI_xSR_LFSDET));
    }
    else if (hsai == &hsai_BlockA1)
    {
        dbg_sai_rx_err_events++;
        dbg_sai_rx_last_err = HAL_SAI_GetError(hsai);
        dbg_sai_rx_sr_flags |= (sr & (SAI_xSR_OVRUDR | SAI_xSR_WCKCFG | SAI_xSR_CNRDY | SAI_xSR_AFSDET | SAI_xSR_LFSDET));
    }
#else
    (void) hsai;
#endif
}

void start_sai(void)
{
    // ========================================
    // リングバッファを�Eリフィル�E�無音で初期化！E
    // SAI DMAが開始直後にHalf割り込みを発生させた時、E
    // リングバッファにチE�EタがなぁE��アンダーランになるため、E
    // 無音チE�Eタを事前に投�Eしておく
    // 96kHzではチE�Eタレートが2倍なのでプリフィルめE倍忁E��E
    // ========================================
    uint32_t prefill_size = SAI_TX_BUF_SIZE;
    memset(sai_tx_rng_buf, 0, prefill_size * sizeof(int32_t));
    sai_tx_rng_buf_index = prefill_size;
    sai_transmit_index   = 0;
    tx_pending_mask      = 0;

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
    dbg_usb_read_size_min      = 0xFFFFu;
    dbg_usb_read_size_max      = 0u;
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
    hsai_BlockA2.Instance->CR1 |= SAI_xCR1_DMAEN;  // ↁEここが「DMAリクエスト有効化、E
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
    hsai_BlockA1.Instance->CR1 |= SAI_xCR1_DMAEN;  // ↁEここが「DMAリクエスト有効化、E
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

    // USBは4ch、SAIめEch�E�そのままコピ�E�E�E
    // USB: [L1][R1][L2][R2][L1][R1][L2][R2]...
    // SAI: [L1][R1][L2][R2][L1][R1][L2][R2]...

    uint32_t sai_words;  // SAIに書くword数�E�Ech刁E��E

    if (current_resolution == 16)
    {
        // 16bit: USBチE�Eタはint16_tとして詰まってぁE�� (4ch)
        int16_t* usb_in_buf_16 = (int16_t*) usb_in_buf;
        uint32_t usb_samples   = spk_data_size / sizeof(int16_t);  // 16bitサンプル数
        sai_words              = usb_samples;                      // SAIに書ぁEchチE�Eタのword数

        if ((int32_t) sai_words > free)
        {
            sai_words = (uint32_t) free;
        }

        // 4ch全てコピ�E、E6bitↁE2bit左詰め変換
        for (uint32_t i = 0; i < sai_words; i++)
        {
            int32_t sample32                                              = ((int32_t) usb_in_buf_16[i]) << 16;
            sai_tx_rng_buf[sai_tx_rng_buf_index & (SAI_RNG_BUF_SIZE - 1)] = sample32;
            sai_tx_rng_buf_index++;
        }
    }
    else
    {
        // 24bit in 32bit slot: 4ch全てそ�Eままコピ�E
        sai_words = spk_data_size / sizeof(int32_t);

        if ((int32_t) sai_words > free)
        {
            sai_words = (uint32_t) free;
        }

        // 4ch全てコピ�E
        for (uint32_t i = 0; i < sai_words; i++)
        {
            sai_tx_rng_buf[sai_tx_rng_buf_index & (SAI_RNG_BUF_SIZE - 1)] = usb_in_buf[i];
            sai_tx_rng_buf_index++;
        }
    }

    // SEGGER_RTT_printf(0, " %d\n", sai_tx_rng_buf_index);
}

static inline void fill_tx_half(uint32_t index0)
{
    const uint32_t n           = (SAI_TX_BUF_SIZE / 2);
    const uint32_t frame_words = 4;  // 4ch x 32bit = 1 frame
    uint32_t pull_words        = n;

    // index0のバウンドチェチE��
    if (index0 >= SAI_TX_BUF_SIZE)
    {
        // 不正な値 - 無音で埋めめE
        return;
    }

    int32_t used = (int32_t) (sai_tx_rng_buf_index - sai_transmit_index);
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
        // 同期ズレは捨てて合わせ直ぁE
        sai_transmit_index = sai_tx_rng_buf_index;
        used               = 0;
    }

    // チE�Eタ不足時�E可能な刁E��け�E生し、残りは末尾フレーム保持で埋める、E
    // ぁE��なり�E無音にせずクリチE��感を抑える、E
    if (used < (int32_t) n)
    {
#if AUDIO_DIAG_LOG
        dbg_tx_underrun_events++;
#endif
        if (used <= 0)
        {
            memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
            return;
        }
        pull_words = ((uint32_t) used / frame_words) * frame_words;
        if (pull_words == 0)
        {
            memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
            return;
        }
    }

    // usedが大きすぎる場合も異常�E�オーバ�Eフロー等！E
    if (used > (int32_t) SAI_RNG_BUF_SIZE)
    {
        // リセチE��して無音で埋めめE
        sai_transmit_index = sai_tx_rng_buf_index;
        memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
        return;
    }

    // 長時間再生時�E USB/SAI クロチE��差を吸収するため、E
    // リング水位に応じて 1 frame だけ消費量を増減する、E
    const int32_t target_level = (int32_t) SAI_TX_TARGET_LEVEL_WORDS;
    const int32_t high_thr     = target_level + (int32_t) (SAI_TX_BUF_SIZE / 2);
    const int32_t low_thr      = target_level - (int32_t) (SAI_TX_BUF_SIZE / 2);

    if (used >= (int32_t) n && used > high_thr && used >= (int32_t) (n + frame_words))
    {
        // バッファ過夁E-> 1 frame 余�Eに消費して追征E
        pull_words = n + frame_words;
#if AUDIO_DIAG_LOG
        dbg_tx_drift_up_events++;
#endif
    }
    else if (used >= (int32_t) n && used < low_thr && n > frame_words)
    {
        // バッファ不足傾吁E-> 1 frame 少なく消費して追征E
        pull_words = n - frame_words;
#if AUDIO_DIAG_LOG
        dbg_tx_drift_dn_events++;
#endif
    }

    // 安�EガーチE
    if ((int32_t) pull_words > used)
    {
        pull_words = (uint32_t) used;
    }
    pull_words = (pull_words / frame_words) * frame_words;

    if (pull_words == 0)
    {
        memset(stereo_out_buf + index0, 0, n * sizeof(int32_t));
        return;
    }

    const uint32_t index1 = sai_transmit_index & (SAI_RNG_BUF_SIZE - 1);
    uint32_t first        = SAI_RNG_BUF_SIZE - index1;
    if (first > pull_words)
        first = pull_words;

    memcpy(stereo_out_buf + index0, sai_tx_rng_buf + index1, first * sizeof(int32_t));
    if (first < pull_words)
        memcpy(stereo_out_buf + index0 + first, sai_tx_rng_buf, (pull_words - first) * sizeof(int32_t));

    if (pull_words < n)
    {
#if AUDIO_DIAG_LOG
        dbg_tx_partial_fill_events++;
#endif
        // 不足刁E�E最後�E1frameを繰り返してクリチE��ノイズを抑える
        uint32_t* dst = (uint32_t*) (stereo_out_buf + index0 + pull_words);
        uint32_t* src = (uint32_t*) (stereo_out_buf + index0 + pull_words - frame_words);
        for (uint32_t i = pull_words; i < n; i += frame_words)
        {
            dst[0] = src[0];
            dst[1] = src[1];
            dst[2] = src[2];
            dst[3] = src[3];
            dst += frame_words;
        }
    }

    sai_transmit_index += pull_words;
}

void copybuf_ring2sai(void)
{
    // ISRから立つ「更新要求」を取り出して、該当halfだぁE回更新する
    uint8_t mask;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    mask            = tx_pending_mask;
    tx_pending_mask = 0;
    __set_PRIMASK(primask);

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
    const uint32_t n = (SAI_RX_BUF_SIZE / 2);  // 半�Eぶん！Eord数�E�E

    // index0のバウンドチェチE��
    if (index0 >= SAI_RX_BUF_SIZE)
    {
        return;
    }

    int32_t used = (int32_t) (sai_rx_rng_buf_index - sai_receive_index);
    if (used < 0)
    {
        sai_receive_index = sai_rx_rng_buf_index;
        used              = 0;
    }

    // usedが大きすぎる場合も異常�E�オーバ�Eフロー等！E
    if (used > (int32_t) SAI_RNG_BUF_SIZE)
    {
        sai_receive_index = sai_rx_rng_buf_index;
        used              = 0;
    }

    int32_t free = (int32_t) (SAI_RNG_BUF_SIZE - 1) - used;
    if (free < (int32_t) n)
    {
        // 追ぁE��けなぁE��ら古ぁE��ータを捨てて空きを作る
        sai_receive_index += (uint32_t) ((int32_t) n - free);
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

    // 頁E��：half→cplt の頁E��処琁E��両方溜まってぁE��場合！E
    if (mask & 0x01)
        fill_rx_half(0);
    if (mask & 0x02)
        fill_rx_half(SAI_RX_BUF_SIZE / 2);
}

// 1msあたり�Eフレーム数
static uint32_t audio_frames_per_ms(void)
{
    // 侁E 48kHz -> 48 frames/ms
    return current_sample_rate / 1000u;
}

static uint16_t audio_out_bytes_per_ms(void)
{
    // Host -> Device (speaker OUT) stream bytes per 1ms
    // 48/96kHz are integer frames per ms in this project.
    uint32_t bytes_per_sample = (current_resolution == 16) ? 2u : 4u;
    uint32_t bytes            = audio_frames_per_ms() * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX * bytes_per_sample;
    if (bytes > sizeof(usb_in_buf))
    {
        bytes = sizeof(usb_in_buf);
    }
    return (uint16_t) bytes;
}

static void copybuf_ring2usb_and_send(void)
{
    if (!tud_audio_mounted())
    {
        return;
    }

    // IN(録音)側ぁEstreaming してぁE��ぁE��ら送らなぁE
    if (!s_streaming_in)
    {
        return;
    }

    if (tud_audio_get_ep_in_ff() == NULL)
    {
        return;
    }

    const uint32_t frames    = audio_frames_per_ms();  // 48 or 96 frames/ms
    const uint32_t sai_words = frames * 2;             // SAIは2ch

    int32_t used = (int32_t) (sai_rx_rng_buf_index - sai_receive_index);
    if (used < 0)
    {
        sai_receive_index = sai_rx_rng_buf_index;
        return;
    }
    if (used < (int32_t) sai_words)
    {
        return;  // 足りなぁE��ら今回は送らなぁE
    }

    // USBは4ch、SAIめEch
    // SAI: [L1][R1][L2][R2][L1][R1][L2][R2]...
    // USB: [L1][R1][L2][R2][L1][R1][L2][R2]...

    uint32_t usb_bytes;
    uint16_t written;

    if (current_resolution == 16)
    {
        // 16bit: SAI(32bit 2ch) ↁEUSB(16bit 4ch) 変換
        usb_bytes = frames * 4 * sizeof(int16_t);  // 4ch刁E

        // 安�E: usb_out_buf が足りなぁE��定なら絶対に書かなぁE
        if (usb_bytes > sizeof(usb_out_buf))
            return;

        int16_t* usb_out_buf_16 = (int16_t*) usb_out_buf;

        for (uint32_t f = 0; f < frames; f++)
        {
            uint32_t r_L1 = (sai_receive_index + f * 4 + 0) & (SAI_RNG_BUF_SIZE - 1);
            uint32_t r_R1 = (sai_receive_index + f * 4 + 1) & (SAI_RNG_BUF_SIZE - 1);
            uint32_t r_L2 = (sai_receive_index + f * 4 + 2) & (SAI_RNG_BUF_SIZE - 1);
            uint32_t r_R2 = (sai_receive_index + f * 4 + 3) & (SAI_RNG_BUF_SIZE - 1);
            // 32bit ↁE16bit (上佁E6bitを取り�EぁE
            // SAIチE�Eタは符号付き32bitなので、算術右シフトで符号を保持
            int32_t sample_L1         = sai_rx_rng_buf[r_L1];
            int32_t sample_R1         = sai_rx_rng_buf[r_R1];
            int32_t sample_L2         = sai_rx_rng_buf[r_L2];
            int32_t sample_R2         = sai_rx_rng_buf[r_R2];
            usb_out_buf_16[f * 4 + 0] = (int16_t) (sample_L1 >> 16);  // L1
            usb_out_buf_16[f * 4 + 1] = (int16_t) (sample_R1 >> 16);  // R1
            usb_out_buf_16[f * 4 + 2] = (int16_t) (sample_L2 >> 16);  // L2
            usb_out_buf_16[f * 4 + 3] = (int16_t) (sample_R2 >> 16);  // R2
        }

        // ISRコンチE��ストから呼ばれるので通常版を使用
        written = tud_audio_write(usb_out_buf, (uint16_t) usb_bytes);

        if (written == 0)
        {
            return;
        }

        // 書けた刁E��け読みポインタを進める
        uint32_t written_frames = ((uint32_t) written) / (4 * sizeof(int16_t));
        if (written_frames > frames)
            written_frames = frames;
        if (written_frames == 0)
            return;
        sai_receive_index += written_frames * 4;  // SAIは4ch刁E
    }
    else
    {
        // 24bit in 32bit slot: SAI(2ch) ↁEUSB(4ch) 変換
        usb_bytes = frames * 4 * sizeof(int32_t);  // 4ch刁E

        // 安�E: usb_out_buf が足りなぁE��定なら絶対に書かなぁE
        if (usb_bytes > sizeof(usb_out_buf))
            return;

        for (uint32_t f = 0; f < frames; f++)
        {
            uint32_t r_L1          = (sai_receive_index + f * 4 + 0) & (SAI_RNG_BUF_SIZE - 1);
            uint32_t r_R1          = (sai_receive_index + f * 4 + 1) & (SAI_RNG_BUF_SIZE - 1);
            uint32_t r_L2          = (sai_receive_index + f * 4 + 2) & (SAI_RNG_BUF_SIZE - 1);
            uint32_t r_R2          = (sai_receive_index + f * 4 + 3) & (SAI_RNG_BUF_SIZE - 1);
            usb_out_buf[f * 4 + 0] = sai_rx_rng_buf[r_L1];  // L1
            usb_out_buf[f * 4 + 1] = sai_rx_rng_buf[r_R1];  // R1
            usb_out_buf[f * 4 + 2] = sai_rx_rng_buf[r_L2];  // L2
            usb_out_buf[f * 4 + 3] = sai_rx_rng_buf[r_R2];  // R2
        }

        // ISRコンチE��ストから呼ばれるので通常版を使用
        written = tud_audio_write(usb_out_buf, (uint16_t) usb_bytes);

        if (written == 0)
        {
            return;
        }

        // 書けた刁E��け読みポインタを進める
        uint32_t written_frames = ((uint32_t) written) / (4 * sizeof(int32_t));
        if (written_frames > frames)
            written_frames = frames;
        if (written_frames == 0)
            return;
        sai_receive_index += written_frames * 4;  // SAIは4ch刁E
    }
}

// TinyUSB TX完亁E��ールバック - USB ISRコンチE��ストで呼ばれる
// ISR冁E��FIFO操作を行うとRX処琁E��競合するため、フラグのみ設宁E
bool tud_audio_tx_done_isr(uint8_t rhport, uint16_t n_bytes_sent, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) n_bytes_sent;
    (void) func_id;
    (void) ep_in;
    (void) cur_alt_setting;

    // ISRではフラグを立てるだぁE- 実際の送信はタスクコンチE��ストで行う
    usb_tx_pending = true;
    return true;
}

// audio_task()呼び出し頻度計測用
static volatile uint32_t audio_task_call_count = 0;
static volatile uint32_t audio_task_last_tick  = 0;
static volatile uint32_t audio_task_frequency  = 0;  // 呼び出し回数/私E

void audio_task(void)
{
    // 呼び出し頻度計測
    audio_task_call_count++;
    uint32_t now = HAL_GetTick();
    if (now - audio_task_last_tick >= 1000)
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
            SEGGER_RTT_printf(0, "[AUD][TX] sr=%lu used_now=%ld used_min=%lu used_max=%lu und=%lu part=%lu drift+%lu drift-%lu usb0=%lu usbB=%lu usbMin=%u usbMax=%u txRw=(%lu,%lu) rxRw=(%lu,%lu) dmae=%lu txe=%lu rxe=%lu txer=0x%08lX rxer=0x%08lX txsr=0x%08lX rxsr=0x%08lX spiC=%lu spiE=%lu spiT=%lu spiM=%lu task_hz=%lu\r\n", (unsigned long) current_sample_rate, (long) tx_used_now, (unsigned long) ((dbg_tx_used_min == 0xFFFFFFFFu) ? 0u : dbg_tx_used_min), (unsigned long) dbg_tx_used_max, (unsigned long) dbg_tx_underrun_events, (unsigned long) dbg_tx_partial_fill_events, (unsigned long) dbg_tx_drift_up_events, (unsigned long) dbg_tx_drift_dn_events, (unsigned long) dbg_usb_read_zero_events, (unsigned long) dbg_usb_read_bytes, (unsigned int) ((dbg_usb_read_size_min == 0xFFFFu) ? 0u : dbg_usb_read_size_min), (unsigned int) dbg_usb_read_size_max, (unsigned long) dbg_tx_half_rewrite_events, (unsigned long) dbg_tx_cplt_rewrite_events, (unsigned long) dbg_rx_half_rewrite_events, (unsigned long) dbg_rx_cplt_rewrite_events, (unsigned long) dbg_dma_err_events, (unsigned long) dbg_sai_tx_err_events, (unsigned long) dbg_sai_rx_err_events, (unsigned long) dbg_sai_tx_last_err, (unsigned long) dbg_sai_rx_last_err, (unsigned long) dbg_sai_tx_sr_flags, (unsigned long) dbg_sai_rx_sr_flags, (unsigned long) (sigma_calls - dbg_sigma_calls_prev), (unsigned long) (sigma_err - dbg_sigma_err_prev), (unsigned long) (sigma_to - dbg_sigma_to_prev), (unsigned long) (sigma_mto - dbg_sigma_mto_prev), (unsigned long) audio_task_frequency);
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
        dbg_usb_read_size_min      = 0xFFFFu;
        dbg_usb_read_size_max      = 0u;
#endif
    }

    if (is_sr_changed)
    {
#if RESET_FROM_FW
        AUDIO_SAI_Reset_ForNewRate();
#endif
        is_sr_changed = false;
    }
    else
    {
        // Feedback EPがFIFO水位を使って送信レート制御するため、E
        // 毎msで「忁E��E��だけ」読む。�E量吸ぁE�Eし�E水位制御を壊す、E
        spk_data_size = tud_audio_read(usb_in_buf, audio_out_bytes_per_ms());
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
        copybuf_usb2ring();
        copybuf_ring2sai();

        // SAI -> USB
        copybuf_sai2ring();

        // USB TX送信 (ISRからのフラグ通知、また�Eストリーミング中は常に試衁E
        if (usb_tx_pending || s_streaming_in)
        {
            usb_tx_pending = false;
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

    /* Stop ADC DMA to prevent parameter changes during ADAU1466 initialization */
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
    dbg_usb_read_size_min      = 0xFFFFu;
    dbg_usb_read_size_max      = 0u;
#endif

    /* Clear all audio buffers to avoid noise from stale data */
    memset(sai_tx_rng_buf, 0, sizeof(sai_tx_rng_buf));
    memset(sai_rx_rng_buf, 0, sizeof(sai_rx_rng_buf));
    memset(stereo_out_buf, 0, sizeof(stereo_out_buf));
    memset(stereo_in_buf, 0, sizeof(stereo_in_buf));
    memset(usb_in_buf, 0, sizeof(usb_in_buf));
    memset(usb_out_buf, 0, sizeof(usb_out_buf));
    __DSB();

    AUDIO_Init_AK4619(96000);
#if RESET_FROM_FW
    if (!AUDIO_Update_ADAU1466_SampleRate(new_hz))
    {
        SEGGER_RTT_printf(0, "[SAI] ADAU1466 sample-rate update failed (%lu Hz)\n", (unsigned long) new_hz);
        SEGGER_RTT_printf(0, "[SAI] fallback to ADAU1466 HW re-init\n");
        AUDIO_Init_ADAU1466(new_hz);
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
