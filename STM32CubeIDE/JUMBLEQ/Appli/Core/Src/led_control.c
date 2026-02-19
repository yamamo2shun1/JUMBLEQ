/*
 * led_control.c
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#include "led_control.h"

#include "tim.h"

#include "audio_control.h"

#include "SigmaStudioFW.h"
#include "JUMBLEQ_DSP_ADAU146xSchematic_1_PARAM.h"

#define RGB            3
#define COL_BITS       8
#define WL_LED_BIT_LEN (RGB * COL_BITS)
#define LED_NUMS       10
#define LED_BUF_NUMS   WL_LED_BIT_LEN* LED_NUMS
#define DMA_BUF_SIZE   (LED_NUMS * WL_LED_BIT_LEN + 1)
#define WL_LED_ONE     16
#define WL_LED_ZERO    7

#define BLINK_COUNT_MAX 64
#define SAVE_BLINK_INTERVAL_MS 100U
#define SAVE_BLINK_TOGGLE_COUNT 6U

__attribute__((section("noncacheable_buffer"), aligned(32))) uint8_t led_buf[DMA_BUF_SIZE] = {0};

uint8_t grb[LED_NUMS][RGB] = {0};

volatile bool is_color_update = false;
static volatile uint8_t s_save_blink_remaining = 0U;
static uint32_t s_save_blink_last_ms = 0U;

uint16_t test = 0;

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} led_rgb_t;

enum
{
    VU_LEVEL_COUNT   = 5,
    XF_SLOT_COUNT    = 5,
    XF_THRESHOLD_NUM = 4,
};

static const float s_vu_db_thresholds[VU_LEVEL_COUNT] = {-45.0f, -36.0f, -27.0f, -18.0f, -9.0f};

static const led_rgb_t s_vu_colors_low_to_high[VU_LEVEL_COUNT] = {
    {0,   32, 0},
    {30,  61, 0},
    {100, 70, 0},
    {120, 38, 0},
    {127, 0,  0},
};

static const uint8_t s_vu_led_index_a[VU_LEVEL_COUNT] = {0, 1, 2, 3, 4};
static const uint8_t s_vu_led_index_b[VU_LEVEL_COUNT] = {9, 8, 7, 6, 5};

static const uint8_t s_xf_thresholds[XF_THRESHOLD_NUM] = {32, 64, 96, 120};
static const uint8_t s_xf_led_index_a[XF_SLOT_COUNT]   = {0, 1, 2, 3, 4};
static const uint8_t s_xf_led_index_b[XF_SLOT_COUNT]   = {9, 8, 7, 6, 5};

static const float s_xf_blink_peak_level = 80.0f;

void update_color_state(void)
{
    is_color_update = true;
}

void led_notify_save_success(void)
{
    s_save_blink_remaining = SAVE_BLINK_TOGGLE_COUNT;
    s_save_blink_last_ms = 0U;
}

void reset_led_buffer(void)
{
    for (int i = 0; i < DMA_BUF_SIZE; i++)
    {
        led_buf[i] = 0x00;
    }

    for (int k = 0; k < LED_NUMS; k++)
    {
        for (int j = 0; j < RGB; j++)
        {
            grb[k][j] = 0x00;
        }
    }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_tx_blinking_task(void)
{
    static uint32_t start_ms = 0;

    // Blink every interval ms
    if (HAL_GetTick() - start_ms < get_tx_blink_interval_ms())
    {
        return;
    }
    start_ms += get_tx_blink_interval_ms();

    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}

void led_rx_blinking_task(void)
{
    static uint32_t start_ms = 0;

    // Blink every interval ms
    if (HAL_GetTick() - start_ms < get_rx_blink_interval_ms())
    {
        return;
    }
    start_ms += get_rx_blink_interval_ms();

    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void set_led_color(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    grb[index][0] = green;
    grb[index][1] = red;
    grb[index][2] = blue;
}

void layer_led_color(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    grb[index][0] |= green;
    grb[index][1] |= red;
    grb[index][2] |= blue;
}

void renew(void)
{
    for (int k = 0; k < LED_NUMS; k++)
    {
        for (int j = 0; j < RGB; j++)
        {
            for (int i = 0; i < COL_BITS; i++)
            {
                const uint8_t val = grb[k][j];

                led_buf[WL_LED_BIT_LEN * k + COL_BITS * j + i] = ((val >> ((COL_BITS - 1) - i)) & 0x01) ? WL_LED_ONE : WL_LED_ZERO;
            }
        }
    }
    led_buf[DMA_BUF_SIZE - 1] = 0x00;

    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*) led_buf, DMA_BUF_SIZE);
}

static float read_dbfs_from_sigma(uint16_t addr)
{
    ADI_REG_TYPE rx_data[4] = {0};
    SIGMA_READ_REGISTER(DEVICE_ADDR_ADAU146XSCHEMATIC_1, addr, 4, rx_data);
    uint32_t val = rx_data[0] << 24 | rx_data[1] << 16 | rx_data[2] << 8 | rx_data[3];

    if (val == 0 || val == 0xFFFFFFFF)
    {
        return -96.0f;
    }
    return 20.0f * log((float) val / pow(2, 23));
}

static uint8_t vu_active_count(float dbfs)
{
    for (uint8_t i = 0; i < VU_LEVEL_COUNT; i++)
    {
        if (dbfs <= s_vu_db_thresholds[i])
        {
            return i;
        }
    }
    return VU_LEVEL_COUNT;
}

static void set_vu_meter_generic(uint16_t sigma_addr, const uint8_t led_index_low_to_high[VU_LEVEL_COUNT])
{
    const float dbfs        = read_dbfs_from_sigma(sigma_addr);
    const uint8_t active_on = vu_active_count(dbfs);

    for (uint8_t i = 0; i < VU_LEVEL_COUNT; i++)
    {
        if (i < active_on)
        {
            set_led_color(led_index_low_to_high[i], s_vu_colors_low_to_high[i].r, s_vu_colors_low_to_high[i].g, s_vu_colors_low_to_high[i].b);
        }
        else
        {
            set_led_color(led_index_low_to_high[i], 0, 0, 0);
        }
    }
}

void set_vu_meter_a(void)
{
    set_vu_meter_generic(MOD_DSPREADBACK_A_VALUE_ADDR, s_vu_led_index_a);
}

void set_vu_meter_b(void)
{
    set_vu_meter_generic(MOD_DSPREADBACK_B_VALUE_ADDR, s_vu_led_index_b);
}

static uint8_t calc_white_level(uint8_t blink_count)
{
    if (blink_count < BLINK_COUNT_MAX / 2)
    {
        return (uint8_t) (s_xf_blink_peak_level * ((float) blink_count / (float) (BLINK_COUNT_MAX / 2)));
    }
    return (uint8_t) (s_xf_blink_peak_level * ((float) ((BLINK_COUNT_MAX - 1) - blink_count) / (float) (BLINK_COUNT_MAX / 2)));
}

static uint8_t calc_xf_slot(uint8_t xf_pos)
{
    for (uint8_t i = 0; i < XF_THRESHOLD_NUM; i++)
    {
        if (xf_pos < s_xf_thresholds[i])
        {
            return i;
        }
    }
    return XF_SLOT_COUNT - 1;
}

static void layer_xf_position(uint8_t led_index, uint8_t white_level)
{
    for (uint8_t i = 0; i < LED_NUMS; i++)
    {
        layer_led_color(i, 0, 0, 0);
    }
    layer_led_color(led_index, white_level, white_level, white_level);
}

void layer_xfA_position(void)
{
    static uint8_t blink_count_a  = 0;
    const uint8_t xf_pos          = get_current_xfA_position();
    const uint8_t white_level     = calc_white_level(blink_count_a);
    const uint8_t slot            = calc_xf_slot(xf_pos);
    const uint8_t led_index_for_a = s_xf_led_index_a[slot];
    layer_xf_position(led_index_for_a, white_level);

    blink_count_a = (blink_count_a + 1) % BLINK_COUNT_MAX;
}

void layer_xfB_position(void)
{
    static uint8_t blink_count_b  = 0;
    const uint8_t xf_pos          = get_current_xfB_position();
    const uint8_t white_level     = calc_white_level(blink_count_b);
    const uint8_t slot            = calc_xf_slot(xf_pos);
    const uint8_t led_index_for_b = s_xf_led_index_b[slot];
    layer_xf_position(led_index_for_b, white_level);

    blink_count_b = (blink_count_b + 1) % BLINK_COUNT_MAX;
}

void rgb_led_task(void)
{
    set_vu_meter_a();
    set_vu_meter_b();
    layer_xfA_position();
    layer_xfB_position();
    renew();

    if (s_save_blink_remaining > 0U)
    {
        uint32_t now = HAL_GetTick();
        if ((s_save_blink_last_ms == 0U) || ((now - s_save_blink_last_ms) >= SAVE_BLINK_INTERVAL_MS))
        {
            s_save_blink_last_ms = now;
            HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
            s_save_blink_remaining--;
            if (s_save_blink_remaining == 0U)
            {
                HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
            }
        }
    }

    if (is_color_update)
    {
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);

        is_color_update = false;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
    __DSB();
}
