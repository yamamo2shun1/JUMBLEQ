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
#include "oto_no_ita_dsp_ADAU146xSchematic_1_PARAM.h"

#define RGB            3
#define COL_BITS       8
#define WL_LED_BIT_LEN (RGB * COL_BITS)
#define LED_NUMS       10
#define LED_BUF_NUMS   WL_LED_BIT_LEN* LED_NUMS
#define DMA_BUF_SIZE   (LED_NUMS * WL_LED_BIT_LEN + 1)
#define WL_LED_ONE     16
#define WL_LED_ZERO    7

#define BLINK_COUNT_MAX 64

__attribute__((section("noncacheable_buffer"), aligned(32))) uint8_t led_buf[DMA_BUF_SIZE] = {0};

uint8_t grb[LED_NUMS][RGB] = {0};

volatile bool is_color_update = false;

uint16_t test = 0;

void update_color_state(void)
{
    is_color_update = true;
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

void set_vu_meter_a(void)
{
    ADI_REG_TYPE rx_data[4] = {0};
    SIGMA_READ_REGISTER(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_DSPREADBACK_A_VALUE_ADDR, 4, rx_data);
    uint32_t val = rx_data[0] << 24 | rx_data[1] << 16 | rx_data[2] << 8 | rx_data[3];
    float dbfs   = 0.0f;
    if (val == 0 || val == 0xFFFFFFFF)
    {
        dbfs = -96.0f;
    }
    else
    {
        dbfs = 20.0f * log((float) val / pow(2, 23));
    }
    if (dbfs > -9.0f)
    {
        set_led_color(4, 127, 0, 0);
        set_led_color(3, 120, 38, 0);
        set_led_color(2, 100, 70, 0);
        set_led_color(1, 30, 61, 0);
        set_led_color(0, 0, 32, 0);
    }
    else if (dbfs > -18.0f)
    {
        set_led_color(4, 0, 0, 0);
        set_led_color(3, 120, 38, 0);
        set_led_color(2, 100, 70, 0);
        set_led_color(1, 30, 61, 0);
        set_led_color(0, 0, 32, 0);
    }
    else if (dbfs > -27.0f)
    {
        set_led_color(4, 0, 0, 0);
        set_led_color(3, 0, 0, 0);
        set_led_color(2, 100, 70, 0);
        set_led_color(1, 30, 61, 0);
        set_led_color(0, 0, 32, 0);
    }
    else if (dbfs > -36.0f)
    {
        set_led_color(4, 0, 0, 0);
        set_led_color(3, 0, 0, 0);
        set_led_color(2, 0, 0, 0);
        set_led_color(1, 30, 61, 0);
        set_led_color(0, 0, 32, 0);
    }
    else if (dbfs > -45.0f)
    {
        set_led_color(4, 0, 0, 0);
        set_led_color(3, 0, 0, 0);
        set_led_color(2, 0, 0, 0);
        set_led_color(1, 0, 0, 0);
        set_led_color(0, 0, 32, 0);
    }
    else
    {
        set_led_color(4, 0, 0, 0);
        set_led_color(3, 0, 0, 0);
        set_led_color(2, 0, 0, 0);
        set_led_color(1, 0, 0, 0);
        set_led_color(0, 0, 0, 0);
    }
}

void set_vu_meter_b(void)
{
    ADI_REG_TYPE rx_data[4] = {0};
    SIGMA_READ_REGISTER(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_DSPREADBACK_B_VALUE_ADDR, 4, rx_data);
    uint32_t val = rx_data[0] << 24 | rx_data[1] << 16 | rx_data[2] << 8 | rx_data[3];
    float dbfs   = 0.0f;
    if (val == 0 || val == 0xFFFFFFFF)
    {
        dbfs = -96.0f;
    }
    else
    {
        dbfs = 20.0f * log((float) val / pow(2, 23));
    }
    if (dbfs > -9.0f)
    {
        set_led_color(5, 127, 0, 0);
        set_led_color(6, 120, 38, 0);
        set_led_color(7, 100, 70, 0);
        set_led_color(8, 30, 61, 0);
        set_led_color(9, 0, 32, 0);
    }
    else if (dbfs > -18.0f)
    {
        set_led_color(5, 0, 0, 0);
        set_led_color(6, 120, 38, 0);
        set_led_color(7, 100, 70, 0);
        set_led_color(8, 30, 61, 0);
        set_led_color(9, 0, 32, 0);
    }
    else if (dbfs > -27.0f)
    {
        set_led_color(5, 0, 0, 0);
        set_led_color(6, 0, 0, 0);
        set_led_color(7, 100, 70, 0);
        set_led_color(8, 30, 61, 0);
        set_led_color(9, 0, 32, 0);
    }
    else if (dbfs > -36.0f)
    {
        set_led_color(5, 0, 0, 0);
        set_led_color(6, 0, 0, 0);
        set_led_color(7, 0, 0, 0);
        set_led_color(8, 30, 61, 0);
        set_led_color(9, 0, 32, 0);
    }
    else if (dbfs > -45.0f)
    {
        set_led_color(5, 0, 0, 0);
        set_led_color(6, 0, 0, 0);
        set_led_color(7, 0, 0, 0);
        set_led_color(8, 0, 0, 0);
        set_led_color(9, 0, 32, 0);
    }
    else
    {
        set_led_color(5, 0, 0, 0);
        set_led_color(6, 0, 0, 0);
        set_led_color(7, 0, 0, 0);
        set_led_color(8, 0, 0, 0);
        set_led_color(9, 0, 0, 0);
    }
}

void layer_xfA_position(void)
{
    static uint8_t blink_count_a = 0;
    uint8_t xf_pos               = get_current_xfA_position();

    uint8_t white_level = 0;

    if (blink_count_a < BLINK_COUNT_MAX / 2)
    {
        white_level = (uint8_t) (80.0f * ((float) blink_count_a / (float) (BLINK_COUNT_MAX / 2)));
    }
    else
    {
        white_level = (uint8_t) (80.0f * ((float) ((BLINK_COUNT_MAX - 1) - blink_count_a) / (float) (BLINK_COUNT_MAX / 2)));
    }

    if (xf_pos < 32)
    {
        layer_led_color(0, white_level, white_level, white_level);
        layer_led_color(1, 0, 0, 0);
        layer_led_color(2, 0, 0, 0);
        layer_led_color(3, 0, 0, 0);
        layer_led_color(4, 0, 0, 0);
        layer_led_color(5, 0, 0, 0);
        layer_led_color(6, 0, 0, 0);
        layer_led_color(7, 0, 0, 0);
        layer_led_color(8, 0, 0, 0);
        layer_led_color(9, 0, 0, 0);
    }
    else if (xf_pos < 64)
    {
        layer_led_color(0, 0, 0, 0);
        layer_led_color(1, white_level, white_level, white_level);
        layer_led_color(2, 0, 0, 0);
        layer_led_color(3, 0, 0, 0);
        layer_led_color(4, 0, 0, 0);
        layer_led_color(5, 0, 0, 0);
        layer_led_color(6, 0, 0, 0);
        layer_led_color(7, 0, 0, 0);
        layer_led_color(8, 0, 0, 0);
        layer_led_color(9, 0, 0, 0);
    }
    else if (xf_pos < 96)
    {
        layer_led_color(0, 0, 0, 0);
        layer_led_color(1, 0, 0, 0);
        layer_led_color(2, white_level, white_level, white_level);
        layer_led_color(3, 0, 0, 0);
        layer_led_color(4, 0, 0, 0);
        layer_led_color(5, 0, 0, 0);
        layer_led_color(6, 0, 0, 0);
        layer_led_color(7, 0, 0, 0);
        layer_led_color(8, 0, 0, 0);
        layer_led_color(9, 0, 0, 0);
    }
    else if (xf_pos < 120)
    {
        layer_led_color(0, 0, 0, 0);
        layer_led_color(1, 0, 0, 0);
        layer_led_color(2, 0, 0, 0);
        layer_led_color(3, white_level, white_level, white_level);
        layer_led_color(4, 0, 0, 0);
        layer_led_color(5, 0, 0, 0);
        layer_led_color(6, 0, 0, 0);
        layer_led_color(7, 0, 0, 0);
        layer_led_color(8, 0, 0, 0);
        layer_led_color(9, 0, 0, 0);
    }
    else
    {
        layer_led_color(0, 0, 0, 0);
        layer_led_color(1, 0, 0, 0);
        layer_led_color(2, 0, 0, 0);
        layer_led_color(3, 0, 0, 0);
        layer_led_color(4, white_level, white_level, white_level);
        layer_led_color(5, 0, 0, 0);
        layer_led_color(6, 0, 0, 0);
        layer_led_color(7, 0, 0, 0);
        layer_led_color(8, 0, 0, 0);
        layer_led_color(9, 0, 0, 0);
    }
    blink_count_a = (blink_count_a + 1) % BLINK_COUNT_MAX;
}

void layer_xfB_position(void)
{
    static uint8_t blink_count_b = 0;
    uint8_t xf_pos               = get_current_xfB_position();

    uint8_t white_level = 0;

    if (blink_count_b < BLINK_COUNT_MAX / 2)
    {
        white_level = (uint8_t) (80.0f * ((float) blink_count_b / (float) (BLINK_COUNT_MAX / 2)));
    }
    else
    {
        white_level = (uint8_t) (80.0f * ((float) ((BLINK_COUNT_MAX - 1) - blink_count_b) / (float) (BLINK_COUNT_MAX / 2)));
    }

    if (xf_pos < 32)
    {
        layer_led_color(0, 0, 0, 0);
        layer_led_color(1, 0, 0, 0);
        layer_led_color(2, 0, 0, 0);
        layer_led_color(3, 0, 0, 0);
        layer_led_color(4, 0, 0, 0);
        layer_led_color(5, 0, 0, 0);
        layer_led_color(6, 0, 0, 0);
        layer_led_color(7, 0, 0, 0);
        layer_led_color(8, 0, 0, 0);
        layer_led_color(9, white_level, white_level, white_level);
    }
    else if (xf_pos < 64)
    {
        layer_led_color(0, 0, 0, 0);
        layer_led_color(1, 0, 0, 0);
        layer_led_color(2, 0, 0, 0);
        layer_led_color(3, 0, 0, 0);
        layer_led_color(4, 0, 0, 0);
        layer_led_color(5, 0, 0, 0);
        layer_led_color(6, 0, 0, 0);
        layer_led_color(7, 0, 0, 0);
        layer_led_color(8, white_level, white_level, white_level);
        layer_led_color(9, 0, 0, 0);
    }
    else if (xf_pos < 96)
    {
        layer_led_color(0, 0, 0, 0);
        layer_led_color(1, 0, 0, 0);
        layer_led_color(2, 0, 0, 0);
        layer_led_color(3, 0, 0, 0);
        layer_led_color(4, 0, 0, 0);
        layer_led_color(5, 0, 0, 0);
        layer_led_color(6, 0, 0, 0);
        layer_led_color(7, white_level, white_level, white_level);
        layer_led_color(8, 0, 0, 0);
        layer_led_color(9, 0, 0, 0);
    }
    else if (xf_pos < 120)
    {
        layer_led_color(0, 0, 0, 0);
        layer_led_color(1, 0, 0, 0);
        layer_led_color(2, 0, 0, 0);
        layer_led_color(3, 0, 0, 0);
        layer_led_color(4, 0, 0, 0);
        layer_led_color(5, 0, 0, 0);
        layer_led_color(6, white_level, white_level, white_level);
        layer_led_color(7, 0, 0, 0);
        layer_led_color(8, 0, 0, 0);
        layer_led_color(9, 0, 0, 0);
    }
    else
    {
        layer_led_color(0, 0, 0, 0);
        layer_led_color(1, 0, 0, 0);
        layer_led_color(2, 0, 0, 0);
        layer_led_color(3, 0, 0, 0);
        layer_led_color(4, 0, 0, 0);
        layer_led_color(5, white_level, white_level, white_level);
        layer_led_color(6, 0, 0, 0);
        layer_led_color(7, 0, 0, 0);
        layer_led_color(8, 0, 0, 0);
        layer_led_color(9, 0, 0, 0);
    }
    blink_count_b = (blink_count_b + 1) % BLINK_COUNT_MAX;
}

void rgb_led_task(void)
{
    set_vu_meter_a();
    set_vu_meter_b();
    layer_xfA_position();
    layer_xfB_position();
    renew();

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
