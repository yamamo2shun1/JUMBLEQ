/*
 * oled_control.c
 *
 *  Created on: 2026/01/26
 *      Author: Shnichi Yamamoto
 */

#include "main_oled.h"
#include "sub_oled.h"
#include "oled_control.h"
#include "app_version.h"

#include "audio_control.h"
#include "ssd1306_fonts.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static void merge_dirty_pages(bool* dirty, uint8_t* dirty_start_page, uint8_t* dirty_end_page, uint8_t page_start, uint8_t page_end)
{
    if (!*dirty)
    {
        *dirty            = true;
        *dirty_start_page = page_start;
        *dirty_end_page   = page_end;
        return;
    }

    if (*dirty_start_page > page_start)
    {
        *dirty_start_page = page_start;
    }
    if (*dirty_end_page < page_end)
    {
        *dirty_end_page = page_end;
    }
}

static void update_main_text_block(char* prev, size_t prev_size, const char* text, uint8_t clear_x1, uint8_t clear_y1, uint8_t clear_x2, uint8_t clear_y2, uint8_t cursor_x, uint8_t cursor_y, uint8_t page_start, uint8_t page_end, bool* dirty, uint8_t* dirty_start_page, uint8_t* dirty_end_page)
{
    if (strcmp(prev, text) == 0)
    {
        return;
    }

    main_oled_FillRectangle(clear_x1, clear_y1, clear_x2, clear_y2, Black);
    main_oled_SetCursor(cursor_x, cursor_y);
    main_oled_WriteString((char*) text, Font_7x10, White);
    snprintf(prev, prev_size, "%s", text);

    merge_dirty_pages(dirty, dirty_start_page, dirty_end_page, page_start, page_end);
}

static void update_sub_text_block(char* prev, size_t prev_size, const char* text, uint8_t clear_x1, uint8_t clear_y1, uint8_t clear_x2, uint8_t clear_y2, uint8_t cursor_x, uint8_t cursor_y, uint8_t page_start, uint8_t page_end, bool* dirty, uint8_t* dirty_start_page, uint8_t* dirty_end_page)
{
    if (strcmp(prev, text) == 0)
    {
        return;
    }

    sub_oled_FillRectangle(clear_x1, clear_y1, clear_x2, clear_y2, Black);
    sub_oled_SetCursor(cursor_x, cursor_y);
    sub_oled_WriteString((char*) text, Font_7x10, White);
    snprintf(prev, prev_size, "%s", text);

    merge_dirty_pages(dirty, dirty_start_page, dirty_end_page, page_start, page_end);
}

static void update_sub_dvs_badge(bool show, bool enabled, uint8_t x, uint8_t y, bool* prev_show, bool* prev_enabled, bool* dirty, uint8_t* dirty_start_page, uint8_t* dirty_end_page)
{
    if ((*prev_show == show) && (!show || (*prev_enabled == enabled)))
    {
        return;
    }

    // "[D]" / "[ ]" in Font_7x10 requires ~3 chars width.
    sub_oled_FillRectangle(x, y, (uint8_t) (x + 20U), (uint8_t) (y + 9U), Black);

    if (show)
    {
        sub_oled_SetCursor(x, y);
        sub_oled_WriteString(enabled ? "[D]" : "[ ]", Font_7x10, White);
    }

    *prev_show    = show;
    *prev_enabled = enabled;
    merge_dirty_pages(dirty, dirty_start_page, dirty_end_page, 0, 1);
}

static const char* nonnull_str(const char* text)
{
    return (text == NULL) ? "" : text;
}

static uint16_t oled_text_width_px(SSD1306_Font_t const* font, const char* text)
{
    if ((font == NULL) || (text == NULL))
    {
        return 0U;
    }

    uint16_t width = 0U;
    for (size_t i = 0; text[i] != '\0'; i++)
    {
        char c = text[i];
        if ((font->char_width != NULL) && (c >= 32) && (c <= 126))
        {
            width = (uint16_t) (width + font->char_width[(uint8_t) c - 32U]);
        }
        else
        {
            width = (uint16_t) (width + font->width);
        }
    }

    return width;
}

static bool wait_main_oled_ready(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (HAL_I2C_IsDeviceReady(&MAIN_OLED_I2C_PORT, MAIN_OLED_I2C_ADDR, 2, 20) == HAL_OK)
        {
            return true;
        }
        osDelay(10);
    }

    return false;
}

void OLED_Init(void)
{
    // Power-up直後はOLED側I2C応答まで時間がかかる場合があるため、初回のみ待機する
    (void) wait_main_oled_ready(500);
    main_oled_Init();
    main_oled_Fill(Black);
    main_oled_UpdateScreen();

    sub_oled_Init();
    sub_oled_SetCursor(5, 16);
    sub_oled_WriteString("JUMBLEQ", Font_16x24, White);
    const uint16_t ver_width = oled_text_width_px(&Font_11x18, APP_VERSION_OLED_STR);
    const uint8_t ver_x      = (ver_width < SUB_OLED_WIDTH) ? (uint8_t) (SUB_OLED_WIDTH - ver_width) : 0U;
    sub_oled_SetCursor(ver_x, 40);
    sub_oled_WriteString(APP_VERSION_OLED_STR, Font_11x18, White);
    sub_oled_UpdateScreen();
}

void OLED_ShowInitStatus(const char* text)
{
    const char* msg = (text == NULL) ? "" : text;

    main_oled_Fill(Black);
    main_oled_SetCursor(0, 4);
    main_oled_WriteString("System Init", Font_7x10, White);
    main_oled_SetCursor(0, 18);
    main_oled_WriteString((char*)msg, Font_7x10, White);
    main_oled_UpdateScreen();
}

void OLED_UpdateTask(void)
{
    char line1_ch2[16];
    char line1_mst[16];
    char line2_c1[16];
    char line2_dw[16];
    static char prev_line1_ch2[16] = {0};
    static char prev_line1_mst[16] = {0};
    static char prev_line2_c1[16]  = {0};
    static char prev_line2_dw[16]  = {0};
    static char prev_srcA[32]      = {0};
    static char prev_srcB[32]      = {0};
    static char prev_typeA[32]     = {0};
    static char prev_typeB[32]     = {0};
    static char prev_srcP[32]      = {0};
    static bool prev_dvsA_show     = false;
    static bool prev_dvsA_enabled  = false;
    static bool prev_dvsB_show     = false;
    static bool prev_dvsB_enabled  = false;
    static bool sub_initialized    = false;
    bool dirty                     = false;
    uint8_t dirty_start_page       = 0xFF;
    uint8_t dirty_end_page         = 0;

    const uint8_t line1_ch2_x = 0;
    const uint8_t line1_mst_x = 64;
    const uint8_t line1_y     = 4;

    snprintf(line1_ch2, sizeof(line1_ch2), "C2:%3ddB", get_current_ch2_db());
    snprintf(line1_mst, sizeof(line1_mst), "Mst:%3ddB", get_current_master_db());
    const uint8_t line2_c1_x = 0;
    const uint8_t line2_dw_x = 64;
    const uint8_t line2_y    = 18;

    snprintf(line2_c1, sizeof(line2_c1), "C1:%3ddB", get_current_ch1_db());
    snprintf(line2_dw, sizeof(line2_dw), "D/W:%3d%%", get_current_dry_wet());

    update_main_text_block(prev_line1_ch2, sizeof(prev_line1_ch2), line1_ch2, 0, line1_y, 63, line1_y + 10, line1_ch2_x, line1_y, 0, 1, &dirty, &dirty_start_page, &dirty_end_page);
    update_main_text_block(prev_line1_mst, sizeof(prev_line1_mst), line1_mst, 64, line1_y, 127, line1_y + 10, line1_mst_x, line1_y, 0, 1, &dirty, &dirty_start_page, &dirty_end_page);
    update_main_text_block(prev_line2_c1, sizeof(prev_line2_c1), line2_c1, 0, line2_y, 63, line2_y + 10, line2_c1_x, line2_y, 2, 3, &dirty, &dirty_start_page, &dirty_end_page);
    update_main_text_block(prev_line2_dw, sizeof(prev_line2_dw), line2_dw, 64, line2_y, 127, line2_y + 10, line2_dw_x, line2_y, 2, 3, &dirty, &dirty_start_page, &dirty_end_page);

    if (dirty)
    {
        main_oled_UpdateScreenPages(dirty_start_page, dirty_end_page);
    }

    const char* srcA  = nonnull_str(get_current_input_srcA_str());
    const char* srcB  = nonnull_str(get_current_input_srcB_str());
    const char* typeA = nonnull_str(get_current_input_typeA_str());
    const char* typeB = nonnull_str(get_current_input_typeB_str());
    const char* srcP  = nonnull_str(get_current_input_srcP_str());

    bool sub_dirty               = false;
    uint8_t sub_dirty_start_page = 0xFF;
    uint8_t sub_dirty_end_page   = 0;

    if (!sub_initialized)
    {
        sub_oled_Fill(Black);

        // xfader (static drawing)
        sub_oled_FillRectangle(5, 22, 55, 20, White);
        sub_oled_FillRectangle(73, 22, 123, 20, White);
        sub_oled_FillRectangle(60, 27, 68, 15, White);
        sub_oled_FillCircle(64, 15, 4, White);
        sub_oled_FillCircle(64, 27, 4, White);

        sub_dirty            = true;
        sub_dirty_start_page = 0;
        sub_dirty_end_page   = (uint8_t) ((SUB_OLED_HEIGHT / 8U) - 1U);
        sub_initialized      = true;
    }

    update_sub_text_block(prev_srcA, sizeof(prev_srcA), srcA, 0, 5, 34, 14, 1, 5, 0, 1, &sub_dirty, &sub_dirty_start_page, &sub_dirty_end_page);
    update_sub_text_block(prev_srcB, sizeof(prev_srcB), srcB, 93, 5, 127, 14, 93, 5, 0, 1, &sub_dirty, &sub_dirty_start_page, &sub_dirty_end_page);
    update_sub_text_block(prev_typeA, sizeof(prev_typeA), typeA, 0, 30, 55, 39, 1, 30, 3, 4, &sub_dirty, &sub_dirty_start_page, &sub_dirty_end_page);
    update_sub_text_block(prev_typeB, sizeof(prev_typeB), typeB, 73, 30, 127, 39, 77, 30, 3, 4, &sub_dirty, &sub_dirty_start_page, &sub_dirty_end_page);
    update_sub_text_block(prev_srcP, sizeof(prev_srcP), srcP, 0, 50, 127, 59, 1, 50, 6, 7, &sub_dirty, &sub_dirty_start_page, &sub_dirty_end_page);

    uint8_t srcA_channel = get_current_input_srcA_channel();
    bool srcA_show_dvs   = (srcA_channel != 0U);
    bool srcA_dvs_enable = (srcA_channel == 1U) ? get_current_ch1_dvs_enabled()
                                                : ((srcA_channel == 2U) ? get_current_ch2_dvs_enabled() : false);
    update_sub_dvs_badge(srcA_show_dvs, srcA_dvs_enable, 35, 5, &prev_dvsA_show, &prev_dvsA_enabled, &sub_dirty, &sub_dirty_start_page, &sub_dirty_end_page);

    uint8_t srcB_channel = get_current_input_srcB_channel();
    bool srcB_show_dvs   = (srcB_channel != 0U);
    bool srcB_dvs_enable = (srcB_channel == 1U) ? get_current_ch1_dvs_enabled()
                                                : ((srcB_channel == 2U) ? get_current_ch2_dvs_enabled() : false);
    update_sub_dvs_badge(srcB_show_dvs, srcB_dvs_enable, 72, 5, &prev_dvsB_show, &prev_dvsB_enabled, &sub_dirty, &sub_dirty_start_page, &sub_dirty_end_page);

    if (sub_dirty)
    {
        sub_oled_UpdateScreenPages(sub_dirty_start_page, sub_dirty_end_page);
    }
}
