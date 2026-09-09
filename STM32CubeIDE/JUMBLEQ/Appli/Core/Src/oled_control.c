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

static void draw_sub_headphones_icon(uint8_t x, uint8_t y);
static void draw_main_headphones_icon(uint8_t x, uint8_t y);
static void draw_main_ch_fader_curve_preview(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t cc_value);
static void draw_main_uf2_transition(UI_Uf2TransitionState_t state, uint8_t seconds_remaining);

static void draw_main_uf2_transition(UI_Uf2TransitionState_t state, uint8_t seconds_remaining)
{
    char timeout_line[20];
    const char* instruction = "";

    main_oled_Fill(Black);

    if (state == UI_UF2_TRANSITION_CANCELLED)
    {
        main_oled_SetCursor(0, 0);
        main_oled_WriteString("UF2 CANCELLED", Font_7x10, White);
        main_oled_SetCursor(0, 12);
        main_oled_WriteString("NO RESTART", Font_7x10, White);
        main_oled_UpdateScreen();
        return;
    }

    if (state == UI_UF2_TRANSITION_TIMED_OUT)
    {
        main_oled_SetCursor(0, 0);
        main_oled_WriteString("UF2 TIMEOUT", Font_7x10, White);
        main_oled_SetCursor(0, 11);
        main_oled_WriteString("TRY AGAIN", Font_7x10, White);
        main_oled_SetCursor(0, 22);
        main_oled_WriteString("OR SW3 + RESET", Font_7x10, White);
        main_oled_UpdateScreen();
        return;
    }

    if (state == UI_UF2_TRANSITION_WAIT_RELEASE)
    {
        instruction = "RELEASE SW3";
    }
    else if (state == UI_UF2_TRANSITION_WAIT_HOLD)
    {
        instruction = "HOLD SW3 2 SEC";
    }
    else
    {
        instruction = "KEEP HOLDING SW3";
    }

    snprintf(timeout_line, sizeof(timeout_line), "TIMEOUT: %us", (unsigned) seconds_remaining);
    main_oled_SetCursor(0, 0);
    main_oled_WriteString("UF2 MODE REQUEST", Font_7x10, White);
    main_oled_SetCursor(0, 11);
    main_oled_WriteString((char*) instruction, Font_7x10, White);
    main_oled_SetCursor(0, 22);
    main_oled_WriteString(timeout_line, Font_7x10, White);
    main_oled_UpdateScreen();
}

static void draw_main_ch_fader_curve_preview(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t cc_value)
{
    if ((width < 2U) || (height < 2U))
    {
        return;
    }

    uint8_t prev_x = x;
    uint8_t prev_y = (uint8_t) (y + height - 1U);

    for (uint8_t i = 0U; i < width; i++)
    {
        const float position = (float) i / (float) (width - 1U);
        const float gain     = ui_control_evaluate_ch_fader_curve_preview(cc_value, position);
        const uint8_t current_x = (uint8_t) (x + i);
        const uint8_t current_y = (uint8_t) (y + height - 1U - (uint8_t) (gain * (float) (height - 1U)));

        if (i > 0U)
        {
            main_oled_Line(prev_x, prev_y, current_x, current_y, White);
        }

        prev_x = current_x;
        prev_y = current_y;
    }
}

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

static void update_sub_reverse_indicators(bool reverse_a, bool reverse_b, bool* prev_reverse_a, bool* prev_reverse_b, bool* dirty, uint8_t* dirty_start_page, uint8_t* dirty_end_page)
{
    if (*prev_reverse_a != reverse_a)
    {
        sub_oled_FillRectangle(0, 15, 55, 26, Black);
        if (reverse_a)
        {
            sub_oled_SetCursor(0, 17);
            sub_oled_WriteString("R", Font_7x10, White);
            sub_oled_FillRectangle(8, 22, 55, 20, White);
        }
        else
        {
            sub_oled_FillRectangle(5, 22, 55, 20, White);
        }

        *prev_reverse_a = reverse_a;
        merge_dirty_pages(dirty, dirty_start_page, dirty_end_page, 1, 3);
    }

    if (*prev_reverse_b != reverse_b)
    {
        sub_oled_FillRectangle(73, 15, 127, 26, Black);
        if (reverse_b)
        {
            sub_oled_FillRectangle(73, 22, 119, 20, White);
            sub_oled_SetCursor(121, 17);
            sub_oled_WriteString("R", Font_7x10, White);
        }
        else
        {
            sub_oled_FillRectangle(73, 22, 123, 20, White);
        }

        *prev_reverse_b = reverse_b;
        merge_dirty_pages(dirty, dirty_start_page, dirty_end_page, 1, 3);
    }
}

static void update_sub_bottom_status(char* prev_srcP, size_t prev_srcP_size, char* prev_hp_src, size_t prev_hp_src_size, const char* srcP, const char* hp_src, bool* dirty, uint8_t* dirty_start_page, uint8_t* dirty_end_page)
{
    const bool srcP_changed   = (strcmp(prev_srcP, srcP) != 0);
    const bool hp_src_changed = (strcmp(prev_hp_src, hp_src) != 0);

    if (!srcP_changed && !hp_src_changed)
    {
        return;
    }

    // Treat the bottom row as one unit so overlapping redraw regions never erase the HP label.
    sub_oled_FillRectangle(0, 50, 127, 59, Black);
    sub_oled_SetCursor(1, 50);
    sub_oled_WriteString((char*) srcP, Font_7x10, White);
    draw_sub_headphones_icon(103, 50);
    sub_oled_SetCursor(120, 51);
    sub_oled_WriteString((char*) hp_src, Font_7x10, White);

    snprintf(prev_srcP, prev_srcP_size, "%s", srcP);
    snprintf(prev_hp_src, prev_hp_src_size, "%s", hp_src);
    merge_dirty_pages(dirty, dirty_start_page, dirty_end_page, 6, 7);
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

static const unsigned char headphones_icon_16x10[] = {
    0x03, 0xC0, // 00000011 11000000
    0x0E, 0x70, // 00001110 01110000
    0x18, 0x18, // 00011000 00011000
    0x30, 0x0C, // 00110000 00001100
    0x20, 0x04, // 00100000 00000100
    0x20, 0x04, // 00100000 00000100
    0x36, 0x6C, // 00110110 01101100
    0x3E, 0x7C, // 00111110 01111100
    0x1E, 0x78, // 00011110 01111000
    0x0E, 0x70, // 00001110 01110000
};

static void draw_sub_headphones_icon(uint8_t x, uint8_t y)
{
    sub_oled_DrawBitmap(x, y, headphones_icon_16x10, 16, 10, White);
}

static void draw_main_headphones_icon(uint8_t x, uint8_t y)
{
    main_oled_DrawBitmap(x, y, headphones_icon_16x10, 16, 10, White);
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
    main_oled_WriteString((char*) msg, Font_7x10, White);
    main_oled_UpdateScreen();
}

void OLED_UpdateTask(void)
{
    const bool curve_edit_mode       = ui_control_is_curve_edit_mode_enabled();
    const UI_Uf2TransitionState_t uf2_transition_state = ui_control_get_uf2_transition_state();
    const uint8_t uf2_seconds_remaining = ui_control_get_uf2_seconds_remaining();
    static bool prev_curve_edit_mode = false;
    static UI_Uf2TransitionState_t prev_uf2_transition_state = UI_UF2_TRANSITION_IDLE;
    static uint8_t prev_uf2_seconds_remaining = 0xFFU;

    char line1_sr[8];
    char line1_ch2[32];
    char line2_ch1[32];
    char line3_sr[32];
    char line3_hp[8];
    char line_edit_mode[20];
    char line_edit_a[20];
    char line_edit_b[20];
    static char prev_line1_sr[8]        = {0};
    static char prev_line1_ch2[32]      = {0};
    static char prev_line2_ch1[32]      = {0};
    static char prev_line3_sr[32]       = {0};
    static char prev_line3_hp[8]        = {0};
    static char prev_line_edit_mode[20] = {0};
    static char prev_line_edit_a[20]    = {0};
    static char prev_line_edit_b[20]    = {0};
    static char prev_srcA[32]           = {0};
    static char prev_srcB[32]           = {0};
    static char prev_typeA[32]          = {0};
    static char prev_typeB[32]          = {0};
    static char prev_srcP[32]           = {0};
    static char prev_hp_src[8]          = {0};
    static bool prev_dvsA_show          = false;
    static bool prev_dvsA_enabled       = false;
    static bool prev_dvsB_show          = false;
    static bool prev_dvsB_enabled       = false;
    static bool prev_reverse_a          = false;
    static bool prev_reverse_b          = false;
    static bool sub_initialized         = false;
    bool dirty                          = false;
    uint8_t dirty_start_page            = 0xFF;
    uint8_t dirty_end_page              = 0;

    if (uf2_transition_state == UI_UF2_TRANSITION_CLEARING_DISPLAYS)
    {
        if (prev_uf2_transition_state != UI_UF2_TRANSITION_CLEARING_DISPLAYS)
        {
            main_oled_Fill(Black);
            main_oled_UpdateScreen();
            sub_oled_Fill(Black);
            sub_oled_UpdateScreen();
            prev_uf2_transition_state = UI_UF2_TRANSITION_CLEARING_DISPLAYS;
            prev_uf2_seconds_remaining = 0U;
            ui_control_notify_uf2_displays_cleared();
        }
        return;
    }

    if (uf2_transition_state != UI_UF2_TRANSITION_IDLE)
    {
        if ((uf2_transition_state != prev_uf2_transition_state) ||
            (uf2_seconds_remaining != prev_uf2_seconds_remaining))
        {
            draw_main_uf2_transition(uf2_transition_state, uf2_seconds_remaining);
            prev_uf2_transition_state = uf2_transition_state;
            prev_uf2_seconds_remaining = uf2_seconds_remaining;
        }
        return;
    }

    if (prev_uf2_transition_state != UI_UF2_TRANSITION_IDLE)
    {
        main_oled_Fill(Black);
        memset(prev_line1_sr, 0, sizeof(prev_line1_sr));
        memset(prev_line1_ch2, 0, sizeof(prev_line1_ch2));
        memset(prev_line2_ch1, 0, sizeof(prev_line2_ch1));
        memset(prev_line3_sr, 0, sizeof(prev_line3_sr));
        memset(prev_line3_hp, 0, sizeof(prev_line3_hp));
        memset(prev_line_edit_mode, 0, sizeof(prev_line_edit_mode));
        memset(prev_line_edit_a, 0, sizeof(prev_line_edit_a));
        memset(prev_line_edit_b, 0, sizeof(prev_line_edit_b));
        memset(prev_srcA, 0, sizeof(prev_srcA));
        memset(prev_srcB, 0, sizeof(prev_srcB));
        memset(prev_typeA, 0, sizeof(prev_typeA));
        memset(prev_typeB, 0, sizeof(prev_typeB));
        memset(prev_srcP, 0, sizeof(prev_srcP));
        memset(prev_hp_src, 0, sizeof(prev_hp_src));
        prev_dvsA_show                = false;
        prev_dvsA_enabled             = false;
        prev_dvsB_show                = false;
        prev_dvsB_enabled             = false;
        prev_reverse_a                = false;
        prev_reverse_b                = false;
        sub_initialized               = false;
        dirty                        = true;
        dirty_start_page             = 0U;
        dirty_end_page               = (uint8_t) ((MAIN_OLED_HEIGHT / 8U) - 1U);
        prev_curve_edit_mode         = curve_edit_mode;
        prev_uf2_transition_state    = UI_UF2_TRANSITION_IDLE;
        prev_uf2_seconds_remaining   = 0xFFU;
    }

    if (curve_edit_mode != prev_curve_edit_mode)
    {
        main_oled_Fill(Black);
        memset(prev_line1_sr, 0, sizeof(prev_line1_sr));
        memset(prev_line1_ch2, 0, sizeof(prev_line1_ch2));
        memset(prev_line2_ch1, 0, sizeof(prev_line2_ch1));
        memset(prev_line3_sr, 0, sizeof(prev_line3_sr));
        memset(prev_line3_hp, 0, sizeof(prev_line3_hp));
        memset(prev_line_edit_mode, 0, sizeof(prev_line_edit_mode));
        memset(prev_line_edit_a, 0, sizeof(prev_line_edit_a));
        memset(prev_line_edit_b, 0, sizeof(prev_line_edit_b));
        dirty                = true;
        dirty_start_page     = 0;
        dirty_end_page       = (uint8_t) ((MAIN_OLED_HEIGHT / 8U) - 1U);
        prev_curve_edit_mode = curve_edit_mode;
    }

    if (!curve_edit_mode)
    {
        bool main_redraw = dirty;
        uint32_t sample_rate_hz = get_current_sample_rate_hz();
        const char* return_src = nonnull_str(get_current_return_src_str());
        snprintf(line1_sr, sizeof(line1_sr), "%luk", (unsigned long) (sample_rate_hz / 1000U));
        snprintf(line1_ch2, sizeof(line1_ch2), "1:%3d 2:%3d", get_current_ch1_out_db(), get_current_ch2_out_db());
        // MAIN OLED shows both Out and In as CH1 and CH2.
        snprintf(line2_ch1, sizeof(line2_ch1), "    IN 1:%3d 2:%3d", get_current_ch1_in_db(), get_current_ch2_in_db());
        if (get_current_return_enabled())
        {
            snprintf(line3_sr, sizeof(line3_sr), "RTN|%-3s:%3d ", return_src, get_current_return_db());
        }
        else
        {
            snprintf(line3_sr, sizeof(line3_sr), "RTN|%-3s:--- ", return_src);
        }
        snprintf(line3_hp, sizeof(line3_hp), ":%3d", get_current_hp_out_db());

        if ((strcmp(prev_line1_sr, line1_sr) != 0) ||
            (strcmp(prev_line1_ch2, line1_ch2) != 0) ||
            (strcmp(prev_line2_ch1, line2_ch1) != 0) ||
            (strcmp(prev_line3_sr, line3_sr) != 0) ||
            (strcmp(prev_line3_hp, line3_hp) != 0))
        {
            main_redraw = true;
        }

        if (main_redraw)
        {
            uint8_t line1_gain_x;
            uint8_t line1_out_x;
            uint8_t hp_icon_x;
            uint8_t hp_text_x;
            const int8_t line1_out_offset = -3;
            const int8_t hp_block_offset = -2;

            main_oled_Fill(Black);
            main_oled_SetCursor(0, 0);
            main_oled_WriteString(line1_sr, Font_7x10, White);
            line1_out_x  = (uint8_t) ((int16_t) oled_text_width_px(&Font_7x10, line1_sr)
                                     + (int16_t) oled_text_width_px(&Font_7x10, " ")
                                     + line1_out_offset);
            line1_gain_x = (uint8_t) (oled_text_width_px(&Font_7x10, line1_sr) + oled_text_width_px(&Font_7x10, " OUT"));
            main_oled_SetCursor(line1_out_x, 0);
            main_oled_WriteString("OUT", Font_7x10, White);
            main_oled_SetCursor(line1_gain_x, 0);
            main_oled_WriteString(line1_ch2, Font_7x10, White);
            main_oled_SetCursor(0, 10);
            main_oled_WriteString(line2_ch1, Font_7x10, White);
            main_oled_SetCursor(0, 22);
            main_oled_WriteString(line3_sr, Font_7x10, White);
            hp_icon_x = (uint8_t) ((int16_t) oled_text_width_px(&Font_7x10, line3_sr) + hp_block_offset);
            draw_main_headphones_icon(hp_icon_x, 22);
            hp_text_x = (uint8_t) (hp_icon_x + 16U);
            main_oled_SetCursor(hp_text_x, 22);
            main_oled_WriteString(line3_hp, Font_7x10, White);

            snprintf(prev_line1_sr, sizeof(prev_line1_sr), "%s", line1_sr);
            snprintf(prev_line1_ch2, sizeof(prev_line1_ch2), "%s", line1_ch2);
            snprintf(prev_line2_ch1, sizeof(prev_line2_ch1), "%s", line2_ch1);
            snprintf(prev_line3_sr, sizeof(prev_line3_sr), "%s", line3_sr);
            snprintf(prev_line3_hp, sizeof(prev_line3_hp), "%s", line3_hp);

            dirty            = true;
            dirty_start_page = 0;
            dirty_end_page   = (uint8_t) ((MAIN_OLED_HEIGHT / 8U) - 1U);
        }
    }
    else
    {
        bool main_redraw = dirty;
        const uint8_t curve_a_cc = ui_control_get_ch_fader_curve_a_cc();
        const uint8_t curve_b_cc = ui_control_get_ch_fader_curve_b_cc();
        const bool dvs_enabled = get_current_ch1_dvs_enabled() || get_current_ch2_dvs_enabled();
        if (dvs_enabled)
        {
            snprintf(line_edit_mode,
                     sizeof(line_edit_mode),
                     "FADER DLY: %ums",
                     (unsigned) ui_control_get_ch_fader_dvs_delay_ms());
        }
        else
        {
            snprintf(line_edit_mode, sizeof(line_edit_mode), "FADER DLY: ---");
        }
        snprintf(line_edit_a, sizeof(line_edit_a), "A:%3u", (unsigned) curve_a_cc);
        snprintf(line_edit_b, sizeof(line_edit_b), "B:%3u", (unsigned) curve_b_cc);

        if ((strcmp(prev_line_edit_mode, line_edit_mode) != 0) ||
            (strcmp(prev_line_edit_a, line_edit_a) != 0) ||
            (strcmp(prev_line_edit_b, line_edit_b) != 0))
        {
            main_redraw = true;
        }

        if (main_redraw)
        {
            main_oled_Fill(Black);
            main_oled_SetCursor(0, 0);
            main_oled_WriteString(line_edit_mode, Font_7x10, White);
            main_oled_SetCursor(18, 11);
            main_oled_WriteString(line_edit_a, Font_7x10, White);
            main_oled_SetCursor(18, 22);
            main_oled_WriteString(line_edit_b, Font_7x10, White);
            draw_main_ch_fader_curve_preview(60U, 11U, 51U, 10U, curve_a_cc);
            draw_main_ch_fader_curve_preview(60U, 22U, 51U, 10U, curve_b_cc);

            snprintf(prev_line_edit_mode, sizeof(prev_line_edit_mode), "%s", line_edit_mode);
            snprintf(prev_line_edit_a, sizeof(prev_line_edit_a), "%s", line_edit_a);
            snprintf(prev_line_edit_b, sizeof(prev_line_edit_b), "%s", line_edit_b);

            dirty            = true;
            dirty_start_page = 0;
            dirty_end_page   = (uint8_t) ((MAIN_OLED_HEIGHT / 8U) - 1U);
        }
    }

    if (dirty)
    {
        main_oled_UpdateScreenPages(dirty_start_page, dirty_end_page);
    }

    const char* srcA  = nonnull_str(get_current_input_srcA_str());
    const char* srcB  = nonnull_str(get_current_input_srcB_str());
    const char* typeA = nonnull_str(get_current_input_typeA_str());
    const char* typeB = nonnull_str(get_current_input_typeB_str());
    const char* srcP  = nonnull_str(get_current_input_srcP_str());
    const char* hpSrc = nonnull_str(get_current_hp_out_src_str());

    bool sub_dirty               = false;
    uint8_t sub_dirty_start_page = 0xFF;
    uint8_t sub_dirty_end_page   = 0;

    if (!sub_initialized)
    {
        sub_oled_Fill(Black);

        // channel fader (static drawing)
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
    update_sub_bottom_status(prev_srcP, sizeof(prev_srcP), prev_hp_src, sizeof(prev_hp_src), srcP, hpSrc, &sub_dirty, &sub_dirty_start_page, &sub_dirty_end_page);
    update_sub_reverse_indicators(ui_control_is_ch_fader_reverse_a_enabled(),
                                  ui_control_is_ch_fader_reverse_b_enabled(),
                                  &prev_reverse_a,
                                  &prev_reverse_b,
                                  &sub_dirty,
                                  &sub_dirty_start_page,
                                  &sub_dirty_end_page);

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
