/*
 * oled_control.c
*
 *  Created on: 2026/01/26
 *      Author: Shnichi Yamamoto
 */

#include "main_oled.h"
#include "sub_oled.h"
#include "oled_control.h"

#include "audio_control.h"
#include "ssd1306_fonts.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

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
    main_oled_SetCursor(0, 22);
    main_oled_WriteString("A:C1(Ln)  B:C2(Ph)", Font_7x10, White);
    main_oled_UpdateScreen();

    sub_oled_Init();
    sub_oled_SetCursor(5, 16);
    sub_oled_WriteString("JUMBLEQ", Font_16x24, White);
    sub_oled_SetCursor(48, 40);
    sub_oled_WriteString("ver0.8", Font_11x18, White);
    sub_oled_UpdateScreen();
}

void OLED_UpdateTask(void)
{
    char line1[32];
    char line2[32];
    char line3[32];
    static char prev_line1[32] = {0};
    static char prev_line2[32] = {0};
    static char prev_line3[32] = {0};
    bool dirty                 = false;
    uint8_t dirty_start_page   = 0xFF;
    uint8_t dirty_end_page     = 0;

    snprintf(line1, sizeof(line1), "C2:%ddB Mst:%ddB", get_current_ch2_db(), get_current_master_db());
    snprintf(line2, sizeof(line2), "C1:%ddB D/W:%d%%", get_current_ch1_db(), get_current_dry_wet());
    snprintf(line3, sizeof(line3), "A:%s B:%s", get_current_input_srcA_str(), get_current_input_srcB_str());

    if (strcmp(prev_line1, line1) != 0)
    {
        main_oled_FillRectangle(0, 0, 127, 10, Black);
        main_oled_SetCursor(0, 0);
        main_oled_WriteString(line1, Font_7x10, White);
        strcpy(prev_line1, line1);

        dirty            = true;
        dirty_start_page = 0;
        dirty_end_page   = 1;
    }

    if (strcmp(prev_line2, line2) != 0)
    {
        main_oled_FillRectangle(0, 11, 127, 21, Black);
        main_oled_SetCursor(0, 11);
        main_oled_WriteString(line2, Font_7x10, White);
        strcpy(prev_line2, line2);

        if (!dirty)
        {
            dirty_start_page = 1;
            dirty_end_page   = 2;
        }
        else
        {
            if (dirty_start_page > 1)
            {
                dirty_start_page = 1;
            }
            if (dirty_end_page < 2)
            {
                dirty_end_page = 2;
            }
        }
        dirty = true;
    }

    if (strcmp(prev_line3, line3) != 0)
	{
		main_oled_FillRectangle(0, 22, 127, 32, Black);
		main_oled_SetCursor(0, 22);
		main_oled_WriteString(line3, Font_7x10, White);
		strcpy(prev_line3, line3);

		if (!dirty)
		{
			dirty_start_page = 2;
			dirty_end_page   = 3;
		}
		else
		{
			if (dirty_start_page > 2)
			{
				dirty_start_page = 2;
			}
			if (dirty_end_page < 3)
			{
				dirty_end_page = 3;
			}
		}
		dirty = true;
	}

    if (dirty)
    {
        main_oled_UpdateScreenPages(dirty_start_page, dirty_end_page);
    }
}
