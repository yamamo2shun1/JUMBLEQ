/*
 * oled_control.c
 *
 *  Created on: 2026/01/26
 *      Author: Shnichi Yamamoto
 */

#include "oled_control.h"

#include "audio_control.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "cmsis_os2.h"

void OLED_Init(void)
{
    ssd1306_Init();
    ssd1306_SetCursor(5, 8);
    ssd1306_WriteString("JUMBLEQ", Font_16x24, White);
    ssd1306_UpdateScreen();
}

void OLED_UpdateTask(void)
{
    char msg[32];

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    snprintf(msg, sizeof(msg), "C2:%ddB Mst:%ddB", get_current_ch2_db(), get_current_master_db());
    ssd1306_WriteString(msg, Font_7x10, White);

    ssd1306_SetCursor(0, 11);
    snprintf(msg, sizeof(msg), "C1:%ddB D/W:%d%%", get_current_ch1_db(), get_current_dry_wet());
    ssd1306_WriteString(msg, Font_7x10, White);

    ssd1306_SetCursor(0, 22);
    ssd1306_WriteString("A:C1(Ln)  B:C2(Ph)", Font_7x10, White);
    ssd1306_UpdateScreen();
}
