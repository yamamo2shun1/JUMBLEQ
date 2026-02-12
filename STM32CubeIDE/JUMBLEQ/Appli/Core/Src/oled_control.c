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

void OLED_Init(void)
{
    main_oled_Init();
    main_oled_SetCursor(5, 8);
    main_oled_WriteString("JUMBLEQ", Font_16x24, White);
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
    char msg[32];

    main_oled_Fill(Black);
    main_oled_SetCursor(0, 0);
    snprintf(msg, sizeof(msg), "C2:%ddB Mst:%ddB", get_current_ch2_db(), get_current_master_db());
    main_oled_WriteString(msg, Font_7x10, White);

    main_oled_SetCursor(0, 11);
    snprintf(msg, sizeof(msg), "C1:%ddB D/W:%d%%", get_current_ch1_db(), get_current_dry_wet());
    main_oled_WriteString(msg, Font_7x10, White);

    main_oled_SetCursor(0, 22);
    main_oled_WriteString("A:C1(Ln)  B:C2(Ph)", Font_7x10, White);
    main_oled_UpdateScreen();
}
