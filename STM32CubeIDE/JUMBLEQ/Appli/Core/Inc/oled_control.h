/*
 * oled_control.h
 *
 *  Created on: 2026/01/26
 *      Author: Shnichi Yamamoto
 */

#ifndef INC_OLED_CONTROL_H_
#define INC_OLED_CONTROL_H_

#include "main.h"

void OLED_Init(void);
void OLED_UpdateTask(void);
void OLED_ShowInitStatus(const char* text);

#endif /* INC_OLED_CONTROL_H_ */
