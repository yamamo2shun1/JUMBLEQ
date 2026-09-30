/*
 * oled_control.h
 *
 *  Created on: 2026/01/26
 *      Author: Shnichi Yamamoto
 */

#ifndef INC_OLED_CONTROL_H_
#define INC_OLED_CONTROL_H_

void oled_init(void);
void oled_update_task(void);
void oled_show_init_status(const char* text);

#endif /* INC_OLED_CONTROL_H_ */
