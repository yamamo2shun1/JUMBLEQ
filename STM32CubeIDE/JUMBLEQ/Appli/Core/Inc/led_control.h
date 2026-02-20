/*
 * led_control.h
 *
 *  Created on: Nov 13, 2025
 *      Author: Shunichi Yamamoto
 */

#ifndef INC_LED_CONTROL_H_
#define INC_LED_CONTROL_H_

#include "main.h"

void update_color_state(void);
void reset_led_buffer(void);

void led_tx_blinking_task(void);
void led_rx_blinking_task(void);

void set_led_color(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
void renew(void);
void rgb_led_task(void);
void led_notify_save_success(void);

#endif /* INC_LED_CONTROL_H_ */
