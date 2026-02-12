#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <stdint.h>

typedef enum {
  LED_STATUS_BOOT_JUMP = 0,
  LED_STATUS_UF2_IDLE,
  LED_STATUS_UF2_WRITE,
  LED_STATUS_VERIFY,
  LED_STATUS_VERIFY_OK,
  LED_STATUS_ERROR,
  LED_STATUS_USB_MISSING
} led_status_t;

void led_status_set(led_status_t status);
void led_status_tick(uint32_t now_ms);

#endif /* LED_STATUS_H */
