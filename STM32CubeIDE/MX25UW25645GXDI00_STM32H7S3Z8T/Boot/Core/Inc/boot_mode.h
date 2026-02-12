#ifndef BOOT_MODE_H
#define BOOT_MODE_H

#include <stdbool.h>

bool boot_mode_sw2_pressed(void);
bool boot_mode_usb_vbus_present(void);
bool boot_mode_should_enter_uf2(void);

#endif /* BOOT_MODE_H */
