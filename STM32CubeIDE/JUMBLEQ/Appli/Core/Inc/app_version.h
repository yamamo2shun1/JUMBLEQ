#ifndef INC_APP_VERSION_H_
#define INC_APP_VERSION_H_

#include <stdint.h>

#define APP_VERSION_MAJOR 0
#define APP_VERSION_MINOR 14
#define APP_VERSION_PATCH 4

#define APP_VERSION_STR_HELPER(x) #x
#define APP_VERSION_STR_VALUE(x)  APP_VERSION_STR_HELPER(x)

#define APP_VERSION_STR      APP_VERSION_STR_VALUE(APP_VERSION_MAJOR) "." APP_VERSION_STR_VALUE(APP_VERSION_MINOR) "." APP_VERSION_STR_VALUE(APP_VERSION_PATCH)
#define APP_VERSION_OLED_STR "ver" APP_VERSION_STR

/* USB bcdDevice layout: major.minor = MM.mm (two BCD digits each). */
#if (APP_VERSION_MAJOR < 0) || (APP_VERSION_MAJOR > 99) || \
    (APP_VERSION_MINOR < 0) || (APP_VERSION_MINOR > 99)
#error "APP_VERSION_USB_BCD supports major and minor values from 0 through 99"
#endif

#define APP_VERSION_USB_BCD ((uint16_t) ((((APP_VERSION_MAJOR) / 10) << 12) | \
                                         (((APP_VERSION_MAJOR) % 10) << 8) | \
                                         (((APP_VERSION_MINOR) / 10) << 4) | \
                                         ((APP_VERSION_MINOR) % 10)))

#endif /* INC_APP_VERSION_H_ */
