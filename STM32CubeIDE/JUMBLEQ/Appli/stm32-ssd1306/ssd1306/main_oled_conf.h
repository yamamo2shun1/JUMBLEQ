/**
 * Private configuration file for the SSD1306 library.
 * This example is configured for STM32F0, I2C and including all fonts.
 */

#ifndef __MAIN_OLED_CONF_H__
#define __MAIN_OLED_CONF_H__

// Choose a microcontroller family
// #define STM32F0
// #define STM32F1
// #define STM32F4
// #define STM32L0
// #define STM32L1
// #define STM32L4
// #define STM32F3
// #define STM32H7
#define STM32H7RS
// #define STM32F7
// #define STM32G0
// #define STM32C0
// #define STM32U5

// Choose a bus
#define MAIN_OLED_USE_I2C
// #define MAIN_OLED_USE_SPI

// I2C Configuration
#define MAIN_OLED_I2C_PORT hi2c3
#define MAIN_OLED_I2C_ADDR (0x3C << 1)

// SPI Configuration
// #define MAIN_OLED_SPI_PORT        hspi1
// #define MAIN_OLED_CS_Port         OLED_CS_GPIO_Port
// #define MAIN_OLED_CS_Pin          OLED_CS_Pin
// #define MAIN_OLED_DC_Port         OLED_DC_GPIO_Port
// #define MAIN_OLED_DC_Pin          OLED_DC_Pin
// #define MAIN_OLED_Reset_Port      OLED_Res_GPIO_Port
// #define MAIN_OLED_Reset_Pin       OLED_Res_Pin

// Mirror the screen if needed
// #define MAIN_OLED_MIRROR_VERT
// #define MAIN_OLED_MIRROR_HORIZ

// Set inverse color if needed
// # define MAIN_OLED_INVERSE_COLOR

// Include only needed fonts
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26

#define SSD1306_INCLUDE_FONT_16x24

#define SSD1306_INCLUDE_FONT_16x15

// The width of the screen can be set using this
// define. The default value is 128.
#define MAIN_OLED_WIDTH 128

// If your screen horizontal axis does not start
// in column 0 you can use this define to
// adjust the horizontal offset
// #define MAIN_OLED_X_OFFSET

// The height can be changed as well if necessary.
// It can be 32, 64 or 128. The default value is 64.
#define MAIN_OLED_HEIGHT 32

#endif /* __MAIN_OLED_CONF_H__ */
