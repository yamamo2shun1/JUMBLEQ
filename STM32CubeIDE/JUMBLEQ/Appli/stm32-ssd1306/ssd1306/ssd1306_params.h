/*
 * ssd1306_params.h
 *
 *  Created on: 2026/02/10
 *      Author: Shnichi Yamamoto
 */

#ifndef INC_SSD1306_PARAMS_H_
#define INC_SSD1306_PARAMS_H_

// Enumeration for screen colors
typedef enum
{
    Black = 0x00,  // Black color, no pixel
    White = 0x01   // Pixel is set. Color depends on OLED
} SSD1306_COLOR;

typedef enum
{
    SSD1306_OK  = 0x00,
    SSD1306_ERR = 0x01  // Generic error.
} SSD1306_Error_t;

/** Font */
typedef struct
{
    const uint8_t width;             /**< Font width in pixels */
    const uint8_t height;            /**< Font height in pixels */
    const uint16_t* const data;      /**< Pointer to font data array */
    const uint8_t* const char_width; /**< Proportional character width in pixels (NULL for monospaced) */
} SSD1306_Font_t;

#endif /* INC_SSD1306_PARAMS_H_ */
