/**
 * This Library was originally written by Olivier Van den Eede (4ilo) in 2016.
 * Some refactoring was done and SPI support was added by Aleksander Alekseev (afiskon) in 2018.
 *
 * https://github.com/afiskon/stm32-ssd1306
 */

#ifndef __MAIN_OLED_H__
#define __MAIN_OLED_H__

#include <stddef.h>
#include <stdint.h>
#include <_ansi.h>

_BEGIN_STD_C

#include "ssd1306_params.h"
#include "main_oled_conf.h"

#include "stm32h7rsxx_hal.h"

#ifdef MAIN_OLED_X_OFFSET
    #define MAIN_OLED_X_OFFSET_LOWER (MAIN_OLED_X_OFFSET & 0x0F)
    #define MAIN_OLED_X_OFFSET_UPPER ((MAIN_OLED_X_OFFSET >> 4) & 0x07)
#else
    #define MAIN_OLED_X_OFFSET_LOWER 0
    #define MAIN_OLED_X_OFFSET_UPPER 0
#endif

/* vvv I2C config vvv */

#ifndef MAIN_OLED_I2C_PORT
    #define MAIN_OLED_I2C_PORT hi2c1
#endif

#ifndef MAIN_OLED_I2C_ADDR
    #define MAIN_OLED_I2C_ADDR (0x3C << 1)
#endif

/* ^^^ I2C config ^^^ */

/* vvv SPI config vvv */

#ifndef MAIN_OLED_SPI_PORT
    #define MAIN_OLED_SPI_PORT hspi2
#endif

#ifndef MAIN_OLED_CS_Port
    #define MAIN_OLED_CS_Port GPIOB
#endif
#ifndef MAIN_OLED_CS_Pin
    #define MAIN_OLED_CS_Pin GPIO_PIN_12
#endif

#ifndef MAIN_OLED_DC_Port
    #define MAIN_OLED_DC_Port GPIOB
#endif
#ifndef MAIN_OLED_DC_Pin
    #define MAIN_OLED_DC_Pin GPIO_PIN_14
#endif

#ifndef MAIN_OLED_Reset_Port
    #define MAIN_OLED_Reset_Port GPIOA
#endif
#ifndef MAIN_OLED_Reset_Pin
    #define MAIN_OLED_Reset_Pin GPIO_PIN_8
#endif

/* ^^^ SPI config ^^^ */

#if defined(MAIN_OLED_USE_I2C)
extern I2C_HandleTypeDef MAIN_OLED_I2C_PORT;
#elif defined(MAIN_OLED_USE_SPI)
extern SPI_HandleTypeDef MAIN_OLED_SPI_PORT;
#else
    #error "You should define MAIN_OLED_USE_SPI or MAIN_OLED_USE_I2C macro!"
#endif

// SSD1306 OLED height in pixels
#ifndef MAIN_OLED_HEIGHT
    #define MAIN_OLED_HEIGHT 64
#endif

// SSD1306 width in pixels
#ifndef MAIN_OLED_WIDTH
    #define MAIN_OLED_WIDTH 128
#endif

#ifndef MAIN_OLED_BUFFER_SIZE
    #define MAIN_OLED_BUFFER_SIZE MAIN_OLED_WIDTH* MAIN_OLED_HEIGHT / 8
#endif

// Struct to store transformations
typedef struct
{
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Initialized;
    uint8_t DisplayOn;
} MAIN_OLED_t;

typedef struct
{
    uint8_t x;
    uint8_t y;
} MAIN_OLED_VERTEX;

// Procedure definitions
void main_oled_Init(void);
void main_oled_Fill(SSD1306_COLOR color);
void main_oled_UpdateScreen(void);
void main_oled_UpdateScreenPages(uint8_t start_page, uint8_t end_page);
void main_oled_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char main_oled_WriteChar(char ch, SSD1306_Font_t Font, SSD1306_COLOR color);
char main_oled_WriteString(char* str, SSD1306_Font_t Font, SSD1306_COLOR color);
void main_oled_SetCursor(uint8_t x, uint8_t y);
void main_oled_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void main_oled_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);
void main_oled_DrawArcWithRadiusLine(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);
void main_oled_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR color);
void main_oled_FillCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color);
void main_oled_Polyline(const MAIN_OLED_VERTEX* par_vertex, uint16_t par_size, SSD1306_COLOR color);
void main_oled_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void main_oled_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);

/**
 * @brief Invert color of pixels in rectangle (include border)
 *
 * @param x1 X Coordinate of top left corner
 * @param y1 Y Coordinate of top left corner
 * @param x2 X Coordinate of bottom right corner
 * @param y2 Y Coordinate of bottom right corner
 * @return MAIN_OLED_Error_t status
 */
SSD1306_Error_t main_oled_InvertRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);

void main_oled_DrawBitmap(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color);

/**
 * @brief Sets the contrast of the display.
 * @param[in] value contrast to set.
 * @note Contrast increases as the value increases.
 * @note RESET = 7Fh.
 */
void main_oled_SetContrast(const uint8_t value);

/**
 * @brief Set Display ON/OFF.
 * @param[in] on 0 for OFF, any for ON.
 */
void main_oled_SetDisplayOn(const uint8_t on);

/**
 * @brief Reads DisplayOn state.
 * @return  0: OFF.
 *          1: ON.
 */
uint8_t main_oled_GetDisplayOn();

// Low-level procedures
void main_oled_Reset(void);
void main_oled_WriteCommand(uint8_t byte);
void main_oled_WriteData(uint8_t* buffer, size_t buff_size);
SSD1306_Error_t main_oled_FillBuffer(uint8_t* buf, uint32_t len);

_END_STD_C

#endif  // __MAIN_OLED_H__
