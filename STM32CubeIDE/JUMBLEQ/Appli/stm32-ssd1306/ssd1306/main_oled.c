#include <main_oled.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>  // For memcpy

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "semphr.h"

// I2C排他制御用ミューテックス
extern osMutexId_t i2cMutexHandle;

static uint8_t main_oled_cmd_byte = 0;  // コマンド用バッファ

#if defined(MAIN_OLED_USE_I2C)

// I2C送信（ブロッキングモード、排他制御付き）
static HAL_StatusTypeDef main_oled_I2C_Write(uint16_t mem_addr, uint8_t* data, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (i2cMutexHandle == NULL)
    {
        return HAL_ERROR;
    }

    // ミューテックスで排他制御（最大200ms待機）
    if (osMutexAcquire(i2cMutexHandle, pdMS_TO_TICKS(200)) == osOK)
    {
        // ブロッキングモードで送信（タイムアウト100ms）
        status = HAL_I2C_Mem_Write(&MAIN_OLED_I2C_PORT, MAIN_OLED_I2C_ADDR, mem_addr, 1, data, size, 100);

        osMutexRelease(i2cMutexHandle);
    }

    return status;
}

void main_oled_Reset(void)
{
    /* for I2C - do nothing */
}

// Send a byte to the command register
void main_oled_WriteCommand(uint8_t byte)
{
    main_oled_cmd_byte = byte;  // staticバッファにコピー
    main_oled_I2C_Write(0x00, &main_oled_cmd_byte, 1);
}

// Send data
void main_oled_WriteData(uint8_t* buffer, size_t buff_size)
{
    main_oled_I2C_Write(0x40, buffer, (uint16_t) buff_size);
}

#elif defined(MAIN_OLED_USE_SPI)

void main_oled_Reset(void)
{
    // CS = High (not selected)
    HAL_GPIO_WritePin(MAIN_OLED_CS_Port, MAIN_OLED_CS_Pin, GPIO_PIN_SET);

    // Reset the OLED
    HAL_GPIO_WritePin(MAIN_OLED_Reset_Port, MAIN_OLED_Reset_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(MAIN_OLED_Reset_Port, MAIN_OLED_Reset_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

// Send a byte to the command register
void main_oled_WriteCommand(uint8_t byte)
{
    HAL_GPIO_WritePin(MAIN_OLED_CS_Port, MAIN_OLED_CS_Pin, GPIO_PIN_RESET);  // select OLED
    HAL_GPIO_WritePin(MAIN_OLED_DC_Port, MAIN_OLED_DC_Pin, GPIO_PIN_RESET);  // command
    HAL_SPI_Transmit(&MAIN_OLED_SPI_PORT, (uint8_t*) &byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(MAIN_OLED_CS_Port, MAIN_OLED_CS_Pin, GPIO_PIN_SET);  // un-select OLED
}

// Send data
void main_oled_WriteData(uint8_t* buffer, size_t buff_size)
{
    HAL_GPIO_WritePin(MAIN_OLED_CS_Port, MAIN_OLED_CS_Pin, GPIO_PIN_RESET);  // select OLED
    HAL_GPIO_WritePin(MAIN_OLED_DC_Port, MAIN_OLED_DC_Pin, GPIO_PIN_SET);    // data
    HAL_SPI_Transmit(&MAIN_OLED_SPI_PORT, buffer, buff_size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(MAIN_OLED_CS_Port, MAIN_OLED_CS_Pin, GPIO_PIN_SET);  // un-select OLED
}

#else
    #error "You should define MAIN_OLED_USE_SPI or MAIN_OLED_USE_I2C macro"
#endif

// Screenbuffer
static uint8_t MAIN_OLED_Buffer[MAIN_OLED_BUFFER_SIZE];

// Screen object
static MAIN_OLED_t MAIN_OLED;

/* Fills the Screenbuffer with values from a given buffer of a fixed length */
SSD1306_Error_t main_oled_FillBuffer(uint8_t* buf, uint32_t len)
{
    SSD1306_Error_t ret = SSD1306_ERR;
    if (len <= MAIN_OLED_BUFFER_SIZE)
    {
        memcpy(MAIN_OLED_Buffer, buf, len);
        ret = SSD1306_OK;
    }
    return ret;
}

/* Initialize the oled screen */
void main_oled_Init(void)
{
    // Reset OLED
    main_oled_Reset();

    // Wait for the screen to boot
    HAL_Delay(100);

    // Init OLED
    main_oled_SetDisplayOn(0);  // display off

    main_oled_WriteCommand(0x20);  // Set Memory Addressing Mode
    main_oled_WriteCommand(0x00);  // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
                                   // 10b,Page Addressing Mode (RESET); 11b,Invalid

    main_oled_WriteCommand(0xB0);  // Set Page Start Address for Page Addressing Mode,0-7

#ifdef MAIN_OLED_MIRROR_VERT
    main_oled_WriteCommand(0xC0);  // Mirror vertically
#else
    main_oled_WriteCommand(0xC8);  // Set COM Output Scan Direction
#endif

    main_oled_WriteCommand(0x00);  //---set low column address
    main_oled_WriteCommand(0x10);  //---set high column address

    main_oled_WriteCommand(0x40);  //--set start line address - CHECK

    main_oled_SetContrast(0xFF);

#ifdef MAIN_OLED_MIRROR_HORIZ
    main_oled_WriteCommand(0xA0);  // Mirror horizontally
#else
    main_oled_WriteCommand(0xA1);  //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef MAIN_OLED_INVERSE_COLOR
    main_oled_WriteCommand(0xA7);  //--set inverse color
#else
    main_oled_WriteCommand(0xA6);  //--set normal color
#endif

// Set multiplex ratio.
#if (MAIN_OLED_HEIGHT == 128)
    // Found in the Luma Python lib for SH1106.
    main_oled_WriteCommand(0xFF);
#else
    main_oled_WriteCommand(0xA8);  //--set multiplex ratio(1 to 64) - CHECK
#endif

#if (MAIN_OLED_HEIGHT == 32)
    main_oled_WriteCommand(0x1F);  //
#elif (MAIN_OLED_HEIGHT == 64)
    main_oled_WriteCommand(0x3F);  //
#elif (MAIN_OLED_HEIGHT == 128)
    main_oled_WriteCommand(0x3F);  // Seems to work for 128px high displays too.
#else
    #error "Only 32, 64, or 128 lines of height are supported!"
#endif

    main_oled_WriteCommand(0xA4);  // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    main_oled_WriteCommand(0xD3);  //-set display offset - CHECK
    main_oled_WriteCommand(0x00);  //-not offset

    main_oled_WriteCommand(0xD5);  //--set display clock divide ratio/oscillator frequency
    main_oled_WriteCommand(0xF0);  //--set divide ratio

    main_oled_WriteCommand(0xD9);  //--set pre-charge period
    main_oled_WriteCommand(0x22);  //

    main_oled_WriteCommand(0xDA);  //--set com pins hardware configuration - CHECK
#if (MAIN_OLED_HEIGHT == 32)
    main_oled_WriteCommand(0x02);
#elif (MAIN_OLED_HEIGHT == 64)
    main_oled_WriteCommand(0x12);
#elif (MAIN_OLED_HEIGHT == 128)
    main_oled_WriteCommand(0x12);
#else
    #error "Only 32, 64, or 128 lines of height are supported!"
#endif

    main_oled_WriteCommand(0xDB);  //--set vcomh
    main_oled_WriteCommand(0x20);  // 0x20,0.77xVcc

    main_oled_WriteCommand(0x8D);  //--set DC-DC enable
    main_oled_WriteCommand(0x14);  //
    main_oled_SetDisplayOn(1);     //--turn on SSD1306 panel

    // Clear screen
    main_oled_Fill(Black);

    // Flush buffer to screen
    main_oled_UpdateScreen();

    // Set default values for screen object
    MAIN_OLED.CurrentX = 0;
    MAIN_OLED.CurrentY = 0;

    MAIN_OLED.Initialized = 1;
}

/* Fill the whole screen with the given color */
void main_oled_Fill(SSD1306_COLOR color)
{
    memset(MAIN_OLED_Buffer, (color == Black) ? 0x00 : 0xFF, sizeof(MAIN_OLED_Buffer));
}

/* Write the screenbuffer with changed to the screen */
void main_oled_UpdateScreen(void)
{
    // Write data to each page of RAM. Number of pages
    // depends on the screen height:
    //
    //  * 32px   ==  4 pages
    //  * 64px   ==  8 pages
    //  * 128px  ==  16 pages
    for (uint8_t i = 0; i < MAIN_OLED_HEIGHT / 8; i++)
    {
        main_oled_WriteCommand(0xB0 + i);  // Set the current RAM page address.
        main_oled_WriteCommand(0x00 + MAIN_OLED_X_OFFSET_LOWER);
        main_oled_WriteCommand(0x10 + MAIN_OLED_X_OFFSET_UPPER);
        main_oled_WriteData(&MAIN_OLED_Buffer[MAIN_OLED_WIDTH * i], MAIN_OLED_WIDTH);
    }
}

/*
 * Draw one pixel in the screenbuffer
 * X => X Coordinate
 * Y => Y Coordinate
 * color => Pixel color
 */
void main_oled_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    if (x >= MAIN_OLED_WIDTH || y >= MAIN_OLED_HEIGHT)
    {
        // Don't write outside the buffer
        return;
    }

    // Draw in the right color
    if (color == White)
    {
        MAIN_OLED_Buffer[x + (y / 8) * MAIN_OLED_WIDTH] |= 1 << (y % 8);
    }
    else
    {
        MAIN_OLED_Buffer[x + (y / 8) * MAIN_OLED_WIDTH] &= ~(1 << (y % 8));
    }
}

/*
 * Draw 1 char to the screen buffer
 * ch       => char om weg te schrijven
 * Font     => Font waarmee we gaan schrijven
 * color    => Black or White
 */
char main_oled_WriteChar(char ch, SSD1306_Font_t Font, SSD1306_COLOR color)
{
    uint32_t i, b, j;

    // Check if character is valid
    if (ch < 32 || ch > 126)
        return 0;

    // Char width is not equal to font width for proportional font
    const uint8_t char_width = Font.char_width ? Font.char_width[ch - 32] : Font.width;
    // Check remaining space on current line
    if (MAIN_OLED_WIDTH < (MAIN_OLED.CurrentX + char_width) ||
        MAIN_OLED_HEIGHT < (MAIN_OLED.CurrentY + Font.height))
    {
        // Not enough space on current line
        return 0;
    }

    // Use the font to write
    for (i = 0; i < Font.height; i++)
    {
        b = Font.data[(ch - 32) * Font.height + i];
        for (j = 0; j < char_width; j++)
        {
            if ((b << j) & 0x8000)
            {
                main_oled_DrawPixel(MAIN_OLED.CurrentX + j, (MAIN_OLED.CurrentY + i), (SSD1306_COLOR) color);
            }
            else
            {
                main_oled_DrawPixel(MAIN_OLED.CurrentX + j, (MAIN_OLED.CurrentY + i), (SSD1306_COLOR) !color);
            }
        }
    }

    // The current space is now taken
    MAIN_OLED.CurrentX += char_width;

    // Return written char for validation
    return ch;
}

/* Write full string to screenbuffer */
char main_oled_WriteString(char* str, SSD1306_Font_t Font, SSD1306_COLOR color)
{
    while (*str)
    {
        if (main_oled_WriteChar(*str, Font, color) != *str)
        {
            // Char could not be written
            return *str;
        }
        str++;
    }

    // Everything ok
    return *str;
}

/* Position the cursor */
void main_oled_SetCursor(uint8_t x, uint8_t y)
{
    MAIN_OLED.CurrentX = x;
    MAIN_OLED.CurrentY = y;
}

/* Draw line by Bresenhem's algorithm */
void main_oled_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    int32_t deltaX = abs(x2 - x1);
    int32_t deltaY = abs(y2 - y1);
    int32_t signX  = ((x1 < x2) ? 1 : -1);
    int32_t signY  = ((y1 < y2) ? 1 : -1);
    int32_t error  = deltaX - deltaY;
    int32_t error2;

    main_oled_DrawPixel(x2, y2, color);

    while ((x1 != x2) || (y1 != y2))
    {
        main_oled_DrawPixel(x1, y1, color);
        error2 = error * 2;
        if (error2 > -deltaY)
        {
            error -= deltaY;
            x1 += signX;
        }

        if (error2 < deltaX)
        {
            error += deltaX;
            y1 += signY;
        }
    }
    return;
}

/* Draw polyline */
void main_oled_Polyline(const MAIN_OLED_VERTEX* par_vertex, uint16_t par_size, SSD1306_COLOR color)
{
    uint16_t i;
    if (par_vertex == NULL)
    {
        return;
    }

    for (i = 1; i < par_size; i++)
    {
        main_oled_Line(par_vertex[i - 1].x, par_vertex[i - 1].y, par_vertex[i].x, par_vertex[i].y, color);
    }

    return;
}

/* Convert Degrees to Radians */
static float main_oled_DegToRad(float par_deg)
{
    return par_deg * (3.14f / 180.0f);
}

/* Normalize degree to [0;360] */
static uint16_t main_oled_NormalizeTo0_360(uint16_t par_deg)
{
    uint16_t loc_angle;
    if (par_deg <= 360)
    {
        loc_angle = par_deg;
    }
    else
    {
        loc_angle = par_deg % 360;
        loc_angle = (loc_angle ? loc_angle : 360);
    }
    return loc_angle;
}

/*
 * DrawArc. Draw angle is beginning from 4 quart of trigonometric circle (3pi/2)
 * start_angle in degree
 * sweep in degree
 */
void main_oled_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color)
{
    static const uint8_t CIRCLE_APPROXIMATION_SEGMENTS = 36;
    float approx_degree;
    uint32_t approx_segments;
    uint8_t xp1, xp2;
    uint8_t yp1, yp2;
    uint32_t count;
    uint32_t loc_sweep;
    float rad;

    loc_sweep = main_oled_NormalizeTo0_360(sweep);

    count           = (main_oled_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_degree   = loc_sweep / (float) approx_segments;
    while (count < approx_segments)
    {
        rad = main_oled_DegToRad(count * approx_degree);
        xp1 = x + (int8_t) (sinf(rad) * radius);
        yp1 = y + (int8_t) (cosf(rad) * radius);
        count++;
        if (count != approx_segments)
        {
            rad = main_oled_DegToRad(count * approx_degree);
        }
        else
        {
            rad = main_oled_DegToRad(loc_sweep);
        }
        xp2 = x + (int8_t) (sinf(rad) * radius);
        yp2 = y + (int8_t) (cosf(rad) * radius);
        main_oled_Line(xp1, yp1, xp2, yp2, color);
    }

    return;
}

/*
 * Draw arc with radius line
 * Angle is beginning from 4 quart of trigonometric circle (3pi/2)
 * start_angle: start angle in degree
 * sweep: finish angle in degree
 */
void main_oled_DrawArcWithRadiusLine(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color)
{
    const uint32_t CIRCLE_APPROXIMATION_SEGMENTS = 36;
    float approx_degree;
    uint32_t approx_segments;
    uint8_t xp1;
    uint8_t xp2 = 0;
    uint8_t yp1;
    uint8_t yp2 = 0;
    uint32_t count;
    uint32_t loc_sweep;
    float rad;

    loc_sweep = main_oled_NormalizeTo0_360(sweep);

    count           = (main_oled_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_degree   = loc_sweep / (float) approx_segments;

    rad                   = main_oled_DegToRad(count * approx_degree);
    uint8_t first_point_x = x + (int8_t) (sinf(rad) * radius);
    uint8_t first_point_y = y + (int8_t) (cosf(rad) * radius);
    while (count < approx_segments)
    {
        rad = main_oled_DegToRad(count * approx_degree);
        xp1 = x + (int8_t) (sinf(rad) * radius);
        yp1 = y + (int8_t) (cosf(rad) * radius);
        count++;
        if (count != approx_segments)
        {
            rad = main_oled_DegToRad(count * approx_degree);
        }
        else
        {
            rad = main_oled_DegToRad(loc_sweep);
        }
        xp2 = x + (int8_t) (sinf(rad) * radius);
        yp2 = y + (int8_t) (cosf(rad) * radius);
        main_oled_Line(xp1, yp1, xp2, yp2, color);
    }

    // Radius line
    main_oled_Line(x, y, first_point_x, first_point_y, color);
    main_oled_Line(x, y, xp2, yp2, color);
    return;
}

/* Draw circle by Bresenhem's algorithm */
void main_oled_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color)
{
    int32_t x   = -par_r;
    int32_t y   = 0;
    int32_t err = 2 - 2 * par_r;
    int32_t e2;

    if (par_x >= MAIN_OLED_WIDTH || par_y >= MAIN_OLED_HEIGHT)
    {
        return;
    }

    do
    {
        main_oled_DrawPixel(par_x - x, par_y + y, par_color);
        main_oled_DrawPixel(par_x + x, par_y + y, par_color);
        main_oled_DrawPixel(par_x + x, par_y - y, par_color);
        main_oled_DrawPixel(par_x - x, par_y - y, par_color);
        e2 = err;

        if (e2 <= y)
        {
            y++;
            err = err + (y * 2 + 1);
            if (-x == y && e2 <= x)
            {
                e2 = 0;
            }
        }

        if (e2 > x)
        {
            x++;
            err = err + (x * 2 + 1);
        }
    } while (x <= 0);

    return;
}

/* Draw filled circle. Pixel positions calculated using Bresenham's algorithm */
void main_oled_FillCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color)
{
    int32_t x   = -par_r;
    int32_t y   = 0;
    int32_t err = 2 - 2 * par_r;
    int32_t e2;

    if (par_x >= MAIN_OLED_WIDTH || par_y >= MAIN_OLED_HEIGHT)
    {
        return;
    }

    do
    {
        for (uint8_t _y = (par_y + y); _y >= (par_y - y); _y--)
        {
            for (uint8_t _x = (par_x - x); _x >= (par_x + x); _x--)
            {
                main_oled_DrawPixel(_x, _y, par_color);
            }
        }

        e2 = err;
        if (e2 <= y)
        {
            y++;
            err = err + (y * 2 + 1);
            if (-x == y && e2 <= x)
            {
                e2 = 0;
            }
        }

        if (e2 > x)
        {
            x++;
            err = err + (x * 2 + 1);
        }
    } while (x <= 0);

    return;
}

/* Draw a rectangle */
void main_oled_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    main_oled_Line(x1, y1, x2, y1, color);
    main_oled_Line(x2, y1, x2, y2, color);
    main_oled_Line(x2, y2, x1, y2, color);
    main_oled_Line(x1, y2, x1, y1, color);

    return;
}

/* Draw a filled rectangle */
void main_oled_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    uint8_t x_start = ((x1 <= x2) ? x1 : x2);
    uint8_t x_end   = ((x1 <= x2) ? x2 : x1);
    uint8_t y_start = ((y1 <= y2) ? y1 : y2);
    uint8_t y_end   = ((y1 <= y2) ? y2 : y1);

    for (uint8_t y = y_start; (y <= y_end) && (y < MAIN_OLED_HEIGHT); y++)
    {
        for (uint8_t x = x_start; (x <= x_end) && (x < MAIN_OLED_WIDTH); x++)
        {
            main_oled_DrawPixel(x, y, color);
        }
    }
    return;
}

SSD1306_Error_t main_oled_InvertRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    if ((x2 >= MAIN_OLED_WIDTH) || (y2 >= MAIN_OLED_HEIGHT))
    {
        return SSD1306_ERR;
    }
    if ((x1 > x2) || (y1 > y2))
    {
        return SSD1306_ERR;
    }
    uint32_t i;
    if ((y1 / 8) != (y2 / 8))
    {
        /* if rectangle doesn't lie on one 8px row */
        for (uint32_t x = x1; x <= x2; x++)
        {
            i = x + (y1 / 8) * MAIN_OLED_WIDTH;
            MAIN_OLED_Buffer[i] ^= 0xFF << (y1 % 8);
            i += MAIN_OLED_WIDTH;
            for (; i < x + (y2 / 8) * MAIN_OLED_WIDTH; i += MAIN_OLED_WIDTH)
            {
                MAIN_OLED_Buffer[i] ^= 0xFF;
            }
            MAIN_OLED_Buffer[i] ^= 0xFF >> (7 - (y2 % 8));
        }
    }
    else
    {
        /* if rectangle lies on one 8px row */
        const uint8_t mask = (0xFF << (y1 % 8)) & (0xFF >> (7 - (y2 % 8)));
        for (i = x1 + (y1 / 8) * MAIN_OLED_WIDTH;
             i <= (uint32_t) x2 + (y2 / 8) * MAIN_OLED_WIDTH; i++)
        {
            MAIN_OLED_Buffer[i] ^= mask;
        }
    }
    return SSD1306_OK;
}

/* Draw a bitmap */
void main_oled_DrawBitmap(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color)
{
    int16_t byteWidth = (w + 7) / 8;  // Bitmap scanline pad = whole byte
    uint8_t byte      = 0;

    if (x >= MAIN_OLED_WIDTH || y >= MAIN_OLED_HEIGHT)
    {
        return;
    }

    for (uint8_t j = 0; j < h; j++, y++)
    {
        for (uint8_t i = 0; i < w; i++)
        {
            if (i & 7)
            {
                byte <<= 1;
            }
            else
            {
                byte = (*(const unsigned char*) (&bitmap[j * byteWidth + i / 8]));
            }

            if (byte & 0x80)
            {
                main_oled_DrawPixel(x + i, y, color);
            }
        }
    }
    return;
}

void main_oled_SetContrast(const uint8_t value)
{
    const uint8_t kSetContrastControlRegister = 0x81;
    main_oled_WriteCommand(kSetContrastControlRegister);
    main_oled_WriteCommand(value);
}

void main_oled_SetDisplayOn(const uint8_t on)
{
    uint8_t value;
    if (on)
    {
        value               = 0xAF;  // Display on
        MAIN_OLED.DisplayOn = 1;
    }
    else
    {
        value               = 0xAE;  // Display off
        MAIN_OLED.DisplayOn = 0;
    }
    main_oled_WriteCommand(value);
}

uint8_t main_oled_GetDisplayOn()
{
    return MAIN_OLED.DisplayOn;
}
