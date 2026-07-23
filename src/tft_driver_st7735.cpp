/**
  Author: sascha_lammers@gmx.de

  Baremetal SPI driver for ST7735 - DMA TX based
*/

#if ARDUINO
#include <Arduino.h>
#endif
#include <stm32f1xx.h>
#include "tft_driver.h"

#if TFT_DRIVER == TFT_DRIVER_ST7735

/* Display color modes */
#define ST7735_COLMOD_65K 0x05
#define ST7735_COLMOD_262K 0x06

/* ST7735 commands */
#define ST7735_NOP 0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID 0x04
#define ST7735_RDDST 0x09
#define ST7735_SLPIN 0x10
#define ST7735_SLPOUT 0x11
#define ST7735_PTLON 0x12
#define ST7735_NORON 0x13
#define ST7735_INVOFF 0x20
#define ST7735_INVON 0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON 0x29
#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C 
#define ST7735_COLMOD 0x3A
#define ST7735_FRMCTR1 0xB1
#define ST7735_MADCTL 0x36
#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_VMCTR1 0xC5
#define ST7735_PWCTR6 0xFC
#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

static uint8_t g_tft_color_chunk[TFT_DMA_TX_CHUNK_PIXELS * 2];
static uint8_t g_tft_pixel_chunk[TFT_DMA_TX_CHUNK_PIXELS * 2];

static void set_column(uint16_t x0, uint16_t x1);
static void set_row(uint16_t y0, uint16_t y1);

/**
 * Write repeated RGB565 color pixels using chunked DMA transfers.
 */
static void write_color_pixels(uint16_t color, uint32_t pixels) 
{
    uint8_t high = (uint8_t)((color >> 8) & 0xFF);
    uint8_t low = (uint8_t)(color & 0xFF);

    for (uint16_t i = 0; i < TFT_DMA_TX_CHUNK_PIXELS; i++) {
        g_tft_color_chunk[(2U * i)] = high;
        g_tft_color_chunk[(2U * i) + 1U] = low;
    }

    TFT_PIN_RS_HIGH();
    TFT_PIN_CS_LOW();
    tft_driver_delay();

    while (pixels > 0) {
        uint16_t chunk_pixels = (pixels > TFT_DMA_TX_CHUNK_PIXELS) ? TFT_DMA_TX_CHUNK_PIXELS : (uint16_t)pixels;
        tft_driver_spi_send_buffer_dma_raw(g_tft_color_chunk, (uint16_t)(chunk_pixels * 2U));
        pixels -= chunk_pixels;
    }

    tft_driver_delay();
    TFT_PIN_CS_HIGH();
}

/**
 * Write an RGB565 pixel buffer as big-endian bytes using chunked DMA transfers.
 */
static void write_pixel_buffer_rgb565(const uint16_t *pixels, uint32_t pixel_count) 
{
    if (!pixels || pixel_count == 0) {
        return;
    }

    TFT_PIN_RS_HIGH();
    TFT_PIN_CS_LOW();
    tft_driver_delay();

    uint32_t offset = 0;
    while (offset < pixel_count) {
        uint16_t chunk_pixels = (pixel_count - offset > TFT_DMA_TX_CHUNK_PIXELS)
                                    ? TFT_DMA_TX_CHUNK_PIXELS
                                    : (uint16_t)(pixel_count - offset);

        for (uint16_t i = 0; i < chunk_pixels; i++) {
            uint16_t c = pixels[offset + i];
            g_tft_pixel_chunk[(2U * i)] = (uint8_t)((c >> 8) & 0xFF);
            g_tft_pixel_chunk[(2U * i) + 1U] = (uint8_t)(c & 0xFF);
        }

        tft_driver_spi_send_buffer_dma_raw(g_tft_pixel_chunk, (uint16_t)(chunk_pixels * 2U));
        offset += chunk_pixels;
    }

    tft_driver_delay();
    TFT_PIN_CS_HIGH();
}

void tft_write_window_pixels(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint16_t *pixels, uint32_t pixel_count) 
{
    set_column(x0, x1);
    set_row(y0, y1);
    tft_driver_send_command(ST7735_RAMWR);
    write_pixel_buffer_rgb565(pixels, pixel_count);
}

/**
 * Set column address range
 */
static void set_column(uint16_t x0, uint16_t x1) 
{
    uint8_t data[4] = {(uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF), 
                       (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF)};
    tft_driver_send_command(ST7735_CASET);
    tft_driver_send_data(data, 4);
}

/**
 * Set row address range
 */
static void set_row(uint16_t y0, uint16_t y1) 
{
    uint8_t data[4] = {(uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF), 
                       (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF)};
    tft_driver_send_command(ST7735_RASET);
    tft_driver_send_data(data, 4);
}

/**
 * Initialize ST7735 display
 */
static void st7735_init(void) 
{
    /* Hardware reset */
    TFT_PIN_RST_LOW();
    HAL_Delay(25);
    TFT_PIN_RST_HIGH();
    HAL_Delay(60);

    /* Software reset */
    tft_driver_send_command(ST7735_SWRESET);
    HAL_Delay(75);

    /* Exit sleep mode */
    tft_driver_send_command(ST7735_SLPOUT);
    HAL_Delay(60);

    /* Frame rate control */
    tft_driver_send_command(ST7735_FRMCTR1);
    uint8_t frmctr[] = {0x01, 0x2C, 0x2D};
    tft_driver_send_data(frmctr, 3);
    HAL_Delay(5);

    /* Memory access control */
    tft_driver_send_command(ST7735_MADCTL);
    uint8_t madctl[] = {0xA0};
    tft_driver_send_data(madctl, 1);

    /* Color mode: 16-bit */
    tft_driver_send_command(ST7735_COLMOD);
    uint8_t colmod[] = {0x05};
    tft_driver_send_data(colmod, 1);
    HAL_Delay(5);

    /* Power control */
    tft_driver_send_command(ST7735_PWCTR1);
    uint8_t pwctr1[] = {0xA2, 0x02, 0x84};
    tft_driver_send_data(pwctr1, 3);

    tft_driver_send_command(ST7735_PWCTR2);
    uint8_t pwctr2[] = {0xC5};
    tft_driver_send_data(pwctr2, 1);

    tft_driver_send_command(ST7735_PWCTR3);
    uint8_t pwctr3[] = {0x0A, 0x00};
    tft_driver_send_data(pwctr3, 2);

    tft_driver_send_command(ST7735_VMCTR1);
    uint8_t vmctr1[] = {0x0E};
    tft_driver_send_data(vmctr1, 1);
    HAL_Delay(5);

    /* Gamma */
    tft_driver_send_command(ST7735_GMCTRP1);
    uint8_t gp[] = {0x10, 0x0E, 0x02, 0x03, 0x0E, 0x07, 0x02, 0x07, 0x0A, 0x12, 0x27, 0x37, 0x00, 0x0D, 0x0E, 0x10};
    tft_driver_send_data(gp, 16);

    tft_driver_send_command(ST7735_GMCTRN1);
    uint8_t gn[] = {0x10, 0x0E, 0x03, 0x03, 0x0F, 0x06, 0x02, 0x08, 0x0A, 0x13, 0x26, 0x36, 0x00, 0x0D, 0x0E, 0x10};
    tft_driver_send_data(gn, 16);
    HAL_Delay(5);

    /* Exit invert */
    tft_driver_send_command(ST7735_INVOFF);
    HAL_Delay(5);

    /* Display on */
    tft_driver_send_command(ST7735_DISPON);
    HAL_Delay(50);
}

/**
 * Initialize driver
 */
void tft_driver_init(void) 
{
    // === init GPIO pins ===
    tft_driver_gpio_init();

    // === SPI setup ===
    tft_driver_spi_init();

    // === Display setup ===
    st7735_init();
}

/**
 * Clear display
 */
void tft_clear_display(uint16_t color) 
{
    set_column(0, TFT_DIM_WIDTH - 1);
    set_row(0, TFT_DIM_HEIGHT - 1);
    tft_driver_send_command(ST7789_RAMWR);

    write_color_pixels(color, (uint32_t)TFT_DIM_WIDTH * TFT_DIM_HEIGHT);
}

#endif // TFT_DRIVER == TFT_DRIVER_ST7735
