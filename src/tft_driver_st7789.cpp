/**
  Author: sascha_lammers@gmx.de

  Baremetal SPI driver for ST7789 - DMA TX based
*/

#include <stm32f1xx.h>
#include <algorithm>
#include <memory>
#include "tft_driver.h"

#if TFT_DRIVER == TFT_DRIVER_ST7789

// Display offset for landscape
#define ST7789_COL_OFS              40
#define ST7789_ROW_OFS              53

// Display color modes
#define ST7789_COLMOD_65K           0x05
#define ST7789_COLMOD_262K          0x06

// ST7789 commands
#define ST7789_NOP                  0x00
#define ST7789_SWRESET              0x01
#define ST7789_RDDID                0x04
#define ST7789_RDDST                0x09
#define ST7789_SLPIN                0x10
#define ST7789_SLPOUT               0x11
#define ST7789_PTLON                0x12
#define ST7789_NORON                0x13
#define ST7789_INVOFF               0x20
#define ST7789_INVON                0x21
#define ST7789_DISPOFF              0x28
#define ST7789_DISPON               0x29
#define ST7789_CASET                0x2A
#define ST7789_RASET                0x2B
#define ST7789_RAMWR                0x2C
#define ST7789_COLMOD               0x3A
#define ST7789_FRMCTR1              0xB1
#define ST7789_MADCTL               0x36
#define ST7789_PWCTR1               0xC0
#define ST7789_PWCTR2               0xC1
#define ST7789_PWCTR3               0xC2
#define ST7789_VMCTR1               0xC5
#define ST7789_PWCTR6               0xFC
#define ST7789_GMCTRP1              0xE0
#define ST7789_GMCTRN1              0xE1

static uint32_t dma_transfer_buffer[TFT_DMA_TX_CHUNK_PIXELS / 2];

static inline void set_column(uint16_t x0, uint16_t x1);
static inline void set_row(uint16_t y0, uint16_t y1);

/**
 * Write repeated RGB565 color pixels using chunked DMA transfers.
 */
static void write_color_pixels(uint16_t color, uint32_t pixels)
{
    const uint32_t pixel_pattern = __REV16((static_cast<uint32_t>(color) << 16U) | color);
    std::fill(std::begin(dma_transfer_buffer), std::end(dma_transfer_buffer), pixel_pattern);

    TFT_PIN_RS_HIGH();
    TFT_PIN_CS_LOW();
    tft_driver_delay();

    while (pixels > 0) {
        uint16_t chunk_pixels = (pixels > TFT_DMA_TX_CHUNK_PIXELS) ? TFT_DMA_TX_CHUNK_PIXELS : pixels;
        tft_driver_spi_send_buffer_dma_raw(dma_transfer_buffer, chunk_pixels * 2U);
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
    TFT_PIN_RS_HIGH();
    TFT_PIN_CS_LOW();
    tft_driver_delay();

    uint32_t offset = 0;
    while (offset < pixel_count) {
        if (pixel_count - offset > TFT_DMA_TX_CHUNK_PIXELS) {
            // 32bit transfer
            uint32_t *dma_buffer = reinterpret_cast<uint32_t *>(dma_transfer_buffer);
            const uint32_t *pixel_buffer = reinterpret_cast<const uint32_t *>(pixels + offset);
            for (uint16_t i = 0; i < TFT_DMA_TX_CHUNK_PIXELS / 2; i++) {
                *dma_buffer++ = __REV16(*pixel_buffer++);
            }
            tft_driver_spi_send_buffer_dma_raw(dma_transfer_buffer, TFT_DMA_TX_CHUNK_PIXELS * 2U);
            offset += TFT_DMA_TX_CHUNK_PIXELS;
        }
        else {
            // 16bit transfer
            const uint16_t chunk_pixels = pixel_count - offset;
            const uint16_t *pixel_buffer = pixels + offset;
            uint16_t *dma_buffer = reinterpret_cast<uint16_t *>(dma_transfer_buffer);
            for (uint16_t i = 0; i < chunk_pixels; i++) {
                *dma_buffer++ = __REV16(*pixel_buffer++);
            }
            tft_driver_spi_send_buffer_dma_raw(dma_transfer_buffer, chunk_pixels * 2U);
            offset += chunk_pixels;
        }
    }

    tft_driver_delay();
    TFT_PIN_CS_HIGH();
}

void tft_write_window_pixels(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint16_t *pixels, uint32_t pixel_count)
{
    set_column(x0, x1);
    set_row(y0, y1);
    tft_driver_send_command(ST7789_RAMWR);
    write_pixel_buffer_rgb565(pixels, pixel_count);
}

/**
 * Set column address range
 */
static inline void set_column(uint16_t x0, uint16_t x1)
{
    constexpr uint32_t kRowOfs = ST7789_COL_OFS;
    const uint32_t data = __REV16((static_cast<uint32_t>(x1 + kRowOfs) << 16U) | static_cast<uint32_t>(x0 + kRowOfs));
    tft_driver_send_command(ST7789_CASET);
    tft_driver_send_data(&data, sizeof(data));
}

/**
 * Set row address range
 */
static inline void set_row(uint16_t y0, uint16_t y1)
{
    constexpr uint32_t kRowOfs = ST7789_ROW_OFS;
    const uint32_t data = __REV16((static_cast<uint32_t>(y1 + kRowOfs) << 16U) | static_cast<uint32_t>(y0 + kRowOfs));
    tft_driver_send_command(ST7789_RASET);
    tft_driver_send_data(&data, sizeof(data));
}

/**
 * Initialize ST7789 display
 */
static void ST7789_init(void)
{
    /* Hardware reset */
    TFT_PIN_RST_LOW();
    HAL_Delay(25);
    TFT_PIN_RST_HIGH();
    HAL_Delay(60);

    /* Software reset */
    tft_driver_send_command(ST7789_SWRESET);
    HAL_Delay(75);

    /* Exit sleep mode */
    tft_driver_send_command(ST7789_SLPOUT);
    HAL_Delay(60);

    /* Frame rate control */
    tft_driver_send_command(ST7789_FRMCTR1);
    uint8_t frmctr[] = {0x01, 0x2C, 0x2D};
    tft_driver_send_data(frmctr, 3);
    HAL_Delay(5);

    /* Memory access control */
    tft_driver_send_command(ST7789_MADCTL);
    uint8_t madctl[] = {0x0};
    tft_driver_send_data(madctl, 1);

    /* Color mode: 16-bit */
    tft_driver_send_command(ST7789_COLMOD);
    uint8_t colmod[] = {0x05};
    tft_driver_send_data(colmod, 1);
    HAL_Delay(5);

    /* Power control */
    tft_driver_send_command(ST7789_PWCTR1);
    uint8_t pwctr1[] = {0xA2, 0x02, 0x84};
    tft_driver_send_data(pwctr1, 3);

    tft_driver_send_command(ST7789_PWCTR2);
    uint8_t pwctr2[] = {0xC5};
    tft_driver_send_data(pwctr2, 1);

    tft_driver_send_command(ST7789_PWCTR3);
    uint8_t pwctr3[] = {0x0A, 0x00};
    tft_driver_send_data(pwctr3, 2);

    tft_driver_send_command(ST7789_VMCTR1);
    uint8_t vmctr1[] = {0x0E};
    tft_driver_send_data(vmctr1, 1);
    HAL_Delay(10);

    /* Enable panel inversion (required on this ST7789 module) */
    tft_driver_send_command(ST7789_INVON);
    HAL_Delay(5);

    /* Display on */
    tft_driver_send_command(ST7789_DISPON);
    HAL_Delay(50);
}

/**
 * Clear display
 */
void tft_clear_display(uint16_t color)
{
    set_column(0, LV_HOR_RES_MAX - 1);
    set_row(0, LV_VER_RES_MAX - 1);
    tft_driver_send_command(ST7789_RAMWR);

    write_color_pixels(color, LV_HOR_RES_MAX * LV_VER_RES_MAX);
}

/**
 * Initialize driver
 */
void tft_driver_init(void)
{
    // === Display setup ===
    ST7789_init();
}

#endif // TFT_DRIVER == TFT_DRIVER_ST7789
