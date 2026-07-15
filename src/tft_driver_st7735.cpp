/**
  Author: sascha_lammers@gmx.de

  Baremetal SPI driver for ST7735 - DMA TX based
*/

#include <Arduino.h>
#include <stm32f1xx.h>
#include "tft_driver.h"

static uint8_t g_tft_color_chunk[TFT_DMA_TX_CHUNK_PIXELS * 2];
static uint8_t g_tft_pixel_chunk[TFT_DMA_TX_CHUNK_PIXELS * 2];

static void send_command(uint8_t cmd);
static void set_column(uint16_t x0, uint16_t x1);
static void set_row(uint16_t y0, uint16_t y1);

/**
 * Initialize DMA1 Channel 5 for SPI2 TX.
 */
static void spi2_dma_init(void) 
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    TFT_DMA_CH->CCR = 0;
    TFT_DMA_CH->CPAR = (uint32_t)&SPI2->DR;
}

/**
 * Send data buffer via SPI2 TX DMA with CS already asserted.
 */
static void spi_send_buffer_dma_raw(const uint8_t *data, uint16_t len) 
{
    if (!data || len == 0) {
        return;
    }

    TFT_DMA_CH->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5;

    TFT_DMA_CH->CMAR = (uint32_t)data;
    TFT_DMA_CH->CNDTR = len;
    TFT_DMA_CH->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_1;

    SPI2->CR2 |= SPI_CR2_TXDMAEN;
    TFT_DMA_CH->CCR |= DMA_CCR_EN;

    int timeout = 200000;
    while (!(DMA1->ISR & DMA_ISR_TCIF5) && timeout--) {
        delay(0);
    }

    TFT_DMA_CH->CCR &= ~DMA_CCR_EN;
    SPI2->CR2 &= ~SPI_CR2_TXDMAEN;
    DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5;

    timeout = 200000;
    while (((SPI2->SR & SPI_SR_TXE) == 0U) && timeout--) {
        delay(0);
    }

    timeout = 200000;
    while ((SPI2->SR & SPI_SR_BSY) && timeout--) {
        delay(0);
    }

    while (SPI2->SR & SPI_SR_RXNE) {
        (void)SPI2->DR;
    }
    (void)SPI2->SR;
}

/**
 * Initialize SPI2 for display communication
 */
static void spi2_init(void) 
{
    /* Enable clocks */
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN;

    /* Configure PB13 (CLK) and PB15 (MOSI) as alternate function push-pull */
    GPIOB->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13 | GPIO_CRH_MODE15 | GPIO_CRH_CNF15);
    GPIOB->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1)  /* PB13: 50MHz */
               | (GPIO_CRH_CNF13_1)                        /* PB13: Alt func push-pull */
               | (GPIO_CRH_MODE15_0 | GPIO_CRH_MODE15_1)  /* PB15: 50MHz */
               | (GPIO_CRH_CNF15_1);                       /* PB15: Alt func push-pull */

    /* Disable SPI first */
    SPI2->CR1 = 0;

    /* Configure SPI2 - BR=0 gives ~18MHz on APB1=36MHz */
    SPI2->CR1 = (0 << 3)          /* Baud rate: BR=0 (36MHz/2 = 18MHz) */
              | SPI_CR1_MSTR      /* Master mode */
              | SPI_CR1_SSM       /* Software slave management */
              | SPI_CR1_SSI       /* Internal slave select */
              | SPI_CR1_CPOL      /* Clock polarity = 1 */
              | SPI_CR1_CPHA;     /* Clock phase = 1 */

    SPI2->CR2 = 0;  /* DMA disabled by default, enabled per transfer */

    /* Enable SPI2 */
    SPI2->CR1 |= SPI_CR1_SPE;

    spi2_dma_init();

}

/**
 * Send single SPI byte (DMA)
 */
static void spi_send_byte(uint8_t byte) 
{
    /* Pull CS low before transfer */
    TFT_PIN_CS_LOW();
    delayMicroseconds(1);

    spi_send_buffer_dma_raw(&byte, 1);

    /* Pull CS high after transfer */
    delayMicroseconds(1);
    TFT_PIN_CS_HIGH();
}

/**
 * Send data buffer via SPI (DMA)
 */
static void spi_send_buffer(const uint8_t *data, uint16_t len) 
{
    TFT_PIN_CS_LOW();
    delayMicroseconds(1);
    spi_send_buffer_dma_raw(data, len);
    delayMicroseconds(1);
    TFT_PIN_CS_HIGH();
}

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
    delayMicroseconds(1);

    while (pixels > 0) {
        uint16_t chunk_pixels = (pixels > TFT_DMA_TX_CHUNK_PIXELS) ? TFT_DMA_TX_CHUNK_PIXELS : (uint16_t)pixels;
        spi_send_buffer_dma_raw(g_tft_color_chunk, (uint16_t)(chunk_pixels * 2U));
        pixels -= chunk_pixels;
    }

    delayMicroseconds(1);
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
    delayMicroseconds(1);

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

        spi_send_buffer_dma_raw(g_tft_pixel_chunk, (uint16_t)(chunk_pixels * 2U));
        offset += chunk_pixels;
    }

    delayMicroseconds(1);
    TFT_PIN_CS_HIGH();
}

void tft_write_window_pixels(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint16_t *pixels, uint32_t pixel_count) 
{
    set_column(x0, x1);
    set_row(y0, y1);
    send_command(ST7735_RAMWR);
    write_pixel_buffer_rgb565(pixels, pixel_count);
}

/**
 * Send SPI command byte (with RS=0)
 */
static void send_command(uint8_t cmd) 
{
    TFT_PIN_RS_LOW();
    delayMicroseconds(5);
    spi_send_byte(cmd);
    delayMicroseconds(5);
    TFT_PIN_RS_HIGH();
}

/**
 * Send SPI data bytes (with RS=1)
 */
static void send_data(const uint8_t *data, uint16_t len) 
{
    TFT_PIN_RS_HIGH();
    spi_send_buffer(data, len);
}

/**
 * Set column address range
 */
static void set_column(uint16_t x0, uint16_t x1) 
{
    uint8_t data[4] = {(uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF), 
                       (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF)};
    send_command(ST7735_CASET);
    send_data(data, 4);
}

/**
 * Set row address range
 */
static void set_row(uint16_t y0, uint16_t y1) 
{
    uint8_t data[4] = {(uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF), 
                       (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF)};
    send_command(ST7735_RASET);
    send_data(data, 4);
}

/**
 * Initialize ST7735 display
 */
static void st7735_init(void) 
{
    /* Hardware reset */
    TFT_PIN_RST_LOW();
    delay(50);
    TFT_PIN_RST_HIGH();
    delay(120);

    /* Software reset */
    send_command(ST7735_SWRESET);
    delay(150);

    /* Exit sleep mode */
    send_command(ST7735_SLPOUT);
    delay(120);

    /* Frame rate control */
    send_command(ST7735_FRMCTR1);
    uint8_t frmctr[] = {0x01, 0x2C, 0x2D};
    send_data(frmctr, 3);
    delay(10);

    /* Memory access control */
    send_command(ST7735_MADCTL);
    uint8_t madctl[] = {0xA0};
    send_data(madctl, 1);

    /* Color mode: 16-bit */
    send_command(ST7735_COLMOD);
    uint8_t colmod[] = {0x05};
    send_data(colmod, 1);
    delay(10);

    /* Power control */
    send_command(ST7735_PWCTR1);
    uint8_t pwctr1[] = {0xA2, 0x02, 0x84};
    send_data(pwctr1, 3);

    send_command(ST7735_PWCTR2);
    uint8_t pwctr2[] = {0xC5};
    send_data(pwctr2, 1);

    send_command(ST7735_PWCTR3);
    uint8_t pwctr3[] = {0x0A, 0x00};
    send_data(pwctr3, 2);

    send_command(ST7735_VMCTR1);
    uint8_t vmctr1[] = {0x0E};
    send_data(vmctr1, 1);
    delay(10);

    /* Gamma */
    send_command(ST7735_GMCTRP1);
    uint8_t gp[] = {0x10, 0x0E, 0x02, 0x03, 0x0E, 0x07, 0x02, 0x07, 0x0A, 0x12, 0x27, 0x37, 0x00, 0x0D, 0x0E, 0x10};
    send_data(gp, 16);

    send_command(ST7735_GMCTRN1);
    uint8_t gn[] = {0x10, 0x0E, 0x03, 0x03, 0x0F, 0x06, 0x02, 0x08, 0x0A, 0x13, 0x26, 0x36, 0x00, 0x0D, 0x0E, 0x10};
    send_data(gn, 16);
    delay(10);

    /* Exit invert */
    send_command(ST7735_INVOFF);
    delay(10);

    /* Display on */
    send_command(ST7735_DISPON);
    delay(100);

}

/**
 * Clear display
 */
void tft_clear_display(void) 
{
    set_column(0, TFT_DIM_WIDTH - 1);
    set_row(0, TFT_DIM_HEIGHT - 1);
    send_command(ST7735_RAMWR);

    write_color_pixels(0x0000, (uint32_t)TFT_DIM_WIDTH * TFT_DIM_HEIGHT);
}

/**
 * Initialize driver
 */
void tft_driver_init(void) 
{
    // === init GPIO pins ===
    tft_driver_gpio_init();

    TFT_PIN_CS_HIGH();
    TFT_PIN_RS_HIGH();

    // === SPI setup ===
    spi2_init();

    // === Display setup ===
    st7735_init();
}
