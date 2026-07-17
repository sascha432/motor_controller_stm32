/**
  Author: sascha_lammers@gmx.de
*/

#ifndef TFT_DRIVER_H
#define TFT_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// TFT pins
#define TFT_PIN_CLK PB13
#define TFT_PIN_MOSI PB15
#define TFT_PIN_RS PC7
#define TFT_PIN_RST PC6
#define TFT_PIN_CS PD15
#define TFT_PIN_LED PB11

// PIN macros
#define TFT_PIN_RS_HIGH() GPIOC->BSRR = (1 << 7)
#define TFT_PIN_RS_LOW() GPIOC->BSRR = (1 << (7 + 16))
#define TFT_PIN_CS_HIGH() GPIOD->BSRR = (1 << 15)
#define TFT_PIN_CS_LOW() GPIOD->BSRR = (1 << (15 + 16))
#define TFT_PIN_RST_HIGH() GPIOC->BSRR = (1 << 6)
#define TFT_PIN_RST_LOW() GPIOC->BSRR = (1 << (6 +  16))

#define TFT_DIM_WIDTH 160
#define TFT_DIM_HEIGHT 128
#define TFT_BUFFER_SIZE (TFT_DIM_WIDTH * 100) // at least 10, increase if RAM is available

#define TFT_DMA_CH DMA1_Channel5
#define TFT_DMA_TX_CHUNK_PIXELS 64

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

/* Function prototypes */
void tft_backlight_pwm_set_raw(uint16_t value);
void tft_backlight_pwm_set(uint8_t value);
void tft_driver_gpio_init(void);

void tft_driver_init(void);
void tft_display_on(void);
void tft_display_off(void);
void tft_clear_display(void);
void tft_test_pattern(void);
void tft_update_ui(uint16_t slider, bool pressed);
void tft_write_window_pixels(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint16_t *pixels, uint32_t pixel_count);

#ifdef __cplusplus
}
#endif

#endif /* TFT_DRIVER_H */
