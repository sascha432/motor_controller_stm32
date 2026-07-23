/**
  Author: sascha_lammers@gmx.de

  Baremetal SPI DMA driver
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "lvgl.h"
#include "pins.h"

// TFT driver selection
#define TFT_DRIVER_ST7735           1
#define TFT_DRIVER_ST7789           2
#ifndef TFT_DRIVER
#define TFT_DRIVER                  TFT_DRIVER_ST7789
#endif

// get dimensions from lvgl conf
#define TFT_DIM_WIDTH               LV_HOR_RES_MAX
#define TFT_DIM_HEIGHT              LV_VER_RES_MAX
#define TFT_BUFFER_SIZE             (TFT_DIM_WIDTH * 64) // at least 10, increase if RAM is available

#define TFT_DMA_CH                  DMA1_Channel5
#define TFT_DMA_TX_CHUNK_PIXELS     64

// Function prototypes
extern lv_disp_draw_buf_t s_lvgl_draw_buf;
extern lv_color_t s_lvgl_buf_1[TFT_BUFFER_SIZE];
extern lv_disp_drv_t s_lvgl_disp_drv;

inline void tft_driver_delay() 
{
    __NOP();
}

void tft_backlight_pwm_set_raw(uint16_t value);
void tft_backlight_pwm_set(uint8_t value);

// Common functions for all TFT drivers
void tft_driver_gpio_init(void);
void tft_driver_spi_init(void);
void tft_driver_spi_send_buffer_dma_raw(const uint8_t *data, uint16_t len);
void tft_driver_spi_send_byte(uint8_t byte);
void tft_driver_spi_send_buffer(const uint8_t *data, uint16_t len) ;
void tft_driver_send_command(uint8_t cmd);
void tft_driver_send_data(const uint8_t *data, uint16_t len);

// Custom functions for each TFT driver
void tft_driver_init(void);
void tft_driver_lvgl_init(void);
void tft_clear_display(uint16_t color = 0x0000);
void tft_write_window_pixels(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint16_t *pixels, uint32_t pixel_count);

// EOF
