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
#define LV_BUFFER_SIZE              (LV_HOR_RES_MAX * LV_BUFFER_LINES)

#define TFT_DMA_CH                  DMA1_Channel5
#define TFT_DMA_TX_CHUNK_PIXELS     256

// Function prototypes
extern lv_disp_draw_buf_t s_lvgl_draw_buf;
extern lv_color_t s_lvgl_buf_1[LV_BUFFER_SIZE];
extern lv_disp_drv_t s_lvgl_disp_drv;

inline void tft_driver_delay()
{
    __NOP();
}

/**
 * @brief Set the raw PWM value for the backlight
 *
 * @param value PWM value (0-999)
 */
inline void tft_backlight_pwm_set_raw(uint16_t value)
{
    if (value > 999) {
        value = 999;
    }
    UI_TFT_BACKLIGHT_SET_PWM(value);
}

/**
 * @brief Set the PWM value for the backlight
 *
 * @param value PWM value (0-100)
 */
inline void tft_backlight_pwm_set(uint8_t value)
{
    tft_backlight_pwm_set_raw(value * 10U);
}

// Common functions for all TFT drivers
void tft_driver_gpio_tim_init(void);
void tft_driver_spi_init(void);
void tft_driver_spi_send_buffer_dma_raw(const void *data, uint16_t len);
void tft_driver_spi_send_byte(uint8_t byte);
void tft_driver_spi_send_buffer(const void *data, uint16_t len) ;
void tft_driver_send_command(uint8_t cmd);
void tft_driver_send_data(const void *data, uint16_t len);

// Custom functions for each TFT driver
void tft_driver_init(void);
void tft_driver_lvgl_init(void);
void tft_clear_display(uint16_t color = 0x0000);
void tft_write_window_pixels(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint16_t *pixels, uint32_t pixel_count);

#if HAVE_SCREENSHOTS
bool tft_driver_screenshot_begin(void);
void tft_driver_screenshot_end(void);
#endif

// EOF
