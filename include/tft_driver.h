/**
  Author: sascha_lammers@gmx.de

  Baremetal SPI DMA driver
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "lvgl.h"

// TFT driver selection
#define TFT_DRIVER_ST7735           1
#define TFT_DRIVER_ST7789           2
#ifndef TFT_DRIVER
#define TFT_DRIVER                  TFT_DRIVER_ST7789
#endif

// get dimensions from lvgl conf
static constexpr uint32_t kLvTotalBufferSize = (LV_HOR_RES_MAX * LV_BUFFER_LINES);

// we use double buffering
static constexpr uint32_t kLvDisplayBufferSize = kLvTotalBufferSize / 2;
static_assert(kLvDisplayBufferSize <= UINT16_MAX, "DMA transfers are limited to 64kb");

#define TFT_DMA_CH                  DMA1_Channel5
#define TFT_DMA                     DMA1

// in draw_buf_flush() lvgl is blocking if s_lvgl_disp_drv.draw_buf->flushing is set
// as long as only lvgl is drawing to the display in the main loop, this does not require a separate locking mechanism
#define HAVE_LVGL_BUFFER_LOCK       0

#if HAVE_LVGL_BUFFER_LOCK
extern volatile bool s_lvgl_buf_busy;
#endif
extern lv_disp_drv_t s_lvgl_disp_drv;

static inline void tft_driver_global_unlock()
{
    #if HAVE_LVGL_BUFFER_LOCK
    // DMA done, remove busy flag
    s_lvgl_buf_busy = false;
    #endif
}

/**
 * @brief Unlock DMA transfer
 *
 */
static inline void tft_driver_dma_transfer_finished_isr()
{
    #if LVGL_VERSION_MAJOR == 8
    s_lvgl_disp_drv.draw_buf->flushing = 0;
    s_lvgl_disp_drv.draw_buf->flushing_last = 0;
    #else
    lv_disp_flush_ready(&s_lvgl_disp_drv);
    #endif

    // Post-Transfer Cleanup is done before the next DMA request indicated by CS pulled low
    tft_driver_global_unlock();
}

/**
 * @brief Internal delay function
 *
 */
static inline void tft_driver_delay()
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
void tft_driver_gpio_init(void);
void tft_driver_spi_init(void);
void tft_driver_spi_send_buffer_dma_raw(const void *data, uint16_t len);
void tft_driver_spi_send_buffer_dma_interrupt(const void *data, uint16_t len);
void tft_driver_spi_send_byte(uint8_t byte);
void tft_driver_spi_send_buffer(const void *data, uint16_t len) ;
void tft_driver_send_command(uint8_t cmd);
void tft_driver_send_data(const void *data, uint16_t len);
void tft_driver_dma_transfer_finished_isr();
void tft_driver_prepare_dma();

// Custom functions for each TFT driver
void tft_driver_init(void);
void tft_driver_lvgl_init(void);
void tft_clear_display(uint16_t color = 0x0000);
void tft_write_window_pixels(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint16_t *pixels, uint32_t pixel_count);

// EOF
