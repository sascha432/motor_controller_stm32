/**
  Author: sascha_lammers@gmx.de

  Baremetal SPI DMA driver
*/

#include <stm32f1xx.h>
#include "tft_driver.h"

lv_disp_draw_buf_t s_lvgl_draw_buf;
lv_color_t s_lvgl_buf_1[TFT_BUFFER_SIZE];
lv_disp_drv_t s_lvgl_disp_drv;

/**
 * @brief init GPIO pins for the SPI TFT display and backlight PWM
 * 
 * This code does not use the PIN macros and needs to modified manually if the pinout is changed
 * 
 */
void tft_driver_gpio_init(void)
{
    // Enable GPIOC and GPIOD clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPBEN;

    // TFT_PIN_RST/PC6, TFT_PIN_RS/PC7 (CRL)
    GPIOC->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOC->CRL |=  ((0x2 << (6 * 4)) |   // PC6 output PP 2MHz
                    (0x2 << (7 * 4)));   // PC7 output PP 2MHz


    // TFT_PIN_CS/PD15 (CRH)
    GPIOD->CRH &= ~(0xF << ((15 - 8) * 4));
    GPIOD->CRH |=  (0x3 << ((15 - 8) * 4)); // PD15 output PP 50MHz

    // set pins to default state
    TFT_PIN_CS_HIGH();
    TFT_PIN_RS_HIGH();

    // backlight PWM on TFT_PIN_LED/PB11
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    AFIO->MAPR &= ~AFIO_MAPR_TIM2_REMAP;
    // TIM2 partial remap 2 routes CH4 to PB11.
    AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_1;

    // TFT_PIN_LED/PB11 = Alternate Function Push-Pull, 50 MHz (CRH)
    GPIOB->CRH &= ~(0xF << ((11 - 8) * 4));
    GPIOB->CRH |=  (0xB << ((11 - 8) * 4));

    // LED_PWM/PB10 = Alternate Function Push-Pull, 50 MHz (CRH)
    GPIOB->CRH &= ~(0xF << ((10 - 8) * 4));
    GPIOB->CRH |=  (0xB << ((10 - 8) * 4));

    TIM2->CR1 = 0;
    TIM2->PSC = 71;      /* 72MHz / (71+1) = 1MHz timer clock */
    TIM2->ARR = 999;     /* 1MHz / (999+1) = 1kHz PWM */

    // Duty cycle 0
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;

    TIM2->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3M | TIM_CCMR2_OC3PE | TIM_CCMR2_CC4S | TIM_CCMR2_OC4M | TIM_CCMR2_OC4PE);
    TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

    // Enable TIM2 Channel 2 & 4 output on the pin
    TIM2->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

/**
 * Initialize SPI2 for display communication
 */
void tft_driver_spi_init(void)
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

    #if defined(STM32F107xC)

        /* Configure SPI2 - BR=0 gives ~18MHz on APB1=36MHz */
        SPI2->CR1 = (0 << 3)          /* Baud rate: BR=0 (36MHz/2 = 18MHz) */
                | SPI_CR1_MSTR      /* Master mode */
                | SPI_CR1_SSM       /* Software slave management */
                | SPI_CR1_SSI       /* Internal slave select */
                | SPI_CR1_CPOL      /* Clock polarity = 1 */
                | SPI_CR1_CPHA;     /* Clock phase = 1 */

    #elif defined(STM32F103xC)

        /* Configure SPI2 - BR=0 gives ~18MHz on APB1=36MHz */
        SPI2->CR1 = (0 << 3)          /* Baud rate: BR=0 (36MHz/2 = 18MHz) */
                | SPI_CR1_MSTR      /* Master mode */
                | SPI_CR1_SSM       /* Software slave management */
                | SPI_CR1_SSI       /* Internal slave select */
                | SPI_CR1_CPOL      /* Clock polarity = 1 */
                | SPI_CR1_CPHA;     /* Clock phase = 1 */

    #else

        #error "Unsupported STM32F1 series"

    #endif

    SPI2->CR2 = 0;  /* DMA disabled by default, enabled per transfer */

    /* Enable SPI2 */
    SPI2->CR1 |= SPI_CR1_SPE;

    // initialize DMA1 Channel 5 for SPI2 TX
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    TFT_DMA_CH->CCR = 0;
    TFT_DMA_CH->CPAR = (uint32_t)&SPI2->DR;
}

/**
 * @brief Set the raw PWM value for the backlight
 * 
 * @param value PWM value (0-999)
 */
void tft_backlight_pwm_set_raw(uint16_t value) 
{
    if (value > 999) {
        value = 999;
    }
    TIM2->CCR4 = value;
}

/**
 * @brief Set the PWM value for the backlight
 * 
 * @param value PWM value (0-100)
 */
void tft_backlight_pwm_set(uint8_t value) 
{
    tft_backlight_pwm_set_raw(value * 10U);
}

/**
 * Send data buffer via SPI2 TX DMA with CS already asserted.
 */
void tft_driver_spi_send_buffer_dma_raw(const uint8_t *data, uint16_t len) 
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
        __NOP();
    }

    TFT_DMA_CH->CCR &= ~DMA_CCR_EN;
    SPI2->CR2 &= ~SPI_CR2_TXDMAEN;
    DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5;

    timeout = 200000;
    while (((SPI2->SR & SPI_SR_TXE) == 0U) && timeout--) {
        __NOP();
    }

    timeout = 200000;
    while ((SPI2->SR & SPI_SR_BSY) && timeout--) {
        __NOP();
    }

    while (SPI2->SR & SPI_SR_RXNE) {
        (void)SPI2->DR;
    }
    (void)SPI2->SR;
}

/**
 * Send single SPI byte (DMA)
 */
void tft_driver_spi_send_byte(uint8_t byte) 
{
    /* Pull CS low before transfer */
    TFT_PIN_CS_LOW();
    tft_driver_delay();
    

    tft_driver_spi_send_buffer_dma_raw(&byte, 1);

    /* Pull CS high after transfer */
    tft_driver_delay();
    TFT_PIN_CS_HIGH();
}

/**
 * Send data buffer via SPI (DMA)
 */
void tft_driver_spi_send_buffer(const uint8_t *data, uint16_t len) 
{
    TFT_PIN_CS_LOW();
    tft_driver_delay();
    tft_driver_spi_send_buffer_dma_raw(data, len);
    tft_driver_delay();
    TFT_PIN_CS_HIGH();
}

/**
 * Send SPI command byte (with RS=0)
 */
void tft_driver_send_command(uint8_t cmd) 
{
    TFT_PIN_RS_LOW();
    tft_driver_delay();
    tft_driver_spi_send_byte(cmd);
    tft_driver_delay();
    TFT_PIN_RS_HIGH();
}

/**
 * Send SPI data bytes (with RS=1)
 */
void tft_driver_send_data(const uint8_t *data, uint16_t len) 
{
    TFT_PIN_RS_HIGH();
    tft_driver_spi_send_buffer(data, len);
}

/**
 * Flush callback for LVGL
 */
void tft_driver_lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) 
{
    uint16_t x0 = (uint16_t)area->x1;
    uint16_t y0 = (uint16_t)area->y1;
    uint16_t x1 = (uint16_t)area->x2;
    uint16_t y1 = (uint16_t)area->y2;
    uint32_t px = (uint32_t)(x1 - x0 + 1U) * (uint32_t)(y1 - y0 + 1U);

    tft_write_window_pixels(x0, y0, x1, y1, (const uint16_t *)color_p, px);
    lv_disp_flush_ready(disp_drv);
}

/**
 * Initialize LVGL display driver and register callback
 */
void tft_driver_lvgl_init(void)
{
    lv_disp_draw_buf_init(&s_lvgl_draw_buf, s_lvgl_buf_1, nullptr, TFT_BUFFER_SIZE);
    lv_disp_drv_init(&s_lvgl_disp_drv);
    s_lvgl_disp_drv.hor_res = TFT_DIM_WIDTH;
    s_lvgl_disp_drv.ver_res = TFT_DIM_HEIGHT;
    s_lvgl_disp_drv.flush_cb = tft_driver_lvgl_flush_cb;
    s_lvgl_disp_drv.draw_buf = &s_lvgl_draw_buf;
    lv_disp_drv_register(&s_lvgl_disp_drv);
}

