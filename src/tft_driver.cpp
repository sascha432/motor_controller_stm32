/**
  Author: sascha_lammers@gmx.de
*/

#include <stm32f1xx.h>
#include "tft_driver.h"

/**
 * @brief init GPIO pins for the SPI TFT display and backlight PWM
 * 
 * This code does not use the PIN macros and needs to modified manually if the pinout is changed
 * 
 */
void tft_driver_gpio_init(void)
{
    // Enable GPIOC and GPIOD clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN;

    // TFT_PIN_RST/PC6, TFT_PIN_RS/PC7 (CRL)
    GPIOC->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOC->CRL |=  ((0x2 << (6 * 4)) |   // PC6 output PP 2MHz
                    (0x2 << (7 * 4)));   // PC7 output PP 2MHz


    // TFT_PIN_CS/PD15 (CRH)
    GPIOD->CRH &= ~(0xF << ((15 - 8) * 4));
    GPIOD->CRH |=  (0x3 << ((15 - 8) * 4)); // PD15 output PP 50MHz

    // backlight PWM on TFT_PIN_LED/PB11
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    AFIO->MAPR &= ~AFIO_MAPR_TIM2_REMAP;
    AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_0 | AFIO_MAPR_TIM2_REMAP_1;

    // TFT_PIN_LED/PB11 = Alternate Function Open Drain, 50 MHz (CRH)
    GPIOB->CRH &= ~(0xF << ((11 - 8) * 4));
    GPIOB->CRH |=  (0xD << ((11 - 8) * 4));

    TIM2->CR1 = 0;
    TIM2->PSC = 71;      /* 72MHz / (71+1) = 1MHz timer clock */
    TIM2->ARR = 999;     /* 1MHz / (999+1) = 1kHz PWM */
    TIM2->CCR4 = 0;      /* 0% duty at startup */

    TIM2->CCMR2 &= ~(TIM_CCMR2_CC4S | TIM_CCMR2_OC4M | TIM_CCMR2_OC4PE);
    TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

    // Enable TIM2 Channel 4 output on the pin
    TIM2->CCER |= TIM_CCER_CC4E;

    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

/**
 * @brief Set the raw PWM value for the backlight
 * 
 * @param value PWM value (0-999)
 */
void tft_backlight_pwm_set_raw(uint16_t value) 
{
    if (value > 999U) {
        value = 999U;
    }
    TIM2->CCR4 = value;
}

/**
 * @brief Set the PWM value for the backlight
 * 
 * @param value PWM value (0-1000)
 */
void tft_backlight_pwm_set(uint32_t value) 
{
    tft_backlight_pwm_set_raw(100 + (uint16_t)((value * 900) / 1000));
}

void tft_update_ui(uint16_t slider, bool pressed) 
{
    (void)slider;
    (void)pressed;
}
