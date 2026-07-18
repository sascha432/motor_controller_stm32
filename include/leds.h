/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "helpers.h"

/**
 * @brief Charlieplexed LEDs
 * 
 */
template <uint32_t GPIO_LEDS_PIN, uint32_t GPIO_ILLUMINATION_LED_PIN, uint32_t GPIO_LEDS_PORT_ADDRESS = (digitalPinToGPIOBase<GPIO_LEDS_PIN>()), uint32_t GPIO_ILLUMINATION_PORT_ADDRESS = (digitalPinToGPIOBase<GPIO_ILLUMINATION_LED_PIN>())>
struct LEDs {

    void init() 
    {
        // Enable GPIO port clock
        RCC->APB2ENR |= RCC_APB2ENR_IOPxEN(GPIO_LEDS_PORT_ADDRESS) | RCC_APB2ENR_IOPxEN(GPIO_ILLUMINATION_PORT_ADDRESS);

        // LED 1 & 2
        offLED1and2();

        // Illumination LED
        // tft_driver_gpio_init() is handling the PWM configuration 
        static_assert(GPIO_ILLUMINATION_LED_PIN == PB10, "Illumination LED pin must be PB10");
    }

    void offLED1and2() 
    {
        // MODE=00, CNF=01 (floating input) to turn both LEDs off
        GPIO_CRx_REG(GPIO_LEDS_PORT_ADDRESS, GPIO_LEDS_PIN) &= ~(0xF << digitalPinShift(GPIO_LEDS_PIN));
        GPIO_CRx_REG(GPIO_LEDS_PORT_ADDRESS, GPIO_LEDS_PIN) |= (0x4 << digitalPinShift(GPIO_LEDS_PIN));
    }

    void onLED1() 
    {
        ((GPIO_TypeDef *)GPIO_LEDS_PORT_ADDRESS)->BSRR = (1U << digitalPinToBit(GPIO_LEDS_PIN)); // set pin high to turn on LED1
        // MODE=10 (2MHz), CNF=00 (push-pull)
        GPIO_CRx_REG(GPIO_LEDS_PORT_ADDRESS, GPIO_LEDS_PIN) &= ~(0xF << digitalPinShift(GPIO_LEDS_PIN));
        GPIO_CRx_REG(GPIO_LEDS_PORT_ADDRESS, GPIO_LEDS_PIN) |= (0x2 << digitalPinShift(GPIO_LEDS_PIN));
    }

    void onLED2() 
    {
        ((GPIO_TypeDef *)GPIO_LEDS_PORT_ADDRESS)->BSRR = (1U << (digitalPinToBit(GPIO_LEDS_PIN) + 16)); // set pin low to turn on LED2
        // MODE=10 (2MHz), CNF=00 (push-pull)
        GPIO_CRx_REG(GPIO_LEDS_PORT_ADDRESS, GPIO_LEDS_PIN) &= ~(0xF << digitalPinShift(GPIO_LEDS_PIN));
        GPIO_CRx_REG(GPIO_LEDS_PORT_ADDRESS, GPIO_LEDS_PIN) |= (0x2 << digitalPinShift(GPIO_LEDS_PIN));
    }

    static void illuminationLedSetPWM(float value) 
    {
        if (value == 0) {
            TIM2->CCR3 = 0;
            return;
        }
        // get more linear brightness by using a gamma curve and offset to avoid flickering at low brightness
        float brightness = std::clamp<float>(value + 15, 15, 115) / 115.0f;

        // old int version, not enough steps for fading brightness in and out
        // float brightness = std::clamp<int32_t>(value + 15, 15, 115) / 115.0f;
        brightness = powf(brightness, 2.2f);
        TIM2->CCR3 = (uint16_t)(brightness * 1000);
    }
};
