/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <algorithm>
#include "helpers.h"
#include "pins.h"

/**
 * @brief Charlieplexed LEDs
 *
 */
template <uint32_t GPIO_LEDS_PIN, uint32_t GPIO_ILLUMINATION_LED_PIN>
struct LEDs_T {

    static constexpr uint32_t kIlluminationResolution = 1024;

    static void init()
    {
        // PWM timer setup handled in tft_driver_gpio_tim_init()
        static_assert(GPIO_ILLUMINATION_LED_PIN == PB10, "Illumination LED pin must be PB10");
        static_assert(GPIO_LEDS_PIN == PD12, "LEDs pin must be PD12");

        // Enable GPIO port clock
        __HAL_RCC_GPIOx_CLK_ENABLE<GPIO_LEDS_PIN>();
        __HAL_RCC_GPIOx_CLK_ENABLE<GPIO_ILLUMINATION_LED_PIN>();

        // LED 1 & 2
        off();
    }

    static void off()
    {
        // MODE=00, CNF=01 (floating input) to turn both LEDs off
        GPIO_CRx_REG<GPIO_LEDS_PIN>() &= ~(0xF << digitalPinShift<GPIO_LEDS_PIN>());
        GPIO_CRx_REG<GPIO_LEDS_PIN>() |= (0x4 << digitalPinShift<GPIO_LEDS_PIN>());
    }

    static bool isErrorLEDOn()
    {
        return digitalPinToGPIO<GPIO_LEDS_PIN>()->ODR & (1U << digitalPinToBit<GPIO_LEDS_PIN>());
    }

    static bool isWarningLEDOn()
    {
        return !(digitalPinToGPIO<GPIO_LEDS_PIN>()->ODR & (1U << digitalPinToBit<GPIO_LEDS_PIN>()));
    }

    static void onLEDError()
    {
        digitalWriteHigh<GPIO_LEDS_PIN>(); // set pin high to turn on LED1
        // MODE=10 (2MHz), CNF=00 (push-pull)
        GPIO_CRx_REG<GPIO_LEDS_PIN>() &= ~(0xF << digitalPinShift<GPIO_LEDS_PIN>());
        GPIO_CRx_REG<GPIO_LEDS_PIN>() |= (0x2 << digitalPinShift<GPIO_LEDS_PIN>());
    }

    static void onLEDWarning()
    {
        digitalWriteLow<GPIO_LEDS_PIN>(); // set pin low to turn on LED2
        // MODE=10 (2MHz), CNF=00 (push-pull)
        GPIO_CRx_REG<GPIO_LEDS_PIN>() &= ~(0xF << digitalPinShift<GPIO_LEDS_PIN>());
        GPIO_CRx_REG<GPIO_LEDS_PIN>() |= (0x2 << digitalPinShift<GPIO_LEDS_PIN>());
    }

    static void illuminationLedSetPWM(uint32_t value)
    {
        if (value == 0) {
            UI_ILLUMINATION_LED_SET_PWM(0);
            return;
        }
        // get more linear brightness by using a gamma curve and zero offset
        constexpr uint32_t kOffset = 15;
        const uint32_t clampedBrightness = std::clamp<uint32_t>(value + (kOffset * kIlluminationResolution), (kOffset * kIlluminationResolution), ((100 + kOffset) * kIlluminationResolution));
        const float brightness = clampedBrightness * (1.0f / ((kOffset + 100.0f) * kIlluminationResolution));
        UI_ILLUMINATION_LED_SET_PWM(powf(brightness, 2.2f) * 1000.0f);
    }
};

using LEDs = LEDs_T<MOTOR_LEDS_PIN, ILLUMINATION_LED_PIN>;
