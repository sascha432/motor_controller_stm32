/**
  Author: sascha_lammers@gmx.de
*/

#include "helpers.h"
#include "debug.h"

//=== float to string conversion ===

void float_to_string_convert(char *buffer, size_t size, float value, uint8_t precision, bool trimTrailingZeros)
{
    if (!buffer || size == 0) {
        return;
    }

    char *out = buffer;
    size_t remaining = size;

    auto append = [&](char c) {
        if (remaining > 1) {
            *out++ = c;
            --remaining;
        }
    };

    if (precision > 6) {
        precision = 6;
    }

    if (value < 0.0f) {
        append('-');
        value = -value;
    }

    uint32_t scale = 1;
    for (uint8_t i = 0; i < precision; ++i) {
        scale *= 10U;
    }

    // Convert to fixed-point and round to the requested precision.
    const float scaledFloat = value * static_cast<float>(scale) + 0.5f;
    const uint32_t scaled = static_cast<uint32_t>(scaledFloat);
    uint32_t intPart = scaled / scale;
    uint32_t fracPart = scaled % scale;

    char intBuf[11];
    uint8_t intLen = 0;
    do {
        intBuf[intLen++] = static_cast<char>('0' + (intPart % 10U));
        intPart /= 10U;
    } while (intPart && intLen < sizeof_array(intBuf));

    while (intLen > 0) {
        append(intBuf[--intLen]);
    }

    if (precision > 0) {
        uint8_t fracDigits = precision;
        if (trimTrailingZeros) {
            while (fracDigits > 1 && (fracPart % 10U) == 0U) {
                fracPart /= 10U;
                --fracDigits;
            }
        }

        append('.');

        uint32_t div = 1U;
        for (uint8_t i = 1; i < fracDigits; ++i) {
            div *= 10U;
        }
        while (div > 0U) {
            append(static_cast<char>('0' + (fracPart / div)));
            fracPart %= div;
            div /= 10U;
        }
    }

    *out = '\0';
}

// === WatchDog implementation ===

WWDG_HandleTypeDef WatchDog::watchdog;
volatile uint32_t WatchDog::ticks;

extern "C" void HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg)
{
    (void)hwwdg;
    __HAL_RCC_WWDG_CLK_ENABLE();
    HAL_NVIC_SetPriority(WWDG_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(WWDG_IRQn);
}

void WatchDog::init()
{
    feed();
    watchdog.Instance = WWDG;
    watchdog.Init.Prescaler = WWDG_PRESCALER_8;
    watchdog.Init.Window = 0x7F;
    watchdog.Init.Counter = 0x7F;
    watchdog.Init.EWIMode = WWDG_EWI_ENABLE;
    if (HAL_WWDG_Init(&watchdog) != HAL_OK) {
        Error_Handler();
    }
}

void WatchDog::feed()
{
    ticks = 0;
}
