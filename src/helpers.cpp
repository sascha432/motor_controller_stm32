/**
  Author: sascha_lammers@gmx.de
*/

#include "pins.h"
#include "debug.h"

// == delay function ===

void delay_us(uint32_t us)
{
    if (us > 1000) {
        HAL_Delay(us / 1000);
        us %= 1000;
    }
    const uint16_t start = TIM7->CNT;
    while ((uint16_t)(TIM7->CNT - start) < us) {
    }
}

// === float to string conversion ===

#if DEBUG

static char floatBuffer[128];
size_t floatBufferPos = 0;

const char *debugFloatToString(float value, uint8_t precision, bool trimTrailingZeros)
{
    if (floatBufferPos > sizeof(floatBuffer) - 24) {
        floatBufferPos = 0;
    }
    float_to_string_convert(floatBuffer + floatBufferPos, sizeof(floatBuffer) - floatBufferPos, value, precision, trimTrailingZeros);
    size_t oldPos = floatBufferPos;
    floatBufferPos += strlen(floatBuffer + floatBufferPos) + 1;
    return floatBuffer + oldPos;
}

#endif

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

void WatchDog::deinit()
{
    __HAL_RCC_WWDG_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(WWDG_IRQn);
}

// === MotorVibes implementation ===

#if HAVE_MOTOR_VIBES

extern TIM_HandleTypeDef tim1;

void MotorVibes::init()
{
    // store values and deinit timer
    PID_MOTOR_PWM_TIMER->CCR1 = 0;
    PID_MOTOR_PWM_TIMER->CCR2 = 0;
    prescaler = tim1.Init.Prescaler;
    period = tim1.Init.Period;
    arr = PID_MOTOR_PWM_TIMER->ARR;
    HAL_TIM_PWM_DeInit(&tim1);

    // store current limits and remove them
    motorCurrentLimit = DAC_GET_MOTOR_CURRENT();
    inputCurrentLimit = DAC_GET_INPUT_CURRENT();
    DAC_SET_MOTOR_CURRENT(0xffff);
    DAC_SET_INPUT_CURRENT(0xffff);

    // change PWM frequency
    tim1.Init.Prescaler = 71; // 72 MHz / 72 = 1 MHz (1 us tick)
    tim1.Init.Period = kTonePeriod;
    HAL_TIM_PWM_Init(&tim1);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_2);
}

void MotorVibes::deinit()
{
    // deinit and restore previous values
    PID_MOTOR_PWM_TIMER->CCR1 = 0;
    PID_MOTOR_PWM_TIMER->CCR2 = 0;
    HAL_TIM_PWM_DeInit(&tim1);

    // restore current limits
    DAC_SET_MOTOR_CURRENT(motorCurrentLimit);
    DAC_SET_INPUT_CURRENT(inputCurrentLimit);

    // re-initialize timer with previous values
    tim1.Init.Prescaler = prescaler;
    tim1.Init.Period = period;
    HAL_TIM_PWM_Init(&tim1);
    __HAL_TIM_SET_AUTORELOAD(&tim1, arr);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_2);
}

void MotorVibes::playTone(uint32_t frequency)
{
    if (frequency == 0) {
        stopTone();
        return;
    }
    // change pwm frequency
    const uint32_t arr = F_CPU / ((kTonePeriod + 1) * frequency) - 1;
    PID_MOTOR_PWM_TIMER->ARR = arr;
    // play tone with reduced duty cycle to prevent the motor from spinning
    PID_MOTOR_PWM_TIMER->CCR1 = arr / kPWMDivider;
    PID_MOTOR_PWM_TIMER->CCR2 = 0;
}

void MotorVibes::stopTone()
{
    // 100% brake in case it spinned up
    PID_MOTOR_PWM_TIMER->CCR1 = PID_MOTOR_PWM_TIMER->ARR;
    PID_MOTOR_PWM_TIMER->CCR2 = PID_MOTOR_PWM_TIMER->ARR;
}

#endif

#if HAVE_USE_LV_MEM_ALLOC

// == use lvgl memory allocation functions for new/delete operators ===

#include <new>
#include "lvgl.h"

void *operator new(std::size_t size) noexcept
{
    void *ptr = lv_mem_alloc(size);
    #if LV_MEM_DEBUG
    if (!ptr) {
        DEBUG_PRINT(DebugType::MEM, "operator new failed size=%u", static_cast<unsigned>(size));
        throw std::bad_alloc();
    }
    #endif
    return ptr;
}

void operator delete(void *ptr) noexcept
{
    lv_mem_free(ptr);
}

void operator delete(void *ptr, std::size_t) noexcept
{
    lv_mem_free(ptr);
}

void *operator new[](std::size_t size) noexcept
{
    void *ptr = lv_mem_alloc(size);
    #if LV_MEM_DEBUG
    if (!ptr) {
        DEBUG_PRINT(DebugType::MEM, "operator new[] failed size=%u", static_cast<unsigned>(size));
        throw std::bad_alloc();
    }
    #endif
    return ptr;
}

void operator delete[](void *ptr) noexcept
{
    lv_mem_free(ptr);
}

void operator delete[](void *ptr, std::size_t) noexcept
{
    lv_mem_free(ptr);
}

void *operator new(std::size_t size, const std::nothrow_t &) noexcept
{
    void *ptr = lv_mem_alloc(size);
    #if LV_MEM_DEBUG
    if (!ptr) {
        DEBUG_PRINT(DebugType::MEM, "operator new failed size=%u", static_cast<unsigned>(size));
        return nullptr;
    }
    #endif
    return ptr;
}

void operator delete(void *ptr, const std::nothrow_t &) noexcept
{
    lv_mem_free(ptr);
}

void *operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
    void *ptr = lv_mem_alloc(size);
    #if LV_MEM_DEBUG
    if (!ptr) {
        DEBUG_PRINT(DebugType::MEM, "operator new[] failed size=%u", static_cast<unsigned>(size));
        return nullptr;
    }
    #endif
    return ptr;
}

void operator delete[](void *ptr, const std::nothrow_t &) noexcept
{
    lv_mem_free(ptr);
}

#endif
