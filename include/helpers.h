/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>
#include <type_traits>
#include <stm32f1xx.h>

/**
 * @brief Float printf converters
 *
 * printf("32.1=" SPRINTF_FP1_FMT "\n", CONVERT_TO_FP2(32.1))
 * printf("123.45=" SPRINTF_FP2_FMT "\n", CONVERT_TO_FP2(123.56))
 *
 */
#define CONVERT_TO_FP1(value)               (int)(value / 1000), ((unsigned)(value / 100) % 10)
#define CONVERT_TO_FP2(value)               (int)(value / 1000), ((unsigned)(value / 10) % 100)
#define CONVERT_TO_FP6(value)               (int)(value / 1000000), ((unsigned)(value / 100000) % 1000000)

#define DEGREE_UTF8                         "\xC2\xB0"

#define SPRINTF_FP1_FMT                     "%d.%u"
#define SPRINTF_FP2_FMT                     "%d.%02u"
#define SPRINTF_FP6_FMT                     "%d.%05u"

#define sizeof_array(arr)                   (sizeof(arr) / sizeof(arr[0]))

// === define arduino pin macros if not defined ===
#ifndef PA0
#if defined(STM32F107xC)
#define PA0 0
#define PA1 1
#define PA2 2
#define PA3 3
#define PA4 4
#define PA5 5
#define PA6 6
#define PA7 7
#define PA8 8
#define PA9 9
#define PA10 10
#define PA11 11
#define PA12 12
#define PA13 13
#define PA14 14
#define PA15 15

#define PB0 16
#define PB1 17
#define PB2 18
#define PB3 19
#define PB4 20
#define PB5 21
#define PB6 22
#define PB7 23
#define PB8 24
#define PB9 25
#define PB10 26
#define PB11 27
#define PB12 28
#define PB13 29
#define PB14 30
#define PB15 31

#define PC0 32
#define PC1 33
#define PC2 34
#define PC3 35
#define PC4 36
#define PC5 37
#define PC6 38
#define PC7 39
#define PC8 40
#define PC9 41
#define PC10 42
#define PC11 43
#define PC12 44
#define PC13 45
#define PC14 46
#define PC15 47

#define PD0 48
#define PD1 49
#define PD2 50
#define PD3 51
#define PD4 52
#define PD5 53
#define PD6 54
#define PD7 55
#define PD8 56
#define PD9 57
#define PD10 58
#define PD11 59
#define PD12 60
#define PD13 61
#define PD14 62
#define PD15 63

#define PE0 64
#define PE1 65
#define PE2 66
#define PE3 67
#define PE4 68
#define PE5 69
#define PE6 70
#define PE7 71
#define PE8 72
#define PE9 73
#define PE10 74
#define PE11 75
#define PE12 76
#define PE13 77
#define PE14 78
#define PE15 79
#else
#error pins undefined for this platform
#endif
#endif

/**
 * @brief translate arduino digital pin number to GPIO pin number
 *
 * @tparam PIN_TYPE
 * @param pin Arduino digital pin number
 * @return constexpr uint8_t GPIO pin number 0-15
 */
template<uint32_t PIN>
constexpr uint8_t digitalPinToBit()
{
    switch(PIN) {
        case PA0: case PB0: case PC0: case PD0: case PE0:
            return 0;
        case PA1: case PB1: case PC1: case PD1: case PE1:
            return 1;
        case PA2: case PB2: case PC2: case PD2: case PE2:
            return 2;
        case PA3: case PB3: case PC3: case PD3: case PE3:
            return 3;
        case PA4: case PB4: case PC4: case PD4: case PE4:
            return 4;
        case PA5: case PB5: case PC5: case PD5: case PE5:
            return 5;
        case PA6: case PB6: case PC6: case PD6: case PE6:
            return 6;
        case PA7: case PB7: case PC7: case PD7: case PE7:
            return 7;
        case PA8: case PB8: case PC8: case PD8: case PE8:
            return 8;
        case PA9: case PB9: case PC9: case PD9: case PE9:
            return 9;
        case PA10: case PB10: case PC10: case PD10: case PE10:
            return 10;
        case PA11: case PB11: case PC11: case PD11: case PE11:
            return 11;
        case PA12: case PB12: case PC12: case PD12: case PE12:
            return 12;
        case PA13: case PB13: case PC13: case PD13: case PE13:
            return 13;
        case PA14: case PB14: case PC14: case PD14: case PE14:
            return 14;
        case PA15: case PB15: case PC15: case PD15: case PE15:
            return 15;
    }
    return 0;
}

/**
 * @brief translate arduino pin macro to GPIOx_BASE address
 *
 * @tparam PIN Arduino pin macro (e.g. PA0, PB10, PD7)
 * @return constexpr uint32_t GPIOx_BASE address, or 0 for invalid port
 */
template<uint32_t PIN>
constexpr uint32_t digitalPinToGPIOBase()
{
    switch(PIN) {
        case PA0: case PA1: case PA2: case PA3: case PA4: case PA5: case PA6: case PA7: case PA8: case PA9: case PA10: case PA11: case PA12: case PA13: case PA14: case PA15:
            return GPIOA_BASE;
        case PB0: case PB1: case PB2: case PB3: case PB4: case PB5: case PB6: case PB7: case PB8: case PB9: case PB10: case PB11: case PB12: case PB13: case PB14: case PB15:
            return GPIOB_BASE;
        case PC0: case PC1: case PC2: case PC3: case PC4: case PC5: case PC6: case PC7: case PC8: case PC9: case PC10: case PC11: case PC12: case PC13: case PC14: case PC15:
            return GPIOC_BASE;
        case PD0: case PD1: case PD2: case PD3: case PD4: case PD5: case PD6: case PD7: case PD8: case PD9: case PD10: case PD11: case PD12: case PD13: case PD14: case PD15:
            return GPIOD_BASE;
        case PE0: case PE1: case PE2: case PE3: case PE4: case PE5: case PE6: case PE7: case PE8: case PE9: case PE10: case PE11: case PE12: case PE13: case PE14: case PE15:
            return GPIOE_BASE;
    }
    return 0;
}

/**
 * @brief translate arduino pin macro to GPIO_TypeDef pointer
 *
 * @tparam PIN Arduino pin macro (e.g. PA0, PB10, PD7)
 * @return constexpr GPIO_TypeDef* GPIO_TypeDef pointer, or nullptr for invalid port
 */
template<uint32_t PIN>
constexpr GPIO_TypeDef *digitalPinToGPIO()
{
    return reinterpret_cast<GPIO_TypeDef *>(digitalPinToGPIOBase<PIN>());
}

/**
 * @brief translate arduino digital pin number to GPIO configuration register shift value
 *
 * @tparam PIN Arduino digital pin number
 * @tparam BITS Defaults to 4, number of bits per pin in the configuration register
 * @return constexpr uint32_t
 */
template<uint32_t PIN, size_t BITS = 4>
constexpr uint32_t digitalPinShift()
{
    return (digitalPinToBit<PIN>() & 7u) * BITS;
}

/**
 * @brief Translate all arduino pins to HAL gpio pins
 *
 * @tparam PIN
 * @return constexpr uint32_t
 */
template<uint8_t PIN>
constexpr uint32_t digitalPinToHAL()
{
    switch(PIN) {
        case PA0: case PB0: case PC0: case PD0: case PE0: return GPIO_PIN_0;
        case PA1: case PB1: case PC1: case PD1: case PE1: return GPIO_PIN_1;
        case PA2: case PB2: case PC2: case PD2: case PE2: return GPIO_PIN_2;
        case PA3: case PB3: case PC3: case PD3: case PE3: return GPIO_PIN_3;
        case PA4: case PB4: case PC4: case PD4: case PE4: return GPIO_PIN_4;
        case PA5: case PB5: case PC5: case PD5: case PE5: return GPIO_PIN_5;
        case PA6: case PB6: case PC6: case PD6: case PE6: return GPIO_PIN_6;
        case PA7: case PB7: case PC7: case PD7: case PE7: return GPIO_PIN_7;
        case PA8: case PB8: case PC8: case PD8: case PE8: return GPIO_PIN_8;
        case PA9: case PB9: case PC9: case PD9: case PE9: return GPIO_PIN_9;
        case PA10: case PB10: case PC10: case PD10: case PE10: return GPIO_PIN_10;
        case PA11: case PB11: case PC11: case PD11: case PE11: return GPIO_PIN_11;
        case PA12: case PB12: case PC12: case PD12: case PE12: return GPIO_PIN_12;
        case PA13: case PB13: case PC13: case PD13: case PE13: return GPIO_PIN_13;
        case PA14: case PB14: case PC14: case PD14: case PE14: return GPIO_PIN_14;
        case PA15: case PB15: case PC15: case PD15: case PE15: return GPIO_PIN_15;
    }
    return 0;
}

/**
 * @brief Read the state of a digital pin
 *
 * @tparam PIN Arduino digital pin number
 * @return true pin is HIGH
 * @return false pin is LOW
 */
template<uint32_t PIN>
inline bool digitalRead()
{
    return (digitalPinToGPIO<PIN>()->IDR & (1 << digitalPinToBit<PIN>())) != 0;
}

/**
 * @brief Enable the GPIO clock for the specified Arduino digital pin
 *
 * @tparam PIN
 */
template<uint8_t PIN>
inline void __HAL_RCC_GPIOx_CLK_ENABLE()
{
    switch (PIN >> 4) {
        case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
        case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
        case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
        case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
        case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
    }
}

// Helper function, constexpr cannot return volatile IO address
template<uint32_t PIN>
constexpr uint32_t __GPIO_CRx_REG()
{
    return (digitalPinToBit<PIN>() < 8) ? (uint32_t)&digitalPinToGPIO<PIN>()->CRL : (uint32_t)&digitalPinToGPIO<PIN>()->CRH;
}

/**
 * @brief Get configuration register for GPIO pin
 *
 * @param gpio_addr GPIOx_BASE address
 * @param pin Arduino PIN number
 * @return volatile uint32_t&
 */
template<uint32_t PIN>
inline volatile uint32_t &GPIO_CRx_REG()
{
    return *reinterpret_cast<volatile uint32_t *>(__GPIO_CRx_REG<PIN>());
}

/**
 * @brief Set the specified digital pin HIGH
 *
 * @tparam PIN Arduino digital pin number
 */
template <uint32_t PIN>
inline void digitalWriteHigh()
{
    digitalPinToGPIO<PIN>()->BSRR = (1 << digitalPinToBit<PIN>());
}

/**
 * @brief Set the specified digital pin LOW
 *
 * @tparam PIN Arduino digital pin number
 */
template <uint32_t PIN>
inline void digitalWriteLow()
{
    digitalPinToGPIO<PIN>()->BSRR = (1 << (digitalPinToBit<PIN>() + 16));
}

/**
 * @brief Calculate the auto-reload register value for a given PWM frequency with no prescaler (PSC=0)
 *
 * @param frequency PWM frequency in Hz
 * @return uint16_t max PWM level
 */
static constexpr uint16_t kPWMFrequencyToARR(uint32_t frequency)
{
    uint32_t tmp = 72000000 / frequency;
    if (tmp > 0xFFFF) {
        tmp = 0xFFFF;
    }
    return static_cast<uint16_t>(tmp);
}

/**
 * @brief Calculate the PWM frequency for a given auto-reload register value (ARR)
 *
 * @param arr
 * @return constexpr uint32_t
 */
static constexpr uint32_t kARRToPWMFrequency(uint16_t arr)
{
    return 72000000 / (arr + 1);
}

/**
 * @brief Delay function for up to 65535 microseconds
 *
 * @tparam microseconds delay
 */
template<uint32_t US>
inline void delay_us()
{
    static_assert(US <= 0xFFFF, "delay_us() value too high for 16bit timer");
    const uint16_t start = TIM7->CNT;
    while ((uint16_t)(TIM7->CNT - start) < (uint16_t)US) {
    }
}

/**
 * @brief Delay function
 *
 * @param us microseconds
 */
void delay_us(uint32_t us);

/**
 * @brief Convert float to string helper
 */
struct FloatToString
{
    static void convert(char *buffer, size_t size, float value, uint8_t precision = 6);
    static void convertTrimmed(char *buffer, size_t size, float value, uint8_t precision = 6);
};

void float_to_string_convert(char *buffer, size_t size, float value, uint8_t precision = 6, bool trimTrailingZeros = false);

/**
 * @brief Convert float to string with specified precision
 *
 * @param buffer Output buffer
 * @param size Size of the output buffer
 * @param value Float value to convert
 * @param precision Number of decimal places
 */
inline void FloatToString::convert(char *buffer, size_t size, float value, uint8_t precision)
{
    float_to_string_convert(buffer, size, value, precision, false);
}

/**
 * @brief Convert float to string with specified precision and trim trailing zeros
 *
 * @param buffer Output buffer
 * @param size Size of the output buffer
 * @param value Float value to convert
 * @param precision Number of decimal places
 */
inline void FloatToString::convertTrimmed(char *buffer, size_t size, float value, uint8_t precision)
{
    float_to_string_convert(buffer, size, value, precision, true);
}

// uses static buffer to return pointer(s) to string(s) for debugging
const char *debugFloatToString(float value, uint8_t precision = 6, bool trimTrailingZeros = false);

/**
 * @brief Simple non-blocking Ring buffer implementation
 *
 * @tparam T
 * @tparam SIZE
 */
template<typename T, size_t SIZE>
class RingBuffer {
public:
    bool push(const T& item)
    {
        size_t next = (head + 1) % SIZE;

        if (next == tail)
            return false; // full

        buffer[head] = item;
        head = next;

        return true;
    }

    bool pop(T& item)
    {
        if (head == tail)
            return false; // empty

        item = buffer[tail];
        tail = (tail + 1) % SIZE;

        return true;
    }

    bool empty() const
    {
        return head == tail;
    }

    void clear()
    {
        __disable_irq();
        head = 0;
        tail = 0;
        __enable_irq();
    }

    bool full() const
    {
        return ((head + 1) % SIZE) == tail;
    }

    size_t available() const
    {
        if (head >= tail)
            return head - tail;

        return SIZE - tail + head;
    }

private:
    T buffer[SIZE];

    volatile size_t head = 0;
    volatile size_t tail = 0;
};

// global error handling
enum class InterruptErrorType : uint32_t {
    ERROR_HANDLER = 0,
    NMI_HANDLER,
    HARD_FAULT_HANDLER,
    MEM_MANAGE_HANDLER,
    BUS_FAULT_HANDLER,
    USAGE_FAULT_HANDLER,
    WATCHDOG_TIMEOUT,
};

extern InterruptErrorType interruptErrorType;

extern "C" void Error_Handler(void);

inline void call_default_error_handler(InterruptErrorType type)
{
    interruptErrorType = type;
    Error_Handler();
}

/**
 * @brief Watchdog helper class
 *
 */
struct WatchDog
{
    static constexpr uint32_t kTimeoutMs = 1000;    // timeout in milliseconds before watchdog triggers error handler

    /**
     * @brief Initialize the watchdog
     *
     */
    static void init();

    /**
     * @brief Disable the watchdog
     *
     */
    static void deinit();

    /**
     * @brief Reset the watchdog timeout counter
     *
     */
    static void feed();

    /**
     * @brief HAL_Delay() with feeding the watchdog
     *
     * @param ms milliseconds
     */
    static void delay(uint32_t ms);

    /**
     * @brief Watchdog tick handler, call this from SysTick_Handler()
     *
     */
    static void tickHandler();

    static volatile uint32_t ticks;
    static WWDG_HandleTypeDef watchdog;
};

inline void WatchDog::delay(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {
        feed();
    }
}

inline void WatchDog::tickHandler()
{
    WWDG->CR = 0x7F; // feed the watchdog
    if (++ticks >= kTimeoutMs) { // 1000ms timeout
        call_default_error_handler(InterruptErrorType::WATCHDOG_TIMEOUT);
    }
}

inline void WatchDog::feed()
{
    ticks = 0;
}

/**
 * @brief Return ((filteredValue * (FILTER - 1)) + value) / FILTER
 *
 * Note: This function uses 32bit operations if necessary to avoid overflow for 16bit values. It won't use 64bit operations for 32bit types of T
 *
 * @tparam T The type of the filtered value
 * @tparam FILTER The filter coefficient
 * @param filteredValue The current filtered value
 * @param value The new value to be filtered
 * @return T The updated filtered value
 */
template<typename T, T FILTER, typename FILTER_TYPE = std::conditional_t<std::is_signed_v<T>, int32_t, uint32_t>>
inline T filterValue(T filteredValue, T value)
{
    if constexpr (FILTER == 1) {
        return value;
    }
    else if constexpr (FILTER == 2) {
        return (static_cast<FILTER_TYPE>(filteredValue) + value) / 2;
    }
    else if constexpr ((FILTER & (FILTER - 1)) == 0) {
        // FILTER is a power of 2, use bitwise operations for efficiency
        return (filteredValue * static_cast<FILTER_TYPE>(FILTER) - filteredValue + value) / FILTER;
    }
    else {
        return (filteredValue * static_cast<FILTER_TYPE>(FILTER - 1) + value) / FILTER;
    }
}
