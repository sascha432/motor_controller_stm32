/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>
#include <type_traits>
#include "main.h"

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

// use lv_mem_alloc() for new/delete operators
#ifndef HAVE_USE_LV_MEM_ALLOC
#define HAVE_USE_LV_MEM_ALLOC               1
#endif

/**
 * @brief Calculate the auto-reload register value for a given PWM frequency with no prescaler (PSC=0)
 *
 * @param frequency PWM frequency in Hz
 * @return uint16_t max PWM level
 */
static constexpr uint16_t kPWMFrequencyToARR(uint32_t frequency)
{
    uint32_t tmp = F_CPU / frequency;
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
    return F_CPU / (arr + 1);
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
    WATCHDOG_TIMEOUT,
    WATCHDOG_TICK_TIMEOUT,
    HARD_FAULT_HANDLER,
    MEM_MANAGE_HANDLER,
    NMI_HANDLER,
    BUS_FAULT_HANDLER,
    USAGE_FAULT_HANDLER,
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
    static void init()
    {
        feed();
    }

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
        call_default_error_handler(InterruptErrorType::WATCHDOG_TICK_TIMEOUT);
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
        (void)filteredValue;
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

/**
 * @brief Check if a value is divisible by a divisor
 *
 * @param value The value to check
 * @param divisor The divisor to check against
 * @return true if value is divisible by divisor
 * @return false otherwise
 */
template<uint32_t DIVISOR>
inline bool kIsDivisible(uint32_t value)
{
    if constexpr ((DIVISOR & (DIVISOR - 1)) == 0) {
        return (value & (DIVISOR - 1)) == 0;
    }
    else {
        return (value % DIVISOR) == 0;
    }
}

static constexpr float kFloatToUint16Multiplier = 65535.0f;
