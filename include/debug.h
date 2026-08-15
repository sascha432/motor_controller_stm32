/**
  Author: sascha_lammers@gmx.de

  Handle debug output to serial/swo/usb etc...
*/

#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stm32f1xx.h>

// === data for SWD PID tuning ===

#define SWO_DATA_FIXED_RAM_ADDRESS 0x2000F000UL

/**
 * @brief SWO class for sending debug output and PID tuning data via SWO
 *
 */
struct SWO
{
    static constexpr uint32_t kSwoTimeoutMillis = 5U;

    enum class EnableState : uint32_t {
        DISABLED = 0,
        SWO = 1,
        #if HAVE_USB_DEVICE || HAVE_SERIAL
            SERIAL = 2,
        #endif
    };

    struct DataType {
        volatile float Kp;
        volatile float Ki;
        volatile float Kd;
        volatile uint16_t antiWindup;
        volatile uint16_t rpm;
        volatile EnableState enabled;
        volatile bool changed;
        struct {
            volatile uint32_t address;
            volatile bool commit;
        } EEPROM;
        volatile bool sendScreenshot;
        DataType() : Kp(0), Ki(0), Kd(0), antiWindup(0), rpm(0), enabled(EnableState::DISABLED), changed(false), EEPROM{0, false}, sendScreenshot(false) {}
    };
    static constexpr size_t kDataTypeSize = sizeof(DataType);
    static_assert(sizeof(DataType) % 4 == 0, "DataType size must be a multiple of 4");

    static void init();
    static void deinit();

    /**
     * @brief Write data to the specified SWO port
     *
     * @param port The SWO port number to write to
     * @param data Pointer to the data to write
     * @param size Number of bytes to write
     * @return size_t Number of bytes actually written
     */
    static size_t write(uint8_t port, const void *data, size_t size);

    /**
     * @brief Check global states and if the port is enabled
     *
     * @param port The SWO port number to check
     * @return true if the port is enabled
     * @return false if the port is not enabled
     */
    inline static bool isPortWritable(uint8_t port)
    {
        return (state) && (ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port));
    }

    /**
     * @brief Wait until the port is ready to write or timeout occurs
     *
     * @param port The SWO port number to check
     * @return true if the port is ready to write
     * @return false if the port is not ready to write within the timeout
     */
    inline static bool waitReadyPort(uint8_t port)
    {
        const uint32_t start = HAL_GetTick();
        while (ITM->PORT[port].u32 == 0) {
            if ((HAL_GetTick() - start) >= kSwoTimeoutMillis) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Write a value of any type to the specified SWO port
     *
     * @tparam T The type of the value to write
     * @param port The SWO port number to write to
     * @param value The value to write
     * @return size_t Number of bytes actually written
     */
    template<typename T>
    inline static size_t write(uint8_t port, const T &value)
    {
        return write(port, &value, sizeof(value));
    }

    /**
     * @brief Write a single byte to the specified SWO port
     *
     * @param port The SWO port number to write to
     * @param value The byte value to write
     * @return size_t Number of bytes actually written
     */
    inline static size_t writeByte(uint8_t port, uint8_t value)
    {
        return write(port, &value, sizeof(value));
    }

    /**
     * @brief Write data to the specified SWO port without checking if the port is enabled
     *
     * @param port The SWO port number to write to
     * @param data Pointer to the data to write
     * @param size Number of bytes to write
     * @return size_t Number of bytes actually written
     */
    inline static size_t writeFast(uint8_t port, const void *data, size_t size)
    {
        const uint8_t *bytes = static_cast<const uint8_t *>(data);
        size_t sent = 0;
        while (sent < size) {
            if (!waitReadyPort(port)) {
                break;
            }
            const size_t remaining = size - sent;
            if (remaining >= 4U) {
                ITM->PORT[port].u32 = *reinterpret_cast<const uint32_t *>(&bytes[sent]);
                sent += 4U;
            }
            else if (remaining >= 2U) {
                ITM->PORT[port].u16 = *reinterpret_cast<const uint16_t *>(&bytes[sent]);
                sent += 2U;
            }
            else {
                ITM->PORT[port].u8 = bytes[sent++];
            }
        }
        return sent;
    }

    /**
     * @brief Write a value of any type to the specified SWO port without checking if the port is enabled
     *
     * @tparam T The type of the value to write
     * @param port The SWO port number to write to
     * @param value The value to write
     * @return size_t Number of bytes actually written
     */
    template<typename T>
    inline static size_t writeFast(uint8_t port, const T &value)
    {
        return writeFast(port, &value, sizeof(value));
    }

    /**
     * @brief Write a single byte to the specified SWO port without checking if the port is enabled
     *
     * @param port The SWO port number to write to
     * @param value The byte value to write
     * @return size_t Number of bytes actually written
     */
    inline static size_t writeByteFast(uint8_t port, uint8_t value)
    {
        if (!waitReadyPort(port)) {
            return 0;
        }
        ITM->PORT[port].u8 = value;
        return 1;
    }

    static bool state;
    static DataType data;
};

// === debug settings ===

#define DEBUG_OUTPUT_NONE       0
#define DEBUG_OUTPUT_SERIAL     1
#define DEBUG_OUTPUT_SWO        2
#define DEBUG_OUTPUT_USB        3

#ifndef DEBUG_OUTPUT
    #define DEBUG_OUTPUT        DEBUG_OUTPUT_SERIAL
#endif

#if defined(DEBUG) && !DEBUG
    #undef DEBUG_OUTPUT
    #define DEBUG_OUTPUT        DEBUG_OUTPUT_NONE
#endif

#if defined(DEBUG) && DEBUG
    #if DEBUG_OUTPUT == DEBUG_OUTPUT_SWO
        #define __DEBUG__BUILD__ "DEBUG SWO"
    #elif DEBUG_OUTPUT == DEBUG_OUTPUT_USB
        #define __DEBUG__BUILD__ "DEBUG USB"
    #elif DEBUG_OUTPUT == DEBUG_OUTPUT_SERIAL
        #define __DEBUG__BUILD__ "DEBUG UART"
    #else
        #define __DEBUG__BUILD__ "DEBUG"
    #endif
#else
    #define __DEBUG__BUILD__ ""
#endif

enum class DebugType : uint32_t
{
    ERROR = 0x01,
    WARNING = 0x02,
    NOTICE = 0x04,
    INFO = 0x08,
    MEM = 0x10,
    UI = 0x20,
    PID = 0x40,
    ALL = 0xFFFFFFFF
};

// old macros
#define DEBUG_ERROR             DebugType::ERROR
#define DEBUG_WARNING           DebugType::WARNING
#define DEBUG_NOTICE            DebugType::NOTICE
#define DEBUG_DEBUG             DebugType::INFO
#define DEBUG_ALL               DebugType::ALL

// debug level to output
// #define DEBUG_LEVEL             static_cast<DebugType>((uint32_t)DebugType::ERROR|(uint32_t)DebugType::WARNING|(uint32_t)DebugType::NOTICE)
// #define DEBUG_LEVEL             static_cast<DebugType>((uint32_t)DebugType::ALL&~(uint32_t)DebugType::UI)
#define DEBUG_LEVEL             static_cast<DebugType>((uint32_t)DebugType::ALL)

inline const char *debugLevelToString(DebugType level)
{
    switch(level) {
        case DebugType::WARNING: return "WARNING";
        case DebugType::NOTICE: return "NOTICE";
        case DebugType::INFO: return "INFO";
        case DebugType::MEM: return "MEM";
        case DebugType::UI: return "UI";
        case DebugType::PID: return "PID";
        case DebugType::ERROR:
        default:
            return "ERROR";
    }
}

// === debug helpers ===

#if defined(__GNUC__) || defined(__clang__)
    #define DEBUG_FUNCTION_SIG __PRETTY_FUNCTION__
#elif defined(_MSC_VER)
    #define DEBUG_FUNCTION_SIG __FUNCSIG__
#else
    #define DEBUG_FUNCTION_SIG __func__
#endif

const char *debug_function_name(const char *signature, char *out, size_t outSize);
void debug_init(void);

#if DEBUG_OUTPUT == DEBUG_OUTPUT_NONE

    #define DEBUG_PRINT_MSG(level, msg, ...) do {} while(0)
    #define DEBUG_PRINT(level, msg, ...) do {} while(0)

#elif DEBUG_OUTPUT == DEBUG_OUTPUT_SERIAL

    #define DEBUG_PRINT_MSG(level, msg, ...) \
        do { \
            if ((uint32_t)(DEBUG_LEVEL) & (uint32_t)(level)) { \
                Serial.printf(msg, ##__VA_ARGS__); \
            } \
        } while(0)

    #define DEBUG_PRINT(level, msg, ...) \
        do { \
            if ((uint32_t)(DEBUG_LEVEL) & (uint32_t)(level)) { \
                char _debug_function[96]; \
                Serial.printf("[%06lu] %s %s " msg "\n", HAL_GetTick(), debugLevelToString(level), debug_function_name(DEBUG_FUNCTION_SIG, _debug_function, sizeof(_debug_function)), ##__VA_ARGS__); \
            } \
        } while(0)

#elif DEBUG_OUTPUT == DEBUG_OUTPUT_SWO

    void debug_swd_printf(const char *fmt, ...);

    #define DEBUG_PRINT_MSG(level, msg, ...) \
        do { \
            if ((uint32_t)(DEBUG_LEVEL) & (uint32_t)(level)) { \
                debug_swd_printf(msg, ##__VA_ARGS__); \
            } \
        } while(0)

    #define DEBUG_PRINT(level, msg, ...) \
        do { \
            if ((uint32_t)(DEBUG_LEVEL) & (uint32_t)(level)) { \
                char _debug_function[96]; \
                debug_swd_printf("[%06lu] %s %s " msg "\n", HAL_GetTick(), debugLevelToString(level), debug_function_name(DEBUG_FUNCTION_SIG, _debug_function, sizeof(_debug_function)), ##__VA_ARGS__); \
            } \
        } while(0)


#elif DEBUG_OUTPUT == DEBUG_OUTPUT_USB

    void debug_usb_printf(const char *fmt, ...);

    #define DEBUG_PRINT_MSG(level, msg, ...) \
        do { \
            if ((uint32_t)(DEBUG_LEVEL) & (uint32_t)(level)) { \
                debug_usb_printf(msg, ##__VA_ARGS__); \
            } \
        } while(0)

    #define DEBUG_PRINT(level, msg, ...) \
        do { \
            if ((uint32_t)(DEBUG_LEVEL) & (uint32_t)(level)) { \
                char _debug_function[96]; \
                debug_usb_printf("[%06lu] %s %s " msg "\n", HAL_GetTick(), debugLevelToString(level), debug_function_name(DEBUG_FUNCTION_SIG, _debug_function, sizeof(_debug_function)), ##__VA_ARGS__); \
            } \
        } while(0)

#else
    #error invalid DEBUG_OUTPUT value
#endif


// === profiler ===

#if HAVE_DWT_TICK_PROFILER

struct TickProfiler {

    struct AverageSumType
    {
        volatile uint32_t sum;
        volatile uint32_t count;
        volatile uint32_t started;
        volatile uint32_t numberOfSamples;
        AverageSumType() : sum(0), count(0), started(0), numberOfSamples(0) {}
    };

    static inline void start(uint32_t numberOfSamples = 128, uint32_t slot = 0)
    {
        slots[slot].numberOfSamples = numberOfSamples;
        slots[slot].started = DWT->CYCCNT;
    }

    static inline void stop(uint32_t slot = 0)
    {
        volatile const uint32_t now = DWT->CYCCNT;
        if (slots[slot].numberOfSamples < 2) {
            slots[slot].sum = (now - slots[slot].started);
            slots[slot].count = 1;
        }
        else {
            slots[slot].sum += (now - slots[slot].started);
            if (++slots[slot].count >= slots[slot].numberOfSamples) {
                slots[slot].sum -= slots[slot].sum / 16;
                slots[slot].count -= slots[slot].count / 16;
            }
        }
    }

    static inline uint32_t getTicks(uint32_t slot = 0)
    {
        return (slots[slot].count ? (slots[slot].sum / slots[slot].count) : 0);
    }

    static void snprintf(char *buf, size_t size, uint32_t slot = 0)
    {
        ::snprintf(buf, size, "%u\n", (unsigned)getTicks(slot));
    }

    static AverageSumType slots[16];
};

#endif
