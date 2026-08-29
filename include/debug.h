/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stm32f1xx.h>
#include "tick_profiler.h"

// === data for SWD PID tuning ===

#define SWO_DATA_FIXED_RAM_ADDRESS 0x2000F000UL

/**
 * @brief SWO class for sending debug output and PID tuning data via SWO
 *
 */
struct SWO
{
    static constexpr uint32_t kSwoTimeoutMillis = 5U;
    static constexpr uint8_t kITMPort = 0U;
    static constexpr uint8_t kPidPort = 1U;
    static constexpr uint8_t kScreenshotPort = 2U;

    enum class EnableState : uint32_t {
        DISABLED = 0,
        SWO = 1,
        #if HAVE_USB_DEVICE
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
        volatile uint16_t inputCurrentLimit;
        volatile uint8_t currentLimitStrength;
        DataType() :
            Kp(0),
            Ki(0),
            Kd(0),
            antiWindup(0),
            rpm(0),
            enabled(EnableState::DISABLED),
            changed(false),
            EEPROM{0, false},
            sendScreenshot(false),
            inputCurrentLimit(0),
            currentLimitStrength(0)
        {}
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
#define DEBUG_OUTPUT_SWO        2
#define DEBUG_OUTPUT_USB        3

#ifndef DEBUG_OUTPUT
    #define DEBUG_OUTPUT        DEBUG_OUTPUT_NONE
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
    #else
        #define __DEBUG__BUILD__ "DEBUG"
    #endif
#else
    #if HAVE_USB_DEVICE
        #define __DEBUG__BUILD__ "USB"
    #else
        #define __DEBUG__BUILD__ "SWO"
    #endif
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
    #if LV_USE_LOG
    LVGL = 0x80,
    #endif
    ALL = 0xFFFFFFFF
};

// old macros
#define DEBUG_ERROR                     DebugType::ERROR
#define DEBUG_WARNING                   DebugType::WARNING
#define DEBUG_NOTICE                    DebugType::NOTICE
#define DEBUG_DEBUG                     DebugType::INFO
#define DEBUG_ALL                       DebugType::ALL
#define DEBUG_LEVEL_RESULT(result)      ((result) ? DebugType::INFO : DebugType::ERROR)

// debug level to output
// #define DEBUG_LEVEL             static_cast<DebugType>((uint32_t)DebugType::ERROR|(uint32_t)DebugType::WARNING|(uint32_t)DebugType::NOTICE)
// #define DEBUG_LEVEL             static_cast<DebugType>((uint32_t)DebugType::ALL&~(uint32_t)DebugType::UI)
#define DEBUG_LEVEL             static_cast<DebugType>(static_cast<uint32_t>(DebugType::ALL))

inline const char *debugLevelToString(DebugType level)
{
    switch(level) {
        case DebugType::WARNING: return "WARNING";
        case DebugType::NOTICE: return "NOTICE";
        case DebugType::INFO: return "INFO";
        case DebugType::MEM: return "MEM";
        case DebugType::UI: return "UI";
        case DebugType::PID: return "PID";
        #if LV_USE_LOG
        case DebugType::LVGL: return "LVGL";
        #endif
        case DebugType::ERROR:
        default:
            return "ERROR";
    }
}

// === debug helpers ===

constexpr const char *debug_source_filename(const char *file)
{
    const char *filename = file;
    for (const char *cursor = file; *cursor != '\0'; ++cursor) {
        if (*cursor == '/' || *cursor == '\\') {
            filename = cursor + 1;
        }
    }
    return filename;
}

#define DEBUG_SOURCE_FILENAME debug_source_filename(__FILE__)

#if DEBUG_OUTPUT == DEBUG_OUTPUT_NONE

    #define DEBUG_PRINT_MSG(level, msg, ...) do {} while(0)
    #define DEBUG_PRINT(level, msg, ...) do {} while(0)

#elif DEBUG_OUTPUT == DEBUG_OUTPUT_SWO

    void debug_swd_printf(const char *fmt, ...);

    #define DEBUG_PRINTF_FUNC debug_swd_printf

#elif DEBUG_OUTPUT == DEBUG_OUTPUT_USB

    void debug_usb_printf(const char *fmt, ...);

    #define DEBUG_PRINTF_FUNC debug_usb_printf

#else

    #error invalid DEBUG_OUTPUT value

#endif

#if DEBUG_OUTPUT == DEBUG_OUTPUT_NONE

    #define DEBUG_PRINT_MSG(level, msg, ...) do {} while(0)
    #define DEBUG_PRINT(level, msg, ...) do {} while(0)
    #define DEBUG_PRINT_SRC(level, msg, ...) do {} while(0)

#else

    #define DEBUG_PRINT_MSG(level, msg, ...) \
        do { \
            if (static_cast<uint32_t>(DEBUG_LEVEL) & static_cast<uint32_t>(level)) { \
                DEBUG_PRINTF_FUNC(msg, ##__VA_ARGS__); \
            } \
        } while(0)

    #define DEBUG_PRINT(level, msg, ...) \
        do { \
            if (static_cast<uint32_t>(DEBUG_LEVEL) & static_cast<uint32_t>(level)) { \
                DEBUG_PRINTF_FUNC("[%06lu] %s " msg "\n", HAL_GetTick(), debugLevelToString(level), ##__VA_ARGS__); \
            } \
        } while(0)

    #define DEBUG_PRINT_SRC(level, msg, ...) \
        do { \
            if (static_cast<uint32_t>(DEBUG_LEVEL) & static_cast<uint32_t>(level)) { \
                DEBUG_PRINTF_FUNC("[%06lu] %s %s:%u " msg "\n", HAL_GetTick(), debugLevelToString(level), DEBUG_SOURCE_FILENAME, __LINE__, ##__VA_ARGS__); \
            } \
        } while(0)

#endif
