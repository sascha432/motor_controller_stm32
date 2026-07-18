/**
  Author: sascha_lammers@gmx.de

  Handle debug output to serial/swo/usb etc...
*/

#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stm32f1xx.h>

// === debug settings ===

#define DEBUG_OUTPUT_NONE       0
#define DEBUG_OUTPUT_SERIAL     1
#define DEBUG_OUTPUT_SERIAL4    2
#define DEBUG_OUTPUT_SWD        3
#define DEBUG_OUTPUT_USB        4

#ifndef DEBUG_OUTPUT
    #define DEBUG_OUTPUT        DEBUG_OUTPUT_SERIAL
#endif

#define DEBUG_ERROR             0
#define DEBUG_WARNING           1
#define DEBUG_NOTICE            2
#define DEBUG_DEBUG             3
#define DEBUG_ALL               99
#define DEBUG_LEVEL             DEBUG_ALL

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

    #define DEBUG_PRINT(level, msg, ...) do {} while(0)

#elif DEBUG_OUTPUT == DEBUG_OUTPUT_SERIAL

    #define DEBUG_PRINT(level, msg, ...) \
        do { \
            if (level <= DEBUG_LEVEL) { \
                char _debug_function[96]; \
                Serial.printf("[%06lu] %s " msg "\n", millis(), debug_function_name(DEBUG_FUNCTION_SIG, _debug_function, sizeof(_debug_function)), ##__VA_ARGS__); \
            } \
        } while(0)

#elif DEBUG_OUTPUT == DEBUG_OUTPUT_SERIAL4

    #define DEBUG_PRINT(level, msg, ...) \
        do { \
            if (level <= DEBUG_LEVEL) { \
                char _debug_function[96]; \
                Serial4.printf("[%06lu] %s " msg "\n", millis(), debug_function_name(DEBUG_FUNCTION_SIG, _debug_function, sizeof(_debug_function)), ##__VA_ARGS__); \
            } \
        } while(0)

#elif DEBUG_OUTPUT == DEBUG_OUTPUT_SWD

    void debug_swd_printf(const char *fmt, ...);

    #define DEBUG_PRINT(level, msg, ...) \
        do { \
            if (level <= DEBUG_LEVEL) { \
                char _debug_function[96]; \
                debug_swd_printf("[%06lu] %s " msg "\n", millis(), debug_function_name(DEBUG_FUNCTION_SIG, _debug_function, sizeof(_debug_function)), ##__VA_ARGS__); \
            } \
        } while(0)

#else
    #error invalid DEBUG_OUTPUT value
#endif
