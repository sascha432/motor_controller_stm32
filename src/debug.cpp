/**
  Author: sascha_lammers@gmx.de
*/

#if ARDUINO
#include <Arduino.h>
#endif
#include "debug.h"

const char *debug_function_name(const char *signature, char *out, size_t outSize)
{
    if (!signature || !out || outSize == 0) {
        return "<?>";
    }

    // Strip trailing template substitution details like " [with ...]".
    const char *end = signature;
    while (*end && *end != '[') {
        ++end;
    }

    // Keep only the last token after return type, e.g. "Class::method()".
    const char *start = end;
    while (start > signature && *(start - 1) != ' ') {
        --start;
    }

    size_t len = static_cast<size_t>(end - start);
    if (len >= outSize) {
        len = outSize - 1;
    }

    memcpy(out, start, len);
    out[len] = '\0';
    return out;
}

#if DEBUG_OUTPUT == DEBUG_OUTPUT_SWD

void debug_swd_write(const char *msg)
{
    if (!msg) {
        return;
    }
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) {
        return;
    }
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U || (ITM->TER & 1UL) == 0U) {
        return;
    }

    while (*msg) {
        ITM_SendChar(static_cast<uint32_t>(*msg++));
    }
}

void debug_swd_printf(const char *fmt, ...)
{
    char buf[192];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    //if (strstr(buf, "EEPROM") || strstr(buf, "I2CHelper"))  
    {
        debug_swd_write(buf);
    }
}

static void debug_swd_init()
{
    // Enable TRCENA in DEMCR (Debug Exception and Monitor Control Register)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Unlock ITM (Instrumentation Trace Macrocell)
    ITM->LAR = 0xC5ACCE55;

    // Enable ITM and set the trace bus ID
    ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_TSENA_Msk | (1U << 16);

    // Enable stimulus port 0
    ITM->TER |= 1UL;
}

#endif

void debug_init(void) 
{
    #if DEBUG_OUTPUT == DEBUG_OUTPUT_SWD
        debug_swd_init();
    #elif DEBUG_OUTPUT == DEBUG_OUTPUT_SERIAL || DEBUG_OUTPUT == DEBUG_OUTPUT_USB
        Serial.begin(115200);
    #elif DEBUG_OUTPUT == DEBUG_OUTPUT_SERIAL4
        Serial4.begin(115200);
    #endif
}
