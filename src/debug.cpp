/**
  Author: sascha_lammers@gmx.de
*/

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

    debug_swd_write(buf);
}

#endif