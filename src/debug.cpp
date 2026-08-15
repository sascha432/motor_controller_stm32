/**
  Author: sascha_lammers@gmx.de
*/

#include "debug.h"
#include "helpers.h"

static inline void debug_swd_init()
{
    // Enable TRCENA in DEMCR (Debug Exception and Monitor Control Register)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Route asynchronous SWO on PB3 and enable trace pins.
    DBGMCU->CR &= ~DBGMCU_CR_TRACE_MODE;
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;

    // Configure TPIU for NRZ/async SWO at 2 MHz.
    constexpr uint32_t kSwoBaud = 2000000UL;
    const uint32_t swoPrescaler = (SystemCoreClock / kSwoBaud);
    TPI->SPPR = 0x00000002UL;
    TPI->ACPR = (swoPrescaler > 0U) ? (swoPrescaler - 1U) : 0U;
    TPI->FFCR = 0x00000100UL;

    // Unlock ITM
    ITM->LAR = 0xC5ACCE55;

    // Enable ITM and set the trace bus ID
    ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_TSENA_Msk | (1U << 16);

    #if DEBUG_OUTPUT == DEBUG_OUTPUT_SWO
        // Enable stimulus ports 0 (text logs) and 1 (pid tuning).
        ITM->TER |= (1UL << 0) | (1UL << 1);
    #else
        // Enable stimulus 1 (pid tuning).
        ITM->TER |= (1UL << 1);
    #endif

    #if HAVE_SCREENSHOTS
        // Enable stimulus 2 for screenshot tiles.
        ITM->TER |= (1UL << 2);
    #endif
}

static inline void debug_swd_deinit()
{
    // do not disable if SWD debugging is enabled
    #if DEBUG_OUTPUT != DEBUG_OUTPUT_SWO
        // Disable ITM
        ITM->TCR = 0;

        // Disable stimulus ports 0 and 1.
        ITM->TER &= ~((1UL << 0) | (1UL << 1));

        #if HAVE_SCREENSHOTS
            ITM->TER &= ~(1UL << 2);
        #endif

        // Lock ITM
        ITM->LAR = 0;

        // Disable trace pins and asynchronous SWO
        DBGMCU->CR &= ~DBGMCU_CR_TRACE_IOEN;
    #endif
}

bool SWO::state = false;
__attribute__((used, section(".ram_fixed"))) SWO::DataType SWO::data;

void SWO::init()
{
    if (state) {
        return;
    }
    debug_swd_init();
    SWO::state = true;
}

void SWO::deinit()
{
    debug_swd_deinit();
    SWO::state = false;
}

bool SWO::waitReadyPort(uint32_t port)
{
    constexpr uint32_t kMaxReadyPolls = 50000U;
    volatile uint32_t polls = 0;
    while (ITM->PORT[port].u32 == 0U) {
        if (++polls >= kMaxReadyPolls) {
            return false;
        }
    }
    return true;
}

size_t SWO::write(uint8_t port, const void *data, size_t size)
{
    if (!state) { // SWO not enabled
        return 0;
    }
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U || (ITM->TER & (1UL << port)) == 0U) { // ITM Port not enabled
        return 0;
    }

    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    size_t sent = 0;

    while (sent < size) {
        if (!waitReadyPort(port)) {
            break;
        }

        size_t remaining = size - sent;
        if (remaining >= 4U) {
            uint32_t word =
                (static_cast<uint32_t>(bytes[sent + 0]) << 0) |
                (static_cast<uint32_t>(bytes[sent + 1]) << 8) |
                (static_cast<uint32_t>(bytes[sent + 2]) << 16) |
                (static_cast<uint32_t>(bytes[sent + 3]) << 24);
            ITM->PORT[port].u32 = word;
            sent += 4U;
        }
        else if (remaining >= 2U) {
            uint16_t half =
                static_cast<uint16_t>(
                    (static_cast<uint16_t>(bytes[sent + 0]) << 0) |
                    (static_cast<uint16_t>(bytes[sent + 1]) << 8)
                );
            ITM->PORT[port].u16 = half;
            sent += 2U;
        }
        else {
            ITM->PORT[port].u8 = bytes[sent++];
        }
    }
    return sent;
}

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

#if DEBUG_OUTPUT == DEBUG_OUTPUT_SWO

static inline bool debug_swd_write_ITM_SendChar(uint32_t ch)
{
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
        ((ITM->TER & 1UL) != 0UL))                  /* ITM Port #0 enabled */
    {
        if (!SWO::waitReadyPort(0)) {
            return false;
        }
        ITM->PORT[0U].u8 = (uint8_t)ch;
        return true;
    }
    return false;
}

static void debug_swd_write(const char *msg)
{
    if (!msg) {
        return;
    }
    if (!SWO::state) { // SWO not enabled
        return;
    }
    while (*msg) {
        if (!debug_swd_write_ITM_SendChar(static_cast<uint32_t>(*msg++))) {
            break;
        }
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

#if DEBUG_OUTPUT == DEBUG_OUTPUT_USB

void debug_usb_printf(const char *fmt, ...)
{
    // Avoid calling CDC_Transmit_FS before the USB stack is fully configured.
    if (hUsbDeviceFS.pClassData == nullptr || hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return;
    }

    char buf[192];
    va_list args;

    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf) - 1, fmt, args);
    va_end(args);
    if (len > 0) {
        USBSerial::write(buf, strlen(buf));
    }
}

#endif

void debug_init(void)
{
    #if DEBUG_OUTPUT == DEBUG_OUTPUT_SERIAL
        //TODO replace old arduino code
        Serial.begin(115200);
    #endif
}

// === profiler ===

#if HAVE_DWT_TICK_PROFILER

TickProfiler::AverageSumType TickProfiler::slots[16];

#endif
