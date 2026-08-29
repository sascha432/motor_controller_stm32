/**
  Author: sascha_lammers@gmx.de
*/

#include "debug.h"
#include "serial.h"

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
        ITM->TER |= (1UL << SWO::kITMPort) | (1UL << SWO::kPidPort);
    #else
        // Enable stimulus 1 (pid tuning).
        ITM->TER |= (1UL << SWO::kPidPort);
    #endif

    #if HAVE_SCREENSHOTS
        // Enable stimulus 2 for screenshot tiles.
        ITM->TER |= (1UL << SWO::kScreenshotPort);
    #endif
}

static inline void debug_swd_deinit()
{
    // do not disable if SWD debugging is enabled
    #if DEBUG_OUTPUT != DEBUG_OUTPUT_SWO
        // Disable ITM
        ITM->TCR = 0;

        // Disable stimulus ports 0 and 1.
        ITM->TER &= ~((1UL << SWO::kITMPort) | (1UL << SWO::kPidPort));

        #if HAVE_SCREENSHOTS
            ITM->TER &= ~(1UL << SWO::kScreenshotPort);
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

size_t SWO::write(uint8_t port, const void *data, size_t size)
{
    if (!isPortWritable(port)) {
        return 0;
    }
    return writeFast(port, data, size);
}

#if DEBUG_OUTPUT == DEBUG_OUTPUT_SWO

static void debug_swd_write(const char *msg)
{
    if (!msg) {
        return;
    }
    if (!SWO::isPortWritable(SWO::kITMPort)) {
        return;
    }
    while (*msg) {
        if (!SWO::waitReadyPort(SWO::kITMPort)) {
            break;
        }
        ITM->PORT[SWO::kITMPort].u8 = static_cast<uint8_t>(*msg++);
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
    if (__get_PRIMASK() || __get_IPSR()) {
        // do not send any debug messages over USB with interrupts disabled or inside ISRs
        return;
    }
    if (!Serial::isConnected()) {
        return;
    }

    char buf[192];
    va_list args;

    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf) - 1, fmt, args);
    va_end(args);
    if (len > 0) {
        Serial::write(buf, strlen(buf));
    }
}

#endif

// === profiler ===

#if HAVE_DWT_TICK_PROFILER

TickProfiler::AverageSumType TickProfiler::slots[16];

#endif
