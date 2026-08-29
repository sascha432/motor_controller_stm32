/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#if HAVE_DWT_TICK_PROFILER

#include <stdio.h>
#include <stm32f1xx.h>

// === profiler ===

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
        ::snprintf(buf, size, "%u\n", static_cast<unsigned>(getTicks(slot)));
    }

    static AverageSumType slots[16];
};

#endif
