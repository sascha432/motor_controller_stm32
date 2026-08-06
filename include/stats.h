/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "adc.h"

namespace Helpers {

    template<uint32_t UPDATE_RATE_TIME = 10000>
    struct MinMax
    {
        MinMax()
        {
            reset();
        }

        void reset()
        {
            min = INT16_MAX;
            max = INT16_MIN;
        }

        void update(int16_t value)
        {
            const uint32_t now = HAL_GetTick();
            if ((now - lastUpdate) > UPDATE_RATE_TIME) {
                reset();
            }
            if (value < min) {
                min = value;
                lastUpdate = now;
            }
            if (value > max) {
                max = value;
                lastUpdate = now;
            }
        }

        inline int16_t getMin() const
        {
            return min;
        }

        inline int16_t getMax() const
        {
            return max;
        }

    private:
        uint32_t lastUpdate;
        int16_t min;
        int16_t max;
    };

    // DECAY_TIME should be a power of 2 so the compiler can optimize multiply/divide operations into bit shifts
    template<uint32_t DECAY_TIME = 1024>
    struct Integral
    {
        static_assert(DECAY_TIME < 60000, "DECAY_TIME must be less than 60000ms");

        static constexpr uint32_t kFactor = 1024;

        Integral()
        {
            reset();
        }

        void reset()
        {
            lastUpdate = HAL_GetTick();
            integral = 0;
        }

        void update(int16_t value)
        {
            const uint32_t now = HAL_GetTick();
            uint32_t dt = now - lastUpdate;
            if (dt == 0) {
                return;
            }
            lastUpdate = now;
            if (dt > DECAY_TIME) {
                dt = DECAY_TIME;
            }
            // decay = 1 - dt / DECAY_TIME
            const uint32_t decay = kFactor - (dt * kFactor) / DECAY_TIME;
            // apply decay
            integral = (integral * decay) / kFactor;
            // add integration term: value * dt / 1000
            integral += ((int32_t)value * dt * kFactor) / 1000;
        }

        int32_t get() const
        {
            return integral / kFactor;
        }

    private:
        uint32_t lastUpdate;
        int32_t integral;   // signed because value can be negative
    };

    // FILTER_TIME should be a power of 2 so the compiler can optimize multiply/divide operations into bit shifts
    template<uint32_t FILTER_TIME = 256>
    struct LowPass
    {
        static constexpr int32_t kFactor = 1024;
        static_assert(FILTER_TIME < 120000, "FILTER_TIME must be less than 120000ms");

        LowPass()
        {
            reset();
        }

        void reset()
        {
            lastUpdate = HAL_GetTick();
            output = 0;
        }

        void update(int16_t value)
        {
            const uint32_t now = HAL_GetTick();
            uint32_t dt = now - lastUpdate;
            if (dt == 0) {
                return;
            }
            lastUpdate = now;
            if (dt > FILTER_TIME) {
                dt = FILTER_TIME;
            }
            // alpha = dt / FILTER_TIME
            const int64_t alpha = (static_cast<int64_t>(dt) * kFactor) / FILTER_TIME;
            // output += alpha * (input - output)
            const int32_t error = (static_cast<int32_t>(value) * kFactor) - output;
            output += (alpha * error) / kFactor;
        }

        int32_t get() const
        {
            return output / kFactor;
        }

    private:
        uint32_t lastUpdate;
        int32_t output;
    };

    template<uint32_t INTERVAL_MS, uint32_t FILTER_TIME_MS, uint32_t FACTOR = 1024, typename OUTPUT_TYPE = int32_t>
    struct FixedLowPass
    {
        static constexpr int32_t kFactor = FACTOR;
        static_assert(INTERVAL_MS > 0, "INTERVAL_MS must be positive");
        static_assert(FILTER_TIME_MS >= INTERVAL_MS, "FILTER_TIME_MS must be >= INTERVAL_MS");
        static constexpr int32_t kAlpha = (INTERVAL_MS * kFactor) / FILTER_TIME_MS;
        static_assert(INT16_MAX * kFactor * kAlpha < INT32_MAX, "kAlpha is too large, may cause overflow.. reduce FACTOR or increase FILTER_TIME_MS");

        FixedLowPass()
        {
            reset();
        }

        void reset()
        {
            output = 0;
        }

        void update(int16_t value)
        {
            const OUTPUT_TYPE error = static_cast<OUTPUT_TYPE>(value) * kFactor - output;
            output += (kAlpha * error) / kFactor;
        }

        OUTPUT_TYPE get() const
        {
            return output / kFactor;
        }

    // private:
        OUTPUT_TYPE output;
    };

    template <uint32_t MAX_COUNT, uint32_t DECAY_DIVIDER>
    struct Average {
        Average() : sum(0), count(0) {}

        void reset()
        {
            *this = Average();
        }

        void update(int32_t value)
        {
            sum += value;
            if (++count > MAX_COUNT) {
                sum -= sum / DECAY_DIVIDER;
                count -= count / DECAY_DIVIDER;
            }
        }

        int32_t get() const
        {
            return count ? sum / count : 0;
        }

    private:
        int32_t sum;
        uint32_t count;
    };

    struct Raw
    {
        Raw() : value(0)
        {}

        void update(int32_t value)
        {
            this->value = value;
        }

        int32_t get() const
        {
            return value;
        }

    private:
        int32_t value;
    };

};

struct Stats
{
    void update();

    // helper variables to store the converted values for display purposes
    // units are millivolts, milliamps, degrees Celsius
    uint32_t vcc;
    uint32_t current;
    int16_t motorTemp;
    int16_t mosfetTemp;
    struct {
        uint32_t vcc;
        uint32_t current;
        int16_t motorTemp;
        int16_t mosfetTemp;
    } min;
    struct {
        uint32_t vcc;
        uint32_t current;
        int16_t motorTemp;
        int16_t mosfetTemp;
    } max;

protected:
    // stats
    struct {
        Helpers::MinMax<30000> vcc;
        Helpers::MinMax<30000> current;
        Helpers::MinMax<60000> motorTemp;
        Helpers::MinMax<60000> mosfetTemp;
    } minMax;
};

extern Stats stats;
