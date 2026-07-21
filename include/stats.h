/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "adc.h"

namespace Helpers {

    // DECAY_TIME should be a power of 2 so the compiler can optimize multiply/divide operations into bit shifts
    template<uint32_t UPDATE_RATE_TIME = 32768>
    struct MinMax {
        uint32_t lastUpdate;
        int16_t min;
        int16_t max;

        MinMax()
        {
            reset();
        }

        void reset()
        {
            lastUpdate = HAL_GetTick();
            min = INT16_MAX;
            max = INT16_MIN;
        }

        void update(int16_t value) 
        {
            if ((HAL_GetTick() - lastUpdate) > UPDATE_RATE_TIME) {
                reset();
            }
            if (value < min) {
                min = value;
            }
            if (value > max) {
                max = value;
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
    };

    // DECAY_TIME should be a power of 2 so the compiler can optimize multiply/divide operations into bit shifts
    template<uint32_t DECAY_TIME = 1024>
    struct Integral {
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
            uint32_t dt = HAL_GetTick() - lastUpdate;
            if (dt == 0)
                return;

            lastUpdate = HAL_GetTick();

            if (dt > DECAY_TIME)
                dt = DECAY_TIME;

            // decay = 1 - dt / DECAY_TIME
            uint32_t decay = kFactor - (dt * kFactor) / DECAY_TIME;

            // apply decay
            integral = (integral * decay) / kFactor;

            // add integration term: value * dt / 1000
            integral += ((int32_t)value * dt * kFactor) / 1000;
        }

        int32_t get() const
        {
            return integral / kFactor;
        }

        uint32_t lastUpdate;
        int32_t integral;   // signed because value can be negative
    };

    // FILTER_TIME should be a power of 2 so the compiler can optimize multiply/divide operations into bit shifts
    template<uint32_t FILTER_TIME = 256>
    class LowPass
    {
        static constexpr int32_t kFactor = 1024;
        static_assert(FILTER_TIME < 120000, "FILTER_TIME must be less than 120000ms");

        LowPass()
        {
            reset();
        }

        void reset(int32_t value = 0)
        {
            lastUpdate = HAL_GetTick();
            output = value * kFactor;
        }

        void update(int16_t value)
        {
            uint32_t now = HAL_GetTick();
            uint32_t dt = now - lastUpdate;

            if (dt == 0)
                return;

            lastUpdate = now;

            if (dt > FILTER_TIME)
                dt = FILTER_TIME;

            // alpha = dt / FILTER_TIME
            int32_t alpha = ((int64_t)dt * kFactor) / FILTER_TIME;

            // output += alpha * (input - output)
            int32_t error = ((int32_t)value * kFactor) - output;

            output += ((int64_t)alpha * error) / kFactor;
        }

        int32_t get() const
        {
            return output / kFactor;
        }

        uint32_t lastUpdate;
        int32_t output;   // fixed point: real value * kFactor
    };

    struct Raw {

        Raw() : value(0) 
        {}

        void update(int16_t value) 
        {
            this->value = value;
        }

        int32_t get() const 
        {
            return value;
        }

        int16_t value;
    };

};

struct Stats {

    void update();

    // stats
    struct {
        Helpers::MinMax<4096> vcc;
        Helpers::MinMax<4096> current;
        Helpers::MinMax<32768> motorTemp;
        Helpers::MinMax<32768> mosfetTemp;
    } minMax;

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
};    

extern Stats stats;
