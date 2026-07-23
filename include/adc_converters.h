/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <math.h>
#include "helpers.h"
#include "debug.h"

struct ADCConverter {

    /**
     * @brief Convert ADC values to millivolt
     *
     * @tparam RES_TOP      Resistor value of the top resistor in the divider
     * @tparam RES_BOTTOM   Resistor value of the bottom resistor in the divider
     * @tparam VREF         Reference voltage in millivolts
     * @tparam ADC_MAX      ADC maximum value
     */
    template<uint32_t RES_TOP, uint32_t RES_BOTTOM, uint32_t AVREF_MILLIVOLT = 3300, uint32_t ADC_MAX = 4095>
    struct VoltageConverterT
    {
        static constexpr uint32_t kAVRef = AVREF_MILLIVOLT;
        static constexpr uint32_t kResistorTop = RES_TOP;
        static constexpr uint32_t kResistorBottom = RES_BOTTOM;
        static constexpr uint32_t kDividerRatio = kAVRef * (static_cast<float>(kResistorTop + kResistorBottom) / static_cast<float>(kResistorBottom));
        static constexpr uint32_t kADCMax = ADC_MAX;

        static constexpr uint32_t convert(uint16_t adcValue) {
            return (adcValue * kDividerRatio) / kADCMax;
        }
    };

    /**
     * @brief Convert ADC values to milliamps
     *
     * @tparam SHUNT_MILLIOHM   Shunt resistor value in milliohms
     * @tparam AVREF_MILLIVOLT  ADC reference voltage in millivolts
     * @tparam ADC_MAX          ADC maximum value
     *
     * Example:
     * ADConverter::CurrentConverterT<10, 3300>  // 10mΩ shunt, 3.3V ADC reference
     */
    template<uint32_t SHUNT_MILLIOHM, uint32_t GAIN, uint32_t AVREF_MILLIVOLT = 3300, uint32_t ADC_MAX = 4095>
    struct CurrentConverterT
    {
        static constexpr uint32_t kShunt = SHUNT_MILLIOHM;
        static constexpr uint32_t kGain = GAIN;
        static constexpr uint32_t kAVRef = AVREF_MILLIVOLT;
        static constexpr uint32_t kADCMax = ADC_MAX;

        /**
         * @brief Convert ADC reading to current
         * @return Current in milliamps
         */
        static constexpr uint32_t convert(uint16_t adcValue)
        {
            return adcValue * (kAVRef * 1000U / 4095U) / (kShunt * kGain);
        }
    };

    /**
     * @brief Convert ADC readings to temperature in Celsius using NTC thermistor
     * 
     * @tparam NTC_SERIES_RESISTANCE    Series resistor value in ohms
     * @tparam NTC_NOMINAL_RESISTANCE   Nominal resistance of the NTC thermistor in ohms
     * @tparam NTC_BETA_COEFF           Beta coefficient of the NTC thermistor
     * @tparam NTC_NOMINAL_TEMP         Nominal temperature of the NTC thermistor in Celsius
     * @tparam HIGH_SIDE_NTC            True if the NTC thermistor is connected to the high side (VCC)
     * @tparam ADC_MAX                  ADC maximum value
     */
    template<uint32_t NTC_SERIES_RESISTANCE = 10000, uint32_t NTC_NOMINAL_RESISTANCE = 10000, uint32_t NTC_BETA_COEFF = 3950, uint32_t NTC_NOMINAL_TEMP = 25, bool HIGH_SIDE_NTC = false, uint32_t ADC_MAX = 4095>
    struct NTCConverterT
    {
        static constexpr uint32_t kSeriesResistance = NTC_SERIES_RESISTANCE;
        static constexpr uint32_t kNominalResistance = NTC_NOMINAL_RESISTANCE;
        static constexpr uint32_t kBetaCoefficient = NTC_BETA_COEFF;
        static constexpr uint32_t kNominalTemperature = NTC_NOMINAL_TEMP;

        static float convert(uint16_t adcValue)
        {
            if (adcValue == 0 || adcValue >= ADC_MAX) {
                return 0.0f;
            }

            float resistance;

            if constexpr (HIGH_SIDE_NTC) {
                // VCC - NTC - ADC - Rseries - GND
                resistance =
                    (static_cast<float>(ADC_MAX) /
                    static_cast<float>(adcValue) - 1.0f)
                    * static_cast<float>(kSeriesResistance);
            }
            else {
                // VCC - Rseries - ADC - NTC - GND
                resistance =
                    static_cast<float>(kSeriesResistance) *
                    static_cast<float>(adcValue) /
                    (static_cast<float>(ADC_MAX) - adcValue);
            }

            float temperature = log(resistance / static_cast<float>(kNominalResistance));
            temperature /= static_cast<float>(kBetaCoefficient);
            temperature += 1.0f / (static_cast<float>(kNominalTemperature) + 273.15f);
            temperature = 1.0f / temperature;
            return temperature - 273.15f;
        }

        static uint16_t reverse(float temperature)
        {
            // Celsius to Kelvin
            float temperatureK = temperature + 273.15f;

            // Calculate NTC resistance
            float resistance =
                kNominalResistance *
                exp(kBetaCoefficient *
                (1.0f / temperatureK -
                1.0f / (kNominalTemperature + 273.15f)));

            float adc;

            if constexpr (HIGH_SIDE_NTC) {
                // VCC - NTC - ADC - Rseries - GND
                adc =
                    static_cast<float>(ADC_MAX) *
                    kSeriesResistance /
                    (resistance + kSeriesResistance);
            }
            else {
                // VCC - Rseries - ADC - NTC - GND
                adc =
                    static_cast<float>(ADC_MAX) *
                    resistance /
                    (resistance + kSeriesResistance);
            }
            return static_cast<uint16_t>(adc);
        }    
    };

    using Voltage = VoltageConverterT<100000, 9100, 3300>;
    using Current = CurrentConverterT<4, 20, 3300>;
    using NTC = NTCConverterT<10000, 10000, 3950, 25>;
};
