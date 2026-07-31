/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <algorithm>
#include "adc_converters.h"
#include "pins.h"

extern "C" void DMA1_Channel1_IRQHandler();

/**
 * @brief ADC class to read multiple channels using DMA
 *
 */
struct ADC {

    static constexpr uint32_t kNumConversions = 4;                              // number of channels
    /**
     * 4 channels @ 12MHz ADC clock
     * 21µs per conversion, 47619 total conversions per second
     * rolling average over ~0.5 second (+-3.25%)
     */
    static constexpr uint16_t kISenseCountMax = (((47619 * 1.065) / kNumConversions) / (1<<4U)) * (1<<4) / 2; // round down to multiple of 16

    /**
     * @brief struct to access the ADC buffer values
     *
     */
    struct BufferType
    {
        uint16_t isense;
        uint16_t vsense;
        uint16_t motor_ntc;
        uint16_t driver_ntc;
    };

    /**
     * @brief Construct a new ADC object
     *
     */
    ADC() :
        isenseSum(0),
        isenseCount(0),
        isenseOcpFiltered(0),
        motorTemperatureFiltered(0),
        mosfetTemperatureFiltered(0)
    {
    }

    /**
     * @brief Initialize the ADC and DMA for reading multiple channels
     *
     */
    void init();

    /**
     * @brief Initialize the DAC for reference voltages
     *
     */
    void initDAC();

    /**
     * @brief Set DAC voltage for the DRV8701 reference voltage
     *
     * @param value
     */
    inline void setMotorCurrentLimit(uint16_t value)
    {
        DAC_SET_MOTOR_CURRENT(ADCConverter::Current::reverse(value));
    }

    /**
     * @brief Set DAC voltage for the INA381 comparator reference voltage
     *
     * @param value
     */
    inline void setInputCurrentLimit(uint16_t value)
    {
        DAC_SET_INPUT_CURRENT(ADCConverter::Current::reverse(value));
    }

    /**
     * @brief Copy ADC buffer into struct
     *
     * @return BufferType
     */
    inline BufferType readAll() const
    {
        return *(BufferType *)adc_buffer;
    }

    /**
     * @brief Get the Input Current average value. Used to display stable current values.
     *
     * @return uint16_t Average current in ADC units
     */
    inline uint16_t getISenseAverageValue() const
    {
        return isenseCount ? (isenseSum / isenseCount) : 0;
    }

    /**
     * @brief Get the Input Current average value for OCP
     *
     * @return uint16_t Filtered current in ADC units
     */
    inline uint16_t getISenseOcpFilteredValue() const
    {
        return isenseOcpFiltered;
    }

    /**
     * @brief Get the Input Voltage value
     *
     * @return uint16_t Voltage in ADC units
     */
    inline uint16_t getVSenseValue() const
    {
        return adc_buffer[1];
    }

    /**
     * @brief Get the Motor Temperature Filtered
     *
     * @return uint16_t Temperature in ADC units
     */
    inline uint16_t getMotorTemperatureFiltered() const
    {
        return motorTemperatureFiltered;
    }

    /**
     * @brief Get the Mosfet Temperature Filtered
     *
     * @return uint16_t Temperature in ADC units
     */
    inline uint16_t getMosfetTemperatureFiltered() const
    {
        return mosfetTemperatureFiltered;
    }

protected:
    friend void DMA1_Channel1_IRQHandler();

    /**
     * @brief Get the Input Current value
     *
     * @return uint16_t Current in ADC units
     */
    inline uint16_t getISenseValue() const
    {
        return adc_buffer[0];
    }

    /**
     * @brief Get the Motor NTC value
     *
     * @return uint16_t Temperature in ADC units
     */
    inline uint16_t getMotorNTCValue() const
    {
        return adc_buffer[2];
    }

        /**
     * @brief Get the Mosfet NTC value
     *
     * @return uint16_t Temperature in ADC units
     */
    inline uint16_t getMosfetNTCValue() const
    {
        return adc_buffer[3];
    }

protected:
    volatile uint16_t adc_buffer[kNumConversions];
    volatile uint32_t isenseSum;
    volatile uint16_t isenseCount;
    volatile uint32_t isenseOcpFiltered;
    volatile uint16_t motorTemperatureFiltered;
    volatile uint16_t mosfetTemperatureFiltered;
};

extern ADC adc;
