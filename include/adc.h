/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "adc_converters.h"

/**
 * @brief ADC class to read multiple channels using DMA
 * 
 */
struct ADC {

    static constexpr uint32_t kNumConversions = 4;                              // number of channels
    /**
     * 4 channels @ 12MHz ADC clock
     * 21µs per conversion, 47619 total conversions per second
     * rolling average over ~1 second (+-3.25%)
     */
    static constexpr uint16_t kISenseCountMax = (((47619 * 1.065) / kNumConversions) / (1<<4U)) * (1<<4); // round down to multiple of 16

    /**
     * @brief struct to access the ADC buffer values
     * 
     */
    struct BufferType {
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
        isensePeak(0) 
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
     * @brief convert eeprom values (current * 500) to DAC values based on the 3.3V vref, 4mΩ shunt and 20x gain
     * 
     * @param limit 
     * @return ** constexpr uint16_t 
     */
    static constexpr uint16_t _currentLimitValueToDAC(uint16_t limit)
    {
        uint32_t dac = ((uint32_t)limit * 1985) / 10000 + 62;
        return std::min<uint32_t>(dac, 4095);
    }

    /**
     * @brief Set DAC voltage for the DRV8701 reference voltage
     * 
     * @param value 
     */
    void setMotorCurrentLimit(uint16_t value)
    {
        DAC->DHR12R1 = _currentLimitValueToDAC(value) & 0xfff;
    }

    /**
     * @brief Set DAC voltage for the INA381 comparator reference voltage
     * 
     * @param value 
     */
    void setInputCurrentLimit(uint16_t value)
    {
        DAC->DHR12R2 = _currentLimitValueToDAC(value) & 0xfff;
    }

    /**
     * @brief Read single ADC value from the buffer
     * 
     * @param index 
     * @return uint16_t 
     */
    uint16_t read(uint8_t index) const {
        return adc_buffer[index];
    }

    /**
     * @brief Copy ADC buffer into struct
     * 
     * @return BufferType 
     */
    BufferType readAll() const {
        return *(BufferType *)adc_buffer;
    }

    /**
     * @brief Get the Input Current value in ADC units
     * 
     */
    inline uint16_t getISenseValue() const {
        return adc_buffer[0];
    }

    /**
     * @brief Get the Input Current peak value in ADC units. Used for OCP protection and fault handling.
     * 
     * @return uint16_t 
     */
    inline uint16_t getISensePeakValue() const {
        return isensePeak;
    }

    /**
     * @brief Get the Input Current average value in ADC units. Used to display stable current values.
     * 
     * @return uint16_t 
     */
    inline uint16_t getISenseAverageValue() const {
        return isenseCount ? (isenseSum / isenseCount) : 0;
    }

    /**
     * @brief Get the Input Voltage value in ADC units
     * 
     */
    inline uint16_t getVSenseValue() const {
        return adc_buffer[1];
    }

    /**
     * @brief Get the Motor NTC value in ADC units
     * 
     * @return uint16_t 
     */
    inline uint16_t getMotorNTCValue() const {
        return adc_buffer[2];
    }

    /**
     * @brief Get the Mosfet NTC value in ADC units
     * 
     * @return uint16_t 
     */
    inline uint16_t getMosfetNTCValue() const {
        return adc_buffer[3];
    }

    volatile uint16_t adc_buffer[kNumConversions];
    volatile uint32_t isenseSum;
    volatile uint16_t isenseCount;
    volatile uint16_t isensePeak;
};

extern ADC adc;
