/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <algorithm>
#include "adc_converters.h"
#include "pins.h"

extern "C" void DMA1_Channel1_IRQHandler();
struct PidController;

static constexpr float kADCClockMHz = 12.0f;    // ADC clock in MHz

static constexpr float kAdcSampleTimeUs(uint32_t sampleBits)
{
    switch (sampleBits & 0x07) {
        case 0: return (1.5f   + 12.5f) / kADCClockMHz;
        case 1: return (7.5f   + 12.5f) / kADCClockMHz;
        case 2: return (13.5f  + 12.5f) / kADCClockMHz;
        case 3: return (28.5f  + 12.5f) / kADCClockMHz;
        case 4: return (41.5f  + 12.5f) / kADCClockMHz;
        case 5: return (55.5f  + 12.5f) / kADCClockMHz;
        case 6: return (71.5f  + 12.5f) / kADCClockMHz;
        case 7: return (239.5f + 12.5f) / kADCClockMHz;
    }
    return 0.0f;
}

/**
 * @brief ADC class to read multiple channels using DMA
 *
 */
struct ADC
{
    // ADC sample times in ADC_CCR register
    static constexpr uint32_t kADC_SampleTime_1_5Cycles   = 0;
    static constexpr uint32_t kADC_SampleTime_7_5Cycles   = 1;
    static constexpr uint32_t kADC_SampleTime_13_5Cycles  = 2;
    static constexpr uint32_t kADC_SampleTime_28_5Cycles  = 3;
    static constexpr uint32_t kADC_SampleTime_41_5Cycles  = 4;
    static constexpr uint32_t kADC_SampleTime_55_5Cycles  = 5;
    static constexpr uint32_t kADC_SampleTime_71_5Cycles  = 6;
    static constexpr uint32_t kADC_SampleTime_239_5Cycles = 7;

    static constexpr uint32_t kNumConversions = 4;                                  // number of channels
    // WARNING/TODO: for same reason only 239.5 cycles work @12MHz, lower or mixed sample times cause invalid readings from all channels
    static constexpr uint32_t kSampleTimeCH2 = kADC_SampleTime_239_5Cycles;         // isense
    static constexpr uint32_t kSampleTimeCH3 = kADC_SampleTime_239_5Cycles;         // vsense
    static constexpr uint32_t kSampleTimeCH14 = kADC_SampleTime_239_5Cycles;        // motor ntc
    static constexpr uint32_t kSampleTimeCH15 = kADC_SampleTime_239_5Cycles;        // mosfet ntc

    static constexpr float kTotalSampleTime = kAdcSampleTimeUs(kSampleTimeCH2) + kAdcSampleTimeUs(kSampleTimeCH3) + kAdcSampleTimeUs(kSampleTimeCH14) + kAdcSampleTimeUs(kSampleTimeCH15); // sum of sample time per channel
    static constexpr float kTotalSamplesPerSecond = 1000000.0f / kTotalSampleTime;  // samples per second for all channels

    static constexpr uint32_t kISenseCountDivider = 16;                             // reduce by 6.5% to avoid overflow in rolling average
    static constexpr float kISenseRollingAverageTime = 1.0f;                        // rolling average over 1.0 second
    static constexpr uint16_t kISenseCountMax = (kTotalSamplesPerSecond * kISenseRollingAverageTime * (1.0f + (0.5f / kISenseCountDivider))) / kNumConversions; // calculate number of samples

    /**
     * @brief Construct ADC object
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
     * @brief Initialize the ADC, GPIO pins and DMA for reading multiple channels
     *
     */
    void init();

    /**
     * @brief Initialize the DAC and GPIO pins for reference voltages
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

    /**
     * @brief Interrupt Service Routine for the ADC. This function is called when a DMA transfer is complete
     *
     */
    void isr();

protected:
    friend void DMA1_Channel1_IRQHandler();
    friend PidController;

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
    uint32_t isenseSum;
    uint16_t isenseCount;
    uint32_t isenseOcpFiltered;
    uint16_t motorTemperatureFiltered;
    uint16_t mosfetTemperatureFiltered;
};

extern ADC adc;
