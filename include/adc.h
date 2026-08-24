/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <algorithm>
#include "adc_converters.h"
#include "pins.h"

extern ADC_HandleTypeDef hadc1;
struct PidController;

static constexpr float kADCClockMHz = 12.0f;    // ADC clock in MHz

static constexpr float kAdcSampleTimeUs(uint32_t sampleBits)
{
    switch (sampleBits & 0x07) {
        case ADC_SAMPLETIME_1CYCLE_5: return (1.5f + 12.5f) / kADCClockMHz;
        case ADC_SAMPLETIME_7CYCLES_5: return (7.5f + 12.5f) / kADCClockMHz;
        case ADC_SAMPLETIME_13CYCLES_5: return (13.5f + 12.5f) / kADCClockMHz;
        case ADC_SAMPLETIME_28CYCLES_5: return (28.5f + 12.5f) / kADCClockMHz;
        case ADC_SAMPLETIME_41CYCLES_5: return (41.5f + 12.5f) / kADCClockMHz;
        case ADC_SAMPLETIME_55CYCLES_5: return (55.5f + 12.5f) / kADCClockMHz;
        case ADC_SAMPLETIME_71CYCLES_5: return (71.5f + 12.5f) / kADCClockMHz;
        case ADC_SAMPLETIME_239CYCLES_5: return (239.5f + 12.5f) / kADCClockMHz;
    }
    return 0.0f;
}

/**
 * @brief ADC class to read multiple channels using DMA
 *
 */
struct ADC
{
    static constexpr uint32_t kNumConversions = 4;                                  // number of channels
    static constexpr uint32_t kSampleTimeCH2 = ADC_SAMPLETIME_239CYCLES_5;          // isense
    static constexpr uint32_t kSampleTimeCH3 = ADC_SAMPLETIME_13CYCLES_5;           // vsense
    static constexpr uint32_t kSampleTimeCH14 = ADC_SAMPLETIME_71CYCLES_5;          // motor ntc
    static constexpr uint32_t kSampleTimeCH15 = ADC_SAMPLETIME_71CYCLES_5;          // mosfet ntc

    static constexpr uint32_t kInjectedTriggerOffsetTicks = 0;                      // offset added to the active PWM compare to delay the injected sample (72MHz timer ticks, ~14ns each)

    static constexpr uint32_t kISenseCountDecayDivider = 16;                        // reduce by 1/16 to avoid overflow in rolling average
    static constexpr float kISenseRollingAverageTime = 1.0f;                        // rolling average over 1000ms
    static constexpr uint16_t kISenseCountMax = ((1000.0 / PID_INTERVAL) * kISenseRollingAverageTime * (1.0f + (0.5f / kISenseCountDecayDivider))); // calculate number of samples

    /**
     * @brief Construct ADC object
     *
     */
    ADC() :
        isenseSum(0),
        isenseCount(0),
        isenseFiltered(0),
        isenseMax(0),
        motorTemperatureFiltered(0),
        mosfetTemperatureFiltered(0),
        dmaTransferComplete(false),
        isenseSmoothing(0)
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
     * @brief Interrupt Service Routine for the ADC. This function is called when a DMA transfer is complete
     *
     */
    void isr();

    /**
     * @brief Interrupt Service Routine for the injected ADC group
     *
     */
    void isrInjected();

protected:
    friend PidController;

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

public:
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
     * @brief Get the Input Voltage value
     *
     * @return uint16_t Voltage in ADC units
     */
    inline uint16_t getVSenseValue() const
    {
        return adc_buffer[1];
    }

    /**
     * @brief Get the and clear max. current value
     *
     * @return uint16_t
     */
    inline uint16_t getAndClearISenseMaxValue()
    {
        const uint16_t value = isenseMax;
        isenseMax = 0;
        return value;
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
     * @brief Start ADC DMA transfer if the DMA is ready
     *
     */
    inline void startDMAIfReady()
    {
        if (isDMAReady()) {
            startDMA();
        }
    }

protected:
    // /**
    //  * @brief Update the TIM1_CH4 compare value so the next injected sample starts at the
    //  *        falling edge (end of duty) of the active PWM channel
    //  */
    // inline void updateInjectedTriggerPoint()
    // {
    //     // sample at the end of the active duty cycle (PWM goes low) plus an optional offset.
    //     // clamp to [1, ARR]: CCR4 == 0 would keep OC4REF flat (PWM mode 2 -> always high) and
    //     // never generate a trigger edge, which would stall the injected ISR
    //     const uint16_t ccr1 = PID_READ_MOTOR_PWM_DRV_IN1();
    //     const uint16_t ccr2 = PID_READ_MOTOR_PWM_DRV_IN2();
    //     updateInjectedTriggerPoint(std::max<uint16_t>(ccr1, ccr2));
    // }

    /**
     * @brief Update the TIM1_CH4 compare value to enable the injection
     *
     */
    inline void updateInjectedTriggerPoint()
    {
        PID_MOTOR_PWM_TIMER->CCR4 = 4 * 72; // start 4us delayed
    }

    /**
     * @brief Disable the injected ADC trigger by flattening OC4REF (CCR4 = 0)
     *
     */
    inline void stopInjectedTrigger()
    {
        PID_MOTOR_PWM_TIMER->CCR4 = 0;
        isenseSum = 0;
        isenseCount = 0;
    }

    void initInjection();

    /**
     * @brief Check if the DMA is ready for a new transfer
     *
     * @return true
     * @return false
     */
    inline bool isDMAReady() const
    {
        return dmaTransferComplete == true;
    }

    /**
     * @brief Start ADC DMA transfer
     *
     */
    inline void startDMA()
    {
        ADC1->CR2 &= ~ADC_CR2_ADON;                 // disable ADC, ADC_CR2_SWSTART is ignored while ADC is running a conversion
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;          // disable DMA
        dmaTransferComplete = false;                // mark DMA as busy
        DMA1_Channel1->CNDTR = kNumConversions;     // reload transfer count
        DMA1_Channel1->CCR |= DMA_CCR_EN;           // enable DMA
        ADC1->CR2 |= ADC_CR2_ADON;                  // enable ADC
        ADC1->CR2 |= ADC_CR2_SWSTART;               // trigger regular conversion group
    }

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
    volatile uint16_t isenseFiltered;
    volatile uint16_t isenseMax;
    volatile uint16_t motorTemperatureFiltered;
    volatile uint16_t mosfetTemperatureFiltered;
    volatile bool dmaTransferComplete;
    uint16_t isenseSmoothing;
};

extern ADC adc;
