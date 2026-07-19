/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "helpers.h"
#include "debug.h"

/**
 * @brief Convert ADC values to millivolt
 *
 * @tparam RES_TOP      Resistor value of the top resistor in the divider
 * @tparam RES_BOTTOM   Resistor value of the bottom resistor in the divider
 * @tparam VREF         Reference voltage in millivolts
 * @tparam ADC_MAX      ADC maximum value
 */
template<uint32_t RES_TOP, uint32_t RES_BOTTOM, uint32_t AVREF_MILLIVOLT = 3300, uint32_t ADC_MAX = 4095>
struct ADCVoltageConverterT
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
 * ADCCurrentConverter<10, 3300>  // 10mΩ shunt, 3.3V ADC reference
 */
template<uint32_t SHUNT_MILLIOHM, uint32_t GAIN, uint32_t AVREF_MILLIVOLT = 3300, uint32_t ADC_MAX = 4095>
struct ADCCurrentConverterT
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

using ADCVoltageConverter = ADCVoltageConverterT<100000, 9100, 3300>;
using ADCCurrentConverter = ADCCurrentConverterT<4, 20, 3300>;
using ADCTemperatureConverter = NTCConverterT<10000, 10000, 3950, 25>;

struct ADCBufferType {
    uint16_t isense;
    uint16_t vsense;
    uint16_t motor_ntc;
    uint16_t driver_ntc;

    uint32_t getInputCurrent() const {
        return ADCCurrentConverter::convert(isense);
    }

    uint32_t getInputVoltage() const {
        return ADCVoltageConverter::convert(vsense);
    }

    float getMotorTemperature() const {
        return ADCTemperatureConverter::convert(motor_ntc);
    }

    float getMosfetTemperature() const {
        return ADCTemperatureConverter::convert(driver_ntc);
    }
};

// static range checks need to be updated if constants are changed
static_assert(ADCVoltageConverter::convert(100) == 966, "static check failed");
static_assert(ADCVoltageConverter::convert(4095) == 39563, "static check failed, uint32_t overflow?");
static_assert(ADCCurrentConverter::convert(100) == 1006, "static check failed");
static_assert(ADCCurrentConverter::convert(4095) == 41205, "static check failed");

/**
 * @brief ADC class to read multiple channels using DMA
 * 
 */
struct ADC {

    static constexpr uint32_t kNumConversions = 4;      // number of channels

    /**
     * @brief Initialize the ADC and DMA for reading multiple channels
     * 
     */
    void init() 
    {
        // Enable GPIOA/GPIOC and ADC1 clocks
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_ADC1EN;

        // Configure analog mode
        GPIOA->CRL &= ~(0xF << (2 * 4));   // PA2 analog
        GPIOA->CRL &= ~(0xF << (3 * 4));   // PA3 analog
        GPIOC->CRL &= ~(0xF << (4 * 4));   // PC4 analog
        GPIOC->CRL &= ~(0xF << (5 * 4));   // PC5 analog        

        // ADC scan mode (multiple channels)
        ADC1->CR1 |= ADC_CR1_SCAN;        
        
        // set the number of conversions in the regular ADC sequence.
        ADC1->SQR1 = ((kNumConversions - 1) << 20);
        ADC1->SQR3 =
            (2  << 0)  |        // rank 1: PA2
            (3  << 5)  |        // rank 2: PA3
            (14 << 10) |        // rank 3: PC4
            (15 << 15);         // rank 4: PC5            

        // Clear and set ADC clock divider
        RCC->CFGR &= ~RCC_CFGR_ADCPRE;
        RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;   // 72MHz / 6 = 12MHz ADC clock

        // sampling time 239.5 cycles = about 20 microseconds per conversion

        // PA2, PA3 in SMPR2
        ADC1->SMPR2 |= (7 << (2 * 3));   // CH2
        ADC1->SMPR2 |= (7 << (3 * 3));   // CH3

        // PC4, PC5 in SMPR1
        ADC1->SMPR1 |= (7 << ((14 - 10) * 3)); // CH14
        ADC1->SMPR1 |= (7 << ((15 - 10) * 3)); // CH15            

        // enable DMA
        RCC->AHBENR |= RCC_AHBENR_DMA1EN;

        // configure DMA
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;

        DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;
        DMA1_Channel1->CMAR  = (uint32_t)adc_buffer;
        DMA1_Channel1->CNDTR = kNumConversions;

        DMA1_Channel1->CCR =
            DMA_CCR_MINC |       // increment memory
            DMA_CCR_PSIZE_0 |    // 16-bit peripheral
            DMA_CCR_MSIZE_0 |    // 16-bit memory
            DMA_CCR_CIRC;        // repeat forever

        DMA1_Channel1->CCR |= DMA_CCR_EN;

        // Enable ADC for calibration
        ADC1->CR2 |= ADC_CR2_ADON;
        delayMicroseconds(10);

        // Reset calibration
        ADC1->CR2 |= ADC_CR2_RSTCAL;
        while (ADC1->CR2 & ADC_CR2_RSTCAL) {
        }

        // Calibrate
        ADC1->CR2 |= ADC_CR2_CAL;
        while (ADC1->CR2 & ADC_CR2_CAL) {
        }

        // Enable ADC again after calibration
        ADC1->CR2 |= ADC_CR2_ADON;
        delayMicroseconds(10);

        ADC1->CR2 |= ADC_CR2_DMA;           // Enable DMA
        ADC1->CR2 |= ADC_CR2_CONT;          // Continuous conversion
        ADC1->CR2 |= ADC_CR2_SWSTART;       // Start
    }

    /**
     * @brief Initialize the DAC for reference voltages
     * 
     */
    void initDAC()
    {
        // Enable GPIOA clock
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
        // Enable DAC clock
        RCC->APB1ENR |= RCC_APB1ENR_DACEN;

        /*
        Configure PA4 and PA5 as analog input mode
        GPIO CRL:
        MODE[1:0] = 00
        CNF[1:0]  = 00
        */
        GPIOA->CRL &= ~(0xf << (4 * 4));   // clear PA4
        GPIOA->CRL &= ~(0xf << (5 * 4));   // clear PA5

        // Enable DAC channel 1 and channel 2
        DAC->CR |= DAC_CR_EN1 | DAC_CR_EN2;
    }

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
     * @return ADCBufferType 
     */
    ADCBufferType readAll() const {
        return *(ADCBufferType *)adc_buffer;
    }

    /**
     * @brief Get the Motor NTC value
     * 
     * @return uint16_t 
     */
    uint16_t getMotorNTCValue() const {
        return adc_buffer[2];
    }

    /**
     * @brief Get the Mosfet NTC value
     * 
     * @return uint16_t 
     */
    uint16_t getMosfetNTCValue() const {
        return adc_buffer[3];
    }

    void debugPrint() const
    {
        auto tmp = *(ADCBufferType *)adc_buffer;
        DEBUG_PRINT(
            DEBUG_DEBUG, 
            "ADC: %umA,%umV,%u.%01uC,%u.%01uC %u %u",
            tmp.getInputCurrent(), 
            tmp.getInputVoltage(), 
            (uint32_t)(tmp.getMotorTemperature()),
            (uint32_t)(tmp.getMotorTemperature() * 10) % 10,
            (uint32_t)(tmp.getMosfetTemperature()),
            (uint32_t)(tmp.getMosfetTemperature() * 10) % 10
        );
    }

    volatile uint16_t adc_buffer[kNumConversions];
};

extern ADC adc;
