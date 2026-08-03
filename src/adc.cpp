/**
  Author: sascha_lammers@gmx.de
*/

#include "adc.h"
#include "pins.h"
#include "pid_controller.h"
#include "leds.h"

ADC adc;

void ADC::init()
{
    // Enable GPIOA/GPIOC and ADC1 clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_ADC1EN;

    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = digitalPinToHAL<PA2>()|digitalPinToHAL<PA3>();
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = digitalPinToHAL<PC4>()|digitalPinToHAL<PC5>();
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;   // 72MHz / 6 = 12MHz ADC clock (kADCClockMHz)

    // PA2, PA3 in SMPR2
    ADC1->SMPR2 &= ~((0x07 << (2 * 3)) | (0x07 << (3 * 3))); // clear
    ADC1->SMPR2 |= (kSampleTimeCH2 << (2 * 3));   // CH2
    ADC1->SMPR2 |= (kSampleTimeCH3 << (3 * 3));   // CH3

    // PC4, PC5 in SMPR1
    ADC1->SMPR1 &= ~((0x07 << ((14 - 10) * 3)) | (0x07 << ((15 - 10) * 3))); // clear
    ADC1->SMPR1 |= (kSampleTimeCH14 << ((14 - 10) * 3)); // CH14
    ADC1->SMPR1 |= (kSampleTimeCH15 << ((15 - 10) * 3)); // CH15

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
        DMA_CCR_CIRC |       // repeat forever
        DMA_CCR_TCIE;        // enable transfer complete interrupt

    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable DMA1 channel 1 interrupt

    DMA1_Channel1->CCR |= DMA_CCR_EN;

    // Enable ADC for calibration
    ADC1->CR2 |= ADC_CR2_ADON;
    delay_us<10>();

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
    delay_us<10>();

    ADC1->CR2 |= ADC_CR2_DMA;           // Enable DMA
    ADC1->CR2 |= ADC_CR2_CONT;          // Continuous conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;       // Start
}

void ADC::initDAC()
{
    __HAL_RCC_DAC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /** DAC GPIO Configuration
    PA4     ------> DAC_OUT1
    PA5     ------> DAC_OUT2
    */
   GPIO_InitTypeDef GPIO_InitStruct = {};
   GPIO_InitStruct.Pin = digitalPinToHAL<DRVOCP_VREF_DAC_PIN>()|digitalPinToHAL<OCP_VREF_DAC_PIN>();
   GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable DAC channel 1 and channel 2
    DAC->CR |= DAC_CR_EN1 | DAC_CR_EN2;
}

void ADC::isr()
{
    // hard fault if the have a OVP condition, mostly likely due to reverse currents while braking
    if (getVSenseValue() > pid.faults.vsenseMax) {
        if (pid.errorCode != PidController::ErrorCodeType::OVP) {
            pid.setErrorCode(PidController::ErrorCodeType::OVP);
        }
    }

    uint16_t value = getISenseValue();
    // store average for display
    isenseSum += value;
    if (++isenseCount >= kISenseCountMax) {
        // reduce by 1/16th to avoid overflow in rolling average
        isenseSum -= isenseSum / kISenseCountDivider;
        isenseCount -= isenseCount / kISenseCountDivider;
    }
    // update filtered value for fast OCP detection
    isenseOcpFiltered = filterValue<uint32_t, 2>(isenseOcpFiltered, value);

    // update filtered temperature values
    motorTemperatureFiltered = filterValue<uint16_t, 16>(motorTemperatureFiltered, getMotorNTCValue());
    mosfetTemperatureFiltered = filterValue<uint16_t, 16>(mosfetTemperatureFiltered, getMosfetNTCValue());
}
