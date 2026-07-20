/**
  Author: sascha_lammers@gmx.de
*/

#include "adc.h"

ADC adc;

void ADC::init()
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
        DMA_CCR_CIRC |       // repeat forever
        DMA_CCR_TCIE;        // enable transfer complete interrupt

    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable DMA1 channel 1 interrupt

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

void ADC::initDAC()
{
    // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    // Enable DAC clock
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    // Configure PA4 and PA5 as analog input mode GPIO CRL: MODE[1:0] = 00 CNF[1:0] = 00
    GPIOA->CRL &= ~(0xf << (4 * 4));   // clear PA4
    GPIOA->CRL &= ~(0xf << (5 * 4));   // clear PA5

    // Enable DAC channel 1 and channel 2
    DAC->CR |= DAC_CR_EN1 | DAC_CR_EN2;
}

// DMA1_Channel1_IRQHandler is the interrupt handler for the DMA1 Channel 1. It is called when a DMA transfer is complete
extern "C" void DMA1_Channel1_IRQHandler()
{
    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
        // clear transfer complete
        DMA1->IFCR = DMA_IFCR_CTCIF1;  

        auto value = adc.getISenseValue();
        // store peak for OCP and fault handling
        if (value > adc.isensePeak) {
            adc.isensePeak = value;
        }
        // store average for display
        adc.isenseSum += value;
        if (++adc.isenseCount >= ADC::kISenseCountMax) {
            // reduce by 6.5% to avoid overflow in rolling average
            adc.isenseSum -= adc.isenseSum >> 4;    
            adc.isenseCount -= adc.isenseCount >> 4;
        }
    }
}
