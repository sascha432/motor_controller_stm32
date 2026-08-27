/**
  Author: sascha_lammers@gmx.de
*/

#include "adc.h"
#include "pid_controller.h"
#include "leds.h"

ADC adc;
ADC_HandleTypeDef hadc1;

#ifndef ADC_CALIBRATION_TIMEOUT
    #define ADC_CALIBRATION_TIMEOUT 10
#endif

void ADC::init()
{
    // Enable GPIOA/GPIOC
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = ISENSE_Pin | VSENSE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(ISENSE_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = EXT_NTC_Pin | DRV_TEMP_Pin;
    HAL_GPIO_Init(EXT_NTC_GPIO_Port, &GPIO_InitStruct);

    // Enable ADC1 clock
    __HAL_RCC_ADC1_CLK_ENABLE();

    // ADC scan mode (multiple channels)
    ADC1->CR1 |= ADC_CR1_SCAN;

    // set the number of conversions in the regular ADC sequence.
    ADC1->SQR1 = ((kNumConversions - 1) << ADC_SQR1_L_Pos);
    ADC1->SQR3 =
        (2U  << ADC_SQR3_SQ1_Pos) |         // rank 1: PA2
        (3U  << ADC_SQR3_SQ2_Pos) |         // rank 2: PA3
        (14U << ADC_SQR3_SQ3_Pos) |         // rank 3: PC4
        (15U << ADC_SQR3_SQ4_Pos);          // rank 4: PC5

    // Clear and set ADC clock divider
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;
    RCC->CFGR |= kADCCFGRClockDiv;          // F_CPU / kADCCFGRClockDiv = kADCClockMHz ADC clock

    // Sample times for PA2, PA3 in SMPR2
    ADC1->SMPR2 &= ~(ADC_SMPR2_SMP2_Msk|ADC_SMPR2_SMP3_Msk);
    ADC1->SMPR2 |= (kSampleTimeCH2 << ADC_SMPR2_SMP2_Pos)       // CH2
                |  (kSampleTimeCH3 << ADC_SMPR2_SMP3_Pos);      // CH3
    // Sample times for PC4, PC5 in SMPR1
    ADC1->SMPR1 &= ~(ADC_SMPR1_SMP14_Msk|ADC_SMPR1_SMP15_Msk);
    ADC1->SMPR1 |= (kSampleTimeCH14 << ADC_SMPR1_SMP14_Pos)     // CH14
                |  (kSampleTimeCH15 << ADC_SMPR1_SMP15_Pos);    // CH15

    // Enable DMA
    __HAL_RCC_DMA1_CLK_ENABLE();

    // Configure DMA
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR  = (uint32_t)adc_buffer;
    DMA1_Channel1->CNDTR = kNumConversions;

    DMA1_Channel1->CCR =
        DMA_CCR_MINC |       // increment memory
        DMA_CCR_PSIZE_0 |    // 16-bit peripheral
        DMA_CCR_MSIZE_0 |    // 16-bit memory
        DMA_CCR_TCIE;        // enable transfer complete interrupt

    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable DMA1 channel 1 interrupt

    // Injected group: PA2 (IN2) current + PA3 (IN3) voltage, triggered by TIM1_CH4
    // compare event (fires at the PWM falling edge / end of duty cycle)
    ADC1->CR2 |= ADC_CR2_JEXTTRIG | ADC_CR2_JEXTSEL_0;              // TIM1_CH4 trigger, rising edge
    ADC1->JSQR = (2U << ADC_JSQR_JSQ3_Pos) |                        // rank 1: IN2 (PA2) current
                 (3U << ADC_JSQR_JSQ4_Pos) |                        // rank 2: IN3 (PA3) voltage
                 ADC_JSQR_JL_0;                                     // JL = 1 -> 2 injected ranks
    ADC1->CR1 |= ADC_CR1_JEOCIE;                                    // enable injected end of sequence interrupt
    NVIC_EnableIRQ(ADC1_2_IRQn);                                    // enable ADC1/2 interrupt

    // Enable ADC for calibration
    ADC1->CR2 |= ADC_CR2_ADON;
    delay_us(ADC_CALIBRATION_TIMEOUT);

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
    delay_us(ADC_CALIBRATION_TIMEOUT);

    ADC1->CR2 &= ~(ADC_CR2_CONT|ADC_CR2_EXTSEL);        // disable continuous conversion and external trigger
    ADC1->CR2 |= ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG;      // enable external trigger (software start)
    ADC1->CR2 |= ADC_CR2_DMA;                           // Enable DMA

    // start first transfer, the injected group will be triggered by the PWM timer while the motor is running
    dmaTransferComplete = false;
    startDMA();
}

void ADC::isr()
{
    // check ovp condition
    pid.ovp_check(getVSenseValue());

    // update filtered temperature values
    motorTemperatureFiltered = filterValue<uint16_t, 16>(motorTemperatureFiltered, getMotorNTCValue());
    mosfetTemperatureFiltered = filterValue<uint16_t, 16>(mosfetTemperatureFiltered, getMosfetNTCValue());

    dmaTransferComplete = true;
}

void ADC::isrInjected()
{
    // rank 1: PA2/IN2 current, rank 2: PA3/IN3 voltage (JDR1 is the first injected conversion)
    const uint16_t iSense = ADC1->JDR1;
    const uint16_t vSense = ADC1->JDR2;

    // Rev1.0 lacks the proper RC filter for the current, so we do it in software (4 and 8 seems to work well, 2 is not enough)
    isenseFiltered = filterValue<uint16_t, 4>(isenseFiltered, iSense);

    // max. value
    if (isenseFiltered > isenseMaxFiltered) {
        isenseMaxFiltered = isenseFiltered;
    }

    // average current for display purposes
    isenseSum += isenseFiltered;
    if (++isenseCount >= isenseSmoothing) {
        isenseSum -= isenseSum / kISenseCountDecayDivider;
        isenseCount -= isenseCount / kISenseCountDecayDivider;
    }

    // check ovp condition
    pid.ovp_check(vSense);

    // update max. voltage
    stats.minMax.vcc.update(vSense);

    // handle OCP detection
    if (isenseFiltered > pid.faults.isenseMax) {
        pid.ocp_start();
    }
    else {
        pid.ocp_stop();
    }
}
