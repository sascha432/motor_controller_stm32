/**
  Author: sascha_lammers@gmx.de
*/

#include "adc.h"
#include "pins.h"
#include "pid_controller.h"
#include "leds.h"

ADC adc;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

void ADC::init()
{
    // Enable GPIOA/GPIOC
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = digitalPinToHAL<PA2>()|digitalPinToHAL<PA3>();
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = digitalPinToHAL<PC4>()|digitalPinToHAL<PC5>();
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Enable ADC1 clock
    __HAL_RCC_ADC1_CLK_ENABLE();

    // Configure ADC
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = kNumConversions;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {};
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = kSampleTimeCH2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = kSampleTimeCH3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    sConfig.SamplingTime = kSampleTimeCH14;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank = ADC_REGULAR_RANK_4;
    sConfig.SamplingTime = kSampleTimeCH15;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // calibrate ADC
    HAL_ADCEx_Calibration_Start(&hadc1);

    // Enable DMA
    __HAL_RCC_DMA1_CLK_ENABLE();

    // Configure DMA for ADC1
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        Error_Handler();
    }

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable DMA1 channel 1 interrupt

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, kNumConversions) != HAL_OK) {
        Error_Handler();
    }
}

void ADC::initDAC()
{
    // Initialize GPIOA for DAC output pins
    __HAL_RCC_GPIOA_CLK_ENABLE();

   GPIO_InitTypeDef GPIO_InitStruct = {};
   GPIO_InitStruct.Pin = digitalPinToHAL<DRVOCP_VREF_DAC_PIN>()|digitalPinToHAL<OCP_VREF_DAC_PIN>();
   GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable DAC channel #1 (PA4/DRVOCP_VREF_DAC_PIN) and channel #2 (PA5/OCP_VREF_DAC_PIN)
    __HAL_RCC_DAC_CLK_ENABLE();
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

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, kNumConversions);
}
