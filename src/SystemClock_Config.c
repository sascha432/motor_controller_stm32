/**
  Author: sascha_lammers@gmx.de

  different clock configuration for STM32F1x series MCUs
*/

#include <stm32f1xx.h>
#include <stm32f1xx_hal_rcc_ex.h>

#if defined(STM32F107xC) 

/*
    * STM32F107 clock configuration
    *
    * External crystal:
    *   HSE = 8 MHz
    *
    * PLL:
    *   8 MHz * 9 = 72 MHz SYSCLK
    *
    * Bus clocks:
    *   AHB  = 72 MHz
    *   APB2 = 72 MHz
    *   APB1 = 36 MHz (maximum allowed for STM32F1)
    *
    * SPI2 clock:
    *   SPI2 is connected to APB1
    *   APB1 = 36 MHz
    *   SPI prescaler BR=0 => divide by 2
    *   SPI2 = max. 18 MHz
    */
void SystemClock_Config(void)
{
    /*
     * Enable external high-speed oscillator (HSE).
     * The MCU starts from HSI after reset, so switch to HSE+PLL.
     */
    RCC->CR |= RCC_CR_HSEON;

    // Wait until HSE oscillator is stable
    while (!(RCC->CR & RCC_CR_HSERDY)) {
    }

    /*
     * Configure Flash access for 72 MHz operation.
     *
     * Prefetch improves performance.
     * Two wait states are required above 48 MHz on STM32F1.
     */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    /*
     * Configure AHB/APB prescalers:
     *
     * HCLK  = SYSCLK / 1  = 72 MHz
     * APB2  = HCLK   / 1  = 72 MHz
     * APB1  = HCLK   / 2  = 36 MHz
     *
     * APB1 must not exceed 36 MHz on STM32F1.
     */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    /*
     * Configure PLL:
     *
     * PLL input:
     *   HSE = 8 MHz
     *
     * PLL multiplier:
     *   x9
     *
     * Result:
     *   8 MHz * 9 = 72 MHz
     */
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);

    // Use HSE as PLL source
    RCC->CFGR |= RCC_CFGR_PLLSRC;

    // PLL multiplier x9
    RCC->CFGR |= RCC_CFGR_PLLMULL9;

    // STM32F107 OTG FS needs a 48 MHz clock source.
    // For this clock tree, select PLL/3 for USB.
    RCC->CFGR &= ~RCC_CFGR_OTGFSPRE;

    /*
     * Enable PLL and wait until it locks.
     */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
    }

    /*
     * Switch system clock source from HSI to PLL.
     *
     * After this:
     *   SYSCLK = 72 MHz
     */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait until PLL is selected as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }

    /*
     * Update CMSIS global clock variable.
     *
     * Used by:
     *   - delay functions
     *   - SysTick
     *   - peripheral clock calculations
     */
    SystemCoreClock = 72000000;

    /*
    * Reconfigure SysTick for the new clock.
    * HAL_Delay() depends on this.
    */
    SysTick_Config(SystemCoreClock / 1000);    
}

#endif
