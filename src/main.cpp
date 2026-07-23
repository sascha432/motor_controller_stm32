/**
  Author: sascha_lammers@gmx.de
*/

#include "i2c.h"
#include "controls.h"
#include "mt6701_encoder.h"
#include "pid_controller.h"
#include "adc.h"
#include "leds.h"
#include "tft_driver.h"
#include "ui.h"
#include "menu.h"
#include "eeprom.h"
#include "stats.h"
#include "debug.h"

static void button_isr() 
{
    #if defined(STM32F107xC)
        uint32_t idrD = ((GPIO_TypeDef *)GPIOD_BASE)->IDR;
        knobButton.isr(idrD);
        backButton.isr(idrD);
        startButton.isr(idrD);
    #elif defined(STM32F103xC)
        uint32_t idrB = ((GPIO_TypeDef *)GPIOB_BASE)->IDR;
        startButton.isr(idrB);
        uint32_t idrA = ((GPIO_TypeDef *)GPIOA_BASE)->IDR;
        knobButton.isr(idrA);
        backButton.isr(idrA);
        TODO pin macros need to be  chagned and isr too
    #else
        #error missing ISR implementation for this MCU
    #endif
}

static void pid_timer_isr()
{
    pid.isr();
}

static void pid_fault_isr()
{
#if ARDUINO
    pid.fault_isr();
#endif
}

#ifndef ARDUINO

void EXTI_init();

TIM_HandleTypeDef tim6;
TIM_HandleTypeDef tim7;

#endif

void setup()
{
    debug_init();

    // Initialize and read EEPROM on I2C1 on PB8/9
    eeprom.init();
    eeprom.read();

    // LEDs
    LEDs::init();

    // motor encoder
    motorEncoder.init();
    if (PidController::kProgramPPR) {
        motorEncoder.programPPR(i2c, PidController::kPPR);
    }

    // buttons
    knobButton.init(button_isr);
    backButton.init(button_isr);
    startButton.init(button_isr);

    // rotary encoder knob
    knob.init();
    knob.enable();

    // ADC with DMA
    adc.init();
    // DAC
    adc.initDAC();
    // PID controller
    pid.init(pid_timer_isr, pid_fault_isr);

    #ifndef ARDUINO
    // Start TIM6 for periodic interrupt
    HAL_TIM_Base_Start_IT(&tim6);

    // Initialize external interrupts for buttons and fault handling
    EXTI_init();
    #endif

    // Initialize display driver
    tft_driver_init();
    tft_clear_display();

    // Initialize LVGL and register flush callback
    lv_init();
    tft_driver_lvgl_init();

#if 0
    // color flashing test loop
    tft_backlight_pwm_set(100);
    uint16_t colors[] = {0xF800, 0x07E0, 0x001F, 0xFFFF};// Red, Green, Blue, White
    int c = 0;

    for(;;) {
        uint32_t start = HAL_GetTick();
        tft_clear_display(colors[c++%4]);
        uint32_t dur = HAL_GetTick() - start;
        DEBUG_PRINT(DEBUG_DEBUG, "Clear display took %lu ms", dur);
        HAL_Delay(250);
    }
#endif

    // Show welcome screen and load main menu
    menu.showWelcomeScreen();
    // Apply settings after welcome screen since it turns the backlight on
    menu.applyEEPROMSettings();

    menu.loadStartScreen();
}

void loop()
{
    // handle buttons
    if (knobButton.isPressed()) {
        menu.handleButtonPress();
    }
    if (backButton.isPressed()) {
        menu.handleBackButtonPress();
    }
    if (startButton.isPressed()) {
        menu.handleStartButtonPress();
    }

    if (pid.faults.count) {
        static uint32_t lastFaultTime = 0;
        if (HAL_GetTick() - lastFaultTime >= 500) {
            lastFaultTime = HAL_GetTick();
            pid.faults.reset();
            LEDs::offLED1and2();
        }
        else {
            if (pid.faults.ocpFault) {
                LEDs::onLED1();
            }
            if (pid.faults.snsoutFault) {
                LEDs::onLED2();
            }
        }
    }

    // handle ui updates and rotary encoder
    static uint32_t lastLvHandler = 0;
    if (HAL_GetTick() - lastLvHandler >= 5) {
        // handle rotary encoder
        int32_t newPosition;
        int32_t delta = knob.getDeltaPosition();
        if (delta) {
            newPosition = menu.updateRotaryValue(delta);
            DEBUG_PRINT(DEBUG_DEBUG, "menu=%d delta=%d", newPosition, delta);
        }
        // handle LVGL updates
        auto &screenFlow = menu.getScreenFlow();
        switch(screenFlow->getId()) {
            case Screen::Type::DASHBOARD:
            case Screen::Type::DIAGNOSTICS:
                stats.update();
                screenFlow->update();
                break;
            default:
                break;
        }
        // check NTC sensors, not time critical and a couple times per seconds is enough
        if (adc.getMotorNTCValue() < eeprom.getMotorTemperatureLimitADC()) {
            if (pid.running) {
                pid.setErrorCode(PidController::ErrorCodeType::MOTOR_OVER_TEMPERATURE);
            }
        }
        if (adc.getMosfetNTCValue() < eeprom.getMosfetTemperatureLimitADC()) {
            if (pid.running) {
                pid.setErrorCode(PidController::ErrorCodeType::MOSFET_OVER_TEMPERATURE);
            }
        }
        
        // update UI
        lv_timer_handler();
        lastLvHandler = HAL_GetTick();
    }

    if (false) {
        static uint32_t lastTime37 = 0;
        if (HAL_GetTick() - lastTime37 >= 100) {
            lastTime37 = HAL_GetTick();
            // extern volatile uint32_t rpm_counter;
            // DEBUG_PRINT(DEBUG_DEBUG, "RPM_COUNTER=%u", rpm_counter);
            // DEBUG_PRINT(DEBUG_DEBUG, "TIM5=%u", TIM5->CNT);
        }
    }

    if (false) { // print faults
        static uint32_t lastTime3 = 0;
        if (HAL_GetTick() - lastTime3 >= 1000) {
            lastTime3 = HAL_GetTick();
            static uint32_t lastCounter = 0;
            if (pid.faults.count != lastCounter) {
                lastCounter = pid.faults.count;
                pid.debugPrintFaults();
            }
        }
    }
}

#ifndef ARDUINO

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

extern "C" void TIM6_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim6);
}

extern "C" void EXTI9_5_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & ((1 << 8) | (1 << 9));
    EXTI->PR = pending; // clear flags
    if (pending & (1 << 8)) {
        // KNOB_BUTTON_PIN/PD8 changed
        knobButton.isr(GPIOD->IDR);
    }
    if (pending & (1 << 9)) {
        // BACK_BUTTON_PIN/PD9 changed
        backButton.isr(GPIOD->IDR);
    }
}

extern "C" void EXTI15_10_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & ((1 << 10) | (1 << 11) | (1 << 12) | (1 << 14));
    EXTI->PR = pending; // clear flags
    if (pending & (1 << 10)) {
        // START_BUTTON_PIN/PD10 changed
        startButton.isr(GPIOD->IDR);
    }
    if (pending & (1 << 11)) {
        // DRV_SNSOUT_PIN/PD11 changed
        pid.faults.snsoutFault = true;
        pid.faults.count++;
    }
    if (pending & (1 << 12)) {
        // OCP_INT_PIN/PB12 changed
        pid.faults.ocpFault = true;
        pid.faults.count++;
    }
    if (pending & (1 << 14)) {
        // DRV8701_FAULT_PIN/PB14 changed
        pid.faults.drv8701Fault = true;
        pid.faults.count++;
    }
}

static uint32_t timer6Counter = 0;

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) { // every 5ms
        if (timer6Counter++ >= 5) { 
            timer6Counter = 0;
            knob.isr(); // every 25ms
        }
        pid.isr(); 
    }
}

void EXTI_init()
{
    // EXTI8-11 -> Port D
    AFIO->EXTICR[2] =
        (0x3 << 0) |    // EXTI8 PD8
        (0x3 << 4) |    // EXTI9 PD9
        (0x3 << 8) |    // EXTI10 PD10
        (0x3 << 12);    // EXTI11 PD11

    // EXTI12, EXTI14 -> Port B
    AFIO->EXTICR[3] =
        (0x1 << 0) |    // EXTI12 PB12
        (0x1 << 8);     // EXTI14 PB14

    // Clear pending flags
    EXTI->PR =  
        (1U<<8)  |   // PD8  BTN_1
        (1U<<9)  |   // PD9  BTN_2
        (1U<<10) |   // PD10 BTN_3
        (1U<<11) |   // PD11 DRV_SNSOUT
        (1U<<12) |   // PB12 OCP_INT
        (1U<<14);    // PB14 DRV_FAULT

    // Enable interrupt lines
    EXTI->IMR |= 
        (1U<<8)  |   // PD8  BTN_1
        (1U<<9)  |   // PD9  BTN_2
        (1U<<10) |   // PD10 BTN_3
        (1U<<11) |   // PD11 DRV_SNSOUT
        (1U<<12) |   // PB12 OCP_INT
        (1U<<14);    // PB14 DRV_FAULT

    // Rising edge: button change interrupt
    EXTI->RTSR |= 
        (1U<<8)  |   // PD8  BTN_1
        (1U<<9)  |   // PD9  BTN_2
        (1U<<10);    // PD10 BTN_3

    // Falling edge: button change + fault inputs
    EXTI->FTSR |= 
        (1U<<8)  |   // PD8  BTN_1
        (1U<<9)  |   // PD9  BTN_2
        (1U<<10) |   // PD10 BTN_3
        (1U<<11) |   // PD11 DRV_SNSOUT
        (1U<<12) |   // PB12 OCP_INT
        (1U<<14);    // PB14 DRV_FAULT

    // Enable NVIC
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Timer_Config() 
{
    // TIM7 for microsecond delay
    tim7.Instance = TIM7;
    tim7.Init.Prescaler = 71; // 72 MHz / 72 = 1 MHz (1 us tick)
    tim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim7.Init.Period = 0xFFFF;
    tim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    __HAL_RCC_TIM7_CLK_ENABLE();
    HAL_TIM_Base_Init(&tim7);
    HAL_TIM_Base_Start(&tim7);

    // TIM6 for periodic interrupts
    tim6.Instance = TIM6;
    tim6.Init.Prescaler = 71; // 72 MHz / 72 = 1 MHz (1 us tick)
    tim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim6.Init.Period = 5000 - 1; // 5000 counts = 5ms
    tim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_TIM_Base_Init(&tim6);
    HAL_NVIC_SetPriority(TIM6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
}

extern "C" void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    Timer_Config();
    setup();

    while (1) {
        loop();
        // TODO feed WDT?
    }
}

#endif

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
extern "C" void SystemClock_Config(void)
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
     *   - HAL_Delay functions
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
