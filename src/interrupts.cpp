/**
  Author: sascha_lammers@gmx.de
*/

#include "helpers.h"
#include "pid_controller.h"
#include "controls.h"

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O3")
#endif

// === global variables ===

WWDG_HandleTypeDef WatchDog::watchdog;
InterruptErrorType interruptErrorType;

// === interrupt handlers ===

// avoid overhead for fast timer interrupt and call timer6 handler directly
// other events might interfere with code that is not handled by HAL, but TIM6 is dedicated for the PID controller timing
#define CALL_TIM6_HANDLER_DIRECTLY 1

/**
 * @brief Periodically call PID controller methods
 *
 * Handler is executed every 5us
 */
static inline void TIM6_Handler(void)
{
    static uint32_t timer6Counter = 0;
    pid.ocp_isr();
    if ((timer6Counter & 0x3ff) == 0) { // every 5.12ms (1024x5us)
        pid.isr();
        if (timer6Counter % 5120 == 0) { // every 25.6ms
            knob.isr();
        }
    }
    timer6Counter++;
}

/**
 * @brief TIM6 IRQ Handler
 *
 */
extern "C" void TIM6_IRQHandler(void)
{
    #if CALL_TIM6_HANDLER_DIRECTLY
        TIM6->SR &= ~TIM_SR_UIF;
        TIM6_Handler();
    #else
        extern TIM_HandleTypeDef tim6;
        HAL_TIM_IRQHandler(&tim6);
    #endif
}

#if !CALL_TIM6_HANDLER_DIRECTLY
/**
 * @brief Callback for TIM period elapsed
 *
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) { // every 5us
        TIM6_Handler();
    }
}
#endif

/**
 * @brief Handle external interrupts for buttons
 *
 */
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

/**
 * @brief Handle external interrupts for buttons and faults
 *
 */
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
        pid.faults.snsoutFault = !digitalRead<DRV_SNSOUT_PIN>();
    }
    if (pending & (1 << 14)) {
        // DRV8701_FAULT_PIN/PB14 changed
        auto fault = pid.faults.drv8701Fault;
        pid.faults.drv8701Fault = !digitalRead<DRV8701_FAULT_PIN>();
        if (!fault && pid.faults.drv8701Fault) {
            LEDs::onLEDError(); // turn fault LED on, main loop resets it after the fault has cleared
        }
    }
    if (pending & (1 << 12)) {
        // OCP_INT_PIN/PB12 falling edge
        pid.faults.ocpFault = true;
        pid.trigger_ocp();
    }
}

/**
 * @brief DMA1_Channel1_IRQHandler is the interrupt handler for the DMA1 Channel 1. It is called when a DMA transfer is complete
 *
 */
extern "C" void DMA1_Channel1_IRQHandler()
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}

/**
 * @brief Called from HAL_DMA_IRQHandler
 *
 */
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    adc.isr();
}

// === interrupt handlers ===

/**
  * @brief This function handles System tick timer.
  */
extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
    WatchDog::tickHandler();
}

/**
  * @brief This function handles Non maskable interrupt.
  */
extern "C" void NMI_Handler(void)
{
    call_default_error_handler(InterruptErrorType::NMI_HANDLER);
}

/**
  * @brief This function handles Hard fault interrupt.
  */
extern "C" void HardFault_Handler(void)
{
    call_default_error_handler(InterruptErrorType::HARD_FAULT_HANDLER);
}

/**
  * @brief This function handles Memory management fault.
  */
extern "C" void MemManage_Handler(void)
{
    call_default_error_handler(InterruptErrorType::MEM_MANAGE_HANDLER);
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
extern "C" void BusFault_Handler(void)
{
    call_default_error_handler(InterruptErrorType::BUS_FAULT_HANDLER);
}

/**
 * @brief This function handles Usage fault interrupt
 *
 */
extern "C" void UsageFault_Handler(void)
{
    call_default_error_handler(InterruptErrorType::USAGE_FAULT_HANDLER);
}

/**
 * @brief This function handles the Window Watchdog interrupt
 */
extern "C" void WWDG_IRQHandler(void)
{
    HAL_WWDG_IRQHandler(&WatchDog::watchdog);
}

/**
 * @brief Handle watchdog timeouts
 */
extern "C" void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
    if (hwwdg != &WatchDog::watchdog) {
        return;
    }
    call_default_error_handler(InterruptErrorType::WATCHDOG_TIMEOUT);
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

// for disabled interrupts, not precise
static void delay_ms(uint32_t ms)
{
    volatile uint32_t count = ms * 5000;
    while (count--) {
        __NOP();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
extern "C" void Error_Handler(void)
{
    // disable interrupts
    __disable_irq();
    // turn motor off
    PID_WRITE_MOTOR_PWM_OFF();
    // disable watchdog
    WatchDog::deinit();

    #if DEBUG && (DEBUG_OUTPUT == DEBUG_OUTPUT_SWD)
    // report error type via SWO
    switch(interruptErrorType) {
        case InterruptErrorType::ERROR_HANDLER:
            SWO::write(0, "EH\n", sizeof("EH\n") - 1);
            break;
        case InterruptErrorType::NMI_HANDLER:
            SWO::write(0, "NMI\n", sizeof("NMI\n") - 1);
            break;
        case InterruptErrorType::HARD_FAULT_HANDLER:
            SWO::write(0, "HF\n", sizeof("HF\n") - 1);
            break;
        case InterruptErrorType::MEM_MANAGE_HANDLER:
            SWO::write(0, "MM\n", sizeof("MM\n") - 1);
            break;
        case InterruptErrorType::BUS_FAULT_HANDLER:
            SWO::write(0, "BF\n", sizeof("BF\n") - 1);
            break;
        case InterruptErrorType::USAGE_FAULT_HANDLER:
            SWO::write(0, "UF\n", sizeof("UF\n") - 1);
            break;
        case InterruptErrorType::WATCHDOG_TIMEOUT:
            SWO::write(0, "WD\n", sizeof("WD\n") - 1);
            break;
    }
    #endif

    // infinite loop to signal error via LED flashes
    while (1) {
        LEDs::onLEDError();
        delay_ms(1000);
        // signal error type via LED flashes
        for(int i = 0; i <= (int)interruptErrorType; i++) {
            LEDs::off();
            delay_ms(500);
            LEDs::onLEDWarning();
            delay_ms(500);
        }
        LEDs::off();
        delay_ms(500);
    }
}
