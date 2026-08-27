/**
  Author: sascha_lammers@gmx.de
*/

#include "helpers.h"
#include "pid_controller.h"
#include "controls.h"
#include "tft_driver.h"

// === global variables ===

WWDG_HandleTypeDef WatchDog::watchdog;
InterruptErrorType interruptErrorType;

// === interrupt handlers ===

/**
 * @brief Periodically call PID controller methods
 */
static inline void TIM6_Handler(void)
{
    // call PID isr
    pid.isr();

    // call knob and button ISRs
    constexpr uint32_t kTicksPerKnobIsr = (uint32_t)(25.0f / PidController::kPIDInterval);
    constexpr float kKnobTimingMillis = kTicksPerKnobIsr * PidController::kPIDInterval;
    (void)kKnobTimingMillis;

    static uint32_t timer6Counter = 0;
    if (++timer6Counter > kTicksPerKnobIsr) {
        timer6Counter = 0;
        knob.isr();
        backButton.isDownIsr();
    }
}

/**
 * @brief TIM6 IRQ Handler
 *
 */
extern "C" void TIM6_IRQHandler(void)
{
    TIM6->SR &= ~TIM_SR_UIF;
    TIM6_Handler();
}

/**
 * @brief Handle external interrupts for buttons
 *
 */
extern "C" void EXTI9_5_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & (BTN_1_Pin | BTN_2_Pin);
    EXTI->PR = pending; // clear flags
    if (pending & BTN_1_Pin) {
        // changed event (rising and falling)
        knobButton.isr(BTN_1_GPIO_Port->IDR);
    }
    if (pending & BTN_2_Pin) {
        // changed event
        backButton.isr(BTN_2_GPIO_Port->IDR);
    }
}

/**
 * @brief Handle external interrupts for buttons and faults
 *
 */
extern "C" void EXTI15_10_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & (BTN_3_Pin | DRV_SNSOUT_Pin | GPIO_PIN_12 | DRV_FAULT_Pin);
    EXTI->PR = pending; // clear flags
    if (pending & BTN_3_Pin) {
        // changed event
        startButton.isr(BTN_3_GPIO_Port->IDR);
    }
    if (pending & DRV_SNSOUT_Pin) {
        // changed event
        pid.faults.resetSnsoutFault();
    }
    if (pending & DRV_FAULT_Pin) {
        // changed event
        auto fault = pid.faults.drv8701Fault;
        pid.faults.resetDrvFault();
        if (!fault && pid.faults.drv8701Fault) {
            LEDs::onLEDError(); // turn fault LED on, main loop resets it after the fault has cleared
        }
    }
}

/**
 * @brief DMA1_Channel1_IRQHandler is the interrupt handler for the DMA1 Channel 1. It is called when a DMA transfer is complete
 *
 */
extern "C" void DMA1_Channel1_IRQHandler()
{
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CGIF1;
        adc.isr();
    }
}

/**
 * @brief DMA1_Channel5_IRQHandler is the interrupt handler for the TFT SPI TX DMA channel
 */
extern "C" void DMA1_Channel5_IRQHandler(void)
{
    if (DMA1->ISR & (DMA_ISR_TCIF5 | DMA_ISR_TEIF5)) {
        DMA1->IFCR = DMA_IFCR_CGIF5;
        tft_driver_dma_transfer_finished_isr();
    }
}

/**
 * @brief ADC1_2_IRQHandler is the interrupt handler for the injected ADC group. It is called
 *        when an injected conversion sequence (JEOC) completes, once per PWM period
 *
 */
extern "C" void ADC1_2_IRQHandler()
{
    if (ADC1->SR & ADC_SR_JEOC) {
        ADC1->SR = ~ADC_SR_JEOC;    // clear injected end of sequence flag
        adc.isrInjected();
    }
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

#if HAVE_USB_DEVICE

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
extern "C" void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

#endif

/**
 * @brief Delay with interrupt disabled
 *
 * @param ms Time in milliseconds
 */
static void delay_ms(uint32_t ms)
{
    constexpr uint32_t kTicksPerMs = F_CPU / 1000;
    constexpr uint32_t kMaxMillis = UINT32_MAX / kTicksPerMs;
    while(ms > kMaxMillis) {
        ms -= kMaxMillis;
        delay_ms(kMaxMillis);
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = ms * kTicksPerMs;
    while ((DWT->CYCCNT - start) < ticks) {
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

    #if DEBUG && (DEBUG_OUTPUT == DEBUG_OUTPUT_SWO)
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
        case InterruptErrorType::WATCHDOG_TICK_TIMEOUT:
            SWO::write(0, "WT\n", sizeof("WT\n") - 1);
            break;
    }
    #endif

    // infinite loop to signal error via LED flashes
    while (1) {
        LEDs::onLEDError();
        delay_ms(500);
        // signal error type via LED flashes
        for(int i = 0; i <= (int)interruptErrorType; i++) {
            LEDs::off();
            delay_ms(250);
            LEDs::onLEDWarning();
            delay_ms(250);
        }
        LEDs::off();
        delay_ms(500);
        // check if the back button is pressed and reset MCU
        if (backButton.readGPIOState()) {
            HAL_NVIC_SystemReset();
        }
    }
}
