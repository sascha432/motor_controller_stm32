/**
  Author: sascha_lammers@gmx.de
*/

#include "helpers.h"
#include "pid_controller.h"
#include "controls.h"

// === interrupt handlers ===

TIM_HandleTypeDef tim6;
uint32_t timer6Counter = 0;

extern "C" void TIM6_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim6);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) { // every 5us
        pid.ocp_isr();
        if ((timer6Counter & 0x3ff) == 0) { // every 5.12ms
            pid.isr();
            if (timer6Counter % 5120 == 0) { // every 25.6ms
                knob.isr();
            }
        }
        timer6Counter++;
    }
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

// DMA1_Channel1_IRQHandler is the interrupt handler for the DMA1 Channel 1. It is called when a DMA transfer is complete
extern "C" void DMA1_Channel1_IRQHandler()
{
    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
        // clear transfer complete
        DMA1->IFCR = DMA_IFCR_CTCIF1;

        // hard fault if the have a OVP condition, mostly likely due to reverse currents while braking
        if (adc.getVSenseValue() > pid.faults.vsenseMax) {
            if (pid.errorCode != PidController::ErrorCodeType::OVP) {
                pid.setErrorCode(PidController::ErrorCodeType::OVP);
                LEDs::onLEDError();
            }
        }

        auto value = adc.getISenseValue();
        // store average for display
        adc.isenseSum += value;
        if (++adc.isenseCount >= ADC::kISenseCountMax) {
            // reduce by 6.5% to avoid overflow in rolling average
            adc.isenseSum -= adc.isenseSum / 16;
            adc.isenseCount -= adc.isenseCount / 16;
        }
        // store filtered value for fast OCP detection
        adc.isenseOcpFiltered = (adc.isenseOcpFiltered + value) / 2;
    }
}

WWDG_HandleTypeDef WatchDog::watchdog;

extern "C" void HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg)
{
    (void)hwwdg;
    __HAL_RCC_WWDG_CLK_ENABLE();
    HAL_NVIC_SetPriority(WWDG_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(WWDG_IRQn);
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

InterruptErrorType interruptErrorType;

// for disabled interrupts
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
    // turn motor off
    PID_WRITE_MOTOR_PWM_OFF();
    #if DEBUG
    #if DEBUG_OUTPUT == DEBUG_OUTPUT_SWD
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
    #endif
    WatchDog::stop();
    __disable_irq();
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
