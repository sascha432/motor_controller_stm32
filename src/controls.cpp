/**
  Author: sascha_lammers@gmx.de
*/

#include "controls.h"
#include "debug.h"

RotaryEncoderKnob knob;
KnobButton knobButton;
StartButton startButton;
BackButton backButton;

template <uint8_t GPIO_PIN, bool ACTIVE_STATE, uint32_t kDebounceTimeMs, uint32_t GPIO_PORT_ADDR>
void Button<GPIO_PIN, ACTIVE_STATE, kDebounceTimeMs, GPIO_PORT_ADDR>::init(InterruptCallbackType callback)
{
    // Enable GPIOx clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPxEN(GPIO_PORT_ADDR);

    // Input with pull-up/pull-down (CNF=10, MODE=00)
    GPIO_CRx_REG(GPIO_PORT_ADDR, GPIO_PIN) &= ~(0xF << digitalPinShift(GPIO_PIN));
    GPIO_CRx_REG(GPIO_PORT_ADDR, GPIO_PIN) |= (0x8 << digitalPinShift(GPIO_PIN));

    // Select pull-up (ODR bit = 1)
    getGPIOPort()->ODR |= (1 << digitalPinToBit(GPIO_PIN));

    // --- interrupt — use Arduino attachInterrupt only
    attachInterrupt(digitalPinToInterrupt(GPIO_PIN), callback, CHANGE);

    lastDebounceTime = 0;
    state = getGPIOPort()->IDR & (1 << digitalPinToBit(GPIO_PIN));
    pressed = (state == ACTIVE_STATE);
    released = !pressed;
}

template <uint8_t GPIO_PIN, bool ACTIVE_STATE, uint32_t kDebounceTimeMs, uint32_t GPIO_PORT_ADDR>
void Button<GPIO_PIN, ACTIVE_STATE, kDebounceTimeMs, GPIO_PORT_ADDR>::isr(uint32_t idr)
{
    bool buttonState = idr & (1 << digitalPinToBit(GPIO_PIN));
    // check if the state has changed
    if (buttonState != state) {
        uint32_t now = HAL_GetTick();
        if (now - lastDebounceTime > kDebounceTimeMs)
        {
            lastDebounceTime = now;
            state = buttonState;
            if (state == ACTIVE_STATE) {
                // once the button is pressed, set pressed to true and released to false
                pressed = true;
                released = false;
            }
            else {
                // once the button is released, set released to true
                released = true;
            }
        }
    }
}

template <uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B, uint32_t GPIO_PORT_ADDR>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B, GPIO_PORT_ADDR>::init()
{
    static_assert(digitalPinToGPIOBase<GPIO_PIN_A>() == digitalPinToGPIOBase<GPIO_PIN_B>(), "Rotary encoder pins must be on the same GPIO port");
    static_assert(GPIO_PIN_A == PA6 && GPIO_PIN_B == PA7, "Rotary encoder TIM3 decoder requires PA6 and PA7");

    // Enable GPIOA clock and TIM3 clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPxEN(GPIO_PORT_ADDR);
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Input with pull-up/pull-down (CNF=10, MODE=00)
    GPIO_CRx_REG<GPIO_PIN_A>() &= ~(0xF << digitalPinShift(GPIO_PIN_A));
    GPIO_CRx_REG<GPIO_PIN_B>() &= ~(0xF << digitalPinShift(GPIO_PIN_B));
    GPIO_CRx_REG<GPIO_PIN_A>() |= (0x8 << digitalPinShift(GPIO_PIN_A));
    GPIO_CRx_REG<GPIO_PIN_B>() |= (0x8 << digitalPinShift(GPIO_PIN_B));

    // // Select pull-up (ODR bit = 1)
    // getGPIOPort()->ODR |= (1 << digitalPinToBit(GPIO_PIN_A)) | (1 << digitalPinToBit(GPIO_PIN_B));

    // TIM3 quadrature encoder on PA6/PA7
    TIM3->CR1 = 0;
    TIM3->SMCR = TIM_SMCR_SMS_0;
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC2PSC_0;
    TIM3->CCER = 0;
    TIM3->PSC = 0;
    TIM3->ARR = 0xFFFF;
    TIM3->CNT = 0;
    TIM3->CR1 |= TIM_CR1_CEN;
}

template <uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B, uint32_t GPIO_PORT_ADDR>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B, GPIO_PORT_ADDR>::clear()
{
    __disable_irq();
    position = 0;
    acceleration = 0;
    __enable_irq();
}

template <uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B, uint32_t GPIO_PORT_ADDR>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B, GPIO_PORT_ADDR>::enable(InterruptCallbackType callback)
{
    timer.setOverflow(25000, MICROSEC_FORMAT);
    timer.attachInterrupt(callback);
    timer.resume();
}

template <uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B, uint32_t GPIO_PORT_ADDR>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B, GPIO_PORT_ADDR>::disable()
{
    timer.pause();
    timer.detachInterrupt();
}

template <uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B, uint32_t GPIO_PORT_ADDR>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B, GPIO_PORT_ADDR>::isr()
{
    int16_t value = TIM3->CNT;
    if (!value) {
        // decay
        acceleration -= (acceleration >> 2) + 1; 
        if (acceleration < 0) {
            acceleration = 0;
        }
    }
    else {
        TIM3->CNT = 0;
        // acceleration
        acceleration += value * value * 2; 
        if (acceleration > maxAcceleration) {
            acceleration = maxAcceleration;
        }
        value *= (acceleration - 1);

        // apply movement
        position -= value;
    }

    // __enable_irq();
    //  DEBUG_PRINT(DEBUG_DEBUG, "cnt=%d a=%d p=%d", value, acceleration, position);
}

template struct RotaryEncoder<ROTARY_ENCODER_PIN_A, ROTARY_ENCODER_PIN_B, digitalPinToGPIOBase<ROTARY_ENCODER_PIN_A>()>;
template struct Button<KNOB_BUTTON_PIN, false>;
template struct Button<START_BUTTON_PIN, false>;
template struct Button<BACK_BUTTON_PIN, false>;
