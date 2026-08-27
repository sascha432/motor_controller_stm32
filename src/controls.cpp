/**
  Author: sascha_lammers@gmx.de
*/

#include "controls.h"
#include "debug.h"

RotaryEncoderKnob knob;
KnobButton knobButton;
BackButton backButton;
StartButton startButton;

template <uint16_t GPIO_Pin, bool ACTIVE_STATE, uint32_t DEBOUNCE_TIME_MILLIS>
void Button<GPIO_Pin, ACTIVE_STATE, DEBOUNCE_TIME_MILLIS>::init(CallbackType releaseCallback, CallbackType isDownCallback)
{
    this->releaseCallback = releaseCallback;
    this->isDownCallback = isDownCallback;

    // Enable GPIO clock
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    reset();
}

template <uint16_t GPIO_Pin, bool ACTIVE_STATE, uint32_t DEBOUNCE_TIME_MILLIS>
void Button<GPIO_Pin, ACTIVE_STATE, DEBOUNCE_TIME_MILLIS>::isDownIsr()
{
    if (isDownCallback && pressed && !released) {
        isDownCallback(HAL_GetTick() - lastPressedTime);
    }
}

template <uint16_t GPIO_Pin, bool ACTIVE_STATE, uint32_t DEBOUNCE_TIME_MILLIS>
void Button<GPIO_Pin, ACTIVE_STATE, DEBOUNCE_TIME_MILLIS>::isr(uint32_t idr)
{
    bool buttonState = (idr & GPIO_Pin);
    // check if the state has changed
    if (buttonState != state) {
        uint32_t now = HAL_GetTick();
        if (now - lastDebounceTime > kDebounceTimeMs)
        {
            lastDebounceTime = now;
            state = buttonState;
            if (state == ACTIVE_STATE) {
                if (!pressed) {
                    lastPressedTime = now;
                }
                // once the button is pressed, set pressed to true and released to false
                pressed = true;
                released = false;
            }
            else {
                // once the button is released, set released to true
                released = true;
                if (releaseCallback) {
                    releaseCallback(now - lastPressedTime);
                }
            }
        }
    }
}

void RotaryEncoder::reset()
{
    __disable_irq();
    position = 0;
    acceleration = 0;
    __enable_irq();
}

void RotaryEncoder::isr()
{
    int16_t value = UI_READ_ROTARY_KNOB_COUNTER();
    if (!value) {
        // decay
        acceleration -= (acceleration >> 2) + 1;
        if (acceleration < 0) {
            acceleration = 0;
        }
    }
    else {
        UI_WRITE_ROTARY_KNOB_COUNTER(0);
        // acceleration
        acceleration += value * value * 2;
        if (acceleration > static_cast<int32_t>(maxAcceleration)) {
            acceleration = maxAcceleration;
        }
        value *= (acceleration - 1);

        // apply movement
        position -= value;
    }
}

template struct Button<BTN_1_Pin>;
template struct Button<BTN_2_Pin>;
template struct Button<BTN_3_Pin>;
