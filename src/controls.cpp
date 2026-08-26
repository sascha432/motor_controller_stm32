/**
  Author: sascha_lammers@gmx.de
*/

#include "controls.h"
#include "debug.h"

RotaryEncoderKnob knob;
KnobButton knobButton;
BackButton backButton;
StartButton startButton;
TIM_HandleTypeDef tim3;

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

void RotaryEncoder::init()
{
    // GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // TIM3 clock
    __HAL_RCC_TIM3_CLK_ENABLE();

    // ENC2_A / ENC2_B as TIM3_CH1 / TIM3_CH2
    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = ENC2_A_Pin | ENC2_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ENC2_A_GPIO_Port, &GPIO_InitStruct);

    // TIM3 encoder init
    tim3.Instance = TIM3;
    tim3.Init.Prescaler = 0;
    tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim3.Init.Period = 0xFFFF;
    tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    TIM_Encoder_InitTypeDef sEncoderConfig = {};
    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    // channel 1
    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 0;
    // channel 2
    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 0;

    HAL_TIM_Encoder_Init(&tim3, &sEncoderConfig);
    HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_ALL);
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
