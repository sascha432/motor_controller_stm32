/**
  Author: sascha_lammers@gmx.de
*/

#include "controls.h"
#include "debug.h"

RotaryEncoderKnob knob;
KnobButton knobButton;
StartButton startButton;
BackButton backButton;

template <uint8_t GPIO_PIN, bool ACTIVE_STATE, uint32_t kDebounceTimeMs>
void Button<GPIO_PIN, ACTIVE_STATE, kDebounceTimeMs>::init()
{
    // Enable GPIO clock
    __HAL_RCC_GPIOx_CLK_ENABLE<GPIO_PIN>();

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = digitalPinToHAL<GPIO_PIN>();
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(digitalPinToGPIO<GPIO_PIN>(), &GPIO_InitStruct);

    // Initial state
    lastDebounceTime = 0;
    state = readState();;
    pressed = (state == ACTIVE_STATE);
    released = !pressed;
}

template <uint8_t GPIO_PIN, bool ACTIVE_STATE, uint32_t kDebounceTimeMs>
void Button<GPIO_PIN, ACTIVE_STATE, kDebounceTimeMs>::isr(uint32_t idr)
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

template <uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B>::init()
{
    static_assert(GPIO_PIN_A == PA6 && GPIO_PIN_B == PA7, "PA6 and PA7 are hardcoded");

    // GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // TIM3 clock
    __HAL_RCC_TIM3_CLK_ENABLE();

    // PA6 / PA7 as TIM3_CH1 / TIM3_CH2
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // TIM3 encoder init
    TIM_HandleTypeDef tim3 = { 0 };
    tim3.Instance = TIM3;
    tim3.Init.Prescaler = 0;
    tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim3.Init.Period = 0xFFFF;
    tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    TIM_Encoder_InitTypeDef sEncoderConfig = { 0 };
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

template <uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B>::clear()
{
    position = 0;
    acceleration = 0;
}

template <uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B>::isr()
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
        if (acceleration > static_cast<int32_t>(maxAcceleration)) {
            acceleration = maxAcceleration;
        }
        value *= (acceleration - 1);

        // apply movement
        position -= value;
    }
}

template struct RotaryEncoder<ROTARY_ENCODER_PIN_A, ROTARY_ENCODER_PIN_B>;
template struct Button<KNOB_BUTTON_PIN, false>;
template struct Button<START_BUTTON_PIN, false>;
template struct Button<BACK_BUTTON_PIN, false>;
