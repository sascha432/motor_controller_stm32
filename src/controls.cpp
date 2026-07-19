/**
  Author: sascha_lammers@gmx.de
*/

#include "controls.h"

template<uint8_t GPIO_PIN, bool ACTIVE_STATE, uint32_t kDebounceTimeMs, uint32_t GPIO_PORT_ADDR>
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

template<uint8_t GPIO_PIN, bool ACTIVE_STATE, uint32_t kDebounceTimeMs, uint32_t GPIO_PORT_ADDR>
void Button<GPIO_PIN, ACTIVE_STATE, kDebounceTimeMs, GPIO_PORT_ADDR>::isr(uint32_t idr)
{
  bool buttonState = idr & (1 << digitalPinToBit(GPIO_PIN));
  // check if the state has changed
  if (buttonState != state) {
    uint32_t now = HAL_GetTick();
    if (now - lastDebounceTime > kDebounceTimeMs) {
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

template<uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B, uint32_t GPIO_PORT_ADDR>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B, GPIO_PORT_ADDR>::init()
{
  static_assert(digitalPinToGPIOBase<GPIO_PIN_A>() == digitalPinToGPIOBase<GPIO_PIN_B>(), "Rotary encoder pins must be on the same GPIO port");

  // Enable GPIOx clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPxEN(GPIO_PORT_ADDR);

  // Input with pull-up/pull-down (CNF=10, MODE=00)
  GPIO_CRx_REG<GPIO_PIN_A>() &= ~(0xF << digitalPinShift(GPIO_PIN_A));
  GPIO_CRx_REG<GPIO_PIN_B>() &= ~(0xF << digitalPinShift(GPIO_PIN_B));
  GPIO_CRx_REG<GPIO_PIN_A>() |= (0x8 << digitalPinShift(GPIO_PIN_A));
  GPIO_CRx_REG<GPIO_PIN_B>() |= (0x8 << digitalPinShift(GPIO_PIN_B));

  // Select pull-up (ODR bit = 1)
  getGPIOPort()->ODR |= (1 << digitalPinToBit(GPIO_PIN_A)) | (1 << digitalPinToBit(GPIO_PIN_B));

  oldState = readState();
  lastTimestamp = ms();
}

template<uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B, uint32_t GPIO_PORT_ADDR>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B, GPIO_PORT_ADDR>::clear()
{
  __disable_irq();
  position = 0;
  lastTimestamp = ms();
  #if ROTARY_ENCODER_USE_ACCELERATION
  deltaFiltered = 0;
  #endif
  oldState = readState();
  __enable_irq();
}

template<uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B, uint32_t GPIO_PORT_ADDR>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B, GPIO_PORT_ADDR>::enable(InterruptCallbackType callback)
{
  attachInterrupt(digitalPinToInterrupt(GPIO_PIN_A), callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(GPIO_PIN_B), callback, CHANGE);
}

template<uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B, uint32_t GPIO_PORT_ADDR>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B, GPIO_PORT_ADDR>::disable()
{
  detachInterrupt(digitalPinToInterrupt(GPIO_PIN_A));
  detachInterrupt(digitalPinToInterrupt(GPIO_PIN_B));
}

template<uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B, uint32_t GPIO_PORT_ADDR>
void RotaryEncoder<GPIO_PIN_A, GPIO_PIN_B, GPIO_PORT_ADDR>::isr()
{
  static const int8_t table[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
    -1,  0,  0,  1,
    0,  1, -1,  0
  };
  uint8_t state = readState();
  uint8_t index = (oldState << 2) | state;
  position -= table[index]; // change this to -= or += to invert direction
  oldState = state;

  uint32_t timestamp = ms();
  uint32_t delta = timestamp - lastTimestamp;
  lastTimestamp = timestamp;
  #if ROTARY_ENCODER_USE_ACCELERATION
  if (!delta) {
    // skip to avoid division by zero
  }
  else if (delta > kResetAccelerationTime) {
    // reset acceleration if no movement for 1 second
    deltaFiltered = 0;
  }
  else {
    // invert delta to get a multiplier for acceleration
    delta = kMultiplierConstant / delta;
    // low-pass filter for acceleration
    // deltaFiltered = (deltaFiltered * 15 + delta) / 16;
    deltaFiltered = deltaFiltered - (deltaFiltered >> 4) + (delta >> 4);
  }
  #endif
}

template struct RotaryEncoder<ROTARY_ENCODER_PIN_A, ROTARY_ENCODER_PIN_B, digitalPinToGPIOBase<ROTARY_ENCODER_PIN_A>()>;
template struct Button<KNOB_BUTTON_PIN, false>;
template struct Button<START_BUTTON_PIN, false>;
template struct Button<BACK_BUTTON_PIN, false>;

