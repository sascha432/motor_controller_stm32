/**
  Author: sascha_lammers@gmx.de
*/

// === Button ===

template <uint16_t GPIO_Pin, bool ACTIVE_STATE, uint32_t DEBOUNCE_TIME_MILLIS>
inline void Button<GPIO_Pin, ACTIVE_STATE, DEBOUNCE_TIME_MILLIS>::reset()
{
    __disable_irq();
    lastDebounceTime = 0;
    lastPressedTime = HAL_GetTick();
    state = readState();
    pressed = (state == kActiveState);
    released = !pressed;
    __enable_irq();
}

template <uint16_t GPIO_Pin, bool ACTIVE_STATE, uint32_t DEBOUNCE_TIME_MILLIS>
inline uint32_t Button<GPIO_Pin, ACTIVE_STATE, DEBOUNCE_TIME_MILLIS>::isReleased()
{
    __disable_irq();
    // check if the button has been pressed and released
    if (released && pressed) {
        pressed = false; // clear pressed flag
        __enable_irq();
        return (HAL_GetTick() - lastPressedTime) + 1;
    }
    __enable_irq();
    return 0;
}

template <uint16_t GPIO_Pin, bool ACTIVE_STATE, uint32_t DEBOUNCE_TIME_MILLIS>
inline bool Button<GPIO_Pin, ACTIVE_STATE, DEBOUNCE_TIME_MILLIS>::isDown() const
{
    return (state == kActiveState);
}

template <uint16_t GPIO_Pin, bool ACTIVE_STATE, uint32_t DEBOUNCE_TIME_MILLIS>
inline bool Button<GPIO_Pin, ACTIVE_STATE, DEBOUNCE_TIME_MILLIS>::isPressed() const
{
    return pressed;
}

template <uint16_t GPIO_Pin, bool ACTIVE_STATE, uint32_t DEBOUNCE_TIME_MILLIS>
inline void Button<GPIO_Pin, ACTIVE_STATE, DEBOUNCE_TIME_MILLIS>::isr()
{
    isr(GPIO_Port->IDR);
}

template <uint16_t GPIO_Pin, bool ACTIVE_STATE, uint32_t DEBOUNCE_TIME_MILLIS>
inline bool Button<GPIO_Pin, ACTIVE_STATE, DEBOUNCE_TIME_MILLIS>::readGPIOState() const
{
    return (readState() == kActiveState);
}

template <uint16_t GPIO_Pin, bool ACTIVE_STATE, uint32_t DEBOUNCE_TIME_MILLIS>
inline bool Button<GPIO_Pin, ACTIVE_STATE, DEBOUNCE_TIME_MILLIS>::readState() const
{
    return (GPIO_Port->IDR & GPIO_Pin);
}

template <uint16_t GPIO_Pin, bool ACTIVE_STATE, uint32_t DEBOUNCE_TIME_MILLIS>
void Button<GPIO_Pin, ACTIVE_STATE, DEBOUNCE_TIME_MILLIS>::isDownIsr()
{
    if (isDownCallback && pressed && !released) {
        isDownCallback(HAL_GetTick() - lastPressedTime);
    }
}

// === RotaryEncoder ===

inline RotaryEncoder::RotaryEncoder() :
    maxAcceleration(1),
    position(0)
{

}

inline void RotaryEncoder::reset()
{
    __disable_irq();
    position = 0;
    acceleration = 0;
    __enable_irq();
}

inline void RotaryEncoder::setMaxAcceleration(uint32_t acceleration)
{
    maxAcceleration = acceleration + 1;
}

inline int32_t RotaryEncoder::getPositionDelta()
{
    __disable_irq();
    int32_t tmpDelta = (position / 2); // full rotations only
    position -= tmpDelta * 2;
    __enable_irq();
    return tmpDelta;
}
