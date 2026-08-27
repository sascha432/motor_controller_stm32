/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "main.h"
#include "helpers.h"
#include "debug.h"

/**
 * @brief Button template for handling GPIO buttons with debounce and interrupt support (GPIOD only!)
 *
 * @tparam GPIO_PIN GPIO pin (MUST BE GPIOD)
 * @tparam ACTIVE_STATE false for active low, true for active high
 * @tparam
 */
template<uint16_t GPIO_Pin, bool ACTIVE_STATE = false, uint32_t DEBOUNCE_TIME_MILLIS = 50>
struct Button
{
    using CallbackType = void (*)(uint32_t duration);

    static constexpr bool kActiveState = ACTIVE_STATE;
    static constexpr uint32_t kDebounceTimeMs = DEBOUNCE_TIME_MILLIS;

    /**
     * @brief Initialize GPIO and states for the button
     *
     */
    void init(CallbackType releaseCallback = nullptr, CallbackType isDownCallback = nullptr);

    /**
     * @brief remove pressed state
     *
     */
    inline void reset()
    {
        __disable_irq();
        lastDebounceTime = 0;
        lastPressedTime = HAL_GetTick();
        state = readState();
        pressed = (state == kActiveState);
        released = !pressed;
        __enable_irq();
    }

    /**
     * @brief check if the button has been released
     *
     * This method can be used to detect a single button release event
     *
     * @return duration in milliseconds if the button has been released
     * @return 0 otherwise
     */
    inline uint32_t isReleased()
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

    /**
     * @brief Get button down state
     *
     * @return true if the button is down/pressed
     * @return false otherwise
     */
    inline bool isDown() const
    {
        return (state == kActiveState);
    }

    /**
     * @brief Get button pressed state
     *
     * @return true if the button has been pressed
     * @return false otherwise
     */
    inline bool isPressed() const
    {
        return pressed;
    }

    /**
     * @brief Interrupt service routine to be called when the button PIN state changes
     *
     * @param idr The GPIO IDR register value of the related PIN
     */
    void isr(uint32_t idr);

    /**
     * @brief Interrupt service routine to be called when the button is pressed down
     *
     */
    void isDownIsr();

    /**
     * @brief Call ISR with the PINs value register
     *
     */
    inline void isr()
    {
        isr(GPIOD->IDR);
    }

    /**
     * @brief Read GPIO pin state directly and return true if the button is pressed (active)
     *
     * @return true
     * @return false
     */
    bool readGPIOState() const
    {
        return (readState() == kActiveState);
    }

protected:
    /**
     * @brief Read the current state of the button PIN
     *
     * @return true
     * @return false
     */
    inline bool readState() const
    {
        return GPIOD->IDR & GPIO_Pin;
    }

protected:
    CallbackType releaseCallback;
    CallbackType isDownCallback;
    uint32_t lastDebounceTime;
    uint32_t lastPressedTime;
    volatile bool state;
    volatile bool pressed;
    volatile bool released;
};

/**
 * @brief Rotary encoder
 *
 */
struct RotaryEncoder
{
    /**
     * @brief Ctor
     *
     */
    RotaryEncoder() :
        maxAcceleration(1),
        position(0)
    {}

    /**
     * @brief Reset rotary encoder value
     *
     */
    void reset();

    /**
     * @brief Interrupt handler to be called every 20-30ms
     *
     */
    void isr();

    /**
     * @brief Set the acceleration factor
     *
     * @param acceleration
     */
    inline void setMaxAcceleration(uint32_t acceleration)
    {
        maxAcceleration = acceleration + 1;
    }

    /**
     * @brief Get the position changes since last call
     *
     * @return int32_t Number of full rotations since last call
     */
    inline int32_t getPositionDelta()
    {
        __disable_irq();
        int32_t tmpDelta = (position / 2); // full rotations only
        position -= tmpDelta * 2;
        __enable_irq();
        return tmpDelta;
    }

protected:
    volatile uint32_t maxAcceleration;
    volatile int32_t position;
    int32_t acceleration;
};

using RotaryEncoderKnob = RotaryEncoder;
using KnobButton = Button<BTN_1_Pin>;           // GPIOD only
using BackButton = Button<BTN_2_Pin>;           // GPIOD only
using StartButton = Button<BTN_3_Pin>;          // GPIOD only

extern RotaryEncoderKnob knob;
extern KnobButton knobButton;
extern StartButton startButton;
extern BackButton backButton;
