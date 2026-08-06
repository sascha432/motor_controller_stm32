/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "helpers.h"
#include "pins.h"
#include "debug.h"

/**
 * @brief Button template for handling GPIO buttons with debounce and interrupt support
 *
 * @tparam GPIO_PIN  GPIO pin number, e.g. PB10, PA0, etc.
 * @tparam GPIO_PORT_ADDR GPIO port address, e.g. GPIOA_BASE, GPIOB_BASE, etc.
 * @tparam ACTIVE_STATE false for active low, true for active high
 */
template<uint8_t GPIO_PIN, bool ACTIVE_STATE, uint32_t kDebounceTimeMs = 50>
struct Button
{
    /**
     * @brief Initialize GPIO and states for the button
     *
     */
    void init();

    /**
     * @brief remove pressed state
     *
     */
    inline void reset()
    {
        __disable_irq();
        lastDebounceTime = 0;
        state = readState();;
        pressed = (state == ACTIVE_STATE);
        released = !pressed;
        __enable_irq();
    }

    /**
     * @brief check if the button has been released
     *
     * This method can be used to detect a single button release event
     *
     * @return true if the button has been released
     * @return false otherwise
     */
    inline bool isReleased()
    {
        __disable_irq();
        // check if the button has been pressed and released
        if (released && pressed) {
            pressed = false; // clear pressed flag
            __enable_irq();
            return true;
        }
        __enable_irq();
        return false;
    }

    /**
     * @brief Get button down state
     *
     * @return true if the button is down/pressed
     * @return false otherwise
     */
    inline bool isDown() const
    {
        return (state == ACTIVE_STATE);
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
     * @brief Call ISR with the PINs value register
     *
     */
    inline void isr()
    {
        isr(digitalPinToGPIO<GPIO_PIN>()->IDR);
    }

protected:
    /**
     * @brief Read the current state of the button PIN
     *
     * @return true
     * @return false
     */
    inline bool readState()
    {
        return digitalRead<GPIO_PIN>();
    }

protected:
    uint32_t lastDebounceTime;
    volatile bool state;
    volatile bool pressed;
    volatile bool released;
};

/**
 * @brief Rotary encoder
 *
 * @tparam PIN_A bit for pin A in the GPIO IDR register
 * @tparam PIN_B bit for pin B in the GPIO IDR register
 * @tparam GPIO_PORT_ADDR address of the GPIO port, pin A and pin B must be on the same port
 */
template<uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B>
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
     * @brief Initialize GPIO and TIM3 for rotary encoder
     *
     */
    void init();

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

using RotaryEncoderKnob = RotaryEncoder<ROTARY_ENCODER_PIN_A, ROTARY_ENCODER_PIN_B>;
using StartButton = Button<START_BUTTON_PIN, false>;
using KnobButton = Button<KNOB_BUTTON_PIN, false>;
using BackButton = Button<BACK_BUTTON_PIN, false>;

extern RotaryEncoderKnob knob;
extern KnobButton knobButton;
extern StartButton startButton;
extern BackButton backButton;
