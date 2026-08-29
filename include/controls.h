/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "main.h"
#include "helpers.h"
#include "debug.h"

/**
 * @brief Button template for handling GPIO buttons with debounce and interrupt support
 *
 * @tparam GPIO_PIN GPIO pin
 * @tparam ACTIVE_STATE false for active low, true for active high
 * @tparam DEBOUNCE_TIME_MILLIS debounce time in milliseconds
 */
template<uint16_t GPIO_Pin, bool ACTIVE_STATE = false, uint32_t DEBOUNCE_TIME_MILLIS = 50>
struct Button
{
    using CallbackType = void (*)(uint32_t duration);

    static constexpr bool kActiveState = ACTIVE_STATE;
    static constexpr uint32_t kDebounceTimeMs = DEBOUNCE_TIME_MILLIS;

    /**
     * @brief ctor
     *
     * @param gpio
     */
    Button(GPIO_TypeDef *gpio);

    /**
     * @brief Initialize GPIO and states for the button
     *
     */
    void init(CallbackType releaseCallback = nullptr, CallbackType isDownCallback = nullptr);

    /**
     * @brief remove pressed state
     *
     */
    void reset();

    /**
     * @brief check if the button has been released
     *
     * This method can be used to detect a single button release event
     *
     * @return duration in milliseconds if the button has been released
     * @return 0 otherwise
     */
    uint32_t isReleased();

    /**
     * @brief Get button down state
     *
     * @return true if the button is down/pressed
     * @return false otherwise
     */
    bool isDown() const;

    /**
     * @brief Get button pressed state
     *
     * @return true if the button has been pressed
     * @return false otherwise
     */
    bool isPressed() const;

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
    void isr();

    /**
     * @brief Read GPIO pin state directly and return true if the button is pressed (active)
     *
     * @return true
     * @return false
     */
    bool readGPIOState() const;

protected:
    /**
     * @brief Read the current state of the button PIN
     *
     * @return true
     * @return false
     */
    bool readState() const;

protected:
    GPIO_TypeDef *GPIO_Port;
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
    RotaryEncoder();

    /**
     * @brief Reset rotary encoder value
     *
     */
    inline void reset();

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
    inline void setMaxAcceleration(uint32_t acceleration);

    /**
     * @brief Get the position changes since last call
     *
     * @return int32_t Number of full rotations since last call
     */
    inline int32_t getPositionDelta();

protected:
    volatile uint32_t maxAcceleration;
    volatile int32_t position;
    int32_t acceleration;
};

#include "controls.hpp"

using RotaryEncoderKnob = RotaryEncoder;
using KnobButton = Button<BTN_1_Pin>;
using BackButton = Button<BTN_2_Pin>;
using StartButton = Button<BTN_3_Pin>;

extern RotaryEncoderKnob knob;
extern KnobButton knobButton;
extern StartButton startButton;
extern BackButton backButton;
