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
template<uint8_t GPIO_PIN, bool ACTIVE_STATE, uint32_t kDebounceTimeMs = 50, uint32_t GPIO_PORT_ADDR = digitalPinToGPIOBase<GPIO_PIN>()>
struct Button 
{
    inline GPIO_TypeDef *getGPIOPort() const { return (GPIO_TypeDef *)GPIO_PORT_ADDR; }

    void init(InterruptCallbackType callback);

    /**
     * @brief remove pressed state
     * 
     */
    void clear()
    {
        isPressed();
    }

    /**
     * @brief check if the button has been released
     * 
     * This method can be used to detect a single button release event.
     * 
     * @return true if the button has been released
     * @return false otherwise
     */
    bool isReleased() {
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
     * @brief check if the button has been pressed
     * 
     * This method can be used to detect a single button press event.
     * 
     * @return true if the button has been pressed
     * @return false otherwise
     */
    bool isPressed() {
        __disable_irq();
        bool result = !released && pressed;
        if (result) {
            pressed = false; // clear pressed flag
        }
        __enable_irq();
        return result;
    }

    /**
     * @brief get button pressed state
     * 
     * @return true if the button is down/pressed
     * @return false otherwise
     */
    bool isDown() {
        __disable_irq();
        bool result = !released && pressed;
        __enable_irq();
        return result;
    }

    void isr(uint32_t idr);

    volatile uint32_t lastDebounceTime;
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
template<uint8_t GPIO_PIN_A, uint8_t GPIO_PIN_B, uint32_t GPIO_PORT_ADDR = digitalPinToGPIOBase<GPIO_PIN_A>()>
struct RotaryEncoder {

    inline GPIO_TypeDef *getGPIOPort() const { return (GPIO_TypeDef *)GPIO_PORT_ADDR; }

    static constexpr uint32_t kResetAccelerationTime = 250;                 // reset acceleration if no movement in milliseconds
    static constexpr uint32_t kDefaultAccelerationFactor = 0xffffff;        // speed multiplier constant - higher value = faster acceleration, lower value = slower acceleration

    static constexpr uint32_t kAccelerationFactorMenu = 0;                 // acceleration factor for menu navigation
    static constexpr uint32_t kAccelerationFactorSlow = 0xfffff;           // acceleration factor for values up to 1000
    static constexpr uint32_t kAccelerationFactorFast = 0xffffff;          // acceleration factor for values up to 10000
    static constexpr uint32_t kAccelerationFactorCurrent = 0x2fffff;       // acceleration factor for current values

    /**
     * @brief Helper to scale depending on the maximum value
     * 
     * @param maxValue 
     * @return constexpr uint32_t 
     */
    static constexpr uint32_t kAccelerationHelper(uint32_t maxValue) 
    {
        if (maxValue < 500) {
            return kAccelerationFactorMenu;
        }
        return (0xfffffULL * maxValue) / 2048;
    }

    RotaryEncoder() : 
        timer(TIM5),
        maxAcceleration(1),
        position(0)
    {}

    void init();
    void clear();

    void enable(InterruptCallbackType callback);
    void disable();

    void isr();

    /**
     * @brief Get the Delta Position object and clean up the position counter
     * 
     * @return int32_t 
     */
    int32_t getDeltaPosition() {
        __disable_irq();
        int32_t tmpDelta = (position / 2); // full rotations only
        position -= tmpDelta * 2;
        __enable_irq();
        return tmpDelta;
    }

    /**
     * @brief Set the acceleration factor
     * 
     * @param acceleration 
     */
    void setMaxAcceleration(uint32_t acceleration) 
    {
        maxAcceleration = acceleration + 1;
    }

    HardwareTimer timer;
    uint32_t maxAcceleration;
    volatile int32_t position;
    volatile int32_t acceleration;
};

using RotaryEncoderKnob = RotaryEncoder<ROTARY_ENCODER_PIN_A, ROTARY_ENCODER_PIN_B>;
using StartButton = Button<START_BUTTON_PIN, false>;
using KnobButton = Button<KNOB_BUTTON_PIN, false>;
using BackButton = Button<BACK_BUTTON_PIN, false>;

extern RotaryEncoderKnob knob;
extern KnobButton knobButton;
extern StartButton startButton;
extern BackButton backButton;
