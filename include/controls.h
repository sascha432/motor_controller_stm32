/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "helpers.h"
#include "pins.h"

#define ROTARY_ENCODER_USE_ACCELERATION 0

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

    static constexpr uint32_t kMultiplierConstant = 0xffffff;       // speed multiplier constant - higher value = faster acceleration, lower value = slower acceleration
    static constexpr uint32_t kResetAccelerationTime = 250;         // reset acceleration if no movement for 1 second

    struct State {
    private:
        int32_t position;
        uint16_t multiplier;

    public:
        State(int32_t position, uint16_t multiplier) : position(position), multiplier(multiplier) {}

        bool hasPosition() const {
            return position != 0;
        }

        int32_t getPosition() const {
            return position * multiplier;
        }
    };

    void init();
    void clear();

    void enable(InterruptCallbackType callback);
    void disable();

    void isr();

    inline uint8_t readState() const {
        // copy volatile register to local variable to let the compiler optimize the bit operations
        uint32_t idr = getGPIOPort()->IDR;
        
        return 
            (((idr >> digitalPinToBit(GPIO_PIN_A)) & 0x1) |         // shift pin A to bit 0
            ((idr >> (digitalPinToBit(GPIO_PIN_B) - 1)) & 0x2));    // shift pin B to bit 1
    }

    inline uint32_t ms() const {
        return HAL_GetTick();
    }

    /**
     * @brief Get the Delta Position object and clean up the position counter
     * 
     * @return int32_t 
     */
    int32_t getDeltaPosition() {
        __disable_irq();
        int32_t tmpDelta = (position / 4); // full rotations only
        position -= tmpDelta * 4;
        __enable_irq();
        return tmpDelta;
    }

    #if ROTARY_ENCODER_USE_ACCELERATION
    /**
     * @brief Get the Delta Position object and clean up the position counter
     * 
     * @return State 
     */
    State getDeltaPositionAndMultiplier() {
        __disable_irq();
        int32_t tmpDelta = (position / 4); // full rotations only
        uint32_t tmpFiltered = deltaFiltered;
        position -= tmpDelta * 4;
        __enable_irq();
        return State(tmpDelta, getMultiplier(tmpFiltered));
    }

    /**
     * @brief Get the multiplier based on the integral value
     * 
     * @param integral The integral value
     * @return uint32_t The calculated multiplier
     */
    uint32_t getMultiplier(uint32_t integral) const {
        // dead zone
        uint32_t multiplier = integral < (3 * 256 * 1024) ? 1 : (integral / (256 * 1024));
        // speed up
        if (multiplier > 18) {
            multiplier *= 6;
        }
        else if (multiplier > 12) {
            multiplier *= 4;
        }
        else if (multiplier > 6) {
            multiplier *= 2;
        }
        return multiplier;
    }
    #endif

    volatile int32_t position;
    volatile uint32_t lastTimestamp;
    #if ROTARY_ENCODER_USE_ACCELERATION
    volatile uint32_t deltaFiltered;
    #endif
    volatile uint8_t oldState;    
};
