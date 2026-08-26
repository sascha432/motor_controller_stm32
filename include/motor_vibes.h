/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#if HAVE_MOTOR_VIBES

#include <stdint.h>

/**
 * @brief Play tones using the motor driver
 *
 */
struct MotorVibes
{
    static constexpr uint32_t kTonePeriod = 21;     // period optimal for 50-1600Hz tones
    static constexpr uint32_t kPWMDivider = 24;     // 4.2% duty cycle should prevent the motor from spinning even at low PWM frequencies, reduce if the motor has less friction

    /**
     * @brief Initialize TIM1 to play tones
     *
     */
    void init();

    /**
     * @brief Deinitialize player and restore previous settings
     *
     */
    void deinit();

    /**
     * @brief Play tone with specified frequency
     *
     * @param frequency
     */
    void playTone(uint32_t frequency);

    /**
     * @brief Stop playing tone
     *
     */
    void stopTone();

private:
    uint16_t prescaler;
    uint16_t period;
    uint16_t arr;
    uint16_t motorCurrentLimit;
    uint16_t inputCurrentLimit;
};

#else
    // set both macros to 0 if not enabled
    #undef HAVE_MOTOR_VIBES
    #undef HAVE_IMPERIAL_MARCH
    #define HAVE_MOTOR_VIBES 0
    #define HAVE_IMPERIAL_MARCH 0
#endif
