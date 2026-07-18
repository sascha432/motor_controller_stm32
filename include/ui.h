/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include "lvgl.h"
#include "tft_driver.h"
#include "debug.h"

// set to 1 to create previous screens using Screen::load() instead of lv_scr_load(screen->screen)
// more CPU vs memory usage tradeoff
#define RECREATE_PREV_SCREEN    1

#define VERSION_MAJOR           1
#define VERSION_MINOR           0
#define VERSION_PATCH           0

// === UI constants ===
struct UIConstants 
{
    // ui slider min/max values
    static constexpr float kMinInputCurrent = 0.1f;                     // min. input current in A
    static constexpr float kMaxInputCurrent = 40.0f;                    // max. input current in A
    static constexpr float kInputCurrentStep = 0.1f;                    // input current step in A (make sure min/max is divisible by this value)
    static constexpr float kMinMotorCurrent = 0.5f;                     // min. peak motor current in A
    static constexpr float kMaxMotorCurrent = 82.5f;                    // max. peak motor current in A
    static constexpr float kMotorCurrentStep = 0.5f;                    // motor current step in A (make sure min/max is divisible by this value)
    static constexpr uint16_t kMinRPM = 10;                             // Min. Motor RPM
    static constexpr uint16_t kMaxRPM = 15000;                          // Max. Motor RPM
    static constexpr uint8_t kMinTFTBrightness = 5;                     // Min. TFT Brightness
    static constexpr uint8_t kMaxTFTBrightness = 100;                   // Max. TFT Brightness
    static constexpr uint8_t kMinLEDBrightness = 0;                     // Min. LED Brightness
    static constexpr uint8_t kMaxLEDBrightness = 100;                   // Max. LED Brightness
    static constexpr uint32_t kMinMotorStallTimeout = 100;              // Motor stall time in milliseconds
    static constexpr uint32_t kMaxMotorStallTimeout = 10000;            // Motor stall time in milliseconds
    static constexpr int32_t kMotorStallTimeoutStep = 100;              // Motor stall time step in milliseconds
    static constexpr uint8_t kMinMosfetTemperature = 50;                // Min. MOSFET temperature in °C
    static constexpr uint8_t kMaxMosfetTemperature = 175;               // Max. MOSFET temperature in °C
    static constexpr uint8_t kMinMotorTemperature = 30;                 // Min. Motor temperature in °C
    static constexpr uint8_t kMaxMotorTemperature = 105;                // Max. Motor temperature in °C

    // eeprom default values
    static constexpr uint8_t kDefaultTFTBrightness = 80;                // Default TFT Brightness
    static constexpr uint8_t kDefaultLEDBrightness = 50;                // Default LED Brightness
    static constexpr float kDefaultInputCurrent = 10.0f;                // Default input current in A
    static constexpr float kDefaultMotorCurrent = 40.0f;                // Default peak motor current in A
    static constexpr uint8_t kDefaultMotorBrake = 50;                   // Default motor brake in percentage (0-100)
    static constexpr uint16_t kDefaultMotorStallTimeout = 1500;         // Default motor stall time in milliseconds
    static constexpr uint16_t kDefaultMinRPM = 10;                      // Default min. motor RPM
    static constexpr uint16_t kDefaultMaxRPM = 15000;                   // Default max. motor RPM
    static constexpr uint8_t kDefaultMosfetTemperatureLimit = 85;       // Default MOSFET temperature limit in °C
    static constexpr uint8_t kDefaultMotorTemperatureLimit = 50;        // Default motor temperature limit in °C
    static constexpr uint8_t kDefaultMaxPWM = 100;                      // Max. PWM value in percentage (0-100)
    static constexpr uint8_t kDefaultMotorPWM = 20;                     // Default motor PWM value in percentage (0-100)
    static constexpr uint16_t kDefaultMotorRPM = 250;                   // Default motor RPM value in RPM

    // ui menu timeouts
    static constexpr uint32_t kWelcomeScreenTimeout = 2000;             // WelcomeScreen timeout in milliseconds
    static constexpr uint32_t kInfoScreenTimeout = 2000;                // Default InfoScreen timeout in milliseconds
};

// === Base Screen class ===
struct Screen
{
    // Screen Type identifier
    enum class Type {
        WELCOME,
        START,
        EEPROM_SAVED,
        MAIN_MENU,
        CONTROL_MODE,
        LED_BRIGHTNESS,
        ADVANCED_MENU,
        MOTOR_RPM_SETTINGS,
        MOTOR_DIRECTION,
        TFT_BRIGHTNESS,
        CURRENT_LIMITS,
        INPUT_CURRENT_LIMIT,
        MOTOR_CURRENT_LIMIT,
        MIN_RPM,
        MAX_RPM,
        CONTROL_MODE_PWM,
        CONTROL_MODE_PID,
        MOTOR_STALL_TIMEOUT,
        MOTOR_BRAKE,
        MOSFET_TEMPERATURE_LIMIT,
        MOTOR_TEMPERATURE_LIMIT,
        RESTORE_DEFAULTS_CONFIRMATION,
        EEPROM_RESTORED,
        MOTOR_SPEED
    };

    // welcome screen style constants
    static constexpr const lv_font_t *kWelcomeScreenLabelFont = &lv_font_montserrat_18;

    // info screen style constants    
    static constexpr const lv_font_t *kInfoScreenLabelFont = &lv_font_montserrat_18;

    // menu screen style constants
    static constexpr lv_coord_t kMenuScreenVisibleItems = 5;
    #if TFT_DIM_HEIGHT == 135
        static constexpr const lv_font_t *kMenuScreenLabelFont = &lv_font_montserrat_18;
        static constexpr lv_coord_t kMenuScreenItemHeight = 26;
    #else
        static constexpr const lv_font_t *kMenuScreenLabelFont = &lv_font_montserrat_14;
        static constexpr lv_coord_t kMenuScreenItemHeight = 20;
    #endif
    static constexpr lv_coord_t kMenuScreenStartX = 10;
    static constexpr lv_coord_t kMenuScreenStartY = TFT_DIM_HEIGHT - (kMenuScreenVisibleItems * kMenuScreenItemHeight) - 1;
    static constexpr lv_coord_t kMenuScreenItemStartX = 8;
    static constexpr lv_coord_t kMenuScreenItemStartY = 2;
    static constexpr lv_coord_t kMenuScreenItemWidth = TFT_DIM_WIDTH - (2 * kMenuScreenStartX);
    static constexpr uint32_t kMenuScreenItemScrollSpeed = 10;
    static constexpr uint8_t kMenuScreenCornerRadius = 4;

    // slider screen style constants
    static constexpr const lv_font_t *kSliderScreenLabelFont = &lv_font_montserrat_14;
    static constexpr const lv_font_t *kSliderScreenValueFont = &lv_font_montserrat_18;
    static constexpr lv_coord_t kSliderScreenContainerX = 16;
    #if TFT_DIM_HEIGHT == 135
        static constexpr lv_coord_t kSliderScreenContainerY = 20;
    #else
        static constexpr lv_coord_t kSliderScreenContainerY = 27;
    #endif
    static constexpr lv_coord_t kSliderScreenContainerWidth = TFT_DIM_WIDTH - 24;
    static constexpr lv_coord_t kSliderScreenContainerHeight = TFT_DIM_HEIGHT - kSliderScreenContainerY;
    static constexpr lv_coord_t kSliderScreenTitleBottomGap = 35;
    static constexpr lv_coord_t kSliderScreenTitleAnimSpeed = 10;
    static constexpr lv_coord_t kSliderScreenSliderHeight = 24;
    static constexpr lv_coord_t kSliderScreenSliderBorder = 2;
    static constexpr lv_coord_t kSliderScreenSliderRadius = 6;
    static constexpr lv_coord_t kSliderScreenKnobSize = 30;
    static constexpr lv_coord_t kSliderScreenValueTopGap = 40;

    Screen(Type id);
    virtual ~Screen();

    Type getId() const;
    virtual void setValue(uint32_t value);
    virtual uint32_t getValue() const;

    virtual void load();

    void _style_screen(lv_obj_t *screen);
    void _fatal_error(const char *msg);

protected:
    lv_obj_t *screen;
    Screen *prevScreen;
    Type id;

    friend struct ScreenFlow;
    static lv_obj_t *emptyScreen;
};

// === Info Screen ===

struct InfoScreen : public Screen
{
    InfoScreen(Type id, const char *message, const lv_font_t *font = Screen::kInfoScreenLabelFont) : 
        Screen(id),
        message(strdup(message)),
        font(font)
    {}

    InfoScreen(Type id, const lv_font_t *font = Screen::kInfoScreenLabelFont) : 
        Screen(id),
        message(nullptr),
        font(font)
    {}

    virtual ~InfoScreen() 
    {
        free(message);
    }

    virtual void load() override;

protected:
    char *message;
    const lv_font_t *font;
};

// === Welcome Screen ===

struct WelcomeScreen : public InfoScreen
{
    WelcomeScreen();
};

// === Menu Screen ===

struct MenuScreen : public Screen
{
    MenuScreen(Type id, const char **itemLabels, size_t itemCount);

    virtual void load() override;
    virtual void setValue(uint32_t index) override;
    virtual uint32_t getValue() const override;

private:
    uint8_t _first_visible_start_index(uint8_t selected_index);
    void _style_menu_row(lv_obj_t *row, bool selected);
    void _style_menu_label(lv_obj_t *label, bool selected);
    lv_obj_t *_style_create_menu_label(lv_obj_t *parent, const char *text, bool selected);

protected:
    void _refreshMenuScreen();

protected:
    lv_obj_t *rows[kMenuScreenVisibleItems];
    lv_obj_t *labels[kMenuScreenVisibleItems];
    const char **itemLabels;
    uint8_t count;
    uint8_t selected;
};

// === Slider Screen ===

struct SliderScreen : public Screen
{
    typedef const char *(*FormatCallbackType)(uint32_t value, char *buf, size_t bufSize);

    SliderScreen(Type id, const char *label, uint32_t minValue, uint32_t maxValue, const char *unit, FormatCallbackType callback = nullptr) : 
        Screen(id),
        value(minValue),
        minValue(minValue),
        maxValue(maxValue),
        label(label),
        unit(unit),
        sliderFill(nullptr),
        sliderKnob(nullptr),
        valueLabel(nullptr),
        formatCallback(callback)
    {
    }

    virtual void load() override;
    virtual void setValue(uint32_t value) override;
    virtual uint32_t getValue() const override;
    
protected:
    void _refreshVisuals();

private:
    uint32_t value;
    uint32_t minValue;
    uint32_t maxValue;
    const char *label;
    const char *unit;
    lv_obj_t *sliderFill;
    lv_obj_t *sliderKnob;
    lv_obj_t *valueLabel;
    FormatCallbackType formatCallback;
};

// === Screen Flow Manager ===

struct ScreenFlow {

    ScreenFlow();

    void init();
    void destroy();
    void setScreen(Screen *newScreen);
    void back();
    void next(Screen *nextScreen);

    Screen *operator->() const;
    Screen *getScreen() const;

protected:
    Screen *screen;
};

