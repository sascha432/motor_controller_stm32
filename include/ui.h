/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "lvgl.h"
#include "tft_driver.h"
#include <stdint.h>

// set to 1 to create previous screens using Screen::load() instead of lv_scr_load(screen->screen)
// more CPU vs memory usage tradeoff
#define RECREATE_PREV_SCREEN 0

#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

// === UI constants ===
struct UIConstants {

    static constexpr float kMinInputCurrent = 0.1f;
    static constexpr float kMaxInputCurrent = 40.0f;
    static constexpr float kMinMotorCurrent = 0.5f;
    static constexpr float kMaxMotorCurrent = 82.5f;
    static constexpr uint16_t kMinRPM = 10;
    static constexpr uint16_t kMaxRPM = 15000;
    static constexpr uint8_t kMinTFTBrightness = 5;
    static constexpr uint8_t kMaxTFTBrightness = 100;   
    static constexpr uint8_t kMinLEDBrightness = 0; 
    static constexpr uint8_t kMaxLEDBrightness = 100;
    static constexpr uint32_t kWelcomeScreenTimeout = 2000; // milliseconds

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
        CONTROL_MODE_PID
    };

    // Screen style constants
    static constexpr const lv_font_t *kWelcomeScreenLabelFont = &lv_font_montserrat_18;

    static constexpr const lv_font_t *kMenuScreenLabelFont = &lv_font_montserrat_14;
    static constexpr lv_coord_t kMenuScreenVisibleItems = 5;
    static constexpr lv_coord_t kMenuScreenStartX = 10;
    static constexpr lv_coord_t kMenuScreenStartY = 9;
    static constexpr lv_coord_t kMenuScreenItemStartX = 8;
    static constexpr lv_coord_t kMenuScreenItemStartY = 2;
    static constexpr lv_coord_t kMenuScreenItemHeight = 20;
    static constexpr lv_coord_t kMenuScreenItemWidth = TFT_DIM_WIDTH - (2 * kMenuScreenStartX);
    static constexpr uint32_t kMenuScreenItemScrollSpeed = 10;
    static constexpr uint8_t kMenuScreenCornerRadius = 4;

    static constexpr const lv_font_t *kSliderScreenLabelFont = &lv_font_montserrat_12;
    static constexpr const lv_font_t *kSliderScreenValueFont = &lv_font_montserrat_12;
    static constexpr lv_coord_t kSliderScreenContainerX = 16;
    static constexpr lv_coord_t kSliderScreenContainerY = 20;
    static constexpr lv_coord_t kSliderScreenContainerWidth = TFT_DIM_WIDTH - 24;
    static constexpr lv_coord_t kSliderScreenTitleBottomGap = 35;
    static constexpr lv_coord_t kSliderScreenSliderHeight = 12;
    static constexpr lv_coord_t kSliderScreenSliderBorder = 2;
    static constexpr lv_coord_t kSliderScreenSliderRadius = 6;
    static constexpr lv_coord_t kSliderScreenKnobSize = 18;
    static constexpr lv_coord_t kSliderScreenValueTopGap = 40;

    Screen(Type id);
    virtual ~Screen();

    Type getId() const;
    virtual void setValue(uint32_t value);
    virtual uint32_t getValue() const;

    virtual void load();

    void style_screen(lv_obj_t *screen);
    void fatal_error(const char *msg);

protected:
    lv_obj_t *screen;
    Screen *prevScreen;
    Type id;

    friend struct ScreenFlow;
    static lv_obj_t *emptyScreen;
};

// === Welcome Screen ===

struct WelcomeScreen : public Screen
{
    WelcomeScreen() : Screen(Type::WELCOME) 
    {}

    virtual void load() override;
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

