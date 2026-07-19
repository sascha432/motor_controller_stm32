/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdlib.h>
#include "ui_constants.h"
#include "lvgl.h"
#include "tft_driver.h"
#include "helpers.h"
#include "debug.h"

// set to true to keep screen objects in memory when switching screens
static constexpr bool kUIKeepScreenObjectsInMemory = false;

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
        MOTOR_SPEED,
        DIAGNOSTICS
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

    static constexpr const lv_font_t *kDiagnosticsScreenLabelFont = &lv_font_montserrat_14;

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

// === Diagnostics Screen ===

struct DiagnosticsScreen :  public Screen
{
    DiagnosticsScreen(Type id) : Screen(id),
        firmwareLabel(nullptr),
        vccLabel(nullptr),
        currentLabel(nullptr),
        motorTempLabel(nullptr),
        mosfetTempLabel(nullptr)
    {}

    virtual void load() override;

    void update() {
        _refreshVisuals();
    }

    template<uint32_t UPDATE_RATE_MILLIS = 30000>
    struct MinMax {
        uint32_t lastUpdate;
        int16_t min;
        int16_t max;

        MinMax()
        {
            reset();
        }

        void reset()
        {
            lastUpdate = HAL_GetTick();
            min = INT16_MAX;
            max = INT16_MIN;
        }

        void update(int16_t value) 
        {
            if ((HAL_GetTick() - lastUpdate) > UPDATE_RATE_MILLIS) {
                reset();
            }
            if (value < min) {
                min = value;
            }
            if (value > max) {
                max = value;
            }
        }
    };

protected:
    void _refreshVisuals();

private:
    lv_obj_t *firmwareLabel;
    lv_obj_t *vccLabel;
    lv_obj_t *currentLabel;
    lv_obj_t *motorTempLabel;
    lv_obj_t *mosfetTempLabel;
    MinMax<5000> vcc;
    MinMax<5000> current;
    MinMax<30000> motorTemp;    
    MinMax<30000> mosfetTemp;
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

