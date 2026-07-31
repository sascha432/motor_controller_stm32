/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <math.h>
#include <stdlib.h>
#include "ui_constants.h"
#include "lvgl.h"
#include "tft_driver.h"
#include "controls.h"
#include "eeprom.h"
#include "debug.h"

// set to true to keep screen objects in memory when switching screens
static constexpr bool kUIKeepScreenObjectsInMemory = false;

// === Color palette and schema ===

#define COLOR_PALETTE_WHITE                     lv_color_make(255, 255, 255)
#define COLOR_PALETTE_BLACK                     lv_color_make(0, 0, 0)
#define COLOR_PALETTE_DARK_GRAY                 lv_color_make(8, 8, 8)
#define COLOR_PALETTE_LIGHT_GRAY                lv_color_make(64, 64, 64)
#define COLOR_PALETTE_RED                       lv_color_make(255, 0, 0)
#define COLOR_PALETTE_GREEN                     lv_color_make(0, 255, 0)
#define COLOR_PALETTE_YELLOW                    lv_color_make(255, 255, 0)
#define COLOR_PALETTE_CYAN                      lv_color_make(0, 255, 255)
#define COLOR_PALETTE_PURPLE                    lv_color_make(255, 0, 255)
#define COLOR_PALETTE_DARK_PURPLE               lv_color_make(128, 0, 128)
#define COLOR_PALETTE_BLUE                      lv_color_make(0, 0, 128)

#define SCREEN_COLOR_TEXT                       COLOR_PALETTE_LIGHT_GRAY
#define SCREEN_COLOR_BG                         COLOR_PALETTE_BLACK
#define SCREEN_COLOR_VOLTAGE                    COLOR_PALETTE_GREEN
#define SCREEN_COLOR_CURRENT                    COLOR_PALETTE_YELLOW
#define SCREEN_COLOR_TEMPERATURE                COLOR_PALETTE_CYAN

#define INFOSCREEN_COLOR_TEXT                   SCREEN_COLOR_TEXT

#define DIAGNOSTICSCREEN_COLOR_TEXT             COLOR_PALETTE_CYAN
#define DIAGNOSTICSCREEN_COLOR_SCROLLBAR_BG     COLOR_PALETTE_DARK_GRAY
#define DIAGNOSTICSCREEN_COLOR_SCROLLBAR_THUMB  COLOR_PALETTE_LIGHT_GRAY

#define STARTSCREEN_COLOR_START_LABEL           SCREEN_COLOR_TEXT
#define STARTSCREEN_COLOR_SPEED                 COLOR_PALETTE_PURPLE
#define STARTSCREEN_COLOR_VOLTAGE               SCREEN_COLOR_VOLTAGE
#define STARTSCREEN_COLOR_CURRENT               SCREEN_COLOR_CURRENT
#define STARTSCREEN_COLOR_TEMPERATURE           SCREEN_COLOR_TEMPERATURE

#define MENUSCREEN_COLOR_ITEM                   COLOR_PALETTE_LIGHT_GRAY
#define MENUSCREEN_COLOR_BG                     COLOR_PALETTE_BLACK
#define MENUSCREEN_COLOR_SELECTED_ITEM          COLOR_PALETTE_LIGHT_GRAY
#define MENUSCREEN_COLOR_SELECTED_BG            COLOR_PALETTE_BLUE

#define DASHBOARDSCREEN_COLOR_VOLTAGE           SCREEN_COLOR_VOLTAGE
#define DASHBOARDSCREEN_COLOR_CURRENT           SCREEN_COLOR_CURRENT
#define DASHBOARDSCREEN_COLOR_TEMPERATURE       SCREEN_COLOR_TEMPERATURE
#define DASHBOARDSCREEN_COLOR_SPEED             COLOR_PALETTE_PURPLE
#define DASHBOARDSCREEN_COLOR_ERROR             COLOR_PALETTE_RED
#define DASHBOARDSCREEN_COLOR_PWM_LABEL         COLOR_PALETTE_CYAN

#define SLIDERSCREEN_COLOR_LABEL                SCREEN_COLOR_TEXT
#define SLIDERSCREEN_COLOR_VALUE                COLOR_PALETTE_CYAN
#define SLIDERSCREEN_COLOR_SLIDER_BG            COLOR_PALETTE_BLACK
#define SLIDERSCREEN_COLOR_SLIDER_FILL          COLOR_PALETTE_DARK_GRAY
#define SLIDERSCREEN_COLOR_SLIDER_FILL_ACTIVE   COLOR_PALETTE_BLUE
#define SLIDERSCREEN_COLOR_SLIDER_BORDER        COLOR_PALETTE_BLACK
#define SLIDERSCREEN_COLOR_SLIDER_KNOB          COLOR_PALETTE_BLUE
#define SLIDERSCREEN_COLOR_SLIDER_KNOB_BORDER   COLOR_PALETTE_BLACK

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
        SENSOR_DIRECTION,
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
        DIAGNOSTICS,
        DASHBOARD,
        PID_PARAMETERS,
        PID_KP,
        PID_KI,
        PID_KD,
        PID_ANTI_WINDUP,
        OVP_PROTECTION,
    };

    // welcome screen style constants
    static constexpr const lv_font_t *kWelcomeScreenLabelFont = &lv_font_montserrat_24;

    // info screen style constants
    static constexpr const lv_font_t *kInfoScreenLabelFont = &lv_font_montserrat_24;

    // menu screen style constants
    static constexpr lv_coord_t kMenuScreenVisibleItems = 5;
    static constexpr const lv_font_t *kMenuScreenLabelFont = &lv_font_montserrat_18;
    static constexpr lv_coord_t kMenuScreenItemHeight = 26;
    static constexpr lv_coord_t kMenuScreenStartX = 10;
    static constexpr lv_coord_t kMenuScreenStartY = TFT_DIM_HEIGHT - (kMenuScreenVisibleItems * kMenuScreenItemHeight) - 1;
    static constexpr lv_coord_t kMenuScreenItemStartX = 8;
    static constexpr lv_coord_t kMenuScreenItemStartY = 2;
    static constexpr lv_coord_t kMenuScreenItemWidth = TFT_DIM_WIDTH - (2 * kMenuScreenStartX);
    static constexpr uint32_t kMenuScreenItemScrollSpeed = 10;
    static constexpr uint8_t kMenuScreenCornerRadius = 4;

    // slider screen style constants
    static constexpr const lv_font_t *kSliderScreenLabelFont = &lv_font_montserrat_14;
    static constexpr const lv_font_t *kSliderScreenLabelFontBig = &lv_font_montserrat_18;
    static constexpr const lv_font_t *kSliderScreenValueFont = &lv_font_montserrat_18;
    static constexpr lv_coord_t kSliderScreenContainerX = 16;
    static constexpr lv_coord_t kSliderScreenContainerY = 20;
    static constexpr lv_coord_t kSliderScreenContainerWidth = TFT_DIM_WIDTH - 24;
    static constexpr lv_coord_t kSliderScreenContainerHeight = TFT_DIM_HEIGHT - kSliderScreenContainerY;
    static constexpr lv_coord_t kSliderScreenTitleBottomGap = 35;
    static constexpr lv_coord_t kSliderScreenTitleAnimSpeed = 10;
    static constexpr lv_coord_t kSliderScreenSliderHeight = 24;
    static constexpr lv_coord_t kSliderScreenSliderBorder = 2;
    static constexpr lv_coord_t kSliderScreenSliderRadius = 6;
    static constexpr lv_coord_t kSliderScreenKnobSize = 30;
    static constexpr lv_coord_t kSliderScreenValueTopGap = 40;

    // diagnostics screen style constants
    static constexpr const lv_font_t *kDiagnosticsScreenLabelFont = &lv_font_montserrat_14;
    static constexpr lv_coord_t kDiagnosticScreenViewportX = 10;
    static constexpr lv_coord_t kDiagnosticScreenViewportY = 10;
    static constexpr lv_coord_t kDiagnosticScreenMargin = 20;
    static constexpr lv_coord_t kDiagnosticScreenScrollbarWidth = 4;
    static constexpr lv_coord_t kDiagnosticScreenRowHeight = 16;
    static constexpr int32_t kDiagnosticScreenRowCount = 7;

    static constexpr lv_coord_t kDashboardScreenContainerWidth = TFT_DIM_WIDTH - 16;
    static constexpr lv_coord_t kDashboardScreenContainerHeight = TFT_DIM_HEIGHT - 12;
    static constexpr lv_coord_t kDashboardScreenColumnWidth = (kDashboardScreenContainerWidth / 2) - 4;
    static constexpr const lv_font_t *kDashboardScreenFont = &lv_font_montserrat_14;
    static constexpr const lv_font_t *kDashboardScreenBigFont = &lv_font_montserrat_24;

    Screen(Type id);
    virtual ~Screen();

    Type getId() const;
    virtual void setValue(uint32_t value);
    virtual uint32_t getValue() const;

    inline void setMaxAcceleration(uint32_t value)
    {
        maxAcceleration = value;
    }

    inline void setSteps(int32_t value)
    {
        steps = value;
    }

    inline int32_t getSteps() const
    {
        return steps;
    }

    virtual void load();
    virtual void update();

    void _style_screen(lv_obj_t *screen);
    void _fatal_error(const char *msg);

protected:
    friend struct ScreenFlow;

    lv_obj_t *screen;
    Screen *prevScreen; // linked list to previous screen for back navigation
    Type id;
    uint32_t maxAcceleration;
    int32_t steps;
    int32_t value;

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
    void setMessage(const char *message)
    {
        if (message) {
            lv_label_set_text(label, message);
        }
    }

protected:
    char *message;
    const lv_font_t *font;
    lv_obj_t *label;
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
        sliderFillAfterActive(nullptr),
        sliderFillAfter(nullptr),
        sliderKnob(nullptr),
        valueLabel(nullptr),
        formatCallback(callback)
    {
        maxAcceleration = sqrt(maxValue - minValue);
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
    lv_obj_t *sliderFillAfterActive;
    lv_obj_t *sliderFillAfter;
    lv_obj_t *sliderKnob;
    lv_obj_t *valueLabel;
    FormatCallbackType formatCallback;
};

// === PID Slider Screen ===

struct PidSliderScreen : public SliderScreen
{
    PidSliderScreen(Type id, const char *label, uint32_t minValue, uint32_t maxValue, FormatCallbackType callback, int32_t steps) :
        SliderScreen(id, label, minValue, maxValue, "", callback)
    {
        maxAcceleration = UIConstants::kMaxPIDParamAcceleration;
        setSteps(steps);
    }
};

// === Diagnostics Screen ===

struct DiagnosticsScreen :  public Screen
{
    DiagnosticsScreen(Type id) : Screen(id),
        viewport(nullptr),
        content(nullptr),
        firmwareLabel(nullptr),
        vccLabel(nullptr),
        currentLabel(nullptr),
        motorTempLabel(nullptr),
        mosfetTempLabel(nullptr),
        rpmPwmLabel(nullptr),
        lastErrorLabel(nullptr),
        scrollbarTrack(nullptr),
        scrollbarThumb(nullptr),
        scrollOffset(0),
        scrollMax(0)
    {
        setMaxAcceleration(1);
        setSteps(-1);
    }

    virtual void load() override;
    virtual void setValue(uint32_t value) override;
    virtual void update() override;

protected:
    void _refreshVisuals();

private:
    lv_obj_t *viewport;
    lv_obj_t *content;
    lv_obj_t *firmwareLabel;
    lv_obj_t *vccLabel;
    lv_obj_t *currentLabel;
    lv_obj_t *motorTempLabel;
    lv_obj_t *mosfetTempLabel;
    lv_obj_t *rpmPwmLabel;
    lv_obj_t *lastErrorLabel;
    lv_obj_t *scrollbarTrack;
    lv_obj_t *scrollbarThumb;
    int32_t scrollOffset;
    int32_t scrollMax;
};

// === Dashboard Screen ===

struct DashboardScreen : public Screen
{
    enum class SelectedValueType : uint32_t {
        SPEED,
        KP,
        KI,
        KD,
        ANTI_WINDUP,
        MAX
    };

    DashboardScreen(Type id = Screen::Type::DASHBOARD) :
        Screen(id),
        voltageLabel(nullptr),
        currentLabel(nullptr),
        motorTempLabel(nullptr),
        mosfetTempLabel(nullptr),
        rpmLabel(nullptr),
        valueLabel(nullptr),
        selectedValue(SelectedValueType::SPEED)
    {
        maxAcceleration = eeprom.isPIDMode() ? 50 : 1;
    }

    void update() {
        _refreshVisuals();
    }

    virtual void load() override;

    SelectedValueType getSelectedValue() const
    {
        return selectedValue;
    }

    void setSelectedValue(SelectedValueType value)
    {
        selectedValue = value;
    }

    SelectedValueType incrSelectedValue()
    {
        selectedValue = static_cast<SelectedValueType>((static_cast<uint32_t>(selectedValue) + 1) % static_cast<uint32_t>(SelectedValueType::MAX));
        return selectedValue;
    }

protected:
    void _refreshVisuals();

protected:
    lv_obj_t *voltageLabel;
    lv_obj_t *currentLabel;
    lv_obj_t *motorTempLabel;
    lv_obj_t *mosfetTempLabel;
    lv_obj_t *rpmLabel;
    lv_obj_t *valueLabel;
    SelectedValueType selectedValue;
};

// === Start Screen ===

struct StartScreen : public Screen
{
    StartScreen() :
        Screen(Screen::Type::START),
        voltageLabel(nullptr),
        currentLabel(nullptr),
        motorTempLabel(nullptr),
        mosfetTempLabel(nullptr),
        directionLabel(nullptr),
        speedLabel(nullptr)
    {
        maxAcceleration = eeprom.isPIDMode() ? 50 : 1;
    }

    virtual void load() override;
    virtual void update() override;

protected:
    void _refreshVisuals();

private:
    lv_obj_t *voltageLabel;
    lv_obj_t *currentLabel;
    lv_obj_t *motorTempLabel;
    lv_obj_t *mosfetTempLabel;
    lv_obj_t *directionLabel;
    lv_obj_t *speedLabel;
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
