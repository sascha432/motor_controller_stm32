/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <math.h>
#include <stdlib.h>
#include "ui_constants.h"
#include "lvgl.h"
#include "lv_custom_fonts.h"
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

#define STARTSCREEN_COLOR_START_LABEL           COLOR_PALETTE_LIGHT_GRAY
#define STARTSCREEN_COLOR_START_BG              COLOR_PALETTE_BLUE
#define STARTSCREEN_COLOR_ERROR                 COLOR_PALETTE_LIGHT_GRAY
#define STARTSCREEN_COLOR_ERROR_LABEL           COLOR_PALETTE_LIGHT_GRAY
#define STARTSCREEN_COLOR_ERROR_BG              COLOR_PALETTE_RED
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
#define DASHBOARDSCREEN_COLOR_PWM_LABEL         COLOR_PALETTE_CYAN
#define DASHBOARDSCREEN_COLOR_GRAPH_BG          COLOR_PALETTE_DARK_GRAY
#define DASHBOARDSCREEN_COLOR_GRAPH_BORDER      COLOR_PALETTE_LIGHT_GRAY
#define DASHBOARDSCREEN_COLOR_GRAPH_RPM         COLOR_PALETTE_CYAN
#define DASHBOARDSCREEN_COLOR_GRAPH_SET_RPM     COLOR_PALETTE_YELLOW

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
        PWM_FREQUENCY,
        PID_KP,
        PID_KI,
        PID_KD,
        PID_ANTI_WINDUP,
        OVP_PROTECTION,
    };

    // screen dimensions
    static constexpr const lv_coord_t kScreenWidth = LV_HOR_RES_MAX;
    static constexpr const lv_coord_t kScreenHeight = LV_VER_RES_MAX;

    // welcome screen style constants
    static constexpr const lv_font_t *kWelcomeScreenLabelFont = &lv_font_montserrat_24;

    // info screen style constants
    static constexpr const lv_font_t *kInfoScreenLabelFont = &lv_font_montserrat_24;

    // menu screen style constants
    static constexpr const lv_font_t *kMenuScreenLabelFont = &lv_font_montserrat_18;
    static constexpr lv_coord_t kMenuScreenVisibleItems = 5;
    static constexpr lv_coord_t kMenuScreenItemHeight = 26;
    static constexpr lv_coord_t kMenuScreenStartX = 10;
    static constexpr lv_coord_t kMenuScreenStartY = kScreenHeight - (kMenuScreenVisibleItems * kMenuScreenItemHeight) - 1;
    static constexpr lv_coord_t kMenuScreenItemStartX = 8;
    static constexpr lv_coord_t kMenuScreenItemStartY = 2;
    static constexpr lv_coord_t kMenuScreenItemWidth = kScreenWidth - (2 * kMenuScreenStartX);
    static constexpr uint32_t kMenuScreenItemScrollSpeed = 10;
    static constexpr lv_coord_t kMenuScreenCornerRadius = 4;

    // slider screen style constants
    static constexpr const lv_font_t *kSliderScreenLabelFont = &lv_font_montserrat_14;
    static constexpr const lv_font_t *kSliderScreenLabelFontBig = &lv_font_montserrat_18;
    static constexpr const lv_font_t *kSliderScreenValueFont = &lv_font_montserrat_18;
    static constexpr lv_coord_t kSliderScreenContainerX = 16;
    static constexpr lv_coord_t kSliderScreenContainerY = 20;
    static constexpr lv_coord_t kSliderScreenContainerWidth = kScreenWidth - 24;
    static constexpr lv_coord_t kSliderScreenContainerHeight = kScreenHeight - kSliderScreenContainerY;
    static constexpr lv_coord_t kSliderScreenTitleBottomGap = 35;
    static constexpr lv_coord_t kSliderScreenTitleAnimSpeed = 10;
    static constexpr lv_coord_t kSliderScreenSliderHeight = 24;
    static constexpr lv_coord_t kSliderScreenSliderBorder = 2;
    static constexpr lv_coord_t kSliderScreenSliderRadius = 6;
    static constexpr lv_coord_t kSliderScreenKnobSize = 30;

    // diagnostics screen style constants
    static constexpr const lv_font_t *kDiagnosticsScreenLabelFont = &lv_font_montserrat_14;
    static constexpr lv_coord_t kDiagnosticScreenViewportX = 10;
    static constexpr lv_coord_t kDiagnosticScreenViewportY = 10;
    static constexpr lv_coord_t kDiagnosticScreenMargin = 20;
    static constexpr lv_coord_t kDiagnosticScreenScrollbarWidth = 4;
    static constexpr lv_coord_t kDiagnosticScreenRowHeight = 16;
    static constexpr int32_t kDiagnosticScreenRowCount = 7;
    static constexpr lv_coord_t kDiagnosticViewportWidth = kScreenWidth - kDiagnosticScreenMargin;
    static constexpr lv_coord_t kDiagnosticViewportHeight = kScreenHeight - kDiagnosticScreenMargin;
    static constexpr lv_coord_t kDiagnosticTextWidth = kDiagnosticViewportWidth - kDiagnosticScreenScrollbarWidth - 4;

    // dashboard screen style constants
    static constexpr const lv_font_t *kDashboardScreenMetricsFont = &lv_font_dejavu_sans_mono_14;
    static constexpr const lv_font_t *kDashboardScreenSpeedFont = &lv_font_dejavu_sans_mono_24;
    static constexpr const lv_font_t *kDashboardScreenValueFixedFont = &lv_font_dejavu_sans_mono_14;
    static constexpr const lv_font_t *kDashboardScreenValueFont = &lv_font_montserrat_14;
    static constexpr lv_coord_t kDashboardScreenContainerX = 8;
    static constexpr lv_coord_t kDashboardScreenContainerY = 6;
    static constexpr lv_coord_t kDashboardScreenContainerWidth = kScreenWidth - (kDashboardScreenContainerX * 2);
    static constexpr lv_coord_t kDashboardScreenContainerHeight = kScreenHeight - (kDashboardScreenContainerY * 2);
    static constexpr lv_coord_t kDashboardScreenColumnWidth = (kDashboardScreenContainerWidth / 2) - 4;
    static constexpr lv_coord_t kDashboardScreenMotorTempOffsetY = 0;
    static constexpr lv_coord_t kDashboardScreenMosfetTempOffsetY = 18;
    static constexpr lv_coord_t kDashboardScreenRpmOffsetY = (kScreenHeight / 2) - 12;
    static constexpr lv_coord_t kDashboardScreenValueBottomOffsetY = kDashboardScreenContainerHeight - 24;
    static constexpr lv_coord_t kDashboardScreenValueTuningOffsetY = 40;
    static constexpr lv_coord_t kDashboardScreenValueLabelHeight = 14;
    static constexpr lv_coord_t kDashboardScreenGraphTopGap = 4;
    static constexpr lv_coord_t kDashboardScreenGraphX = 0;
    static constexpr lv_coord_t kDashboardScreenGraphY = kDashboardScreenValueTuningOffsetY + kDashboardScreenValueLabelHeight + kDashboardScreenGraphTopGap;
    static constexpr lv_coord_t kDashboardScreenGraphWidth = kDashboardScreenContainerWidth;
    static constexpr lv_coord_t kDashboardScreenGraphHeight = kDashboardScreenContainerHeight - kDashboardScreenGraphY - 1;
    static constexpr int32_t kDashboardScreenGraphMinRpmSpan = 250;
    static constexpr size_t kDashboardScreenGraphPointCount = 128;

    // start screen style constants
    static constexpr const lv_font_t *kStartScreenDirectionFont = &lv_font_montserrat_24;
    static constexpr const lv_font_t *kStartScreenMetricsFont = &lv_font_dejavu_sans_mono_14;
    static constexpr const lv_font_t *kStartScreenSpeedFont = &lv_font_dejavu_sans_mono_24;
    static constexpr lv_coord_t kStartScreenContainerX = 8;
    static constexpr lv_coord_t kStartScreenContainerY = 6;
    static constexpr lv_coord_t kStartScreenContainerWidth = kScreenWidth - (kStartScreenContainerX * 2);
    static constexpr lv_coord_t kStartScreenContainerHeight = kScreenHeight - (kStartScreenContainerY * 2);
    static constexpr lv_coord_t kStartScreenColumnWidth = (kStartScreenContainerWidth / 2) - 4;
    static constexpr lv_coord_t kStartScreenMotorTempOffsetY = kDashboardScreenMotorTempOffsetY;
    static constexpr lv_coord_t kStartScreenMosfetTempOffsetY = kDashboardScreenMosfetTempOffsetY;
    static constexpr lv_coord_t kStartScreenDirectionOffsetY = (kScreenHeight / 2) - 16;
    static constexpr lv_coord_t kStartScreenSpeedOffsetY = (kScreenHeight / 2) + 32;
    static constexpr lv_coord_t kStartScreenDirectionCornerRadius = 12;
    static constexpr lv_coord_t kStartScreenDirectionBorderWidth = 2;
    static constexpr lv_coord_t kStartScreenDirectionPadding = 2;

    Screen(Type id);
    virtual ~Screen();

    /**
     * @brief Get the Id object
     *
     * @return Type
     */
    Type getId() const;

    /**
     * @brief Set the value and normalize, update any visual elements on the screen to reflect the new value
     *
     * @param value
     */
    virtual void setValue(uint32_t value);

    /**
     * @brief Get the stored and normalized value
     *
     * @return uint32_t
     */
    virtual uint32_t getValue() const;

    /**
     * @brief Set the max acceleration
     *
     * @param value 1 is minimum (menus, etc...), 100000 is a high acceleration. sqrt(range) gives a good acceleration for most cases
     */
    inline void setMaxAcceleration(uint32_t value)
    {
        maxAcceleration = value;
    }

    /**
     * @brief Set the steps per change, useful to increase the steps or change direction of a rotary encoder to match the precision displayed while maintaining an internal value
     *
     * @param value negative will reverse direction
     */
    inline void setSteps(int32_t value)
    {
        steps = value;
    }

    /**
     * @brief Get steps
     *
     * @return int32_t
     */
    inline int32_t getSteps() const
    {
        return steps;
    }

    /**
     * @brief Create screen and load into LVGL
     *
     */
    virtual void load();

    /**
     * @brief Update screen contents
     *
     */
    virtual void update();

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
    InfoScreen(Type id, const char *message = nullptr, const lv_font_t *font = Screen::kInfoScreenLabelFont) :
        Screen(id),
        message(message),
        font(font)
    {}

    virtual void load() override;

protected:
    const char *message;
    const lv_font_t *font;
    lv_obj_t *label;
};

// === Welcome Screen ===

struct WelcomeScreen : public InfoScreen
{
    WelcomeScreen();

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
    inline uint32_t _first_visible_start_index(uint32_t selected_index);

protected:
    void _refreshMenuScreen();

protected:
    lv_obj_t *rows[kMenuScreenVisibleItems];
    lv_obj_t *labels[kMenuScreenVisibleItems];
    const char **itemLabels;
    uint32_t count;
    uint32_t selected;
};

// === Slider Screen ===

struct SliderScreen : public Screen
{
    typedef const char *(*FormatCallbackType)(uint32_t value, char *buf, size_t bufSize);

    SliderScreen(Type id, const char *label, uint32_t minValue, uint32_t maxValue, const char *unit, FormatCallbackType callback = nullptr, const char *zeroLabel = nullptr) :
        Screen(id),
        value(minValue),
        minValue(minValue),
        maxValue(maxValue),
        label(label),
        unit(unit),
        zeroLabel(zeroLabel),
        sliderFill(nullptr),
        sliderFillAfterActive(nullptr),
        sliderFillAfter(nullptr),
        sliderKnob(nullptr),
        valueLabel(nullptr),
        formatCallback(callback)
    {
        maxAcceleration = std::sqrt(maxValue - minValue);
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
    const char *zeroLabel;
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
    struct GraphSample {
        uint16_t rpm;
        uint16_t setRpm;
    };

    enum class SelectedValueType : uint32_t {
        SPEED,      // speed, pwm and power
        SPEED2,     // speed with tuning graph
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
        graphContainer(nullptr),
        graphRpmLine(nullptr),
        graphSetRpmLine(nullptr),
        graphWriteIndex(0),
        graphDirty(true),
        selectedValue(SelectedValueType::SPEED),
        lastSelectedValue(SelectedValueType::MAX)
    {
        maxAcceleration = eeprom.isPIDMode() ? UIConstants::kStepsRPM : UIConstants::kStepsPWM;
    }

    virtual void load() override;
    virtual void setValue(uint32_t value) override;
    virtual void update() override;

    /**
     * @brief Get the value that is selected for adjustment (speed, PID parameters, etc...)
     *
     * @return SelectedValueType
     */
    SelectedValueType getSelectedValue() const
    {
        return selectedValue;
    }

    /**
     * @brief Set the value that is selected for adjustment (speed, PID parameters, etc...)
     *
     * @param value
     */
    void setSelectedValue(SelectedValueType value)
    {
        selectedValue = value;
    }

    /**
     * @brief Increment the value that is selected for adjustment (speed, PID parameters, etc...) and wrap around to the first value if the last value is reached
     *
     * @return SelectedValueType New selected value
     */
    SelectedValueType incrSelectedValue()
    {
        selectedValue = static_cast<SelectedValueType>((static_cast<uint32_t>(selectedValue) + 1) % static_cast<uint32_t>(SelectedValueType::MAX));
        return selectedValue;
    }

    /**
     * @brief Record RPM for the graph in the ring buffer
     *
     * @param rpm
     */
    void _sampleGraph(int32_t rpm);

protected:
    void _refreshVisuals();
    void _rebuildGraphPoints();

protected:
    lv_obj_t *voltageLabel;
    lv_obj_t *currentLabel;
    lv_obj_t *motorTempLabel;
    lv_obj_t *mosfetTempLabel;
    lv_obj_t *rpmLabel;
    lv_obj_t *valueLabel;
    lv_obj_t *graphContainer;
    lv_obj_t *graphRpmLine;
    lv_obj_t *graphSetRpmLine;
    lv_point_t graphRpmPoints[kDashboardScreenGraphPointCount];
    lv_point_t graphSetRpmPoints[kDashboardScreenGraphPointCount];
    GraphSample graphSamples[kDashboardScreenGraphPointCount];
    volatile size_t graphWriteIndex;
    volatile bool graphDirty;
    SelectedValueType selectedValue;
    SelectedValueType lastSelectedValue;
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
        maxAcceleration = eeprom.isPIDMode() ? UIConstants::kStepsRPM : UIConstants::kStepsPWM;
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

#include "ui_screen_flow.h"
