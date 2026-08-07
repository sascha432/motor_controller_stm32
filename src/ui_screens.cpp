/**
  Author: sascha_lammers@gmx.de

  UI implementation using LVGL library
*/

#include "ui.h"
#include "adc.h"
#include "pid_controller.h"
#include "stats.h"
#include "controls.h"
#include "menu.h"

// === Helpers ===

void screen_style_screen(lv_obj_t *screen)
{
    lv_obj_set_style_bg_color(screen, SCREEN_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(screen, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN);
}

inline constexpr lv_coord_t diagnostic_screen_get_ypos_for_row(int32_t row)
{
    switch(row) {
        case 0: return 0;
        default:
            break;
    }
    return Screen::kDiagnosticScreenRowHeight * row + (Screen::kDiagnosticScreenRowHeight * 3);
}

inline constexpr int32_t diagnostic_screen_content_height()
{
    return diagnostic_screen_get_ypos_for_row(Screen::kDiagnosticScreenRowCount) + 10;
}

inline int32_t diagnostic_screen_scroll_max_lines(int32_t viewportHeight)
{
    const int32_t hiddenPixels = std::max<int32_t>(0, diagnostic_screen_content_height() - viewportHeight);
    return hiddenPixels / Screen::kDiagnosticScreenRowHeight;
}

lv_obj_t *diagnostic_screen_create_label(lv_obj_t *parent, lv_coord_t width, int32_t row)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_color(label, DIAGNOSTICSCREEN_COLOR_TEXT, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, Screen::kDiagnosticsScreenLabelFont, LV_PART_MAIN);
    lv_obj_set_pos(label, 0, diagnostic_screen_get_ypos_for_row(row));
    lv_obj_set_width(label, width);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    return label;
}

inline void diagnostic_screen_set_label_row(lv_obj_t *label, int32_t row, int32_t scrollOffset)
{
    lv_obj_set_y(label, diagnostic_screen_get_ypos_for_row(row) - scrollOffset);
}

void start_screen_update_top_status_labels(lv_obj_t *voltageLabel, lv_obj_t *currentLabel, lv_obj_t *motorTempLabel, lv_obj_t *mosfetTempLabel)
{
    char buf[32];

    snprintf(buf, sizeof(buf) - 1, "%u.%uV (%u.%uV)", CONVERT_TO_FP1(stats.vcc), CONVERT_TO_FP1(stats.max.vcc));
    lv_label_set_text(voltageLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "%u.%uA (%u.%uA)", CONVERT_TO_FP1(stats.current), CONVERT_TO_FP1(stats.max.current));
    lv_label_set_text(currentLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "%d" DEGREE_UTF8 "C", stats.motorTemp);
    lv_label_set_text(motorTempLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "%d" DEGREE_UTF8 "C", stats.mosfetTemp);
    lv_label_set_text(mosfetTempLabel, buf);
}

void menuscreen_style_menu_row(lv_obj_t *row, bool selected)
{
    lv_obj_set_size(row, Screen::kMenuScreenItemWidth, Screen::kMenuScreenItemHeight);
    lv_obj_set_style_radius(row, Screen::kMenuScreenCornerRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row, selected ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_color(row, selected ? MENUSCREEN_COLOR_SELECTED_BG : MENUSCREEN_COLOR_BG, LV_PART_MAIN);
}

void menuscreen_style_menu_label(lv_obj_t *label, bool selected)
{
    lv_obj_set_style_text_color(label, selected ? MENUSCREEN_COLOR_SELECTED_ITEM : MENUSCREEN_COLOR_ITEM, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, Screen::kMenuScreenLabelFont, LV_PART_MAIN);
    lv_label_set_long_mode(label, selected ? LV_LABEL_LONG_SCROLL_CIRCULAR : LV_LABEL_LONG_CLIP);
}

lv_obj_t *menuscreen_create_menu_label(lv_obj_t *parent, const char *text, bool selected)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text_static(label, text);
    menuscreen_style_menu_label(label, selected);
    // row boundaries and settings for label/clipping scrolling
    lv_obj_set_style_anim_speed(label, Screen::kMenuScreenItemScrollSpeed, LV_PART_MAIN);
    lv_obj_set_width(label, Screen::kMenuScreenItemWidth - (2 * Screen::kMenuScreenItemStartX));
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    // position in the row
    lv_obj_set_pos(label, Screen::kMenuScreenItemStartX, Screen::kMenuScreenItemStartY);
    return label;
}

// === Base Screen ===

lv_obj_t *Screen::emptyScreen = nullptr;

Screen::Screen(Type id) :
    screen(nullptr),
    prevScreen(nullptr),
    id(id),
    maxAcceleration(1),
    steps(1),
    value(0)
{
    DEBUG_PRINT(DebugType::UI, "ctor");
}

Screen::~Screen()
{
    DEBUG_PRINT(DebugType::UI, "screen=%p", screen);
     if (screen) {
        lv_scr_load(emptyScreen);
        lv_obj_del(screen);
    }
}

void Screen::load()
{
    DEBUG_PRINT(DebugType::UI, "screen=%p", screen);
    if (screen) {
        auto tmp = screen;
        screen = lv_obj_create(nullptr);
        lv_scr_load(emptyScreen);
        lv_obj_del(tmp);
    }
    else {
        screen = lv_obj_create(nullptr);
    }
    screen_style_screen(screen);
    knob.setMaxAcceleration(maxAcceleration);
}

void Screen::update()
{
    DEBUG_PRINT(DebugType::UI, "screen=%p", screen);
}

Screen::Type Screen::getId() const
{
    return id;
}

void Screen::setValue(uint32_t value)
{
    this->value = value;
}

uint32_t Screen::getValue() const
{
    return value;
}

// === Welcome Screen ===

WelcomeScreen::WelcomeScreen() : InfoScreen(Type::WELCOME, nullptr, kWelcomeScreenLabelFont)
{
}

void WelcomeScreen::load()
{
    InfoScreen::load();
    lv_label_set_text_static(label, "Version " STR(VERSION_MAJOR) "." STR(VERSION_MINOR) "." STR(VERSION_PATCH));
}

// === Info Screen ===

void InfoScreen::load()
{
    DEBUG_PRINT(DebugType::UI, "message=%s", message ? message : "<NULL>");
    Screen::load();
    label = lv_label_create(screen);
    lv_label_set_text_static(label, message);
    lv_obj_set_style_text_color(label, INFOSCREEN_COLOR_TEXT, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    lv_obj_center(label);
    lv_scr_load(screen);
}

// === Menu Screen ===

MenuScreen::MenuScreen(Type id, const char **itemLabels, size_t itemCount) :
    Screen(id),
    rows{},
    labels{},
    itemLabels(itemLabels),
    count(itemCount),
    selected(0)
{
    steps = -1; // invert for menus
}

void MenuScreen::load()
{
    DEBUG_PRINT(DebugType::UI, "items=%u selected=%u", count, selected);
    Screen::load();
    lv_obj_t *menu = lv_obj_create(screen);

    lv_obj_remove_style_all(menu);
    lv_obj_set_pos(menu, kMenuScreenStartX, kMenuScreenStartY);
    lv_obj_set_size(menu, kMenuScreenItemWidth, kMenuScreenVisibleItems * kMenuScreenItemHeight);
    lv_obj_set_style_bg_opa(menu, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(menu, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(menu, 0, LV_PART_MAIN);
    lv_obj_set_style_clip_corner(menu, true, LV_PART_MAIN);

    for (uint32_t i = 0; i < kMenuScreenVisibleItems && i < count; ++i) {
        rows[i] = lv_obj_create(menu);
        lv_obj_remove_style_all(rows[i]);
        menuscreen_style_menu_row(rows[i], false);
        lv_obj_set_pos(rows[i], 0, i * Screen::kMenuScreenItemHeight);
        labels[i] = menuscreen_create_menu_label(rows[i], itemLabels[i], false);
    }

    _refreshMenuScreen();
    lv_scr_load(screen);
}

inline uint32_t MenuScreen::_first_visible_start_index(uint32_t selected_index)
{
    return static_cast<uint32_t>(
        std::clamp<int32_t>(
            selected_index - 2,
            0,
            (count > MenuScreen::kMenuScreenVisibleItems) ? (count - MenuScreen::kMenuScreenVisibleItems) : 0
        )
    );
}

void MenuScreen::_refreshMenuScreen()
{
    const uint32_t first_index = _first_visible_start_index(selected);
    for (uint32_t i = 0; i < kMenuScreenVisibleItems && i < count; ++i) {
        const uint32_t item_index = first_index + i;
        const bool selected = (item_index == this->selected);
        menuscreen_style_menu_row(rows[i], selected);
        lv_label_set_text_static(labels[i], itemLabels[item_index]);
        menuscreen_style_menu_label(labels[i], selected);
    }
}

void MenuScreen::setValue(uint32_t index)
{
    #if 0
        // allows negative values to wrap around the menu items
        selected = (((int32_t)index % count) + count) % count;
    #else
        // no wrapping
        selected = std::clamp<int32_t>(index, 0, count - 1);
    #endif
    _refreshMenuScreen();
}

uint32_t MenuScreen::getValue() const
{
    return selected;
}

// === Slider Screen ===

void SliderScreen::load()
{
    DEBUG_PRINT(DebugType::UI, "range=%u-%u value=%u label=%s unit=%s", minValue, maxValue, value, label, unit ? unit : "<NULL>");
    Screen::load();

    lv_obj_t *container = lv_obj_create(screen);
    lv_obj_remove_style_all(container);
    lv_obj_set_pos(container, kSliderScreenContainerX, kSliderScreenContainerY);
    lv_obj_set_size(container, kSliderScreenContainerWidth, kSliderScreenContainerHeight);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);

    auto titleFont = (lv_txt_get_width(label, strlen(label), kSliderScreenLabelFontBig, 0, LV_TEXT_FLAG_NONE) < kSliderScreenContainerWidth) ?
        kSliderScreenLabelFontBig :
        kSliderScreenLabelFont;

    lv_obj_t *titleRow = lv_obj_create(container);
    lv_obj_remove_style_all(titleRow);
    lv_obj_set_size(titleRow, kSliderScreenContainerWidth, lv_font_get_line_height(titleFont));
    lv_obj_set_style_bg_opa(titleRow, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(titleRow, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(titleRow, 0, LV_PART_MAIN);
    lv_obj_set_style_clip_corner(titleRow, true, LV_PART_MAIN);
    lv_obj_set_pos(titleRow, 0, 0);

    lv_obj_t *titleObj = lv_label_create(titleRow);
    lv_label_set_text_static(titleObj, label);
    lv_obj_set_style_text_color(titleObj, SLIDERSCREEN_COLOR_LABEL, LV_PART_MAIN);
    lv_obj_set_style_text_font(titleObj, titleFont, LV_PART_MAIN);
    lv_obj_set_width(titleObj, kSliderScreenContainerWidth);
    lv_obj_set_style_text_align(titleObj, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_style_anim_speed(titleObj, kSliderScreenTitleAnimSpeed, LV_PART_MAIN);
    lv_label_set_long_mode(titleObj, LV_LABEL_LONG_CLIP);
    lv_obj_set_pos(titleObj, 0, 0);
    lv_label_set_long_mode(titleObj, LV_LABEL_LONG_SCROLL_CIRCULAR);

    const lv_coord_t sliderVisualHeight = std::max<lv_coord_t>(kSliderScreenSliderHeight, kSliderScreenKnobSize + 4);
    const lv_coord_t sliderFillHeight = std::max<lv_coord_t>(6, kSliderScreenSliderHeight - 8);

    lv_obj_t *slider = lv_obj_create(container);
    lv_obj_remove_style_all(slider);
    lv_obj_set_pos(slider, 0, lv_obj_get_height(titleRow) + kSliderScreenTitleBottomGap);
    lv_obj_set_size(slider, kSliderScreenContainerWidth, sliderVisualHeight);
    lv_obj_set_style_bg_color(slider, SLIDERSCREEN_COLOR_SLIDER_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(slider, kSliderScreenSliderRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(slider, kSliderScreenSliderBorder, LV_PART_MAIN);
    lv_obj_set_style_border_color(slider, SLIDERSCREEN_COLOR_SLIDER_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(slider, 0, LV_PART_MAIN);

    sliderFill = lv_obj_create(slider);
    lv_obj_remove_style_all(sliderFill);
    lv_obj_set_pos(sliderFill, 0, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFill, 0, sliderFillHeight);
    lv_obj_set_style_bg_color(sliderFill, SLIDERSCREEN_COLOR_SLIDER_FILL_ACTIVE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sliderFill, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(sliderFill, kSliderScreenSliderRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(sliderFill, 0, LV_PART_MAIN);

    sliderFillAfterActive = lv_obj_create(slider);
    lv_obj_remove_style_all(sliderFillAfterActive);
    lv_obj_set_pos(sliderFillAfterActive, 0, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFillAfterActive, 0, sliderFillHeight);
    lv_obj_set_style_bg_color(sliderFillAfterActive, SLIDERSCREEN_COLOR_SLIDER_FILL_ACTIVE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sliderFillAfterActive, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(sliderFillAfterActive, kSliderScreenSliderRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(sliderFillAfterActive, 0, LV_PART_MAIN);

    sliderFillAfter = lv_obj_create(slider);
    lv_obj_remove_style_all(sliderFillAfter);
    lv_obj_set_pos(sliderFillAfter, 0, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFillAfter, 0, sliderFillHeight);
    lv_obj_set_style_bg_color(sliderFillAfter, SLIDERSCREEN_COLOR_SLIDER_FILL, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sliderFillAfter, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(sliderFillAfter, kSliderScreenSliderRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(sliderFillAfter, 0, LV_PART_MAIN);

    sliderKnob = lv_obj_create(slider);
    lv_obj_remove_style_all(sliderKnob);
    lv_obj_set_size(sliderKnob, kSliderScreenKnobSize, kSliderScreenKnobSize);
    lv_obj_set_style_bg_color(sliderKnob, SLIDERSCREEN_COLOR_SLIDER_KNOB, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sliderKnob, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(sliderKnob, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(sliderKnob, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(sliderKnob, SLIDERSCREEN_COLOR_SLIDER_KNOB_BORDER, LV_PART_MAIN);
    lv_obj_set_pos(sliderKnob, 0, (sliderVisualHeight - kSliderScreenKnobSize) / 2);

    valueLabel = lv_label_create(container);
    lv_obj_set_style_text_color(valueLabel, SLIDERSCREEN_COLOR_VALUE, LV_PART_MAIN);
    lv_obj_set_style_text_font(valueLabel, kSliderScreenValueFont, LV_PART_MAIN);
    lv_obj_set_width(valueLabel, kSliderScreenContainerWidth);
    lv_obj_set_style_text_align(valueLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align_to(valueLabel, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

    _refreshVisuals();
    lv_scr_load(screen);
}

void SliderScreen::_refreshVisuals()
{
    const uint32_t range = std::max<int32_t>(maxValue - minValue, 1);
    const uint32_t percent = ((value - minValue) * 100U) / range;
    const lv_coord_t sliderVisualHeight = std::max<lv_coord_t>(kSliderScreenSliderHeight, kSliderScreenKnobSize + 4);
    const lv_coord_t sliderFillHeight = std::max<lv_coord_t>(6, kSliderScreenSliderHeight - 8);

    const lv_coord_t knobTravel = std::max<lv_coord_t>(0, kSliderScreenContainerWidth - kSliderScreenKnobSize);
    lv_coord_t knobX = static_cast<lv_coord_t>((static_cast<uint32_t>(knobTravel) * percent) / 100U);

    const lv_coord_t fillWidth = std::min<lv_coord_t>(kSliderScreenContainerWidth, knobX + (kSliderScreenKnobSize / 2));
    lv_obj_set_pos(sliderFill, 0, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFill, fillWidth, sliderFillHeight);

    const lv_coord_t knobRight = std::min<lv_coord_t>(kSliderScreenContainerWidth, knobX + kSliderScreenKnobSize);
    const lv_coord_t trackWidth = kSliderScreenContainerWidth;

    lv_obj_set_x(sliderKnob, knobX - 1);

    // Desired rendering: blue active segment, then knob, then gray remainder.
    const lv_coord_t afterStart = knobRight;
    const lv_coord_t remaining = std::max<lv_coord_t>(0, trackWidth - afterStart);
    const lv_coord_t activeAfterWidth = 0;

    lv_obj_set_pos(sliderFillAfterActive, afterStart, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFillAfterActive, activeAfterWidth, sliderFillHeight);

    lv_obj_set_pos(sliderFillAfter, afterStart + activeAfterWidth - 8, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFillAfter, remaining + 8 - 2, sliderFillHeight);

    if (formatCallback) {
        char buf[64];
        lv_label_set_text(valueLabel, formatCallback(value, buf, sizeof(buf) - 1));
    }
    else {
        if (zeroLabel && value == 0) {
            lv_label_set_text_static(valueLabel, zeroLabel);
        }
        else {
            lv_label_set_text_fmt(valueLabel, "%u%s", static_cast<unsigned>(value), unit);
        }
    }
}

void SliderScreen::setValue(uint32_t value)
{
    this->value = std::clamp<int32_t>(value, minValue, maxValue);
    _refreshVisuals();
}

uint32_t SliderScreen::getValue() const
{
    return value;
}

// === Diagnostics Screen ===

void DiagnosticsScreen::load()
{
    Screen::load();

    viewport = lv_obj_create(screen);
    lv_obj_remove_style_all(viewport);
    lv_obj_set_pos(viewport, kDiagnosticScreenViewportX, kDiagnosticScreenViewportY);
    lv_obj_set_size(viewport, kDiagnosticViewportWidth, kDiagnosticViewportHeight);
    lv_obj_set_style_bg_opa(viewport, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(viewport, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(viewport, 0, LV_PART_MAIN);

    content = viewport;

    scrollbarTrack = lv_obj_create(viewport);
    lv_obj_remove_style_all(scrollbarTrack);
    lv_obj_set_size(scrollbarTrack, kDiagnosticScreenScrollbarWidth, kDiagnosticViewportHeight);
    lv_obj_set_pos(scrollbarTrack, kDiagnosticViewportWidth - kDiagnosticScreenScrollbarWidth, 0);
    lv_obj_set_style_bg_color(scrollbarTrack, DIAGNOSTICSCREEN_COLOR_SCROLLBAR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scrollbarTrack, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(scrollbarTrack, 2, LV_PART_MAIN);

    scrollbarThumb = lv_obj_create(scrollbarTrack);
    lv_obj_remove_style_all(scrollbarThumb);
    lv_obj_set_size(scrollbarThumb, kDiagnosticScreenScrollbarWidth, 16);
    lv_obj_set_pos(scrollbarThumb, 0, 0);
    lv_obj_set_style_bg_color(scrollbarThumb, DIAGNOSTICSCREEN_COLOR_SCROLLBAR_THUMB, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scrollbarThumb, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(scrollbarThumb, 2, LV_PART_MAIN);

    firmwareLabel = diagnostic_screen_create_label(content, kDiagnosticTextWidth, 0);
    lv_label_set_text_fmt(firmwareLabel,
        "Firmware " STR(VERSION_MAJOR) "." STR(VERSION_MINOR) "." STR(VERSION_PATCH) " " __DEBUG__BUILD__ "\n"
        "PCB Rev " STR(PCB_REV_MAJOR) "." STR(PCB_REV_MINOR) "\n"
        "Build " __DATE__ " " __TIME__ "\n"
        "EEPROM cycle #%u", (unsigned)eeprom.getData().sequence
    );

    vccLabel = diagnostic_screen_create_label(content, kDiagnosticTextWidth, 1);
    currentLabel = diagnostic_screen_create_label(content, kDiagnosticTextWidth, 2);
    motorTempLabel = diagnostic_screen_create_label(content, kDiagnosticTextWidth, 3);
    mosfetTempLabel = diagnostic_screen_create_label(content, kDiagnosticTextWidth, 4);
    rpmPwmLabel = diagnostic_screen_create_label(content, kDiagnosticTextWidth, 5);
    lastErrorLabel = diagnostic_screen_create_label(content, kDiagnosticTextWidth, 6);

    scrollMax = diagnostic_screen_scroll_max_lines(kDiagnosticViewportHeight);
    scrollOffset = 0;

    lv_scr_load(screen); // update is using lv_obj_get_height() etc, load screen first
    update();
}

void DiagnosticsScreen::setValue(uint32_t value)
{
    scrollMax = diagnostic_screen_scroll_max_lines(lv_obj_get_height(viewport));
    this->value = std::clamp<int32_t>(value, 0, scrollMax);
}

void DiagnosticsScreen::update()
{
    _refreshVisuals();

    scrollMax = diagnostic_screen_scroll_max_lines(lv_obj_get_height(viewport));
    const int32_t scrollLine = std::clamp<int32_t>(value, 0, scrollMax);
    scrollOffset = scrollLine * Screen::kDiagnosticScreenRowHeight;

    // Keep labels in one container and scroll in full line-height steps.
    diagnostic_screen_set_label_row(firmwareLabel, 0, scrollOffset);
    diagnostic_screen_set_label_row(vccLabel, 1, scrollOffset);
    diagnostic_screen_set_label_row(currentLabel, 2, scrollOffset);
    diagnostic_screen_set_label_row(motorTempLabel, 3, scrollOffset);
    diagnostic_screen_set_label_row(mosfetTempLabel, 4, scrollOffset);
    diagnostic_screen_set_label_row(rpmPwmLabel, 5, scrollOffset);
    diagnostic_screen_set_label_row(lastErrorLabel, 6, scrollOffset);

    const int32_t trackHeightRaw = lv_obj_get_height(scrollbarTrack);
    const int32_t trackHeight = std::max<int32_t>(trackHeightRaw, 10);

    // Line-based scrollbar: thumb size and movement are based on scroll steps.
    const int32_t stepCount = scrollMax + 1;
    int32_t thumbHeight = (stepCount > 0) ? (trackHeight / stepCount) : trackHeight;
    thumbHeight = std::clamp<int32_t>(thumbHeight, 8, trackHeight);
    lv_obj_set_height(scrollbarThumb, thumbHeight);

    int32_t thumbY = 0;
    if (scrollMax > 0 && (trackHeight - thumbHeight) > 0) {
        thumbY = (scrollLine * (trackHeight - thumbHeight)) / scrollMax;
    }
    lv_obj_set_y(scrollbarThumb, thumbY);
}

void DiagnosticsScreen::_refreshVisuals()
{
    char buf[64];

    snprintf(buf, sizeof(buf) - 1, "VCC %u.%uV (%u.%uV/%u.%uV)",
        CONVERT_TO_FP1(stats.vcc),
        CONVERT_TO_FP1(stats.min.vcc),
        CONVERT_TO_FP1(stats.max.vcc)
    );
    lv_label_set_text(vccLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "Current %u.%02uA (%u.%02uA/%u.%02uA)",
        CONVERT_TO_FP2(stats.current),
        CONVERT_TO_FP2(stats.min.current),
        CONVERT_TO_FP2(stats.max.current)
    );
    lv_label_set_text(currentLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "Motor %d" DEGREE_UTF8 "C (%d" DEGREE_UTF8 "C/%d" DEGREE_UTF8 "C)",
        stats.motorTemp,
        stats.min.motorTemp,
        stats.max.motorTemp
    );
    lv_label_set_text(motorTempLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "MOSFETs %d" DEGREE_UTF8 "C (%d" DEGREE_UTF8 "C/%d" DEGREE_UTF8 "C)",
        stats.mosfetTemp,
        stats.min.mosfetTemp,
        stats.max.mosfetTemp
    );
    lv_label_set_text(mosfetTempLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "RPM %d/%u",
        (signed)pid.stats.rpm.get(),
        (unsigned)pid.getRPM()
    );
    lv_label_set_text(rpmPwmLabel, buf);

    pid.errorPrintf(buf, sizeof(buf) - 1);
    lv_label_set_text_fmt(lastErrorLabel, "Last Error %s", buf);
}

// === Dashboard Screen ===

void DashboardScreen::load()
{
    Screen::load();

    lv_obj_t *container = lv_obj_create(screen);
    lv_obj_remove_style_all(container);
    lv_obj_set_pos(container, kDashboardScreenContainerX, kDashboardScreenContainerY);
    lv_obj_set_size(container, kDashboardScreenContainerWidth, kDashboardScreenContainerHeight);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);

    voltageLabel = lv_label_create(container);
    lv_obj_set_style_text_color(voltageLabel, DASHBOARDSCREEN_COLOR_VOLTAGE, LV_PART_MAIN);
    lv_obj_set_style_text_font(voltageLabel, kDashboardScreenMetricsFont, LV_PART_MAIN);
    lv_obj_set_width(voltageLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(voltageLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_pos(voltageLabel, 0, 0);

    currentLabel = lv_label_create(container);
    lv_obj_set_style_text_color(currentLabel, DASHBOARDSCREEN_COLOR_CURRENT, LV_PART_MAIN);
    lv_obj_set_style_text_font(currentLabel, kDashboardScreenMetricsFont, LV_PART_MAIN);
    lv_obj_set_width(currentLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(currentLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_pos(currentLabel, 0, 18);

    motorTempLabel = lv_label_create(container);
    lv_obj_set_style_text_color(motorTempLabel, DASHBOARDSCREEN_COLOR_TEMPERATURE, LV_PART_MAIN);
    lv_obj_set_style_text_font(motorTempLabel, kDashboardScreenMetricsFont, LV_PART_MAIN);
    lv_obj_set_width(motorTempLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(motorTempLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(motorTempLabel, kDashboardScreenContainerWidth - kDashboardScreenColumnWidth, kDashboardScreenMotorTempOffsetY);

    mosfetTempLabel = lv_label_create(container);
    lv_obj_set_style_text_color(mosfetTempLabel, DASHBOARDSCREEN_COLOR_TEMPERATURE, LV_PART_MAIN);
    lv_obj_set_style_text_font(mosfetTempLabel, kDashboardScreenMetricsFont, LV_PART_MAIN);
    lv_obj_set_width(mosfetTempLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(mosfetTempLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(mosfetTempLabel, kDashboardScreenContainerWidth - kDashboardScreenColumnWidth, kDashboardScreenMosfetTempOffsetY);

    rpmLabel = lv_label_create(container);
    lv_obj_set_style_text_color(rpmLabel, DASHBOARDSCREEN_COLOR_SPEED, LV_PART_MAIN);
    lv_obj_set_style_text_font(rpmLabel, kDashboardScreenSpeedFont, LV_PART_MAIN);
    lv_obj_set_width(rpmLabel, kDashboardScreenContainerWidth);
    lv_obj_set_style_text_align(rpmLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_pos(rpmLabel, 0, kDashboardScreenRpmOffsetY);

    valueLabel = lv_label_create(container);
    lv_obj_set_style_text_color(valueLabel, DASHBOARDSCREEN_COLOR_PWM_LABEL, LV_PART_MAIN);
    lv_obj_set_style_text_font(valueLabel, kDashboardScreenValueFont, LV_PART_MAIN);
    lv_obj_set_width(valueLabel, kDashboardScreenContainerWidth);
    lv_obj_set_style_text_align(valueLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_pos(valueLabel, 0, kDashboardScreenValueBottomOffsetY);

    _refreshVisuals();

    lv_scr_load(screen);
}

void DashboardScreen::_refreshVisuals()
{
    char buf[32];

    start_screen_update_top_status_labels(voltageLabel, currentLabel, motorTempLabel, mosfetTempLabel);

    lv_obj_set_style_text_color(rpmLabel, DASHBOARDSCREEN_COLOR_SPEED, LV_PART_MAIN);
    lv_obj_set_style_text_font(rpmLabel, kDashboardScreenSpeedFont, LV_PART_MAIN);

    if (eeprom.isPIDMode()) {
        snprintf(buf, sizeof(buf) - 1, "%u RPM (%u)", (unsigned)pid.clampRPM(pid.stats.rpm.get()), (unsigned)pid.getRPM());
    }
    else {
        snprintf(buf, sizeof(buf) - 1, "%u RPM", (unsigned)pid.clampRPM(pid.stats.rpm.get()));
    }
    lv_label_set_text(rpmLabel, buf);

    switch(getSelectedValue()) {
        case SelectedValueType::SPEED:
            lv_label_set_text_fmt(valueLabel, "PWM %d%% %u.%uW", (int)((pid.stats.pwm.get() * 100) / pid.getPWMLevelARR()), CONVERT_TO_FP1(stats.vcc * stats.current / 1000U));
            break;
        case SelectedValueType::KP:
            FloatToString::convertTrimmed(buf, sizeof(buf) - 1, eeprom.getKp(), 6);
            lv_label_set_text_fmt(valueLabel, "Kp %s", buf);
            break;
        case SelectedValueType::KI:
            FloatToString::convertTrimmed(buf, sizeof(buf) - 1, eeprom.getKi(), 6);
            lv_label_set_text_fmt(valueLabel, "Ki %s", buf);
            break;
        case SelectedValueType::KD:
            FloatToString::convertTrimmed(buf, sizeof(buf) - 1, eeprom.getKd(), 6);
            lv_label_set_text_fmt(valueLabel, "Kd %s", buf);
            break;
        case SelectedValueType::ANTI_WINDUP: {
            const uint32_t antiWindup = (eeprom.getAntiWindup() * 1000) / UIConstants::kAntiWindupFactor;
            lv_label_set_text_fmt(valueLabel, "Anti-windup " SPRINTF_FP2_FMT "%%", CONVERT_TO_FP2(antiWindup));
            break;
        }
        case SelectedValueType::MAX:
            break;
    }
}

void DashboardScreen::setValue(uint32_t value)
{
    Screen::setValue(value);
    // change to font with all available glyphs
    lv_obj_set_style_text_font(valueLabel, (getSelectedValue() == SelectedValueType::SPEED) ? kDashboardScreenValueFixedFont : kDashboardScreenValueFont, LV_PART_MAIN);
}

void DashboardScreen::update()
{
    _refreshVisuals();
}

// === Start Screen ===

void StartScreen::load()
{
    Screen::load();

    lv_obj_t *container = lv_obj_create(screen);
    lv_obj_remove_style_all(container);
    lv_obj_set_pos(container, kStartScreenContainerX, kStartScreenContainerY);
    lv_obj_set_size(container, kStartScreenContainerWidth, kStartScreenContainerHeight);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);

    voltageLabel = lv_label_create(container);
    lv_obj_set_style_text_color(voltageLabel, STARTSCREEN_COLOR_VOLTAGE, LV_PART_MAIN);
    lv_obj_set_style_text_font(voltageLabel, kStartScreenMetricsFont, LV_PART_MAIN);
    lv_obj_set_width(voltageLabel, kStartScreenContainerWidth);
    lv_obj_set_style_text_align(voltageLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_pos(voltageLabel, 0, 0);

    currentLabel = lv_label_create(container);
    lv_obj_set_style_text_color(currentLabel, STARTSCREEN_COLOR_CURRENT, LV_PART_MAIN);
    lv_obj_set_style_text_font(currentLabel, kStartScreenMetricsFont, LV_PART_MAIN);
    lv_obj_set_width(currentLabel, kStartScreenContainerWidth);
    lv_obj_set_style_text_align(currentLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_pos(currentLabel, 0, 18);

    motorTempLabel = lv_label_create(container);
    lv_obj_set_style_text_color(motorTempLabel, STARTSCREEN_COLOR_TEMPERATURE, LV_PART_MAIN);
    lv_obj_set_style_text_font(motorTempLabel, kStartScreenMetricsFont, LV_PART_MAIN);
    lv_obj_set_width(motorTempLabel, kStartScreenColumnWidth);
    lv_obj_set_style_text_align(motorTempLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(motorTempLabel, kStartScreenContainerWidth - kStartScreenColumnWidth, kStartScreenMotorTempOffsetY);

    mosfetTempLabel = lv_label_create(container);
    lv_obj_set_style_text_color(mosfetTempLabel, STARTSCREEN_COLOR_TEMPERATURE, LV_PART_MAIN);
    lv_obj_set_style_text_font(mosfetTempLabel, kStartScreenMetricsFont, LV_PART_MAIN);
    lv_obj_set_width(mosfetTempLabel, kStartScreenColumnWidth);
    lv_obj_set_style_text_align(mosfetTempLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(mosfetTempLabel, kStartScreenContainerWidth - kStartScreenColumnWidth, kStartScreenMosfetTempOffsetY);

    directionLabel = lv_label_create(container);
    lv_obj_set_style_text_color(directionLabel, STARTSCREEN_COLOR_START_LABEL, LV_PART_MAIN);
    lv_obj_set_style_text_font(directionLabel, kStartScreenDirectionFont, LV_PART_MAIN);
    lv_obj_set_width(directionLabel, kStartScreenContainerWidth);
    lv_obj_set_style_text_align(directionLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_pos(directionLabel, 0, kStartScreenDirectionOffsetY);
    lv_obj_set_style_radius(directionLabel, kStartScreenDirectionCornerRadius, LV_PART_MAIN);
    lv_obj_set_style_pad_all(directionLabel, kStartScreenDirectionPadding, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(directionLabel, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(directionLabel, STARTSCREEN_COLOR_START_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(directionLabel, kStartScreenDirectionBorderWidth, LV_PART_MAIN);
    lv_obj_set_style_border_color(directionLabel, STARTSCREEN_COLOR_START_LABEL, LV_PART_MAIN);

    speedLabel = lv_label_create(container);
    lv_obj_set_style_text_color(speedLabel, STARTSCREEN_COLOR_SPEED, LV_PART_MAIN);
    lv_obj_set_style_text_font(speedLabel, kStartScreenSpeedFont, LV_PART_MAIN);
    lv_obj_set_width(speedLabel, kStartScreenContainerWidth);
    lv_obj_set_style_text_align(speedLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_pos(speedLabel, 0, kStartScreenSpeedOffsetY);

    _refreshVisuals();

    lv_scr_load(screen);
}

void StartScreen::_refreshVisuals()
{
    char buf[32];

    start_screen_update_top_status_labels(voltageLabel, currentLabel, motorTempLabel, mosfetTempLabel);

    // blink any errors
    if (((HAL_GetTick() / 1024) & 0x01) == 0 && pid.hasErrorCode()) {
        pid.errorPrintf(buf, sizeof(buf) - 1);
        lv_label_set_text(directionLabel, buf);
        lv_obj_set_style_text_color(directionLabel, STARTSCREEN_COLOR_ERROR, LV_PART_MAIN);
        lv_obj_set_style_border_color(directionLabel, STARTSCREEN_COLOR_ERROR_LABEL, LV_PART_MAIN);
        lv_obj_set_style_bg_color(directionLabel, STARTSCREEN_COLOR_ERROR_BG, LV_PART_MAIN);
    }
    else {
        lv_label_set_text_static(directionLabel, pid.isForwardMotorDirection() ? "START FORWARD" : "START REVERSE");
        lv_obj_set_style_text_color(directionLabel, STARTSCREEN_COLOR_START_LABEL, LV_PART_MAIN);
        lv_obj_set_style_border_color(directionLabel, STARTSCREEN_COLOR_START_LABEL, LV_PART_MAIN);
        lv_obj_set_style_bg_color(directionLabel, STARTSCREEN_COLOR_START_BG, LV_PART_MAIN);
    }

    snprintf(buf, sizeof(buf) - 1, eeprom.isPIDMode() ? "%u RPM" : "%u%% PWM", (unsigned)eeprom.getSpeed());
    lv_label_set_text(speedLabel, buf);
}

void StartScreen::update()
{
    _refreshVisuals();
}
