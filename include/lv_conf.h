/**
  Author: sascha_lammers@gmx.de
*/

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/* Color and display */
#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 1
#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 135

/* On screen monitoring */
#ifndef LV_USE_PERF_MONITOR
#define LV_USE_PERF_MONITOR 0
#endif
#ifndef LV_USE_MEM_MONITOR
#define LV_USE_MEM_MONITOR 0
#endif

/* Memory */
#define LV_MEM_CUSTOM 0
// 8kb was enough, 6kb caused some out of memory issues after the fragmentation increased, recommendation is at least 16kb
// lvgl and new/delete is using the same block
#define LV_MEM_SIZE (24U * 1024U)
// redraw performance start and pid tuning graph
// 10 lines start=128ms graph=169ms
// 16 lines start=117ms graph=159ms
// 20 lines start=102ms graph=144ms
// 24 lines start=102ms graph=134ms
// 32 lines start=102ms graph=134ms
// 70 lines start= 92ms graph=130ms
#if HAVE_USB_DEVICE
#define LV_BUFFER_LINES 48
#else
#define LV_BUFFER_LINES 64
#endif

/* sprintf */
#define LV_SPRINTF_CUSTOM 1
#define LV_SPRINTF_INCLUDE <stdio.h>
#define lv_snprintf  snprintf
#define lv_vsnprintf vsnprintf

/* Timing */
#define LV_TICK_CUSTOM 1
#define LV_TICK_CUSTOM_INCLUDE "stm32f1xx.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (HAL_GetTick())
#define LV_DISP_DEF_REFR_PERIOD 30

/* Rounded corners/circles require complex drawing */
#define LV_DRAW_COMPLEX 1

/* Logging and asserts */
#ifndef LV_USE_LOG
#define LV_USE_LOG 0
#endif

#if LV_USE_LOG
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
#define LV_USE_ASSERT_MALLOC 1
#define LV_USE_ASSERT_MEM_INTEGRITY 1
#define LV_USE_ASSERT_NULL 0
#define LV_USE_ASSERT_STYLE 0
#define LV_USE_ASSERT_OBJ 1
#else
#define LV_USE_ASSERT_MALLOC 0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_NULL 0
#define LV_USE_ASSERT_STYLE 0
#define LV_USE_ASSERT_OBJ 0
#endif

/* Theme/layout */
#define LV_USE_THEME_DEFAULT 0
#define LV_USE_THEME_BASIC 0
#define LV_USE_THEME_MONO 0
#define LV_USE_FLEX 0
#define LV_USE_GRID 0

/* Widgets: keep only bare object infrastructure */
#define LV_USE_ARC 0
#define LV_USE_BAR 0
#define LV_USE_BTN 0
#define LV_USE_BTNMATRIX 0
#define LV_USE_CANVAS 0
#define LV_USE_CHECKBOX 0
#define LV_USE_DROPDOWN 0
#define LV_USE_IMG 0
#define LV_USE_LABEL 1
#define LV_USE_LINE 1
#define LV_USE_ROLLER 0
#define LV_USE_SLIDER 0
#define LV_USE_SWITCH 0
#define LV_USE_TABLE 0
#define LV_USE_TEXTAREA 0

/* Extra widgets */
#define LV_USE_ANIMIMG 0
#define LV_USE_CALENDAR 0
#define LV_USE_CHART 0
#define LV_USE_COLORWHEEL 0
#define LV_USE_IMGBTN 0
#define LV_USE_KEYBOARD 0
#define LV_USE_LED 0
#define LV_USE_LIST 0
#define LV_USE_MENU 0
#define LV_USE_METER 0
#define LV_USE_MSGBOX 0
#define LV_USE_SPAN 0
#define LV_USE_SPINBOX 0
#define LV_USE_SPINNER 0
#define LV_USE_TABVIEW 0
#define LV_USE_TILEVIEW 0
#define LV_USE_WIN 0

/* Others */
#define LV_USE_ANIMATION 0
#define LV_USE_SHADOW 0
#define LV_USE_FONT_COMPRESSED 1

/* Image/decoder libs */
#define LV_USE_BMP 0
#define LV_USE_GIF 0
#define LV_USE_PNG 0
#define LV_USE_SJPG 0
#define LV_USE_QRCODE 0

/* Fonts */
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_24 1

#endif
