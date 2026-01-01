// Splash Screen Header for 370zMonitor
// 2018 Passion Red Nissan 370Z
// 5-second splash with fade transition to main screen

#ifndef _UI_SCREENSPLASH_H
#define _UI_SCREENSPLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

// Splash screen object
extern lv_obj_t * ui_ScreenSplash;

// Splash screen labels (for animation access if needed)
extern lv_obj_t * ui_Splash_Title;
extern lv_obj_t * ui_Splash_Subtitle;
extern lv_obj_t * ui_Splash_Year;
extern lv_obj_t * ui_Splash_LoadingBar;

// Initialize splash screen
void ui_ScreenSplash_screen_init(void);

// Destroy splash screen (cleanup)
void ui_ScreenSplash_screen_destroy(void);

// Callback for screen loaded event (triggers transition)
void ui_ScreenSplash_loaded_cb(lv_event_t * e);

// Splash duration in milliseconds
#define SPLASH_DURATION_MS      5000
#define SPLASH_FADE_TIME_MS     250

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
