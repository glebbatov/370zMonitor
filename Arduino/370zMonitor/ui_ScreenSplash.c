// Splash Screen for 370zMonitor
// 2018 Passion Red Nissan 370Z
// 5-second splash with fade transition to main screen

#include "ui_ScreenSplash.h"
#include "ui.h"
#include "ui_Screen1.h"
#include "Images/splash_450x300_cropped.c"

// Screen objects
lv_obj_t * ui_ScreenSplash = NULL;
lv_obj_t * ui_Splash_Title = NULL;
lv_obj_t * ui_Splash_Subtitle = NULL;
lv_obj_t * ui_Splash_Year = NULL;
lv_obj_t * ui_Splash_LoadingBar = NULL;
lv_obj_t * ui_Splash_CarImage = NULL;

// Passion Red color (Nissan A54 Vibrant Red / Passion Red)
#define PASSION_RED_COLOR       lv_color_hex(0xA31621)
#define PASSION_RED_BRIGHT      lv_color_hex(0xD41F2D)
#define DARK_BACKGROUND         lv_color_hex(0x0A0A0A)
#define ACCENT_GRAY             lv_color_hex(0x333333)
#define TEXT_WHITE              lv_color_hex(0xFFFFFF)
#define TEXT_GRAY               lv_color_hex(0xAAAAAA)

// Animation for loading bar
static lv_anim_t loading_anim;

// Loading bar animation callback
static void loading_bar_anim_cb(void * obj, int32_t value) {
    lv_bar_set_value((lv_obj_t *)obj, value, LV_ANIM_OFF);
}

// Screen loaded callback - triggers transition to main screen
void ui_ScreenSplash_loaded_cb(lv_event_t * e) {
    LV_UNUSED(e);
    
    // Load main screen after SPLASH_DURATION_MS with fade animation
    // Parameters: screen, animation type, fade time, delay before start, auto-delete old screen
    // Note: Using false for auto_del to keep splash in memory (one-time cost, prevents dangling refs)
    lv_screen_load_anim(ui_Screen1, LV_SCR_LOAD_ANIM_FADE_IN, SPLASH_FADE_TIME_MS, 
                        SPLASH_DURATION_MS - SPLASH_FADE_TIME_MS, false);
}

void ui_ScreenSplash_screen_init(void) {
    // Create splash screen (NULL parent = root screen)
    ui_ScreenSplash = lv_obj_create(NULL);
    lv_obj_remove_flag(ui_ScreenSplash, LV_OBJ_FLAG_SCROLLABLE);
    
    // Dark background
    lv_obj_set_style_bg_color(ui_ScreenSplash, DARK_BACKGROUND, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ScreenSplash, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    
    // ========== Background car silhouette image ==========
    //ui_Splash_CarImage = lv_image_create(ui_ScreenSplash);
    //lv_image_set_src(ui_Splash_CarImage, &splash_450x300_cropped);
    //lv_obj_align(ui_Splash_CarImage, LV_ALIGN_CENTER, 0, 20);  // Slightly below center
    //lv_obj_set_style_image_opa(ui_Splash_CarImage, 100, 0);    // ~40% opacity
    //lv_image_set_scale(ui_Splash_CarImage, 400);               // Scale to ~1.56x (800px wide)
    
    // ========== Top accent line (Passion Red) ==========
    lv_obj_t * top_line = lv_obj_create(ui_ScreenSplash);
    lv_obj_remove_flag(top_line, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(top_line, 800, 4);
    lv_obj_align(top_line, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(top_line, PASSION_RED_COLOR, 0);
    lv_obj_set_style_bg_opa(top_line, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(top_line, 0, 0);
    lv_obj_set_style_radius(top_line, 0, 0);
    
    // ========== Main Title: "PERFORMANCE" (red) + "MONITOR" (white) ==========
    // Create PERFORMANCE in red
    ui_Splash_Title = lv_label_create(ui_ScreenSplash);
    lv_label_set_text(ui_Splash_Title, "PERFORMANCE");
    lv_obj_set_style_text_font(ui_Splash_Title, &ui_font_OrbitronExtraBold34, LV_PART_MAIN);
    lv_obj_set_style_text_color(ui_Splash_Title, PASSION_RED_BRIGHT, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(ui_Splash_Title, 5, LV_PART_MAIN);
    lv_obj_align(ui_Splash_Title, LV_ALIGN_CENTER, -115, -50);
    
    // Create MONITOR in white, positioned right of PERFORMANCE
    lv_obj_t * monitor_label = lv_label_create(ui_ScreenSplash);
    lv_label_set_text(monitor_label, "MONITOR");
    lv_obj_set_style_text_font(monitor_label, &ui_font_OrbitronExtraBold34, LV_PART_MAIN);
    lv_obj_set_style_text_color(monitor_label, TEXT_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(monitor_label, 5, LV_PART_MAIN);
    lv_obj_align_to(monitor_label, ui_Splash_Title, LV_ALIGN_OUT_RIGHT_MID, 12, 0);
    
    // ========== Year/Version line ==========
    ui_Splash_Year = lv_label_create(ui_ScreenSplash);
    lv_label_set_text(ui_Splash_Year, "TRACK MONITOR SYSTEM");
    lv_obj_set_style_text_font(ui_Splash_Year, &ui_font_OrbitronRegular12, LV_PART_MAIN);
    lv_obj_set_style_text_color(ui_Splash_Year, ACCENT_GRAY, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(ui_Splash_Year, 3, LV_PART_MAIN);
    lv_obj_align(ui_Splash_Year, LV_ALIGN_CENTER, 0, 0);
    
    // ========== Loading bar (animated) ==========
    ui_Splash_LoadingBar = lv_bar_create(ui_ScreenSplash);
    lv_obj_set_size(ui_Splash_LoadingBar, 400, 6);
    lv_obj_align(ui_Splash_LoadingBar, LV_ALIGN_CENTER, 0, 100);
    lv_bar_set_range(ui_Splash_LoadingBar, 0, 100);
    lv_bar_set_value(ui_Splash_LoadingBar, 0, LV_ANIM_OFF);
    
    // Bar styling
    lv_obj_set_style_bg_color(ui_Splash_LoadingBar, ACCENT_GRAY, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_Splash_LoadingBar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(ui_Splash_LoadingBar, 3, LV_PART_MAIN);
    
    // Indicator (filled part) styling
    lv_obj_set_style_bg_color(ui_Splash_LoadingBar, PASSION_RED_COLOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(ui_Splash_LoadingBar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(ui_Splash_LoadingBar, 3, LV_PART_INDICATOR);
    
    // ========== Bottom accent line ==========
    lv_obj_t * bottom_line = lv_obj_create(ui_ScreenSplash);
    lv_obj_remove_flag(bottom_line, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(bottom_line, 800, 4);
    lv_obj_align(bottom_line, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(bottom_line, PASSION_RED_COLOR, 0);
    lv_obj_set_style_bg_opa(bottom_line, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(bottom_line, 0, 0);
    lv_obj_set_style_radius(bottom_line, 0, 0);
    
    // ========== Version info (bottom right) ==========
    lv_obj_t * version_label = lv_label_create(ui_ScreenSplash);
    lv_label_set_text(version_label, "v5.1");
    lv_obj_set_style_text_font(version_label, &ui_font_OrbitronRegular12, LV_PART_MAIN);
    lv_obj_set_style_text_color(version_label, ACCENT_GRAY, LV_PART_MAIN);
    lv_obj_align(version_label, LV_ALIGN_BOTTOM_RIGHT, -15, -15);
    
    // ========== Start loading bar animation ==========
    lv_anim_init(&loading_anim);
    lv_anim_set_var(&loading_anim, ui_Splash_LoadingBar);
    lv_anim_set_values(&loading_anim, 0, 100);
    lv_anim_set_duration(&loading_anim, SPLASH_DURATION_MS - 200);  // Complete just before fade
    lv_anim_set_exec_cb(&loading_anim, loading_bar_anim_cb);
    lv_anim_set_path_cb(&loading_anim, lv_anim_path_ease_in_out);
    lv_anim_start(&loading_anim);
    
    // ========== Register screen loaded callback ==========
    lv_obj_add_event_cb(ui_ScreenSplash, ui_ScreenSplash_loaded_cb, LV_EVENT_SCREEN_LOADED, NULL);
}

void ui_ScreenSplash_screen_destroy(void) {
    if (ui_ScreenSplash != NULL) {
        lv_obj_delete(ui_ScreenSplash);
        ui_ScreenSplash = NULL;
    }
    ui_Splash_Title = NULL;
    ui_Splash_Subtitle = NULL;
    ui_Splash_Year = NULL;
    ui_Splash_LoadingBar = NULL;
    ui_Splash_CarImage = NULL;
}
