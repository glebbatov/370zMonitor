//-----------------------------------------------------------------

/*
 * DIAGNOSTIC VERSION v3 - Using CLIB malloc with PSRAM
 * ESP32-S3 automatically uses PSRAM for large allocations when configured
 */

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <lvgl.h>
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>
#include <TAMC_GT911.h>  // Touch controller

//-----------------------------------------------------------------

// ===== CRITICAL: Configure ESP32 to prefer PSRAM for malloc =====
// This runs before setup() to configure the heap
__attribute__((constructor)) void configurePSRAM() {
    // This hint tells ESP32 to prefer PSRAM for allocations > 32KB
    heap_caps_malloc_extmem_enable(32 * 1024);
}

//-----------------------------------------------------------------

// ===== FEATURE FLAGS =====
#define ENABLE_TOUCH        1
#define ENABLE_UI_UPDATES   1   // Enable bar/label updates
#define ENABLE_CHARTS       1   // Re-enable charts
#define UPDATE_INTERVAL_MS  150 // Update every (150ms original speed)

//-----------------------------------------------------------------

// CH422 IO Expander (used for backlight ON/OFF and touch reset only - NOT for PWM brightness)
#define CH422_ADDR_SYSTEM 0x24  // Mode register - DO NOT write brightness values here!
#define CH422_ADDR_IOWR   0x38  // IO output register
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQ_HZ 100000
#define EXIO_TP_RST   1
#define EXIO_DISP     2
#define EXIO_SD_CS    4
#define TOUCH_INT_PIN 4

static uint8_t g_exio_state = 0;
static bool g_ioexp_ok = false;

// Brightness control (0-255, controls LVGL overlay opacity)
static uint8_t g_brightness_level = 255;  // Start at 100%

//-----------------------------------------------------------------

#define TARGET_FPS 50
#define FRAME_TIME_MS (1000 / TARGET_FPS)  // ~16ms

//-----------------------------------------------------------------

#pragma region Gauges data

//OIL PRESS
//	-Spread:
//		0-150 PSI
//	-VALUE CRITICAL:
//		<= 10 PSI per 1000 RPM minimum
//		>= 120 PSI
int OIL_PRESS_Min_PSI = 0;
int OIL_PRESS_Max_PSI = 150;
int OIL_PRESS_ValueCriticalAbsolute = 120;
bool OIL_PRESS_ValueCriticalRPM = false;

//OIL TEMP [Oil pan/C]
//	(Oil pan, Out line - post termostatic valve)
//	-Spread:
//		150-300 F*
//	-VALUE CRITICAL:
//		>= 260 F*
int OIL_TEMP_Value_Pan_Celsius;
int OIL_TEMP_Value_Pan_Fahrenheit;
int OIL_TEMP_Value_Cooled_Celsius;
int OIL_TEMP_Value_Cooled_Fahrenheit;
int OIL_TEMP_Min_F = 150;
int OIL_TEMP_Max_F = 300;
int OIL_TEMP_ValueCriticalF = 260;

//WATER TEMP [H/C]
//	(Upper/Lower radiator hose)
//	-Spread:
//		100-260 F*
//	-VALUE CRITICAL:
//		>= 230 F*
int W_TEMP_Value_Hot_Celsius;
int W_TEMP_Value_Hot_Fahrenheit;
int W_TEMP_Value_Cooled_Celsius;
int W_TEMP_Value_Cooled_Fahrenheit;
int W_TEMP_Min_F = 100;
int W_TEMP_Max_F = 260;
int W_TEMP_ValueCritical_F = 230;

//TRAN TEMP [H/C]
//	(In/Out lines - pre termostatic valve)
//	-Spread:
//		80-280 F*
//	-VALUE CRITICAL:
//		>= 230 F*
int TRAN_TEMP_Value_Hot_Celsius;
int TRAN_TEMP_Value_Hot_Fahrenheit;
int TRAN_TEMP_Value_Cooled_Celsius;
int TRAN_TEMP_Value_Cooled_Fahrenheit;
int TRAN_TEMP_Min_F = 80;
int TRAN_TEMP_Max_F = 280;
int TRAN_TEMP_ValueCritical_F = 230;

//STEER TEMP [H/C]
//	(In/Out lines)
//	-Spread:
//		60-300 F*
//	-VALUE CRITICAL:
//		>= 230 F*
int STEER_TEMP_Value_Hot_Celsius;
int STEER_TEMP_Value_Hot_Fahrenheit;
int STEER_TEMP_Value_Cooled_Celsius;
int STEER_TEMP_Value_Cooled_Fahrenheit;
int STEER_TEMP_Min_F = 60;
int STEER_TEMP_Max_F = 300;
int STEER_TEMP_ValueCritical_F = 230;

//DIFF TEMP [H/C]
//	(In/Out lines)
//	-Spread:
//		60-320 F*
//	-VALUE CRITICAL:
//		>= 270 F*
int DIFF_TEMP_Value_Hot_Celsius;
int DIFF_TEMP_Value_Hot_Fahrenheit;
int DIFF_TEMP_Value_Cooled_Celsius;
int DIFF_TEMP_Value_Cooled_Fahrenheit;
int DIFF_TEMP_Min_F = 60;
int DIFF_TEMP_Max_F = 320;
int DIFF_TEMP_ValueCritical_F = 270;

//FUEL TRUST
//	-Spread:
//		0-100%
//	-VALUE CRITICAL:
//		<= 50%
int FUEL_TRUST_Min_F = 0;
int FUEL_TRUST_Max_F = 100;
int FUEL_TRUST_ValueCritical_F = 50;

#pragma endregion Gauges data

//-----------------------------

#pragma region OBD Data

int RPM = 0;

#pragma endregion OBD Data

//-----------------------------

#pragma region Sensors Data

int oilPressurePSI = 0;

#pragma endregion Sensors Data

//-----------------------------

#pragma region Animation

// Smoothing value
static float smooth_oil_pressure = 75.0f;
static float smooth_oil_temp_f = 210.0f;
static float smooth_fuel_trust = 100.0f;

// Smoothing factor: 0.3 = responsive, 0.1 = very smooth
#define SMOOTH_FACTOR 0.3f

#pragma endregion Animation

//-----------------------------

#pragma region Colors

// Colors
static int hexRed = 0xFF0000;
static int hexOrange = 0xFF4619;

#pragma endregion Colors

//-----------------------------

#pragma region UI objects

#include "ui.h"

//OIL PRESS
extern lv_obj_t * ui_OIL_PRESS_Bar;
extern lv_obj_t * ui_OIL_PRESS_CHART;
extern lv_obj_t * ui_OIL_PRESS_Value;
extern lv_obj_t * ui_OIL_PRESS_VALUE_CRITICAL_Label;

//OIL TEMP [Oil pan/C]
extern lv_obj_t * ui_OIL_TEMP_Bar;
extern lv_obj_t * ui_OIL_TEMP_CHART;
extern lv_obj_t * ui_OIL_TEMP_Value_P;
extern lv_obj_t * ui_OIL_TEMP_Value_C;

//WATER TEMP [H/C]
extern lv_obj_t * ui_W_TEMP_Bar;
extern lv_obj_t * ui_W_TEMP_CHART;
extern lv_obj_t* ui_W_TEMP_Value_H;
extern lv_obj_t* ui_W_TEMP_Value_C;

//TRAN TEMP [H/C]
extern lv_obj_t * ui_TRAN_TEMP_Bar;
extern lv_obj_t * ui_TRAN_TEMP_CHART;
extern lv_obj_t* ui_TRAN_TEMP_Value_H;
extern lv_obj_t* ui_TRAN_TEMP_Value_C;


//STEER TEMP [H/C]
extern lv_obj_t * ui_STEER_TEMP_Bar;
extern lv_obj_t * ui_STEER_TEMP_CHART;
extern lv_obj_t * ui_STEER_TEMP_Value_H;
extern lv_obj_t * ui_STEER_TEMP_Value_C;

//DIFF TEMP [H/C]
extern lv_obj_t * ui_DIFF_TEMP_Bar;
extern lv_obj_t * ui_DIFF_TEMP_CHART;
extern lv_obj_t * ui_DIFF_TEMP_Value_H;
extern lv_obj_t * ui_DIFF_TEMP_Value_C;

//FUEL TRUST
extern lv_obj_t * ui_FUEL_TRUST_Bar;
extern lv_obj_t * ui_FUEL_TRUST_CHART;
extern lv_obj_t * ui_FUEL_TRUST_Value;

#pragma endregion UI objects

//-----------------------------------------------------------------

extern lv_obj_t * ui_Screen1;

// Display - with corrected timing for proper alignment
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    5,   // DE
    3,   // VSYNC
    46,  // HSYNC
    7,   // PCLK
    1, 2, 42, 41, 40,           // R3-R7
    39, 0, 45, 48, 47, 21,      // G2-G7
    14, 38, 18, 17, 10,         // B3-B7
    0,   // hsync_polarity
    8,   // hsync_front_porch (reduced from 40)
    4,   // hsync_pulse_width (reduced from 48)
    8,   // hsync_back_porch (reduced from 40)
    0,   // vsync_polarity
    8,   // vsync_front_porch (reduced from 13)
    4,   // vsync_pulse_width (reduced from 3)
    8,   // vsync_back_porch (reduced from 32)
    1,   // pclk_active_neg
    14000000,  // pixel clock
    true,      // auto_flush
    0,   // de_idle_high
    0    // pclk_idle_high
);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, rgbpanel, 0, true);

//-----------------------------------------------------------------

// LVGL
#define LVGL_BUFFER_SIZE (800 * 69)  // 69 lines - will go to PSRAM | 480 ÷ 7 ≈ 69 lines per strip
static lv_display_t *disp;
static lv_indev_t *indev;
static uint8_t *disp_draw_buf1;
static uint8_t *disp_draw_buf2;

//-----------------------------------------------------------------

// GT911 Touch Controller
// RST is handled via IO expander (EXIO_TP_RST), so we pass -1
TAMC_GT911 touch = TAMC_GT911(I2C_SDA, I2C_SCL, TOUCH_INT_PIN, -1, 800, 480);

//-----------------------------------------------------------------

// Counters
static uint32_t loop_count = 0;
static uint32_t flush_count = 0;
static uint32_t update_count = 0;

// CPU usage tracking
static uint32_t cpu_busy_time = 0;  // Time spent working (not in delay)

// Combined utility box (FPS, CPU, Brightness)
static lv_obj_t * utility_box = NULL;
static lv_obj_t * g_dim_overlay = NULL;          // LVGL black overlay for "fake" brightness
static lv_timer_t * g_util_single_tap_timer = NULL;  // used to disambiguate single vs double tap
static lv_obj_t * utility_label = NULL;
static bool utilities_visible = true;

// Touch controller state (declared early for use in setBrightness)
static uint32_t consecutive_invalid = 0;
static uint32_t last_touch_reset = 0;

// Chart series
static lv_chart_series_t * chart_series_oil_press = NULL;
static lv_chart_series_t * chart_series_oil_temp = NULL;

// Chart history for 5-second averaging
static int32_t pressure_sum = 0;
static uint32_t pressure_samples = 0;
static uint32_t pressure_bucket_start = 0;
static int32_t temp_sum = 0;
static uint32_t temp_samples = 0;
static uint32_t temp_bucket_start = 0;
#define CHART_BUCKET_MS 5000  // 5 seconds per bar

// Chart history arrays for color lookup (stores actual values for each bar)
#define CHART_POINTS 24
static int32_t oil_press_history[CHART_POINTS] = {0};
static int32_t oil_temp_history[CHART_POINTS] = {0};  // Stored in °C

//=================================================================

static inline int OilPressMin_FromRPM()
{
    //VALUE CRITICAL <= 10 PSI per 1000 RPM minimum

    int value = (RPM * 10 + 999) / 1000;
    const bool isCritical = (oilPressurePSI < value);

    return isCritical;
}

//-----------------------------------------------------------------

#pragma region Chart draw callbacks for bar coloring

// Oil pressure: red < 20, orange 20-100, red > 100
static void oil_press_chart_draw_cb(lv_event_t * e) {
    lv_draw_task_t * draw_task = lv_event_get_draw_task(e);

    lv_draw_fill_dsc_t * fill_dsc = lv_draw_task_get_fill_dsc(draw_task);
    if(fill_dsc == NULL) return;

    // In LVGL v9 draw descriptors start with lv_draw_dsc_base_t
    lv_draw_dsc_base_t * base_dsc = (lv_draw_dsc_base_t *)fill_dsc;
    if(base_dsc->part != LV_PART_ITEMS) return;

    uint32_t idx = base_dsc->id2;  // Point index
    if(idx >= CHART_POINTS) return;

    int32_t psi = oil_press_history[idx];

    bool is_red = (psi < 20) || (psi > 100);     // change this if you only want >100 to be red
    fill_dsc->color = is_red ? lv_color_hex(hexRed) : lv_color_hex(hexOrange);
}

// Oil temp: red < 100°F or > 260°F, orange in between
static void oil_temp_chart_draw_cb(lv_event_t * e) {
    lv_draw_task_t * draw_task = lv_event_get_draw_task(e);

    lv_draw_fill_dsc_t * fill_dsc = lv_draw_task_get_fill_dsc(draw_task);
    if(fill_dsc == NULL) return;

    lv_draw_dsc_base_t * base_dsc = (lv_draw_dsc_base_t *)fill_dsc;
    if(base_dsc->part != LV_PART_ITEMS) return;

    uint32_t idx = base_dsc->id2;  // Point index
    if(idx >= CHART_POINTS) return;

    int32_t temp_c = oil_temp_history[idx];

    bool is_red = (temp_c < W_TEMP_Min_F) || (temp_c > W_TEMP_Max_F);
    fill_dsc->color = is_red ? lv_color_hex(hexRed) : lv_color_hex(hexOrange);
}

// Helper to shift history array left and add new value
static void shift_history(int32_t* history, int32_t new_value) {
    for (int i = 0; i < CHART_POINTS - 1; i++) {
        history[i] = history[i + 1];
    }
    history[CHART_POINTS - 1] = new_value;
}

#pragma endregion Chart draw callbacks for bar coloring

//-----------------------------

#pragma region Utility box callbacks (single tap = brightness, double tap = hide/show)

#define DOUBLE_TAP_TIMEOUT_MS 300  // Max time between taps for double-tap

// We *delay* the single-tap action until the double-tap window expires.
// That way: double-tap does NOT also change brightness on the first tap.
static void utility_box_single_tap_cb(lv_timer_t * t) {
    LV_UNUSED(t);
    g_util_single_tap_timer = NULL;

    // Toggle brightness: 100% <-> 35%
    if (g_brightness_level == 255) {
        setBrightness(89);   // 35% (good night mode dimming)
    } else {
        setBrightness(255);  // 100%
    }

    Serial.printf("[UI] Single-tap: Brightness -> %d%%\n", (g_brightness_level * 100) / 255);
}

static void utility_box_tap_cb(lv_event_t * e) {
    LV_UNUSED(e);

    // Second tap arrived before the single-tap timer fired => treat as double tap
    if (g_util_single_tap_timer) {
        lv_timer_del(g_util_single_tap_timer);
        g_util_single_tap_timer = NULL;

        utilities_visible = !utilities_visible;
        if (utility_box) {
            if (utilities_visible) {
                lv_obj_set_style_opa(utility_box, LV_OPA_COVER, 0);      // visible
            } else {
                lv_obj_set_style_opa(utility_box, LV_OPA_TRANSP, 0);     // hidden but still tappable
            }
        }

        Serial.printf("[UI] Double-tap: Utility box %s\n", utilities_visible ? "shown" : "hidden");
        return;
    }

    // First tap: arm a timer. If no 2nd tap arrives in time, it'll become a single tap.
    g_util_single_tap_timer = lv_timer_create(utility_box_single_tap_cb, DOUBLE_TAP_TIMEOUT_MS, NULL);
    lv_timer_set_repeat_count(g_util_single_tap_timer, 1);
}

// Update the combined utility label
static void update_utility_label(int fps, int cpu_percent) {
    if (utility_label) {
        char buf[48];
        int bri_percent = (g_brightness_level * 100) / 255;
        snprintf(buf, sizeof(buf), "%3d FPS\n%3d%% CPU\n%3d%% BRI", fps, cpu_percent, bri_percent);
        lv_label_set_text(utility_label, buf);
    }
}

#pragma endregion

//-----------------------------------------------------------------

#pragma region Hardware functions

//-----------------------------

#pragma region IO Expander Functions

static bool ch422_write_system(uint8_t sys_param) {
    Wire.beginTransmission(CH422_ADDR_SYSTEM);
    Wire.write(sys_param);
    return Wire.endTransmission(true) == 0;
}

static bool ch422_write_io(uint8_t io_state) {
    Wire.beginTransmission(CH422_ADDR_IOWR);
    Wire.write(io_state);
    return Wire.endTransmission(true) == 0;
}

static void exio_set(uint8_t exio_bit, bool level) {
    if (exio_bit > 7) return;
    if (level) g_exio_state |= (1u << exio_bit);
    else g_exio_state &= ~(1u << exio_bit);
    ch422_write_io(g_exio_state);
}

bool initIOExtension() {
    delay(10);
    if (!ch422_write_system(0x11)) return false;
    delay(10);
    g_exio_state = (1u << EXIO_TP_RST) | (1u << EXIO_SD_CS);
    return ch422_write_io(g_exio_state);
}

void setBacklight(bool on) {
    if (g_ioexp_ok) exio_set(EXIO_DISP, on);
}

void setBrightness(uint8_t brightness) {
    // IMPORTANT:
    // The CH422G on this Waveshare board is an IO expander used for reset/backlight ON/OFF.
    // Writing "brightness" bytes to its 0x24 register was overwriting its MODE register,
    // which breaks TP_RST/LCD_BL control and can make GT911 start returning 65535 -> "stuck".
    //
    // So: do NOT do PWM over CH422G.
    // Instead we do "UI brightness": a full-screen black overlay with variable opacity.
    // It looks like dimming, doesn't mess with I2C, and touch stays stable.

    g_brightness_level = brightness;

    // Create the overlay lazily (after LVGL is up)
    if (!g_dim_overlay) {
        lv_obj_t * top = lv_layer_top();
        g_dim_overlay = lv_obj_create(top);
        lv_obj_remove_style_all(g_dim_overlay);

        int w = lv_disp_get_hor_res(NULL);
        int h = lv_disp_get_ver_res(NULL);
        lv_obj_set_size(g_dim_overlay, w, h);
        lv_obj_set_pos(g_dim_overlay, 0, 0);

        // Let touches pass through
        lv_obj_clear_flag(g_dim_overlay, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(g_dim_overlay, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(g_dim_overlay, LV_OBJ_FLAG_IGNORE_LAYOUT);

        lv_obj_set_style_bg_color(g_dim_overlay, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(g_dim_overlay, LV_OPA_TRANSP, 0);
    }

    if (brightness >= 250) {
        lv_obj_add_flag(g_dim_overlay, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(g_dim_overlay, LV_OBJ_FLAG_HIDDEN);

        // brightness: 255 = no dim, 0 = full black
        uint8_t opa = (uint8_t)(255 - brightness);
        lv_obj_set_style_bg_opa(g_dim_overlay, opa, 0);
    }

    Serial.printf("[BRIGHTNESS] UI dim -> %d%%\n", (brightness * 100) / 255);
}

#pragma endregion IO Expander Functions

//-----------------------------

#pragma region LVGL Callbacks
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    uint32_t len = w * h;
    
    // Byte swap (optimized 32-bit)
    uint32_t *buf32 = (uint32_t *)px_map;
    uint32_t len32 = len >> 1;
    for (uint32_t i = 0; i < len32; i++) {
        uint32_t v = buf32[i];
        buf32[i] = ((v & 0x00FF00FF) << 8) | ((v & 0xFF00FF00) >> 8);
    }
    if (len & 1) {
        uint16_t *buf16 = (uint16_t *)px_map;
        buf16[len - 1] = (buf16[len - 1] >> 8) | (buf16[len - 1] << 8);
    }
    
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
    lv_display_flush_ready(disp);
    flush_count++;
}

#pragma endregion LVGL Callbacks

//-----------------------------

#pragma region Touch Callbacks
// Touch thresholds for detecting stuck controller
#define MAX_CONSECUTIVE_INVALID 50  // Higher threshold since I2C issue is fixed
#define TOUCH_RESET_COOLDOWN_MS 5000  // 5 seconds between resets (rarely needed now)

void my_touch_read(lv_indev_t *indev, lv_indev_data_t *data) {
    touch.read();
    
    static bool was_touched = false;
    uint32_t now = millis();
    
    if (touch.isTouched) {
        // Get raw GT911 coordinates
        int raw_x = touch.points[0].x;
        int raw_y = touch.points[0].y;
        
        // Filter out invalid readings (GT911 returns 65535 on errors)
        if (raw_x == 65535 || raw_y == 65535 || raw_x > 800 || raw_y > 800) {
            consecutive_invalid++;
            
            // Safety: reset touch controller if truly stuck (should be rare now)
            if (consecutive_invalid >= MAX_CONSECUTIVE_INVALID && 
                (now - last_touch_reset) > TOUCH_RESET_COOLDOWN_MS) {
                Serial.println("[TOUCH] Controller stuck - hardware reset");
                exio_set(EXIO_TP_RST, false);  // Pull reset LOW
                delay(10);
                exio_set(EXIO_TP_RST, true);   // Release reset HIGH  
                delay(50);                      // GT911 needs time to initialize
                touch.begin();
                touch.setRotation(0);
                consecutive_invalid = 0;
                last_touch_reset = now;
            }
            
            data->state = LV_INDEV_STATE_RELEASED;
            was_touched = false;
            return;
        }
        
        // Valid touch - reset counter
        consecutive_invalid = 0;
        was_touched = true;
        data->state = LV_INDEV_STATE_PRESSED;
        
        // Touch calibration: X and Y are SWAPPED on this panel
        // Raw Y: ~30 = LEFT, ~775 = RIGHT  -> maps to screen_x: 0 to 799
        // Raw X: ~770 = TOP, ~350 = BOTTOM -> maps to screen_y: 0 to 479
        int screen_x = (raw_y - 30) * 800 / 745;
        int screen_y = (770 - raw_x) * 480 / 420;
        
        // Clamp to valid range
        if (screen_x < 0) screen_x = 0;
        if (screen_x > 799) screen_x = 799;
        if (screen_y < 0) screen_y = 0;
        if (screen_y > 479) screen_y = 479;
        
        data->point.x = screen_x;
        data->point.y = screen_y;
        
        // Debug: print touch coordinates periodically
        static uint32_t last_touch_debug = 0;
        if (now - last_touch_debug > 200) {
            Serial.printf("[TOUCH] raw(%d,%d) -> screen(%d,%d)\n", raw_x, raw_y, screen_x, screen_y);
            last_touch_debug = now;
        }
    } else {
        consecutive_invalid = 0;  // Reset counter when not touched
        if (was_touched) {
            Serial.println("[TOUCH] Released");
            was_touched = false;
        }
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#pragma endregion Touch Callbacks

//-----------------------------

#pragma endregion Hardware functions

//=================================================================

void setup() {

    Serial.begin(115200);
    delay(1000);
    
#pragma region DIAGNOSTICS

    Serial.println("\n========================================");
    Serial.println("   DIAGNOSTIC v3 - CLIB malloc + PSRAM");
    Serial.println("========================================");
    
    // Check PSRAM first
    if (psramFound()) {
        Serial.printf("✓ PSRAM: %u bytes total, %u free\n", 
                      ESP.getPsramSize(), ESP.getFreePsram());
    } else {
        Serial.println("✗ WARNING: No PSRAM detected!");
        Serial.println("  Check: Tools -> PSRAM -> 'OPI PSRAM'");
    }
    
    Serial.printf("Internal heap: %u free\n", ESP.getFreeHeap());
    Serial.println("========================================\n");
    
    // I2C init
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
    Wire.setTimeOut(50);
    delay(50);
    
    // IO expander
    Serial.println("[1/6] IO Expander...");
    g_ioexp_ok = initIOExtension();
    Serial.printf("      %s\n", g_ioexp_ok ? "OK" : "FAILED");
    
    // Touch controller (must be after IO expander releases reset)
    Serial.println("[1.5/6] Touch controller...");
    delay(100);  // Give GT911 time after reset release
    touch.begin();
    touch.setRotation(0);  // Match display orientation
    Serial.println("      GT911 initialized");
    
    // Display
    Serial.println("[2/6] Display init...");
    gfx->begin();
    gfx->fillScreen(0x0000);
    Serial.println("      OK");
    
    // Backlight
    Serial.println("[3/6] Backlight...");
    setBacklight(true);
    delay(100);
    Serial.println("      OK");
    
    // LVGL
    Serial.println("[4/6] LVGL init...");
    lv_init();
    
    size_t buf_bytes = LVGL_BUFFER_SIZE * sizeof(lv_color_t);
    Serial.printf("      Buffer size: %u bytes\n", buf_bytes);
    
    // Explicitly allocate buffer1 from PSRAM
    disp_draw_buf1 = (uint8_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!disp_draw_buf1) {
        Serial.println("      PSRAM alloc 1 failed, trying internal...");
        disp_draw_buf1 = (uint8_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    }
    if (!disp_draw_buf1) {
        Serial.println("      FATAL: Buffer 1 alloc failed!");
        while(1) delay(100);
    }
    // Explicitly allocate buffer2 from PSRAM
    disp_draw_buf2 = (uint8_t*)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!disp_draw_buf2) {
        Serial.println("      PSRAM alloc 2 failed, trying internal...");
        disp_draw_buf2 = (uint8_t*)heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    }
    if (!disp_draw_buf2) {
        Serial.println("      FATAL: Buffer 2 alloc failed!");
        while (1) delay(100);
    }
    
    // Check where buffer1 was allocated
    bool in_psram1 = esp_ptr_external_ram(disp_draw_buf1);
    Serial.printf("      Buffer 1 in %s at 0x%08X\n", in_psram1 ? "PSRAM" : "internal", (uint32_t)disp_draw_buf1);
    // Check where buffer2 was allocated
    bool in_psram2 = esp_ptr_external_ram(disp_draw_buf2);
    Serial.printf("      Buffer 2 in %s at 0x%08X\n", in_psram2 ? "PSRAM" : "internal", (uint32_t)disp_draw_buf2);
    
    disp = lv_display_create(800, 480);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, disp_draw_buf1, disp_draw_buf2, buf_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    // Input device (GT911 touch)
    indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touch_read);
    
    Serial.printf("      PSRAM free: %u\n", ESP.getFreePsram());
    Serial.println("      OK");
    
    // Load UI
    Serial.println("[5/6] Loading UI...");
    ui_init();
    
    if (!ui_Screen1) {
        Serial.println("      FATAL: ui_Screen1 is NULL!");
        while(1) delay(100);
    }
    Serial.printf("      PSRAM free: %u\n", ESP.getFreePsram());
    Serial.println("      OK");
    
    // Simplify UI
    //Serial.println("[6/6] Applying stability fixes...");
    //simplify_ui();
    
#pragma endregion

    //-----------------------------

    // Bars Animation Speeds
    if (ui_OIL_PRESS_Bar) {
        lv_obj_set_style_anim_duration(ui_OIL_PRESS_Bar, 100, LV_PART_MAIN);  // 100ms animation
    }
    if (ui_OIL_TEMP_Bar) {
        lv_obj_set_style_anim_duration(ui_OIL_TEMP_Bar, 100, LV_PART_MAIN);
    }
    if (ui_FUEL_TRUST_Bar) {
        lv_obj_set_style_anim_duration(ui_FUEL_TRUST_Bar, 100, LV_PART_MAIN);
    }

    //-----------------------------

    // Create combined utility box (FPS, CPU, Brightness) - positioned at TOP-LEFT
    if (ui_Screen1) {
        // Box container
        utility_box = lv_obj_create(ui_Screen1);
        lv_obj_set_size(utility_box, 105, 65);
        lv_obj_align(utility_box, LV_ALIGN_TOP_LEFT, 5, 5);  // TOP-LEFT corner
        lv_obj_set_style_bg_color(utility_box, lv_color_hex(0x444444), 0);            // black
        lv_obj_set_style_bg_opa(utility_box, LV_OPA_70, 0);
        lv_obj_set_style_border_color(utility_box, lv_color_hex(0x444444), 0);        // black
        lv_obj_set_style_border_width(utility_box, 1, 0);
        lv_obj_set_style_radius(utility_box, 5, 0);
        lv_obj_set_style_pad_all(utility_box, 5, 0);
        lv_obj_add_flag(utility_box, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_remove_flag(utility_box, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(utility_box, utility_box_tap_cb, LV_EVENT_CLICKED, NULL);
        
        // Combined label for FPS, CPU, BRI
        utility_label = lv_label_create(utility_box);
        lv_label_set_text(utility_label, "--- FPS\n---% CPU\n---% BRI");
        lv_obj_set_style_text_color(utility_label, lv_color_hex(0xffff00), 0);
        lv_obj_set_style_text_font(utility_label, &lv_font_montserrat_14, 0);
        lv_obj_align(utility_label, LV_ALIGN_TOP_LEFT, 0, 0);
        
        Serial.println("[UI] Combined utility box created (tap=brightness, double-tap=hide)");
    }
    
    // Set initial brightness to 100%
    setBrightness(255);
    
    //-----------------------------

    // Initialize charts
    #if ENABLE_CHARTS
    if (ui_OIL_PRESS_CHART) {
        // Remove any existing series
        lv_chart_series_t * ser;
        while ((ser = lv_chart_get_series_next(ui_OIL_PRESS_CHART, NULL)) != NULL) {
            lv_chart_remove_series(ui_OIL_PRESS_CHART, ser);
        }
        
        // Configure chart
        lv_chart_set_type(ui_OIL_PRESS_CHART, LV_CHART_TYPE_BAR);
        lv_chart_set_point_count(ui_OIL_PRESS_CHART, 24);
        lv_chart_set_range(ui_OIL_PRESS_CHART, LV_CHART_AXIS_PRIMARY_Y, 0, 150);
        lv_chart_set_update_mode(ui_OIL_PRESS_CHART, LV_CHART_UPDATE_MODE_SHIFT);
        lv_chart_set_div_line_count(ui_OIL_PRESS_CHART, 0, 0);
        
        // Add series
        chart_series_oil_press = lv_chart_add_series(ui_OIL_PRESS_CHART, lv_color_hex(hexOrange), LV_CHART_AXIS_PRIMARY_Y);
        
        // Initialize with zeros
        for (int i = 0; i < 24; i++) {
            lv_chart_set_next_value(ui_OIL_PRESS_CHART, chart_series_oil_press, -100);
        }
        lv_chart_refresh(ui_OIL_PRESS_CHART);
        Serial.println("Oil pressure chart initialized (zeros)");
    }
    
    if (ui_OIL_TEMP_CHART) {
        // Remove any existing series
        lv_chart_series_t * ser;
        while ((ser = lv_chart_get_series_next(ui_OIL_TEMP_CHART, NULL)) != NULL) {
            lv_chart_remove_series(ui_OIL_TEMP_CHART, ser);
        }
        
        // Configure chart
        lv_chart_set_type(ui_OIL_TEMP_CHART, LV_CHART_TYPE_BAR);
        lv_chart_set_point_count(ui_OIL_TEMP_CHART, 24);
        lv_chart_set_range(ui_OIL_TEMP_CHART, LV_CHART_AXIS_PRIMARY_Y, W_TEMP_Min_F, W_TEMP_Max_F);
        lv_chart_set_update_mode(ui_OIL_TEMP_CHART, LV_CHART_UPDATE_MODE_SHIFT);
        lv_chart_set_div_line_count(ui_OIL_TEMP_CHART, 0, 0);
        
        // Add series
        chart_series_oil_temp = lv_chart_add_series(ui_OIL_TEMP_CHART, lv_color_hex(hexOrange), LV_CHART_AXIS_PRIMARY_Y);
        
        // Initialize with zeros
        for (int i = 0; i < 24; i++) {
            lv_chart_set_next_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, -100);
        }
        lv_chart_refresh(ui_OIL_TEMP_CHART);
        Serial.println("Oil temp chart initialized (zeros)");
    }
    #endif
    
    //-----------------------------

    // Force first render
    Serial.println("\nForcing initial render...");
    uint32_t t0 = millis();
    lv_refr_now(NULL);
    Serial.printf("Initial render took: %u ms\n", millis() - t0);
    Serial.printf("PSRAM free: %u\n", ESP.getFreePsram());
    
    Serial.println("\n========================================");
    Serial.println("         RUNNING - Watch for freezes");
    Serial.println("========================================\n");
}

//=================================================================

void loop() {
    static uint32_t frame_start = 0;
    frame_start = millis();

    static uint32_t last_tick = 0;
    static uint32_t last_status = 0;
    static uint32_t last_update = 0;

    static int oil_pressure = 0;
    static int oil_temp_p = 180;
    static int oil_temp_c = 180;
    static int fuel_trust = 100;

    OIL_PRESS_ValueCriticalRPM = OilPressMin_FromRPM();
    
    uint32_t now = millis();
    
    // LVGL tick
    if (last_tick == 0) last_tick = now;
    lv_tick_inc(now - last_tick);
    last_tick = now;
    
    loop_count++;
    
    // Status every 1 second - update utility box
    if (now - last_status >= 1000) {
        // Calculate CPU percentage
        int cpu_percent = (cpu_busy_time * 100) / 1000;
        if (cpu_percent > 100) cpu_percent = 100;
        
        // Update combined utility label (FPS, CPU, BRI)
        update_utility_label(flush_count, cpu_percent);
        
        Serial.printf("[STATUS] loops=%u flushes=%u cpu=%u%% heap=%u psram=%u\n",
                      loop_count, flush_count, (cpu_busy_time * 100) / 1000,
                      ESP.getFreeHeap(), ESP.getFreePsram());
        flush_count = 0;  // Reset for next second
        cpu_busy_time = 0;  // Reset CPU tracking
        last_status = now;
    }
    
    // UI updates
    #if ENABLE_UI_UPDATES
    if (now - last_update >= UPDATE_INTERVAL_MS) {
        update_count++;
        
        //-----------------------

        #pragma region oil pressure

        // Animate oil pressure - sine wave oscillation: 1s up, 1s down (ease-in-out)
        {
            static uint32_t oil_press_anim_start = 0;
            if (oil_press_anim_start == 0) oil_press_anim_start = now;
            
            uint32_t cycle_ms = 16000;  // 16 second full cycle (8s up + 8s down)
            uint32_t elapsed = (now - oil_press_anim_start) % cycle_ms;
            
            // Sine wave: starts at min, eases to max, eases back to min
            // sin() ranges -1 to 1, we shift to 0 to 1
            float angle = (float)elapsed / (float)cycle_ms * 2.0f * PI;
            float progress = (1.0f - cos(angle)) / 2.0f;  // 0 at start, 1 at middle, 0 at end
            
            oil_pressure = OIL_PRESS_Min_PSI + (int)(progress * (OIL_PRESS_Max_PSI - OIL_PRESS_Min_PSI) + 0.5f);
        }
        
        // Direct assignment for smooth sine animation (no additional smoothing needed)
        smooth_oil_pressure = (float)oil_pressure;
        int display_pressure = oil_pressure;
        // Update pressure bar
        if (ui_OIL_PRESS_Bar) {
            lv_bar_set_value(ui_OIL_PRESS_Bar, smooth_oil_pressure, LV_ANIM_ON);
            // Color: red if <20 or >120, else orange
            bool press_critical = (oil_pressure < 20) || (oil_pressure > OIL_PRESS_ValueCriticalAbsolute);
            lv_obj_set_style_bg_color(ui_OIL_PRESS_Bar, press_critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange), LV_PART_INDICATOR);
            
            // Show/hide critical label with blinking animation
            // Keep visible for 3 extra seconds after critical condition clears
            if (ui_OIL_PRESS_VALUE_CRITICAL_Label) {
                static bool was_critical = false;
                static bool critical_visible = false;     // Track if label is currently shown
                static uint32_t critical_exit_time = 0;   // When critical condition ended
                const uint32_t CRITICAL_LINGER_MS = 2000; // Keep visible for 2 seconds after exit
                
                if (press_critical) {
                    // Currently in critical state
                    critical_exit_time = 0;  // Reset exit timer
                    
                    if (!critical_visible) {
                        // Start showing label and blinking animation
                        critical_visible = true;
                        lv_obj_set_style_text_opa(ui_OIL_PRESS_VALUE_CRITICAL_Label, 255, LV_PART_MAIN);
                        
                        // Create blinking animation using LVGL's anim system
                        lv_anim_t anim;
                        lv_anim_init(&anim);
                        lv_anim_set_var(&anim, ui_OIL_PRESS_VALUE_CRITICAL_Label);
                        lv_anim_set_values(&anim, 0, 255);
                        lv_anim_set_duration(&anim, 200);               // 200ms per half-cycle (fast blink)
                        lv_anim_set_repeat_count(&anim, LV_ANIM_REPEAT_INFINITE);
                        lv_anim_set_playback_duration(&anim, 200);      // 200ms back
                        lv_anim_set_exec_cb(&anim, [](void* obj, int32_t val) {
                            lv_obj_t* label = (lv_obj_t*)obj;
                            // val goes 0->255->0, use midpoint (128) as threshold for equal time
                            if (val < 128) {
                                lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White
                            }
                            else {
                                lv_obj_set_style_text_color(label, lv_color_hex(0x960000), LV_PART_MAIN);  // Dark Red
                            }
                        });
                        lv_anim_start(&anim);
                    }
                }
                else {
                    // Not currently critical
                    if (was_critical && critical_exit_time == 0) {
                        // Just exited critical state - start the linger timer
                        critical_exit_time = now;
                    }
                    
                    // Check if linger period has elapsed
                    if (critical_visible && critical_exit_time > 0 && 
                        (now - critical_exit_time) >= CRITICAL_LINGER_MS) {
                        // Linger period over - hide the label
                        critical_visible = false;
                        lv_anim_delete(ui_OIL_PRESS_VALUE_CRITICAL_Label, NULL);
                        lv_obj_set_style_text_opa(ui_OIL_PRESS_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
                        critical_exit_time = 0;
                    }
                }
                
                was_critical = press_critical;
            }
        }

        // Smooth the animation for the oil pressure label (only updating when the displayed value actually changes)
        static int last_displayed_pressure = -1;
        int display_pressure_label = (int)(smooth_oil_pressure + 0.5f);
        // Update pressure label
        if (ui_OIL_PRESS_Value && display_pressure_label != last_displayed_pressure) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d PSI", display_pressure_label);
            lv_label_set_text(ui_OIL_PRESS_Value, buf);
            last_displayed_pressure = display_pressure_label;
        }

        #pragma endregion oil pressure

        //-----------------------
        
        #pragma region oil temp

        // Animate oil temp
        oil_temp_p += random(-2, 3);
        if (oil_temp_p > W_TEMP_Max_F) oil_temp_p = W_TEMP_Max_F;
        if (oil_temp_p < W_TEMP_Min_F) oil_temp_p = W_TEMP_Min_F;

        // Animate temp bar
        if (ui_OIL_TEMP_Bar) {
            lv_bar_set_value(ui_OIL_TEMP_Bar, oil_temp_p, LV_ANIM_ON);
            bool temp_red = (oil_temp_p > OIL_TEMP_ValueCriticalF);
            lv_obj_set_style_bg_color(ui_OIL_TEMP_Bar, temp_red ? lv_color_hex(hexRed) : lv_color_hex(hexOrange), LV_PART_INDICATOR);
        }
        
        //-----------------------

        // Animate temp labels
        if (ui_OIL_TEMP_Value_P) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [P]", oil_temp_p);
            lv_label_set_text(ui_OIL_TEMP_Value_P, buf);
        }
        if (ui_OIL_TEMP_Value_C) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [C]", oil_temp_p);
            lv_label_set_text(ui_OIL_TEMP_Value_C, buf);
        }

        #pragma endregion oil temp

        //-----------------------

        #pragma region 



        #pragma endregion

        //-----------------------
        
        #pragma region fuel trust

        // Animate fuel trust (0-100% range)
        static uint32_t fuel_trust_dip_time = 0;
        static bool fuel_trust_dipping = false;
        uint32_t now = millis();

        if (!fuel_trust_dipping) {
            // Normal state: hover around 95-100
            fuel_trust = 95 + random(0, 6);  // 95-100

            // Random chance to start a dip every 5-20 seconds
            if (fuel_trust_dip_time == 0) {
                fuel_trust_dip_time = now + random(5000, 20001);
            }

            if (now >= fuel_trust_dip_time) {
                fuel_trust_dipping = true;
                fuel_trust = 40 + random(0, 35);  // Dip to 40-75
            }
        }
        else {
            // Dipping state: quickly recover back to normal
            fuel_trust += random(10, 20);  // Fast recovery

            if (fuel_trust >= 95) {
                fuel_trust = 95 + random(0, 6);
                fuel_trust_dipping = false;
                fuel_trust_dip_time = now + random(5000, 20001);  // Schedule next dip
            }
        }

        // Smooth the animation for the fuel trust
        smooth_fuel_trust = smooth_fuel_trust * (1.0f - SMOOTH_FACTOR) + fuel_trust * SMOOTH_FACTOR;
        int display_fuel_trust = (int)(smooth_fuel_trust + 0.5f);

        // Update fuel trust bar and label
        if (ui_FUEL_TRUST_Bar) {
            lv_bar_set_value(ui_FUEL_TRUST_Bar, display_fuel_trust, LV_ANIM_ON);
            // Color: red if <50, else orange
            bool press_red = (fuel_trust < 50);
            lv_obj_set_style_bg_color(ui_FUEL_TRUST_Bar,
                press_red ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
        if (ui_FUEL_TRUST_Value) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d %%", fuel_trust);
            lv_label_set_text(ui_FUEL_TRUST_Value, buf);
        }

        #pragma endregion fuel trust

        //----------------------------------------------
        
        // Accumulate samples for chart averaging
        #if ENABLE_CHARTS
        pressure_sum += oil_pressure;
        pressure_samples++;
        temp_sum += oil_temp_p;
        temp_samples++;
        
        // Initialize bucket start times
        if (pressure_bucket_start == 0) pressure_bucket_start = now;
        if (temp_bucket_start == 0) temp_bucket_start = now;
        
        // Push to pressure chart every 5 seconds
        if ((now - pressure_bucket_start) >= CHART_BUCKET_MS && chart_series_oil_press) {
            int32_t avg = (pressure_samples > 0) ? (pressure_sum / pressure_samples) : 0;
            if (avg < 0) avg = 0;
            if (avg > 150) avg = 150;
            lv_chart_set_next_value(ui_OIL_PRESS_CHART, chart_series_oil_press, avg);
            pressure_sum = 0;
            pressure_samples = 0;
            pressure_bucket_start = now;
        }
        
        // Push to temp chart every 5 seconds
        if ((now - temp_bucket_start) >= CHART_BUCKET_MS && chart_series_oil_temp) {
            int32_t avg = (temp_samples > 0) ? (temp_sum / temp_samples) : 0;
            if (avg < 0) avg = 0;
            if (avg > 140) avg = 140;
            lv_chart_set_next_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, avg);
            temp_sum = 0;
            temp_samples = 0;
            temp_bucket_start = now;
        }
        #endif
        
        last_update = now;
    }
    #endif
    
    // LVGL handler with timing
    uint32_t t0 = micros();
    lv_timer_handler();
    uint32_t dt = micros() - t0;
    
    // Warn if LVGL took too long
    if (dt > 100000) {
        Serial.printf("[WARN] lv_timer_handler took %u us!\n", dt);
    }
    
    uint32_t elapsed = millis() - frame_start;
    cpu_busy_time += elapsed;  // Track time spent working
    if (elapsed < FRAME_TIME_MS) {
        delay(FRAME_TIME_MS - elapsed);
    }
}
