//-----------------------------------------------------------------

/*
 * 370zMonitor v4.1 - Data Provider Architecture
 * Supports Demo Mode (animated values) and Live Mode (sensors data/OBD data)
 * ESP32-S3 with PSRAM, LVGL, GT911 Touch
 *
 * v4.1 Changes:
 * - Fixed live mode startup (shows "---" until data arrives)
 * - Fixed demo mode (all values use simple sine wave oscillation)
 * - Added unit conversion (C/F for temps, PSI/Bar/kPa for pressure)
 * - Added tap-to-cycle units with persistent preferences
 *
 * Firmware download mode: Hold BOOT, then RESET
 * USB Mass Storage Mode: Hold BOOT button during power-on to enter [press RESET, then BOOT right away] (USB drive mode (SD card accessible via USB-C))
 */

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <lvgl.h>
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>
#include <TAMC_GT911.h>  // Touch controller
#include <SD.h>
#include <SPI.h>
#include <Preferences.h>  // For persistent unit preferences
#include "USB.h"        // ESP32-S3 native USB
#include "USBMSC.h"     // USB Mass Storage Class

 //-----------------------------------------------------------------

 // ===== CRITICAL: Configure ESP32 to prefer PSRAM for malloc =====
__attribute__((constructor)) void configurePSRAM() {
    heap_caps_malloc_extmem_enable(32 * 1024);
}

//-----------------------------------------------------------------

// ===== FEATURE FLAGS =====
#define ENABLE_TOUCH        1
#define ENABLE_UI_UPDATES   1   // Enable bar/label updates
#define ENABLE_CHARTS       1   // Enable charts
#define ENABLE_SD_LOGGING   1   // Enable SD card data logging
#define ENABLE_USB_MSC      1   // Enable USB Mass Storage mode (hold BOOT at startup)
#define UPDATE_INTERVAL_MS  150 // Update every 150ms

// USB MSC Configuration
#define USB_MSC_BOOT_PIN    0   // GPIO0 = BOOT button on most ESP32-S3 boards

//-----------------------------------------------------------------

// ===== UNIT TYPES =====
enum TempUnit { TEMP_FAHRENHEIT = 0, TEMP_CELSIUS = 1 };
enum PressureUnit { PRESS_PSI = 0, PRESS_BAR = 1, PRESS_KPA = 2 };

// Forward declarations to prevent Arduino preprocessor from generating incorrect prototypes
float tempToInternal(float value, TempUnit source_unit);
float tempToDisplay(float value_f, TempUnit display_unit);
float pressToInternal(float value, PressureUnit source_unit);
float pressToDisplay(float value_psi, PressureUnit display_unit);
const char* getTempUnitStr(TempUnit unit);
const char* getPressureUnitStr(PressureUnit unit);

// Preferences for persistent storage
Preferences g_prefs;

// Current display units (loaded from/saved to flash)
// Per-gauge temperature units - each gauge can be set independently
static PressureUnit g_pressure_unit = PRESS_PSI;
static TempUnit g_oil_temp_unit = TEMP_FAHRENHEIT;
static TempUnit g_water_temp_unit = TEMP_FAHRENHEIT;
static TempUnit g_trans_temp_unit = TEMP_FAHRENHEIT;
static TempUnit g_steer_temp_unit = TEMP_FAHRENHEIT;
static TempUnit g_diff_temp_unit = TEMP_FAHRENHEIT;

// Source data unit assumptions (what the sensors output)
// These can be changed if your sensors output different units
static TempUnit g_sensor_temp_unit = TEMP_CELSIUS;      // Most temp sensors output Celsius
static PressureUnit g_sensor_pressure_unit = PRESS_PSI; // Most pressure sensors output PSI

//-----------------------------------------------------------------

// ===== UNIT CONVERSION FUNCTIONS =====

// Temperature conversions
float celsiusToFahrenheit(float c) { return c * 9.0f / 5.0f + 32.0f; }
float fahrenheitToCelsius(float f) { return (f - 32.0f) * 5.0f / 9.0f; }

// Convert from source unit to internal storage (Fahrenheit)
// Uses int for parameter to avoid Arduino preprocessor issues
float tempToInternal(float value, TempUnit source_unit) {
    if (source_unit == TEMP_CELSIUS) {
        return celsiusToFahrenheit(value);
    }
    return value; // Already Fahrenheit
}

// Convert from internal (Fahrenheit) to display unit
float tempToDisplay(float value_f, TempUnit display_unit) {
    if (display_unit == TEMP_CELSIUS) {
        return fahrenheitToCelsius(value_f);
    }
    return value_f; // Already Fahrenheit
}

// Pressure conversions
float psiToBar(float psi) { return psi * 0.0689476f; }
float psiToKpa(float psi) { return psi * 6.89476f; }
float barToPsi(float bar) { return bar / 0.0689476f; }
float kpaToPsi(float kpa) { return kpa / 6.89476f; }

// Convert from source unit to internal storage (PSI)
float pressToInternal(float value, PressureUnit source_unit) {
    switch (source_unit) {
    case PRESS_BAR: return barToPsi(value);
    case PRESS_KPA: return kpaToPsi(value);
    default: return value; // Already PSI
    }
}

// Convert from internal (PSI) to display unit
float pressToDisplay(float value_psi, PressureUnit display_unit) {
    switch (display_unit) {
    case PRESS_BAR: return psiToBar(value_psi);
    case PRESS_KPA: return psiToKpa(value_psi);
    default: return value_psi; // Already PSI
    }
}

// Get unit suffix strings
const char* getTempUnitStr(TempUnit unit) {
    return (unit == TEMP_CELSIUS) ? "C" : "F";
}

const char* getPressureUnitStr(PressureUnit unit) {
    switch (unit) {
    case PRESS_BAR: return "BAR";
    case PRESS_KPA: return "kPa";
    default: return "PSI";
    }
}

// Save/Load unit preferences - stores 6 independent unit settings
void saveUnitPreferences() {
    g_prefs.begin("units", false);
    g_prefs.putUChar("press", (uint8_t)g_pressure_unit);
    g_prefs.putUChar("oil_t", (uint8_t)g_oil_temp_unit);
    g_prefs.putUChar("water_t", (uint8_t)g_water_temp_unit);
    g_prefs.putUChar("trans_t", (uint8_t)g_trans_temp_unit);
    g_prefs.putUChar("steer_t", (uint8_t)g_steer_temp_unit);
    g_prefs.putUChar("diff_t", (uint8_t)g_diff_temp_unit);
    g_prefs.end();
    Serial.println("[PREFS] Units saved");
}

void loadUnitPreferences() {
    g_prefs.begin("units", true);
    g_pressure_unit = (PressureUnit)g_prefs.getUChar("press", PRESS_PSI);
    g_oil_temp_unit = (TempUnit)g_prefs.getUChar("oil_t", TEMP_FAHRENHEIT);
    g_water_temp_unit = (TempUnit)g_prefs.getUChar("water_t", TEMP_FAHRENHEIT);
    g_trans_temp_unit = (TempUnit)g_prefs.getUChar("trans_t", TEMP_FAHRENHEIT);
    g_steer_temp_unit = (TempUnit)g_prefs.getUChar("steer_t", TEMP_FAHRENHEIT);
    g_diff_temp_unit = (TempUnit)g_prefs.getUChar("diff_t", TEMP_FAHRENHEIT);
    g_prefs.end();
    Serial.printf("[PREFS] Loaded: Press=%s, OilT=%s, WaterT=%s, TransT=%s, SteerT=%s, DiffT=%s\n",
        getPressureUnitStr(g_pressure_unit),
        getTempUnitStr(g_oil_temp_unit), getTempUnitStr(g_water_temp_unit),
        getTempUnitStr(g_trans_temp_unit), getTempUnitStr(g_steer_temp_unit),
        getTempUnitStr(g_diff_temp_unit));
}

//-----------------------------------------------------------------

// ===== DATA PROVIDER ARCHITECTURE =====
// This structure holds all vehicle data from either demo or real sources
// The UI layer reads from this - it doesn't care where the data comes from
// All values stored internally in base units: Fahrenheit for temp, PSI for pressure

struct VehicleData {
    // Oil Pressure (stored in PSI internally)
    int oil_pressure_psi;
    bool oil_pressure_valid;

    // Oil Temperature (stored in Fahrenheit internally)
    int oil_temp_pan_f;
    int oil_temp_cooled_f;
    bool oil_temp_valid;

    // Water/Coolant Temperature (stored in Fahrenheit internally)
    int water_temp_hot_f;
    int water_temp_cooled_f;
    bool water_temp_valid;

    // Transmission Temperature (stored in Fahrenheit internally)
    int trans_temp_hot_f;
    int trans_temp_cooled_f;
    bool trans_temp_valid;

    // Power Steering Temperature (stored in Fahrenheit internally)
    int steer_temp_hot_f;
    int steer_temp_cooled_f;
    bool steer_temp_valid;

    // Differential Temperature (stored in Fahrenheit internally)
    int diff_temp_hot_f;
    int diff_temp_cooled_f;
    bool diff_temp_valid;

    // Fuel Trust (confidence percentage)
    int fuel_trust_percent;
    bool fuel_trust_valid;

    // OBD Data
    int rpm;
    bool rpm_valid;

    // Flag to track if ANY valid data has ever been received
    // Used to determine if UI should show "---" or actual values
    bool has_received_data;
};

// Global vehicle data - updated by data providers, read by UI
static VehicleData g_vehicle_data = { 0 };

// Demo mode flag - toggled by 5-second hold on Utility block
static bool g_demo_mode = false;  // Start in LIVE mode (default)

// Reset all vehicle data to invalid/zero state
void resetVehicleData() {
    memset(&g_vehicle_data, 0, sizeof(g_vehicle_data));
    // All _valid flags are now false
    // has_received_data is now false
}

// Forward declarations for reset functions (defined after variables)
void resetSmoothingState();
void resetDemoState();
void updateTapBoxVisibility();
void resetUIElements();

//-----------------------------------------------------------------

// CH422 IO Expander
#define CH422_ADDR_SYSTEM 0x24
#define CH422_ADDR_IOWR   0x38
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
static uint8_t g_brightness_level = 255;

//-----------------------------------------------------------------

#define TARGET_FPS 50
#define FRAME_TIME_MS (1000 / TARGET_FPS)

//-----------------------------------------------------------------

#pragma region Gauges Configuration

// OIL PRESS: 0-150 PSI, Critical: <=10 PSI per 1000 RPM, >=120 PSI
int OIL_PRESS_Min_PSI = 0;
int OIL_PRESS_Max_PSI = 150;
int OIL_PRESS_ValueCriticalAbsolute = 120;
int OIL_PRESS_ValueCriticalLow = 10;

// OIL TEMP: 150-300°F, Critical: >=260°F
int OIL_TEMP_Min_F = 150;
int OIL_TEMP_Max_F = 300;
int OIL_TEMP_ValueCriticalF = 260;

// WATER TEMP: 100-260°F, Critical: >=230°F
int W_TEMP_Min_F = 100;
int W_TEMP_Max_F = 260;
int W_TEMP_ValueCritical_F = 230;

// TRANS TEMP: 80-280°F, Critical: >=230°F
int TRAN_TEMP_Min_F = 80;
int TRAN_TEMP_Max_F = 280;
int TRAN_TEMP_ValueCritical_F = 230;

// STEER TEMP: 60-300°F, Critical: >=230°F
int STEER_TEMP_Min_F = 60;
int STEER_TEMP_Max_F = 300;
int STEER_TEMP_ValueCritical_F = 230;

// DIFF TEMP: 60-320°F, Critical: >=270°F
int DIFF_TEMP_Min_F = 60;
int DIFF_TEMP_Max_F = 320;
int DIFF_TEMP_ValueCritical_F = 270;

// FUEL TRUST: 0-100%, Critical: <=50%
int FUEL_TRUST_Min = 0;
int FUEL_TRUST_Max = 100;
int FUEL_TRUST_ValueCritical = 50;

#pragma endregion Gauges Configuration

//-----------------------------------------------------------------

#pragma region Animation/Smoothing

// Smoothed display values (for UI animation)
// These get reset when switching modes
static float smooth_oil_pressure = -1.0f;  // -1 = uninitialized/no data
static float smooth_oil_temp_f = -1.0f;
static float smooth_water_temp_f = -1.0f;
static float smooth_trans_temp_f = -1.0f;
static float smooth_steer_temp_f = -1.0f;
static float smooth_diff_temp_f = -1.0f;
static float smooth_fuel_trust = -1.0f;

// Smoothing factor: 0.3 = responsive, 0.1 = very smooth
#define SMOOTH_FACTOR 0.3f

// Reset smoothing variables (must be after variable declarations)
void resetSmoothingState() {
    smooth_oil_pressure = -1.0f;
    smooth_oil_temp_f = -1.0f;
    smooth_water_temp_f = -1.0f;
    smooth_trans_temp_f = -1.0f;
    smooth_steer_temp_f = -1.0f;
    smooth_diff_temp_f = -1.0f;
    smooth_fuel_trust = -1.0f;
}

#pragma endregion Animation/Smoothing

//-----------------------------------------------------------------

#pragma region Colors

static int hexRed = 0xFF0000;
static int hexOrange = 0xFF4619;
static int hexGreen = 0x00FF00;

#pragma endregion Colors

//-----------------------------------------------------------------

#pragma region UI Objects

#include "ui.h"

//OIL PRESS
extern lv_obj_t* ui_OIL_PRESS_Bar;
extern lv_obj_t* ui_OIL_PRESS_CHART;
extern lv_obj_t* ui_OIL_PRESS_Value;
extern lv_obj_t* ui_OIL_PRESS_VALUE_CRITICAL_Label;

//OIL TEMP [Oil pan/C]
extern lv_obj_t* ui_OIL_TEMP_Bar;
extern lv_obj_t* ui_OIL_TEMP_CHART;
extern lv_obj_t* ui_OIL_TEMP_Value_P;
extern lv_obj_t* ui_OIL_TEMP_Value_C;
extern lv_obj_t* ui_OIL_TEMP_VALUE_CRITICAL_Label;

//WATER TEMP [H/C]
extern lv_obj_t* ui_W_TEMP_Bar;
extern lv_obj_t* ui_W_TEMP_CHART;
extern lv_obj_t* ui_W_TEMP_Value_H;
extern lv_obj_t* ui_W_TEMP_Value_c;
extern lv_obj_t* ui_W_TEMP_VALUE_CRITICAL_Label;

//TRAN TEMP [H/C]
extern lv_obj_t* ui_TRAN_TEMP_Bar;
extern lv_obj_t* ui_TRAN_TEMP_CHART;
extern lv_obj_t* ui_TRAN_TEMP_Value_H;
extern lv_obj_t* ui_TRAN_TEMP_Value_C;
extern lv_obj_t* ui_TRAN_TEMP_VALUE_CRITICAL_Label;

//STEER TEMP [H/C]
extern lv_obj_t* ui_STEER_TEMP_Bar;
extern lv_obj_t* ui_STEER_TEMP_CHART;
extern lv_obj_t* ui_STEER_TEMP_Value_H;
extern lv_obj_t* ui_STEER_TEMP_Value_C;
extern lv_obj_t* ui_STEER_TEMP_VALUE_CRITICAL_Label;

//DIFF TEMP [H/C]
extern lv_obj_t* ui_DIFF_TEMP_Bar;
extern lv_obj_t* ui_DIFF_TEMP_CHART;
extern lv_obj_t* ui_DIFF_TEMP_Value_H;
extern lv_obj_t* ui_DIFF_TEMP_Value_C;
extern lv_obj_t* ui_DIFF_TEMP_VALUE_CRITICAL_Label;

//FUEL TRUST
extern lv_obj_t* ui_FUEL_TRUST_Bar;
extern lv_obj_t* ui_FUEL_TRUST_CHART;
extern lv_obj_t* ui_FUEL_TRUST_Value;
extern lv_obj_t* ui_FUEL_TRUST_VALUE_CRITICAL_Label;

#pragma endregion UI Objects

//-----------------------------------------------------------------

extern lv_obj_t* ui_Screen1;

// Display
Arduino_ESP32RGBPanel* rgbpanel = new Arduino_ESP32RGBPanel(
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
Arduino_RGB_Display* gfx = new Arduino_RGB_Display(800, 480, rgbpanel, 0, true);

//-----------------------------------------------------------------

// LVGL
#define LVGL_BUFFER_SIZE (800 * 69)  // 69 lines - will go to PSRAM | 480 ÷ 7 ≈ 69 lines per strip
static lv_display_t* disp;
static lv_indev_t* indev;
static uint8_t* disp_draw_buf1;
static uint8_t* disp_draw_buf2;

//-----------------------------------------------------------------

// GT911 Touch Controller
TAMC_GT911 touch = TAMC_GT911(I2C_SDA, I2C_SCL, TOUCH_INT_PIN, -1, 800, 480);

//-----------------------------------------------------------------

// Counters and tracking
static uint32_t loop_count = 0;
static uint32_t flush_count = 0;
static uint32_t update_count = 0;
static uint32_t cpu_busy_time = 0;

// Touch controller state
static uint32_t consecutive_invalid = 0;
static uint32_t last_touch_reset = 0;

// Utility box
static lv_obj_t* utility_box = NULL;
static lv_obj_t* g_dim_overlay = NULL;
static lv_timer_t* g_util_single_tap_timer = NULL;
static lv_obj_t* utility_label = NULL;
static lv_obj_t* mode_indicator = NULL;  // Shows "DEMO" or "LIVE"
static bool utilities_visible = false;  // Start hidden, double-tap to reveal

// Long press tracking for demo mode toggle
static uint32_t g_utility_press_start = 0;
static bool g_utility_long_press_triggered = false;
#define DEMO_MODE_TOGGLE_HOLD_MS 5000  // 5 seconds to toggle demo mode

// Chart series
static lv_chart_series_t* chart_series_oil_press = NULL;
static lv_chart_series_t* chart_series_oil_temp = NULL;
static lv_chart_series_t* chart_series_water_temp = NULL;
static lv_chart_series_t* chart_series_transmission_temp = NULL;
static lv_chart_series_t* chart_series_steering_temp = NULL;
static lv_chart_series_t* chart_series_differencial_temp = NULL;
static lv_chart_series_t* chart_series_fuel_trust = NULL;

// Chart history
// oil_press
static int32_t oil_pressure_sum = 0;
static uint32_t oil_pressure_samples = 0;
static uint32_t oil_pressure_bucket_start = 0;
// oil_temp
static int32_t oil_temp_sum = 0;
static uint32_t oil_temp_samples = 0;
static uint32_t oil_temp_bucket_start = 0;
// water_temp
static int32_t water_temp_sum = 0;
static uint32_t water_temp_samples = 0;
static uint32_t water_temp_bucket_start = 0;
// transmission_temp
static int32_t transmission_temp_sum = 0;
static uint32_t transmission_temp_samples = 0;
static uint32_t transmission_temp_bucket_start = 0;
// steering_temp
static int32_t steering_temp_sum = 0;
static uint32_t steering_temp_samples = 0;
static uint32_t steering_temp_bucket_start = 0;
// differencial_temp
static int32_t differencial_temp_sum = 0;
static uint32_t differencial_temp_samples = 0;
static uint32_t differencial_temp_bucket_start = 0;
// fuel_trust
static int32_t fuel_trust_sum = 0;
static uint32_t fuel_trust_samples = 0;
static uint32_t fuel_trust_start = 0;

#define CHART_BUCKET_MS 5000

#define CHART_POINTS 24
static int32_t oil_press_history[CHART_POINTS] = { 0 };
static int32_t oil_temp_history[CHART_POINTS] = { 0 };
static int32_t water_temp_history[CHART_POINTS] = { 0 };
static int32_t transmission_temp_history[CHART_POINTS] = { 0 };
static int32_t steering_temp_history[CHART_POINTS] = { 0 };
static int32_t differencial_temp_history[CHART_POINTS] = { 0 };
static int32_t fuel_trust_history[CHART_POINTS] = { 0 };

// Critical bar blinking
static bool g_critical_blink_phase = false;
static uint32_t g_last_blink_toggle = 0;
#define CHART_BLINK_INTERVAL_MS 200

// Tap box objects for reliable touch detection
static lv_obj_t* tap_box_oil_press = NULL;
static lv_obj_t* tap_box_oil_temp = NULL;
static lv_obj_t* tap_box_water_temp = NULL;
static lv_obj_t* tap_box_trans_temp = NULL;
static lv_obj_t* tap_box_steer_temp = NULL;
static lv_obj_t* tap_box_diff_temp = NULL;

// Debug mode for tap boxes (set to 0 for transparent in production)
#define TAP_BOX_DEBUG 1  // Set to 0 to hide tap boxes in demo mode
#define TAP_BOX_OPACITY LV_OPA_COVER  // No expensive transparency - fully visible or invisible

//=================================================================
// UI RESET FUNCTIONS (defined after all variables are declared)
//=================================================================

// Reset all UI elements to default/empty state
void resetUIElements() {
    // Reset all bars to 0 (no animation for instant reset)
    if (ui_OIL_PRESS_Bar) lv_bar_set_value(ui_OIL_PRESS_Bar, 0, LV_ANIM_OFF);
    if (ui_OIL_TEMP_Bar) lv_bar_set_value(ui_OIL_TEMP_Bar, 0, LV_ANIM_OFF);
    if (ui_W_TEMP_Bar) lv_bar_set_value(ui_W_TEMP_Bar, 0, LV_ANIM_OFF);
    if (ui_TRAN_TEMP_Bar) lv_bar_set_value(ui_TRAN_TEMP_Bar, 0, LV_ANIM_OFF);
    if (ui_STEER_TEMP_Bar) lv_bar_set_value(ui_STEER_TEMP_Bar, 0, LV_ANIM_OFF);
    if (ui_DIFF_TEMP_Bar) lv_bar_set_value(ui_DIFF_TEMP_Bar, 0, LV_ANIM_OFF);
    if (ui_FUEL_TRUST_Bar) lv_bar_set_value(ui_FUEL_TRUST_Bar, 0, LV_ANIM_OFF);

    char buf[32];

    // Oil Pressure
    snprintf(buf, sizeof(buf), "--- %s", getPressureUnitStr(g_pressure_unit));
    if (ui_OIL_PRESS_Value) lv_label_set_text(ui_OIL_PRESS_Value, buf);

    // Oil Temp - uses g_oil_temp_unit
    const char* oilTempUnit = getTempUnitStr(g_oil_temp_unit);
    snprintf(buf, sizeof(buf), "---°%s [P]", oilTempUnit);
    if (ui_OIL_TEMP_Value_P) lv_label_set_text(ui_OIL_TEMP_Value_P, buf);
    snprintf(buf, sizeof(buf), "---°%s [C]", oilTempUnit);
    if (ui_OIL_TEMP_Value_C) lv_label_set_text(ui_OIL_TEMP_Value_C, buf);

    // Water Temp - uses g_water_temp_unit
    const char* waterTempUnit = getTempUnitStr(g_water_temp_unit);
    snprintf(buf, sizeof(buf), "---°%s [H]", waterTempUnit);
    if (ui_W_TEMP_Value_H) lv_label_set_text(ui_W_TEMP_Value_H, buf);
    snprintf(buf, sizeof(buf), "---°%s [C]", waterTempUnit);
    if (ui_W_TEMP_Value_C) lv_label_set_text(ui_W_TEMP_Value_C, buf);

    // Trans Temp - uses g_trans_temp_unit
    const char* transTempUnit = getTempUnitStr(g_trans_temp_unit);
    snprintf(buf, sizeof(buf), "---°%s [H]", transTempUnit);
    if (ui_TRAN_TEMP_Value_H) lv_label_set_text(ui_TRAN_TEMP_Value_H, buf);
    snprintf(buf, sizeof(buf), "---°%s [C]", transTempUnit);
    if (ui_TRAN_TEMP_Value_C) lv_label_set_text(ui_TRAN_TEMP_Value_C, buf);

    // Steer Temp - uses g_steer_temp_unit
    const char* steerTempUnit = getTempUnitStr(g_steer_temp_unit);
    snprintf(buf, sizeof(buf), "---°%s [H]", steerTempUnit);
    if (ui_STEER_TEMP_Value_H) lv_label_set_text(ui_STEER_TEMP_Value_H, buf);
    snprintf(buf, sizeof(buf), "---°%s [C]", steerTempUnit);
    if (ui_STEER_TEMP_Value_C) lv_label_set_text(ui_STEER_TEMP_Value_C, buf);

    // Diff Temp - uses g_diff_temp_unit
    const char* diffTempUnit = getTempUnitStr(g_diff_temp_unit);
    snprintf(buf, sizeof(buf), "---°%s [H]", diffTempUnit);
    if (ui_DIFF_TEMP_Value_H) lv_label_set_text(ui_DIFF_TEMP_Value_H, buf);
    snprintf(buf, sizeof(buf), "---°%s [C]", diffTempUnit);
    if (ui_DIFF_TEMP_Value_C) lv_label_set_text(ui_DIFF_TEMP_Value_C, buf);

    if (ui_FUEL_TRUST_Value) lv_label_set_text(ui_FUEL_TRUST_Value, "--- %");

    // Reset oil pressure label styling to default
    if (ui_OIL_PRESS_Value) {
        lv_obj_set_style_text_color(ui_OIL_PRESS_Value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_OIL_PRESS_Value, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_OIL_PRESS_Value, 0, 0);
    }

    // Reset oil temp label styling to default
    if (ui_OIL_TEMP_Value_P) {
        lv_obj_set_style_text_color(ui_OIL_TEMP_Value_P, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_OIL_TEMP_Value_P, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_OIL_TEMP_Value_P, 0, 0);
    }
    if (ui_OIL_TEMP_Value_C) {
        lv_obj_set_style_text_color(ui_OIL_TEMP_Value_C, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_OIL_TEMP_Value_C, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_OIL_TEMP_Value_C, 0, 0);
    }

    // Reset water temp label styling to default
    if (ui_W_TEMP_Value_H) {
        lv_obj_set_style_text_color(ui_W_TEMP_Value_H, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_W_TEMP_Value_H, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_W_TEMP_Value_H, 0, 0);
    }
    if (ui_W_TEMP_Value_C) {
        lv_obj_set_style_text_color(ui_W_TEMP_Value_C, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_W_TEMP_Value_C, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_W_TEMP_Value_C, 0, 0);
    }

    // Reset trans temp label styling to default
    if (ui_TRAN_TEMP_Value_H) {
        lv_obj_set_style_text_color(ui_TRAN_TEMP_Value_H, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_TRAN_TEMP_Value_H, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_TRAN_TEMP_Value_H, 0, 0);
    }
    if (ui_TRAN_TEMP_Value_C) {
        lv_obj_set_style_text_color(ui_TRAN_TEMP_Value_C, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_TRAN_TEMP_Value_C, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_TRAN_TEMP_Value_C, 0, 0);
    }

    // Reset steer temp label styling to default
    if (ui_STEER_TEMP_Value_H) {
        lv_obj_set_style_text_color(ui_STEER_TEMP_Value_H, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_STEER_TEMP_Value_H, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_STEER_TEMP_Value_H, 0, 0);
    }
    if (ui_STEER_TEMP_Value_C) {
        lv_obj_set_style_text_color(ui_STEER_TEMP_Value_C, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_STEER_TEMP_Value_C, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_STEER_TEMP_Value_C, 0, 0);
    }

    // Reset diff temp label styling to default
    if (ui_DIFF_TEMP_Value_H) {
        lv_obj_set_style_text_color(ui_DIFF_TEMP_Value_H, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_DIFF_TEMP_Value_H, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_DIFF_TEMP_Value_H, 0, 0);
    }
    if (ui_DIFF_TEMP_Value_C) {
        lv_obj_set_style_text_color(ui_DIFF_TEMP_Value_C, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_DIFF_TEMP_Value_C, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_DIFF_TEMP_Value_C, 0, 0);
    }

    // Reset fuel trust label styling to default
    if (ui_FUEL_TRUST_Value) {
        lv_obj_set_style_text_color(ui_FUEL_TRUST_Value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_FUEL_TRUST_Value, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_FUEL_TRUST_Value, 0, 0);
    }

    // Hide all critical labels
    lv_obj_t* critical_labels[] = {
        ui_OIL_PRESS_VALUE_CRITICAL_Label,
        ui_OIL_TEMP_VALUE_CRITICAL_Label,
        ui_W_TEMP_VALUE_CRITICAL_Label,
        ui_TRAN_TEMP_VALUE_CRITICAL_Label,
        ui_STEER_TEMP_VALUE_CRITICAL_Label,
        ui_DIFF_TEMP_VALUE_CRITICAL_Label,
        ui_FUEL_TRUST_VALUE_CRITICAL_Label
    };

    for (int i = 0; i < 7; i++) {
        if (critical_labels[i]) {
            lv_anim_delete(critical_labels[i], NULL);
            lv_obj_set_style_text_opa(critical_labels[i], 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(critical_labels[i], 0, LV_PART_MAIN);
        }
    }

    // Reset all bar colors to default orange
    if (ui_OIL_PRESS_Bar) lv_obj_set_style_bg_color(ui_OIL_PRESS_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_OIL_TEMP_Bar) lv_obj_set_style_bg_color(ui_OIL_TEMP_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_W_TEMP_Bar) lv_obj_set_style_bg_color(ui_W_TEMP_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_TRAN_TEMP_Bar) lv_obj_set_style_bg_color(ui_TRAN_TEMP_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_STEER_TEMP_Bar) lv_obj_set_style_bg_color(ui_STEER_TEMP_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_DIFF_TEMP_Bar) lv_obj_set_style_bg_color(ui_DIFF_TEMP_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_FUEL_TRUST_Bar) lv_obj_set_style_bg_color(ui_FUEL_TRUST_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
}

// Reset chart data and history
void resetCharts() {
#if ENABLE_CHARTS
    // Reset oil pressure chart
    if (ui_OIL_PRESS_CHART && chart_series_oil_press) {
        for (int i = 0; i < CHART_POINTS; i++) {
            oil_press_history[i] = 0;
        }
        // Clear all points by setting to below range
        lv_chart_set_all_value(ui_OIL_PRESS_CHART, chart_series_oil_press, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_OIL_PRESS_CHART);
    }
    // Reset oil temp chart
    if (ui_OIL_TEMP_CHART && chart_series_oil_temp) {
        for (int i = 0; i < CHART_POINTS; i++) {
            oil_temp_history[i] = 0;
        }
        lv_chart_set_all_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_OIL_TEMP_CHART);
    }
    // Reset water temp chart
    if (ui_W_TEMP_CHART && chart_series_water_temp) {
        for (int i = 0; i < CHART_POINTS; i++) {
            water_temp_history[i] = 0;
        }
        lv_chart_set_all_value(ui_W_TEMP_CHART, chart_series_water_temp, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_W_TEMP_CHART);
    }
    // Reset transmission temp chart
    if (ui_TRAN_TEMP_CHART && chart_series_transmission_temp) {
        for (int i = 0; i < CHART_POINTS; i++) {
            transmission_temp_history[i] = 0;
        }
        lv_chart_set_all_value(ui_TRAN_TEMP_CHART, chart_series_transmission_temp, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_TRAN_TEMP_CHART);
    }
    // Reset steering temp chart
    if (ui_STEER_TEMP_CHART && chart_series_steering_temp) {
        for (int i = 0; i < CHART_POINTS; i++) {
            steering_temp_history[i] = 0;
        }
        lv_chart_set_all_value(ui_STEER_TEMP_CHART, chart_series_steering_temp, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_STEER_TEMP_CHART);
    }
    // Reset differencial temp chart
    if (ui_DIFF_TEMP_CHART && chart_series_differencial_temp) {
        for (int i = 0; i < CHART_POINTS; i++) {
            differencial_temp_history[i] = 0;
        }
        lv_chart_set_all_value(ui_DIFF_TEMP_CHART, chart_series_differencial_temp, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_DIFF_TEMP_CHART);
    }
    // Reset fuel trust chart
    if (ui_FUEL_TRUST_CHART && chart_series_fuel_trust) {
        for (int i = 0; i < CHART_POINTS; i++) {
            fuel_trust_history[i] = 0;
        }
        lv_chart_set_all_value(ui_FUEL_TRUST_CHART, chart_series_fuel_trust, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_FUEL_TRUST_CHART);
    }

    // Reset chart accumulation state
    // oil_press
    oil_pressure_sum = 0;
    oil_pressure_samples = 0;
    oil_pressure_bucket_start = 0;
    // oil_temp
    oil_temp_sum = 0;
    oil_temp_samples = 0;
    oil_temp_bucket_start = 0;
    // water_temp
    water_temp_sum = 0;
    water_temp_samples = 0;
    water_temp_bucket_start = 0;
    // transmission_temp
    transmission_temp_sum = 0;
    transmission_temp_samples = 0;
    transmission_temp_bucket_start = 0;
    // steering_temp
    steering_temp_sum = 0;
    steering_temp_samples = 0;
    steering_temp_bucket_start = 0;
    // differencial_temp
    differencial_temp_sum = 0;
    differencial_temp_samples = 0;
    differencial_temp_bucket_start = 0;
    // fuel_trust
    fuel_trust_sum = 0;
    fuel_trust_samples = 0;
    fuel_trust_start = 0;

#endif
}

//=================================================================
// DEMO DATA PROVIDER
// Generates animated/simulated values for testing
// All values use simple sine wave oscillation (min to max and back)
//=================================================================

#pragma region Demo Data Provider

// Demo animation state - each value has its own cycle timing for visual variety
static struct {
    uint32_t start_time;  // When demo mode started
} g_demo_state = { 0 };

// Demo cycle durations (different periods for visual interest)
#define DEMO_CYCLE_OIL_PRESS_MS   16000   // 16 seconds
#define DEMO_CYCLE_OIL_TEMP_MS    20000   // 20 seconds
#define DEMO_CYCLE_WATER_TEMP_MS  18000   // 18 seconds
#define DEMO_CYCLE_TRANS_TEMP_MS  22000   // 22 seconds
#define DEMO_CYCLE_STEER_TEMP_MS  19000   // 19 seconds
#define DEMO_CYCLE_DIFF_TEMP_MS   21000   // 21 seconds
#define DEMO_CYCLE_FUEL_TRUST_MS  14000   // 14 seconds
#define DEMO_CYCLE_RPM_MS         10000   // 10 seconds

// Reset demo state to initial values
void resetDemoState() {
    g_demo_state.start_time = millis();
}

// Calculate sine wave value between min and max with given cycle time
// Uses (1 - cos) / 2 for smooth "slow at edges" effect
int calcDemoValue(int min_val, int max_val, uint32_t cycle_ms, uint32_t offset_ms = 0) {
    uint32_t now = millis();
    uint32_t elapsed = (now - g_demo_state.start_time + offset_ms) % cycle_ms;
    float angle = (float)elapsed / (float)cycle_ms * 2.0f * PI;
    float progress = (1.0f - cos(angle)) / 2.0f;  // 0 to 1 with slow at edges
    return min_val + (int)(progress * (max_val - min_val) + 0.5f);
}

void updateDemoData() {
    // Initialize start time if needed
    if (g_demo_state.start_time == 0) {
        g_demo_state.start_time = millis();
    }

    // Mark that we have data (for UI to know to show values instead of "---")
    g_vehicle_data.has_received_data = true;

    // ----- Oil Pressure: Simple sine wave 0-150 PSI -----
    g_vehicle_data.oil_pressure_psi = calcDemoValue(OIL_PRESS_Min_PSI, OIL_PRESS_Max_PSI, DEMO_CYCLE_OIL_PRESS_MS);
    g_vehicle_data.oil_pressure_valid = true;

    // ----- Oil Temperature: Simple sine wave 150-300°F -----
    int oil_temp_main = calcDemoValue(OIL_TEMP_Min_F, OIL_TEMP_Max_F, DEMO_CYCLE_OIL_TEMP_MS);
    g_vehicle_data.oil_temp_pan_f = oil_temp_main;
    g_vehicle_data.oil_temp_cooled_f = oil_temp_main - 20;  // Cooled line is ~20°F lower
    g_vehicle_data.oil_temp_valid = true;

    // ----- Water Temperature: Simple sine wave 100-260°F -----
    int water_temp_main = calcDemoValue(W_TEMP_Min_F, W_TEMP_Max_F, DEMO_CYCLE_WATER_TEMP_MS);
    g_vehicle_data.water_temp_hot_f = water_temp_main;
    g_vehicle_data.water_temp_cooled_f = water_temp_main - 20;
    g_vehicle_data.water_temp_valid = true;

    // ----- Trans Temperature: Simple sine wave 80-280°F -----
    int trans_temp_main = calcDemoValue(TRAN_TEMP_Min_F, TRAN_TEMP_Max_F, DEMO_CYCLE_TRANS_TEMP_MS);
    g_vehicle_data.trans_temp_hot_f = trans_temp_main;
    g_vehicle_data.trans_temp_cooled_f = trans_temp_main - 25;
    g_vehicle_data.trans_temp_valid = true;

    // ----- Steering Temperature: Simple sine wave 60-300°F -----
    int steer_temp_main = calcDemoValue(STEER_TEMP_Min_F, STEER_TEMP_Max_F, DEMO_CYCLE_STEER_TEMP_MS);
    g_vehicle_data.steer_temp_hot_f = steer_temp_main;
    g_vehicle_data.steer_temp_cooled_f = steer_temp_main - 20;
    g_vehicle_data.steer_temp_valid = true;

    // ----- Diff Temperature: Simple sine wave 60-320°F -----
    int diff_temp_main = calcDemoValue(DIFF_TEMP_Min_F, DIFF_TEMP_Max_F, DEMO_CYCLE_DIFF_TEMP_MS);
    g_vehicle_data.diff_temp_hot_f = diff_temp_main;
    g_vehicle_data.diff_temp_cooled_f = diff_temp_main - 30;
    g_vehicle_data.diff_temp_valid = true;

    // ----- Fuel Trust: Simple sine wave 0-100% -----
    g_vehicle_data.fuel_trust_percent = calcDemoValue(FUEL_TRUST_Min, FUEL_TRUST_Max, DEMO_CYCLE_FUEL_TRUST_MS);
    g_vehicle_data.fuel_trust_valid = true;

    // ----- RPM: Simple sine wave 700-6500 -----
    g_vehicle_data.rpm = calcDemoValue(700, 6500, DEMO_CYCLE_RPM_MS);
    g_vehicle_data.rpm_valid = true;
}

#pragma endregion Demo Data Provider

//=================================================================
// SENSOR DATA PROVIDER
// Reads from physical sensors (thermistors, pressure sensors, etc.)
// TODO: Implement actual sensor reading code
//=================================================================

#pragma region Sensor Data Provider

void initSensors() {
    // TODO: Initialize ADC pins for temperature/pressure sensors
    // Example:
    // - Oil pressure: analog input with voltage divider
    // - Oil temp: thermistor with known resistance curve
    // - etc.

    Serial.println("[SENSORS] Sensor initialization - NOT IMPLEMENTED");
}

void updateSensorData() {
    // TODO: Read from actual sensors and populate g_vehicle_data
    //
    // Example implementation:
    // float raw_oil_press = analogRead(OIL_PRESS_PIN);
    // float sensor_value = mapToSensorUnits(raw_oil_press);
    // // Convert from sensor units to internal (PSI)
    // g_vehicle_data.oil_pressure_psi = (int)pressToInternal(sensor_value, g_sensor_pressure_unit);
    // g_vehicle_data.oil_pressure_valid = true;
    // g_vehicle_data.has_received_data = true;
    //
    // float raw_oil_temp = analogRead(OIL_TEMP_PIN);
    // float temp_sensor = thermistorToValue(raw_oil_temp);  // Returns in sensor unit
    // // Convert from sensor units to internal (Fahrenheit)
    // g_vehicle_data.oil_temp_pan_f = (int)tempToInternal(temp_sensor, g_sensor_temp_unit);
    // g_vehicle_data.oil_temp_valid = true;

    // For now, mark all sensor data as invalid (no readings)
    // This will cause the UI to show "---" 
    g_vehicle_data.oil_pressure_valid = false;
    g_vehicle_data.oil_temp_valid = false;
    g_vehicle_data.water_temp_valid = false;
    g_vehicle_data.trans_temp_valid = false;
    g_vehicle_data.steer_temp_valid = false;
    g_vehicle_data.diff_temp_valid = false;
    g_vehicle_data.fuel_trust_valid = false;
}

#pragma endregion Sensor Data Provider

//=================================================================
// OBD DATA PROVIDER
// Reads from OBD-II via CAN bus or Bluetooth ELM327
// TODO: Implement actual OBD reading code
//=================================================================

#pragma region OBD Data Provider

void initOBD() {
    // TODO: Initialize CAN bus or Bluetooth connection
    // Example for CAN:
    // - Configure MCP2515 or ESP32's built-in CAN controller
    // - Set baud rate (typically 500kbps for OBD-II)
    // - Set up message filters for relevant PIDs

    Serial.println("[OBD] OBD initialization - NOT IMPLEMENTED");
}

void updateOBDData() {
    // TODO: Read from OBD-II and populate g_vehicle_data
    //
    // Example implementation:
    // sendOBDRequest(PID_RPM);
    // if (receiveOBDResponse(&rpm_data)) {
    //     g_vehicle_data.rpm = parseRPM(rpm_data);
    //     g_vehicle_data.rpm_valid = true;
    // }

    // For now, mark OBD data as invalid
    g_vehicle_data.rpm_valid = false;
}

#pragma endregion OBD Data Provider

//=================================================================
// SD CARD DATA LOGGER
// Logs all vehicle data to CSV files on SD card
//=================================================================

#pragma region SD Card Logger

#if ENABLE_SD_LOGGING

// SD Card Configuration
// Waveshare ESP32-S3 7" Touch uses IO expander for SD_CS (EXIO_SD_CS bit 4)
// SPI pins: SCK=12, MISO=13, MOSI=11 (directly connected, no IO expander)
// IMPORTANT: GPIO10 is used by display (Blue B7) - do NOT use for SD!
#define SD_SCK_PIN        12      // SPI Clock
#define SD_MISO_PIN       13      // SPI MISO (Master In Slave Out)
#define SD_MOSI_PIN       11      // SPI MOSI (Master Out Slave In)  
#define SD_CS_PIN         -1      // Use -1 for manual CS control via IO expander
#define SD_SPI_FREQ       4000000 // 4MHz SPI (conservative for reliability)
#define SD_WRITE_INTERVAL_MS 1000 // Write every 1 second (configurable)
#define SD_BUFFER_SIZE    256     // Smaller buffer for more frequent flushes
#define SD_MAX_RETRIES    3       // Max retries on write failure
#define SD_FREE_SPACE_PERCENT 5   // Keep at least 5% free space
#define SD_MIN_FREE_BYTES (1024 * 1024)  // Absolute minimum 1MB free

// SD Card State
static struct {
    bool initialized;
    bool card_present;
    bool file_open;
    bool logging_enabled;
    bool rtc_available;         // DS3231 RTC detected
    File data_file;
    char current_filename[32];
    uint32_t session_start_ms;
    uint32_t last_write_ms;
    uint32_t write_count;
    uint32_t error_count;
    uint32_t bytes_written;
    uint32_t boot_count;        // Persisted boot counter
    uint64_t total_bytes;       // Total card size
    uint64_t used_bytes;        // Used space
} g_sd_state = { 0 };

// Forward declarations
bool sdInit();
bool sdStartSession();
void sdEndSession();
void sdLogData();
void sdSafeFlush();
float getCpuLoadPercent();
bool sdCheckAndManageSpace();
bool sdDeleteOldestLog();
uint32_t sdReadBootCount();
void sdWriteBootCount(uint32_t count);
bool sdDetectRTC();

// Initialize SD card
bool sdInit() {
    Serial.println("[SD] Initializing SD card...");

    // CRITICAL: The Waveshare ESP32-S3 7" Touch LCD uses:
    // - GPIO10 for display Blue channel B7 - DO NOT TOUCH!
    // - IO expander bit 4 (EXIO_SD_CS) for SD card chip select
    // - SPI pins: SCK=12, MISO=13, MOSI=11

    // Deselect SD card CS via IO expander (set high)
    if (g_ioexp_ok) {
        exio_set(EXIO_SD_CS, true);  // CS high = deselected
        delay(10);
    }
    else {
        Serial.println("[SD] ERROR: IO expander not available for CS control!");
        return false;
    }

    // Configure SPI for SD card - DO NOT include CS pin!
    // Using HSPI peripheral on ESP32-S3
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN);  // No CS pin - we control it via IO expander

    // Select SD card via IO expander for initialization
    exio_set(EXIO_SD_CS, false);  // CS low = selected
    delay(10);

    // Initialize SD card with software CS control
    // The actual CS is controlled via IO expander (EXIO_SD_CS)
    // We need to pass SOME pin to SD.begin() - use GPIO15 as dummy
    // (GPIO6 might be used by flash on some ESP32-S3 variants)
    pinMode(15, OUTPUT);
    digitalWrite(15, HIGH);  // Keep dummy CS high (deselected)

    if (!SD.begin(15, SPI, SD_SPI_FREQ)) {
        Serial.println("[SD] Card mount failed!");
        exio_set(EXIO_SD_CS, true);  // Deselect on failure
        g_sd_state.initialized = false;
        g_sd_state.card_present = false;
        return false;
    }

    // Check card type
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("[SD] No SD card attached!");
        g_sd_state.initialized = false;
        g_sd_state.card_present = false;
        return false;
    }

    const char* cardTypeName = "UNKNOWN";
    switch (cardType) {
    case CARD_MMC:  cardTypeName = "MMC";  break;
    case CARD_SD:   cardTypeName = "SD";   break;
    case CARD_SDHC: cardTypeName = "SDHC"; break;
    }

    Serial.printf("[SD] Card type: %s\n", cardTypeName);
    Serial.printf("[SD] Card size: %llu MB\n", SD.cardSize() / (1024 * 1024));

    g_sd_state.total_bytes = SD.totalBytes();
    g_sd_state.used_bytes = SD.usedBytes();
    g_sd_state.initialized = true;
    g_sd_state.card_present = true;
    g_sd_state.logging_enabled = true;

    g_sd_state.boot_count = sdReadBootCount();
    g_sd_state.boot_count++;
    sdWriteBootCount(g_sd_state.boot_count);
    Serial.printf("[SD] Boot count: %lu\n", g_sd_state.boot_count);

    g_sd_state.rtc_available = sdDetectRTC();

    sdCheckAndManageSpace();

    Serial.println("[SD] Initialization successful");
    return true;
}

// Generate unique session filename
void sdGenerateFilename() {
    // Format: SESS_NNNNN.csv (boot count based)
    // If RTC available in future: YYYY-MM-DD_HH-MM-SS.csv
    snprintf(g_sd_state.current_filename, sizeof(g_sd_state.current_filename),
        "/SESS_%05lu.csv", g_sd_state.boot_count);
    Serial.printf("[SD] Session file: %s\n", g_sd_state.current_filename);
}

// Start a new logging session
bool sdStartSession() {
    if (!g_sd_state.initialized || !g_sd_state.card_present) {
        return false;
    }

    // Check space before starting
    if (!sdCheckAndManageSpace()) {
        return false;
    }

    // Generate filename
    sdGenerateFilename();

    // Open file for APPEND (safer than WRITE for corruption prevention)
    g_sd_state.data_file = SD.open(g_sd_state.current_filename, FILE_APPEND);
    if (!g_sd_state.data_file) {
        // Try FILE_WRITE if file doesn't exist
        g_sd_state.data_file = SD.open(g_sd_state.current_filename, FILE_WRITE);
    }
    if (!g_sd_state.data_file) {
        Serial.printf("[SD] Failed to create file: %s\n", g_sd_state.current_filename);
        return false;
    }

    g_sd_state.file_open = true;
    g_sd_state.session_start_ms = millis();
    g_sd_state.last_write_ms = 0;
    g_sd_state.write_count = 0;
    g_sd_state.error_count = 0;
    g_sd_state.bytes_written = 0;

    const char* header = "timestamp_ms,elapsed_s,cpu_percent,mode,"
        "oil_press_psi,oil_press_valid,"
        "oil_temp_pan_f,oil_temp_cooled_f,oil_temp_valid,"
        "water_temp_hot_f,water_temp_cooled_f,water_temp_valid,"
        "trans_temp_hot_f,trans_temp_cooled_f,trans_temp_valid,"
        "steer_temp_hot_f,steer_temp_cooled_f,steer_temp_valid,"
        "diff_temp_hot_f,diff_temp_cooled_f,diff_temp_valid,"
        "fuel_trust_percent,fuel_trust_valid,"
        "rpm,rpm_valid\n";

    size_t written = g_sd_state.data_file.print(header);
    if (written == 0) {
        g_sd_state.data_file.close();
        g_sd_state.file_open = false;
        return false;
    }

    g_sd_state.bytes_written += written;
    sdSafeFlush();  // Immediate flush after header

    Serial.printf("[SD] Session started: %s\n", g_sd_state.current_filename);
    return true;
}

// End current logging session
void sdEndSession() {
    if (!g_sd_state.file_open) return;

    // Final flush and sync
    sdSafeFlush();

    // Close file
    g_sd_state.data_file.close();
    g_sd_state.file_open = false;
    Serial.printf("[SD] Session ended: %lu writes, %lu bytes\n",
        g_sd_state.write_count, g_sd_state.bytes_written);
}

// Safe flush - ensures data is written to card
void sdSafeFlush() {
    if (!g_sd_state.file_open) return;
    g_sd_state.data_file.flush();
    // Note: flush() should sync to card, but for extra safety on some SD cards:
    // We could close and reopen, but that's slow. flush() is usually sufficient.
}

// Calculate CPU load percentage (based on loop timing)
float getCpuLoadPercent() {
    extern uint32_t cpu_busy_time;
    return (float)(cpu_busy_time * 100) / 1000.0f;
}

// Log current data point to SD card - IMMEDIATE WRITE for corruption safety
void sdLogData() {
    if (!g_sd_state.file_open || !g_sd_state.logging_enabled) return;

    uint32_t now = millis();

    // Check if it's time to write
    if ((now - g_sd_state.last_write_ms) < SD_WRITE_INTERVAL_MS) return;
    g_sd_state.last_write_ms = now;

    // Periodic space check (every 60 writes = ~1 minute at 1Hz)
    if (g_sd_state.write_count % 60 == 0 && g_sd_state.write_count > 0) {
        if (!sdCheckAndManageSpace()) {
            g_sd_state.logging_enabled = false;
            return;
        }
    }

    // Calculate elapsed time and CPU load
    float elapsed_s = (float)(now - g_sd_state.session_start_ms) / 1000.0f;
    float cpu_pct = getCpuLoadPercent();

    // Format and write data line directly (no buffering for safety)
    char line[256];
    int len = snprintf(line, sizeof(line),
        "%lu,%.2f,%.1f,%s,"
        "%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,"
        "%d,%d\n",
        now, elapsed_s, cpu_pct, g_demo_mode ? "DEMO" : "LIVE",
        g_vehicle_data.oil_pressure_psi, g_vehicle_data.oil_pressure_valid ? 1 : 0,
        g_vehicle_data.oil_temp_pan_f, g_vehicle_data.oil_temp_cooled_f, g_vehicle_data.oil_temp_valid ? 1 : 0,
        g_vehicle_data.water_temp_hot_f, g_vehicle_data.water_temp_cooled_f, g_vehicle_data.water_temp_valid ? 1 : 0,
        g_vehicle_data.trans_temp_hot_f, g_vehicle_data.trans_temp_cooled_f, g_vehicle_data.trans_temp_valid ? 1 : 0,
        g_vehicle_data.steer_temp_hot_f, g_vehicle_data.steer_temp_cooled_f, g_vehicle_data.steer_temp_valid ? 1 : 0,
        g_vehicle_data.diff_temp_hot_f, g_vehicle_data.diff_temp_cooled_f, g_vehicle_data.diff_temp_valid ? 1 : 0,
        g_vehicle_data.fuel_trust_percent, g_vehicle_data.fuel_trust_valid ? 1 : 0,
        g_vehicle_data.rpm, g_vehicle_data.rpm_valid ? 1 : 0
    );

    // Write with retry
    bool success = false;
    for (int retry = 0; retry < SD_MAX_RETRIES && !success; retry++) {
        size_t written = g_sd_state.data_file.print(line);
        if (written > 0) {
            g_sd_state.bytes_written += written;
            g_sd_state.write_count++;
            success = true;

            // IMMEDIATE FLUSH after every write for corruption safety
            sdSafeFlush();
        }
        else {
            delay(5);
        }
    }

    if (!success) {
        g_sd_state.error_count++;
        if (g_sd_state.error_count >= 10) {
            g_sd_state.logging_enabled = false;
        }
    }
}

// Test write to SD card - writes CPU load test data
bool sdTestWrite() {
    // Create test file
    if (!g_sd_state.initialized) return false;

    File testFile = SD.open("/test_write.csv", FILE_WRITE);
    // Write header
    if (!testFile) return false;

    testFile.println("timestamp_ms,cpu_percent,test_value");
    testFile.flush();

    // Write 10 test entries with flush after each
    uint32_t start = millis();
    for (int i = 0; i < 10; i++) {
        float cpu = getCpuLoadPercent();
        char line[64];
        snprintf(line, sizeof(line), "%lu,%.1f,%d", millis(), cpu, i * 10);
        testFile.println(line);
        testFile.flush();  // Flush after each write
        delay(100);
    }

    testFile.close();

    Serial.println("[SD] Test write successful!");
    return true;
}

//=================================================================
// BOOT COUNTER MANAGEMENT
// Persists boot count on SD card for session numbering
//=================================================================

#define BOOT_COUNT_FILE "/.bootcount"

uint32_t sdReadBootCount() {
    File f = SD.open(BOOT_COUNT_FILE, FILE_READ);
    if (!f) {
        return 0;  // First boot or file missing
    }

    char buf[16] = { 0 };
    f.readBytes(buf, sizeof(buf) - 1);
    f.close();

    return (uint32_t)atol(buf);
}

void sdWriteBootCount(uint32_t count) {
    File f = SD.open(BOOT_COUNT_FILE, FILE_WRITE);
    if (!f) {
        Serial.println("[SD] Warning: Could not write boot count");
        return;
    }

    f.printf("%lu", count);
    f.flush();
    f.close();
}

//=================================================================
// RTC DETECTION (Optional DS3231)
//=================================================================

#define DS3231_ADDR 0x68

bool sdDetectRTC() {
    // Try to detect DS3231 on I2C
    Wire.beginTransmission(DS3231_ADDR);
    return (Wire.endTransmission() == 0);
}

// TODO: If RTC detected, add functions to read/write time
// For now, just detection - full RTC support can be added later

//=================================================================
// FREE SPACE MANAGEMENT
// Keeps at least 5% free, deletes oldest session files if needed
//=================================================================

// Check if enough free space, delete old files if needed
bool sdCheckAndManageSpace() {
    if (!g_sd_state.initialized) return false;

    // Update space tracking
    g_sd_state.total_bytes = SD.totalBytes();
    g_sd_state.used_bytes = SD.usedBytes();

    uint64_t free_bytes = g_sd_state.total_bytes - g_sd_state.used_bytes;
    uint64_t min_free = (g_sd_state.total_bytes * SD_FREE_SPACE_PERCENT) / 100;

    if (min_free < SD_MIN_FREE_BYTES) {
        min_free = SD_MIN_FREE_BYTES;
    }

    if (free_bytes >= min_free) {
        return true;
    }

    int deleted = 0;
    while (free_bytes < min_free && deleted < 10) {
        if (sdDeleteOldestLog()) {
            deleted++;
            // Refresh space calculation
            g_sd_state.used_bytes = SD.usedBytes();
            free_bytes = g_sd_state.total_bytes - g_sd_state.used_bytes;
        }
        else {
            break;  // No more files to delete
        }
    }

    return (free_bytes >= min_free);
}

// Find and delete the oldest SESS_NNNNN.csv file
bool sdDeleteOldestLog() {
    File root = SD.open("/");
    if (!root || !root.isDirectory()) return false;

    char oldest_name[32] = { 0 };
    uint32_t oldest_num = UINT32_MAX;

    File entry;
    while ((entry = root.openNextFile())) {
        const char* name = entry.name();

        // Check if it matches SESS_NNNNN.csv pattern
        if (strncmp(name, "SESS_", 5) == 0) {
            // Extract number
            uint32_t num = atol(name + 5);

            // Don't delete current session file
            if (num < oldest_num && num != g_sd_state.boot_count) {
                oldest_num = num;
                strncpy(oldest_name, name, sizeof(oldest_name) - 1);
            }
        }

        // Also check old LOG_NNNN.csv format
        if (strncmp(name, "LOG_", 4) == 0) {
            uint32_t num = atol(name + 4);
            if (num < oldest_num) {
                oldest_num = num;
                strncpy(oldest_name, name, sizeof(oldest_name) - 1);
            }
        }

        entry.close();
    }
    root.close();

    if (oldest_name[0] == '\0') return false;

    char path[40];
    snprintf(path, sizeof(path), "/%s", oldest_name);

    //=================================================================
    // STATUS AND CONTROL FUNCTIONS
    //=================================================================
    return SD.remove(path);
}

// Pause SD logging (keeps file open)
void sdPauseLogging() {
    g_sd_state.logging_enabled = false;
}

// Resume SD logging
void sdResumeLogging() {
    if (g_sd_state.file_open) {
        g_sd_state.logging_enabled = true;
    }
}

// Check if SD logging is active
bool sdIsLogging() {
    return g_sd_state.file_open && g_sd_state.logging_enabled;
}

// Get current log file name
const char* sdGetCurrentFilename() {
    return g_sd_state.current_filename;
}

// Get write statistics
void sdGetStats(uint32_t* writes, uint32_t* bytes, uint32_t* errors) {
    if (writes) *writes = g_sd_state.write_count;
    if (bytes) *bytes = g_sd_state.bytes_written;
    if (errors) *errors = g_sd_state.error_count;
}

// Get SD card status string for display
void sdGetStatusString(char* buf, size_t buf_size) {
    if (!g_sd_state.initialized) {
        snprintf(buf, buf_size, "SD:NONE");
    }
    else if (!g_sd_state.file_open) {
        snprintf(buf, buf_size, "SD:READY");
    }
    else if (!g_sd_state.logging_enabled) {
        snprintf(buf, buf_size, "SD:PAUSE");
    }
    else if (g_sd_state.error_count > 0) {
        snprintf(buf, buf_size, "SD:E%lu", g_sd_state.error_count);
    }
    else {
        // Show KB written
        uint32_t kb = g_sd_state.bytes_written / 1024;
        if (kb < 1000) {
            snprintf(buf, buf_size, "SD:%luK", kb);
        }
        else {
            snprintf(buf, buf_size, "SD:%luM", kb / 1024);
        }
    }
}

#endif // ENABLE_SD_LOGGING

#pragma endregion SD Card Logger

//=================================================================
// USB MASS STORAGE MODE
// Allows SD card to be accessed via USB-C as a flash drive
// Enter by holding BOOT button during power-on
//=================================================================

#pragma region USB Mass Storage

#if ENABLE_USB_MSC && ENABLE_SD_LOGGING

// USB MSC for SD Card access
// Uses SD library's internal sdcard functions

#include "sd_diskio.h"  // ESP32 SD diskio functions

static USBMSC msc;  // device that pretends to be a storage drive over Universal Serial Bus
static bool g_usb_msc_mode = false;

// SD card info
static uint32_t g_sd_sector_count = 0;
static const uint16_t g_sd_sector_size = 512;
static uint8_t g_sd_pdrv = 0xFF;  // physical drive number (0xFF means “not ready”)

// USB MSC callbacks
static int32_t onMscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
    if (g_sd_pdrv == 0xFF) return -1;

    // Reject partial-sector requests (offset must be 0)
    if (offset != 0) return -1;

    uint32_t sectors = bufsize / g_sd_sector_size;
    if (sectors == 0) sectors = 1;

    // Read sectors one at a time
    for (uint32_t i = 0; i < sectors; i++) {
        if (!sd_read_raw(g_sd_pdrv, (uint8_t*)buffer + (i * 512), lba + i)) {
            return -1;
        }
    }
    return bufsize;
}

static int32_t onMscWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
    if (g_sd_pdrv == 0xFF) return -1;

    // Reject partial-sector requests (offset must be 0)
    if (offset != 0) return -1;

    uint32_t sectors = bufsize / g_sd_sector_size;
    if (sectors == 0) sectors = 1;

    // Write sectors one at a time
    for (uint32_t i = 0; i < sectors; i++) {
        if (!sd_write_raw(g_sd_pdrv, buffer + (i * 512), lba + i)) {
            return -1;
        }
    }
    return bufsize;
}

static bool onMscStartStop(uint8_t power_condition, bool start, bool load_eject) {
    return true;
}

// Check if we should enter USB MSC mode (BOOT button held at startup)
bool checkUSBMSCMode() {
    // Check GPIO0 (BOOT button) immediately - before any other init
    // BOOT button is active LOW (pressed = LOW)
    pinMode(USB_MSC_BOOT_PIN, INPUT_PULLUP);
    delay(50);  // Short debounce

    // Check multiple times for reliability
    int pressed = 0;
    for (int i = 0; i < 10; i++) {
        if (digitalRead(USB_MSC_BOOT_PIN) == LOW) pressed++;
        delay(20);
    }
    // Need at least 7/10 reads as LOW to trigger
    return (pressed >= 7);
}

// Initialize and run USB Mass Storage mode (never returns)
void runUSBMSCMode() {
    g_usb_msc_mode = true;

    const uint16_t GRAY_BG = 0x0842;
    const uint16_t GREEN = 0x001F;
    const uint16_t WHITE = 0xFFFF;

    const uint16_t BLACK = 0x0000;
    const uint16_t PINK = 0x07E0;
    const uint16_t BLUE = 0xF800;
    const uint16_t YELLOW = 0x07FF;
    const uint16_t AQUA = 0xF81F;
    const uint16_t PURPLE = 0xFFE0;

    // =========================================================
    // VISUAL FEEDBACK - Initialize display to show USB MSC mode
    // Serial won't work when USB is in MSC mode!
    // =========================================================

    // Initialize I2C for IO expander
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
    Wire.setTimeOut(50);
    delay(50);

    // Initialize IO expander
    g_ioexp_ok = initIOExtension();
    if (!g_ioexp_ok) {
        // Can't do much without IO expander - just hang
        while (1) { delay(1000); }
    }

    // Turn on backlight
    setBacklight(true);

    // Initialize display for visual feedback
    gfx->begin();
    gfx->fillScreen(GRAY_BG);

    // Draw text on display (simple, no LVGL needed)
    gfx->setTextSize(3);
    gfx->setTextColor(WHITE);
    gfx->setCursor(211, 112);
    gfx->print("USB MASS STORAGE MODE");
    gfx->setTextSize(2);
    gfx->setCursor(232, 172);
    gfx->print("SD Card accessible via USB-C");
    gfx->setCursor(286, 212);
    gfx->print("Power cycle to exit");
    gfx->setCursor(262, 272);
    gfx->setTextColor(WHITE);
    gfx->print("Initializing SD card...");

    // Initialize SPI for SD card
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN);
    SPI.setFrequency(SD_SPI_FREQ);

    // CRITICAL: Select SD card via IO expander BEFORE SD.begin()
    // The IO expander controls the real CS pin, not GPIO15
    exio_set(EXIO_SD_CS, false);  // CS LOW = selected
    delay(10);

    // Use GPIO15 as dummy CS for SD library (not physically connected)
    pinMode(15, OUTPUT);
    digitalWrite(15, HIGH);  // Keep dummy high

    // Initialize SD card using Arduino library (for card info)
    if (!SD.begin(15, SPI, SD_SPI_FREQ)) {
        // SD init failed - show error
        gfx->fillRect(100, 262, 600, 40, GRAY_BG);  // Clear status area
        gfx->setCursor(250, 272);
        gfx->setTextColor(0x07E0);  // Red
        gfx->print("ERROR: SD card not found!");
        while (1) { delay(500); }
    }

    // Get SD card info
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        gfx->fillRect(100, 262, 600, 40, GRAY_BG);
        gfx->setCursor(238, 272);
        gfx->setTextColor(0x07E0);
        gfx->print("ERROR: No SD card detected!");
        while (1) { delay(1000); }
    }

    uint64_t cardSize = SD.cardSize();
    g_sd_sector_count = cardSize / g_sd_sector_size;

    // Update display with card info
    gfx->fillRect(100, 262, 600, 80, GRAY_BG);
    gfx->setCursor(226, 312);
    gfx->setTextColor(WHITE);
    gfx->print("Connect USB-C to computer now");
    // Initialize USB Mass Storage

    g_sd_pdrv = 0;

    msc.vendorID("370zMon");
    msc.productID("SD Card");
    msc.productRevision("1.0");
    msc.onStartStop(onMscStartStop);
    msc.onRead(onMscRead);
    msc.onWrite(onMscWrite);
    msc.mediaPresent(true);
    msc.begin(g_sd_sector_count, g_sd_sector_size);

    // Start USB, money line
    USB.begin();

    bool blink = false;
    while (1) {
        delay(500);
        blink = !blink;

        // clear just the "USB Ready!" strip
        gfx->fillRect(100, 342, 600, 40, GRAY_BG);

        // force solid overwrite: foreground + background
        gfx->setCursor(340, 352);
        gfx->setTextColor(blink ? WHITE : GRAY_BG, GRAY_BG);
        gfx->print("USB Ready!");
    }
}

#else
// Stubs when USB MSC is disabled
bool checkUSBMSCMode() { return false; }
void runUSBMSCMode() { }
#endif // ENABLE_USB_MSC && ENABLE_SD_LOGGING

#pragma endregion USB Mass Storage

//=================================================================
// DATA PROVIDER DISPATCHER
//=================================================================

void updateVehicleData() {
    if (g_demo_mode) {
        updateDemoData();
    }
    else {
        updateSensorData();
        updateOBDData();
    }
}

//=================================================================
// HELPER FUNCTIONS
//=================================================================

// Check if oil pressure is critically low based on RPM
static inline bool isOilPressureCriticalRPM() {
    if (!g_vehicle_data.rpm_valid || !g_vehicle_data.oil_pressure_valid) {
        return false;
    }
    // Critical: <= 10 PSI per 1000 RPM
    int min_required = (g_vehicle_data.rpm * 10 + 999) / 1000;
    return (g_vehicle_data.oil_pressure_psi < min_required);
}

// Check if oil pressure is in critical range (absolute)
static inline bool isOilPressureCritical() {
    if (!g_vehicle_data.oil_pressure_valid) return false;
    return (g_vehicle_data.oil_pressure_psi < OIL_PRESS_ValueCriticalLow) ||
        (g_vehicle_data.oil_pressure_psi > OIL_PRESS_ValueCriticalAbsolute);
}

// Shift history array left and add new value
static void shift_history(int32_t* history, int32_t new_value) {
    for (int i = 0; i < CHART_POINTS - 1; i++) {
        history[i] = history[i + 1];
    }
    history[CHART_POINTS - 1] = new_value;
}

//-----------------------------------------------------------------

#pragma region Chart Draw Callbacks

static void oil_press_chart_draw_cb(lv_event_t* e) {
    lv_draw_task_t* draw_task = lv_event_get_draw_task(e);
    lv_draw_fill_dsc_t* fill_dsc = lv_draw_task_get_fill_dsc(draw_task);
    if (fill_dsc == NULL) return;

    lv_draw_dsc_base_t* base_dsc = (lv_draw_dsc_base_t*)fill_dsc;
    if (base_dsc->part != LV_PART_ITEMS) return;

    uint32_t idx = base_dsc->id2;
    if (idx >= CHART_POINTS) return;

    int32_t psi = oil_press_history[idx];
    bool is_critical = (psi < OIL_PRESS_ValueCriticalLow) || (psi > OIL_PRESS_ValueCriticalAbsolute);

    if (is_critical && psi > 0) {
        fill_dsc->color = g_critical_blink_phase ? lv_color_hex(0xFFFFFF) : lv_color_hex(hexRed);
    }
    else {
        fill_dsc->color = lv_color_hex(hexOrange);
    }
}

static void oil_temp_chart_draw_cb(lv_event_t* e) {
    lv_draw_task_t* draw_task = lv_event_get_draw_task(e);
    lv_draw_fill_dsc_t* fill_dsc = lv_draw_task_get_fill_dsc(draw_task);
    if (fill_dsc == NULL) return;

    lv_draw_dsc_base_t* base_dsc = (lv_draw_dsc_base_t*)fill_dsc;
    if (base_dsc->part != LV_PART_ITEMS) return;

    uint32_t idx = base_dsc->id2;
    if (idx >= CHART_POINTS) return;

    int32_t temp_f = oil_temp_history[idx];
    bool is_critical = (temp_f > OIL_TEMP_ValueCriticalF);

    if (is_critical && temp_f > 0) {
        fill_dsc->color = g_critical_blink_phase ? lv_color_hex(0xFFFFFF) : lv_color_hex(hexRed);
    }
    else {
        fill_dsc->color = lv_color_hex(hexRed);
    }
}

// Generic chart draw callback for critical values (temperature based)
static void generic_temp_chart_draw_cb(lv_event_t* e, int32_t* history, int critical_threshold) {
    lv_draw_task_t* draw_task = lv_event_get_draw_task(e);
    lv_draw_fill_dsc_t* fill_dsc = lv_draw_task_get_fill_dsc(draw_task);
    if (fill_dsc == NULL) return;

    lv_draw_dsc_base_t* base_dsc = (lv_draw_dsc_base_t*)fill_dsc;
    if (base_dsc->part != LV_PART_ITEMS) return;

    uint32_t idx = base_dsc->id2;
    if (idx >= CHART_POINTS) return;

    int32_t temp_f = history[idx];
    bool is_critical = (temp_f > critical_threshold);

    if (is_critical && temp_f > 0) {
        fill_dsc->color = g_critical_blink_phase ? lv_color_hex(0xFFFFFF) : lv_color_hex(hexRed);
    }
    else {
        fill_dsc->color = lv_color_hex(hexRed);
    }
}

static void water_temp_chart_draw_cb(lv_event_t* e) {
    generic_temp_chart_draw_cb(e, water_temp_history, W_TEMP_ValueCritical_F);
}

static void trans_temp_chart_draw_cb(lv_event_t* e) {
    generic_temp_chart_draw_cb(e, transmission_temp_history, TRAN_TEMP_ValueCritical_F);
}

static void steer_temp_chart_draw_cb(lv_event_t* e) {
    generic_temp_chart_draw_cb(e, steering_temp_history, STEER_TEMP_ValueCritical_F);
}

static void diff_temp_chart_draw_cb(lv_event_t* e) {
    generic_temp_chart_draw_cb(e, differencial_temp_history, DIFF_TEMP_ValueCritical_F);
}

static void fuel_trust_chart_draw_cb(lv_event_t* e) {
    lv_draw_task_t* draw_task = lv_event_get_draw_task(e);
    lv_draw_fill_dsc_t* fill_dsc = lv_draw_task_get_fill_dsc(draw_task);
    if (fill_dsc == NULL) return;

    lv_draw_dsc_base_t* base_dsc = (lv_draw_dsc_base_t*)fill_dsc;
    if (base_dsc->part != LV_PART_ITEMS) return;

    uint32_t idx = base_dsc->id2;
    if (idx >= CHART_POINTS) return;

    int32_t trust = fuel_trust_history[idx];
    bool is_critical = (trust < FUEL_TRUST_ValueCritical) && (trust > 0);

    if (is_critical) {
        fill_dsc->color = g_critical_blink_phase ? lv_color_hex(0xFFFFFF) : lv_color_hex(hexRed);
    }
    else {
        fill_dsc->color = lv_color_hex(hexRed);
    }
}

#pragma endregion Chart Draw Callbacks

//-----------------------------------------------------------------

#pragma region Utility Box Callbacks

#define DOUBLE_TAP_TIMEOUT_MS 300

// Update mode indicator text and color
static void updateModeIndicator() {
    if (mode_indicator) {
        if (g_demo_mode) {
            lv_label_set_text(mode_indicator, "DEMO");
            lv_obj_set_style_text_color(mode_indicator, lv_color_hex(0xFF00FF), 0);
        }
        else {
            lv_label_set_text(mode_indicator, "LIVE");
            lv_obj_set_style_text_color(mode_indicator, lv_color_hex(0x00FF00), 0);
        }
    }
    // Update tap box visibility based on mode
    updateTapBoxVisibility();
}

static void utility_box_single_tap_cb(lv_timer_t* t) {
    LV_UNUSED(t);
    g_util_single_tap_timer = NULL;

    if (g_brightness_level == 255) {
        setBrightness(89);
    }
    else {
        setBrightness(255);
    }

    Serial.printf("[UI] Single-tap: Brightness -> %d%%\n", (g_brightness_level * 100) / 255);
}

static void utility_box_tap_cb(lv_event_t* e) {
    LV_UNUSED(e);

    // If we're in a long-press situation, ignore tap
    if (g_utility_long_press_triggered) {
        return;
    }

    // Second tap arrived before timer fired => double tap
    if (g_util_single_tap_timer) {
        lv_timer_del(g_util_single_tap_timer);
        g_util_single_tap_timer = NULL;

        utilities_visible = !utilities_visible;
        if (utility_box) {
            if (utilities_visible) {
                lv_obj_set_style_opa(utility_box, LV_OPA_COVER, 0);
            }
            else {
                lv_obj_set_style_opa(utility_box, LV_OPA_TRANSP, 0);
            }
        }

        Serial.printf("[UI] Double-tap: Utility box %s\n", utilities_visible ? "shown" : "hidden");
        return;
    }

    // First tap: arm timer for single tap
    g_util_single_tap_timer = lv_timer_create(utility_box_single_tap_cb, DOUBLE_TAP_TIMEOUT_MS, NULL);
    lv_timer_set_repeat_count(g_util_single_tap_timer, 1);
}

// Press event - track start time for long press detection + visual feedback
static void utility_box_press_cb(lv_event_t* e) {
    LV_UNUSED(e);
    g_utility_press_start = millis();
    g_utility_long_press_triggered = false;

    // Visual feedback: lighten background on press (no transparency)
    if (utility_box) {
        lv_obj_set_style_bg_color(utility_box, lv_color_hex(0x666666), 0);  // Lighter gray
    }
}

// Release event - reset tracking + visual feedback
static void utility_box_release_cb(lv_event_t* e) {
    LV_UNUSED(e);
    g_utility_press_start = 0;
    g_utility_long_press_triggered = false;

    // Visual feedback: restore normal background (no transparency)
    if (utility_box) {
        lv_obj_set_style_bg_color(utility_box, lv_color_hex(0x444444), 0);  // Original dark gray
    }
}

// Called periodically to check for long press
static void checkUtilityLongPress() {
    if (g_utility_press_start > 0 && !g_utility_long_press_triggered) {
        uint32_t held_time = millis() - g_utility_press_start;

        if (held_time >= DEMO_MODE_TOGGLE_HOLD_MS) {
            // 5-second hold detected - toggle demo mode
            g_demo_mode = !g_demo_mode;
            g_utility_long_press_triggered = true;

            // Cancel any pending single-tap timer
            if (g_util_single_tap_timer) {
                lv_timer_del(g_util_single_tap_timer);
                g_util_single_tap_timer = NULL;
            }

            // IMPORTANT: Reset all data when switching modes
            // This ensures clean separation between demo and live data
            resetVehicleData();
            resetSmoothingState();
            resetUIElements();  // Reset bars and labels
            resetCharts();      // Reset chart data

            if (g_demo_mode) {
                // Entering demo mode - reset demo animation state
                resetDemoState();
            }

            updateModeIndicator();

            Serial.printf("[MODE] Demo mode %s (5-second hold) - data reset\n", g_demo_mode ? "ENABLED" : "DISABLED");
        }
    }
}

static void update_utility_label(int fps, int cpu_percent) {
    if (utility_label) {
        char buf[64];
        int bri_percent = (g_brightness_level * 100) / 255;

#if ENABLE_SD_LOGGING
        char sd_status[16];
        sdGetStatusString(sd_status, sizeof(sd_status));
        snprintf(buf, sizeof(buf), "%3d FPS\n%3d%% CPU\n%3d%% BRI\n%s", fps, cpu_percent, bri_percent, sd_status);
#else
        snprintf(buf, sizeof(buf), "%3d FPS\n%3d%% CPU\n%3d%% BRI", fps, cpu_percent, bri_percent);
#endif

        lv_label_set_text(utility_label, buf);
    }
}

#pragma endregion Utility Box Callbacks

//-----------------------------------------------------------------

#pragma region Unit Tap Box Callbacks

// Press feedback - show on press (demo mode only)
static void tap_box_press_cb(lv_event_t* e) {
    lv_obj_t* box = (lv_obj_t*)lv_event_get_target(e);
    // Only show visual feedback in demo mode
    if (g_demo_mode) {
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    }
}

static void tap_box_release_cb(lv_event_t* e) {
    lv_obj_t* box = (lv_obj_t*)lv_event_get_target(e);
    // Only show visual feedback in demo mode
    if (g_demo_mode) {
        lv_obj_set_style_bg_opa(box, TAP_BOX_OPACITY, 0);
    }
}

// Pressure tap - cycles PSI/Bar/kPa
static void oil_press_tap_cb(lv_event_t* e) {
    LV_UNUSED(e);
    g_pressure_unit = (PressureUnit)((g_pressure_unit + 1) % 3);
    saveUnitPreferences();
    smooth_oil_pressure = -1.0f;
    Serial.printf("[UNITS] Pressure -> %s\n", getPressureUnitStr(g_pressure_unit));
    // Immediately update display (works even with no valid data)
    if (!g_vehicle_data.oil_pressure_valid && ui_OIL_PRESS_Value) {
        char buf[32];
        snprintf(buf, sizeof(buf), "--- %s", getPressureUnitStr(g_pressure_unit));
        lv_label_set_text(ui_OIL_PRESS_Value, buf);
    }
}

// Individual temperature tap callbacks
static void oil_temp_tap_cb(lv_event_t* e) {
    LV_UNUSED(e);
    g_oil_temp_unit = (g_oil_temp_unit == TEMP_FAHRENHEIT) ? TEMP_CELSIUS : TEMP_FAHRENHEIT;
    saveUnitPreferences();
    smooth_oil_temp_f = -1.0f;
    Serial.printf("[UNITS] Oil Temp -> %s\n", getTempUnitStr(g_oil_temp_unit));
    // Immediately update display (works even with no valid data)
    if (!g_vehicle_data.oil_temp_valid) {
        char buf[32];
        const char* unit = getTempUnitStr(g_oil_temp_unit);
        if (ui_OIL_TEMP_Value_P) {
            snprintf(buf, sizeof(buf), "---°%s [P]", unit);
            lv_label_set_text(ui_OIL_TEMP_Value_P, buf);
        }
        if (ui_OIL_TEMP_Value_C) {
            snprintf(buf, sizeof(buf), "---°%s [C]", unit);
            lv_label_set_text(ui_OIL_TEMP_Value_C, buf);
        }
    }
}

static void water_temp_tap_cb(lv_event_t* e) {
    LV_UNUSED(e);
    g_water_temp_unit = (g_water_temp_unit == TEMP_FAHRENHEIT) ? TEMP_CELSIUS : TEMP_FAHRENHEIT;
    saveUnitPreferences();
    smooth_water_temp_f = -1.0f;
    Serial.printf("[UNITS] Water Temp -> %s\n", getTempUnitStr(g_water_temp_unit));
    // Immediately update display (works even with no valid data)
    if (!g_vehicle_data.water_temp_valid) {
        char buf[32];
        const char* unit = getTempUnitStr(g_water_temp_unit);
        if (ui_W_TEMP_Value_H) {
            snprintf(buf, sizeof(buf), "---°%s [H]", unit);
            lv_label_set_text(ui_W_TEMP_Value_H, buf);
        }
        if (ui_W_TEMP_Value_C) {
            snprintf(buf, sizeof(buf), "---°%s [C]", unit);
            lv_label_set_text(ui_W_TEMP_Value_C, buf);
        }
    }
}

static void trans_temp_tap_cb(lv_event_t* e) {
    LV_UNUSED(e);
    g_trans_temp_unit = (g_trans_temp_unit == TEMP_FAHRENHEIT) ? TEMP_CELSIUS : TEMP_FAHRENHEIT;
    saveUnitPreferences();
    smooth_trans_temp_f = -1.0f;
    Serial.printf("[UNITS] Trans Temp -> %s\n", getTempUnitStr(g_trans_temp_unit));
    // Immediately update display (works even with no valid data)
    if (!g_vehicle_data.trans_temp_valid) {
        char buf[32];
        const char* unit = getTempUnitStr(g_trans_temp_unit);
        if (ui_TRAN_TEMP_Value_H) {
            snprintf(buf, sizeof(buf), "---°%s [H]", unit);
            lv_label_set_text(ui_TRAN_TEMP_Value_H, buf);
        }
        if (ui_TRAN_TEMP_Value_C) {
            snprintf(buf, sizeof(buf), "---°%s [C]", unit);
            lv_label_set_text(ui_TRAN_TEMP_Value_C, buf);
        }
    }
}

static void steer_temp_tap_cb(lv_event_t* e) {
    LV_UNUSED(e);
    g_steer_temp_unit = (g_steer_temp_unit == TEMP_FAHRENHEIT) ? TEMP_CELSIUS : TEMP_FAHRENHEIT;
    saveUnitPreferences();
    smooth_steer_temp_f = -1.0f;
    Serial.printf("[UNITS] Steer Temp -> %s\n", getTempUnitStr(g_steer_temp_unit));
    // Immediately update display (works even with no valid data)
    if (!g_vehicle_data.steer_temp_valid) {
        char buf[32];
        const char* unit = getTempUnitStr(g_steer_temp_unit);
        if (ui_STEER_TEMP_Value_H) {
            snprintf(buf, sizeof(buf), "---°%s [H]", unit);
            lv_label_set_text(ui_STEER_TEMP_Value_H, buf);
        }
        if (ui_STEER_TEMP_Value_C) {
            snprintf(buf, sizeof(buf), "---°%s [C]", unit);
            lv_label_set_text(ui_STEER_TEMP_Value_C, buf);
        }
    }
}

static void diff_temp_tap_cb(lv_event_t* e) {
    LV_UNUSED(e);
    g_diff_temp_unit = (g_diff_temp_unit == TEMP_FAHRENHEIT) ? TEMP_CELSIUS : TEMP_FAHRENHEIT;
    saveUnitPreferences();
    smooth_diff_temp_f = -1.0f;
    Serial.printf("[UNITS] Diff Temp -> %s\n", getTempUnitStr(g_diff_temp_unit));
    // Immediately update display (works even with no valid data)
    if (!g_vehicle_data.diff_temp_valid) {
        char buf[32];
        const char* unit = getTempUnitStr(g_diff_temp_unit);
        if (ui_DIFF_TEMP_Value_H) {
            snprintf(buf, sizeof(buf), "---°%s [H]", unit);
            lv_label_set_text(ui_DIFF_TEMP_Value_H, buf);
        }
        if (ui_DIFF_TEMP_Value_C) {
            snprintf(buf, sizeof(buf), "---°%s [C]", unit);
            lv_label_set_text(ui_DIFF_TEMP_Value_C, buf);
        }
    }
}

// Create tap box over a label
lv_obj_t* createTapBox(lv_obj_t* parent, lv_obj_t* anchor, int w, int h, uint32_t color, lv_event_cb_t cb) {
    if (!parent || !anchor) return NULL;
    lv_obj_t* box = lv_obj_create(parent);
    lv_obj_remove_style_all(box);
    lv_obj_set_size(box, w, h);
    lv_obj_align_to(box, anchor, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(box, lv_color_hex(color), 0);
    // Only show debug colors in demo mode
    lv_obj_set_style_bg_opa(box, g_demo_mode ? TAP_BOX_OPACITY : LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(box, 4, 0);
    lv_obj_add_flag(box, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(box, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(box, tap_box_press_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(box, tap_box_release_cb, LV_EVENT_RELEASED, NULL);
    return box;
}

// Update tap box visibility based on demo mode
void updateTapBoxVisibility() {
    lv_opa_t opa = g_demo_mode ? TAP_BOX_OPACITY : LV_OPA_TRANSP;
    if (tap_box_oil_press) lv_obj_set_style_bg_opa(tap_box_oil_press, opa, 0);
    if (tap_box_oil_temp) lv_obj_set_style_bg_opa(tap_box_oil_temp, opa, 0);
    if (tap_box_water_temp) lv_obj_set_style_bg_opa(tap_box_water_temp, opa, 0);
    if (tap_box_trans_temp) lv_obj_set_style_bg_opa(tap_box_trans_temp, opa, 0);
    if (tap_box_steer_temp) lv_obj_set_style_bg_opa(tap_box_steer_temp, opa, 0);
    if (tap_box_diff_temp) lv_obj_set_style_bg_opa(tap_box_diff_temp, opa, 0);
}

void setupUnitTapHandlers() {
    if (ui_OIL_PRESS_Value)
        tap_box_oil_press = createTapBox(ui_Screen1, ui_OIL_PRESS_Value, 80, 40, 0x00FFFF, oil_press_tap_cb);
    if (ui_OIL_TEMP_Value_P)
        tap_box_oil_temp = createTapBox(ui_Screen1, ui_OIL_TEMP_Value_P, 80, 50, 0xFF0000, oil_temp_tap_cb);
    if (ui_W_TEMP_Value_H)
        tap_box_water_temp = createTapBox(ui_Screen1, ui_W_TEMP_Value_H, 80, 50, 0x00FF00, water_temp_tap_cb);
    if (ui_TRAN_TEMP_Value_H)
        tap_box_trans_temp = createTapBox(ui_Screen1, ui_TRAN_TEMP_Value_H, 80, 50, 0x0000FF, trans_temp_tap_cb);
    if (ui_STEER_TEMP_Value_H)
        tap_box_steer_temp = createTapBox(ui_Screen1, ui_STEER_TEMP_Value_H, 80, 50, 0xFFFF00, steer_temp_tap_cb);
    if (ui_DIFF_TEMP_Value_H)
        tap_box_diff_temp = createTapBox(ui_Screen1, ui_DIFF_TEMP_Value_H, 80, 50, 0xFF00FF, diff_temp_tap_cb);

    Serial.println("[UI] Tap boxes: CYAN=Press, RED=OilT, GREEN=WaterT, BLUE=TransT, YELLOW=SteerT, MAGENTA=DiffT");
}

#pragma endregion Unit Tap Box Callbacks

//-----------------------------------------------------------------

#pragma region Hardware Functions

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
    g_brightness_level = brightness;

    if (!g_dim_overlay) {
        lv_obj_t* top = lv_layer_top();
        g_dim_overlay = lv_obj_create(top);
        lv_obj_remove_style_all(g_dim_overlay);

        int w = lv_disp_get_hor_res(NULL);
        int h = lv_disp_get_ver_res(NULL);
        lv_obj_set_size(g_dim_overlay, w, h);
        lv_obj_set_pos(g_dim_overlay, 0, 0);

        lv_obj_clear_flag(g_dim_overlay, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(g_dim_overlay, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(g_dim_overlay, LV_OBJ_FLAG_IGNORE_LAYOUT);

        lv_obj_set_style_bg_color(g_dim_overlay, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(g_dim_overlay, LV_OPA_TRANSP, 0);
    }

    if (brightness >= 250) {
        lv_obj_add_flag(g_dim_overlay, LV_OBJ_FLAG_HIDDEN);
    }
    else {
        lv_obj_clear_flag(g_dim_overlay, LV_OBJ_FLAG_HIDDEN);
        uint8_t opa = (uint8_t)(255 - brightness);
        lv_obj_set_style_bg_opa(g_dim_overlay, opa, 0);
    }

    Serial.printf("[BRIGHTNESS] UI dim -> %d%%\n", (brightness * 100) / 255);
}

#pragma endregion Hardware Functions

//-----------------------------------------------------------------

#pragma region LVGL Callbacks

void my_disp_flush(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    uint32_t len = w * h;

    uint32_t* buf32 = (uint32_t*)px_map;
    uint32_t len32 = len >> 1;
    for (uint32_t i = 0; i < len32; i++) {
        uint32_t v = buf32[i];
        buf32[i] = ((v & 0x00FF00FF) << 8) | ((v & 0xFF00FF00) >> 8);
    }
    if (len & 1) {
        uint16_t* buf16 = (uint16_t*)px_map;
        buf16[len - 1] = (buf16[len - 1] >> 8) | (buf16[len - 1] << 8);
    }

    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t*)px_map, w, h);
    lv_display_flush_ready(disp);
    flush_count++;
}

#pragma endregion LVGL Callbacks

//-----------------------------------------------------------------

#pragma region Touch Callbacks

#define MAX_CONSECUTIVE_INVALID 50
#define TOUCH_RESET_COOLDOWN_MS 5000

void my_touch_read(lv_indev_t* indev, lv_indev_data_t* data) {
    touch.read();

    static bool was_touched = false;
    uint32_t now = millis();

    if (touch.isTouched) {
        int raw_x = touch.points[0].x;
        int raw_y = touch.points[0].y;

        if (raw_x == 65535 || raw_y == 65535 || raw_x > 800 || raw_y > 800) {
            consecutive_invalid++;

            if (consecutive_invalid >= MAX_CONSECUTIVE_INVALID &&
                (now - last_touch_reset) > TOUCH_RESET_COOLDOWN_MS) {
                Serial.println("[TOUCH] Controller stuck - hardware reset");
                exio_set(EXIO_TP_RST, false);
                delay(10);
                exio_set(EXIO_TP_RST, true);
                delay(50);
                touch.begin();
                touch.setRotation(0);
                consecutive_invalid = 0;
                last_touch_reset = now;
            }

            data->state = LV_INDEV_STATE_RELEASED;
            was_touched = false;
            return;
        }

        consecutive_invalid = 0;
        was_touched = true;
        data->state = LV_INDEV_STATE_PRESSED;

        int screen_x = (raw_y - 30) * 800 / 745;
        int screen_y = (770 - raw_x) * 480 / 420;

        if (screen_x < 0) screen_x = 0;
        if (screen_x > 799) screen_x = 799;
        if (screen_y < 0) screen_y = 0;
        if (screen_y > 479) screen_y = 479;

        data->point.x = screen_x;
        data->point.y = screen_y;

        static uint32_t last_touch_debug = 0;
        if (now - last_touch_debug > 200) {
            Serial.printf("[TOUCH] raw(%d,%d) -> screen(%d,%d)\n", raw_x, raw_y, screen_x, screen_y);
            last_touch_debug = now;
        }
    }
    else {
        consecutive_invalid = 0;
        if (was_touched) {
            Serial.println("[TOUCH] Released");
            was_touched = false;
        }
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

#pragma endregion Touch Callbacks

//=================================================================
// CRITICAL LABEL ANIMATION HELPER
//=================================================================

// Generic function to manage critical label visibility and blinking
void updateCriticalLabel(lv_obj_t* label, bool is_critical, bool* was_critical, bool* is_visible, uint32_t* exit_time) {
    if (!label) return;

    const uint32_t CRITICAL_LINGER_MS = 2000;
    uint32_t now = millis();

    if (is_critical) {
        *exit_time = 0;

        if (!*is_visible) {
            *is_visible = true;
            lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN);
            lv_obj_set_style_bg_color(label, lv_color_hex(0x000000), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(label, 225, LV_PART_MAIN);
            lv_obj_set_style_pad_all(label, 4, LV_PART_MAIN);

            lv_anim_t anim;
            lv_anim_init(&anim);
            lv_anim_set_var(&anim, label);
            lv_anim_set_values(&anim, 0, 255);
            lv_anim_set_duration(&anim, 200);
            lv_anim_set_repeat_count(&anim, LV_ANIM_REPEAT_INFINITE);
            lv_anim_set_playback_duration(&anim, 200);
            lv_anim_set_exec_cb(&anim, [](void* obj, int32_t val) {
                lv_obj_t* lbl = (lv_obj_t*)obj;
            if (val < 128) {
                lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
            }
            else {
                lv_obj_set_style_text_color(lbl, lv_color_hex(0x960000), LV_PART_MAIN);
            }
                });
            lv_anim_start(&anim);
        }
    }
    else {
        if (*was_critical && *exit_time == 0) {
            *exit_time = now;
        }

        if (*is_visible && *exit_time > 0 && (now - *exit_time) >= CRITICAL_LINGER_MS) {
            *is_visible = false;
            lv_anim_delete(label, NULL);
            lv_obj_set_style_text_opa(label, 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(label, 0, LV_PART_MAIN);
            *exit_time = 0;
        }
    }

    *was_critical = is_critical;
}

//=================================================================
// UI UPDATE FUNCTION
// Reads from g_vehicle_data and updates all UI elements
//=================================================================

// Static state for critical label tracking
static bool oil_press_was_critical = false, oil_press_visible = false;
static uint32_t oil_press_exit_time = 0;
static bool oil_temp_was_critical = false, oil_temp_visible = false;
static uint32_t oil_temp_exit_time = 0;
static bool water_temp_was_critical = false, water_temp_visible = false;
static uint32_t water_temp_exit_time = 0;
static bool trans_temp_was_critical = false, trans_temp_visible = false;
static uint32_t trans_temp_exit_time = 0;
static bool steer_temp_was_critical = false, steer_temp_visible = false;
static uint32_t steer_temp_exit_time = 0;
static bool diff_temp_was_critical = false, diff_temp_visible = false;
static uint32_t diff_temp_exit_time = 0;
static bool fuel_trust_was_critical = false, fuel_trust_visible = false;
static uint32_t fuel_trust_exit_time = 0;

void updateUI() {
    char buf[32];
    const char* pressUnit = getPressureUnitStr(g_pressure_unit);

    // ----- Oil Pressure -----
    // Only update UI if we have valid data
    if (g_vehicle_data.oil_pressure_valid) {
        int pressure_psi = g_vehicle_data.oil_pressure_psi;
        float display_pressure = pressToDisplay((float)pressure_psi, g_pressure_unit);

        // Initialize smoothing if first data
        if (smooth_oil_pressure < 0) {
            smooth_oil_pressure = display_pressure;
        }
        else {
            smooth_oil_pressure = smooth_oil_pressure * (1.0f - SMOOTH_FACTOR) + display_pressure * SMOOTH_FACTOR;
        }
        int display_val = (int)(smooth_oil_pressure + 0.5f);

        // Update bar
        if (ui_OIL_PRESS_Bar) {
            // Bar always uses PSI internally for range
            lv_bar_set_value(ui_OIL_PRESS_Bar, pressure_psi, LV_ANIM_ON);
            bool critical = isOilPressureCritical();
            lv_obj_set_style_bg_color(ui_OIL_PRESS_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }

        if (ui_OIL_PRESS_Value) {
            if (g_pressure_unit == PRESS_KPA) {
                snprintf(buf, sizeof(buf), "%d %s", display_val, pressUnit);
            }
            else if (g_pressure_unit == PRESS_BAR) {
                snprintf(buf, sizeof(buf), "%.1f %s", smooth_oil_pressure, pressUnit);
            }
            else {
                snprintf(buf, sizeof(buf), "%d %s", display_val, pressUnit);
            }
            lv_label_set_text(ui_OIL_PRESS_Value, buf);

            // Style label based on critical
            bool value_critical = isOilPressureCritical();
            static bool was_value_critical = false;
            if (value_critical != was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_OIL_PRESS_Value, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_OIL_PRESS_Value, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_OIL_PRESS_Value, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_OIL_PRESS_Value, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_OIL_PRESS_Value, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_OIL_PRESS_Value, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_OIL_PRESS_Value, 0, 0);
                }
                was_value_critical = value_critical;
            }
        }

        updateCriticalLabel(ui_OIL_PRESS_VALUE_CRITICAL_Label, isOilPressureCritical(),
            &oil_press_was_critical, &oil_press_visible, &oil_press_exit_time);
    }
    // If not valid, don't update - UI stays at reset state ("---" and 0)

    // ----- Oil Temperature -----
    if (g_vehicle_data.oil_temp_valid) {
        const char* oilTempUnit = getTempUnitStr(g_oil_temp_unit);
        float temp_pan_disp = tempToDisplay((float)g_vehicle_data.oil_temp_pan_f, g_oil_temp_unit);
        float temp_cooled_disp = tempToDisplay((float)g_vehicle_data.oil_temp_cooled_f, g_oil_temp_unit);

        if (smooth_oil_temp_f < 0) {
            smooth_oil_temp_f = temp_pan_disp;
        }
        else {
            smooth_oil_temp_f = smooth_oil_temp_f * (1.0f - SMOOTH_FACTOR) + temp_pan_disp * SMOOTH_FACTOR;
        }

        if (ui_OIL_TEMP_Bar) {
            lv_bar_set_value(ui_OIL_TEMP_Bar, g_vehicle_data.oil_temp_pan_f, LV_ANIM_ON);
            bool critical = (g_vehicle_data.oil_temp_pan_f > OIL_TEMP_ValueCriticalF);
            lv_obj_set_style_bg_color(ui_OIL_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }

        if (ui_OIL_TEMP_Value_P) {
            snprintf(buf, sizeof(buf), "%d°%s [P]", (int)(temp_pan_disp + 0.5f), oilTempUnit);
            lv_label_set_text(ui_OIL_TEMP_Value_P, buf);

            // Style label based on critical
            bool value_critical = (g_vehicle_data.oil_temp_pan_f > OIL_TEMP_ValueCriticalF);
            static bool oil_temp_was_value_critical = false;
            if (value_critical != oil_temp_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_OIL_TEMP_Value_P, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_OIL_TEMP_Value_P, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_OIL_TEMP_Value_P, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_OIL_TEMP_Value_P, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_OIL_TEMP_Value_P, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_OIL_TEMP_Value_P, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_OIL_TEMP_Value_P, 0, 0);
                }
                oil_temp_was_value_critical = value_critical;
            }
        }

        if (ui_OIL_TEMP_Value_C) {
            snprintf(buf, sizeof(buf), "%d°%s [C]", (int)(temp_cooled_disp + 0.5f), oilTempUnit);
            lv_label_set_text(ui_OIL_TEMP_Value_C, buf);

            // Style label based on critical (uses pan temp for critical check)
            bool value_critical = (g_vehicle_data.oil_temp_pan_f > OIL_TEMP_ValueCriticalF);
            static bool oil_temp_c_was_value_critical = false;
            if (value_critical != oil_temp_c_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_OIL_TEMP_Value_C, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_OIL_TEMP_Value_C, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_OIL_TEMP_Value_C, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_OIL_TEMP_Value_C, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_OIL_TEMP_Value_C, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_OIL_TEMP_Value_C, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_OIL_TEMP_Value_C, 0, 0);
                }
                oil_temp_c_was_value_critical = value_critical;
            }
        }

        bool critical = (g_vehicle_data.oil_temp_pan_f > OIL_TEMP_ValueCriticalF);
        updateCriticalLabel(ui_OIL_TEMP_VALUE_CRITICAL_Label, critical,
            &oil_temp_was_critical, &oil_temp_visible, &oil_temp_exit_time);
    }

    // ----- Water Temperature -----
    if (g_vehicle_data.water_temp_valid) {
        const char* waterTempUnit = getTempUnitStr(g_water_temp_unit);
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.water_temp_hot_f, g_water_temp_unit);
        float temp_cooled_disp = tempToDisplay((float)g_vehicle_data.water_temp_cooled_f, g_water_temp_unit);

        if (ui_W_TEMP_Value_H) {
            snprintf(buf, sizeof(buf), "%d°%s [H]", (int)(temp_hot_disp + 0.5f), waterTempUnit);
            lv_label_set_text(ui_W_TEMP_Value_H, buf);

            // Style label based on critical
            bool value_critical = (g_vehicle_data.water_temp_hot_f > W_TEMP_ValueCritical_F);
            static bool water_temp_was_value_critical = false;
            if (value_critical != water_temp_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_W_TEMP_Value_H, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_W_TEMP_Value_H, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_W_TEMP_Value_H, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_W_TEMP_Value_H, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_W_TEMP_Value_H, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_W_TEMP_Value_H, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_W_TEMP_Value_H, 0, 0);
                }
                water_temp_was_value_critical = value_critical;
            }
        }
        if (ui_W_TEMP_Value_C) {
            snprintf(buf, sizeof(buf), "%d°%s [C]", (int)(temp_cooled_disp + 0.5f), waterTempUnit);
            lv_label_set_text(ui_W_TEMP_Value_C, buf);

            // Style label based on critical (uses hot temp for critical check)
            bool value_critical = (g_vehicle_data.water_temp_hot_f > W_TEMP_ValueCritical_F);
            static bool water_temp_c_was_value_critical = false;
            if (value_critical != water_temp_c_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_W_TEMP_Value_C, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_W_TEMP_Value_C, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_W_TEMP_Value_C, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_W_TEMP_Value_C, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_W_TEMP_Value_C, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_W_TEMP_Value_C, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_W_TEMP_Value_C, 0, 0);
                }
                water_temp_c_was_value_critical = value_critical;
            }
        }
        if (ui_W_TEMP_Bar) {
            lv_bar_set_value(ui_W_TEMP_Bar, g_vehicle_data.water_temp_hot_f, LV_ANIM_ON);
            bool critical = (g_vehicle_data.water_temp_hot_f > W_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_W_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }

        bool critical = (g_vehicle_data.water_temp_hot_f > W_TEMP_ValueCritical_F);
        updateCriticalLabel(ui_W_TEMP_VALUE_CRITICAL_Label, critical,
            &water_temp_was_critical, &water_temp_visible, &water_temp_exit_time);
    }

    // ----- Trans Temperature -----
    if (g_vehicle_data.trans_temp_valid) {
        const char* transTempUnit = getTempUnitStr(g_trans_temp_unit);
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.trans_temp_hot_f, g_trans_temp_unit);
        float temp_cooled_disp = tempToDisplay((float)g_vehicle_data.trans_temp_cooled_f, g_trans_temp_unit);

        if (ui_TRAN_TEMP_Value_H) {
            snprintf(buf, sizeof(buf), "%d°%s [H]", (int)(temp_hot_disp + 0.5f), transTempUnit);
            lv_label_set_text(ui_TRAN_TEMP_Value_H, buf);

            // Style label based on critical
            bool value_critical = (g_vehicle_data.trans_temp_hot_f > TRAN_TEMP_ValueCritical_F);
            static bool trans_temp_was_value_critical = false;
            if (value_critical != trans_temp_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_TRAN_TEMP_Value_H, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_TRAN_TEMP_Value_H, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_TRAN_TEMP_Value_H, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_TRAN_TEMP_Value_H, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_TRAN_TEMP_Value_H, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_TRAN_TEMP_Value_H, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_TRAN_TEMP_Value_H, 0, 0);
                }
                trans_temp_was_value_critical = value_critical;
            }
        }
        if (ui_TRAN_TEMP_Value_C) {
            snprintf(buf, sizeof(buf), "%d°%s [C]", (int)(temp_cooled_disp + 0.5f), transTempUnit);
            lv_label_set_text(ui_TRAN_TEMP_Value_C, buf);

            // Style label based on critical (uses hot temp for critical check)
            bool value_critical = (g_vehicle_data.trans_temp_hot_f > TRAN_TEMP_ValueCritical_F);
            static bool trans_temp_c_was_value_critical = false;
            if (value_critical != trans_temp_c_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_TRAN_TEMP_Value_C, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_TRAN_TEMP_Value_C, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_TRAN_TEMP_Value_C, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_TRAN_TEMP_Value_C, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_TRAN_TEMP_Value_C, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_TRAN_TEMP_Value_C, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_TRAN_TEMP_Value_C, 0, 0);
                }
                trans_temp_c_was_value_critical = value_critical;
            }
        }
        if (ui_TRAN_TEMP_Bar) {
            lv_bar_set_value(ui_TRAN_TEMP_Bar, g_vehicle_data.trans_temp_hot_f, LV_ANIM_ON);
            bool critical = (g_vehicle_data.trans_temp_hot_f > TRAN_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_TRAN_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }

        bool critical = (g_vehicle_data.trans_temp_hot_f > TRAN_TEMP_ValueCritical_F);
        updateCriticalLabel(ui_TRAN_TEMP_VALUE_CRITICAL_Label, critical,
            &trans_temp_was_critical, &trans_temp_visible, &trans_temp_exit_time);
    }

    // ----- Steering Temperature -----
    if (g_vehicle_data.steer_temp_valid) {
        const char* steerTempUnit = getTempUnitStr(g_steer_temp_unit);
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.steer_temp_hot_f, g_steer_temp_unit);
        float temp_cooled_disp = tempToDisplay((float)g_vehicle_data.steer_temp_cooled_f, g_steer_temp_unit);

        if (ui_STEER_TEMP_Value_H) {
            snprintf(buf, sizeof(buf), "%d°%s [H]", (int)(temp_hot_disp + 0.5f), steerTempUnit);
            lv_label_set_text(ui_STEER_TEMP_Value_H, buf);

            // Style label based on critical
            bool value_critical = (g_vehicle_data.steer_temp_hot_f > STEER_TEMP_ValueCritical_F);
            static bool steer_temp_was_value_critical = false;
            if (value_critical != steer_temp_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_STEER_TEMP_Value_H, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_STEER_TEMP_Value_H, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_STEER_TEMP_Value_H, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_STEER_TEMP_Value_H, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_STEER_TEMP_Value_H, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_STEER_TEMP_Value_H, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_STEER_TEMP_Value_H, 0, 0);
                }
                steer_temp_was_value_critical = value_critical;
            }
        }
        if (ui_STEER_TEMP_Value_C) {
            snprintf(buf, sizeof(buf), "%d°%s [C]", (int)(temp_cooled_disp + 0.5f), steerTempUnit);
            lv_label_set_text(ui_STEER_TEMP_Value_C, buf);

            // Style label based on critical (uses hot temp for critical check)
            bool value_critical = (g_vehicle_data.steer_temp_hot_f > STEER_TEMP_ValueCritical_F);
            static bool steer_temp_c_was_value_critical = false;
            if (value_critical != steer_temp_c_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_STEER_TEMP_Value_C, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_STEER_TEMP_Value_C, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_STEER_TEMP_Value_C, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_STEER_TEMP_Value_C, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_STEER_TEMP_Value_C, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_STEER_TEMP_Value_C, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_STEER_TEMP_Value_C, 0, 0);
                }
                steer_temp_c_was_value_critical = value_critical;
            }
        }
        if (ui_STEER_TEMP_Bar) {
            lv_bar_set_value(ui_STEER_TEMP_Bar, g_vehicle_data.steer_temp_hot_f, LV_ANIM_ON);
            bool critical = (g_vehicle_data.steer_temp_hot_f > STEER_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_STEER_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }

        bool critical = (g_vehicle_data.steer_temp_hot_f > STEER_TEMP_ValueCritical_F);
        updateCriticalLabel(ui_STEER_TEMP_VALUE_CRITICAL_Label, critical,
            &steer_temp_was_critical, &steer_temp_visible, &steer_temp_exit_time);
    }

    // ----- Diff Temperature -----
    if (g_vehicle_data.diff_temp_valid) {
        const char* diffTempUnit = getTempUnitStr(g_diff_temp_unit);
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.diff_temp_hot_f, g_diff_temp_unit);
        float temp_cooled_disp = tempToDisplay((float)g_vehicle_data.diff_temp_cooled_f, g_diff_temp_unit);

        if (ui_DIFF_TEMP_Value_H) {
            snprintf(buf, sizeof(buf), "%d°%s [H]", (int)(temp_hot_disp + 0.5f), diffTempUnit);
            lv_label_set_text(ui_DIFF_TEMP_Value_H, buf);

            // Style label based on critical
            bool value_critical = (g_vehicle_data.diff_temp_hot_f > DIFF_TEMP_ValueCritical_F);
            static bool diff_temp_was_value_critical = false;
            if (value_critical != diff_temp_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_DIFF_TEMP_Value_H, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_DIFF_TEMP_Value_H, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_DIFF_TEMP_Value_H, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_DIFF_TEMP_Value_H, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_DIFF_TEMP_Value_H, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_DIFF_TEMP_Value_H, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_DIFF_TEMP_Value_H, 0, 0);
                }
                diff_temp_was_value_critical = value_critical;
            }
        }
        if (ui_DIFF_TEMP_Value_C) {
            snprintf(buf, sizeof(buf), "%d°%s [C]", (int)(temp_cooled_disp + 0.5f), diffTempUnit);
            lv_label_set_text(ui_DIFF_TEMP_Value_C, buf);

            // Style label based on critical (uses hot temp for critical check)
            bool value_critical = (g_vehicle_data.diff_temp_hot_f > DIFF_TEMP_ValueCritical_F);
            static bool diff_temp_c_was_value_critical = false;
            if (value_critical != diff_temp_c_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_DIFF_TEMP_Value_C, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_DIFF_TEMP_Value_C, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_DIFF_TEMP_Value_C, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_DIFF_TEMP_Value_C, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_DIFF_TEMP_Value_C, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_DIFF_TEMP_Value_C, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_DIFF_TEMP_Value_C, 0, 0);
                }
                diff_temp_c_was_value_critical = value_critical;
            }
        }
        if (ui_DIFF_TEMP_Bar) {
            lv_bar_set_value(ui_DIFF_TEMP_Bar, g_vehicle_data.diff_temp_hot_f, LV_ANIM_ON);
            bool critical = (g_vehicle_data.diff_temp_hot_f > DIFF_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_DIFF_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }

        bool critical = (g_vehicle_data.diff_temp_hot_f > DIFF_TEMP_ValueCritical_F);
        updateCriticalLabel(ui_DIFF_TEMP_VALUE_CRITICAL_Label, critical,
            &diff_temp_was_critical, &diff_temp_visible, &diff_temp_exit_time);
    }

    // ----- Fuel Trust -----
    if (g_vehicle_data.fuel_trust_valid) {
        int fuel = g_vehicle_data.fuel_trust_percent;

        if (smooth_fuel_trust < 0) {
            smooth_fuel_trust = (float)fuel;
        }
        else {
            smooth_fuel_trust = smooth_fuel_trust * (1.0f - SMOOTH_FACTOR) + fuel * SMOOTH_FACTOR;
        }
        int display_fuel = (int)(smooth_fuel_trust + 0.5f);

        if (ui_FUEL_TRUST_Bar) {
            lv_bar_set_value(ui_FUEL_TRUST_Bar, display_fuel, LV_ANIM_ON);
            bool critical = (fuel < FUEL_TRUST_ValueCritical);
            lv_obj_set_style_bg_color(ui_FUEL_TRUST_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }

        if (ui_FUEL_TRUST_Value) {
            snprintf(buf, sizeof(buf), "%d %%", fuel);
            lv_label_set_text(ui_FUEL_TRUST_Value, buf);

            // Style label based on critical
            bool value_critical = (fuel < FUEL_TRUST_ValueCritical);
            static bool fuel_trust_was_value_critical = false;
            if (value_critical != fuel_trust_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_FUEL_TRUST_Value, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_FUEL_TRUST_Value, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_FUEL_TRUST_Value, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_FUEL_TRUST_Value, 4, 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_FUEL_TRUST_Value, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_FUEL_TRUST_Value, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_FUEL_TRUST_Value, 0, 0);
                }
                fuel_trust_was_value_critical = value_critical;
            }
        }

        bool critical = (fuel < FUEL_TRUST_ValueCritical);
        updateCriticalLabel(ui_FUEL_TRUST_VALUE_CRITICAL_Label, critical,
            &fuel_trust_was_critical, &fuel_trust_visible, &fuel_trust_exit_time);
    }
}

//=================================================================
// CHART UPDATE FUNCTION
//=================================================================

void updateCharts() {
#if ENABLE_CHARTS
    uint32_t now = millis();

    // Only accumulate if data is valid
    if (g_vehicle_data.oil_pressure_valid) {
        oil_pressure_sum += g_vehicle_data.oil_pressure_psi;
        oil_pressure_samples++;
    }
    if (g_vehicle_data.oil_temp_valid) {
        oil_temp_sum += g_vehicle_data.oil_temp_pan_f;
        oil_temp_samples++;
    }
    if (g_vehicle_data.water_temp_valid) {
        water_temp_sum += g_vehicle_data.water_temp_hot_f;
        water_temp_samples++;
    }
    if (g_vehicle_data.trans_temp_valid) {
        transmission_temp_sum += g_vehicle_data.trans_temp_hot_f;
        transmission_temp_samples++;
    }
    if (g_vehicle_data.steer_temp_valid) {
        steering_temp_sum += g_vehicle_data.steer_temp_hot_f;
        steering_temp_samples++;
    }
    if (g_vehicle_data.diff_temp_valid) {
        differencial_temp_sum += g_vehicle_data.diff_temp_hot_f;
        differencial_temp_samples++;
    }
    if (g_vehicle_data.fuel_trust_valid) {
        fuel_trust_sum += g_vehicle_data.fuel_trust_percent;
        fuel_trust_samples++;
    }

    // Initialize bucket start times
    if (oil_pressure_bucket_start == 0) oil_pressure_bucket_start = now;
    if (oil_temp_bucket_start == 0) oil_temp_bucket_start = now;
    if (water_temp_bucket_start == 0) water_temp_bucket_start = now;
    if (transmission_temp_bucket_start == 0) transmission_temp_bucket_start = now;
    if (steering_temp_bucket_start == 0) steering_temp_bucket_start = now;
    if (differencial_temp_bucket_start == 0) differencial_temp_bucket_start = now;
    if (fuel_trust_start == 0) fuel_trust_start = now;

    // Push to charts every CHART_BUCKET_MS (only if we have samples)
    if ((now - oil_pressure_bucket_start) >= CHART_BUCKET_MS && chart_series_oil_press) {
        if (oil_pressure_samples > 0) {
            int32_t avg = oil_pressure_sum / oil_pressure_samples;
            if (avg < 0) avg = 0;
            if (avg > 150) avg = 150;
            shift_history(oil_press_history, avg);
            lv_chart_set_next_value(ui_OIL_PRESS_CHART, chart_series_oil_press, avg);
        }
        oil_pressure_sum = 0;
        oil_pressure_samples = 0;
        oil_pressure_bucket_start = now;
    }

    if ((now - oil_temp_bucket_start) >= CHART_BUCKET_MS && chart_series_oil_temp) {
        if (oil_temp_samples > 0) {
            int32_t avg = oil_temp_sum / oil_temp_samples;
            shift_history(oil_temp_history, avg);
            lv_chart_set_next_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, avg);
        }
        oil_temp_sum = 0;
        oil_temp_samples = 0;
        oil_temp_bucket_start = now;
    }

    if ((now - water_temp_bucket_start) >= CHART_BUCKET_MS && chart_series_water_temp) {
        if (water_temp_samples > 0) {
            int32_t avg = water_temp_sum / water_temp_samples;
            shift_history(water_temp_history, avg);
            lv_chart_set_next_value(ui_W_TEMP_CHART, chart_series_water_temp, avg);
        }
        water_temp_sum = 0;
        water_temp_samples = 0;
        water_temp_bucket_start = now;
    }

    if ((now - transmission_temp_bucket_start) >= CHART_BUCKET_MS && chart_series_transmission_temp) {
        if (transmission_temp_samples > 0) {
            int32_t avg = transmission_temp_sum / transmission_temp_samples;
            shift_history(transmission_temp_history, avg);
            lv_chart_set_next_value(ui_TRAN_TEMP_CHART, chart_series_transmission_temp, avg);
        }
        transmission_temp_sum = 0;
        transmission_temp_samples = 0;
        transmission_temp_bucket_start = now;
    }

    if ((now - steering_temp_bucket_start) >= CHART_BUCKET_MS && chart_series_steering_temp) {
        if (steering_temp_samples > 0) {
            int32_t avg = steering_temp_sum / steering_temp_samples;
            shift_history(steering_temp_history, avg);
            lv_chart_set_next_value(ui_STEER_TEMP_CHART, chart_series_steering_temp, avg);
        }
        steering_temp_sum = 0;
        steering_temp_samples = 0;
        steering_temp_bucket_start = now;
    }

    if ((now - differencial_temp_bucket_start) >= CHART_BUCKET_MS && chart_series_differencial_temp) {
        if (differencial_temp_samples > 0) {
            int32_t avg = differencial_temp_sum / differencial_temp_samples;
            shift_history(differencial_temp_history, avg);
            lv_chart_set_next_value(ui_DIFF_TEMP_CHART, chart_series_differencial_temp, avg);
        }
        differencial_temp_sum = 0;
        differencial_temp_samples = 0;
        differencial_temp_bucket_start = now;
    }

    if ((now - fuel_trust_start) >= CHART_BUCKET_MS && chart_series_fuel_trust) {
        if (fuel_trust_samples > 0) {
            int32_t avg = fuel_trust_sum / fuel_trust_samples;
            shift_history(fuel_trust_history, avg);
            lv_chart_set_next_value(ui_FUEL_TRUST_CHART, chart_series_fuel_trust, avg);
        }
        fuel_trust_sum = 0;
        fuel_trust_samples = 0;
        fuel_trust_start = now;
    }

    // Update blink phase for critical bars
    if (now - g_last_blink_toggle >= CHART_BLINK_INTERVAL_MS) {
        g_critical_blink_phase = !g_critical_blink_phase;
        g_last_blink_toggle = now;

        // Invalidate all charts with draw callbacks
        if (ui_OIL_PRESS_CHART) lv_obj_invalidate(ui_OIL_PRESS_CHART);
        if (ui_OIL_TEMP_CHART) lv_obj_invalidate(ui_OIL_TEMP_CHART);
        if (ui_W_TEMP_CHART) lv_obj_invalidate(ui_W_TEMP_CHART);
        if (ui_TRAN_TEMP_CHART) lv_obj_invalidate(ui_TRAN_TEMP_CHART);
        if (ui_STEER_TEMP_CHART) lv_obj_invalidate(ui_STEER_TEMP_CHART);
        if (ui_DIFF_TEMP_CHART) lv_obj_invalidate(ui_DIFF_TEMP_CHART);
        if (ui_FUEL_TRUST_CHART) lv_obj_invalidate(ui_FUEL_TRUST_CHART);
    }
#endif
}

//=================================================================
// CHART INITIALIZATION HELPER
//=================================================================

// Configures each chart widget - sets type (bar), range (min/max), creates the data series, attaches the draw callback for critical coloring, and initializes with empty data.
void initChart(lv_obj_t* chart, lv_chart_series_t** series, int min_val, int max_val,
    int32_t* history, void (*draw_cb)(lv_event_t*)) {
    if (!chart) return;

    // Remove existing series
    lv_chart_series_t* ser;
    while ((ser = lv_chart_get_series_next(chart, NULL)) != NULL) {
        lv_chart_remove_series(chart, ser);
    }

    lv_chart_set_type(chart, LV_CHART_TYPE_BAR);
    lv_chart_set_point_count(chart, CHART_POINTS);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, min_val, max_val);
    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_div_line_count(chart, 0, 0);

    *series = lv_chart_add_series(chart, lv_color_hex(hexRed), LV_CHART_AXIS_PRIMARY_Y);

    if (draw_cb) {
        lv_obj_add_event_cb(chart, draw_cb, LV_EVENT_DRAW_TASK_ADDED, NULL);
        lv_obj_add_flag(chart, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS);
    }

    // Initialize with no data
    for (int i = 0; i < CHART_POINTS; i++) {
        lv_chart_set_next_value(chart, *series, LV_CHART_POINT_NONE);
        if (history) history[i] = 0;
    }
    lv_chart_refresh(chart);
}

//=================================================================
// SETUP
//=================================================================

void setup() {
    // =========================================================
    // CHECK FOR USB MSC MODE FIRST - before anything else!
    // Hold BOOT button (GPIO0) during power-on to enter USB drive mode
    // =========================================================
#if ENABLE_USB_MSC
    if (checkUSBMSCMode()) {
        runUSBMSCMode();
    }
    // Normal boot continues here...
#endif

    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("   370zMonitor v4.1 - Unit Conversion");
    Serial.println("========================================");

    if (psramFound()) {
        Serial.printf("✓ PSRAM: %u bytes total, %u free\n", ESP.getPsramSize(), ESP.getFreePsram());
    }
    else {
        Serial.println("✗ WARNING: No PSRAM detected!");
    }

    // Load unit preferences from flash
    loadUnitPreferences();

    // I2C init
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
    Wire.setTimeOut(50);
    delay(50);

    Serial.println("[1/8] IO Expander...");
    g_ioexp_ok = initIOExtension();
    Serial.printf("      %s\n", g_ioexp_ok ? "OK" : "FAILED");

    Serial.println("[2/8] Touch controller...");
    delay(100);
    touch.begin();
    touch.setRotation(0);
    Serial.println("      GT911 initialized");

    Serial.println("[3/8] Display init...");
    gfx->begin();
    gfx->fillScreen(0x0000);
    Serial.println("      OK");

    Serial.println("[4/8] Backlight...");
    setBacklight(true);
    delay(100);
    Serial.println("      OK");

    Serial.println("[5/8] LVGL init...");
    lv_init();

    size_t buf_bytes = LVGL_BUFFER_SIZE * sizeof(lv_color_t);

    disp_draw_buf1 = (uint8_t*)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!disp_draw_buf1) {
        disp_draw_buf1 = (uint8_t*)heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    }
    if (!disp_draw_buf1) {
        Serial.println("      FATAL: Buffer 1 alloc failed!");
        while (1) delay(100);
    }

    disp_draw_buf2 = (uint8_t*)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!disp_draw_buf2) {
        disp_draw_buf2 = (uint8_t*)heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    }
    if (!disp_draw_buf2) {
        Serial.println("      FATAL: Buffer 2 alloc failed!");
        while (1) delay(100);
    }

    disp = lv_display_create(800, 480);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, disp_draw_buf1, disp_draw_buf2, buf_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);

    indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touch_read);

    Serial.println("      OK");

    Serial.println("[6/8] Loading UI...");
    ui_init();

    if (!ui_Screen1) {
        Serial.println("      FATAL: ui_Screen1 is NULL!");
        while (1) delay(100);
    }
    Serial.println("      OK");

    Serial.println("[7/8] Data providers...");
    initSensors();
    initOBD();
    Serial.println("      OK");

#if ENABLE_SD_LOGGING
    Serial.println("[8/8] SD Card...");
    if (sdInit()) {
        sdTestWrite();
        sdStartSession();
    }
    Serial.println("      OK");
#endif

    // Bar animation speeds
    lv_obj_t* bars[] = { ui_OIL_PRESS_Bar, ui_OIL_TEMP_Bar, ui_FUEL_TRUST_Bar,
                        ui_W_TEMP_Bar, ui_TRAN_TEMP_Bar, ui_STEER_TEMP_Bar, ui_DIFF_TEMP_Bar };
    for (int i = 0; i < 7; i++) {
        if (bars[i]) lv_obj_set_style_anim_duration(bars[i], 100, LV_PART_MAIN);
    }

    // Create utility box
    if (ui_Screen1) {
        utility_box = lv_obj_create(ui_Screen1);
#if ENABLE_SD_LOGGING
        lv_obj_set_size(utility_box, 105, 105);
#else
        lv_obj_set_size(utility_box, 105, 85);
#endif
        lv_obj_align(utility_box, LV_ALIGN_TOP_LEFT, 5, 5);
        lv_obj_set_style_bg_color(utility_box, lv_color_hex(0x444444), 0);
        lv_obj_set_style_bg_opa(utility_box, LV_OPA_COVER, 0);  // Fully opaque background (no transparency)
        lv_obj_set_style_opa(utility_box, LV_OPA_TRANSP, 0);    // Start hidden (double-tap to reveal)
        lv_obj_set_style_border_color(utility_box, lv_color_hex(0x444444), 0);
        lv_obj_set_style_border_width(utility_box, 1, 0);
        lv_obj_set_style_radius(utility_box, 5, 0);
        lv_obj_set_style_pad_all(utility_box, 5, 0);
        lv_obj_add_flag(utility_box, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_remove_flag(utility_box, LV_OBJ_FLAG_SCROLLABLE);

        // Event callbacks for tap and long-press
        lv_obj_add_event_cb(utility_box, utility_box_tap_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_add_event_cb(utility_box, utility_box_press_cb, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(utility_box, utility_box_release_cb, LV_EVENT_RELEASED, NULL);

        // FPS/CPU/BRI/SD label
        utility_label = lv_label_create(utility_box);
#if ENABLE_SD_LOGGING
        lv_label_set_text(utility_label, "--- FPS\n---% CPU\n---% BRI\nSD: ---");
#else
        lv_label_set_text(utility_label, "--- FPS\n---% CPU\n---% BRI");
#endif
        lv_obj_set_style_text_color(utility_label, lv_color_hex(0xffff00), 0);
        lv_obj_set_style_text_font(utility_label, &lv_font_montserrat_14, 0);
        lv_obj_align(utility_label, LV_ALIGN_TOP_LEFT, 0, 0);

        // Mode indicator (DEMO/LIVE)
        mode_indicator = lv_label_create(utility_box);
        lv_obj_set_style_text_font(mode_indicator, &lv_font_montserrat_14, 0);
        lv_obj_align(mode_indicator, LV_ALIGN_BOTTOM_LEFT, 0, 0);
        updateModeIndicator();

        Serial.println("[UI] Utility box created");
    }

    // Setup unit tap handlers
    setupUnitTapHandlers();

    setBrightness(255);

    // Initialize all charts with draw callbacks
#if ENABLE_CHARTS
    initChart(ui_OIL_PRESS_CHART, &chart_series_oil_press, 0, 150, oil_press_history, oil_press_chart_draw_cb);
    initChart(ui_OIL_TEMP_CHART, &chart_series_oil_temp, OIL_TEMP_Min_F, OIL_TEMP_Max_F, oil_temp_history, oil_temp_chart_draw_cb);
    initChart(ui_W_TEMP_CHART, &chart_series_water_temp, W_TEMP_Min_F, W_TEMP_Max_F, water_temp_history, water_temp_chart_draw_cb);
    initChart(ui_TRAN_TEMP_CHART, &chart_series_transmission_temp, TRAN_TEMP_Min_F, TRAN_TEMP_Max_F, transmission_temp_history, trans_temp_chart_draw_cb);
    initChart(ui_STEER_TEMP_CHART, &chart_series_steering_temp, STEER_TEMP_Min_F, STEER_TEMP_Max_F, steering_temp_history, steer_temp_chart_draw_cb);
    initChart(ui_DIFF_TEMP_CHART, &chart_series_differencial_temp, DIFF_TEMP_Min_F, DIFF_TEMP_Max_F, differencial_temp_history, diff_temp_chart_draw_cb);
    initChart(ui_FUEL_TRUST_CHART, &chart_series_fuel_trust, 0, 100, fuel_trust_history, fuel_trust_chart_draw_cb);
    Serial.println("[CHARTS] All charts initialized with draw callbacks");
#endif

    // CRITICAL: Reset UI to show "---" on startup in live mode
    resetVehicleData();
    resetSmoothingState();
    resetUIElements();
    resetCharts();

    Serial.println("\nForcing initial render...");
    uint32_t t0 = millis();
    lv_refr_now(NULL);
    Serial.printf("Initial render took: %u ms\n", millis() - t0);

    Serial.println("\n========================================");
    Serial.println("         RUNNING");
    Serial.printf("  Mode: %s\n", g_demo_mode ? "DEMO" : "LIVE");
    Serial.printf("  Pressure Unit: %s\n", getPressureUnitStr(g_pressure_unit));
    Serial.println("  Temp units: Per-gauge (tap to cycle)");
    Serial.println("  Hold utility box 5s to toggle mode");
    Serial.println("========================================\n");
}

//=================================================================
// MAIN LOOP
//=================================================================

void loop() {
    static uint32_t frame_start = 0;
    frame_start = millis();

    static uint32_t last_tick = 0;
    static uint32_t last_status = 0;
    static uint32_t last_update = 0;

    uint32_t now = millis();

    // LVGL tick
    if (last_tick == 0) last_tick = now;
    lv_tick_inc(now - last_tick);
    last_tick = now;

    loop_count++;

    // Check for long press on utility box (for demo mode toggle)
    checkUtilityLongPress();

    // Status every 1 second
    if (now - last_status >= 1000) {
        int cpu_percent = (cpu_busy_time * 100) / 1000;
        if (cpu_percent > 100) cpu_percent = 100;

        update_utility_label(flush_count, cpu_percent);

        Serial.printf("[STATUS] loops=%u flushes=%u cpu=%u%% heap=%u psram=%u mode=%s\n",
            loop_count, flush_count, (cpu_busy_time * 100) / 1000,
            ESP.getFreeHeap(), ESP.getFreePsram(),
            g_demo_mode ? "DEMO" : "LIVE");
        flush_count = 0;
        cpu_busy_time = 0;
        last_status = now;
    }

    // Update data and UI
    #if ENABLE_UI_UPDATES

    if (now - last_update >= UPDATE_INTERVAL_MS) {
        update_count++;

        // Step 1: Get data from appropriate provider
        updateVehicleData();

        // Step 2: Update UI from g_vehicle_data
        updateUI();

        // Step 3: Update charts
        updateCharts();

        // Step 4: Log data to SD card
        #if ENABLE_SD_LOGGING

        sdLogData();

        #endif


        last_update = now;
    }
    #endif

    uint32_t t0 = micros();
    lv_timer_handler();
    uint32_t dt = micros() - t0;

    if (dt > 100000) {
        Serial.printf("[WARN] lv_timer_handler took %u us!\n", dt);
    }

    uint32_t elapsed = millis() - frame_start;
    cpu_busy_time += elapsed;
    if (elapsed < FRAME_TIME_MS) {
        delay(FRAME_TIME_MS - elapsed);
    }
}
