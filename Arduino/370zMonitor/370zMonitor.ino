//-----------------------------------------------------------------

/*
 * 370zMonitor v4 - Data Provider Architecture
 * Supports Demo Mode (animated values) and Live Mode (sensor/OBD data)
 * ESP32-S3 with PSRAM, LVGL, GT911 Touch
 */

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <lvgl.h>
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>
#include <TAMC_GT911.h>  // Touch controller

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
#define UPDATE_INTERVAL_MS  150 // Update every 150ms

//-----------------------------------------------------------------

// ===== DATA PROVIDER ARCHITECTURE =====
// This structure holds all vehicle data from either demo or real sources
// The UI layer reads from this - it doesn't care where the data comes from

struct VehicleData {
    // Oil Pressure
    int oil_pressure_psi;
    bool oil_pressure_valid;
    
    // Oil Temperature (dual sensor: pan and cooled line)
    int oil_temp_pan_f;
    int oil_temp_cooled_f;
    bool oil_temp_valid;
    
    // Water/Coolant Temperature (dual sensor: hot and cooled)
    int water_temp_hot_f;
    int water_temp_cooled_f;
    bool water_temp_valid;
    
    // Transmission Temperature (dual sensor)
    int trans_temp_hot_f;
    int trans_temp_cooled_f;
    bool trans_temp_valid;
    
    // Power Steering Temperature (dual sensor)
    int steer_temp_hot_f;
    int steer_temp_cooled_f;
    bool steer_temp_valid;
    
    // Differential Temperature (dual sensor)
    int diff_temp_hot_f;
    int diff_temp_cooled_f;
    bool diff_temp_valid;
    
    // Fuel Trust (confidence percentage)
    int fuel_trust_percent;
    bool fuel_trust_valid;
    
    // OBD Data
    int rpm;
    bool rpm_valid;
};

// Global vehicle data - updated by data providers, read by UI
static VehicleData g_vehicle_data = {0};

// Demo mode flag - toggled by 5-second hold on Utility block
static bool g_demo_mode = false;  // Start in LIVE mode (default)

// Reset all vehicle data to invalid/zero state
void resetVehicleData() {
    memset(&g_vehicle_data, 0, sizeof(g_vehicle_data));
    // All _valid flags are now false
}

// Forward declarations for reset functions (defined after variables)
void resetSmoothingState();
void resetDemoState();
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
int OIL_PRESS_ValueCriticalLow = 20;

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
static float smooth_oil_pressure = 0.0f;
static float smooth_oil_temp_f = 0.0f;
static float smooth_fuel_trust = 0.0f;

// Smoothing factor: 0.3 = responsive, 0.1 = very smooth
#define SMOOTH_FACTOR 0.3f

// Reset smoothing variables (must be after variable declarations)
void resetSmoothingState() {
    smooth_oil_pressure = 0.0f;
    smooth_oil_temp_f = 0.0f;
    smooth_fuel_trust = 0.0f;
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
extern lv_obj_t * ui_W_TEMP_Value_H;
extern lv_obj_t * ui_W_TEMP_Value_C;

//TRAN TEMP [H/C]
extern lv_obj_t * ui_TRAN_TEMP_Bar;
extern lv_obj_t * ui_TRAN_TEMP_CHART;
extern lv_obj_t * ui_TRAN_TEMP_Value_H;
extern lv_obj_t * ui_TRAN_TEMP_Value_C;

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

#pragma endregion UI Objects

//-----------------------------------------------------------------

extern lv_obj_t * ui_Screen1;

// Display
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
static lv_obj_t * utility_box = NULL;
static lv_obj_t * g_dim_overlay = NULL;
static lv_timer_t * g_util_single_tap_timer = NULL;
static lv_obj_t * utility_label = NULL;
static lv_obj_t * mode_indicator = NULL;  // Shows "DEMO" or "LIVE"
static bool utilities_visible = true;

// Long press tracking for demo mode toggle
static uint32_t g_utility_press_start = 0;
static bool g_utility_long_press_triggered = false;
#define DEMO_MODE_TOGGLE_HOLD_MS 5000  // 5 seconds to toggle demo mode

// Chart series
static lv_chart_series_t * chart_series_oil_press = NULL;
static lv_chart_series_t * chart_series_oil_temp = NULL;

// Chart history
static int32_t pressure_sum = 0;
static uint32_t pressure_samples = 0;
static uint32_t pressure_bucket_start = 0;
static int32_t temp_sum = 0;
static uint32_t temp_samples = 0;
static uint32_t temp_bucket_start = 0;
#define CHART_BUCKET_MS 5000

#define CHART_POINTS 24
static int32_t oil_press_history[CHART_POINTS] = {0};
static int32_t oil_temp_history[CHART_POINTS] = {0};

// Critical bar blinking
static bool g_critical_blink_phase = false;
static uint32_t g_last_blink_toggle = 0;
#define CHART_BLINK_INTERVAL_MS 200

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
    
    // Reset all labels to "---"
    if (ui_OIL_PRESS_Value) lv_label_set_text(ui_OIL_PRESS_Value, "--- PSI");
    if (ui_OIL_TEMP_Value_P) lv_label_set_text(ui_OIL_TEMP_Value_P, "---°F [P]");
    if (ui_OIL_TEMP_Value_C) lv_label_set_text(ui_OIL_TEMP_Value_C, "---°F [C]");
    if (ui_W_TEMP_Value_H) lv_label_set_text(ui_W_TEMP_Value_H, "---°F [H]");
    if (ui_W_TEMP_Value_C) lv_label_set_text(ui_W_TEMP_Value_C, "---°F [C]");
    if (ui_TRAN_TEMP_Value_H) lv_label_set_text(ui_TRAN_TEMP_Value_H, "---°F [H]");
    if (ui_TRAN_TEMP_Value_C) lv_label_set_text(ui_TRAN_TEMP_Value_C, "---°F [C]");
    if (ui_STEER_TEMP_Value_H) lv_label_set_text(ui_STEER_TEMP_Value_H, "---°F [H]");
    if (ui_STEER_TEMP_Value_C) lv_label_set_text(ui_STEER_TEMP_Value_C, "---°F [C]");
    if (ui_DIFF_TEMP_Value_H) lv_label_set_text(ui_DIFF_TEMP_Value_H, "---°F [H]");
    if (ui_DIFF_TEMP_Value_C) lv_label_set_text(ui_DIFF_TEMP_Value_C, "---°F [C]");
    if (ui_FUEL_TRUST_Value) lv_label_set_text(ui_FUEL_TRUST_Value, "--- %");
    
    // Reset oil pressure label styling to default
    if (ui_OIL_PRESS_Value) {
        lv_obj_set_style_text_color(ui_OIL_PRESS_Value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_OIL_PRESS_Value, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_OIL_PRESS_Value, 0, 0);
    }
    
    // Hide critical label
    if (ui_OIL_PRESS_VALUE_CRITICAL_Label) {
        lv_anim_delete(ui_OIL_PRESS_VALUE_CRITICAL_Label, NULL);
        lv_obj_set_style_text_opa(ui_OIL_PRESS_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(ui_OIL_PRESS_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
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
        lv_chart_set_all_value(ui_OIL_PRESS_CHART, chart_series_oil_press, -100);
        lv_chart_refresh(ui_OIL_PRESS_CHART);
    }
    
    // Reset oil temp chart
    if (ui_OIL_TEMP_CHART && chart_series_oil_temp) {
        for (int i = 0; i < CHART_POINTS; i++) {
            oil_temp_history[i] = 0;
        }
        lv_chart_set_all_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, -100);
        lv_chart_refresh(ui_OIL_TEMP_CHART);
    }
    
    // Reset chart accumulation state
    pressure_sum = 0;
    pressure_samples = 0;
    pressure_bucket_start = 0;
    temp_sum = 0;
    temp_samples = 0;
    temp_bucket_start = 0;
    #endif
}

//=================================================================
// DEMO DATA PROVIDER
// Generates animated/simulated values for testing and spectacle
//=================================================================

#pragma region Demo Data Provider

// Demo animation state
static struct {
    uint32_t oil_press_anim_start;
    uint32_t fuel_trust_dip_time;
    bool fuel_trust_dipping;
    int oil_temp;
    int water_temp;
    int trans_temp;
    int steer_temp;
    int diff_temp;
    int rpm;
    uint32_t next_rev_time;
    bool revving;
} g_demo_state = {0};

// Reset demo state to initial values
void resetDemoState() {
    memset(&g_demo_state, 0, sizeof(g_demo_state));
    // Set reasonable starting values for demo
    g_demo_state.oil_temp = 210;
    g_demo_state.water_temp = 195;
    g_demo_state.trans_temp = 180;
    g_demo_state.steer_temp = 140;
    g_demo_state.diff_temp = 160;
    g_demo_state.rpm = 750;
}

void updateDemoData() {
    uint32_t now = millis();
    
    // ----- Oil Pressure: Sine wave oscillation -----
    if (g_demo_state.oil_press_anim_start == 0) {
        g_demo_state.oil_press_anim_start = now;
    }
    
    uint32_t cycle_ms = 16000;  // 16 second full cycle
    uint32_t elapsed = (now - g_demo_state.oil_press_anim_start) % cycle_ms;
    float angle = (float)elapsed / (float)cycle_ms * 2.0f * PI;
    float progress = (1.0f - cos(angle)) / 2.0f;
    
    g_vehicle_data.oil_pressure_psi = OIL_PRESS_Min_PSI + 
        (int)(progress * (OIL_PRESS_Max_PSI - OIL_PRESS_Min_PSI) + 0.5f);
    g_vehicle_data.oil_pressure_valid = true;
    
    // ----- Oil Temperature: Random walk -----
    g_demo_state.oil_temp += random(-2, 3);
    if (g_demo_state.oil_temp > OIL_TEMP_Max_F) g_demo_state.oil_temp = OIL_TEMP_Max_F;
    if (g_demo_state.oil_temp < OIL_TEMP_Min_F) g_demo_state.oil_temp = OIL_TEMP_Min_F;
    
    g_vehicle_data.oil_temp_pan_f = g_demo_state.oil_temp;
    g_vehicle_data.oil_temp_cooled_f = g_demo_state.oil_temp - 15;
    g_vehicle_data.oil_temp_valid = true;
    
    // ----- Water Temperature: Random walk -----
    g_demo_state.water_temp += random(-1, 2);
    if (g_demo_state.water_temp > W_TEMP_Max_F) g_demo_state.water_temp = W_TEMP_Max_F;
    if (g_demo_state.water_temp < W_TEMP_Min_F) g_demo_state.water_temp = W_TEMP_Min_F;
    
    g_vehicle_data.water_temp_hot_f = g_demo_state.water_temp;
    g_vehicle_data.water_temp_cooled_f = g_demo_state.water_temp - 20;
    g_vehicle_data.water_temp_valid = true;
    
    // ----- Trans Temperature: Random walk -----
    g_demo_state.trans_temp += random(-1, 2);
    if (g_demo_state.trans_temp > TRAN_TEMP_Max_F) g_demo_state.trans_temp = TRAN_TEMP_Max_F;
    if (g_demo_state.trans_temp < TRAN_TEMP_Min_F) g_demo_state.trans_temp = TRAN_TEMP_Min_F;
    
    g_vehicle_data.trans_temp_hot_f = g_demo_state.trans_temp;
    g_vehicle_data.trans_temp_cooled_f = g_demo_state.trans_temp - 25;
    g_vehicle_data.trans_temp_valid = true;
    
    // ----- Steering Temperature: Random walk -----
    g_demo_state.steer_temp += random(-1, 2);
    if (g_demo_state.steer_temp > STEER_TEMP_Max_F) g_demo_state.steer_temp = STEER_TEMP_Max_F;
    if (g_demo_state.steer_temp < STEER_TEMP_Min_F) g_demo_state.steer_temp = STEER_TEMP_Min_F;
    
    g_vehicle_data.steer_temp_hot_f = g_demo_state.steer_temp;
    g_vehicle_data.steer_temp_cooled_f = g_demo_state.steer_temp - 20;
    g_vehicle_data.steer_temp_valid = true;
    
    // ----- Diff Temperature: Random walk -----
    g_demo_state.diff_temp += random(-1, 2);
    if (g_demo_state.diff_temp > DIFF_TEMP_Max_F) g_demo_state.diff_temp = DIFF_TEMP_Max_F;
    if (g_demo_state.diff_temp < DIFF_TEMP_Min_F) g_demo_state.diff_temp = DIFF_TEMP_Min_F;
    
    g_vehicle_data.diff_temp_hot_f = g_demo_state.diff_temp;
    g_vehicle_data.diff_temp_cooled_f = g_demo_state.diff_temp - 30;
    g_vehicle_data.diff_temp_valid = true;
    
    // ----- Fuel Trust: Periodic dips -----
    if (!g_demo_state.fuel_trust_dipping) {
        g_vehicle_data.fuel_trust_percent = 95 + random(0, 6);  // 95-100%
        
        if (g_demo_state.fuel_trust_dip_time == 0) {
            g_demo_state.fuel_trust_dip_time = now + random(5000, 20001);
        }
        
        if (now >= g_demo_state.fuel_trust_dip_time) {
            g_demo_state.fuel_trust_dipping = true;
            g_vehicle_data.fuel_trust_percent = 40 + random(0, 35);  // Dip to 40-75%
        }
    } else {
        g_vehicle_data.fuel_trust_percent += random(10, 20);  // Fast recovery
        
        if (g_vehicle_data.fuel_trust_percent >= 95) {
            g_vehicle_data.fuel_trust_percent = 95 + random(0, 6);
            g_demo_state.fuel_trust_dipping = false;
            g_demo_state.fuel_trust_dip_time = now + random(5000, 20001);
        }
    }
    g_vehicle_data.fuel_trust_valid = true;
    
    // ----- RPM: Simulated idle with occasional revs -----
    if (!g_demo_state.revving) {
        g_demo_state.rpm = 700 + random(0, 100);  // Idle: 700-800
        if (g_demo_state.next_rev_time == 0) {
            g_demo_state.next_rev_time = now + random(8000, 15001);
        }
        if (now >= g_demo_state.next_rev_time) {
            g_demo_state.revving = true;
            g_demo_state.rpm = 2500 + random(0, 2000);  // Rev up
        }
    } else {
        g_demo_state.rpm -= random(200, 400);
        if (g_demo_state.rpm <= 800) {
            g_demo_state.rpm = 750;
            g_demo_state.revving = false;
            g_demo_state.next_rev_time = now + random(8000, 15001);
        }
    }
    
    g_vehicle_data.rpm = g_demo_state.rpm;
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
    // int raw_oil_press = analogRead(OIL_PRESS_PIN);
    // g_vehicle_data.oil_pressure_psi = mapOilPressure(raw_oil_press);
    // g_vehicle_data.oil_pressure_valid = true;
    //
    // int raw_oil_temp = analogRead(OIL_TEMP_PIN);
    // g_vehicle_data.oil_temp_pan_f = thermistorToFahrenheit(raw_oil_temp);
    // g_vehicle_data.oil_temp_valid = true;
    
    // For now, mark all sensor data as invalid (no readings)
    // This will cause the UI to show "---" or similar
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
// DATA PROVIDER DISPATCHER
// Calls the appropriate data provider based on mode
//=================================================================

void updateVehicleData() {
    if (g_demo_mode) {
        // Demo mode: use animated/simulated values
        updateDemoData();
    } else {
        // Live mode: read from real sensors and OBD
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

static void oil_press_chart_draw_cb(lv_event_t * e) {
    lv_draw_task_t * draw_task = lv_event_get_draw_task(e);
    lv_draw_fill_dsc_t * fill_dsc = lv_draw_task_get_fill_dsc(draw_task);
    if(fill_dsc == NULL) return;

    lv_draw_dsc_base_t * base_dsc = (lv_draw_dsc_base_t *)fill_dsc;
    if(base_dsc->part != LV_PART_ITEMS) return;

    uint32_t idx = base_dsc->id2;
    if(idx >= CHART_POINTS) return;

    int32_t psi = oil_press_history[idx];
    bool is_critical = (psi < OIL_PRESS_ValueCriticalLow) || (psi > OIL_PRESS_ValueCriticalAbsolute);
    
    if (is_critical) {
        fill_dsc->color = g_critical_blink_phase ? lv_color_hex(0xFFFFFF) : lv_color_hex(hexRed);
    } else {
        fill_dsc->color = lv_color_hex(hexOrange);
    }
}

static void oil_temp_chart_draw_cb(lv_event_t * e) {
    lv_draw_task_t * draw_task = lv_event_get_draw_task(e);
    lv_draw_fill_dsc_t * fill_dsc = lv_draw_task_get_fill_dsc(draw_task);
    if(fill_dsc == NULL) return;

    lv_draw_dsc_base_t * base_dsc = (lv_draw_dsc_base_t *)fill_dsc;
    if(base_dsc->part != LV_PART_ITEMS) return;

    uint32_t idx = base_dsc->id2;
    if(idx >= CHART_POINTS) return;

    int32_t temp_f = oil_temp_history[idx];
    bool is_critical = (temp_f > OIL_TEMP_ValueCriticalF);
    
    if (is_critical) {
        fill_dsc->color = g_critical_blink_phase ? lv_color_hex(0xFFFFFF) : lv_color_hex(hexRed);
    } else {
        fill_dsc->color = lv_color_hex(hexOrange);
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
            lv_obj_set_style_text_color(mode_indicator, lv_color_hex(0xFF00FF), 0);  // Magenta
        } else {
            lv_label_set_text(mode_indicator, "LIVE");
            lv_obj_set_style_text_color(mode_indicator, lv_color_hex(0x00FF00), 0);  // Green
        }
    }
}

static void utility_box_single_tap_cb(lv_timer_t * t) {
    LV_UNUSED(t);
    g_util_single_tap_timer = NULL;

    // Toggle brightness: 100% <-> 35%
    if (g_brightness_level == 255) {
        setBrightness(89);
    } else {
        setBrightness(255);
    }

    Serial.printf("[UI] Single-tap: Brightness -> %d%%\n", (g_brightness_level * 100) / 255);
}

static void utility_box_tap_cb(lv_event_t * e) {
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
            } else {
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
static void utility_box_press_cb(lv_event_t * e) {
    LV_UNUSED(e);
    g_utility_press_start = millis();
    g_utility_long_press_triggered = false;
    
    // Visual feedback: darken background on press
    if (utility_box) {
        lv_obj_set_style_bg_color(utility_box, lv_color_hex(0x666666), 0);  // Lighter gray
        lv_obj_set_style_bg_opa(utility_box, LV_OPA_90, 0);  // More opaque
    }
}

// Release event - reset tracking + visual feedback
static void utility_box_release_cb(lv_event_t * e) {
    LV_UNUSED(e);
    g_utility_press_start = 0;
    g_utility_long_press_triggered = false;
    
    // Visual feedback: restore normal background
    if (utility_box) {
        lv_obj_set_style_bg_color(utility_box, lv_color_hex(0x444444), 0);  // Original dark gray
        lv_obj_set_style_bg_opa(utility_box, LV_OPA_70, 0);  // Original opacity
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
        char buf[48];
        int bri_percent = (g_brightness_level * 100) / 255;
        snprintf(buf, sizeof(buf), "%3d FPS\n%3d%% CPU\n%3d%% BRI", fps, cpu_percent, bri_percent);
        lv_label_set_text(utility_label, buf);
    }
}

#pragma endregion Utility Box Callbacks

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
        lv_obj_t * top = lv_layer_top();
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
    } else {
        lv_obj_clear_flag(g_dim_overlay, LV_OBJ_FLAG_HIDDEN);
        uint8_t opa = (uint8_t)(255 - brightness);
        lv_obj_set_style_bg_opa(g_dim_overlay, opa, 0);
    }

    Serial.printf("[BRIGHTNESS] UI dim -> %d%%\n", (brightness * 100) / 255);
}

#pragma endregion Hardware Functions

//-----------------------------------------------------------------

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

//-----------------------------------------------------------------

#pragma region Touch Callbacks

#define MAX_CONSECUTIVE_INVALID 50
#define TOUCH_RESET_COOLDOWN_MS 5000

void my_touch_read(lv_indev_t *indev, lv_indev_data_t *data) {
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
        
        // Touch calibration
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
    } else {
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
// UI UPDATE FUNCTION
// Reads from g_vehicle_data and updates all UI elements
// This is the SAME code regardless of demo/live mode
//=================================================================

void updateUI() {
    // ----- Oil Pressure -----
    // Only update UI if we have valid data
    if (g_vehicle_data.oil_pressure_valid) {
        int pressure = g_vehicle_data.oil_pressure_psi;
        
        // Smooth animation
        smooth_oil_pressure = smooth_oil_pressure * (1.0f - SMOOTH_FACTOR) + pressure * SMOOTH_FACTOR;
        int display_pressure = (int)(smooth_oil_pressure + 0.5f);
        
        // Update bar
        if (ui_OIL_PRESS_Bar) {
            lv_bar_set_value(ui_OIL_PRESS_Bar, display_pressure, LV_ANIM_ON);
            bool critical = isOilPressureCritical();
            lv_obj_set_style_bg_color(ui_OIL_PRESS_Bar, 
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange), 
                LV_PART_INDICATOR);
        }
        
        // Update label
        static int last_displayed_pressure = -1;
        if (ui_OIL_PRESS_Value && display_pressure != last_displayed_pressure) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d PSI", display_pressure);
            lv_label_set_text(ui_OIL_PRESS_Value, buf);
            last_displayed_pressure = display_pressure;
        }
        
        // Style label based on critical
        if (ui_OIL_PRESS_Value) {
            static bool was_value_critical = false;
            bool value_critical = isOilPressureCritical();
            
            if (value_critical != was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_OIL_PRESS_Value, lv_color_hex(0x000000), 0);
                    lv_obj_set_style_bg_color(ui_OIL_PRESS_Value, lv_color_hex(0xFF0000), 0);
                    lv_obj_set_style_bg_opa(ui_OIL_PRESS_Value, LV_OPA_COVER, 0);
                    lv_obj_set_style_pad_all(ui_OIL_PRESS_Value, 4, 0);
                } else {
                    lv_obj_set_style_text_color(ui_OIL_PRESS_Value, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_style_bg_opa(ui_OIL_PRESS_Value, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_pad_all(ui_OIL_PRESS_Value, 0, 0);
                }
                was_value_critical = value_critical;
            }
        }
        
        // Critical label with blinking
        if (ui_OIL_PRESS_VALUE_CRITICAL_Label) {
            static bool was_critical = false;
            static bool critical_visible = false;
            static uint32_t critical_exit_time = 0;
            const uint32_t CRITICAL_LINGER_MS = 2000;
            
            bool press_critical = isOilPressureCritical();
            uint32_t now = millis();
            
            if (press_critical) {
                critical_exit_time = 0;
                
                if (!critical_visible) {
                    critical_visible = true;
                    lv_obj_set_style_text_opa(ui_OIL_PRESS_VALUE_CRITICAL_Label, 255, LV_PART_MAIN);
                    lv_obj_set_style_bg_color(ui_OIL_PRESS_VALUE_CRITICAL_Label, lv_color_hex(0x000000), LV_PART_MAIN);
                    lv_obj_set_style_bg_opa(ui_OIL_PRESS_VALUE_CRITICAL_Label, 225, LV_PART_MAIN);
                    lv_obj_set_style_pad_all(ui_OIL_PRESS_VALUE_CRITICAL_Label, 4, LV_PART_MAIN);
                    
                    lv_anim_t anim;
                    lv_anim_init(&anim);
                    lv_anim_set_var(&anim, ui_OIL_PRESS_VALUE_CRITICAL_Label);
                    lv_anim_set_values(&anim, 0, 255);
                    lv_anim_set_duration(&anim, 200);
                    lv_anim_set_repeat_count(&anim, LV_ANIM_REPEAT_INFINITE);
                    lv_anim_set_playback_duration(&anim, 200);
                    lv_anim_set_exec_cb(&anim, [](void* obj, int32_t val) {
                        lv_obj_t* label = (lv_obj_t*)obj;
                        if (val < 128) {
                            lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
                        } else {
                            lv_obj_set_style_text_color(label, lv_color_hex(0x960000), LV_PART_MAIN);
                        }
                    });
                    lv_anim_start(&anim);
                }
            } else {
                if (was_critical && critical_exit_time == 0) {
                    critical_exit_time = now;
                }
                
                if (critical_visible && critical_exit_time > 0 && 
                    (now - critical_exit_time) >= CRITICAL_LINGER_MS) {
                    critical_visible = false;
                    lv_anim_delete(ui_OIL_PRESS_VALUE_CRITICAL_Label, NULL);
                    lv_obj_set_style_text_opa(ui_OIL_PRESS_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
                    lv_obj_set_style_bg_opa(ui_OIL_PRESS_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
                    critical_exit_time = 0;
                }
            }
            
            was_critical = press_critical;
        }
    }
    // If not valid, don't update - UI stays at reset state ("---" and 0)
    
    // ----- Oil Temperature -----
    if (g_vehicle_data.oil_temp_valid) {
        int temp_pan = g_vehicle_data.oil_temp_pan_f;
        int temp_cooled = g_vehicle_data.oil_temp_cooled_f;
        
        // Smooth animation
        smooth_oil_temp_f = smooth_oil_temp_f * (1.0f - SMOOTH_FACTOR) + temp_pan * SMOOTH_FACTOR;
        int display_temp = (int)(smooth_oil_temp_f + 0.5f);
        
        if (ui_OIL_TEMP_Bar) {
            lv_bar_set_value(ui_OIL_TEMP_Bar, display_temp, LV_ANIM_ON);
            bool critical = (temp_pan > OIL_TEMP_ValueCriticalF);
            lv_obj_set_style_bg_color(ui_OIL_TEMP_Bar, 
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange), 
                LV_PART_INDICATOR);
        }
        
        if (ui_OIL_TEMP_Value_P) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [P]", temp_pan);
            lv_label_set_text(ui_OIL_TEMP_Value_P, buf);
        }
        
        if (ui_OIL_TEMP_Value_C) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [C]", temp_cooled);
            lv_label_set_text(ui_OIL_TEMP_Value_C, buf);
        }
    }
    // If not valid, don't update - UI stays at reset state
    
    // ----- Water Temperature -----
    if (g_vehicle_data.water_temp_valid) {
        if (ui_W_TEMP_Value_H) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [H]", g_vehicle_data.water_temp_hot_f);
            lv_label_set_text(ui_W_TEMP_Value_H, buf);
        }
        if (ui_W_TEMP_Value_C) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [C]", g_vehicle_data.water_temp_cooled_f);
            lv_label_set_text(ui_W_TEMP_Value_C, buf);
        }
        if (ui_W_TEMP_Bar) {
            lv_bar_set_value(ui_W_TEMP_Bar, g_vehicle_data.water_temp_hot_f, LV_ANIM_ON);
            bool critical = (g_vehicle_data.water_temp_hot_f > W_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_W_TEMP_Bar, 
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange), 
                LV_PART_INDICATOR);
        }
    }
    // If not valid, don't update - UI stays at reset state
    
    // ----- Trans Temperature -----
    if (g_vehicle_data.trans_temp_valid) {
        if (ui_TRAN_TEMP_Value_H) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [H]", g_vehicle_data.trans_temp_hot_f);
            lv_label_set_text(ui_TRAN_TEMP_Value_H, buf);
        }
        if (ui_TRAN_TEMP_Value_C) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [C]", g_vehicle_data.trans_temp_cooled_f);
            lv_label_set_text(ui_TRAN_TEMP_Value_C, buf);
        }
        if (ui_TRAN_TEMP_Bar) {
            lv_bar_set_value(ui_TRAN_TEMP_Bar, g_vehicle_data.trans_temp_hot_f, LV_ANIM_ON);
            bool critical = (g_vehicle_data.trans_temp_hot_f > TRAN_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_TRAN_TEMP_Bar, 
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange), 
                LV_PART_INDICATOR);
        }
    }
    // If not valid, don't update - UI stays at reset state
    
    // ----- Steering Temperature -----
    if (g_vehicle_data.steer_temp_valid) {
        if (ui_STEER_TEMP_Value_H) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [H]", g_vehicle_data.steer_temp_hot_f);
            lv_label_set_text(ui_STEER_TEMP_Value_H, buf);
        }
        if (ui_STEER_TEMP_Value_C) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [C]", g_vehicle_data.steer_temp_cooled_f);
            lv_label_set_text(ui_STEER_TEMP_Value_C, buf);
        }
        if (ui_STEER_TEMP_Bar) {
            lv_bar_set_value(ui_STEER_TEMP_Bar, g_vehicle_data.steer_temp_hot_f, LV_ANIM_ON);
            bool critical = (g_vehicle_data.steer_temp_hot_f > STEER_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_STEER_TEMP_Bar, 
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange), 
                LV_PART_INDICATOR);
        }
    }
    // If not valid, don't update - UI stays at reset state
    
    // ----- Diff Temperature -----
    if (g_vehicle_data.diff_temp_valid) {
        if (ui_DIFF_TEMP_Value_H) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [H]", g_vehicle_data.diff_temp_hot_f);
            lv_label_set_text(ui_DIFF_TEMP_Value_H, buf);
        }
        if (ui_DIFF_TEMP_Value_C) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F [C]", g_vehicle_data.diff_temp_cooled_f);
            lv_label_set_text(ui_DIFF_TEMP_Value_C, buf);
        }
        if (ui_DIFF_TEMP_Bar) {
            lv_bar_set_value(ui_DIFF_TEMP_Bar, g_vehicle_data.diff_temp_hot_f, LV_ANIM_ON);
            bool critical = (g_vehicle_data.diff_temp_hot_f > DIFF_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_DIFF_TEMP_Bar, 
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange), 
                LV_PART_INDICATOR);
        }
    }
    // If not valid, don't update - UI stays at reset state
    
    // ----- Fuel Trust -----
    if (g_vehicle_data.fuel_trust_valid) {
        int fuel = g_vehicle_data.fuel_trust_percent;
        
        smooth_fuel_trust = smooth_fuel_trust * (1.0f - SMOOTH_FACTOR) + fuel * SMOOTH_FACTOR;
        int display_fuel = (int)(smooth_fuel_trust + 0.5f);
        
        if (ui_FUEL_TRUST_Bar) {
            lv_bar_set_value(ui_FUEL_TRUST_Bar, display_fuel, LV_ANIM_ON);
            bool critical = (fuel < FUEL_TRUST_ValueCritical);
            lv_obj_set_style_bg_color(ui_FUEL_TRUST_Bar, 
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange), 
                LV_PART_INDICATOR);
        }
        
        if (ui_FUEL_TRUST_Value) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d %%", fuel);
            lv_label_set_text(ui_FUEL_TRUST_Value, buf);
        }
    }
    // If not valid, don't update - UI stays at reset state
}

//=================================================================
// CHART UPDATE FUNCTION
//=================================================================

void updateCharts() {
    #if ENABLE_CHARTS
    uint32_t now = millis();
    
    // Accumulate samples
    if (g_vehicle_data.oil_pressure_valid) {
        pressure_sum += g_vehicle_data.oil_pressure_psi;
        pressure_samples++;
    }
    if (g_vehicle_data.oil_temp_valid) {
        temp_sum += g_vehicle_data.oil_temp_pan_f;
        temp_samples++;
    }
    
    // Initialize bucket start times
    if (pressure_bucket_start == 0) pressure_bucket_start = now;
    if (temp_bucket_start == 0) temp_bucket_start = now;
    
    // Push to pressure chart every CHART_BUCKET_MS
    if ((now - pressure_bucket_start) >= CHART_BUCKET_MS && chart_series_oil_press) {
        int32_t avg = (pressure_samples > 0) ? (pressure_sum / pressure_samples) : 0;
        if (avg < 0) avg = 0;
        if (avg > 150) avg = 150;
        shift_history(oil_press_history, avg);
        lv_chart_set_next_value(ui_OIL_PRESS_CHART, chart_series_oil_press, avg);
        pressure_sum = 0;
        pressure_samples = 0;
        pressure_bucket_start = now;
    }
    
    // Push to temp chart every CHART_BUCKET_MS
    if ((now - temp_bucket_start) >= CHART_BUCKET_MS && chart_series_oil_temp) {
        int32_t avg = (temp_samples > 0) ? (temp_sum / temp_samples) : 0;
        if (avg < 0) avg = 0;
        if (avg > OIL_TEMP_Max_F) avg = OIL_TEMP_Max_F;
        shift_history(oil_temp_history, avg);
        lv_chart_set_next_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, avg);
        temp_sum = 0;
        temp_samples = 0;
        temp_bucket_start = now;
    }
    
    // Update blink phase for critical bars
    if (now - g_last_blink_toggle >= CHART_BLINK_INTERVAL_MS) {
        g_critical_blink_phase = !g_critical_blink_phase;
        g_last_blink_toggle = now;
        
        if (ui_OIL_PRESS_CHART) lv_obj_invalidate(ui_OIL_PRESS_CHART);
        if (ui_OIL_TEMP_CHART) lv_obj_invalidate(ui_OIL_TEMP_CHART);
    }
    #endif
}

//=================================================================
// SETUP
//=================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("   370zMonitor v4 - Data Provider Arch");
    Serial.println("========================================");
    
    // Check PSRAM
    if (psramFound()) {
        Serial.printf("✓ PSRAM: %u bytes total, %u free\n", ESP.getPsramSize(), ESP.getFreePsram());
    } else {
        Serial.println("✗ WARNING: No PSRAM detected!");
    }
    
    Serial.printf("Internal heap: %u free\n", ESP.getFreeHeap());
    Serial.println("========================================\n");
    
    // I2C init
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
    Wire.setTimeOut(50);
    delay(50);
    
    // IO expander
    Serial.println("[1/7] IO Expander...");
    g_ioexp_ok = initIOExtension();
    Serial.printf("      %s\n", g_ioexp_ok ? "OK" : "FAILED");
    
    // Touch controller
    Serial.println("[2/7] Touch controller...");
    delay(100);
    touch.begin();
    touch.setRotation(0);
    Serial.println("      GT911 initialized");
    
    // Display
    Serial.println("[3/7] Display init...");
    gfx->begin();
    gfx->fillScreen(0x0000);
    Serial.println("      OK");
    
    // Backlight
    Serial.println("[4/7] Backlight...");
    setBacklight(true);
    delay(100);
    Serial.println("      OK");
    
    // LVGL
    Serial.println("[5/7] LVGL init...");
    lv_init();
    
    size_t buf_bytes = LVGL_BUFFER_SIZE * sizeof(lv_color_t);
    Serial.printf("      Buffer size: %u bytes\n", buf_bytes);
    
    disp_draw_buf1 = (uint8_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!disp_draw_buf1) {
        disp_draw_buf1 = (uint8_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    }
    if (!disp_draw_buf1) {
        Serial.println("      FATAL: Buffer 1 alloc failed!");
        while(1) delay(100);
    }
    
    disp_draw_buf2 = (uint8_t*)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!disp_draw_buf2) {
        disp_draw_buf2 = (uint8_t*)heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    }
    if (!disp_draw_buf2) {
        Serial.println("      FATAL: Buffer 2 alloc failed!");
        while (1) delay(100);
    }
    
    bool in_psram1 = esp_ptr_external_ram(disp_draw_buf1);
    bool in_psram2 = esp_ptr_external_ram(disp_draw_buf2);
    Serial.printf("      Buffer 1 in %s\n", in_psram1 ? "PSRAM" : "internal");
    Serial.printf("      Buffer 2 in %s\n", in_psram2 ? "PSRAM" : "internal");
    
    disp = lv_display_create(800, 480);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, disp_draw_buf1, disp_draw_buf2, buf_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touch_read);
    
    Serial.printf("      PSRAM free: %u\n", ESP.getFreePsram());
    Serial.println("      OK");
    
    // Load UI
    Serial.println("[6/7] Loading UI...");
    ui_init();
    
    if (!ui_Screen1) {
        Serial.println("      FATAL: ui_Screen1 is NULL!");
        while(1) delay(100);
    }
    Serial.printf("      PSRAM free: %u\n", ESP.getFreePsram());
    Serial.println("      OK");
    
    // Initialize data providers
    Serial.println("[7/7] Data providers...");
    initSensors();
    initOBD();
    Serial.println("      OK");
    
    // Bar animation speeds
    if (ui_OIL_PRESS_Bar) lv_obj_set_style_anim_duration(ui_OIL_PRESS_Bar, 100, LV_PART_MAIN);
    if (ui_OIL_TEMP_Bar) lv_obj_set_style_anim_duration(ui_OIL_TEMP_Bar, 100, LV_PART_MAIN);
    if (ui_FUEL_TRUST_Bar) lv_obj_set_style_anim_duration(ui_FUEL_TRUST_Bar, 100, LV_PART_MAIN);
    if (ui_W_TEMP_Bar) lv_obj_set_style_anim_duration(ui_W_TEMP_Bar, 100, LV_PART_MAIN);
    if (ui_TRAN_TEMP_Bar) lv_obj_set_style_anim_duration(ui_TRAN_TEMP_Bar, 100, LV_PART_MAIN);
    if (ui_STEER_TEMP_Bar) lv_obj_set_style_anim_duration(ui_STEER_TEMP_Bar, 100, LV_PART_MAIN);
    if (ui_DIFF_TEMP_Bar) lv_obj_set_style_anim_duration(ui_DIFF_TEMP_Bar, 100, LV_PART_MAIN);
    
    // Create utility box
    if (ui_Screen1) {
        utility_box = lv_obj_create(ui_Screen1);
        lv_obj_set_size(utility_box, 105, 85);  // Taller for mode indicator
        lv_obj_align(utility_box, LV_ALIGN_TOP_LEFT, 5, 5);
        lv_obj_set_style_bg_color(utility_box, lv_color_hex(0x444444), 0);
        lv_obj_set_style_bg_opa(utility_box, LV_OPA_70, 0);
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
        
        // FPS/CPU/BRI label
        utility_label = lv_label_create(utility_box);
        lv_label_set_text(utility_label, "--- FPS\n---% CPU\n---% BRI");
        lv_obj_set_style_text_color(utility_label, lv_color_hex(0xffff00), 0);
        lv_obj_set_style_text_font(utility_label, &lv_font_montserrat_14, 0);
        lv_obj_align(utility_label, LV_ALIGN_TOP_LEFT, 0, 0);
        
        // Mode indicator (DEMO/LIVE)
        mode_indicator = lv_label_create(utility_box);
        lv_obj_set_style_text_font(mode_indicator, &lv_font_montserrat_14, 0);
        lv_obj_align(mode_indicator, LV_ALIGN_BOTTOM_LEFT, 0, 0);
        updateModeIndicator();
        
        Serial.println("[UI] Utility box created");
        Serial.println("     - Single tap: toggle brightness");
        Serial.println("     - Double tap: hide/show");
        Serial.println("     - 5-sec hold: toggle DEMO/LIVE mode");
    }
    
    // Set initial brightness
    setBrightness(255);
    
    // Initialize charts
    #if ENABLE_CHARTS
    if (ui_OIL_PRESS_CHART) {
        lv_chart_series_t * ser;
        while ((ser = lv_chart_get_series_next(ui_OIL_PRESS_CHART, NULL)) != NULL) {
            lv_chart_remove_series(ui_OIL_PRESS_CHART, ser);
        }
        
        lv_chart_set_type(ui_OIL_PRESS_CHART, LV_CHART_TYPE_BAR);
        lv_chart_set_point_count(ui_OIL_PRESS_CHART, CHART_POINTS);
        lv_chart_set_range(ui_OIL_PRESS_CHART, LV_CHART_AXIS_PRIMARY_Y, 0, 150);
        lv_chart_set_update_mode(ui_OIL_PRESS_CHART, LV_CHART_UPDATE_MODE_SHIFT);
        lv_chart_set_div_line_count(ui_OIL_PRESS_CHART, 0, 0);
        
        chart_series_oil_press = lv_chart_add_series(ui_OIL_PRESS_CHART, lv_color_hex(hexOrange), LV_CHART_AXIS_PRIMARY_Y);
        
        lv_obj_add_event_cb(ui_OIL_PRESS_CHART, oil_press_chart_draw_cb, LV_EVENT_DRAW_TASK_ADDED, NULL);
        lv_obj_add_flag(ui_OIL_PRESS_CHART, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS);
        
        for (int i = 0; i < CHART_POINTS; i++) {
            lv_chart_set_next_value(ui_OIL_PRESS_CHART, chart_series_oil_press, -100);
            oil_press_history[i] = 0;
        }
        lv_chart_refresh(ui_OIL_PRESS_CHART);
        Serial.println("Oil pressure chart initialized");
    }
    
    if (ui_OIL_TEMP_CHART) {
        lv_chart_series_t * ser;
        while ((ser = lv_chart_get_series_next(ui_OIL_TEMP_CHART, NULL)) != NULL) {
            lv_chart_remove_series(ui_OIL_TEMP_CHART, ser);
        }
        
        lv_chart_set_type(ui_OIL_TEMP_CHART, LV_CHART_TYPE_BAR);
        lv_chart_set_point_count(ui_OIL_TEMP_CHART, CHART_POINTS);
        lv_chart_set_range(ui_OIL_TEMP_CHART, LV_CHART_AXIS_PRIMARY_Y, OIL_TEMP_Min_F, OIL_TEMP_Max_F);
        lv_chart_set_update_mode(ui_OIL_TEMP_CHART, LV_CHART_UPDATE_MODE_SHIFT);
        lv_chart_set_div_line_count(ui_OIL_TEMP_CHART, 0, 0);
        
        chart_series_oil_temp = lv_chart_add_series(ui_OIL_TEMP_CHART, lv_color_hex(hexOrange), LV_CHART_AXIS_PRIMARY_Y);
        
        lv_obj_add_event_cb(ui_OIL_TEMP_CHART, oil_temp_chart_draw_cb, LV_EVENT_DRAW_TASK_ADDED, NULL);
        lv_obj_add_flag(ui_OIL_TEMP_CHART, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS);
        
        for (int i = 0; i < CHART_POINTS; i++) {
            lv_chart_set_next_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, -100);
            oil_temp_history[i] = 0;
        }
        lv_chart_refresh(ui_OIL_TEMP_CHART);
        Serial.println("Oil temp chart initialized");
    }
    #endif
    
    // Force first render
    Serial.println("\nForcing initial render...");
    uint32_t t0 = millis();
    lv_refr_now(NULL);
    Serial.printf("Initial render took: %u ms\n", millis() - t0);
    Serial.printf("PSRAM free: %u\n", ESP.getFreePsram());
    
    Serial.println("\n========================================");
    Serial.println("         RUNNING");
    Serial.printf("  Mode: %s\n", g_demo_mode ? "DEMO" : "LIVE");
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
        
        last_update = now;
    }
    #endif
    
    // LVGL handler
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
