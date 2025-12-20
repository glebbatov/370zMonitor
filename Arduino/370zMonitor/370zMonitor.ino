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

//-----------------------------------------------------------------

// ===== CRITICAL: Configure ESP32 to prefer PSRAM for malloc =====
// This runs before setup() to configure the heap
__attribute__((constructor)) void configurePSRAM() {
    // This hint tells ESP32 to prefer PSRAM for allocations > 32KB
    heap_caps_malloc_extmem_enable(32 * 1024);
}

//-----------------------------------------------------------------

// ===== FEATURE FLAGS =====
#define ENABLE_TOUCH        0   // Disabled for now
#define ENABLE_UI_UPDATES   1   // Enable bar/label updates
#define ENABLE_CHARTS       1   // Re-enable charts
#define UPDATE_INTERVAL_MS  125 // Update every (150ms original speed)

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

//-----------------------------------------------------------------

// OBD Data

int RPM = 0;

//-----------------------------------------------------------------

// Sensors Data

int oilPressurePSI = 0;

//-----------------------------------------------------------------

// UI objects
#include "ui.h"

//OIL PRESS
//	-Spread:
//		0-150 PSI
//	-VALUE CRITICAL:
//		<= 10 PSI per 1000 RPM minimum
//		>= 120 PSI
extern lv_obj_t * ui_OIL_PRESS_Bar;
extern lv_obj_t * ui_OIL_PRESS_CHART;
extern lv_obj_t * ui_OIL_PRESS_Value;
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
extern lv_obj_t * ui_OIL_TEMP_Bar;
extern lv_obj_t * ui_OIL_TEMP_CHART;
extern lv_obj_t * ui_OIL_TEMP_Value_P;
extern lv_obj_t * ui_OIL_TEMP_Value_C;
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
extern lv_obj_t * ui_W_TEMP_Bar;
extern lv_obj_t * ui_W_TEMP_CHART;
extern lv_obj_t* ui_W_TEMP_Value_H;
extern lv_obj_t* ui_W_TEMP_Value_C;
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
extern lv_obj_t * ui_TRAN_TEMP_Bar;
extern lv_obj_t * ui_TRAN_TEMP_CHART;
extern lv_obj_t* ui_TRAN_TEMP_Value_H;
extern lv_obj_t* ui_TRAN_TEMP_Value_C;
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
extern lv_obj_t * ui_STEER_TEMP_Bar;
extern lv_obj_t * ui_STEER_TEMP_CHART;
extern lv_obj_t * ui_STEER_TEMP_Value_H;
extern lv_obj_t * ui_STEER_TEMP_Value_C;
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
extern lv_obj_t * ui_DIFF_TEMP_Bar;
extern lv_obj_t * ui_DIFF_TEMP_CHART;
extern lv_obj_t * ui_DIFF_TEMP_Value_H;
extern lv_obj_t * ui_DIFF_TEMP_Value_C;
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
extern lv_obj_t * ui_FUEL_TRUST_Bar;
extern lv_obj_t * ui_FUEL_TRUST_CHART;
extern lv_obj_t * ui_FUEL_TRUST_Value;
int FUEL_TRUST_Min_F = 0;
int FUEL_TRUST_Max_F = 100;
int FUEL_TRUST_ValueCritical_F = 50;

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
#define LVGL_BUFFER_SIZE (800 * 48)  // 48 lines - will go to PSRAM
static lv_display_t *disp;
static lv_indev_t *indev;
static uint8_t *disp_draw_buf1;
static uint8_t *disp_draw_buf2;

//-----------------------------------------------------------------

// Counters
static uint32_t loop_count = 0;
static uint32_t flush_count = 0;
static uint32_t update_count = 0;
static lv_obj_t * fps_label = NULL;

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

// ===== Chart draw callbacks for bar coloring =====
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
    fill_dsc->color = is_red ? lv_color_hex(0xFF0000) : lv_color_hex(0xFF4619);
}

// Oil temp: red < 180°F or > 260°F, orange in between
static void oil_temp_chart_draw_cb(lv_event_t * e) {
    lv_draw_task_t * draw_task = lv_event_get_draw_task(e);

    lv_draw_fill_dsc_t * fill_dsc = lv_draw_task_get_fill_dsc(draw_task);
    if(fill_dsc == NULL) return;

    lv_draw_dsc_base_t * base_dsc = (lv_draw_dsc_base_t *)fill_dsc;
    if(base_dsc->part != LV_PART_ITEMS) return;

    uint32_t idx = base_dsc->id2;  // Point index
    if(idx >= CHART_POINTS) return;

    int32_t temp_c = oil_temp_history[idx];
    int32_t temp_f = (temp_c * 9) / 5 + 32;

    bool is_red = (temp_f < 180) || (temp_f > 260);
    fill_dsc->color = is_red ? lv_color_hex(0xFF0000) : lv_color_hex(0xFF4619);
}

// Helper to shift history array left and add new value
static void shift_history(int32_t* history, int32_t new_value) {
    for (int i = 0; i < CHART_POINTS - 1; i++) {
        history[i] = history[i + 1];
    }
    history[CHART_POINTS - 1] = new_value;
}

//-----------------------------------------------------------------

#pragma region Hardware functions

// ===== IO Expander Functions =====
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

//-----------------------------------------------------------------

// ===== LVGL Callbacks =====
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

void my_touch_read(lv_indev_t *indev, lv_indev_data_t *data) {
    data->state = LV_INDEV_STATE_RELEASED;
}

#pragma endregion

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
    
    // Input device (dummy)
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

    OIL_PRESS_ValueCriticalRPM = OilPressMin_FromRPM();

    //-----------------------------

    // Create FPS counter label
    if (ui_Screen1 != NULL) {
        fps_label = lv_label_create(ui_Screen1);
        lv_obj_align(fps_label, LV_ALIGN_TOP_RIGHT, -10, 10);
        lv_label_set_text(fps_label, "--- FPS");
        lv_obj_set_style_text_color(fps_label, lv_color_hex(0x00FF00), LV_PART_MAIN);
        lv_obj_set_style_text_font(fps_label, &lv_font_montserrat_20, LV_PART_MAIN);
    }
    
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
        chart_series_oil_press = lv_chart_add_series(ui_OIL_PRESS_CHART, lv_color_hex(0xFF4619), LV_CHART_AXIS_PRIMARY_Y);
        
        // Initialize with zeros
        for (int i = 0; i < 24; i++) {
            lv_chart_set_next_value(ui_OIL_PRESS_CHART, chart_series_oil_press, 0);
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
        lv_chart_set_range(ui_OIL_TEMP_CHART, LV_CHART_AXIS_PRIMARY_Y, 0, 140);
        lv_chart_set_update_mode(ui_OIL_TEMP_CHART, LV_CHART_UPDATE_MODE_SHIFT);
        lv_chart_set_div_line_count(ui_OIL_TEMP_CHART, 0, 0);
        
        // Add series
        chart_series_oil_temp = lv_chart_add_series(ui_OIL_TEMP_CHART, lv_color_hex(0xFF4619), LV_CHART_AXIS_PRIMARY_Y);
        
        // Initialize with zeros
        for (int i = 0; i < 24; i++) {
            lv_chart_set_next_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, 0);
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
    static uint32_t last_tick = 0;
    static uint32_t last_status = 0;
    static uint32_t last_update = 0;

    static int oil_pressure = 75;
    static int oil_temp_c = 98;
    static int fuel_trust = 100;
    
    uint32_t now = millis();
    
    // LVGL tick
    if (last_tick == 0) last_tick = now;
    lv_tick_inc(now - last_tick);
    last_tick = now;
    
    loop_count++;
    
    // Status every 1 seconds
    if (now - last_status >= 1000) {
        // Update FPS label
        if (fps_label != NULL) {
            char fps_buf[16];
            snprintf(fps_buf, sizeof(fps_buf), "%3d FPS", flush_count);
            lv_label_set_text(fps_label, fps_buf);
        }
        
        Serial.printf("[STATUS] loops=%u flushes=%u heap=%u psram=%u\n",
                      loop_count, flush_count,
                      ESP.getFreeHeap(), ESP.getFreePsram());
        flush_count = 0;  // Reset for next second
        last_status = now;
    }
    
    // UI updates
    #if ENABLE_UI_UPDATES
    if (now - last_update >= UPDATE_INTERVAL_MS) {
        update_count++;
        
        //-----------------------

        // Animate oil pressure (0-150 PSI range)
        oil_pressure += random(-7, 8);
        if (oil_pressure > 150) oil_pressure = 150;
        if (oil_pressure < 0) oil_pressure = 0;
        
        // Update pressure bar and label
        if (ui_OIL_PRESS_Bar) {
            lv_bar_set_value(ui_OIL_PRESS_Bar, oil_pressure, LV_ANIM_OFF);
            // Color: red if <20 or >100, else orange
            bool press_red = (oil_pressure < 20) || (oil_pressure > 100);
            lv_obj_set_style_bg_color(ui_OIL_PRESS_Bar,
                press_red ? lv_color_hex(0xFF0000) : lv_color_hex(0xFF4619),
                LV_PART_INDICATOR);
        }
        if (ui_OIL_PRESS_Value) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d PSI", oil_pressure);
            lv_label_set_text(ui_OIL_PRESS_Value, buf);
        }

        //-----------------------
        
        // Animate temp bar
        int temp_f = (oil_temp_c * 9) / 5 + 32;
        if (ui_OIL_TEMP_Bar) {
            // Bar range is 150-300°F
            lv_bar_set_value(ui_OIL_TEMP_Bar, temp_f, LV_ANIM_OFF);
            bool temp_red = (temp_f < 180) || (temp_f > 260);
            lv_obj_set_style_bg_color(ui_OIL_TEMP_Bar,
                temp_red ? lv_color_hex(0xFF0000) : lv_color_hex(0xFF4619),
                LV_PART_INDICATOR);
        }
        
        //-----------------------

        // Animate temp labels
        if (ui_OIL_TEMP_Value_C) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F", temp_f);
            lv_label_set_text(ui_OIL_TEMP_Value_C, buf);
        }
        if (ui_OIL_TEMP_Value_P) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d°F", oil_temp_c);
            lv_label_set_text(ui_OIL_TEMP_Value_P, buf);
        }

        //-----------------------

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

        // Update fuel trust  bar and label
        if (ui_FUEL_TRUST_Bar) {
            lv_bar_set_value(ui_FUEL_TRUST_Bar, fuel_trust, LV_ANIM_OFF);
            // Color: red if <50, else orange
            bool press_red = (fuel_trust < 50);
            lv_obj_set_style_bg_color(ui_FUEL_TRUST_Bar,
                press_red ? lv_color_hex(0xFF0000) : lv_color_hex(0xFF4619),
                LV_PART_INDICATOR);
        }
        if (ui_FUEL_TRUST_Value) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d %%", fuel_trust);
            lv_label_set_text(ui_FUEL_TRUST_Value, buf);
        }

        //-----------------------
        
        // Accumulate samples for chart averaging
        #if ENABLE_CHARTS
        pressure_sum += oil_pressure;
        pressure_samples++;
        temp_sum += oil_temp_c;
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
    
    delay(10);
}
