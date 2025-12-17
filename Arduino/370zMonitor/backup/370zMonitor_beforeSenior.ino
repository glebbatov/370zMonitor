/*
 * Waveshare ESP32-S3 7" (800x480)
 * SquareLine Studio UI - GaugeAndChartDesign
 * Board: ESP32-S3-Touch-LCD-7 (NOT 7B!)
 */

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <TAMC_GT911.h>
#include <lvgl.h>
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>
#include <math.h>
#include "ui.h"

// External UI objects
extern lv_obj_t * ui_OIL_PRESS_Bar;
extern lv_obj_t * ui_OIL_PRESS_Value;
extern lv_obj_t * ui_OIL_PRESS_CHART;
extern lv_obj_t * ui_OIL_TEMP_Bar;
extern lv_obj_t * ui_OIL_TEMP_Value;
extern lv_obj_t * ui_OIL_TEMP_CHART;
extern lv_obj_t * ui_Screen1;

// FPS counter label and tracking
static lv_obj_t * fps_label = NULL;
static int flush_count = 0;  // Count actual screen refreshes, not loop iterations

// ===== Stability knobs =====
// I2C bus on these boards can get flaky at 400kHz; 100kHz is slower but much harder to break.
#define I2C_FREQ_HZ 100000
// LVGL bar animations can stack up when you update them every 200ms. We do manual smoothing instead.
#define BAR_ANIM LV_ANIM_OFF

static bool g_ioexp_ok = false; // CH422/CH422G init status; if false, we stop talking to it to avoid I2C lockups.

// CH422 uses an I2C-like two-wire interface, but the *I2C address byte itself* is the CH422 "command byte1".
// Examples from datasheet:
//  - System command byte1 = 0x48  => I2C address 0x24 (0x24<<1 = 0x48)
//  - IO write command byte1 = 0x70 => I2C address 0x38 (0x38<<1 = 0x70)
#define CH422_ADDR_SYSTEM 0x24
#define CH422_ADDR_IOWR   0x38

// I2C pins
#define I2C_SDA 8
#define I2C_SCL 9

// Waveshare mapping (from the board wiki):
//  - EXIO1: TP_RST (touch reset)
//  - EXIO2: DISP (backlight enable)
//  - EXIO4: SD_CS (active low)
//  - EXIO5: USB_SEL / CAN_SEL (0=USB, 1=CAN)
// NOTE: LCD reset is NOT listed as EXIO on this board wiki; do NOT toggle a made-up "LCD_RST" bit.
#define EXIO_TP_RST   1
#define EXIO_DISP     2
#define EXIO_SD_CS    4
#define EXIO_USB_SEL  5

// Touch interrupt pin
#define TOUCH_INT_PIN 4

static uint8_t g_exio_state = 0; // IO7..IO0 output latch (EXIO7..EXIO0)

static bool ch422_write_system(uint8_t sys_param)
{
    Wire.beginTransmission(CH422_ADDR_SYSTEM);
    Wire.write(sys_param);
    uint8_t err = Wire.endTransmission(true);
    if (err != 0)
    {
        Serial.printf("CH422 system init failed: %u\n", (unsigned)err);
        return false;
    }
    return true;
}

static bool ch422_write_io(uint8_t io_state)
{
    Wire.beginTransmission(CH422_ADDR_IOWR);
    Wire.write(io_state);
    uint8_t err = Wire.endTransmission(true);
    if (err != 0)
    {
        // Return false; caller can decide whether to give up.
        return false;
    }
    return true;
}

static void exio_set(uint8_t exio_bit, bool level)
{
    if (exio_bit > 7) return;
    if (level) g_exio_state |=  (1u << exio_bit);
    else       g_exio_state &= ~(1u << exio_bit);
    (void)ch422_write_io(g_exio_state);
}

// Initialize CH422 IO expander (sets up EXIO lines we care about)
bool initIOExtension() {
    delay(10);

    // Datasheet example: 0x11 -> IO outputs enabled + OC open-drain.
    if (!ch422_write_system(0x11))
        return false;

    delay(10);

    // Default-safe state:
    // - Backlight OFF (EXIO2=0)
    // - Touch not held in reset (EXIO1=1)
    // - SD chip select inactive (EXIO4=1)
    // - USB mode selected (EXIO5=0)
    g_exio_state = 0;
    g_exio_state |= (1u << EXIO_TP_RST);
    g_exio_state |= (1u << EXIO_SD_CS);
    // EXIO_DISP left low for now
    // EXIO_USB_SEL left low (USB mode)

    if (!ch422_write_io(g_exio_state))
    {
        Serial.println("CH422 IO write failed");
        return false;
    }

    delay(20);
    return true;
}

// Set backlight state (ON/OFF only, no PWM on this board)
void setBacklight(bool on) {
    if(!g_ioexp_ok) return;
    exio_set(EXIO_DISP, on);
}

// Initialize touch controller reset sequence
void initTouch() {
    if(!g_ioexp_ok) {
        Serial.println("CH422G not OK -> skipping touch reset sequence (avoids I2C lockups)");
        return;
    }

    // Standard GT911 reset dance: hold reset low, hold INT low, release reset, then release INT.
    pinMode(TOUCH_INT_PIN, OUTPUT);
    digitalWrite(TOUCH_INT_PIN, LOW);

    exio_set(EXIO_TP_RST, false); // reset LOW
    delay(20);
    exio_set(EXIO_TP_RST, true);  // reset HIGH
    delay(80);

    pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
}

// RGB Panel - Waveshare ESP32-S3 7" (800x480)
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    // Pin mapping (800x480 specific)
    5,   // DE (Data Enable)
    3,   // VSYNC
    46,  // HSYNC
    7,   // PCLK
    1,   // R3
    2,   // R4
    42,  // R5
    41,  // R6
    40,  // R7
    
    39,  // G2
    0,   // G3
    45,  // G4
    48,  // G5
    47,  // G6
    21,  // G7
    
    14,  // B3
    38,  // B4
    18,  // B5
    17,  // B6
    10,  // B7
    
    // Timing parameters for 800x480
    0,   // hsync_polarity (0=active low)
    40,  // hsync_front_porch
    48,  // hsync_pulse_width
    40,  // hsync_back_porch
    0,   // vsync_polarity (0=active low)
    13,  // vsync_front_porch
    3,   // vsync_pulse_width
    32,  // vsync_back_porch
    1,   // pclk_active_neg (data on falling edge)
    14000000,  // 14MHz pixel clock
    true,      // auto_flush
    0,   // de_idle_high
    0    // pclk_idle_high
);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, rgbpanel, 0, true);

// GT911 Touch Controller
TAMC_GT911 touch = TAMC_GT911(I2C_SDA, I2C_SCL, TOUCH_INT_PIN, -1, 800, 480);

// Track if touch controller initialized successfully
static bool g_touch_init_ok = false;

// ===== Touch caching (keeps LVGL input callback I2C-free) =====
static volatile bool g_touch_down = false;
static volatile uint16_t g_touch_x = 0;
static volatile uint16_t g_touch_y = 0;

// ===== I2C bus recovery (fixes "SDA stuck low" hard-lockups) =====
static void i2c_bus_clear()
{
    // Release lines
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    delayMicroseconds(5);

    // If SDA is stuck low, pulse SCL up to 9 times to force any slave to release.
    if (digitalRead(I2C_SDA) == LOW) {
        pinMode(I2C_SCL, OUTPUT_OPEN_DRAIN);
        digitalWrite(I2C_SCL, HIGH);
        for (int i = 0; i < 9; i++) {
            digitalWrite(I2C_SCL, LOW);
            delayMicroseconds(5);
            digitalWrite(I2C_SCL, HIGH);
            delayMicroseconds(5);
            if (digitalRead(I2C_SDA) == HIGH) break;
        }
    }

    // Send a STOP condition manually: SDA low -> SCL high -> SDA high
    pinMode(I2C_SDA, OUTPUT_OPEN_DRAIN);
    pinMode(I2C_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(I2C_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C_SDA, HIGH);
    delayMicroseconds(5);

    // Return to pulled-up inputs
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
}

// Wire timeout helper: keep I2C transactions from blocking forever on a wedged bus.
// ESP32 Arduino core uses setTimeOut(); some other cores use setTimeout().
static inline void wire_set_timeout_ms(uint32_t ms)
{
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Wire.setTimeOut((uint16_t)ms);
#else
    // Many non-ESP32 cores implement setTimeout().
    Wire.setTimeout(ms);
#endif
}


// Touch poll with comprehensive guardrails:
// - never call into I2C from the LVGL callback
// - only poll when INT is asserted
// - check I2C bus health before every transaction
// - recover immediately if bus is wedged
// - limit how long we can spend in I2C operations
static void touch_poll_cached()
{
    // GT911 INT is typically active-low when there's new touch data.
    // If it's high (idle), skip the I2C transaction entirely.
    if (digitalRead(TOUCH_INT_PIN) == HIGH) {
        g_touch_down = false;
        return;
    }

    // CRITICAL: If SDA is stuck low, DO NOT attempt I2C - recover first
    if (digitalRead(I2C_SDA) == LOW) {
        Serial.println("⚠️  I2C: SDA stuck LOW detected in touch_poll - recovering bus");
        i2c_bus_clear();
        Wire.end();
        delay(2);
        Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
        delay(2);
        wire_set_timeout_ms(20);
        g_touch_down = false; // Skip this touch read cycle
        return;
    }

    // Ensure Wire timeout is set to prevent infinite blocking
    wire_set_timeout_ms(20);

    // Try to read touch data - if this hangs despite timeout, we have bigger problems
    touch.read();

    if (touch.isTouched) {
        g_touch_down = true;
        g_touch_x = (uint16_t)touch.points[0].x;
        g_touch_y = (uint16_t)touch.points[0].y;
    } else {
        g_touch_down = false;
    }
    
    // Paranoid check: verify I2C bus is still healthy after the transaction
    if (digitalRead(I2C_SDA) == LOW) {
        Serial.println("⚠️  I2C: SDA went LOW after touch read - bus may be wedged");
    }
}

static inline void i2c_ensure_alive()
{
    if (digitalRead(I2C_SDA) == LOW) {
        Serial.println("I2C: SDA stuck LOW -> bus clear + Wire restart");
        i2c_bus_clear();
        Wire.end();
        delay(2);
        Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
        delay(2);
    }
}

// Wire timeout helper that compiles no matter which core version you're on.
// Prefers setTimeOut(ms), then setTimeout(ms), else does nothing.

// LVGL Configuration
#define LVGL_BUFFER_SIZE (800 * 30)
static lv_display_t *disp;
static lv_indev_t *indev;
static uint8_t *disp_draw_buf;

// Oil pressure chart history (24 bars, 5-second buckets; newest on the right)
static const uint32_t OIL_PRESS_BARS = 24;
static const uint32_t OIL_PRESS_BUCKET_MS = 5000;

// Seed history (oldest -> newest)
static int32_t oil_press_hist[OIL_PRESS_BARS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static lv_chart_series_t * chart_series_oil_press = NULL;  // Single series (Primary Y)

// 5-second averaging accumulator
static int32_t pressure_sum = 0;
static uint32_t pressure_samples = 0;
static uint32_t bucket_start_ms = 0;

// Oil temp chart history (24 bars, 5-second buckets; newest on the right)
static const uint32_t OIL_TEMP_BARS = 24;
static const uint32_t OIL_TEMP_BUCKET_MS = 5000;

// Seed history with temps around 85-95°C (oldest -> newest)
static int32_t oil_temp_hist[OIL_TEMP_BARS] = {85, 88, 90, 92, 95, 93, 91, 89, 87, 90, 92, 94, 96, 98, 95, 93, 88, 86, 89, 91, 94, 96, 92, 90};
static lv_chart_series_t * chart_series_oil_temp = NULL;

// 5-second averaging accumulator for temp
static int32_t temp_sum = 0;
static uint32_t temp_samples = 0;
static uint32_t temp_bucket_start_ms = 0;

// Recolor oil pressure chart bars based on their value (orange <=100, red >100)
static void oil_press_chart_draw_event_cb(lv_event_t * e)
{
    // LVGL v9: event target and draw descriptors are typeless (void*) in C++; cast them.
    lv_draw_task_t * draw_task = (lv_draw_task_t *)lv_event_get_draw_task(e);
    lv_draw_dsc_base_t * base_dsc = (lv_draw_dsc_base_t *)lv_draw_task_get_draw_dsc(draw_task);
    if(base_dsc == NULL || base_dsc->part != LV_PART_ITEMS) return;

    lv_draw_fill_dsc_t * fill_dsc = lv_draw_task_get_fill_dsc(draw_task);
    if(fill_dsc == NULL) return;

    // base_dsc->id2 is the point index for chart items (0..point_count-1).
    // Use our own history buffer so colors never "drift" during shift/animation.
    uint32_t idx = base_dsc->id2;
    if(idx >= OIL_PRESS_BARS) idx = base_dsc->id1;   // fallback (future-proofing)
    if(idx >= OIL_PRESS_BARS) return;

    int32_t v = oil_press_hist[idx];

    // Red < 20 psi, orange 20..100, red > 100
    const bool is_red = (v < 20) || (v > 100);
    fill_dsc->color = is_red ? lv_color_hex(0xFF0000) : lv_color_hex(0xFF4619);
}

// Recolor oil temp chart bars based on their value in °F (red <180°F or >260°F, else orange)
static void oil_temp_chart_draw_event_cb(lv_event_t * e)
{
    lv_draw_task_t * draw_task = (lv_draw_task_t *)lv_event_get_draw_task(e);
    lv_draw_dsc_base_t * base_dsc = (lv_draw_dsc_base_t *)lv_draw_task_get_draw_dsc(draw_task);
    if(base_dsc == NULL || base_dsc->part != LV_PART_ITEMS) return;

    lv_draw_fill_dsc_t * fill_dsc = lv_draw_task_get_fill_dsc(draw_task);
    if(fill_dsc == NULL) return;

    uint32_t idx = base_dsc->id2;
    if(idx >= OIL_TEMP_BARS) idx = base_dsc->id1;   // fallback (future-proofing)
    if(idx >= OIL_TEMP_BARS) return;

    int32_t c = oil_temp_hist[idx];
    int32_t f = (c * 9) / 5 + 32;

    const bool is_red = (f < 180) || (f > 260);
    fill_dsc->color = is_red ? lv_color_hex(0xFF0000) : lv_color_hex(0xFF4619);
}

// Push a new 5-second average into the rolling 24-bar history (newest on the right)
static void oil_press_chart_push(int32_t avg_psi)
{
    // Clamp to the chart range
    if(avg_psi < 0) avg_psi = 0;
    if(avg_psi > 150) avg_psi = 150;

    // Shift history left (oldest falls off), append newest at the end (rightmost bar)
    for(uint32_t i = 0; i < OIL_PRESS_BARS - 1; i++) {
        oil_press_hist[i] = oil_press_hist[i + 1];
    }
    oil_press_hist[OIL_PRESS_BARS - 1] = avg_psi;

    // Let LVGL do the same shift for the visuals (keeps the existing chart animation behavior)
    if(ui_OIL_PRESS_CHART != NULL && chart_series_oil_press != NULL) {
        lv_chart_set_next_value(ui_OIL_PRESS_CHART, chart_series_oil_press, avg_psi);
    }
}

static void oil_temp_chart_push(int32_t avg_c)
{
    // Clamp to the UI/chart range (0-140°C)
    if(avg_c < 0) avg_c = 0;
    if(avg_c > 140) avg_c = 140;

    // Shift history left (oldest falls off), append newest at the end (rightmost bar)
    for(uint32_t i = 0; i < OIL_TEMP_BARS - 1; i++) {
        oil_temp_hist[i] = oil_temp_hist[i + 1];
    }
    oil_temp_hist[OIL_TEMP_BARS - 1] = avg_c;

    // Keep existing rolling animation behavior
    if(ui_OIL_TEMP_CHART != NULL && chart_series_oil_temp != NULL) {
        lv_chart_set_next_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, avg_c);
    }
}



// LVGL display flush callback - WITH byte swap (panel needs it)
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    
    // Swap bytes for RGB565 display  
    uint16_t *buf = (uint16_t *)px_map;
    uint32_t len = w * h;
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = (buf[i] >> 8) | (buf[i] << 8);
    }
    
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
    lv_display_flush_ready(disp);
    
    // Count actual screen refreshes for accurate FPS
    flush_count++;
}

// LVGL touch input callback
void my_touch_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    // IMPORTANT: keep this callback I2C-free. It runs inside LVGL's input processing.
    if (g_touch_down) {
        data->point.x = g_touch_x;
        data->point.y = g_touch_y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== Waveshare ESP32-S3 7\" 800x480 ===");
    Serial.println("SquareLine Studio UI: GaugeAndChartDesign");
    
    // I2C is a common freeze source on these boards when a slave wedges SDA low.
    // Clear the bus first, then only probe the expected addresses (avoid full-range scanners).
    Serial.println("\n=== I2C Probe ===");
    i2c_bus_clear();
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
    delay(50);
    i2c_ensure_alive();

    auto probe = [](uint8_t addr) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        Serial.printf("I2C probe 0x%02X -> %s (err=%u)\n", addr, (err == 0) ? "ACK" : "NO", (unsigned)err);
    };

    probe(CH422_ADDR_SYSTEM);
    probe(CH422_ADDR_IOWR);
    probe(0x5D); // GT911 commonly reports at 0x5D on these panels
    Serial.println("===================\n");
    
    // Give I2C bus time to stabilize after probing
    delay(100);
    wire_set_timeout_ms(100);  // Set timeout for subsequent operations
    
    // Initialize IO expander and backlight
    Serial.println("Initializing CH422G IO Expander...");
    g_ioexp_ok = initIOExtension();
    if (!g_ioexp_ok) {
        Serial.println("ERROR: CH422G initialization failed!");
        Serial.println("Continuing anyway to test display...");
    }
    
    // Initialize touch controller reset
    Serial.println("Initializing Touch Controller...");
    initTouch();
    pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
    
    // Initialize display
    Serial.println("Initializing RGB Display...");
    gfx->begin();
    gfx->fillScreen(0x0000);
    
    // Turn on backlight
    Serial.println("Turning on Backlight...");
    setBacklight(true);
    delay(50);
    
    // Initialize touch library with timeout protection and retries
    Serial.println("Initializing GT911 Touch...");
    
    // CRITICAL: Set Wire timeout BEFORE touch.begin() to prevent I2C hangs
    wire_set_timeout_ms(100);
    
    // Ensure bus is alive before attempting init
    i2c_ensure_alive();
    
    // Try to initialize touch with multiple retries and bus recovery
    bool touch_ok = false;
    for (int retry = 0; retry < 3 && !touch_ok; retry++) {
        if (retry > 0) {
            Serial.printf("GT911 init retry %d/3...\n", retry);
            // Full I2C bus recovery
            i2c_bus_clear();
            Wire.end();
            delay(100);
            Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
            delay(100);
            wire_set_timeout_ms(100);
            
            // Redo touch reset sequence
            if (g_ioexp_ok) {
                pinMode(TOUCH_INT_PIN, OUTPUT);
                digitalWrite(TOUCH_INT_PIN, LOW);
                exio_set(EXIO_TP_RST, false);
                delay(20);
                exio_set(EXIO_TP_RST, true);
                delay(80);
                pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
                delay(50);
            }
        }
        
        // touch.begin() can hang if GT911 is wedged - Wire timeout helps but isn't perfect
        Serial.println("Calling touch.begin()...");
        touch.begin();
        delay(100);
        
        // Verify GT911 is actually responding on I2C
        Wire.beginTransmission(0x5D); // GT911 typical address
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            touch_ok = true;
            Serial.println("✅ GT911 verified on I2C bus");
        } else {
            Serial.printf("⚠️  GT911 not responding (I2C err=%u)\n", (unsigned)err);
        }
    }
    
    if (touch_ok) {
        touch.setRotation(ROTATION_NORMAL);
        g_touch_init_ok = true;  // Mark touch as successfully initialized
        Serial.println("✅ GT911 Touch fully initialized");
    } else {
        g_touch_init_ok = false;  // Touch init failed
        Serial.println("❌ GT911 Touch initialization FAILED after retries");
        Serial.println("   Continuing anyway - display will work but touch may not");
    }
    
    // Initialize LVGL
    Serial.println("Initializing LVGL...");
    lv_init();
    
    // Allocate LVGL draw buffer
    // NOTE: Some Arduino board configs ship with PSRAM disabled even if the module has it.
    // If we allocate SPIRAM-only and PSRAM isn't available, you'll "freeze" here.
    const size_t buf_bytes = (size_t)LVGL_BUFFER_SIZE * sizeof(lv_color_t);

    Serial.printf("PSRAM: found=%s, size=%u bytes, free=%u bytes\n",
                  psramFound() ? "YES" : "NO",
                  (unsigned)ESP.getPsramSize(),
                  (unsigned)ESP.getFreePsram());

    // 1) Prefer INTERNAL DMA-capable RAM (most stable for display/touch stacks)
    disp_draw_buf = (uint8_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);

    // 2) Fallback to PSRAM (works on many setups, but some display paths are picky)
    if (!disp_draw_buf && psramFound()) {
        Serial.println("Internal DMA buffer alloc failed, falling back to PSRAM...");
        disp_draw_buf = (uint8_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }

    if (!disp_draw_buf) {
        Serial.printf("FATAL: Display buffer alloc failed (%u bytes).\n",  (unsigned)buf_bytes);
        Serial.println("Fix: Arduino IDE -> Tools -> PSRAM -> 'OPI PSRAM' (if your board has it), or reduce LVGL_BUFFER_SIZE.");
        while (1) delay(100);
    }
    
    // Create LVGL display
    disp = lv_display_create(800, 480);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, disp_draw_buf, NULL, LVGL_BUFFER_SIZE * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    // Create LVGL input device
    indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touch_read);
    
    // Load SquareLine Studio UI
    Serial.println("Loading UI...");
    ui_init();
    
    // Verify critical UI objects were created
    if (ui_Screen1 == NULL) {
        Serial.println("❌ FATAL: ui_Screen1 not created by ui_init()!");
        while(1) { delay(1000); Serial.println("UI init failed - halted"); }
    }
    if (ui_OIL_PRESS_Bar == NULL || ui_OIL_PRESS_Value == NULL || ui_OIL_PRESS_CHART == NULL) {
        Serial.println("⚠️  Warning: Some oil pressure UI objects missing");
    }
    if (ui_OIL_TEMP_Bar == NULL || ui_OIL_TEMP_Value == NULL || ui_OIL_TEMP_CHART == NULL) {
        Serial.println("⚠️  Warning: Some oil temp UI objects missing");
    }
    Serial.println("✅ UI objects validated");
    
    // Create FPS counter label in top right corner
    if (ui_Screen1 != NULL) {
        fps_label = lv_label_create(ui_Screen1);
        lv_obj_align(fps_label, LV_ALIGN_TOP_RIGHT, -10, 10);
        lv_label_set_text(fps_label, "--- FPS");  // Fixed width format
        lv_obj_set_style_text_color(fps_label, lv_color_hex(0x00FF00), LV_PART_MAIN);
        lv_obj_set_style_text_align(fps_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    }
    
    // ====== CHARTS COMPLETELY DISABLED FOR TESTING ======
    Serial.println("⚠️  ALL CHART OPERATIONS DISABLED FOR STABILITY TESTING");
    
    /*
    // Initialize oil pressure chart (single series + draw-task recolor)
if (ui_OIL_PRESS_CHART != NULL) {
    // Remove any SquareLine-created series (and any leftovers)
    lv_chart_series_t * ser = NULL;
    while ((ser = lv_chart_get_series_next(ui_OIL_PRESS_CHART, NULL)) != NULL) {
        lv_chart_remove_series(ui_OIL_PRESS_CHART, ser);
    }

    // Make the bars ~3x wider:
    // LVGL controls bar width mainly via the column padding (gap) on the chart.
    // Negative values intentionally overlap bars to increase thickness.
    lv_obj_set_style_pad_column(ui_OIL_PRESS_CHART, 4, LV_PART_MAIN | LV_STATE_DEFAULT);  // moderate width (no overlap)

    // Keep the chart "tight" (no dead padding / no grid)
    lv_obj_set_style_pad_left(ui_OIL_PRESS_CHART, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_OIL_PRESS_CHART, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_OIL_PRESS_CHART, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_OIL_PRESS_CHART, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_chart_set_div_line_count(ui_OIL_PRESS_CHART, 0, 0);

    // Shift mode so lv_chart_set_next_value() keeps "rolling" the history
    lv_chart_set_update_mode(ui_OIL_PRESS_CHART, LV_CHART_UPDATE_MODE_SHIFT);

    // One series on Primary Y
    // CHART DRAW CALLBACK DISABLED - was causing random freezing
    chart_series_oil_press = lv_chart_add_series(ui_OIL_PRESS_CHART, lv_color_hex(0xFF4619), LV_CHART_AXIS_PRIMARY_Y);

    // DISABLED: Hook draw-task callback (recolors bars based on value)
    // This was causing the random freezing issue
    // lv_obj_add_event_cb(ui_OIL_PRESS_CHART, oil_press_chart_draw_event_cb, LV_EVENT_DRAW_TASK_ADDED, NULL);
    // lv_obj_add_flag(ui_OIL_PRESS_CHART, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS);

    // Seed initial chart data
    if (chart_series_oil_press != NULL) {
        for (int i = 0; i < 24; i++) {
            lv_chart_set_next_value(ui_OIL_PRESS_CHART, chart_series_oil_press, oil_press_hist[i]);
        }
    }

    lv_chart_refresh(ui_OIL_PRESS_CHART);
    Serial.println("✅ Chart initialized (single series, extra-wide bars)");
}


    // Initialize oil temp chart (single series + draw-task recolor)
    if (ui_OIL_TEMP_CHART != NULL) {
        // Remove any SquareLine-created series (and any leftovers)
        lv_chart_series_t * ser2 = NULL;
        while ((ser2 = lv_chart_get_series_next(ui_OIL_TEMP_CHART, NULL)) != NULL) {
            lv_chart_remove_series(ui_OIL_TEMP_CHART, ser2);
        }

        // Match oil pressure chart bar thickness
        lv_obj_set_style_pad_column(ui_OIL_TEMP_CHART, 4, LV_PART_MAIN | LV_STATE_DEFAULT);

        // Keep the chart "tight" (no dead padding / no grid)
        lv_obj_set_style_pad_left(ui_OIL_TEMP_CHART, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(ui_OIL_TEMP_CHART, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_pad_top(ui_OIL_TEMP_CHART, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(ui_OIL_TEMP_CHART, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_chart_set_div_line_count(ui_OIL_TEMP_CHART, 0, 0);

        // Shift mode so lv_chart_set_next_value() keeps "rolling" the history
        lv_chart_set_update_mode(ui_OIL_TEMP_CHART, LV_CHART_UPDATE_MODE_SHIFT);

        // One series on Primary Y
        // CHART DRAW CALLBACK DISABLED - was causing random freezing
        chart_series_oil_temp = lv_chart_add_series(ui_OIL_TEMP_CHART, lv_color_hex(0xFF4619), LV_CHART_AXIS_PRIMARY_Y);

        // DISABLED: Hook draw-task callback (recolors bars based on value)
        // This was causing the random freezing issue
        // lv_obj_add_event_cb(ui_OIL_TEMP_CHART, oil_temp_chart_draw_event_cb, LV_EVENT_DRAW_TASK_ADDED, NULL);
        // lv_obj_add_flag(ui_OIL_TEMP_CHART, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS);

        // Seed initial chart data
        if (chart_series_oil_temp != NULL) {
            for (int i = 0; i < 24; i++) {
                lv_chart_set_next_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, oil_temp_hist[i]);
            }
        }

        lv_chart_refresh(ui_OIL_TEMP_CHART);
        Serial.println("✅ Oil temp chart initialized (single series, extra-wide bars)");
    }
    */
    // ====== END CHARTS DISABLED ======
    
    // Force initial screen refresh
    lv_refr_now(NULL);
    
    Serial.println("\n=== READY ===");
    Serial.println("✅ Display: Working");
    Serial.println("✅ Touch: Ready");
    Serial.println("✅ Backlight: ON");
    Serial.println("✅ UI: Loaded");
}

void loop()
{
    // Update LVGL's internal time tracking
    static unsigned long last_tick = 0;
    unsigned long now = millis();
    // Heartbeat: proves loop is running even if the screen appears frozen.
    static uint32_t last_hb = 0;
    if ((uint32_t)now - last_hb >= 1000) {
        last_hb = (uint32_t)now;
        Serial.printf("[HB] heap=%u psram=%u\n", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());
    }

    if(last_tick == 0) last_tick = now;
    lv_tick_inc(now - last_tick);
    last_tick = now;
    
    // FPS calculation (count actual screen refreshes, not loop iterations)
    static unsigned long last_fps_update = 0;
    
    if (now - last_fps_update >= 1000) {  // Update FPS every second
        if (fps_label != NULL) {
            char fps_buf[16];
            snprintf(fps_buf, sizeof(fps_buf), "%3d FPS", flush_count);  // Use flush_count for accurate FPS
            lv_label_set_text(fps_label, fps_buf);
        }
        flush_count = 0;  // Reset flush counter
        last_fps_update = now;
    }
    
    // Touch: keep it light and safe.
    // - only poll if touch successfully initialized
    // - poll at most every 100ms (reduced from 50ms for better I2C stability)
    // - only poll when INT is asserted
    // - recover I2C if it wedges
    static unsigned long last_touch_read = 0;
    static bool chart_updating = false;
    
    if (g_touch_init_ok && !chart_updating && now - last_touch_read >= 100) {
        touch_poll_cached();
        last_touch_read = now;
    }
    
    // Animate oil pressure (0-150 PSI) + oil temp (0-140°C / display also in °F)
    // These variables update the bars, labels, and rolling charts
    static int oil_pressure = 75;  // PSI
    static unsigned long last_update = 0;

    // Oil temp base is °C (UI range 0-140°C). We generate a smooth "hover" around 210°F ±5°F.
    static float oil_temp_f = 210.0f;  // internal animation helper (°F)

    if (now - last_update > 200) {  // Update every 200ms
        // ---------- OIL PRESSURE (PSI) ----------
        // Erratic random changes: -15 to +15 PSI per update
        int change = random(-15, 16);
        oil_pressure += change;

        // Clamp to valid range (0-150)
        if (oil_pressure > 150) {
            oil_pressure = 150;
        } else if (oil_pressure < 0) {
            oil_pressure = 0;
        }

        // Update the bar value + color
        if (ui_OIL_PRESS_Bar != NULL) {
            lv_bar_set_value(ui_OIL_PRESS_Bar, oil_pressure, BAR_ANIM);

            const bool press_red = (oil_pressure < 20) || (oil_pressure > 100);
            lv_obj_set_style_bg_color(ui_OIL_PRESS_Bar,
                                      press_red ? lv_color_hex(0xFF0000) : lv_color_hex(0xFF4619),
                                      LV_PART_INDICATOR | LV_STATE_DEFAULT);
        }

        // Update the label (format: "XXX PSI")
        if (ui_OIL_PRESS_Value != NULL) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d PSI", oil_pressure);
            lv_label_set_text(ui_OIL_PRESS_Value, buf);
        }

        // Accumulate samples for 5-second average chart (bucketed on real time)
        if(bucket_start_ms == 0) bucket_start_ms = now;
        pressure_sum += oil_pressure;
        pressure_samples++;

        // ---------- OIL TEMP (°C base; show °C and °F) ----------
        // Smooth hover around 210°F ±5°F with a little noise
        const float wave = 3.0f * sinf((float)now / 2000.0f);              // slow ~2s-ish sway
        const float noise = (float)random(-10, 11) / 10.0f;                // -1.0 .. +1.0
        oil_temp_f = 210.0f + wave + noise;

        if(oil_temp_f < 205.0f) oil_temp_f = 205.0f;
        if(oil_temp_f > 215.0f) oil_temp_f = 215.0f;

        float oil_temp_c_f = (oil_temp_f - 32.0f) * (5.0f / 9.0f);
        if(oil_temp_c_f < 0.0f) oil_temp_c_f = 0.0f;
        if(oil_temp_c_f > 140.0f) oil_temp_c_f = 140.0f;

        const int oil_temp_c = (int)lroundf(oil_temp_c_f);
        const int oil_temp_f_i = (int)lroundf(oil_temp_f);

        // Update the bar value + color (thresholds defined in °F)
        if (ui_OIL_TEMP_Bar != NULL) {
            lv_bar_set_value(ui_OIL_TEMP_Bar, oil_temp_c, BAR_ANIM);

            const bool temp_red = (oil_temp_f_i < 180) || (oil_temp_f_i > 260);
            lv_obj_set_style_bg_color(ui_OIL_TEMP_Bar,
                                      temp_red ? lv_color_hex(0xFF0000) : lv_color_hex(0xFF4619),
                                      LV_PART_INDICATOR | LV_STATE_DEFAULT);
        }

        // Update the temp label (base °C, converted °F)
        if (ui_OIL_TEMP_Value != NULL) {
            char buf[32];
            snprintf(buf, sizeof(buf), "%d°C\n%d°F", oil_temp_c, oil_temp_f_i);
            lv_label_set_text(ui_OIL_TEMP_Value, buf);
        }

        // Accumulate for 5-second average temp chart (°C buckets)
        if(temp_bucket_start_ms == 0) temp_bucket_start_ms = now;
        temp_sum += oil_temp_c;
        temp_samples++;

        // ---------- Push rolling averages every 5 seconds ----------
        // Oil pressure
        if((now - bucket_start_ms) >= OIL_PRESS_BUCKET_MS) {
            chart_updating = true;  // Briefly block touch reading during update

            int32_t average_pressure = (pressure_samples > 0) ? (pressure_sum / (int32_t)pressure_samples) : 0;
            // DISABLED FOR TESTING: oil_press_chart_push(average_pressure);

            // Reset accumulator for next 5-second period
            pressure_sum = 0;
            pressure_samples = 0;
            bucket_start_ms = now;

            chart_updating = false;
        }

        // Oil temp
        if((now - temp_bucket_start_ms) >= OIL_TEMP_BUCKET_MS) {
            chart_updating = true;

            int32_t average_temp_c = (temp_samples > 0) ? (temp_sum / (int32_t)temp_samples) : 0;
            // DISABLED FOR TESTING: oil_temp_chart_push(average_temp_c);

            temp_sum = 0;
            temp_samples = 0;
            temp_bucket_start_ms = now;

            chart_updating = false;
        }

        last_update = now;
    }
// Process LVGL tasks
    uint32_t t0 = micros();
    lv_timer_handler();
    uint32_t dt = micros() - t0;
    if (dt > 50000) {
        Serial.printf("[WARN] lv_timer_handler %u us\n", (unsigned)dt);
    }
    
    delay(5);
}
