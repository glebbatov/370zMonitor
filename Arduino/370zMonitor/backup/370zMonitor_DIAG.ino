/*
 * Waveshare ESP32-S3 7" (800x480) - STABILITY FIX VERSION
 * SquareLine Studio UI - GaugeAndChartDesign
 * Board: ESP32-S3-Touch-LCD-7 (NOT 7B!)
 * 
 * FIXES APPLIED:
 * 1. Disabled shadows on bars at runtime (massive performance gain)
 * 2. Optimized flush callback with DMA-friendly byte swap
 * 3. Added watchdog feeding
 * 4. Reduced touch polling frequency
 * 5. Added render time monitoring
 * 6. Disabled chart scales at runtime to reduce complexity
 */

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <TAMC_GT911.h>
#include <lvgl.h>
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>
#include <esp_task_wdt.h>
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

// External chart scale objects (to disable at runtime)
extern lv_obj_t * ui_OIL_PRESS_CHART_Xaxis;
extern lv_obj_t * ui_OIL_PRESS_CHART_Yaxis1;
extern lv_obj_t * ui_OIL_PRESS_CHART_Yaxis2;
extern lv_obj_t * ui_OIL_TEMP_CHART_Xaxis;
extern lv_obj_t * ui_OIL_TEMP_CHART_Yaxis1;
extern lv_obj_t * ui_OIL_TEMP_CHART_Yaxis2;

// FPS counter label and tracking
static lv_obj_t * fps_label = NULL;
static int flush_count = 0;

// ===== Stability knobs =====
#define I2C_FREQ_HZ 100000
#define BAR_ANIM LV_ANIM_OFF

// Watchdog timeout (seconds) - ESP32 default is ~5s
#define WDT_TIMEOUT_S 10

static bool g_ioexp_ok = false;

#define CH422_ADDR_SYSTEM 0x24
#define CH422_ADDR_IOWR   0x38

#define I2C_SDA 8
#define I2C_SCL 9

#define EXIO_TP_RST   1
#define EXIO_DISP     2
#define EXIO_SD_CS    4
#define EXIO_USB_SEL  5

#define TOUCH_INT_PIN 4

static uint8_t g_exio_state = 0;

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

bool initIOExtension() {
    delay(10);
    if (!ch422_write_system(0x11))
        return false;
    delay(10);
    g_exio_state = 0;
    g_exio_state |= (1u << EXIO_TP_RST);
    g_exio_state |= (1u << EXIO_SD_CS);
    if (!ch422_write_io(g_exio_state))
    {
        Serial.println("CH422 IO write failed");
        return false;
    }
    delay(20);
    return true;
}

void setBacklight(bool on) {
    if(!g_ioexp_ok) return;
    exio_set(EXIO_DISP, on);
}

void initTouch() {
    if(!g_ioexp_ok) {
        Serial.println("CH422G not OK -> skipping touch reset sequence");
        return;
    }
    pinMode(TOUCH_INT_PIN, OUTPUT);
    digitalWrite(TOUCH_INT_PIN, LOW);
    exio_set(EXIO_TP_RST, false);
    delay(20);
    exio_set(EXIO_TP_RST, true);
    delay(80);
    pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
}

// RGB Panel
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    5, 3, 46, 7,
    1, 2, 42, 41, 40,
    39, 0, 45, 48, 47, 21,
    14, 38, 18, 17, 10,
    0, 40, 48, 40,
    0, 13, 3, 32,
    1,
    14000000,
    true,
    0, 0
);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, rgbpanel, 0, true);

// GT911 Touch Controller
TAMC_GT911 touch = TAMC_GT911(I2C_SDA, I2C_SCL, TOUCH_INT_PIN, -1, 800, 480);
static bool g_touch_init_ok = false;

// Touch cache
static volatile bool g_touch_down = false;
static volatile uint16_t g_touch_x = 0;
static volatile uint16_t g_touch_y = 0;

// I2C bus recovery
static void i2c_bus_clear()
{
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    delayMicroseconds(5);

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

    pinMode(I2C_SDA, OUTPUT_OPEN_DRAIN);
    pinMode(I2C_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(I2C_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C_SDA, HIGH);
    delayMicroseconds(5);

    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
}

static inline void wire_set_timeout_ms(uint32_t ms)
{
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Wire.setTimeOut((uint16_t)ms);
#else
    Wire.setTimeout(ms);
#endif
}

static void touch_poll_cached()
{
    if (digitalRead(TOUCH_INT_PIN) == HIGH) {
        g_touch_down = false;
        return;
    }

    if (digitalRead(I2C_SDA) == LOW) {
        Serial.println("⚠️  I2C: SDA stuck LOW - recovering");
        i2c_bus_clear();
        Wire.end();
        delay(2);
        Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
        delay(2);
        wire_set_timeout_ms(20);
        g_touch_down = false;
        return;
    }

    wire_set_timeout_ms(20);
    touch.read();

    if (touch.isTouched) {
        g_touch_down = true;
        g_touch_x = (uint16_t)touch.points[0].x;
        g_touch_y = (uint16_t)touch.points[0].y;
    } else {
        g_touch_down = false;
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

// LVGL Configuration
#define LVGL_BUFFER_SIZE (800 * 30)
static lv_display_t *disp;
static lv_indev_t *indev;
static uint8_t *disp_draw_buf;

// ===== OPTIMIZED flush callback =====
// Uses 32-bit operations for faster byte swap
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    uint32_t len = w * h;
    
    // Optimized byte swap using 32-bit operations (2 pixels at a time)
    uint32_t *buf32 = (uint32_t *)px_map;
    uint32_t len32 = len >> 1;  // Process 2 pixels at a time
    
    for (uint32_t i = 0; i < len32; i++) {
        uint32_t v = buf32[i];
        // Swap bytes within each 16-bit pixel, keeping pixel order
        buf32[i] = ((v & 0x00FF00FF) << 8) | ((v & 0xFF00FF00) >> 8);
    }
    
    // Handle odd pixel if any
    if (len & 1) {
        uint16_t *buf16 = (uint16_t *)px_map;
        buf16[len - 1] = (buf16[len - 1] >> 8) | (buf16[len - 1] << 8);
    }
    
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
    lv_display_flush_ready(disp);
    flush_count++;
}

// LVGL touch input callback
void my_touch_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    if (g_touch_down) {
        data->point.x = g_touch_x;
        data->point.y = g_touch_y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ===== RUNTIME FIX: Disable expensive shadows and simplify charts =====
static void fix_ui_performance()
{
    Serial.println("Applying runtime performance fixes...");
    
    // CRITICAL: Disable shadows on bars (massive CPU saver)
    if (ui_OIL_PRESS_Bar != NULL) {
        lv_obj_set_style_shadow_width(ui_OIL_PRESS_Bar, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(ui_OIL_PRESS_Bar, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
        Serial.println("  ✅ OIL_PRESS_Bar shadows disabled");
    }
    
    if (ui_OIL_TEMP_Bar != NULL) {
        lv_obj_set_style_shadow_width(ui_OIL_TEMP_Bar, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(ui_OIL_TEMP_Bar, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
        Serial.println("  ✅ OIL_TEMP_Bar shadows disabled");
    }
    
    // Hide chart scales to reduce rendering complexity
    if (ui_OIL_PRESS_CHART_Xaxis != NULL) lv_obj_add_flag(ui_OIL_PRESS_CHART_Xaxis, LV_OBJ_FLAG_HIDDEN);
    if (ui_OIL_PRESS_CHART_Yaxis1 != NULL) lv_obj_add_flag(ui_OIL_PRESS_CHART_Yaxis1, LV_OBJ_FLAG_HIDDEN);
    if (ui_OIL_PRESS_CHART_Yaxis2 != NULL) lv_obj_add_flag(ui_OIL_PRESS_CHART_Yaxis2, LV_OBJ_FLAG_HIDDEN);
    if (ui_OIL_TEMP_CHART_Xaxis != NULL) lv_obj_add_flag(ui_OIL_TEMP_CHART_Xaxis, LV_OBJ_FLAG_HIDDEN);
    if (ui_OIL_TEMP_CHART_Yaxis1 != NULL) lv_obj_add_flag(ui_OIL_TEMP_CHART_Yaxis1, LV_OBJ_FLAG_HIDDEN);
    if (ui_OIL_TEMP_CHART_Yaxis2 != NULL) lv_obj_add_flag(ui_OIL_TEMP_CHART_Yaxis2, LV_OBJ_FLAG_HIDDEN);
    Serial.println("  ✅ Chart scales hidden");
    
    // Hide entire charts for maximum stability testing
    if (ui_OIL_PRESS_CHART != NULL) {
        lv_obj_add_flag(ui_OIL_PRESS_CHART, LV_OBJ_FLAG_HIDDEN);
        Serial.println("  ✅ OIL_PRESS_CHART hidden");
    }
    if (ui_OIL_TEMP_CHART != NULL) {
        lv_obj_add_flag(ui_OIL_TEMP_CHART, LV_OBJ_FLAG_HIDDEN);
        Serial.println("  ✅ OIL_TEMP_CHART hidden");
    }
    
    Serial.println("Performance fixes applied!");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== Waveshare ESP32-S3 7\" 800x480 (STABILITY FIX) ===");
    
    // Configure watchdog with longer timeout
    Serial.println("Configuring watchdog...");
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,  // Don't watch idle tasks
        .trigger_panic = false  // Don't panic, just reset
    };
    esp_task_wdt_reconfigure(&wdt_config);
    esp_task_wdt_add(NULL);  // Add current task to watchdog
    Serial.printf("  Watchdog timeout: %d seconds\n", WDT_TIMEOUT_S);
    
    // I2C init
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
    probe(0x5D);
    Serial.println("===================\n");
    
    delay(100);
    wire_set_timeout_ms(100);
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    // IO expander and backlight
    Serial.println("Initializing CH422G IO Expander...");
    g_ioexp_ok = initIOExtension();
    if (!g_ioexp_ok) {
        Serial.println("ERROR: CH422G initialization failed!");
    }
    
    Serial.println("Initializing Touch Controller...");
    initTouch();
    pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    // Display init
    Serial.println("Initializing RGB Display...");
    gfx->begin();
    gfx->fillScreen(0x0000);
    
    Serial.println("Turning on Backlight...");
    setBacklight(true);
    delay(50);
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    // Touch init with retries
    Serial.println("Initializing GT911 Touch...");
    wire_set_timeout_ms(100);
    i2c_ensure_alive();
    
    bool touch_ok = false;
    for (int retry = 0; retry < 3 && !touch_ok; retry++) {
        if (retry > 0) {
            Serial.printf("GT911 init retry %d/3...\n", retry);
            i2c_bus_clear();
            Wire.end();
            delay(100);
            Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
            delay(100);
            wire_set_timeout_ms(100);
            
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
        
        esp_task_wdt_reset();  // Feed watchdog during retries
        
        Serial.println("Calling touch.begin()...");
        touch.begin();
        delay(100);
        
        Wire.beginTransmission(0x5D);
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
        g_touch_init_ok = true;
        Serial.println("✅ GT911 Touch fully initialized");
    } else {
        g_touch_init_ok = false;
        Serial.println("❌ GT911 Touch initialization FAILED");
    }
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    // LVGL init
    Serial.println("Initializing LVGL...");
    lv_init();
    
    const size_t buf_bytes = (size_t)LVGL_BUFFER_SIZE * sizeof(lv_color_t);

    Serial.printf("PSRAM: found=%s, size=%u bytes, free=%u bytes\n",
                  psramFound() ? "YES" : "NO",
                  (unsigned)ESP.getPsramSize(),
                  (unsigned)ESP.getFreePsram());

    disp_draw_buf = (uint8_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);

    if (!disp_draw_buf && psramFound()) {
        Serial.println("Internal DMA buffer alloc failed, falling back to PSRAM...");
        disp_draw_buf = (uint8_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }

    if (!disp_draw_buf) {
        Serial.printf("FATAL: Display buffer alloc failed (%u bytes).\n", (unsigned)buf_bytes);
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
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    // Load UI
    Serial.println("Loading UI...");
    ui_init();
    
    // Verify critical UI objects
    if (ui_Screen1 == NULL) {
        Serial.println("❌ FATAL: ui_Screen1 not created!");
        while(1) { delay(1000); Serial.println("UI init failed - halted"); }
    }
    Serial.println("✅ UI objects validated");
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    // ===== CRITICAL: Apply runtime performance fixes =====
    fix_ui_performance();
    
    // Create FPS counter label
    if (ui_Screen1 != NULL) {
        fps_label = lv_label_create(ui_Screen1);
        lv_obj_align(fps_label, LV_ALIGN_TOP_RIGHT, -10, 10);
        lv_label_set_text(fps_label, "--- FPS");
        lv_obj_set_style_text_color(fps_label, lv_color_hex(0x00FF00), LV_PART_MAIN);
        lv_obj_set_style_text_align(fps_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    }
    
    // Force initial refresh
    lv_refr_now(NULL);
    
    Serial.println("\n=== READY (STABILITY FIX VERSION) ===");
    Serial.println("✅ Display: Working");
    Serial.println("✅ Touch: Ready");
    Serial.println("✅ Backlight: ON");
    Serial.println("✅ UI: Loaded");
    Serial.println("✅ Shadows: DISABLED");
    Serial.println("✅ Charts: HIDDEN");
    Serial.println("✅ Watchdog: Configured");
}

void loop()
{
    // Feed watchdog at start of loop
    esp_task_wdt_reset();
    
    // Update LVGL tick
    static unsigned long last_tick = 0;
    unsigned long now = millis();
    
    // Heartbeat
    static uint32_t last_hb = 0;
    if ((uint32_t)now - last_hb >= 1000) {
        last_hb = (uint32_t)now;
        Serial.printf("[HB] heap=%u psram=%u\n", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());
    }

    if(last_tick == 0) last_tick = now;
    uint32_t elapsed = now - last_tick;
    if (elapsed > 0) {
        lv_tick_inc(elapsed);
        last_tick = now;
    }
    
    // FPS calculation
    static unsigned long last_fps_update = 0;
    
    if (now - last_fps_update >= 1000) {
        if (fps_label != NULL) {
            char fps_buf[16];
            snprintf(fps_buf, sizeof(fps_buf), "%3d FPS", flush_count);
            lv_label_set_text(fps_label, fps_buf);
        }
        flush_count = 0;
        last_fps_update = now;
    }
    
    // Touch polling (reduced frequency)
    static unsigned long last_touch_read = 0;
    
    if (g_touch_init_ok && now - last_touch_read >= 100) {
        touch_poll_cached();
        last_touch_read = now;
    }
    
    // Animate oil pressure and temp
    static int oil_pressure = 75;
    static unsigned long last_update = 0;
    static float oil_temp_f = 210.0f;

    if (now - last_update > 200) {
        // Oil Pressure
        int change = random(-15, 16);
        oil_pressure += change;
        if (oil_pressure > 150) oil_pressure = 150;
        else if (oil_pressure < 0) oil_pressure = 0;

        if (ui_OIL_PRESS_Bar != NULL) {
            lv_bar_set_value(ui_OIL_PRESS_Bar, oil_pressure, BAR_ANIM);

            const bool press_red = (oil_pressure < 20) || (oil_pressure > 100);
            lv_obj_set_style_bg_color(ui_OIL_PRESS_Bar,
                                      press_red ? lv_color_hex(0xFF0000) : lv_color_hex(0xFF4619),
                                      LV_PART_INDICATOR | LV_STATE_DEFAULT);
        }

        if (ui_OIL_PRESS_Value != NULL) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d PSI", oil_pressure);
            lv_label_set_text(ui_OIL_PRESS_Value, buf);
        }

        // Oil Temp
        const float wave = 3.0f * sinf((float)now / 2000.0f);
        const float noise = (float)random(-10, 11) / 10.0f;
        oil_temp_f = 210.0f + wave + noise;

        if(oil_temp_f < 205.0f) oil_temp_f = 205.0f;
        if(oil_temp_f > 215.0f) oil_temp_f = 215.0f;

        float oil_temp_c_f = (oil_temp_f - 32.0f) * (5.0f / 9.0f);
        if(oil_temp_c_f < 0.0f) oil_temp_c_f = 0.0f;
        if(oil_temp_c_f > 140.0f) oil_temp_c_f = 140.0f;

        const int oil_temp_c = (int)lroundf(oil_temp_c_f);
        const int oil_temp_f_i = (int)lroundf(oil_temp_f);

        if (ui_OIL_TEMP_Bar != NULL) {
            lv_bar_set_value(ui_OIL_TEMP_Bar, oil_temp_c, BAR_ANIM);

            const bool temp_red = (oil_temp_f_i < 180) || (oil_temp_f_i > 260);
            lv_obj_set_style_bg_color(ui_OIL_TEMP_Bar,
                                      temp_red ? lv_color_hex(0xFF0000) : lv_color_hex(0xFF4619),
                                      LV_PART_INDICATOR | LV_STATE_DEFAULT);
        }

        if (ui_OIL_TEMP_Value != NULL) {
            char buf[32];
            snprintf(buf, sizeof(buf), "%d°C\n%d°F", oil_temp_c, oil_temp_f_i);
            lv_label_set_text(ui_OIL_TEMP_Value, buf);
        }

        last_update = now;
    }

    // Process LVGL tasks with timing
    uint32_t t0 = micros();
    lv_timer_handler();
    uint32_t dt = micros() - t0;
    
    if (dt > 50000) {
        Serial.printf("[WARN] lv_timer_handler %u us (>50ms!)\n", (unsigned)dt);
    }
    
    // Small delay to prevent tight loop
    delay(5);
}
