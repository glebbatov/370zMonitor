//-----------------------------------------------------------------

/*
 * 370zMonitor v5.5
 * Supports Demo Mode (animated values) and Live Mode (sensors data/OBD data)
 * ESP32-S3 with PSRAM, LVGL, GT911 Touch
 *
 * v5.5 Changes:
 * - FIXED: UI shows stale values when Modbus disconnects (now shows "---" with unit)
 * - FIXED: Oil, Trans, Steer, Diff temps reset to "---°F" on sensor disconnect
 * - FIXED: Lightweight bars reset to 0 on sensor disconnect
 * - FIXED: Critical labels hidden immediately on sensor disconnect
 * - IMPROVED: Modbus logging consolidated to single line per second
 * - IMPROVED: Compact log format: P:0 OilT:80F Trans:--- Steer:--- Diff:---
 * - ADDED: "!" indicator in logs for temps exceeding critical thresholds
 *
 * v5.4 Changes:
 * - Expanded Modbus to 5 channels: Oil Press, Oil Temp, Trans Temp, Steer Temp, Diff Temp
 * - All temperature channels use PRTXI 4-20mA sensors with hysteresis
 * - Auto-configure CH2-CH5 for 4-20mA mode at startup
 * - Individual sensor connect/disconnect logging per channel
 * - Added periodic value logging for CH3-CH5 (every ~2 sec when connected)
 * - Toast system monitors all 5 sensors for online/offline status
 *
 * v5.3 Changes:
 * - Added +5°C calibration offset to PRTXI temperature reading
 * - Ice water test confirmed PRTXI accuracy (0°C in 0°C ice water)
 * - Offset provides conservative margin for metal surface measurements
 * - Updated clamp range to -45°C to 205°C (accounts for offset)
 * - Added temperature hysteresis (0.3°C) to prevent display jumping
 *
 * v5.2 Changes:
 * - FIXED: Waveshare mode register address (0x0001 → 0x1001 per wiki)
 * - CH2 now correctly configured to Mode 3 (4-20mA current input)
 * - Module returns µA directly - no mV→mA conversion needed!
 * - Simplified convertToTempC(): direct µA → °C linear conversion
 * - Updated debug logging to show µA values
 *
 * v5.1 Changes:
 * - Attempted voltage-mode fix (incorrect - wrong register address)
 * - Oil temp gauge simplified to single value per unit
 *
 * v5.0 Changes:
 * - PRTXI-1/2N-1/4-4-IO RTD transmitter on Modbus CH2
 * - PRTXI outputs 4-20mA for -50°C to +200°C (loop-powered, 2-wire)
 * - CH2 jumper ON (connected) for current mode input
 *
 * v4.9 Changes (historical):
 * - Added oil temperature sensor on Modbus CH2
 * - Now reading 2 channels from Waveshare 8-Ch module (pressure + temp)
 * - Added convertToTempC() for linear current to temperature conversion
 * - Oil temperature displayed via existing gauge with F/C unit toggle
 * - Toast system monitors oil temp sensor connection status
 *
 * v4.8 Changes:
 * - Added system status toast notification on startup
 * - Green toast "All Systems Online" if all systems OK (3 sec)
 * - Red toast lists all failures if any system offline (15 sec)
 * - Checks: SD Card, Logs Writing, RTC (HW-084), Time Sync, Modbus RTU, Sensors
 * - Reusable showToast() function for future notifications
 * - Comprehensive Serial.printf logging for all system checks
 * - FIXED: Touch not working on cold boot (GT911 power-on timing)
 * - Double-reset cycle ensures reliable GT911 init on cold boot
 *
 * v4.7 Changes:
 * - File browser OPTIMIZED: Uses boot_count to generate filenames (instant load)
 * - Session filenames expanded: 5-digit → 8-digit (SESS_00000542 format)
 * - Supports up to 99,999,999 sessions before overflow
 * - Back button alignment fixed (LV_ALIGN_LEFT_MID)
 * - Text viewer uses white text for readability
 *
 * v4.6 Changes:
 * - Added SD card file browser mode (view logs in the field without PC)
 * - Entry: Double-tap screen -> tap FILES button in utility box
 * - Supports CSV files (grid view with auto-width columns)
 * - Supports text files (.txt, .ini, .log, .cfg)
 * - Navigate folders, view files, exit by backing from root
 * - Loading indicator for large files
 *
 * v4.5 Changes:
 * - Added sensor/modbus failure detection and UI feedback
 * - UI shows "---" when sensor disconnected or modbus fails
 * - Critical value label hidden when sensor invalid
 * - Sensor disconnect detection (0 mV = sensor not connected)
 * - Faster modbus error response (3 errors vs 5)
 * - Improved logging for sensor status changes
 *
 * v4.4 Changes:
 * - Added 5-second splash screen with "370zMONITOR" branding
 * - Splash shows 2018 NISSAN 370Z subtitle with Passion Red accents
 * - Animated loading bar during startup
 * - Smooth fade transition to main gauge screen
 *
 * v4.3 Changes:
 * - Fixed USB MSC screen to show SD card type/size
 * - Fixed duplicate CSV header bug (FILE_APPEND -> FILE_WRITE)
 *
 * v4.2 Changes:
 * - Dual-core architecture: Touch + SD I/O on Core 0, LVGL on Core 1
 * - Touch polling via dedicated FreeRTOS task with mutex
 * - SD logging via queue-based producer/consumer pattern
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

/*
 * Quick performance points:
 * UPDATE_INTERVAL_MS - how often the UI updates with new values
 * I2C_FREQ_HZ - speed of communication with the GT911 touch controller
 * CHART_BLINK_INTERVAL_MS - how fast critical chart bars blink red/dim
 * LABEL_BLINK_INTERVAL_MS - how fast value_critical lable blinks
 * LVGL_BUFFER_SIZE - how much of the screen LVGL renders at once
 * SD_FLUSH_INTERVAL_MS - flush to sd card interval in ms
*/

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <lvgl.h>

// Forward declaration for LVGL's optimized RGB565 byte-swap function
extern "C" void lv_draw_sw_rgb565_swap(void * buf, uint32_t buf_size_px);
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>
#include <TAMC_GT911.h>             // Touch controller
#include <SD.h>
#include <SPI.h>
#include <Preferences.h>            // For persistent unit preferences
#include "USB.h"                    // ESP32-S3 native USB
#include "USBMSC.h"                 // USB Mass Storage Class
#include "esp_freertos_hooks.h"     // to see the CPU load
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <WiFi.h>                   // WiFi for NTP time sync
#include <time.h>                   // Time functions
#include <stdarg.h>                 // For va_list in SerialLogf
#include <vector>                   // For file browser

//-----------------------------------------------------------------

// ===== FEATURE FLAGS (must be before TeeSerial) =====
#define ENABLE_SD_LOGGING       1   // Enable SD card data logging
#define ENABLE_FILE_BROWSER     1   // Enable SD card file browser (tap FILES in utility box)

// Global flag for file browser to pause SD writes (defined before file_browser.h include)
volatile bool g_fb_pause_sd_writes = false;

// Export boot count for file browser to generate recent session filenames
uint32_t g_current_boot_count = 0;

// File browser include (must be after SD.h and lvgl.h)
#if ENABLE_FILE_BROWSER
#include "file_browser.h"
#endif

//=================================================================
// TESERIAL - TRANSPARENT SERIAL WRAPPER (must be early in file!)
// Captures ALL Serial output to SD card automatically
// No code changes needed - just use Serial.println() as normal!
//=================================================================

#if ENABLE_SD_LOGGING

#define SERIAL_LOG_MAX_MSG_LEN 256

// Forward declaration for SD queue function
void sdLogSerialWrite(const char* msg);

// Store reference to the REAL Serial before we redefine it
// Use auto& because Serial is HWCDC when USB CDC is enabled, HardwareSerial otherwise
auto& _RealSerial = Serial;

class TeeSerial : public Print {
private:
    char _lineBuffer[SERIAL_LOG_MAX_MSG_LEN];
    size_t _linePos;
    
public:
    TeeSerial() : _linePos(0) {
        memset(_lineBuffer, 0, sizeof(_lineBuffer));
    }
    
    // Core write function - called by all print/println variants
    size_t write(uint8_t c) override {
        // Always forward to real Serial immediately
        _RealSerial.write(c);
        
        // Buffer the character for SD logging
        if (c == '\n' || c == '\r') {
            // End of line - queue the buffer if we have content
            if (_linePos > 0) {
                _lineBuffer[_linePos] = '\0';
                sdLogSerialWrite(_lineBuffer);
                _linePos = 0;
            }
        } else {
            // Add to buffer if there's room
            if (_linePos < sizeof(_lineBuffer) - 1) {
                _lineBuffer[_linePos++] = c;
            }
        }
        
        return 1;
    }
    
    // Bulk write - more efficient for larger outputs
    size_t write(const uint8_t* buffer, size_t size) override {
        for (size_t i = 0; i < size; i++) {
            write(buffer[i]);
        }
        return size;
    }
    
    // Forward HardwareSerial methods we need
    // Note: HWCDC::begin() only takes baud, ignores config parameter
    void begin(unsigned long baud) { _RealSerial.begin(baud); }
    void begin(unsigned long baud, uint32_t config) { (void)config; _RealSerial.begin(baud); }
    void end() { _RealSerial.end(); }
    int available() { return _RealSerial.available(); }
    int read() { return _RealSerial.read(); }
    int peek() { return _RealSerial.peek(); }
    void flush() { _RealSerial.flush(); }
    
    // Implement printf (not inherited from Print)
    int printf(const char* format, ...) __attribute__((format(printf, 2, 3))) {
        char buf[SERIAL_LOG_MAX_MSG_LEN];
        va_list args;
        va_start(args, format);
        int len = vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);
        write((const uint8_t*)buf, len > 0 ? len : 0);
        return len;
    }
    
    // Explicit bool conversion for if(Serial) checks
    explicit operator bool() const { return true; }
};

// Create global instance
TeeSerial _TeeSerialInstance;

// MAGIC: Redefine Serial to use our wrapper
// All Serial.print/println/printf calls now automatically go to SD card too!
#define Serial _TeeSerialInstance

#endif // ENABLE_SD_LOGGING

//-----------------------------------------------------------------

 // ===== CRITICAL: Configure ESP32 to prefer PSRAM for malloc =====
__attribute__((constructor)) void configurePSRAM() {
    heap_caps_malloc_extmem_enable(32 * 1024);
}

//-----------------------------------------------------------------

// ===== FEATURE FLAGS =====
#define ENABLE_TOUCH                    1
#define ENABLE_UI_UPDATES               1       // Enable all UI updates (master switch)

#define ENABLE_BARS                     0       // DISABLED - expensive SquareLine bars cause CPU spikes
//OR
#define ENABLE_LIGHTWEIGHT_BARS         1       // Simple rectangle overlays (much cheaper than lv_bar)

#define ENABLE_CRITICAL_LABEL_BLINK     0       // 0 = static white/black, 1 = blinking animation
#define ENABLE_VALUE_CRITICAL           1       // Enable "Value Critical" labels

#define ENABLE_CHARTS                   1       // Enable charts
// ENABLE_SD_LOGGING defined at top of file (before TeeSerial)
#define ENABLE_USB_MSC                  1       // Enable USB Mass Storage mode (hold BOOT at startup)
#define ENABLE_MODBUS_SENSORS           1       // Enable Modbus RS485 sensor reading (8-Ch Analog Module)
#define UPDATE_INTERVAL_MS              25      // default 250ms

// USB MSC Configuration
#define USB_MSC_BOOT_PIN                0       // GPIO0 = BOOT button on most ESP32-S3 boards

//-----------------------------------------------------------------

// ===== UNIT TYPES =====
enum TempUnit { TEMP_FAHRENHEIT = 0, TEMP_CELSIUS = 1 };
enum PressureUnit { PRESS_PSI = 0, PRESS_BAR = 1, PRESS_KPA = 2, PRESS_ATM = 3 };

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
float psiToAtm(float psi) { return (psi + 14.6959f) / 14.6959f; }  // Gauge PSI to absolute ATM
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
    case PRESS_ATM: return psiToAtm(value_psi);
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
    case PRESS_ATM: return "ATM";
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
// (!) valid - flags indicate whether real sensor data has been received for that value

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
void resetTapPanelOpacity();
void resetDemoState();
void updateTapBoxVisibility();
void resetUIElements();

//-----------------------------------------------------------------

//=================================================================
//=================================================================
// MODBUS RS485 SENSOR INTEGRATION
//=================================================================
//
// Reads analog sensor data from Waveshare Industrial 8-Ch Analog 
// Acquisition Module via RS485 Modbus RTU protocol.
//
// HARDWARE SETUP:
// ---------------
// The Waveshare ESP32-S3-Touch-LCD-7 has an onboard SP3485 RS485 
// transceiver with auto-direction control (no DE/RE pin needed).
//
//   ESP32-S3                SP3485                 Modbus Device
//   ┌─────────┐           ┌─────────┐             ┌─────────┐
//   │  GPIO16 │── TX ────>│ DI      │             │         │
//   │  (TX)   │           │      A ─│─────────────│─ A      │
//   │         │           │         │  RS485 Bus  │         │
//   │  GPIO15 │<── RX ────│ RO      │             │         │
//   │  (RX)   │           │      B ─│─────────────│─ B      │
//   └─────────┘           └─────────┘             └─────────┘
//
// MODBUS RTU PROTOCOL:
// --------------------
// Modbus RTU is a binary protocol where each message has this structure:
//
//   ┌──────────┬───────────┬────────────┬───────────┐
//   │ Address  │ Function  │   Data     │  CRC-16   │
//   │ (1 byte) │ (1 byte)  │ (N bytes)  │ (2 bytes) │
//   └──────────┴───────────┴────────────┴───────────┘
//
// This code uses Function Code 0x04 = "Read Input Registers" (for analog inputs).
//
// REQUEST FORMAT (Read 1 register from address 0x0000):
//   Byte:  [0]   [1]   [2]   [3]   [4]   [5]   [6]   [7]
//   Value: 0x01  0x04  0x00  0x00  0x00  0x01  0x31  0xCA
//          │     │     │     │     │     │     └───────── CRC-16 (little-endian)
//          │     │     │     │     └─────┴── Quantity: 1 register
//          │     │     └─────┴── Starting address: 0x0000
//          │     └── Function: 0x04 (Read Input Registers)
//          └── Slave address: 1
//
// RESPONSE FORMAT (Slave returns register value):
//   Byte:  [0]   [1]   [2]   [3]   [4]   [5]   [6]
//   Value: 0x01  0x04  0x02  0x01  0x33  0xF8  0xB5
//          │     │     │     │     │     └───────── CRC-16
//          │     │     │     └─────┴── Data: 0x0133 = 307 mV
//          │     │     └── Byte count: 2 bytes of data
//          │     └── Function: 0x04 (echo)
//          └── Slave address: 1 (echo)
//
// TIMING DIAGRAM:
// ---------------
//          ESP32                              8-Ch Module
//            │                                     │
//            │  ┌───────────────────────────────┐  │
//            │──│ TX: 01 04 00 00 00 01 31 CA   │──│  Request
//            │  └───────────────────────────────┘  │
//            │                                     │
//            │         ~50-100ms delay             │
//            │                                     │
//            │  ┌───────────────────────────────┐  │
//            │──│ RX: 01 04 02 01 33 F8 B5      │──│  Response
//            │  └───────────────────────────────┘  │
//            │                                     │
//            │   Parse: 0x0133 = 307 mV            │
//            │   Convert: 307 mV → 0.0 PSI         │
//
// PRESSURE SENSOR CONVERSION:
// ---------------------------
// Sensor: PX3AN2BH150PSAAX outputs 0.5V-4.5V for 0-150 PSI
// Voltage divider: 10kΩ / 22kΩ reduces voltage by factor of 0.6875
//
//   Modbus reads ~307mV at 0 PSI (500mV * 0.6875 ≈ 344mV theoretical)
//   Formula: PSI = (raw_mV * 1.4545 - 500) * 0.0375
//
// ERROR HANDLING:
// ---------------
//   | Condition            | Detection                    | Action                        |
//   |----------------------|------------------------------|-------------------------------|
//   | No response          | received == 0                | Increment error count         |
//   | Partial response     | received < expected          | Log debug, increment error    |
//   | CRC mismatch         | receivedCRC != calculatedCRC | Return false                  |
//   | Sensor disconnected  | mV < 100                     | Mark oil_pressure_valid=false |
//   | 3+ consecutive errors| error_count >= 3             | Mark invalid, show "---"      |
//
// RS485 CABLE LIMITS (at 9600 baud):
// -----------------------------------
//   - Maximum distance: ~1200m (4000 ft)
//   - Your 8-foot cable is well within safe limits
//   - Termination resistor (120Ω): optional for short runs, recommended >50ft
//   - Shielded cable recommended in automotive environment
//   - Ground shield at ONE end only to avoid ground loops
//
//=================================================================

#if ENABLE_MODBUS_SENSORS

// RS485 Serial Port Configuration
// Waveshare ESP32-S3-Touch-LCD-7 uses SP3485 with auto-direction (no DE pin needed)
#define RS485_SERIAL        Serial1
#define RS485_BAUD          9600
#define RS485_RX_PIN        15      // GPIO15 = RS485_RXD (confirmed from schematic)
#define RS485_TX_PIN        16      // GPIO16 = RS485_TXD (confirmed from schematic)
#define RS485_CONFIG        SERIAL_8N1

// Modbus Configuration
#define MODBUS_SLAVE_ADDR   1       // 8-Ch module default address
#define MODBUS_TIMEOUT_MS   100     // Response timeout
#define MODBUS_RETRY_COUNT  2       // Retries on failure
#define MODBUS_READ_INTERVAL_MS 100 // How often to poll sensors (ms)

// Sensor Channel Mapping on 8-Ch Module
#define MODBUS_CH_OIL_PRESSURE  0   // Channel 1 (AI1) - PX3 pressure sensor (0-10V)
#define MODBUS_CH_OIL_TEMP      1   // Channel 2 (AI2) - PRTXI RTD transmitter (4-20mA)
#define MODBUS_CH_TRANS_TEMP    2   // Channel 3 (AI3) - PRTXI RTD transmitter (4-20mA)
#define MODBUS_CH_STEER_TEMP    3   // Channel 4 (AI4) - PRTXI RTD transmitter (4-20mA)
#define MODBUS_CH_DIFF_TEMP     4   // Channel 5 (AI5) - PRTXI RTD transmitter (4-20mA)

// Number of channels to read
#define MODBUS_NUM_CHANNELS     5   // Oil pressure + 4 temperature sensors

// Waveshare Channel Mode Configuration (Holding Registers)
// Per wiki: Register 4x1000-4x1007 = Channels 1-8 data types
// Mode values: 0=0-10V, 1=2-10V, 2=0-20mA, 3=4-20mA, 4=raw ADC
#define WAVESHARE_MODE_0_10V    0x00  // Mode 0: 0-10V voltage (B version default)
#define WAVESHARE_MODE_2_10V    0x01  // Mode 1: 2-10V voltage
#define WAVESHARE_MODE_0_20MA   0x02  // Mode 2: 0-20mA current
#define WAVESHARE_MODE_4_20MA   0x03  // Mode 3: 4-20mA current <-- We need this for PRTXI
#define WAVESHARE_MODE_RAW_ADC  0x04  // Mode 4: Raw 4096-scale ADC code
#define WAVESHARE_CH1_MODE_REG  0x1000  // Holding register for CH1 mode
#define WAVESHARE_CH2_MODE_REG  0x1001  // Holding register for CH2 mode
#define WAVESHARE_CH3_MODE_REG  0x1002  // Holding register for CH3 mode
#define WAVESHARE_CH4_MODE_REG  0x1003  // Holding register for CH4 mode
#define WAVESHARE_CH5_MODE_REG  0x1004  // Holding register for CH5 mode

// Pressure Sensor Calibration (PX3AN2BH150PSAAX with voltage divider)
// Your voltage divider: 10kΩ / 22kΩ = ratio of 0.6875
#define PRESSURE_DIVIDER_RATIO  1.4545f  // Inverse of 0.6875
#define PRESSURE_OFFSET_MV      500.0f   // 0.5V = 500mV at 0 PSI
#define PRESSURE_SCALE          0.0375f  // 150 PSI / 4000 mV range

// PRTXI Temperature Sensor Calibration (4-20mA output, loop-powered)
// PRTXI-1/2N-1/4-4-IO outputs 4-20mA linear for -50°C to +200°C (250°C range)
//
// CONFIGURATION: Waveshare module set to Mode 3 (4-20mA) via register 0x1001
// With CH2 jumper ON (connected), module returns microamps (µA) directly!
// No voltage-to-current conversion needed.
//
// Conversion: µA → °C (simple linear)
//   temp_C = ((µA - 4000) / 16000) * 250 - 50
//
// Expected readings at known temperatures:
//   -50°C: 4000 µA (4.00 mA)
//   25°C:  8800 µA (8.80 mA)  - room temp
//   54°C: 10656 µA (10.66 mA) - hot water test
//   200°C: 20000 µA (20.00 mA)
//
#define PRTXI_MIN_CURRENT_UA    4000      // 4000 µA (4mA) = -50°C
#define PRTXI_CURRENT_SPAN_UA   16000     // 20000 - 4000 = 16000 µA span
#define PRTXI_TEMP_SPAN_C       250.0f    // +200°C - (-50°C) = 250°C span
#define PRTXI_OFFSET_C          -50.0f    // 4mA = -50°C
#define PRTXI_MIN_VALID_UA      3000      // Below this = sensor disconnected (~3mA)

// Sensor Health Detection
// When sensor is disconnected, modbus reads 0 mV
// Valid sensor at 0 PSI outputs ~307-308 mV after voltage divider (500mV / 1.4545 = 344mV theoretical)
// Use threshold below which we consider sensor disconnected
#define SENSOR_MIN_VALID_MV     100     // Below this = sensor disconnected
#define MODBUS_ERROR_THRESHOLD  3       // Mark invalid after this many consecutive errors

// Modbus State
static bool g_modbus_initialized = false;
static uint32_t g_modbus_last_read_ms = 0;
static uint32_t g_modbus_success_count = 0;
static uint32_t g_modbus_error_count = 0;
static uint16_t g_modbus_channel_values[8] = {0};

// Sensor and communication state tracking for logging
static bool g_modbus_comm_ok = false;           // Is modbus communication working?
static bool g_sensor_ch1_connected = false;     // Is CH1 (oil pressure) sensor connected?
static bool g_sensor_ch2_connected = false;     // Is CH2 (oil temp PRTXI) sensor connected?
static bool g_sensor_ch3_connected = false;     // Is CH3 (trans temp PRTXI) sensor connected?
static bool g_sensor_ch4_connected = false;     // Is CH4 (steer temp PRTXI) sensor connected?
static bool g_sensor_ch5_connected = false;     // Is CH5 (diff temp PRTXI) sensor connected?

// Hysteresis for stable display (prevents jumping between adjacent values)
#define TEMP_HYSTERESIS_C   0.3f                // Only update if change > 0.3°C
static float g_last_oil_temp_c = -999.0f;       // Last stable oil temp reading
static float g_last_trans_temp_c = -999.0f;     // Last stable trans temp reading
static float g_last_steer_temp_c = -999.0f;     // Last stable steer temp reading
static float g_last_diff_temp_c = -999.0f;      // Last stable diff temp reading

//-----------------------------------------------------------------------------
// modbusCRC16() - Calculate CRC-16 checksum for Modbus RTU
//-----------------------------------------------------------------------------
// Modbus uses a specific CRC-16 polynomial (0xA001, which is 0x8005 bit-reversed).
// The CRC is transmitted LSB first (little-endian).
//
// Algorithm:
//   1. Initialize CRC to 0xFFFF
//   2. For each byte: XOR into low byte of CRC
//   3. For each of 8 bits:
//      - If LSB is 1: shift right, XOR with 0xA001
//      - If LSB is 0: just shift right
//   4. Return final CRC (low byte first when transmitting)
//
static uint16_t modbusCRC16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;                    // Start with all 1s
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];             // XOR byte into CRC
        for (uint8_t j = 0; j < 8; j++) {     // Process each bit
            if (crc & 0x0001) {               // If LSB is 1
                crc >>= 1;                    // Shift right
                crc ^= 0xA001;                // XOR with polynomial
            }
            else {
                crc >>= 1;                    // Just shift right
            }
        }
    }
    return crc;  // Returns little-endian (low byte first)
}

//-----------------------------------------------------------------------------
// modbusTransaction() - Send request and receive response over RS485
//-----------------------------------------------------------------------------
// RS485 is half-duplex, so we must:
//   1. Clear any stale RX data
//   2. Send the request
//   3. Wait for TX to complete (flush) before listening
//   4. Receive response with timeout
//
// Frame detection: Modbus RTU uses silence (3.5 char times) to mark frame
// boundaries. At 9600 baud, that's ~4ms. We use 10ms inter-byte timeout
// to detect end of frame.
//
// Parameters:
//   request       - Buffer containing request bytes to send
//   requestLen    - Number of bytes in request
//   response      - Buffer to store received response
//   maxResponseLen- Maximum bytes to receive
//
// Returns: Number of bytes received (0 if timeout/no response)
//
static int modbusTransaction(uint8_t* request, uint8_t requestLen, uint8_t* response, uint8_t maxResponseLen) {
    // 1. Clear any stale data in receive buffer
    while (RS485_SERIAL.available()) RS485_SERIAL.read();
    // 2. Send the request
    RS485_SERIAL.write(request, requestLen);
    // Timeout-based receive (wait for bytes, don't just peek once)
    RS485_SERIAL.flush();  // Wait for TX complete (important for half-duplex!)
    // 3. Wait for response with timeout
    uint32_t t0 = millis();
    uint32_t lastByteMs = t0;
    int got = 0;
    const uint32_t timeoutMs = 200;  // Overall timeout
    const uint32_t interByteTimeout = 10;  // Gap after last byte = end of frame
    
    while ((millis() - t0) < timeoutMs && got < maxResponseLen) {
        int avail = RS485_SERIAL.available();
        if (avail > 0) {
            // Read available bytes
            int take = min(avail, (int)(maxResponseLen - got));
            got += RS485_SERIAL.readBytes(response + got, take);
            lastByteMs = millis();
        }
        else {
            // After receiving starts, a gap means frame is complete
            if (got > 0 && (millis() - lastByteMs) > interByteTimeout) {
                break;  // Frame complete
            }
            delay(1);  // Small yield while waiting
        }
    }

    return got;  // Number of bytes received
}

//-----------------------------------------------------------------------------
// modbusReadInputRegisters() - Read input registers from Modbus slave
//-----------------------------------------------------------------------------
// Builds and sends a Function 0x04 request, validates response.
//
// Request frame (8 bytes):
//   [0] Slave address
//   [1] Function code (0x04)
//   [2] Start register high byte
//   [3] Start register low byte  
//   [4] Quantity high byte
//   [5] Quantity low byte
//   [6] CRC low byte
//   [7] CRC high byte
//
// Expected response frame:
//   [0] Slave address (echo)
//   [1] Function code (echo)
//   [2] Byte count (2 * numRegs)
//   [3..N] Register data (high byte first per register)
//   [N+1, N+2] CRC
//
// Validation steps:
//   1. Check received length >= expected
//   2. Verify slave address matches
//   3. Verify function code matches
//   4. Verify byte count matches
//   5. Verify CRC matches calculated
//
static bool modbusReadInputRegisters(uint8_t slaveAddr, uint16_t startReg, 
                                     uint16_t numRegs, uint16_t* values) {
    uint8_t request[8];
    request[0] = slaveAddr;
    request[1] = 0x04;  // Function code: Read Input Registers
    request[2] = (startReg >> 8) & 0xFF;
    request[3] = startReg & 0xFF;
    request[4] = (numRegs >> 8) & 0xFF;
    request[5] = numRegs & 0xFF;
    
    uint16_t crc = modbusCRC16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    
    uint8_t expectedLen = 3 + (numRegs * 2) + 2;
    uint8_t response[64];
    
    int received = modbusTransaction(request, 8, response, expectedLen + 5);
    
    // Debug: log raw response on failures (periodically)
    static uint32_t lastDebugLog = 0;
    if (received < expectedLen && (millis() - lastDebugLog > 5000)) {
        lastDebugLog = millis();
        Serial.printf("[MODBUS DEBUG] Got %d bytes (expected %d): ", received, expectedLen);
        for (int i = 0; i < received && i < 20; i++) {
            Serial.printf("%02X ", response[i]);
        }
        Serial.println();
    }
    
    if (received < expectedLen) {
        return false;
    }
    if (response[0] != slaveAddr || response[1] != 0x04) {
        return false;
    }
    
    uint8_t byteCount = response[2];
    if (byteCount != numRegs * 2) {
        return false;
    }
    
    uint16_t receivedCRC = response[3 + byteCount] | (response[4 + byteCount] << 8);
    uint16_t calculatedCRC = modbusCRC16(response, 3 + byteCount);
    if (receivedCRC != calculatedCRC) {
        return false;
    }
    
    for (int i = 0; i < numRegs; i++) {
        values[i] = (response[3 + i*2] << 8) | response[4 + i*2];
    }
    return true;
}

//-----------------------------------------------------------------------------
// modbusWriteHoldingRegister() - Write a single holding register
//-----------------------------------------------------------------------------
// Modbus Function Code 0x06: Write Single Register
// Used to configure Waveshare channel modes (voltage vs current input)
//
// Request frame (8 bytes):
//   [0] Slave address
//   [1] Function code (0x06)
//   [2] Register address high byte
//   [3] Register address low byte
//   [4] Value high byte
//   [5] Value low byte
//   [6] CRC low byte
//   [7] CRC high byte
//
// Response: Echo of request (success) or exception response (failure)
//
static bool modbusWriteHoldingRegister(uint8_t slaveAddr, uint16_t regAddr, uint16_t value) {
    uint8_t request[8];
    request[0] = slaveAddr;
    request[1] = 0x06;  // Function code: Write Single Register
    request[2] = (regAddr >> 8) & 0xFF;
    request[3] = regAddr & 0xFF;
    request[4] = (value >> 8) & 0xFF;
    request[5] = value & 0xFF;
    
    uint16_t crc = modbusCRC16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    
    uint8_t response[8];
    int received = modbusTransaction(request, 8, response, 8);
    
    // Success = echo of request (same first 6 bytes)
    if (received >= 8 && 
        response[0] == slaveAddr && 
        response[1] == 0x06 &&
        response[2] == request[2] &&
        response[3] == request[3] &&
        response[4] == request[4] &&
        response[5] == request[5]) {
        return true;
    }
    
    // Check for exception response (function code with high bit set)
    if (received >= 5 && response[1] == 0x86) {
        Serial.printf("[MODBUS] Write exception: code 0x%02X\n", response[2]);
    }
    
    return false;
}

//-----------------------------------------------------------------------------
// modbusReadHoldingRegister() - Read a single holding register
//-----------------------------------------------------------------------------
// Modbus Function Code 0x03: Read Holding Registers
// Used to read configuration values from Waveshare module
//
static bool modbusReadHoldingRegister(uint8_t slaveAddr, uint16_t regAddr, uint16_t* value) {
    uint8_t request[8];
    request[0] = slaveAddr;
    request[1] = 0x03;  // Function code: Read Holding Registers
    request[2] = (regAddr >> 8) & 0xFF;
    request[3] = regAddr & 0xFF;
    request[4] = 0x00;  // Number of registers high byte
    request[5] = 0x01;  // Number of registers low byte (1 register)
    
    uint16_t crc = modbusCRC16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    
    uint8_t response[16];
    int received = modbusTransaction(request, 8, response, 16);
    
    // Expected response: slaveAddr, 0x03, byteCount(2), dataHi, dataLo, crcLo, crcHi
    if (received >= 7 && 
        response[0] == slaveAddr && 
        response[1] == 0x03 &&
        response[2] == 0x02) {  // 2 bytes of data
        *value = (response[3] << 8) | response[4];
        return true;
    }
    
    // Check for exception response
    if (received >= 5 && response[1] == 0x83) {
        Serial.printf("[MODBUS] Read exception: code 0x%02X\n", response[2]);
    }
    
    return false;
}

//-----------------------------------------------------------------------------
// convertToPSI() - Convert Modbus mV reading to PSI
//-----------------------------------------------------------------------------
// Sensor: PX3AN2BH150PSAAX (0-150 PSI, 0.5V-4.5V output)
//
// Wiring includes a voltage divider (10kΩ / 22kΩ) to scale 5V sensor
// output to safe 3.3V range for Modbus module input.
//
//   Actual sensor output: 500mV (0 PSI) to 4500mV (150 PSI)
//   After divider (0.6875): 344mV (0 PSI) to 3094mV (150 PSI)
//   Modbus reads: ~307mV at 0 PSI (close to theoretical 344mV)
//
// Conversion formula:
//   1. Multiply by divider ratio inverse (1.4545) to get original mV
//   2. Subtract 500mV offset (sensor outputs 500mV at 0 PSI)
//   3. Multiply by scale (150 PSI / 4000 mV = 0.0375)
//
static float convertToPSI(uint16_t modbus_mV) {
    float raw_mV = (float)modbus_mV * PRESSURE_DIVIDER_RATIO;
    float psi = (raw_mV - PRESSURE_OFFSET_MV) * PRESSURE_SCALE;
    if (psi < 0.0f) psi = 0.0f;
    if (psi > 150.0f) psi = 150.0f;
    return psi;
}

//-----------------------------------------------------------------------------
// convertToTempC() - Convert Modbus µA reading to Temperature (Celsius)
//-----------------------------------------------------------------------------
// Sensor: PRTXI-1/2N-1/4-4-IO RTD Temperature Transmitter (4-20mA output)
//
// Waveshare module configured to Mode 3 (4-20mA) via register 0x1001.
// With CH2 jumper ON, module returns microamps (µA) directly!
//
// Simple linear conversion: µA → °C
//   temp_C = ((µA - 4000) / 16000) * 250 + (-50)
//
// Expected readings at known temperatures:
//   4000 µA  (4.00 mA)  = -50°C  (sensor min)
//   8800 µA  (8.80 mA)  =  25°C  (room temp)
//   12000 µA (12.00 mA) =  75°C  (midpoint)
//   20000 µA (20.00 mA) = +200°C (sensor max)
//
// Example: At room temp 25°C, expect ~8800 µA
//          ((8800 - 4000) / 16000) * 250 - 50 = 25°C ✓
//
static float convertToTempC(uint16_t modbus_uA) {
    // Direct linear conversion: µA → °C
    // +5°C calibration offset for conservative reading on metal surfaces
    // (Ice water test confirmed sensor accuracy; offset is for peace of mind)
    float temp_c = ((float)(modbus_uA - PRTXI_MIN_CURRENT_UA) / (float)PRTXI_CURRENT_SPAN_UA) 
                   * PRTXI_TEMP_SPAN_C + PRTXI_OFFSET_C + 5.0f;
    
    // Clamp to valid sensor range (offset means we can read up to 205°C display)
    if (temp_c < -45.0f) temp_c = -45.0f;
    if (temp_c > 205.0f) temp_c = 205.0f;
    return temp_c;
}

//-----------------------------------------------------------------------------
// initModbusSensors() - Initialize RS485 and test communication
//-----------------------------------------------------------------------------
// Called once at startup. Configures Serial1 for RS485 communication
// and sends a test request to verify the Modbus module is responding.
//
// The test request reads 1 register (CH1) and checks for valid response.
// This confirms wiring is correct before entering normal polling loop.
//
// Hardware note: Waveshare ESP32-S3-Touch-LCD-7 uses SP3485 transceiver
// with automatic direction control - no manual DE/RE pin toggling needed.
//
void initModbusSensors() {
    Serial.println("[MODBUS] Initializing RS485...");
    Serial.printf("[MODBUS] Config: RX=%d, TX=%d, Baud=%d\n", 
                  RS485_RX_PIN, RS485_TX_PIN, RS485_BAUD);
    
    RS485_SERIAL.begin(RS485_BAUD, RS485_CONFIG, RS485_RX_PIN, RS485_TX_PIN);
    delay(100);
    
    // Clear any garbage
    while (RS485_SERIAL.available()) RS485_SERIAL.read();
    
    // Test: Send Modbus request and check for response
    uint8_t testReq[8] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x01, 0x31, 0xCA};
    uint8_t testResp[32];
    
    Serial.print("[MODBUS] TX: ");
    for (int i = 0; i < 8; i++) Serial.printf("%02X ", testReq[i]);
    Serial.println();
    
    RS485_SERIAL.write(testReq, 8);
    RS485_SERIAL.flush();  // Wait for TX complete
    
    // Wait for response (100ms works reliably)
    delay(100);
    
    int avail = RS485_SERIAL.available();
    Serial.printf("[MODBUS] RX buffer: %d bytes\n", avail);
    
    int rxCount = 0;
    while (RS485_SERIAL.available() && rxCount < 32) {
        testResp[rxCount++] = RS485_SERIAL.read();
    }
    
    if (rxCount > 0) {
        Serial.printf("[MODBUS] RX: ");
        for (int i = 0; i < rxCount; i++) {
            Serial.printf("%02X ", testResp[i]);
        }
        Serial.println();
        
        // Check for valid Modbus response (01 04 02 XX XX CRC CRC)
        if (rxCount >= 7 && testResp[0] == 0x01 && testResp[1] == 0x04 && testResp[2] == 0x02) {
            uint16_t value = (testResp[3] << 8) | testResp[4];
            Serial.printf("[MODBUS] SUCCESS! CH1 = %d mV\n", value);
            g_modbus_initialized = true;
            g_modbus_comm_ok = true;
            
            // Set initial sensor state
            g_sensor_ch1_connected = (value >= SENSOR_MIN_VALID_MV);
            if (!g_sensor_ch1_connected) {
                Serial.printf("[MODBUS] CH1: Sensor not connected at startup (%d mV)\n", value);
            }
            
            // ========== CONFIGURE TEMPERATURE CHANNELS FOR 4-20mA ==========
            // CH2-CH5 all use PRTXI temperature sensors (4-20mA output)
            // This writes Mode 3 (4-20mA) to each channel's mode register
            Serial.println("[MODBUS] Configuring temperature channels for 4-20mA mode...");
            
            // Channel configuration array: {register, name}
            struct ChannelConfig {
                uint16_t reg;
                const char* name;
            };
            ChannelConfig tempChannels[] = {
                {WAVESHARE_CH2_MODE_REG, "CH2 (Oil Temp)"},
                {WAVESHARE_CH3_MODE_REG, "CH3 (Trans Temp)"},
                {WAVESHARE_CH4_MODE_REG, "CH4 (Steer Temp)"},
                {WAVESHARE_CH5_MODE_REG, "CH5 (Diff Temp)"}
            };
            
            for (int i = 0; i < 4; i++) {
                uint16_t currentMode = 0;
                if (modbusReadHoldingRegister(MODBUS_SLAVE_ADDR, tempChannels[i].reg, &currentMode)) {
                    Serial.printf("[MODBUS] %s current mode: %d\n", tempChannels[i].name, currentMode);
                }
                
                if (modbusWriteHoldingRegister(MODBUS_SLAVE_ADDR, tempChannels[i].reg, WAVESHARE_MODE_4_20MA)) {
                    Serial.printf("[MODBUS] %s configured for 4-20mA (Mode 3)\n", tempChannels[i].name);
                } else {
                    Serial.printf("[MODBUS] WARNING: Failed to configure %s!\n", tempChannels[i].name);
                    Serial.println("[MODBUS]   Check: Is jumper set to 'I' or 'mA' position?");
                }
                delay(10);  // Small delay between writes
            }
            // ==============================================================
        } else {
            Serial.println("[MODBUS] Got data but unexpected format");
            g_modbus_initialized = false;
            g_modbus_comm_ok = false;
        }
    } else {
        Serial.println("[MODBUS] FAILED - no response");
        Serial.println("[MODBUS] Check: A<->A, B<->B wiring, module power");
        g_modbus_initialized = false;
        g_modbus_comm_ok = false;
    }
}

//-----------------------------------------------------------------------------
// readModbusSensors() - Poll sensors and update vehicle data
//-----------------------------------------------------------------------------
// Called from main loop. Reads sensor values at regular intervals
// (MODBUS_READ_INTERVAL_MS = 100ms = 10 Hz polling rate).
//
// State machine handles:
//   - Communication failures (retry, then mark invalid after threshold)
//   - Sensor disconnection (detected by mV < 100 threshold)
//   - Reconnection (logs state changes for debugging)
//
// Updates g_vehicle_data struct which is read by UI update code.
// Invalid readings cause UI to display "---" instead of stale values.
//
void readModbusSensors() {
    if (!g_modbus_initialized) {
        static uint32_t lastRetry = 0;
        if (millis() - lastRetry > 5000) {
            lastRetry = millis();
            initModbusSensors();
        }
        return;
    }
    
    uint32_t now = millis();
    if ((now - g_modbus_last_read_ms) < MODBUS_READ_INTERVAL_MS) {
        return;
    }
    g_modbus_last_read_ms = now;
    
    bool success = false;
    for (int retry = 0; retry < MODBUS_RETRY_COUNT && !success; retry++) {
        success = modbusReadInputRegisters(MODBUS_SLAVE_ADDR, 0, MODBUS_NUM_CHANNELS, 
                                           g_modbus_channel_values);
        if (!success && retry < MODBUS_RETRY_COUNT - 1) delay(10);
    }
    
    if (success) {
        // Modbus communication restored?
        if (!g_modbus_comm_ok) {
            Serial.println("[MODBUS] Communication restored");
            g_modbus_comm_ok = true;
        }
        
        // Channel 1: Oil Pressure
        uint16_t oil_press_mV = g_modbus_channel_values[MODBUS_CH_OIL_PRESSURE];
        bool sensor_connected = (oil_press_mV >= SENSOR_MIN_VALID_MV);
        
        // Log sensor state changes
        if (sensor_connected != g_sensor_ch1_connected) {
            if (sensor_connected) {
                Serial.printf("[MODBUS] CH1: Sensor CONNECTED (%d mV)\n", oil_press_mV);
            } else {
                Serial.printf("[MODBUS] CH1: Sensor DISCONNECTED (%d mV < %d mV threshold)\n", 
                             oil_press_mV, SENSOR_MIN_VALID_MV);
            }
            g_sensor_ch1_connected = sensor_connected;
        }
        
        if (sensor_connected) {
            // Sensor connected and reading valid
            float oil_press_psi = convertToPSI(oil_press_mV);
            
            g_vehicle_data.oil_pressure_psi = (int)(oil_press_psi + 0.5f);
            g_vehicle_data.oil_pressure_valid = true;
            g_vehicle_data.has_received_data = true;
        } else {
            // Sensor disconnected
            g_vehicle_data.oil_pressure_valid = false;
        }
        
        // Channel 2: Oil Temperature (PRTXI 4-20mA transmitter)
        // Waveshare configured to Mode 3 (4-20mA) - returns µA directly
        uint16_t oil_temp_uA = g_modbus_channel_values[MODBUS_CH_OIL_TEMP];
        bool temp_sensor_connected = (oil_temp_uA >= PRTXI_MIN_VALID_UA);
        
        // Log sensor state changes
        if (temp_sensor_connected != g_sensor_ch2_connected) {
            if (temp_sensor_connected) {
                Serial.printf("[MODBUS] CH2: PRTXI Sensor CONNECTED (%d uA = %.2f mA)\n", 
                             oil_temp_uA, oil_temp_uA / 1000.0f);
            } else {
                Serial.printf("[MODBUS] CH2: PRTXI Sensor DISCONNECTED (%d uA < %d uA threshold)\n", 
                             oil_temp_uA, PRTXI_MIN_VALID_UA);
            }
            g_sensor_ch2_connected = temp_sensor_connected;
        }
        
        if (temp_sensor_connected) {
            // Sensor connected and reading valid
            float oil_temp_c = convertToTempC(oil_temp_uA);
            
            // Apply hysteresis to prevent display jumping between adjacent values
            // Only update if this is first reading OR change exceeds threshold
            if (g_last_oil_temp_c < -900.0f || fabsf(oil_temp_c - g_last_oil_temp_c) >= TEMP_HYSTERESIS_C) {
                g_last_oil_temp_c = oil_temp_c;  // Update stable reading
            }
            // Use the stable (hysteresis-filtered) value for display
            float stable_temp_c = g_last_oil_temp_c;
            float oil_temp_f = celsiusToFahrenheit(stable_temp_c);
            
            // Only set pan temperature - cooled gauge will show "---"
            g_vehicle_data.oil_temp_pan_f = (int)(oil_temp_f + 0.5f);
            // NOTE: oil_temp_cooled_f intentionally NOT set (single sensor)
            g_vehicle_data.oil_temp_valid = true;
            g_vehicle_data.has_received_data = true;
        } else {
            // Sensor disconnected
            g_vehicle_data.oil_temp_valid = false;
        }
        
        // Channel 3: Transmission Temperature (PRTXI 4-20mA transmitter)
        uint16_t trans_temp_uA = g_modbus_channel_values[MODBUS_CH_TRANS_TEMP];
        bool trans_sensor_connected = (trans_temp_uA >= PRTXI_MIN_VALID_UA);
        
        // Log sensor state changes
        if (trans_sensor_connected != g_sensor_ch3_connected) {
            if (trans_sensor_connected) {
                Serial.printf("[MODBUS] CH3: Trans Temp Sensor CONNECTED (%d uA = %.2f mA)\n", 
                             trans_temp_uA, trans_temp_uA / 1000.0f);
            } else {
                Serial.printf("[MODBUS] CH3: Trans Temp Sensor DISCONNECTED (%d uA < %d uA)\n", 
                             trans_temp_uA, PRTXI_MIN_VALID_UA);
            }
            g_sensor_ch3_connected = trans_sensor_connected;
        }
        
        if (trans_sensor_connected) {
            float trans_temp_c = convertToTempC(trans_temp_uA);
            
            // Apply hysteresis
            if (g_last_trans_temp_c < -900.0f || fabsf(trans_temp_c - g_last_trans_temp_c) >= TEMP_HYSTERESIS_C) {
                g_last_trans_temp_c = trans_temp_c;
            }
            float stable_temp_c = g_last_trans_temp_c;
            float trans_temp_f = celsiusToFahrenheit(stable_temp_c);
            
            g_vehicle_data.trans_temp_hot_f = (int)(trans_temp_f + 0.5f);
            g_vehicle_data.trans_temp_valid = true;
            g_vehicle_data.has_received_data = true;
        } else {
            g_vehicle_data.trans_temp_valid = false;
        }
        
        // Channel 4: Power Steering Temperature (PRTXI 4-20mA transmitter)
        uint16_t steer_temp_uA = g_modbus_channel_values[MODBUS_CH_STEER_TEMP];
        bool steer_sensor_connected = (steer_temp_uA >= PRTXI_MIN_VALID_UA);
        
        // Log sensor state changes
        if (steer_sensor_connected != g_sensor_ch4_connected) {
            if (steer_sensor_connected) {
                Serial.printf("[MODBUS] CH4: Steer Temp Sensor CONNECTED (%d uA = %.2f mA)\n", 
                             steer_temp_uA, steer_temp_uA / 1000.0f);
            } else {
                Serial.printf("[MODBUS] CH4: Steer Temp Sensor DISCONNECTED (%d uA < %d uA)\n", 
                             steer_temp_uA, PRTXI_MIN_VALID_UA);
            }
            g_sensor_ch4_connected = steer_sensor_connected;
        }
        
        if (steer_sensor_connected) {
            float steer_temp_c = convertToTempC(steer_temp_uA);
            
            // Apply hysteresis
            if (g_last_steer_temp_c < -900.0f || fabsf(steer_temp_c - g_last_steer_temp_c) >= TEMP_HYSTERESIS_C) {
                g_last_steer_temp_c = steer_temp_c;
            }
            float stable_temp_c = g_last_steer_temp_c;
            float steer_temp_f = celsiusToFahrenheit(stable_temp_c);
            
            g_vehicle_data.steer_temp_hot_f = (int)(steer_temp_f + 0.5f);
            g_vehicle_data.steer_temp_valid = true;
            g_vehicle_data.has_received_data = true;
        } else {
            g_vehicle_data.steer_temp_valid = false;
        }
        
        // Channel 5: Differential Temperature (PRTXI 4-20mA transmitter)
        uint16_t diff_temp_uA = g_modbus_channel_values[MODBUS_CH_DIFF_TEMP];
        bool diff_sensor_connected = (diff_temp_uA >= PRTXI_MIN_VALID_UA);
        
        // Log sensor state changes
        if (diff_sensor_connected != g_sensor_ch5_connected) {
            if (diff_sensor_connected) {
                Serial.printf("[MODBUS] CH5: Diff Temp Sensor CONNECTED (%d uA = %.2f mA)\n", 
                             diff_temp_uA, diff_temp_uA / 1000.0f);
            } else {
                Serial.printf("[MODBUS] CH5: Diff Temp Sensor DISCONNECTED (%d uA < %d uA)\n", 
                             diff_temp_uA, PRTXI_MIN_VALID_UA);
            }
            g_sensor_ch5_connected = diff_sensor_connected;
        }
        
        if (diff_sensor_connected) {
            float diff_temp_c = convertToTempC(diff_temp_uA);
            
            // Apply hysteresis
            if (g_last_diff_temp_c < -900.0f || fabsf(diff_temp_c - g_last_diff_temp_c) >= TEMP_HYSTERESIS_C) {
                g_last_diff_temp_c = diff_temp_c;
            }
            float stable_temp_c = g_last_diff_temp_c;
            float diff_temp_f = celsiusToFahrenheit(stable_temp_c);
            
            g_vehicle_data.diff_temp_hot_f = (int)(diff_temp_f + 0.5f);
            g_vehicle_data.diff_temp_valid = true;
            g_vehicle_data.has_received_data = true;
        } else {
            g_vehicle_data.diff_temp_valid = false;
        }
        
        // ========== UNIFIED MODBUS LOGGING (all channels on one line, every 1 second) ==========
        static uint32_t unifiedLogCounter = 0;
        if (++unifiedLogCounter >= 10) {  // Every ~1 second (10 × 100ms polling)
            unifiedLogCounter = 0;
            
            // Build single-line log with all channels
            char logBuf[256];
            int pos = 0;
            
            // CH1: Oil Pressure
            if (sensor_connected) {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, "Oil-Press:%.0fPSI", convertToPSI(oil_press_mV));
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, "Oil-Press:n/a");
            }
            
            // CH2: Oil Temp
            if (temp_sensor_connected) {
                bool crit = (g_vehicle_data.oil_temp_pan_f > 260);
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Oil-Temp:%.0f°F%s", 
                               celsiusToFahrenheit(g_last_oil_temp_c), crit ? "!" : "");
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Oil-Temp:n/a");
            }
            
            // CH3: Trans Temp
            if (trans_sensor_connected) {
                bool crit = (g_vehicle_data.trans_temp_hot_f > 230);
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Trans:%.0f°F%s", 
                               celsiusToFahrenheit(g_last_trans_temp_c), crit ? "!" : "");
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Trans:n/a");
            }
            
            // CH4: Steer Temp
            if (steer_sensor_connected) {
                bool crit = (g_vehicle_data.steer_temp_hot_f > 230);
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Steer:%.0f°F%s", 
                               celsiusToFahrenheit(g_last_steer_temp_c), crit ? "!" : "");
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Steer:n/a");
            }
            
            // CH5: Diff Temp
            if (diff_sensor_connected) {
                bool crit = (g_vehicle_data.diff_temp_hot_f > 270);
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Diff:%.0f°F%s", 
                               celsiusToFahrenheit(g_last_diff_temp_c), crit ? "!" : "");
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Diff:n/a");
            }
            
            Serial.printf("[MODBUS] %s\n", logBuf);
        }
        
        // Reset error count if we got here (communication succeeded)
        g_modbus_success_count++;
        g_modbus_error_count = 0;
    } else {
        // Modbus communication failed
        g_modbus_error_count++;
        
        // Mark invalid after threshold errors (fast response)
        if (g_modbus_error_count >= MODBUS_ERROR_THRESHOLD) {
            // Log communication loss once
            if (g_modbus_comm_ok) {
                Serial.println("[MODBUS] Communication LOST - marking all sensors invalid");
                g_modbus_comm_ok = false;
            }
            g_vehicle_data.oil_pressure_valid = false;
            g_vehicle_data.oil_temp_valid = false;
            g_vehicle_data.trans_temp_valid = false;
            g_vehicle_data.steer_temp_valid = false;
            g_vehicle_data.diff_temp_valid = false;
        }
        
        static uint32_t lastErrorLog = 0;
        if (now - lastErrorLog > 2000) {
            lastErrorLog = now;
            Serial.printf("[MODBUS] Read failed - errors: %lu\n", g_modbus_error_count);
        }
    }
}

#endif // ENABLE_MODBUS_SENSORS

//-----------------------------------------------------------------

// CH422 IO Expander
#define CH422_ADDR_SYSTEM 0x24
#define CH422_ADDR_IOWR   0x38
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQ_HZ 400000  // 400000 - max value
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

// Panel critical state tracking (file-scoped for reset capability)
static bool g_oil_press_panel_was_critical = false;
static bool g_oil_temp_panel_was_critical = false;
static bool g_water_temp_panel_was_critical = false;
static bool g_trans_temp_panel_was_critical = false;
static bool g_steer_temp_panel_was_critical = false;
static bool g_diff_temp_panel_was_critical = false;
static bool g_fuel_panel_was_critical = false;

// Flag to force panel opacity reset on next updateGauges() call
static bool g_force_panel_reset = false;

void resetTapPanelOpacity() {
    g_force_panel_reset = true;
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

// File Browser Module - SD card file viewer
// Entry: Hold BOOT button 5s during operation
// Exit: Navigate back from root, or hold BOOT 5s again
#if ENABLE_FILE_BROWSER
#include "file_browser.h"
#endif

//OIL PRESS
extern lv_obj_t* ui_OIL_PRESS_Bar;
extern lv_obj_t* ui_OIL_PRESS_CHART;
extern lv_obj_t* ui_OIL_PRESS_Value;
extern lv_obj_t* ui_OIL_PRESS_VALUE_CRITICAL_Label;
extern lv_obj_t* ui_OIL_PRESS_Value_Tap_Panel;

//OIL TEMP
extern lv_obj_t* ui_OIL_TEMP_Bar;
extern lv_obj_t* ui_OIL_TEMP_CHART;
extern lv_obj_t* ui_OIL_TEMP_Value;
extern lv_obj_t* ui_OIL_TEMP_VALUE_CRITICAL_Label;
extern lv_obj_t* ui_OIL_TEMP_Value_Tap_Panel;

//WATER TEMP
extern lv_obj_t* ui_W_TEMP_Bar;
extern lv_obj_t* ui_W_TEMP_CHART;
extern lv_obj_t* ui_W_TEMP_Value;
extern lv_obj_t* ui_W_TEMP_VALUE_CRITICAL_Label;
extern lv_obj_t* ui_W_TEMP_Value_Tap_Panel;

//TRAN TEMP
extern lv_obj_t* ui_TRAN_TEMP_Bar;
extern lv_obj_t* ui_TRAN_TEMP_CHART;
extern lv_obj_t* ui_TRAN_TEMP_Value;
extern lv_obj_t* ui_TRAN_TEMP_VALUE_CRITICAL_Label;
extern lv_obj_t* ui_TRAN_TEMP_Value_Tap_Panel;

//STEER TEMP
extern lv_obj_t* ui_STEER_TEMP_Bar;
extern lv_obj_t* ui_STEER_TEMP_CHART;
extern lv_obj_t* ui_STEER_TEMP_Value;
extern lv_obj_t* ui_STEER_TEMP_VALUE_CRITICAL_Label;
extern lv_obj_t* ui_STEER_TEMP_Value_Tap_Panel;

//DIFF TEMP
extern lv_obj_t* ui_DIFF_TEMP_Bar;
extern lv_obj_t* ui_DIFF_TEMP_CHART;
extern lv_obj_t* ui_DIFF_TEMP_Value;
extern lv_obj_t* ui_DIFF_TEMP_VALUE_CRITICAL_Label;
extern lv_obj_t* ui_DIFF_TEMP_Value_Tap_Panel;

//FUEL TRUST
extern lv_obj_t* ui_FUEL_TRUST_Bar;
extern lv_obj_t* ui_FUEL_TRUST_CHART;
extern lv_obj_t* ui_FUEL_TRUST_Value;
extern lv_obj_t* ui_FUEL_TRUST_VALUE_CRITICAL_Label;
extern lv_obj_t* ui_FUEL_TRUST_Value_Tap_Panel;

#pragma endregion UI Objects

//-----------------------------------------------------------------
// LIGHTWEIGHT BARS - Simple rectangles that overlay SquareLine bars
// Much cheaper than lv_bar widgets (no gradients, no indicator styling)
//-----------------------------------------------------------------
#if ENABLE_LIGHTWEIGHT_BARS

// Lightweight bar configuration
struct LightweightBar {
    lv_obj_t* obj;           // The simple rectangle object
    lv_obj_t* parent;        // Parent panel (ui_OIL_PRESS, etc.)
    int16_t min_val;         // Minimum value
    int16_t max_val;         // Maximum value
    int16_t max_width;       // Maximum width in pixels
    int16_t last_width;      // Last rendered width (to avoid redundant updates)
};

// 7 lightweight bars
static LightweightBar g_light_bars[7];

// Only update bar if width changed by more than this many pixels
#define LIGHT_BAR_WIDTH_THRESHOLD 4

// Only update bars every N frames (reduces CPU load)
#define LIGHT_BAR_FRAME_SKIP 3
static uint8_t g_light_bar_frame_counter = 0;

// Bar dimensions (from SquareLine)
#define LIGHT_BAR_WIDTH     331
#define LIGHT_BAR_HEIGHT    65
#define LIGHT_BAR_LEFT_MARGIN 309  // Left edge position from parent left

// Colors
#define LIGHT_BAR_BG_COLOR      0x32231E  // Dark brown background
#define LIGHT_BAR_COLOR         0xFF4500  // Orange (static - no color changes)

// Forward declaration
void initLightweightBars();
void updateLightweightBar(int index, float value);
bool shouldUpdateLightweightBars();

#endif // ENABLE_LIGHTWEIGHT_BARS

//-----------------------------------------------------------------

extern lv_obj_t* ui_Screen1;

// Display - Waveshare ESP32-S3 7" 800x480 RGB
// Using Waveshare recommended porch timings for stable sync
Arduino_ESP32RGBPanel* rgbpanel = new Arduino_ESP32RGBPanel(
    5,   // DE
    3,   // VSYNC
    46,  // HSYNC
    7,   // PCLK
    1, 2, 42, 41, 40,           // R3-R7
    39, 0, 45, 48, 47, 21,      // G2-G7
    14, 38, 18, 17, 10,         // B3-B7
    0,   // hsync_polarity
    40,  // hsync_front_porch (Waveshare recommended)
    48,  // hsync_pulse_width (Waveshare recommended)
    40,  // hsync_back_porch (Waveshare recommended)
    0,   // vsync_polarity
    13,  // vsync_front_porch (Waveshare recommended)
    3,   // vsync_pulse_width (Waveshare recommended)
    32,  // vsync_back_porch (Waveshare recommended)
    1,   // pclk_active_neg
    14000000,  // pixel clock
    true,      // auto_flush
    0,   // de_idle_high
    0    // pclk_idle_high
);
Arduino_RGB_Display* gfx = new Arduino_RGB_Display(800, 480, rgbpanel, 0, true);

//-----------------------------------------------------------------

// LVGL
#define LVGL_BUFFER_SIZE (800 * 30)  // 30 lines in internal DMA RAM (~48KB per buffer) - prevents PSRAM contention/tearing
static lv_display_t* disp;
static lv_indev_t* indev;
static uint8_t* disp_draw_buf1;
static uint8_t* disp_draw_buf2;

//-----------------------------------------------------------------

// GT911 Touch Controller
TAMC_GT911 touch = TAMC_GT911(I2C_SDA, I2C_SCL, TOUCH_INT_PIN, -1, 800, 480);

//=================================================================
// DUAL-CORE ARCHITECTURE
// Core 0: SD write task + Time sync
// Core 1: Main loop (LVGL rendering, data processing) + Touch polling
//=================================================================

// Touch state shared between Core 0 (producer) and Core 1 (consumer)
struct TouchState {
    volatile int16_t x;
    volatile int16_t y;
    volatile bool pressed;
    volatile bool valid;           // false if touch data is garbage
    volatile uint32_t timestamp;   // when this touch was recorded
};
static TouchState g_touch_state = {0, 0, false, false, 0};
static SemaphoreHandle_t g_touch_mutex = NULL;

// Task handle for touch (always needed)
static TaskHandle_t g_touch_task_handle = NULL;

// SD logging queue - main loop pushes data, Core 0 task writes to SD
#if ENABLE_SD_LOGGING
struct SDLogEntry {
    uint32_t timestamp_ms;
    float elapsed_s;
    float cpu_pct;
    bool demo_mode;
    VehicleData data;
};
#define SD_QUEUE_SIZE 16
static QueueHandle_t g_sd_queue = NULL;
static TaskHandle_t g_sd_task_handle = NULL;
#endif

//-----------------------------------------------------------------

// Counters and tracking
static uint32_t loop_count = 0;
static volatile uint32_t flush_count = 0;   // flushes per second (partial screen updates)
static volatile uint32_t frame_count = 0;   // REAL frames per second (complete screen refreshes)
static uint32_t update_count = 0;
static uint32_t cpu_busy_time = 0;

// Touch controller state
static uint32_t consecutive_invalid = 0;
static uint32_t last_touch_reset = 0;

// Utility box
static lv_obj_t* utility_box = NULL;
static lv_obj_t* g_dim_overlay = NULL;
static lv_timer_t* g_util_single_tap_timer = NULL;
// Individual labels for utility box (allows per-line coloring)
#if ENABLE_SD_LOGGING
static lv_obj_t* util_labels[9] = {NULL};  // FPS, CPU0, CPU1, SRAM, PSRAM, BRI, SD, TIME, DATE
#if ENABLE_FILE_BROWSER
static lv_obj_t* files_btn = NULL;         // FILES button at top of utility box
#endif
#define UTIL_IDX_FPS   0
#define UTIL_IDX_CPU0  1
#define UTIL_IDX_CPU1  2
#define UTIL_IDX_SRAM  3
#define UTIL_IDX_PSRAM 4
#define UTIL_IDX_BRI   5
#define UTIL_IDX_SD    6
#define UTIL_IDX_TIME  7
#define UTIL_IDX_DATE  8
#define UTIL_LABEL_COUNT 9
#else
static lv_obj_t* util_labels[6] = {NULL};  // FPS, CPU0, CPU1, SRAM, PSRAM, BRI
#if ENABLE_FILE_BROWSER
static lv_obj_t* files_btn = NULL;         // FILES button at top of utility box
#endif
#define UTIL_IDX_FPS   0
#define UTIL_IDX_CPU0  1
#define UTIL_IDX_CPU1  2
#define UTIL_IDX_SRAM  3
#define UTIL_IDX_PSRAM 4
#define UTIL_IDX_BRI   5
#define UTIL_LABEL_COUNT 6
#endif
static lv_obj_t* mode_indicator = NULL;  // Shows "DEMO" or "LIVE"
static bool utilities_visible = false;  // Start hidden, double-tap to reveal

// Per-core CPU measurement using idle hooks
static volatile uint32_t g_idle_count_core0 = 0;
static volatile uint32_t g_idle_count_core1 = 0;
static uint32_t g_last_idle0 = 0, g_last_idle1 = 0;

// Idle hooks - called when each core is idle
// Idle hooks - called when each core is idle (must return false to allow normal idle processing)
static bool idle_hook_core0() { g_idle_count_core0++; return false; }
static bool idle_hook_core1() { g_idle_count_core1++; return false; }

// Long press tracking for demo mode toggle
static uint32_t g_utility_press_start = 0;
static bool g_utility_long_press_triggered = false;
static bool g_utility_long_press_consumed = false;  // Prevents tap after long-press release
#define DEMO_MODE_TOGGLE_HOLD_MS 5000  // 5 seconds to toggle demo mode

//=================================================================
// SYSTEM STATUS TOAST - Shows "All systems checked" on startup
//=================================================================
static lv_obj_t* g_toast_obj = NULL;
static lv_timer_t* g_toast_timer = NULL;
static lv_timer_t* g_system_check_timer = NULL;
static lv_timer_t* g_system_monitor_timer = NULL;  // Background monitoring timer
#define TOAST_SUCCESS_MS        3000   // Green toast duration
#define TOAST_ERROR_MS         30000   // Red toast duration (30 seconds)
#define TOAST_RECOVERY_MS      15000   // Green recovery toast duration (15 seconds)
#define SYSTEM_CHECK_DELAY_MS   5000   // Delay after main screen loads to check systems
#define SYSTEM_MONITOR_INTERVAL_MS 10000  // Background monitoring interval (10 seconds)
#define TOAST_COLOR_SUCCESS   0x2E7D32 // Green
#define TOAST_COLOR_ERROR     0xB71C1C // Dark red

// Previous system status for change detection (background monitoring)
static struct {
    bool sd_card_ok;
    bool logs_writing_ok;
    bool rtc_ok;
    bool time_sync_ok;
    bool modbus_ok;
    bool oil_pressure_sensor_ok;
    bool oil_temp_sensor_ok;
    bool trans_temp_sensor_ok;
    bool steer_temp_sensor_ok;
    bool diff_temp_sensor_ok;
    bool initialized;  // Set true after first check
} g_prev_system_status = {true, true, true, true, true, true, true, true, true, true, false};

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

#define CHART_BUCKET_MS 5000    // crate a new bar every (ms)

#define CHART_POINTS 24
#define CHART_NO_DATA INT32_MIN  // Sentinel value for "no data" in history arrays
static int32_t oil_press_history[CHART_POINTS] = { 0 };
static int32_t oil_temp_history[CHART_POINTS] = { 0 };
static int32_t water_temp_history[CHART_POINTS] = { 0 };
static int32_t transmission_temp_history[CHART_POINTS] = { 0 };
static int32_t steering_temp_history[CHART_POINTS] = { 0 };
static int32_t differencial_temp_history[CHART_POINTS] = { 0 };
static int32_t fuel_trust_history[CHART_POINTS] = { 0 };

// Track which charts currently have critical values (for selective blink invalidation)
static bool g_chart_has_critical_oil_press = false;
static bool g_chart_has_critical_oil_temp = false;
static bool g_chart_has_critical_water_temp = false;
static bool g_chart_has_critical_trans_temp = false;
static bool g_chart_has_critical_steer_temp = false;
static bool g_chart_has_critical_diff_temp = false;
static bool g_chart_has_critical_fuel_trust = false;

// Critical bar blinking
static bool g_critical_blink_phase = false;
static uint32_t g_last_blink_toggle = 0;
#define CHART_BLINK_INTERVAL_MS 1000     // Chart bars blink rate (ms)
#define LABEL_BLINK_INTERVAL_MS 1000     // Critical labels blink rate (ms)

// Value Tap Panels from SquareLine Studio - used for tap detection AND critical background
// These are declared in ui_Screen1.h:
// - ui_OIL_PRESS_Value_Tap_Panel
// - ui_OIL_TEMP_Value_Tap_Panel  
// - ui_W_TEMP_Value_Tap_Panel
// - ui_TRAN_TEMP_Value_Tap_Panel
// - ui_STEER_TEMP_Value_Tap_Panel
// - ui_DIFF_TEMP_Value_Tap_Panel
// - ui_FUEL_TRUST_Value_Tap_Panel

//=================================================================
// UI RESET FUNCTIONS (defined after all variables are declared)
//=================================================================

// Reset all UI elements to default/empty state
void resetUIElements() {
#if ENABLE_BARS
    // Reset all bars to 0 (no animation for instant reset)
    if (ui_OIL_PRESS_Bar) lv_bar_set_value(ui_OIL_PRESS_Bar, 0, LV_ANIM_OFF);
    if (ui_OIL_TEMP_Bar) lv_bar_set_value(ui_OIL_TEMP_Bar, 0, LV_ANIM_OFF);
    if (ui_W_TEMP_Bar) lv_bar_set_value(ui_W_TEMP_Bar, 0, LV_ANIM_OFF);
    if (ui_TRAN_TEMP_Bar) lv_bar_set_value(ui_TRAN_TEMP_Bar, 0, LV_ANIM_OFF);
    if (ui_STEER_TEMP_Bar) lv_bar_set_value(ui_STEER_TEMP_Bar, 0, LV_ANIM_OFF);
    if (ui_DIFF_TEMP_Bar) lv_bar_set_value(ui_DIFF_TEMP_Bar, 0, LV_ANIM_OFF);
    if (ui_FUEL_TRUST_Bar) lv_bar_set_value(ui_FUEL_TRUST_Bar, 0, LV_ANIM_OFF);
#endif

    char buf[32];

    // Oil Pressure
    snprintf(buf, sizeof(buf), "--- %s", getPressureUnitStr(g_pressure_unit));
    if (ui_OIL_PRESS_Value) lv_label_set_text(ui_OIL_PRESS_Value, buf);

    // Oil Temp - uses g_oil_temp_unit
    const char* oilTempUnit = getTempUnitStr(g_oil_temp_unit);
    snprintf(buf, sizeof(buf), "---°%s", oilTempUnit);
    if (ui_OIL_TEMP_Value) lv_label_set_text(ui_OIL_TEMP_Value, buf);

    // Water Temp - uses g_water_temp_unit
    const char* waterTempUnit = getTempUnitStr(g_water_temp_unit);
    snprintf(buf, sizeof(buf), "---°%s", waterTempUnit);
    if (ui_W_TEMP_Value) lv_label_set_text(ui_W_TEMP_Value, buf);

    // Trans Temp - uses g_trans_temp_unit
    const char* transTempUnit = getTempUnitStr(g_trans_temp_unit);
    snprintf(buf, sizeof(buf), "---°%s", transTempUnit);
    if (ui_TRAN_TEMP_Value) lv_label_set_text(ui_TRAN_TEMP_Value, buf);

    // Steer Temp - uses g_steer_temp_unit
    const char* steerTempUnit = getTempUnitStr(g_steer_temp_unit);
    snprintf(buf, sizeof(buf), "---°%s", steerTempUnit);
    if (ui_STEER_TEMP_Value) lv_label_set_text(ui_STEER_TEMP_Value, buf);

    // Diff Temp - uses g_diff_temp_unit
    const char* diffTempUnit = getTempUnitStr(g_diff_temp_unit);
    snprintf(buf, sizeof(buf), "---°%s", diffTempUnit);
    if (ui_DIFF_TEMP_Value) lv_label_set_text(ui_DIFF_TEMP_Value, buf);

    if (ui_FUEL_TRUST_Value) lv_label_set_text(ui_FUEL_TRUST_Value, "--- %");

    // Reset oil pressure label styling to default
    if (ui_OIL_PRESS_Value) {
        lv_obj_set_style_text_color(ui_OIL_PRESS_Value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_OIL_PRESS_Value, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_OIL_PRESS_Value, 0, 0);
    }

    // Reset oil temp label styling to default
    if (ui_OIL_TEMP_Value) {
        lv_obj_set_style_text_color(ui_OIL_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_OIL_TEMP_Value, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_OIL_TEMP_Value, 0, 0);
    }

    // Reset water temp label styling to default
    if (ui_W_TEMP_Value) {
        lv_obj_set_style_text_color(ui_W_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_W_TEMP_Value, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_W_TEMP_Value, 0, 0);
    }

    // Reset trans temp label styling to default
    if (ui_TRAN_TEMP_Value) {
        lv_obj_set_style_text_color(ui_TRAN_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_TRAN_TEMP_Value, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_TRAN_TEMP_Value, 0, 0);
    }

    // Reset steer temp label styling to default
    if (ui_STEER_TEMP_Value) {
        lv_obj_set_style_text_color(ui_STEER_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_STEER_TEMP_Value, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_STEER_TEMP_Value, 0, 0);
    }

    // Reset diff temp label styling to default
    if (ui_DIFF_TEMP_Value) {
        lv_obj_set_style_text_color(ui_DIFF_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(ui_DIFF_TEMP_Value, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(ui_DIFF_TEMP_Value, 0, 0);
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
            // Just hide labels - no animations to delete (static styling now)
            lv_obj_set_style_text_opa(critical_labels[i], 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(critical_labels[i], 0, LV_PART_MAIN);
        }
    }

#if ENABLE_BARS
    // Reset all bar colors to default orange
    if (ui_OIL_PRESS_Bar) lv_obj_set_style_bg_color(ui_OIL_PRESS_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_OIL_TEMP_Bar) lv_obj_set_style_bg_color(ui_OIL_TEMP_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_W_TEMP_Bar) lv_obj_set_style_bg_color(ui_W_TEMP_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_TRAN_TEMP_Bar) lv_obj_set_style_bg_color(ui_TRAN_TEMP_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_STEER_TEMP_Bar) lv_obj_set_style_bg_color(ui_STEER_TEMP_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_DIFF_TEMP_Bar) lv_obj_set_style_bg_color(ui_DIFF_TEMP_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
    if (ui_FUEL_TRUST_Bar) lv_obj_set_style_bg_color(ui_FUEL_TRUST_Bar, lv_color_hex(hexOrange), LV_PART_INDICATOR);
#endif

#if ENABLE_LIGHTWEIGHT_BARS
    resetLightweightBars();
#endif
}

//=================================================================
// LIGHTWEIGHT BARS - Simple rectangles (much cheaper than lv_bar)
//=================================================================
#if ENABLE_LIGHTWEIGHT_BARS

void initLightweightBars() {
    Serial.println("[LIGHT_BARS] Initializing lightweight bars...");
    
    // Force LVGL to calculate layout for all objects before reading geometry
    // This fixes the FUEL_TRUST bar having 0x0 size because layout wasn't complete
    lv_obj_update_layout(ui_Screen1);
    
    // Reference to original SquareLine bars to clone position/size
    lv_obj_t* original_bars[] = {
        ui_OIL_PRESS_Bar, ui_OIL_TEMP_Bar, ui_W_TEMP_Bar,
        ui_TRAN_TEMP_Bar, ui_STEER_TEMP_Bar, ui_DIFF_TEMP_Bar, ui_FUEL_TRUST_Bar
    };
    
    // Debug: Check which bars are NULL
    const char* bar_names[] = {
        "OIL_PRESS", "OIL_TEMP", "W_TEMP",
        "TRAN_TEMP", "STEER_TEMP", "DIFF_TEMP", "FUEL_TRUST"
    };
    
    for (int i = 0; i < 7; i++) {
        if (!original_bars[i]) {
            Serial.printf("[LIGHT_BARS] WARNING: %s_Bar is NULL!\n", bar_names[i]);
        }
    }
    
    // Value ranges for each bar
    struct BarConfig {
        int16_t min_val;
        int16_t max_val;
    };
    
    BarConfig configs[] = {
        {5, 150},    // Oil Pressure (PSI)
        {150, 300},  // Oil Temp (F)
        {100, 260},  // Water Temp (F)
        {80, 280},   // Trans Temp (F)
        {60, 300},   // Steer Temp (F)
        {60, 320},   // Diff Temp (F)
        {5, 100}     // Fuel Trust (%)
    };
    
    for (int i = 0; i < 7; i++) {
        lv_obj_t* ref = original_bars[i];
        if (!ref) {
            g_light_bars[i].obj = NULL;
            Serial.printf("[LIGHT_BARS] %d - no reference bar\n", i);
            _RealSerial.flush();  // Force output
            continue;
        }
        
        lv_obj_t* parent = lv_obj_get_parent(ref);
        if (!parent) {
            g_light_bars[i].obj = NULL;
            Serial.printf("[LIGHT_BARS] %d - parent is NULL\n", i);
            _RealSerial.flush();  // Force output
            continue;
        }
        
        // Read real geometry from the SquareLine bar
        lv_coord_t x = lv_obj_get_x(ref);
        lv_coord_t y = lv_obj_get_y(ref);
        lv_coord_t h = lv_obj_get_height(ref);
        lv_coord_t w = lv_obj_get_width(ref);
        
        // Safety check - skip if geometry is invalid
        if (w <= 0 || h <= 0) {
            g_light_bars[i].obj = NULL;
            Serial.printf("[LIGHT_BARS] %d - invalid geometry w=%d h=%d\n", i, w, h);
            _RealSerial.flush();  // Force output
            continue;
        }
        
        // Hide the original bar's indicator so only our overlay shows
        lv_obj_set_style_bg_opa(ref, LV_OPA_TRANSP, LV_PART_INDICATOR);
        
        // Create overlay rectangle
        lv_obj_t* bar = lv_obj_create(parent);
        lv_obj_remove_style_all(bar);
        
        // Use FLOATING flag to bypass flex/grid layout (LVGL v9)
        lv_obj_add_flag(bar, LV_OBJ_FLAG_FLOATING);
        
        // Exact overlay placement matching original bar
        lv_obj_set_pos(bar, x, y);
        lv_obj_set_size(bar, 0, h);  // Start 0px wide (hidden), full real height
        lv_obj_add_flag(bar, LV_OBJ_FLAG_HIDDEN);  // Start hidden until valid data
        
        // Visuals
        lv_obj_set_style_bg_color(bar, lv_color_hex(LIGHT_BAR_COLOR), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(bar, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(bar, 0, LV_PART_MAIN);
        
        // No interaction
        lv_obj_remove_flag(bar, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        
        // Put it right above the original bar
        lv_obj_move_to_index(bar, lv_obj_get_index(ref) + 1);
        
        Serial.printf("[LIGHT_BARS] %d ref(x=%d y=%d w=%d h=%d) overlay(h=%d)\n",
                      i, x, y, w, h, lv_obj_get_height(bar));
        
        // Store configuration
        g_light_bars[i].obj = bar;
        g_light_bars[i].parent = parent;
        g_light_bars[i].min_val = configs[i].min_val;
        g_light_bars[i].max_val = configs[i].max_val;
        g_light_bars[i].max_width = w;
        g_light_bars[i].last_width = -1;  // Force first update
    }
    
    Serial.println("[LIGHT_BARS] Initialized 7 lightweight bars");
}

void updateLightweightBar(int index, float value) {
    if (index < 0 || index >= 7) return;
    LightweightBar& bar = g_light_bars[index];
    if (!bar.obj) return;
    
    // Calculate width based on value
    float range = bar.max_val - bar.min_val;
    float normalized = (value - bar.min_val) / range;
    normalized = constrain(normalized, 0.0f, 1.0f);
    
    int16_t new_width = (int16_t)(normalized * bar.max_width);
    // Allow 0 width - bar will be hidden when empty
    
    // Only update if width changed by threshold (reduces redraws)
    int16_t diff = new_width - bar.last_width;
    if (diff < 0) diff = -diff;  // abs
    
    if (diff >= LIGHT_BAR_WIDTH_THRESHOLD || bar.last_width < 0) {
        if (new_width <= 0) {
            // Hide bar completely when no value
            lv_obj_add_flag(bar.obj, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_width(bar.obj, 0);
        } else {
            // Show bar and set width
            lv_obj_remove_flag(bar.obj, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_width(bar.obj, new_width);
        }
        bar.last_width = new_width;
    }
}

// Call this once per UI update cycle - returns true if bars should update this frame
bool shouldUpdateLightweightBars() {
    g_light_bar_frame_counter++;
    if (g_light_bar_frame_counter >= LIGHT_BAR_FRAME_SKIP) {
        g_light_bar_frame_counter = 0;
        return true;
    }
    return false;
}

void resetLightweightBars() {
    for (int i = 0; i < 7; i++) {
        if (g_light_bars[i].obj) {
            lv_obj_add_flag(g_light_bars[i].obj, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_width(g_light_bars[i].obj, 0);
            g_light_bars[i].last_width = 0;
        }
    }
}

#endif // ENABLE_LIGHTWEIGHT_BARS

// Reset chart data and history
void resetCharts() {
#if ENABLE_CHARTS
    // Reset oil pressure chart
    if (ui_OIL_PRESS_CHART && chart_series_oil_press) {
        for (int i = 0; i < CHART_POINTS; i++) {
            oil_press_history[i] = CHART_NO_DATA;
        }
        // Clear all points by setting to below range
        lv_chart_set_all_value(ui_OIL_PRESS_CHART, chart_series_oil_press, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_OIL_PRESS_CHART);
    }
    // Reset oil temp chart
    if (ui_OIL_TEMP_CHART && chart_series_oil_temp) {
        for (int i = 0; i < CHART_POINTS; i++) {
            oil_temp_history[i] = CHART_NO_DATA;
        }
        lv_chart_set_all_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_OIL_TEMP_CHART);
    }
    // Reset water temp chart
    if (ui_W_TEMP_CHART && chart_series_water_temp) {
        for (int i = 0; i < CHART_POINTS; i++) {
            water_temp_history[i] = CHART_NO_DATA;
        }
        lv_chart_set_all_value(ui_W_TEMP_CHART, chart_series_water_temp, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_W_TEMP_CHART);
    }
    // Reset transmission temp chart
    if (ui_TRAN_TEMP_CHART && chart_series_transmission_temp) {
        for (int i = 0; i < CHART_POINTS; i++) {
            transmission_temp_history[i] = CHART_NO_DATA;
        }
        lv_chart_set_all_value(ui_TRAN_TEMP_CHART, chart_series_transmission_temp, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_TRAN_TEMP_CHART);
    }
    // Reset steering temp chart
    if (ui_STEER_TEMP_CHART && chart_series_steering_temp) {
        for (int i = 0; i < CHART_POINTS; i++) {
            steering_temp_history[i] = CHART_NO_DATA;
        }
        lv_chart_set_all_value(ui_STEER_TEMP_CHART, chart_series_steering_temp, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_STEER_TEMP_CHART);
    }
    // Reset differencial temp chart
    if (ui_DIFF_TEMP_CHART && chart_series_differencial_temp) {
        for (int i = 0; i < CHART_POINTS; i++) {
            differencial_temp_history[i] = CHART_NO_DATA;
        }
        lv_chart_set_all_value(ui_DIFF_TEMP_CHART, chart_series_differencial_temp, LV_CHART_POINT_NONE);
        lv_chart_refresh(ui_DIFF_TEMP_CHART);
    }
    // Reset fuel trust chart
    if (ui_FUEL_TRUST_CHART && chart_series_fuel_trust) {
        for (int i = 0; i < CHART_POINTS; i++) {
            fuel_trust_history[i] = CHART_NO_DATA;
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

// Pre-calculated sine table (256 entries)
// This is (1-cos(x))/2 * 255 for x from 0 to 2*PI, giving smooth "slow at edges" effect
// Values range 0-255, eliminating expensive floating-point trig operations
static const uint8_t SINE_TABLE[256] PROGMEM = {
    // (1-cos(i*2*PI/256))/2 * 255 - smooth ease-in-ease-out, one complete cycle
      0,   0,   0,   0,   1,   1,   2,   2,   3,   4,   5,   6,   7,   8,   9,  10,
     12,  13,  15,  17,  18,  20,  22,  24,  26,  28,  31,  33,  35,  38,  40,  43,
     45,  48,  51,  54,  57,  60,  63,  66,  69,  72,  76,  79,  82,  86,  89,  93,
     96, 100, 103, 107, 111, 114, 118, 121, 125, 129, 132, 136, 140, 143, 147, 151,
    154, 158, 162, 165, 169, 172, 176, 179, 183, 186, 190, 193, 196, 200, 203, 206,
    209, 212, 215, 218, 221, 223, 226, 229, 231, 234, 236, 238, 241, 243, 245, 247,
    248, 250, 251, 253, 254, 255, 255, 255, 255, 255, 255, 255, 254, 253, 251, 250,
    248, 247, 245, 243, 241, 238, 236, 234, 231, 229, 226, 223, 221, 218, 215, 212,
    209, 206, 203, 200, 196, 193, 190, 186, 183, 179, 176, 172, 169, 165, 162, 158,
    154, 151, 147, 143, 140, 136, 132, 129, 125, 121, 118, 114, 111, 107, 103, 100,
     96,  93,  89,  86,  82,  79,  76,  72,  69,  66,  63,  60,  57,  54,  51,  48,
     45,  43,  40,  38,  35,  33,  31,  28,  26,  24,  22,  20,  18,  17,  15,  13,
     12,  10,   9,   8,   7,   6,   5,   4,   3,   2,   2,   1,   1,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,
      2,   2,   3,   4,   5,   6,   7,   8,   9,  10,  12,  13,  15,  17,  18,  20
};

// Fast sine approximation using lookup table - no floating point trig!
// Replicates the "slow at edges" (1-cos)/2 behavior of the original
int calcDemoValue(int min_val, int max_val, uint32_t cycle_ms, uint32_t offset_ms = 0) {
    uint32_t now = millis();
    uint32_t elapsed = (now - g_demo_state.start_time + offset_ms) % cycle_ms;

    // Map elapsed time to table index (0-255)
    uint32_t index = (elapsed * 256UL) / cycle_ms;

    // Get pre-calculated (1-cos)/2 value scaled 0-255
    uint8_t table_val = pgm_read_byte(&SINE_TABLE[index & 0xFF]);

    // Map to output range
    int32_t range = max_val - min_val;
    return min_val + (int)((range * table_val) / 255);
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
#if ENABLE_MODBUS_SENSORS
    // Read from Modbus 8-Ch Analog Module
    readModbusSensors();
    
    // Note: All Modbus sensors (CH1-CH5) are now handled by readModbusSensors()
    // Only mark non-Modbus sensors as invalid
    g_vehicle_data.water_temp_valid = false;   // Water temp not yet on Modbus
    g_vehicle_data.fuel_trust_valid = false;   // Fuel trust not yet on Modbus
#else
    // No sensors configured - show "---" on all gauges
    g_vehicle_data.oil_pressure_valid = false;
    g_vehicle_data.oil_temp_valid = false;
    g_vehicle_data.water_temp_valid = false;
    g_vehicle_data.trans_temp_valid = false;
    g_vehicle_data.steer_temp_valid = false;
    g_vehicle_data.diff_temp_valid = false;
    g_vehicle_data.fuel_trust_valid = false;
#endif
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
#define SD_FLUSH_INTERVAL_MS 1000 // Flush to card every 1 second (not every write)
#define SD_BUFFER_SIZE    256     // Smaller buffer for more frequent flushes
#define SD_MAX_RETRIES    3       // Max retries on write failure
#define SD_FREE_SPACE_PERCENT 5   // Keep at least 5% free space
#define SD_MIN_FREE_BYTES (1024 * 1024)  // Absolute minimum 1MB free

// ===== TIMEKEEPING CONFIGURATION =====
// WiFi credentials loaded from SD card config file (/wifi.cfg)
// Config file format (one per line):
//   WIFI_SSID=YourNetworkName
//   WIFI_PASSWORD=YourPassword
#define WIFI_CONFIG_FILE "/wifi.cfg"
static char g_wifi_ssid[64] = "";      // Loaded from SD config
static char g_wifi_password[64] = ""; // Loaded from SD config
#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.nist.gov"
#define NTP_SERVER_3 "time.google.com"
#define GMT_OFFSET_SEC (-6 * 3600)       // CST = UTC-6 (adjust for your timezone)
#define DAYLIGHT_OFFSET_SEC 0            // Set to 3600 if DST is active
#define WIFI_CONNECT_TIMEOUT_MS 10000    // 10 second timeout for WiFi connection
#define NTP_SYNC_TIMEOUT_MS 3000         // 3 second timeout per NTP server

// SD Card State
struct SDState {
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
    uint32_t last_flush_ms;     // Time-based flush tracking
    uint32_t boot_count;        // Persisted boot counter
    uint64_t total_bytes;       // Total card size
    uint64_t used_bytes;        // Used space
    // Serial log state
    File log_file;
    char log_filename[32];
    bool log_file_open;
    uint32_t log_bytes_written;
    uint32_t last_log_flush_ms;
};
static SDState g_sd_state = { 0 };

//=================================================================
// SERIAL LOG QUEUE - Buffers serial output for SD card writing
// (TeeSerial class that intercepts Serial is defined at top of file)
//=================================================================

#define SERIAL_LOG_QUEUE_SIZE 32
// SERIAL_LOG_MAX_MSG_LEN is defined at top of file with TeeSerial

struct SerialLogEntry {
    uint32_t timestamp_ms;
    char message[SERIAL_LOG_MAX_MSG_LEN];
};

static QueueHandle_t g_serial_log_queue = NULL;

// Timekeeping State
enum TimeSyncStatus { TIME_SYNC_IDLE, TIME_SYNC_CONNECTING, TIME_SYNC_SYNCING, TIME_SYNC_OK, TIME_SYNC_FAILED };
static struct {
    bool time_available;        // True if we have valid time from any source
    bool rtc_active;            // Using DS3231 RTC
    bool wifi_time_active;      // Using WiFi NTP time
    TimeSyncStatus sync_status; // Background sync status
    struct tm current_time;     // Current time structure
    char time_string[24];       // "MM/DD/YYYY HH:MM:SS" or status
    uint32_t last_update_ms;    // Last time update timestamp
} g_time_state = { 0 };

// Time sync task handle
static TaskHandle_t g_time_sync_task_handle = NULL;
static SemaphoreHandle_t g_time_mutex = NULL;  // Protects g_time_state from race conditions

// Forward declarations
bool sdInit();
bool sdTryReinit();
bool sdStartSession();
bool loadWifiConfig();  // Load WiFi credentials from SD card
void sdEndSession();
void sdLogData();
void sdSafeFlush();
float getCpuLoadPercent();
bool sdCheckAndManageSpace();
bool sdDeleteOldestLog();
uint32_t sdReadBootCount();
void sdWriteBootCount(uint32_t count);
bool sdDetectRTC();
void initTimeKeeping();
void timeSyncTask(void* parameter);
bool tryNTPSync(const char* server);
bool readRTC(struct tm* timeinfo);
bool writeRTC(struct tm* timeinfo);
bool isRTCTimeValid();
void clearRTCOSFlag();
void updateTime();
uint8_t bcdToDec(uint8_t val);
uint8_t decToBcd(uint8_t val);
bool sdStartLogSession();
void sdEndLogSession();
void sdLogSerialFlush();

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

    // SD.begin() requires a CS pin, but real CS is via IO expander (EXIO_SD_CS)
    // GPIO6 is unused - safe as dummy. Avoid: GPIO15 (RS485), GPIO46 (HSYNC)
    pinMode(6, OUTPUT);
    digitalWrite(6, HIGH);  // Keep dummy CS high (deselected)

    if (!SD.begin(6, SPI, SD_SPI_FREQ)) {
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
    
    // ONE-TIME RESET: Remove these 2 lines after first boot to resume normal numbering
    //g_sd_state.boot_count = 0;  // Reset to start fresh
    // END ONE-TIME RESET
    
    g_sd_state.boot_count++;
    sdWriteBootCount(g_sd_state.boot_count);
    Serial.printf("[SD] Boot count: %lu\n", g_sd_state.boot_count);

    g_sd_state.rtc_available = sdDetectRTC();

    sdCheckAndManageSpace();

    Serial.println("[SD] Initialization successful");
    return true;
}

// Try to reinitialize SD card after hot-swap (card was removed and reinserted)
// Returns true if card is back online and session started
bool sdTryReinit() {
    // Only attempt if card is currently offline
    if (g_sd_state.initialized && g_sd_state.card_present) {
        return true;  // Already online
    }
    
    Serial.println("[SD] Attempting hot-swap recovery...");
    
    // Clean up any previous state
    SD.end();
    delay(50);
    
    // Select SD card via IO expander
    exio_set(EXIO_SD_CS, false);  // CS low = selected
    delay(10);
    
    // Try to initialize
    if (!SD.begin(6, SPI, SD_SPI_FREQ)) {
        exio_set(EXIO_SD_CS, true);  // Deselect on failure
        return false;  // Card still not present
    }
    
    // Check card type
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        return false;
    }
    
    const char* cardTypeName = "UNKNOWN";
    switch (cardType) {
    case CARD_MMC:  cardTypeName = "MMC";  break;
    case CARD_SD:   cardTypeName = "SD";   break;
    case CARD_SDHC: cardTypeName = "SDHC"; break;
    }
    
    Serial.printf("[SD] Hot-swap: Card detected - %s\n", cardTypeName);
    
    // Restore state
    g_sd_state.total_bytes = SD.totalBytes();
    g_sd_state.used_bytes = SD.usedBytes();
    g_sd_state.initialized = true;
    g_sd_state.card_present = true;
    g_sd_state.logging_enabled = true;
    
    // Read boot count (might be different card)
    g_sd_state.boot_count = sdReadBootCount();
    g_sd_state.boot_count++;
    sdWriteBootCount(g_sd_state.boot_count);
    Serial.printf("[SD] Hot-swap: Boot count: %lu\n", g_sd_state.boot_count);
    
    // Start new session
    if (sdStartSession()) {
        Serial.println("[SD] Hot-swap recovery successful!");
        return true;
    } else {
        Serial.println("[SD] Hot-swap: Card online but session start failed");
        return false;
    }
}

// Generate unique session filename
void sdGenerateFilename() {
    // Format: SESS_NNNNN.csv (boot count based)
    // If RTC available in future: YYYY-MM-DD_HH-MM-SS.csv
    snprintf(g_sd_state.current_filename, sizeof(g_sd_state.current_filename),
        "/SESS_%08lu.csv", g_sd_state.boot_count);
    
    // Export boot count for file browser
    g_current_boot_count = g_sd_state.boot_count;
    // Also generate matching log filename
    snprintf(g_sd_state.log_filename, sizeof(g_sd_state.log_filename),
        "/SESS_%08lu.log", g_sd_state.boot_count);
    Serial.printf("[SD] Session files: %s, %s\n", g_sd_state.current_filename, g_sd_state.log_filename);
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

    // Always use FILE_WRITE to create fresh file (each boot has unique count)
    // FILE_APPEND was causing duplicate headers when boot count got reused
    g_sd_state.data_file = SD.open(g_sd_state.current_filename, FILE_WRITE);
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
    g_sd_state.last_flush_ms = millis();

    const char* header = "datetime,rpm,oil_press_psi,"
        "oil_temp_pan_f,oil_temp_cooled_f,"
        "water_temp_hot_f,water_temp_cooled_f,"
        "trans_temp_hot_f,trans_temp_cooled_f,"
        "steer_temp_hot_f,steer_temp_cooled_f,"
        "diff_temp_hot_f,diff_temp_cooled_f,"
        "fuel_trust_percent,fuel_trust_valid,"
        "rpm_valid,oil_press_valid,oil_temp_valid,"
        "water_temp_valid,trans_temp_valid,steer_temp_valid,diff_temp_valid,"
        "timestamp_ms,elapsed_s,cpu_percent,mode\n";

    size_t written = g_sd_state.data_file.print(header);
    if (written == 0) {
        g_sd_state.data_file.close();
        g_sd_state.file_open = false;
        return false;
    }

    g_sd_state.bytes_written += written;
    sdSafeFlush();  // Immediate flush after header

    Serial.printf("[SD] Session started: %s\n", g_sd_state.current_filename);
    
    // Also start the serial log session
    sdStartLogSession();
    
    return true;
}

// Start serial log session (creates .log file)
bool sdStartLogSession() {
    if (!g_sd_state.initialized || !g_sd_state.card_present) {
        return false;
    }
    
    // Create log file (fresh file for each boot)
    g_sd_state.log_file = SD.open(g_sd_state.log_filename, FILE_WRITE);
    if (!g_sd_state.log_file) {
        Serial.printf("[SD] Failed to create log file: %s\n", g_sd_state.log_filename);
        return false;
    }
    
    g_sd_state.log_file_open = true;
    g_sd_state.log_bytes_written = 0;
    g_sd_state.last_log_flush_ms = millis();
    
    // Write header
    char header[128];
    snprintf(header, sizeof(header), "=== 370zMonitor Serial Log - Session %lu ===\n", g_sd_state.boot_count);
    g_sd_state.log_file.print(header);
    g_sd_state.log_bytes_written += strlen(header);
    g_sd_state.log_file.flush();
    
    Serial.printf("[SD] Log session started: %s\n", g_sd_state.log_filename);
    return true;
}

// End serial log session
void sdEndLogSession() {
    if (!g_sd_state.log_file_open) return;
    
    g_sd_state.log_file.flush();
    g_sd_state.log_file.close();
    g_sd_state.log_file_open = false;
    Serial.printf("[SD] Log session ended: %lu bytes\n", g_sd_state.log_bytes_written);
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
    
    // Also end log session
    sdEndLogSession();
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

//=================================================================
// SD WRITE TASK - RUNS ON CORE 0
// Receives log entries via queue, writes to SD card
//=================================================================

void sdWriteTask(void* parameter) {
    // Use _RealSerial directly to avoid TeeSerial queue (chicken-egg problem)
    _RealSerial.println("[CORE0/SD] SD write task RUNNING");
    _RealSerial.printf("[CORE0/SD] file_open=%d log_open=%d\n", g_sd_state.file_open, g_sd_state.log_file_open);
    _RealSerial.flush();
    
    SDLogEntry entry;
    SerialLogEntry logEntry;
    char line[300];  // Increased for datetime column
    
    // Debug counter for periodic status
    uint32_t loop_counter = 0;
    uint32_t last_debug_ms = millis();
    uint32_t writes_since_debug = 0;
    
    while (true) {
        loop_counter++;
        
        // Check if file browser wants us to pause
        if (g_fb_pause_sd_writes) {
            vTaskDelay(pdMS_TO_TICKS(100));  // Sleep while paused
            continue;
        }
        
        // Periodic debug output (every 10 seconds)
        uint32_t now_dbg = millis();
        if (now_dbg - last_debug_ms >= 10000) {
            _RealSerial.printf("[CORE0/SD] alive: loops=%lu writes=%lu csv=%d log=%d\n", 
                               loop_counter, writes_since_debug, g_sd_state.file_open, g_sd_state.log_file_open);
            _RealSerial.flush();
            last_debug_ms = now_dbg;
            writes_since_debug = 0;
        }
        
        // Process serial log entries first (higher priority for responsiveness)
        while (g_serial_log_queue && 
               xQueueReceive(g_serial_log_queue, &logEntry, 0) == pdTRUE) {
            if (g_sd_state.log_file_open) {
                // Format: [timestamp_ms] message
                char logLine[SERIAL_LOG_MAX_MSG_LEN + 32];
                int len = snprintf(logLine, sizeof(logLine), "[%lu] %s\n", 
                                   logEntry.timestamp_ms, logEntry.message);
                
                size_t written = g_sd_state.log_file.print(logLine);
                if (written > 0) {
                    g_sd_state.log_bytes_written += written;
                }
            }
        }
        
        // Periodic flush of log file
        uint32_t now = millis();
        if (g_sd_state.log_file_open && 
            (now - g_sd_state.last_log_flush_ms) >= SD_FLUSH_INTERVAL_MS) {
            g_sd_state.log_file.flush();
            g_sd_state.last_log_flush_ms = now;
        }
        
        // Wait for data from queue (blocks until data available or timeout)
        if (xQueueReceive(g_sd_queue, &entry, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (!g_sd_state.file_open || !g_sd_state.logging_enabled) {
                continue;  // Discard if logging disabled
            }
            
            // Periodic space check (every 60 writes)
            if (g_sd_state.write_count % 60 == 0 && g_sd_state.write_count > 0) {
                if (!sdCheckAndManageSpace()) {
                    g_sd_state.logging_enabled = false;
                    continue;
                }
            }
            
            // Format data line (datetime first, then existing columns)
            // Copy time string with mutex to prevent race condition
            char datetime_copy[24];
            if (g_time_mutex && xSemaphoreTake(g_time_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                strncpy(datetime_copy, g_time_state.time_string, sizeof(datetime_copy) - 1);
                datetime_copy[sizeof(datetime_copy) - 1] = '\0';
                xSemaphoreGive(g_time_mutex);
            } else {
                strcpy(datetime_copy, "N/A");
            }
            
            int len = snprintf(line, sizeof(line),
                "%s,%d,%d,"
                "%d,%d,"
                "%d,%d,"
                "%d,%d,"
                "%d,%d,"
                "%d,%d,"
                "%d,%d,"
                "%d,%d,%d,"
                "%d,%d,%d,%d,"
                "%lu,%.2f,%.1f,%s\n",
                datetime_copy,  // datetime
                entry.data.rpm, entry.data.oil_pressure_psi,  // rpm, oil_press_psi
                entry.data.oil_temp_pan_f, entry.data.oil_temp_cooled_f,  // oil temps
                entry.data.water_temp_hot_f, entry.data.water_temp_cooled_f,  // water temps
                entry.data.trans_temp_hot_f, entry.data.trans_temp_cooled_f,  // trans temps
                entry.data.steer_temp_hot_f, entry.data.steer_temp_cooled_f,  // steer temps
                entry.data.diff_temp_hot_f, entry.data.diff_temp_cooled_f,  // diff temps
                entry.data.fuel_trust_percent, entry.data.fuel_trust_valid ? 1 : 0,  // fuel trust
                entry.data.rpm_valid ? 1 : 0, entry.data.oil_pressure_valid ? 1 : 0,
                entry.data.oil_temp_valid ? 1 : 0,  // validity flags part 1
                entry.data.water_temp_valid ? 1 : 0, entry.data.trans_temp_valid ? 1 : 0,
                entry.data.steer_temp_valid ? 1 : 0, entry.data.diff_temp_valid ? 1 : 0,  // validity flags part 2
                entry.timestamp_ms, entry.elapsed_s, entry.cpu_pct,
                entry.demo_mode ? "DEMO" : "LIVE"  // metadata at end
            );
            
            // Write with retry
            bool success = false;
            for (int retry = 0; retry < SD_MAX_RETRIES && !success; retry++) {
                size_t written = g_sd_state.data_file.print(line);
                if (written > 0) {
                    g_sd_state.bytes_written += written;
                    g_sd_state.write_count++;
                    success = true;
                }
                else {
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
            }
            
            if (success) {
                writes_since_debug++;
            } else {
                g_sd_state.error_count++;
                if (g_sd_state.error_count >= 10) {
                    g_sd_state.logging_enabled = false;
                    _RealSerial.println("[SD/CORE0] Too many errors, logging disabled");
                }
            }
            
            // Time-based flush
            uint32_t now = millis();
            if ((now - g_sd_state.last_flush_ms) >= SD_FLUSH_INTERVAL_MS) {
                g_sd_state.data_file.flush();
                g_sd_state.last_flush_ms = now;
            }
        }
        // No data received - just continue waiting
    }
}

//=================================================================
// SERIAL LOGGING FUNCTIONS
// Queue handler for SD card logging (TeeSerial class is at top of file)
//=================================================================

// Queue a message for SD card logging (called by TeeSerial)
void sdLogSerialWrite(const char* msg) {
    if (!g_serial_log_queue) return;
    
    SerialLogEntry entry;
    entry.timestamp_ms = millis();
    strncpy(entry.message, msg, SERIAL_LOG_MAX_MSG_LEN - 1);
    entry.message[SERIAL_LOG_MAX_MSG_LEN - 1] = '\0';
    
    // Non-blocking queue send - drop if full
    xQueueSend(g_serial_log_queue, &entry, 0);
}

// Queue data for SD logging - called from main loop (Core 1)
// Non-blocking: if queue is full, data is dropped
void sdLogData() {
    if (!g_sd_state.file_open || !g_sd_state.logging_enabled) return;
    if (!g_sd_queue) return;

    uint32_t now = millis();

    // Check if it's time to log
    if ((now - g_sd_state.last_write_ms) < SD_WRITE_INTERVAL_MS) return;
    g_sd_state.last_write_ms = now;

    // Prepare log entry
    SDLogEntry entry;
    entry.timestamp_ms = now;
    entry.elapsed_s = (float)(now - g_sd_state.session_start_ms) / 1000.0f;
    entry.cpu_pct = getCpuLoadPercent();
    entry.demo_mode = g_demo_mode;
    entry.data = g_vehicle_data;  // Copy current vehicle data

    // Queue the entry (non-blocking)
    if (xQueueSend(g_sd_queue, &entry, 0) != pdTRUE) {
        // Queue full - data dropped (happens if Core 0 is backed up)
        static uint32_t last_drop_warn = 0;
        if (now - last_drop_warn > 5000) {
            Serial.println("[SD] Warning: Queue full, data dropped");
            last_drop_warn = now;
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

//=================================================================
// WIFI CONFIG LOADING
// Reads SSID and password from /wifi.cfg on SD card
//=================================================================

bool loadWifiConfig() {
    if (!g_sd_state.initialized) {
        Serial.println("[WIFI] SD not initialized, cannot load config");
        return false;
    }
    
    File configFile = SD.open(WIFI_CONFIG_FILE, FILE_READ);
    if (!configFile) {
        Serial.printf("[WIFI] Config file %s not found, creating template...\n", WIFI_CONFIG_FILE);
        
        // Create template config file
        File templateFile = SD.open(WIFI_CONFIG_FILE, FILE_WRITE);
        if (templateFile) {
            templateFile.println("WIFI_SSID=");
            templateFile.println("WIFI_PASSWORD=");
            templateFile.close();
            Serial.printf("[WIFI] Template created at %s - please edit with your credentials\n", WIFI_CONFIG_FILE);
        } else {
            Serial.println("[WIFI] Failed to create template config file");
        }
        return false;
    }
    
    Serial.printf("[WIFI] Loading config from %s\n", WIFI_CONFIG_FILE);
    
    bool ssid_found = false;
    bool pass_found = false;
    
    while (configFile.available()) {
        String line = configFile.readStringUntil('\n');
        line.trim();  // Remove whitespace and CR/LF
        
        // Skip empty lines and comments
        if (line.length() == 0 || line.startsWith("#") || line.startsWith("//")) {
            continue;
        }
        
        // Parse key=value format
        int eqPos = line.indexOf('=');
        if (eqPos > 0) {
            String key = line.substring(0, eqPos);
            String value = line.substring(eqPos + 1);
            key.trim();
            value.trim();
            
            if (key.equalsIgnoreCase("WIFI_SSID") || key.equalsIgnoreCase("SSID")) {
                strncpy(g_wifi_ssid, value.c_str(), sizeof(g_wifi_ssid) - 1);
                g_wifi_ssid[sizeof(g_wifi_ssid) - 1] = '\0';
                ssid_found = true;
                Serial.printf("[WIFI] SSID loaded: %s\n", g_wifi_ssid);
            }
            else if (key.equalsIgnoreCase("WIFI_PASSWORD") || key.equalsIgnoreCase("PASSWORD")) {
                strncpy(g_wifi_password, value.c_str(), sizeof(g_wifi_password) - 1);
                g_wifi_password[sizeof(g_wifi_password) - 1] = '\0';
                pass_found = true;
                Serial.println("[WIFI] Password loaded: ********");
            }
        }
    }
    
    configFile.close();
    
    if (ssid_found && pass_found) {
        Serial.println("[WIFI] Config loaded successfully");
        return true;
    } else {
        Serial.printf("[WIFI] Config incomplete - SSID:%s PASS:%s\n", 
                      ssid_found ? "OK" : "MISSING", 
                      pass_found ? "OK" : "MISSING");
        return false;
    }
}

//=================================================================
// TIMEKEEPING FUNCTIONS
// DS3231 RTC (primary) -> WiFi NTP background task (fallback)
// WiFi sync runs on Core 0 and does NOT block startup
//=================================================================

// BCD to decimal conversion helper
uint8_t bcdToDec(uint8_t val) {
    return ((val / 16 * 10) + (val % 16));
}

// Decimal to BCD conversion (for writing to RTC)
uint8_t decToBcd(uint8_t val) {
    return ((val / 10 * 16) + (val % 10));
}

// Read time from DS3231 RTC
bool readRTC(struct tm* timeinfo) {
    if (!g_sd_state.rtc_available) return false;
    
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(0x00);  // Start at register 0 (seconds)
    if (Wire.endTransmission() != 0) {
        return false;
    }
    
    Wire.requestFrom(DS3231_ADDR, 7);
    if (Wire.available() < 7) {
        return false;
    }
    
    uint8_t seconds = bcdToDec(Wire.read() & 0x7F);
    uint8_t minutes = bcdToDec(Wire.read());
    uint8_t hours = bcdToDec(Wire.read() & 0x3F);  // 24-hour mode
    Wire.read();  // Skip day of week
    uint8_t day = bcdToDec(Wire.read());
    uint8_t month = bcdToDec(Wire.read() & 0x1F);
    uint8_t year = bcdToDec(Wire.read());
    
    timeinfo->tm_sec = seconds;
    timeinfo->tm_min = minutes;
    timeinfo->tm_hour = hours;
    timeinfo->tm_mday = day;
    timeinfo->tm_mon = month - 1;      // tm_mon is 0-11
    timeinfo->tm_year = year + 100;    // tm_year is years since 1900
    
    return true;
}

// Write time TO DS3231 RTC (used after NTP sync)
bool writeRTC(struct tm* timeinfo) {
    if (!g_sd_state.rtc_available) return false;
    
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(0x00);  // Start at register 0 (seconds)
    Wire.write(decToBcd(timeinfo->tm_sec));
    Wire.write(decToBcd(timeinfo->tm_min));
    Wire.write(decToBcd(timeinfo->tm_hour));   // 24-hour format
    Wire.write(decToBcd(timeinfo->tm_wday + 1)); // Day of week (1-7)
    Wire.write(decToBcd(timeinfo->tm_mday));
    Wire.write(decToBcd(timeinfo->tm_mon + 1)); // Month (1-12)
    Wire.write(decToBcd(timeinfo->tm_year - 100)); // Year (0-99)
    
    return (Wire.endTransmission() == 0);
}

// Check if RTC has valid time (not factory default or lost power without battery)
// Returns false if oscillator stopped flag is set or time is clearly invalid
bool isRTCTimeValid() {
    if (!g_sd_state.rtc_available) return false;
    
    // Read status register (0x0F) to check oscillator stopped flag (OSF)
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(0x0F);  // Status register address
    if (Wire.endTransmission() != 0) return false;
    
    Wire.requestFrom(DS3231_ADDR, 1);
    if (Wire.available() < 1) return false;
    
    uint8_t status = Wire.read();
    
    // Bit 7 (OSF) = Oscillator Stop Flag
    // If set, oscillator stopped at some point (battery dead or first power on)
    if (status & 0x80) {
        Serial.println("[RTC] OSF flag set - time not reliable");
        return false;
    }
    
    // Additionally check if time is reasonable (year >= 2024)
    struct tm timeinfo;
    if (!readRTC(&timeinfo)) return false;
    
    // If year is before 2024, time was never set properly
    if ((timeinfo.tm_year + 1900) < 2024) {
        Serial.printf("[RTC] Year %d too old - time not set\n", timeinfo.tm_year + 1900);
        return false;
    }
    
    return true;
}

// Clear the OSF flag after setting time (acknowledges new time is valid)
void clearRTCOSFlag() {
    if (!g_sd_state.rtc_available) return;
    
    // Read current status register
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(0x0F);
    Wire.endTransmission();
    
    Wire.requestFrom(DS3231_ADDR, 1);
    if (Wire.available() < 1) return;
    
    uint8_t status = Wire.read();
    
    // Clear OSF (bit 7) by writing 0 to it
    status &= ~0x80;
    
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(0x0F);
    Wire.write(status);
    Wire.endTransmission();
    
    Serial.println("[RTC] OSF flag cleared");
}

// Try to sync time from a specific NTP server
bool tryNTPSync(const char* server) {
    Serial.printf("[TIME] Trying NTP server: %s\n", server);
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, server);
    
    struct tm timeinfo;
    uint32_t start_ms = millis();
    while ((millis() - start_ms) < NTP_SYNC_TIMEOUT_MS) {
        if (getLocalTime(&timeinfo, 100)) {  // 100ms timeout per attempt
            Serial.printf("[TIME] NTP sync successful from %s\n", server);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    Serial.printf("[TIME] NTP server %s timeout\n", server);
    return false;
}

// Background task for WiFi time sync (runs on Core 0)
void timeSyncTask(void* parameter) {
    Serial.println("[TIME/CORE0] Time sync task started");
    
    bool rtc_had_valid_time = isRTCTimeValid();
    
    // Check if WiFi credentials are configured
    if (strlen(g_wifi_ssid) < 2) {
        Serial.println("[TIME/CORE0] WiFi SSID not configured (check /wifi.cfg on SD)");
        
        if (rtc_had_valid_time) {
            // RTC time available, no WiFi - use RTC time
            Serial.println("[TIME/CORE0] Using RTC time (no WiFi config)");
            g_time_state.rtc_active = true;
            g_time_state.time_available = true;
            g_time_state.sync_status = TIME_SYNC_OK;
            updateTime();
        } else {
            // No RTC time, no WiFi config - FAIL
            Serial.println("[TIME/CORE0] NO TIME AVAILABLE - no RTC, no WiFi");
            g_time_state.sync_status = TIME_SYNC_FAILED;
            g_time_state.time_available = false;
            strcpy(g_time_state.time_string, "N/A");
        }
        
        g_time_sync_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    
    // Connect to WiFi
    g_time_state.sync_status = TIME_SYNC_CONNECTING;
    Serial.printf("[TIME/CORE0] Connecting to WiFi: %s\n", g_wifi_ssid);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(g_wifi_ssid, g_wifi_password);
    
    uint32_t start_ms = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start_ms > WIFI_CONNECT_TIMEOUT_MS) {
            Serial.println("[TIME/CORE0] WiFi connection timeout");
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            
            if (rtc_had_valid_time) {
                // WiFi failed but RTC has time - use RTC
                Serial.println("[TIME/CORE0] WiFi failed - keeping RTC time");
                g_time_state.rtc_active = true;
                g_time_state.time_available = true;
                g_time_state.sync_status = TIME_SYNC_OK;
                updateTime();
            } else {
                // WiFi failed and no RTC time - FAIL
                Serial.println("[TIME/CORE0] WiFi failed - NO TIME AVAILABLE");
                g_time_state.sync_status = TIME_SYNC_FAILED;
                g_time_state.time_available = false;
                strcpy(g_time_state.time_string, "N/A");
            }
            
            g_time_sync_task_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    Serial.printf("[TIME/CORE0] WiFi connected: %s\n", WiFi.localIP().toString().c_str());
    
    // Try NTP servers in sequence
    g_time_state.sync_status = TIME_SYNC_SYNCING;
    const char* servers[] = { NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3 };
    bool synced = false;
    
    for (int i = 0; i < 3 && !synced; i++) {
        synced = tryNTPSync(servers[i]);
    }
    
    // Handle NTP result
    if (synced) {
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
            Serial.printf("[TIME/CORE0] NTP time: %02d/%02d/%04d %02d:%02d:%02d\n",
                timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_year + 1900,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            
            // *** WRITE NTP TIME TO RTC ***
            if (g_sd_state.rtc_available) {
                if (writeRTC(&timeinfo)) {
                    clearRTCOSFlag();  // Mark time as valid
                    Serial.println("[TIME/CORE0] RTC updated from NTP");
                    g_time_state.rtc_active = true;
                } else {
                    Serial.println("[TIME/CORE0] Failed to write to RTC");
                }
            }
            
            g_time_state.wifi_time_active = true;
            g_time_state.time_available = true;
            g_time_state.sync_status = TIME_SYNC_OK;
            updateTime();
            Serial.printf("[TIME/CORE0] Time synced: %s\n", g_time_state.time_string);
        }
    } else {
        // NTP failed
        Serial.println("[TIME/CORE0] All NTP servers failed");
        
        if (rtc_had_valid_time) {
            // NTP failed but RTC has time - use RTC
            Serial.println("[TIME/CORE0] NTP failed - keeping RTC time");
            g_time_state.rtc_active = true;
            g_time_state.time_available = true;
            g_time_state.sync_status = TIME_SYNC_OK;
            updateTime();
        } else {
            // NTP failed and no RTC time - FAIL
            g_time_state.sync_status = TIME_SYNC_FAILED;
            g_time_state.time_available = false;
            strcpy(g_time_state.time_string, "N/A");
        }
    }
    
    // Disconnect WiFi to save power
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("[TIME/CORE0] WiFi disconnected");
    
    // Task complete
    g_time_sync_task_handle = NULL;
    vTaskDelete(NULL);
}

// Update current time from best available source
void updateTime() {
    // Take mutex to prevent race conditions with SD logging
    if (g_time_mutex && xSemaphoreTake(g_time_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bool success = false;
        struct tm timeinfo;
        
        // Priority 1: DS3231 RTC
        if (g_sd_state.rtc_available) {
            if (readRTC(&timeinfo)) {
                g_time_state.current_time = timeinfo;
                g_time_state.rtc_active = true;
                success = true;
            } else {
                g_time_state.rtc_active = false;
            }
        }
        
        // Priority 2: WiFi/NTP time (cached in ESP32 internal RTC)
        if (!success && g_time_state.wifi_time_active) {
            if (getLocalTime(&timeinfo)) {
                g_time_state.current_time = timeinfo;
                success = true;
            }
        }
        
        g_time_state.time_available = success;
        g_time_state.last_update_ms = millis();
        
        // Format time string based on state
        if (success) {
            strftime(g_time_state.time_string, sizeof(g_time_state.time_string),
                     "%m/%d/%Y %H:%M:%S", &g_time_state.current_time);
        } else if (g_time_state.sync_status == TIME_SYNC_CONNECTING) {
            strcpy(g_time_state.time_string, "WIFI...");
        } else if (g_time_state.sync_status == TIME_SYNC_SYNCING) {
            strcpy(g_time_state.time_string, "NTP...");
        } else if (g_time_state.sync_status == TIME_SYNC_FAILED) {
            // Keep the failure message already set
        } else {
            strcpy(g_time_state.time_string, "N/A");    // NO TIME
        }
        
        xSemaphoreGive(g_time_mutex);
    }
}

// Initialize timekeeping - NON-BLOCKING
// RTC check is immediate, WiFi sync runs in background task
// Initialize timekeeping - Implements user's specified logic:
// 1. Check if RTC has valid time
// 2. Always try WiFi sync (even if RTC time exists) to keep RTC accurate
// 3. Fallback logic: RTC → WiFi → N/A
void initTimeKeeping() {
    Serial.println("[TIME] Initializing timekeeping...");
    
    // Create mutex for thread-safe access to g_time_state
    if (!g_time_mutex) {
        g_time_mutex = xSemaphoreCreateMutex();
    }
    
    g_time_state.sync_status = TIME_SYNC_IDLE;
    g_time_state.time_available = false;
    g_time_state.rtc_active = false;
    g_time_state.wifi_time_active = false;
    strcpy(g_time_state.time_string, "---");
    
    // Load WiFi config from SD card
    loadWifiConfig();
    
    // IMPORTANT: Always detect RTC here, even if SD card failed
    // RTC is on I2C and works independently of SD card
    // This ensures g_sd_state.rtc_available is set correctly
    if (!g_sd_state.rtc_available) {
        g_sd_state.rtc_available = sdDetectRTC();
    }
    
    // Check RTC status
    if (g_sd_state.rtc_available) {
        Serial.println("[TIME] DS3231 RTC detected on I2C");
        
        if (isRTCTimeValid()) {
            // RTC has valid time - use it immediately
            // WiFi sync will run in background and update if successful
            struct tm timeinfo;
            if (readRTC(&timeinfo)) {
                g_time_state.current_time = timeinfo;
                g_time_state.time_available = true;
                g_time_state.rtc_active = true;
                strftime(g_time_state.time_string, sizeof(g_time_state.time_string),
                         "%m/%d/%Y %H:%M:%S", &timeinfo);
                Serial.printf("[TIME] RTC time valid: %s\n", g_time_state.time_string);
                Serial.println("[TIME] Will attempt WiFi sync in background to update RTC");
            }
        } else {
            Serial.println("[TIME] RTC time not valid (first use or battery was dead)");
            Serial.println("[TIME] Will set RTC from WiFi/NTP if available");
        }
    } else {
        Serial.println("[TIME] No RTC detected - relying on WiFi/NTP only");
    }
    
    // Always start background WiFi sync task
    // - If RTC had valid time: sync updates RTC to keep it accurate
    // - If RTC had no time: sync sets RTC for first time
    // - If no RTC: sync provides WiFi time only
    if (!g_time_state.time_available) {
        strcpy(g_time_state.time_string, "SYNC...");
    }
    
    BaseType_t result = xTaskCreatePinnedToCore(
        timeSyncTask,           
        "TimeSyncTask",         
        4096,                   
        NULL,                   
        1,                      
        &g_time_sync_task_handle,
        0                       
    );
    
    if (result == pdPASS) {
        Serial.println("[TIME] Background sync task started on Core 0");
    } else {
        Serial.println("[TIME] ERROR: Failed to create sync task");
        if (!g_time_state.time_available) {
            g_time_state.sync_status = TIME_SYNC_FAILED;
            strcpy(g_time_state.time_string, "TASK ERR");
        }
    }
}

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
            snprintf(buf, buf_size, "%luK", kb);
        }
        else {
            snprintf(buf, buf_size, "%luM", kb / 1024);
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

// USB MSC state
static volatile bool g_usb_connected = false;  // Set when host connects

// USB MSC callbacks
static int32_t onMscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
    g_usb_connected = true;  // PC is reading - definitely connected!
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
    g_usb_connected = true;  // Host has connected!
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

    // Select SD via IO expander (real CS), then call SD.begin() with dummy pin
    exio_set(EXIO_SD_CS, false);  // CS LOW = selected
    delay(10);

    // GPIO6 is unused - safe as dummy. Avoid: GPIO15 (RS485), GPIO46 (HSYNC)
    pinMode(6, OUTPUT);
    digitalWrite(6, HIGH);  // Keep dummy high

    // Initialize SD card using Arduino library (for card info)
    if (!SD.begin(6, SPI, SD_SPI_FREQ)) {
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
    const char* cardTypeName = "UNKNOWN";
    switch (cardType) {
    case CARD_MMC:  cardTypeName = "MMC";  break;
    case CARD_SD:   cardTypeName = "SD";   break;
    case CARD_SDHC: cardTypeName = "SDHC"; break;
    }
    
    gfx->fillRect(100, 262, 600, 100, GRAY_BG);
    gfx->setCursor(245, 272);
    gfx->setTextColor(WHITE);
    char infoLine[64];
    snprintf(infoLine, sizeof(infoLine), "Card: %s  Size: %llu MB", cardTypeName, cardSize / (1024 * 1024));
    gfx->print(infoLine);
    gfx->setCursor(226, 309);
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

    // Give USB stack time to initialize before resetting the flag
    delay(100);
    
    // Reset connection flag AFTER USB.begin() - callbacks during init don't count
    g_usb_connected = false;

    // Show waiting message with blinking dots to indicate activity
    // Note: Minimal display updates to prevent SPI/USB conflicts
    gfx->fillRect(100, 342, 600, 40, GRAY_BG);
    gfx->setCursor(250, 352);
    gfx->setTextColor(WHITE);
    gfx->print("Waiting for USB host...");

    // Wait for USB host to connect (detected via onStartStop or onRead callback)
    // Blink dots to show we're alive, but minimize display updates
    int dot_count = 0;
    while (!g_usb_connected) {
        delay(500);  // Check every 500ms
        
        // Update dots to show activity (minimal update)
        dot_count = (dot_count + 1) % 4;
        gfx->fillRect(502, 352, 50, 20, GRAY_BG);  // Clear dot area only
        gfx->setCursor(502, 352);
        for (int i = 0; i < dot_count; i++) gfx->print(".");
    }

    // Clear the "Waiting for USB host..." text before showing "USB Ready!"
    // Use larger clear area to ensure complete removal
    gfx->fillRect(50, 340, 700, 50, GRAY_BG);

    // USB is now connected - start blinking "USB Ready!"
    bool blink = true;  // Start with text visible
    while (1) {
        // Draw first, then delay (so text appears immediately)
        gfx->fillRect(50, 340, 700, 50, GRAY_BG);
        gfx->setCursor(340, 352);  // Centered position for "USB Ready!"
        gfx->setTextColor(blink ? WHITE : GRAY_BG, GRAY_BG);
        gfx->print("USB Ready!");
        
        delay(500);
        blink = !blink;
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
    if (psi == CHART_NO_DATA) return;  // Skip no-data points
    
    bool is_critical = (psi < OIL_PRESS_ValueCriticalLow) || (psi > OIL_PRESS_ValueCriticalAbsolute);

    if (is_critical && psi > 0) {
        fill_dsc->color = g_critical_blink_phase ? lv_color_hex(0xFFFFFF) : lv_color_hex(hexRed);
    }
    else {
        fill_dsc->color = lv_color_hex(hexRed);
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
    if (temp_f == CHART_NO_DATA) return;  // Skip no-data points
    
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
    if (temp_f == CHART_NO_DATA) return;  // Skip no-data points
    
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
    if (trust == CHART_NO_DATA) return;  // Skip no-data points
    
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

#define DOUBLE_TAP_TIMEOUT_MS 400  // 400ms window for double-tap detection

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

//=================================================================
// TOAST NOTIFICATION SYSTEM
// Shows temporary messages on screen that auto-disappear
//=================================================================

// Toast auto-hide callback
static void toast_hide_cb(lv_timer_t* t) {
    LV_UNUSED(t);
    if (g_toast_obj) {
        lv_obj_delete(g_toast_obj);
        g_toast_obj = NULL;
    }
    g_toast_timer = NULL;
}

// Hide toast immediately (used when system recovers)
static void hideToast() {
    if (g_toast_obj) {
        lv_obj_delete(g_toast_obj);
        g_toast_obj = NULL;
    }
    if (g_toast_timer) {
        lv_timer_delete(g_toast_timer);
        g_toast_timer = NULL;
    }
}

// Show a toast notification
// msg: Text to display
// color: Background color (e.g., 0x2E7D32 for green success)
// duration_ms: How long to show (0 = use default TOAST_DISPLAY_MS)
static void showToast(const char* msg, uint32_t color, uint32_t duration_ms) {
    if (!ui_Screen1) return;
    
    // Remove any existing toast
    if (g_toast_obj) {
        lv_obj_delete(g_toast_obj);
        g_toast_obj = NULL;
    }
    if (g_toast_timer) {
        lv_timer_delete(g_toast_timer);
        g_toast_timer = NULL;
    }
    
    // Create toast container
    g_toast_obj = lv_obj_create(ui_Screen1);
    lv_obj_set_size(g_toast_obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(g_toast_obj, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_bg_color(g_toast_obj, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(g_toast_obj, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(g_toast_obj, 0, 0);
    lv_obj_set_style_border_width(g_toast_obj, 0, 0);
    lv_obj_set_style_pad_hor(g_toast_obj, 30, 0);
    lv_obj_set_style_pad_ver(g_toast_obj, 18, 0);
    lv_obj_remove_flag(g_toast_obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(g_toast_obj, LV_OBJ_FLAG_CLICKABLE);
    
    // Create label inside toast
    lv_obj_t* label = lv_label_create(g_toast_obj);
    lv_label_set_text(label, msg);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    lv_obj_center(label);
    
    // Set timer to auto-hide
    uint32_t dur = (duration_ms > 0) ? duration_ms : TOAST_SUCCESS_MS;
    g_toast_timer = lv_timer_create(toast_hide_cb, dur, NULL);
    lv_timer_set_repeat_count(g_toast_timer, 1);
    
    Serial.printf("[TOAST] %s\n", msg);
}

// Check all "hidden" systems and show appropriate toast
// Green toast if all systems online, Red toast listing failures
static void checkSystemsAndShowToast() {
    // System status flags
    bool sd_card_ok = true;
    bool logs_writing_ok = true;
    bool rtc_ok = true;
    bool time_sync_ok = true;
    bool modbus_ok = true;
    bool oil_pressure_sensor_ok = true;
    bool oil_temp_sensor_ok = true;
    bool trans_temp_sensor_ok = true;
    bool steer_temp_sensor_ok = true;
    bool diff_temp_sensor_ok = true;
    
    // Build list of failures
    char error_msg[256] = "";
    int error_count = 0;
    
    Serial.println("\n[SYSTEM CHECK] ========== Starting System Check ==========");
    
#if ENABLE_SD_LOGGING
    // Check SD Card
    sd_card_ok = g_sd_state.card_present && g_sd_state.initialized;
    Serial.printf("[SYSTEM CHECK] SD Card: %s (present=%d, init=%d)\n", 
                  sd_card_ok ? "OK" : "FAIL", g_sd_state.card_present, g_sd_state.initialized);
    if (!sd_card_ok) {
        if (error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "SD Card offline");
        error_count++;
    }
    
    // Check Logs Writing (only if SD card is OK)
    if (sd_card_ok) {
        logs_writing_ok = g_sd_state.file_open || g_sd_state.log_file_open;
        Serial.printf("[SYSTEM CHECK] Logs Writing: %s (data=%d, log=%d)\n", 
                      logs_writing_ok ? "OK" : "FAIL", g_sd_state.file_open, g_sd_state.log_file_open);
        if (!logs_writing_ok) {
            if (error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Logs writing offline");
            error_count++;
        }
    } else {
        logs_writing_ok = false;  // Can't write logs without SD
        Serial.println("[SYSTEM CHECK] Logs Writing: SKIP (no SD card)");
    }
    
    // Check RTC (HW-084 / DS3231)
    rtc_ok = g_sd_state.rtc_available;
    Serial.printf("[SYSTEM CHECK] RTC (HW-084): %s\n", rtc_ok ? "OK" : "FAIL");
    if (!rtc_ok) {
        if (error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "Time keeper offline");
        error_count++;
    }
    
    // Check Time Sync (fail only if BOTH RTC AND WiFi/NTP failed)
    // If RTC is OK, time_sync is OK even without WiFi
    time_sync_ok = g_time_state.time_available;
    Serial.printf("[SYSTEM CHECK] Time Sync: %s (rtc=%d, wifi=%d, available=%d)\n", 
                  time_sync_ok ? "OK" : "FAIL", 
                  g_time_state.rtc_active, g_time_state.wifi_time_active, g_time_state.time_available);
    // Only report time sync failed if RTC also failed (WiFi alone failing is expected away from home)
    if (!time_sync_ok && !rtc_ok) {
        if (error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "Time sync failed");
        error_count++;
    }
#else
    Serial.println("[SYSTEM CHECK] SD Logging disabled - skipping SD/RTC/Time checks");
#endif

#if ENABLE_MODBUS_SENSORS
    // Check Modbus RTU
    modbus_ok = g_modbus_initialized && g_modbus_comm_ok;
    Serial.printf("[SYSTEM CHECK] Modbus RTU: %s (init=%d, comm=%d)\n", 
                  modbus_ok ? "OK" : "FAIL", g_modbus_initialized, g_modbus_comm_ok);
    if (!modbus_ok) {
        if (error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "Modbus RTU offline");
        error_count++;
    }
    
    // Check Oil Pressure Sensor (only if Modbus is OK)
    if (modbus_ok) {
        oil_pressure_sensor_ok = g_sensor_ch1_connected;
        Serial.printf("[SYSTEM CHECK] Oil Pressure Sensor: %s\n", 
                      oil_pressure_sensor_ok ? "OK" : "FAIL");
        if (!oil_pressure_sensor_ok) {
            if (error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Sensor Oil Pressure offline");
            error_count++;
        }
        
        // Check Oil Temperature Sensor (PRTXI 4-20mA transmitter)
        oil_temp_sensor_ok = g_sensor_ch2_connected;
        Serial.printf("[SYSTEM CHECK] Oil Temp Sensor (PRTXI): %s\n", 
                      oil_temp_sensor_ok ? "OK" : "FAIL");
        if (!oil_temp_sensor_ok) {
            if (error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Sensor Oil Temp offline");
            error_count++;
        }
        
        // Check Trans Temperature Sensor (PRTXI 4-20mA transmitter)
        trans_temp_sensor_ok = g_sensor_ch3_connected;
        Serial.printf("[SYSTEM CHECK] Trans Temp Sensor (PRTXI): %s\n", 
                      trans_temp_sensor_ok ? "OK" : "FAIL");
        if (!trans_temp_sensor_ok) {
            if (error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Sensor Trans Temp offline");
            error_count++;
        }
        
        // Check Steer Temperature Sensor (PRTXI 4-20mA transmitter)
        steer_temp_sensor_ok = g_sensor_ch4_connected;
        Serial.printf("[SYSTEM CHECK] Steer Temp Sensor (PRTXI): %s\n", 
                      steer_temp_sensor_ok ? "OK" : "FAIL");
        if (!steer_temp_sensor_ok) {
            if (error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Sensor Steer Temp offline");
            error_count++;
        }
        
        // Check Diff Temperature Sensor (PRTXI 4-20mA transmitter)
        diff_temp_sensor_ok = g_sensor_ch5_connected;
        Serial.printf("[SYSTEM CHECK] Diff Temp Sensor (PRTXI): %s\n", 
                      diff_temp_sensor_ok ? "OK" : "FAIL");
        if (!diff_temp_sensor_ok) {
            if (error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Sensor Diff Temp offline");
            error_count++;
        }
    } else {
        oil_pressure_sensor_ok = false;
        oil_temp_sensor_ok = false;
        trans_temp_sensor_ok = false;
        steer_temp_sensor_ok = false;
        diff_temp_sensor_ok = false;
        Serial.println("[SYSTEM CHECK] Sensors: SKIP (no Modbus)");
    }
#else
    Serial.println("[SYSTEM CHECK] Modbus disabled - skipping sensor checks");
#endif
    
    Serial.println("[SYSTEM CHECK] ========== System Check Complete ==========");
    Serial.printf("[SYSTEM CHECK] Result: %d error(s)\n\n", error_count);
    
    // Save current status for background monitoring
    g_prev_system_status.sd_card_ok = sd_card_ok;
    g_prev_system_status.logs_writing_ok = logs_writing_ok;
    g_prev_system_status.rtc_ok = rtc_ok;
    g_prev_system_status.time_sync_ok = time_sync_ok;
    g_prev_system_status.modbus_ok = modbus_ok;
    g_prev_system_status.oil_pressure_sensor_ok = oil_pressure_sensor_ok;
    g_prev_system_status.oil_temp_sensor_ok = oil_temp_sensor_ok;
    g_prev_system_status.trans_temp_sensor_ok = trans_temp_sensor_ok;
    g_prev_system_status.steer_temp_sensor_ok = steer_temp_sensor_ok;
    g_prev_system_status.diff_temp_sensor_ok = diff_temp_sensor_ok;
    g_prev_system_status.initialized = true;
    
    // Show appropriate toast
    if (error_count == 0) {
        // All systems online!
        showToast("All Systems Online", TOAST_COLOR_SUCCESS, TOAST_SUCCESS_MS);
    } else {
        // Show error toast with list of failures
        showToast(error_msg, TOAST_COLOR_ERROR, TOAST_ERROR_MS);
    }
}

// Background system monitor - checks for NEW failures AND recoveries
static void backgroundSystemMonitor() {
    // Current system status
    bool sd_card_ok = true;
    bool logs_writing_ok = true;
    bool rtc_ok = true;
    bool time_sync_ok = true;
    bool modbus_ok = true;
    bool oil_pressure_sensor_ok = true;
    bool oil_temp_sensor_ok = true;
    bool trans_temp_sensor_ok = true;
    bool steer_temp_sensor_ok = true;
    bool diff_temp_sensor_ok = true;
    
    // Build list of NEW failures (things that just went offline)
    char error_msg[256] = "";
    int new_error_count = 0;
    int recovery_count = 0;  // Track recoveries to show green toast
    char recovery_msg[256] = "";
    
#if ENABLE_SD_LOGGING
    // Runtime probe: Check if SD card is still accessible
    // Try to get card type - returns CARD_NONE if card was removed
    if (g_sd_state.initialized) {
        uint8_t cardType = SD.cardType();
        if (cardType == CARD_NONE) {
            g_sd_state.card_present = false;
            g_sd_state.initialized = false;
            g_sd_state.file_open = false;
            g_sd_state.log_file_open = false;
        }
    } else {
        // Card is offline - try hot-swap recovery
        if (sdTryReinit()) {
            // Recovery successful - card is back online
            Serial.println("[MONITOR] SD Card hot-swap recovery successful");
        }
    }
    
    sd_card_ok = g_sd_state.card_present && g_sd_state.initialized;
    if (!sd_card_ok && g_prev_system_status.sd_card_ok) {
        if (new_error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "SD Card offline");
        new_error_count++;
        Serial.println("[MONITOR] SD Card went OFFLINE");
    } else if (sd_card_ok && !g_prev_system_status.sd_card_ok) {
        if (recovery_count > 0) strcat(recovery_msg, "\n");
        strcat(recovery_msg, "SD Card back online");
        recovery_count++;
        Serial.println("[MONITOR] SD Card RECOVERED");
    }
    
    if (sd_card_ok) {
        logs_writing_ok = g_sd_state.file_open || g_sd_state.log_file_open;
        if (!logs_writing_ok && g_prev_system_status.logs_writing_ok) {
            if (new_error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Logs writing offline");
            new_error_count++;
            Serial.println("[MONITOR] Logs writing went OFFLINE");
        } else if (logs_writing_ok && !g_prev_system_status.logs_writing_ok) {
            if (recovery_count > 0) strcat(recovery_msg, "\n");
            strcat(recovery_msg, "Logs writing back online");
            recovery_count++;
            Serial.println("[MONITOR] Logs writing RECOVERED");
        }
    } else {
        logs_writing_ok = false;
    }
    
    // Runtime probe: Check if RTC is still accessible on I2C
    Wire.beginTransmission(DS3231_ADDR);
    rtc_ok = (Wire.endTransmission() == 0);
    g_sd_state.rtc_available = rtc_ok;  // Update global state
    
    if (!rtc_ok && g_prev_system_status.rtc_ok) {
        if (new_error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "Time keeper offline");
        new_error_count++;
        Serial.println("[MONITOR] RTC went OFFLINE");
    } else if (rtc_ok && !g_prev_system_status.rtc_ok) {
        if (recovery_count > 0) strcat(recovery_msg, "\n");
        strcat(recovery_msg, "Time keeper back online");
        recovery_count++;
        Serial.println("[MONITOR] RTC RECOVERED");
    }
    
    time_sync_ok = g_time_state.time_available;
    // Only report if both RTC and time sync failed (and wasn't already failed)
    if (!time_sync_ok && !rtc_ok && (g_prev_system_status.time_sync_ok || g_prev_system_status.rtc_ok)) {
        if (new_error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "Time sync failed");
        new_error_count++;
        Serial.println("[MONITOR] Time sync went OFFLINE");
    } else if ((time_sync_ok || rtc_ok) && !g_prev_system_status.time_sync_ok && !g_prev_system_status.rtc_ok) {
        if (recovery_count > 0) strcat(recovery_msg, "\n");
        strcat(recovery_msg, "Time sync back online");
        recovery_count++;
        Serial.println("[MONITOR] Time sync RECOVERED");
    }
#endif

#if ENABLE_MODBUS_SENSORS
    modbus_ok = g_modbus_initialized && g_modbus_comm_ok;
    if (!modbus_ok && g_prev_system_status.modbus_ok) {
        if (new_error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "Modbus RTU offline");
        new_error_count++;
        Serial.println("[MONITOR] Modbus RTU went OFFLINE");
    } else if (modbus_ok && !g_prev_system_status.modbus_ok) {
        if (recovery_count > 0) strcat(recovery_msg, "\n");
        strcat(recovery_msg, "Modbus RTU back online");
        recovery_count++;
        Serial.println("[MONITOR] Modbus RTU RECOVERED");
    }
    
    if (modbus_ok) {
        // Oil Pressure Sensor (CH1)
        oil_pressure_sensor_ok = g_sensor_ch1_connected;
        if (!oil_pressure_sensor_ok && g_prev_system_status.oil_pressure_sensor_ok) {
            if (new_error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Sensor Oil Pressure offline");
            new_error_count++;
            Serial.println("[MONITOR] Oil Pressure Sensor went OFFLINE");
        } else if (oil_pressure_sensor_ok && !g_prev_system_status.oil_pressure_sensor_ok) {
            if (recovery_count > 0) strcat(recovery_msg, "\n");
            strcat(recovery_msg, "Sensor Oil Pressure back online");
            recovery_count++;
            Serial.println("[MONITOR] Oil Pressure Sensor RECOVERED");
        }
        
        // Oil Temperature Sensor (CH2 - PRTXI 4-20mA)
        oil_temp_sensor_ok = g_sensor_ch2_connected;
        if (!oil_temp_sensor_ok && g_prev_system_status.oil_temp_sensor_ok) {
            if (new_error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Sensor Oil Temp offline");
            new_error_count++;
            Serial.println("[MONITOR] Oil Temp Sensor (PRTXI) went OFFLINE");
        } else if (oil_temp_sensor_ok && !g_prev_system_status.oil_temp_sensor_ok) {
            if (recovery_count > 0) strcat(recovery_msg, "\n");
            strcat(recovery_msg, "Sensor Oil Temp back online");
            recovery_count++;
            Serial.println("[MONITOR] Oil Temp Sensor (PRTXI) RECOVERED");
        }
        
        // Trans Temperature Sensor (CH3 - PRTXI 4-20mA)
        trans_temp_sensor_ok = g_sensor_ch3_connected;
        if (!trans_temp_sensor_ok && g_prev_system_status.trans_temp_sensor_ok) {
            if (new_error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Sensor Trans Temp offline");
            new_error_count++;
            Serial.println("[MONITOR] Trans Temp Sensor (PRTXI) went OFFLINE");
        } else if (trans_temp_sensor_ok && !g_prev_system_status.trans_temp_sensor_ok) {
            if (recovery_count > 0) strcat(recovery_msg, "\n");
            strcat(recovery_msg, "Sensor Trans Temp back online");
            recovery_count++;
            Serial.println("[MONITOR] Trans Temp Sensor (PRTXI) RECOVERED");
        }
        
        // Steer Temperature Sensor (CH4 - PRTXI 4-20mA)
        steer_temp_sensor_ok = g_sensor_ch4_connected;
        if (!steer_temp_sensor_ok && g_prev_system_status.steer_temp_sensor_ok) {
            if (new_error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Sensor Steer Temp offline");
            new_error_count++;
            Serial.println("[MONITOR] Steer Temp Sensor (PRTXI) went OFFLINE");
        } else if (steer_temp_sensor_ok && !g_prev_system_status.steer_temp_sensor_ok) {
            if (recovery_count > 0) strcat(recovery_msg, "\n");
            strcat(recovery_msg, "Sensor Steer Temp back online");
            recovery_count++;
            Serial.println("[MONITOR] Steer Temp Sensor (PRTXI) RECOVERED");
        }
        
        // Diff Temperature Sensor (CH5 - PRTXI 4-20mA)
        diff_temp_sensor_ok = g_sensor_ch5_connected;
        if (!diff_temp_sensor_ok && g_prev_system_status.diff_temp_sensor_ok) {
            if (new_error_count > 0) strcat(error_msg, "\n");
            strcat(error_msg, "Sensor Diff Temp offline");
            new_error_count++;
            Serial.println("[MONITOR] Diff Temp Sensor (PRTXI) went OFFLINE");
        } else if (diff_temp_sensor_ok && !g_prev_system_status.diff_temp_sensor_ok) {
            if (recovery_count > 0) strcat(recovery_msg, "\n");
            strcat(recovery_msg, "Sensor Diff Temp back online");
            recovery_count++;
            Serial.println("[MONITOR] Diff Temp Sensor (PRTXI) RECOVERED");
        }
    } else {
        oil_pressure_sensor_ok = false;
        oil_temp_sensor_ok = false;
        trans_temp_sensor_ok = false;
        steer_temp_sensor_ok = false;
        diff_temp_sensor_ok = false;
    }
#endif
    
    // Update previous status
    g_prev_system_status.sd_card_ok = sd_card_ok;
    g_prev_system_status.logs_writing_ok = logs_writing_ok;
    g_prev_system_status.rtc_ok = rtc_ok;
    g_prev_system_status.time_sync_ok = time_sync_ok;
    g_prev_system_status.modbus_ok = modbus_ok;
    g_prev_system_status.oil_pressure_sensor_ok = oil_pressure_sensor_ok;
    g_prev_system_status.oil_temp_sensor_ok = oil_temp_sensor_ok;
    g_prev_system_status.trans_temp_sensor_ok = trans_temp_sensor_ok;
    g_prev_system_status.steer_temp_sensor_ok = steer_temp_sensor_ok;
    g_prev_system_status.diff_temp_sensor_ok = diff_temp_sensor_ok;
    
    // Handle toast display
    if (new_error_count > 0) {
        // Show toast for NEW failures (takes priority)
        showToast(error_msg, TOAST_COLOR_ERROR, TOAST_ERROR_MS);
    } else if (recovery_count > 0) {
        // System(s) recovered - show green recovery toast
        showToast(recovery_msg, TOAST_COLOR_SUCCESS, TOAST_RECOVERY_MS);
    }
}

// Timer callback for background system monitoring
static void system_monitor_timer_cb(lv_timer_t* t) {
    LV_UNUSED(t);
    backgroundSystemMonitor();
}

// Timer callback for delayed system check (runs once after main screen loads)
static void system_check_timer_cb(lv_timer_t* t) {
    LV_UNUSED(t);
    g_system_check_timer = NULL;
    checkSystemsAndShowToast();
    
    // Start background monitoring (repeating timer)
    if (g_system_monitor_timer) {
        lv_timer_delete(g_system_monitor_timer);
    }
    g_system_monitor_timer = lv_timer_create(system_monitor_timer_cb, SYSTEM_MONITOR_INTERVAL_MS, NULL);
    // No repeat count set = runs indefinitely
}

// Called when main screen finishes loading - schedules system check
// Note: onMainScreenLoaded was removed because ui_init() loads Screen1 directly,
// causing LV_EVENT_SCREEN_LOADED to fire before we could register the callback.
// The system check timer is now set up directly in setup() after ui_init().

static void utility_box_single_tap_cb(lv_timer_t* t) {
    LV_UNUSED(t);
    g_util_single_tap_timer = NULL;

    if (g_brightness_level == 255) {
        setBrightness(90);
    }
    else {
        setBrightness(255);
    }

    Serial.printf("[UI] Single-tap: Brightness -> %d%%\n", (g_brightness_level * 100) / 255);
}

static void utility_box_tap_cb(lv_event_t* e) {
    LV_UNUSED(e);

    // If a long-press was just consumed, ignore this tap and clear the flag
    if (g_utility_long_press_consumed) {
        g_utility_long_press_consumed = false;
        Serial.println("[UI] Tap ignored (long-press was consumed)");
        return;
    }

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
                // Re-enable files button when utility box is visible
                if (files_btn) {
                    lv_obj_add_flag(files_btn, LV_OBJ_FLAG_CLICKABLE);
                }
            }
            else {
                lv_obj_set_style_opa(utility_box, LV_OPA_TRANSP, 0);
                // Disable files button when utility box is hidden
                if (files_btn) {
                    lv_obj_remove_flag(files_btn, LV_OBJ_FLAG_CLICKABLE);
                }
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
    
    // If long-press was triggered, mark it as consumed so the upcoming CLICKED event is ignored
    if (g_utility_long_press_triggered) {
        g_utility_long_press_consumed = true;
    }
    
    g_utility_press_start = 0;
    g_utility_long_press_triggered = false;

    // Visual feedback: restore normal background (no transparency)
    if (utility_box) {
        lv_obj_set_style_bg_color(utility_box, lv_color_hex(0x444444), 0);  // Original dark gray
    }
}

//=================================================================
// FILES BUTTON CALLBACK - Opens SD card file browser
//=================================================================
#if ENABLE_FILE_BROWSER
static void files_btn_press_cb(lv_event_t* e) {
    LV_UNUSED(e);
    // Visual feedback: change to darker gray on press
    if (files_btn) {
        lv_obj_set_style_bg_color(files_btn, lv_color_hex(0x8B9A9A), 0);
    }
}

static void files_btn_release_cb(lv_event_t* e) {
    LV_UNUSED(e);
    // Visual feedback: restore original gray
    if (files_btn) {
        lv_obj_set_style_bg_color(files_btn, lv_color_hex(0xC1CDCD), 0);
    }
}

static void files_btn_click_cb(lv_event_t* e) {
    LV_UNUSED(e);
    Serial.println("[UI] FILES button tapped - entering file browser");
    fb_enter();
}
#endif

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
#if ENABLE_SD_LOGGING
            // Flush SD card before mode switch (safety flush)
            sdSafeFlush();
#endif
            resetVehicleData();
            resetSmoothingState();
            resetTapPanelOpacity();  // Reset critical backgrounds
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

static void update_utility_label(int fps, int cpu0_percent, int cpu1_percent) {
    if (!util_labels[0]) return;  // Not initialized yet
    
    char buf[32];
    int bri_percent = (g_brightness_level * 100) / 255;

    // Internal SRAM (fast, limited ~320KB usable)
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t total_internal = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    int sram_percent = (total_internal > 0) ? (100 - (free_internal * 100 / total_internal)) : 0;

    // PSRAM (external, large ~8MB)
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    int psram_percent = (total_psram > 0) ? (100 - (free_psram * 100 / total_psram)) : 0;

    // Critical color (red) vs normal (yellow)
    lv_color_t clr_yellow = lv_color_hex(0xffff00);
    lv_color_t clr_red = lv_color_hex(0xff0000);

    // FPS - always yellow
    snprintf(buf, sizeof(buf), "FPS:  %3d", fps);
    lv_label_set_text(util_labels[UTIL_IDX_FPS], buf);

    // CPU0 - red if >95% (with state tracking to avoid redundant invalidations)
    static bool cpu0_was_critical = false;
    bool cpu0_critical = (cpu0_percent > 95);
    snprintf(buf, sizeof(buf), "CPU0: %3d%%", cpu0_percent);
    lv_label_set_text(util_labels[UTIL_IDX_CPU0], buf);
    if (cpu0_critical != cpu0_was_critical) {
        lv_obj_set_style_text_color(util_labels[UTIL_IDX_CPU0], cpu0_critical ? clr_red : clr_yellow, 0);
        cpu0_was_critical = cpu0_critical;
    }

    // CPU1 - red if >95%
    static bool cpu1_was_critical = false;
    bool cpu1_critical = (cpu1_percent > 95);
    snprintf(buf, sizeof(buf), "CPU1: %3d%%", cpu1_percent);
    lv_label_set_text(util_labels[UTIL_IDX_CPU1], buf);
    if (cpu1_critical != cpu1_was_critical) {
        lv_obj_set_style_text_color(util_labels[UTIL_IDX_CPU1], cpu1_critical ? clr_red : clr_yellow, 0);
        cpu1_was_critical = cpu1_critical;
    }

    // SRAM - red if >95%
    static bool sram_was_critical = false;
    bool sram_critical = (sram_percent > 95);
    snprintf(buf, sizeof(buf), "SRAM: %3d%%", sram_percent);
    lv_label_set_text(util_labels[UTIL_IDX_SRAM], buf);
    if (sram_critical != sram_was_critical) {
        lv_obj_set_style_text_color(util_labels[UTIL_IDX_SRAM], sram_critical ? clr_red : clr_yellow, 0);
        sram_was_critical = sram_critical;
    }

    // PSRAM - red if >95%
    static bool psram_was_critical = false;
    bool psram_critical = (psram_percent > 95);
    snprintf(buf, sizeof(buf), "PSRAM:%3d%%", psram_percent);
    lv_label_set_text(util_labels[UTIL_IDX_PSRAM], buf);
    if (psram_critical != psram_was_critical) {
        lv_obj_set_style_text_color(util_labels[UTIL_IDX_PSRAM], psram_critical ? clr_red : clr_yellow, 0);
        psram_was_critical = psram_critical;
    }

    // BRI - always yellow
    snprintf(buf, sizeof(buf), "BRI:  %3d%%", bri_percent);
    lv_label_set_text(util_labels[UTIL_IDX_BRI], buf);

#if ENABLE_SD_LOGGING
    // SD status - always yellow
    char sd_status[16];
    sdGetStatusString(sd_status, sizeof(sd_status));
    snprintf(buf, sizeof(buf), "SD:   %s", sd_status);
    lv_label_set_text(util_labels[UTIL_IDX_SD], buf);

    // TIME - red if N/A (with mutex protection for thread safety)
    bool time_critical = false;
    bool time_avail = false;
    struct tm time_copy;
    char time_str_copy[24] = "N/A";
    
    if (g_time_mutex && xSemaphoreTake(g_time_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        time_avail = g_time_state.time_available;
        time_copy = g_time_state.current_time;
        strncpy(time_str_copy, g_time_state.time_string, sizeof(time_str_copy) - 1);
        xSemaphoreGive(g_time_mutex);
    }
    time_critical = !time_avail && strcmp(time_str_copy, "N/A") == 0;
    
    if (time_avail) {
        strftime(buf, sizeof(buf), "TIME: %H:%M:%S", &time_copy);
    } else {
        snprintf(buf, sizeof(buf), "TIME: %s", time_str_copy);
    }
    lv_label_set_text(util_labels[UTIL_IDX_TIME], buf);
    
    // Only update color on state change
    static bool time_was_critical = false;
    if (time_critical != time_was_critical) {
        lv_obj_set_style_text_color(util_labels[UTIL_IDX_TIME], time_critical ? clr_red : clr_yellow, 0);
        time_was_critical = time_critical;
    }

    // DATE - always yellow (only shown when time available)
    if (time_avail) {
        strftime(buf, sizeof(buf), "      %m/%d/%y", &time_copy);
        lv_label_set_text(util_labels[UTIL_IDX_DATE], buf);
    } else {
        lv_label_set_text(util_labels[UTIL_IDX_DATE], "");
    }
#endif
}

#pragma endregion Utility Box Callbacks

//-----------------------------------------------------------------

#pragma region Unit Tap Panel Callbacks

// Note: Press/release visual feedback removed - panels now show critical state

// Pressure tap - cycles PSI/Bar/kPa
static void oil_press_tap_cb(lv_event_t* e) {
    LV_UNUSED(e);
    g_pressure_unit = (PressureUnit)((g_pressure_unit + 1) % 4);
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
        if (ui_OIL_TEMP_Value) {
            snprintf(buf, sizeof(buf), "---°%s", unit);
            lv_label_set_text(ui_OIL_TEMP_Value, buf);
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
        if (ui_W_TEMP_Value) {
            snprintf(buf, sizeof(buf), "---°%s", unit);
            lv_label_set_text(ui_W_TEMP_Value, buf);
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
        if (ui_TRAN_TEMP_Value) {
            snprintf(buf, sizeof(buf), "---°%s", unit);
            lv_label_set_text(ui_TRAN_TEMP_Value, buf);
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
        if (ui_STEER_TEMP_Value) {
            snprintf(buf, sizeof(buf), "---°%s", unit);
            lv_label_set_text(ui_STEER_TEMP_Value, buf);
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
        if (ui_DIFF_TEMP_Value) {
            snprintf(buf, sizeof(buf), "---°%s", unit);
            lv_label_set_text(ui_DIFF_TEMP_Value, buf);
        }
    }
}

// Update tap box visibility - panels are always transparent when not critical
// (critical state is handled in updateUI)
void updateTapBoxVisibility() {
    // No-op - panel visibility is now controlled by critical state in updateUI()
}

// Configure a SquareLine panel for tap detection
void configureTapPanel(lv_obj_t* panel, lv_event_cb_t cb) {
    if (!panel) return;
    
    // Make it clickable
    lv_obj_add_flag(panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    
    // Remove any border that might show
    lv_obj_set_style_border_width(panel, 0, 0);
    
    // Background is already red (0xFF0000) from SquareLine, opacity 0
    // This will be set to LV_OPA_COVER when critical
    
    // Add click handler
    lv_obj_add_event_cb(panel, cb, LV_EVENT_CLICKED, NULL);
}

void setupUnitTapHandlers() {
    // Configure the SquareLine-created Value Tap Panels
    configureTapPanel(ui_OIL_PRESS_Value_Tap_Panel, oil_press_tap_cb);
    configureTapPanel(ui_OIL_TEMP_Value_Tap_Panel, oil_temp_tap_cb);
    configureTapPanel(ui_W_TEMP_Value_Tap_Panel, water_temp_tap_cb);
    configureTapPanel(ui_TRAN_TEMP_Value_Tap_Panel, trans_temp_tap_cb);
    configureTapPanel(ui_STEER_TEMP_Value_Tap_Panel, steer_temp_tap_cb);
    configureTapPanel(ui_DIFF_TEMP_Value_Tap_Panel, diff_temp_tap_cb);
    // Note: FUEL_TRUST doesn't have a tap handler (no unit conversion)
    
    Serial.println("[UI] Value Tap Panels configured");
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

    // Use LVGL's optimized byte-swap (8-pixel loop unrolling)
    lv_draw_sw_rgb565_swap(px_map, len);

    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t*)px_map, w, h);

    // Count flushes (for diagnostics)
    flush_count++;

    // Count REAL frames: only when this is the last flush of the current refresh
    // LVGL can flush multiple rectangles per frame, so we only count complete refreshes
#if LVGL_VERSION_MAJOR >= 9
    if (lv_display_flush_is_last(disp)) {
        frame_count++;
    }
#else
    if (lv_disp_flush_is_last((lv_disp_t*)disp)) {
        frame_count++;
    }
#endif

    lv_display_flush_ready(disp);
}

#pragma endregion LVGL Callbacks

//-----------------------------------------------------------------

#pragma region Touch Callbacks - DUAL CORE

#define MAX_CONSECUTIVE_INVALID 50
#define TOUCH_RESET_COOLDOWN_MS 5000
#define TOUCH_POLL_INTERVAL_MS 10   // Poll touch every 10ms on Core 0
#define TOUCH_RELEASE_DEBOUNCE 2     // Require 2 consecutive "no touch" reads before releasing (20ms)
#define TOUCH_RESET_MAX_RETRIES 3    // Max reset attempts before giving up
#define GT911_I2C_ADDR 0x5D          // GT911 default I2C address

// I2C bus recovery - clocks out stuck transactions WITHOUT calling Wire.end()
// Wire.end() causes heap corruption on ESP32-S3 when other tasks use I2C
static void i2cBusRecovery() {
    Serial.println("[TOUCH/CORE1] I2C bus recovery...");
    
    // Use a simple approach: just delay and let any stuck transaction timeout
    // The ESP32 I2C driver has built-in timeout handling
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Try to unstick by sending a few dummy reads
    for (int i = 0; i < 3; i++) {
        Wire.beginTransmission(GT911_I2C_ADDR);
        Wire.endTransmission(true);  // Send STOP
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    Serial.println("[TOUCH/CORE1] I2C bus recovery complete");
}

// Verify GT911 is responding AND functional by reading product ID
static bool verifyGT911() {
    // First check basic I2C ACK
    Wire.beginTransmission(GT911_I2C_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[TOUCH/CORE1] GT911 no ACK");
        return false;
    }
    
    // Read Product ID register (0x8140-0x8143) to verify chip is functional
    Wire.beginTransmission(GT911_I2C_ADDR);
    Wire.write(0x81);  // High byte of register address
    Wire.write(0x40);  // Low byte - Product ID register
    if (Wire.endTransmission(false) != 0) {
        Serial.println("[TOUCH/CORE1] GT911 register write failed");
        return false;
    }
    
    uint8_t bytesRead = Wire.requestFrom(GT911_I2C_ADDR, (uint8_t)4);
    if (bytesRead < 4) {
        Serial.printf("[TOUCH/CORE1] GT911 read failed (got %d bytes)\n", bytesRead);
        return false;
    }
    
    // Read and verify product ID (should be "911" in ASCII)
    char productId[5] = {0};
    for (int i = 0; i < 4 && Wire.available(); i++) {
        productId[i] = Wire.read();
    }
    
    // GT911 should return "911\0" or similar
    if (productId[0] == '9' && productId[1] == '1' && productId[2] == '1') {
        Serial.printf("[TOUCH/CORE1] GT911 verified (ID: %s)\n", productId);
        return true;
    }
    
    Serial.printf("[TOUCH/CORE1] GT911 bad product ID: 0x%02X%02X%02X%02X\n", 
                  productId[0], productId[1], productId[2], productId[3]);
    return false;
}

// Core 1 Touch Task - polls GT911 and updates shared state
// Runs on Core 1 to avoid I2C bus contention with RTC/SD on Core 0
void touchTask(void* parameter) {
    Serial.println("[CORE1] Touch task started");
    
    // Wait for GT911 to fully stabilize after startup reset
    // This delay prevents false "stuck" detection during initialization
    // and allows Core 0 startup I2C traffic (RTC, WiFi) to complete
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    // Verify GT911 is ready before starting polling
    int startup_retries = 0;
    while (!verifyGT911() && startup_retries < 5) {
        Serial.printf("[TOUCH/CORE1] Startup: GT911 not ready, attempt %d/5\n", startup_retries + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        startup_retries++;
    }
    
    if (startup_retries >= 5) {
        Serial.println("[TOUCH/CORE1] WARNING: GT911 not responding at startup");
    } else {
        Serial.println("[TOUCH/CORE1] GT911 ready, starting polling");
    }
    
    static bool was_touched = false;
    static uint32_t local_consecutive_invalid = 0;
    static uint32_t local_last_reset = 0;
    static uint8_t release_debounce_count = 0;  // Debounce counter for release
    static int16_t last_valid_x = 0, last_valid_y = 0;  // Last known good position
    
    while (true) {
        uint32_t now = millis();
        
        // Read touch controller (I2C operation)
        touch.read();
        

        
        TouchState new_state = {0, 0, false, false, now};
        
        if (touch.isTouched) {
            int raw_x = touch.points[0].x;
            int raw_y = touch.points[0].y;
            

            
            // Validate touch data - GT911 returns 65535 for invalid reads
            // Valid range: raw_x 550-1000, raw_y 50-780
            if (raw_x == 65535 || raw_y == 65535 || raw_x > 1000 || raw_y > 800) {
                local_consecutive_invalid++;
                
                // Hardware reset if stuck
                if (local_consecutive_invalid >= MAX_CONSECUTIVE_INVALID &&
                    (now - local_last_reset) > TOUCH_RESET_COOLDOWN_MS) {
                    Serial.println("[TOUCH/CORE1] Controller stuck - attempting recovery");
                    
                    // CRITICAL: Clear touch state BEFORE reset to prevent spurious release events
                    was_touched = false;
                    release_debounce_count = 0;
                    
                    bool recovery_success = false;
                    
                    for (int attempt = 1; attempt <= TOUCH_RESET_MAX_RETRIES && !recovery_success; attempt++) {
                        Serial.printf("[TOUCH/CORE1] Reset attempt %d/%d\n", attempt, TOUCH_RESET_MAX_RETRIES);
                        
                        // Try I2C bus recovery first (in case SDA is stuck)
                        if (attempt > 1) {
                            i2cBusRecovery();
                        }
                        
                        // Hardware reset sequence
                        exio_set(EXIO_TP_RST, false);
                        vTaskDelay(pdMS_TO_TICKS(20));  // Hold reset low
                        exio_set(EXIO_TP_RST, true);
                        vTaskDelay(pdMS_TO_TICKS(200)); // GT911 needs time to boot (increased from 150)
                        
                        // Re-initialize the touch controller
                        touch.begin();
                        vTaskDelay(pdMS_TO_TICKS(100));  // Allow I2C to stabilize (increased from 50)
                        touch.setRotation(0);
                        
                        // Do a dummy read to flush any stale data
                        touch.read();
                        vTaskDelay(pdMS_TO_TICKS(20));
                        
                        // Verify GT911 is responding AND functional
                        if (verifyGT911()) {
                            // Do another dummy read after verification
                            touch.read();
                            recovery_success = true;
                        } else {
                            Serial.println("[TOUCH/CORE1] GT911 verification failed, retrying...");
                            vTaskDelay(pdMS_TO_TICKS(100));  // Wait before retry
                        }
                    }
                    
                    if (!recovery_success) {
                        Serial.println("[TOUCH/CORE1] WARNING: Recovery FAILED after all attempts");
                        // Try one final I2C bus recovery
                        i2cBusRecovery();
                        touch.begin();
                        touch.setRotation(0);
                    }
                    
                    local_consecutive_invalid = 0;
                    local_last_reset = millis();  // Use fresh timestamp
                }
                
                new_state.valid = false;
                new_state.pressed = false;
            }
            else {
                // Valid touch - convert coordinates
                local_consecutive_invalid = 0;
                release_debounce_count = 0;  // Reset release debounce
                
                // Convert raw coordinates to screen coordinates
                // GT911 raw ranges (from testing): raw_x 600-1000, raw_y 75-750
                // Screen: 800x480, origin at top-left
                // raw_x maps to screen_y (inverted: high raw_x = top of screen)
                // raw_y maps to screen_x (direct: low raw_y = left of screen)
                int screen_x = (raw_y - 75) * 800 / 675;    // raw_y 75-750 -> screen_x 0-800
                int screen_y = (1000 - raw_x) * 480 / 400;  // raw_x 600-1000 -> screen_y 480-0
                
                if (screen_x < 0) screen_x = 0;
                if (screen_x > 799) screen_x = 799;
                if (screen_y < 0) screen_y = 0;
                if (screen_y > 479) screen_y = 479;
                
                new_state.x = screen_x;
                new_state.y = screen_y;
                new_state.pressed = true;
                new_state.valid = true;
                
                // Save last valid position
                last_valid_x = screen_x;
                last_valid_y = screen_y;
                
                // Log on initial touch only
                if (!was_touched) {
                    Serial.printf("[TOUCH/CORE1] raw(%d,%d) -> screen(%d,%d)\n", 
                                  raw_x, raw_y, screen_x, screen_y);
                }
                was_touched = true;
            }
        }
        else {
            local_consecutive_invalid = 0;
            
            // Debounce release - require multiple consecutive "no touch" reads
            if (was_touched) {
                release_debounce_count++;
                if (release_debounce_count < TOUCH_RELEASE_DEBOUNCE) {
                    // Still debouncing - keep reporting as pressed at last position
                    new_state.x = last_valid_x;
                    new_state.y = last_valid_y;
                    new_state.pressed = true;
                    new_state.valid = true;
                } else {
                    // Debounce complete - actually released
                    new_state.valid = true;
                    new_state.pressed = false;
                    Serial.println("[TOUCH/CORE1] Released");
                    was_touched = false;
                    release_debounce_count = 0;
                }
            } else {
                new_state.valid = true;
                new_state.pressed = false;
            }
        }
        
        // Update shared state (thread-safe)
        if (xSemaphoreTake(g_touch_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_touch_state = new_state;
            xSemaphoreGive(g_touch_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_INTERVAL_MS));
    }
}

// LVGL indev callback - reads from shared state (runs on Core 1)
void my_touch_read(lv_indev_t* indev, lv_indev_data_t* data) {
    TouchState local_state;
    
    // Read shared state (thread-safe)
    if (xSemaphoreTake(g_touch_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        local_state = g_touch_state;
        xSemaphoreGive(g_touch_mutex);
    }
    else {
        // Mutex timeout - report no touch
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }
    
    if (local_state.valid && local_state.pressed) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = local_state.x;
        data->point.y = local_state.y;
    }
    else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

#pragma endregion Touch Callbacks - DUAL CORE

//=================================================================
// CRITICAL LABEL ANIMATION HELPER
//=================================================================

// Generic function to manage critical label visibility and blinking
// last_blink_phase is per-label tracking (fixes bug where static var was shared across all 7 labels)
void updateCriticalLabel(lv_obj_t* label, bool is_critical, bool* was_critical, bool* is_visible, uint32_t* exit_time, bool* last_blink_phase) {
    if (!label) return;

    const uint32_t CRITICAL_LINGER_MS = 2000;
    uint32_t now = millis();

    if (is_critical) {
        *exit_time = 0;

        // Only set styles on TRANSITION to critical (not every frame!)
        bool just_became_critical = !*was_critical;
        
        if (!*is_visible) {
            *is_visible = true;
        }
        
#if ENABLE_CRITICAL_LABEL_BLINK
        // Blinking mode: alternate colors (no partial opacity - that's expensive)
        // Only update when blink phase changes (per-label tracking via last_blink_phase parameter)
        if (just_became_critical || g_critical_blink_phase != *last_blink_phase) {
            if (g_critical_blink_phase) {
                // Phase 1: white text on black background
                lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
            } else {
                // Phase 2: red text on black background
                lv_obj_set_style_text_color(label, lv_color_hex(0xFF0000), LV_PART_MAIN);
            }
            *last_blink_phase = g_critical_blink_phase;
        }
        // Only set bg/opa on transition
        if (just_became_critical) {
            lv_obj_set_style_bg_color(label, lv_color_hex(0x000000), LV_PART_MAIN);
            lv_obj_set_style_text_opa(label, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(label, LV_OPA_COVER, LV_PART_MAIN);
        }
#else
        // Static mode: ONLY set styles on transition to critical (huge CPU savings!)
        // last_blink_phase parameter unused in static mode but kept for API consistency
        (void)last_blink_phase;
        if (just_became_critical) {
            lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN);
            lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
            lv_obj_set_style_bg_color(label, lv_color_hex(0x000000), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(label, LV_OPA_COVER, LV_PART_MAIN);
        }
#endif
    }
    else {
        if (*was_critical && *exit_time == 0) {
            *exit_time = now;
        }

        if (*is_visible && *exit_time > 0 && (now - *exit_time) >= CRITICAL_LINGER_MS) {
            *is_visible = false;
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
static bool oil_press_last_blink = false;  // Per-label blink phase tracking

static bool oil_temp_was_critical = false, oil_temp_visible = false;
static uint32_t oil_temp_exit_time = 0;
static bool oil_temp_last_blink = false;

static bool water_temp_was_critical = false, water_temp_visible = false;
static uint32_t water_temp_exit_time = 0;
static bool water_temp_last_blink = false;

static bool trans_temp_was_critical = false, trans_temp_visible = false;
static uint32_t trans_temp_exit_time = 0;
static bool trans_temp_last_blink = false;

static bool steer_temp_was_critical = false, steer_temp_visible = false;
static uint32_t steer_temp_exit_time = 0;
static bool steer_temp_last_blink = false;

static bool diff_temp_was_critical = false, diff_temp_visible = false;
static uint32_t diff_temp_exit_time = 0;
static bool diff_temp_last_blink = false;

static bool fuel_trust_was_critical = false, fuel_trust_visible = false;
static uint32_t fuel_trust_exit_time = 0;
static bool fuel_trust_last_blink = false;

void updateUI() {
    char buf[32];
    const char* pressUnit = getPressureUnitStr(g_pressure_unit);

#if ENABLE_LIGHTWEIGHT_BARS
    bool update_bars_this_frame = shouldUpdateLightweightBars();
#endif

    // Static variables to track last displayed values - only update labels when changed
    static int last_oil_press_display = -9999;
    static int last_oil_temp_pan_display = -9999;
    static int last_oil_temp_cooled_display = -9999;
    static int last_water_temp_hot_display = -9999;
    static int last_water_temp_cooled_display = -9999;
    static int last_trans_temp_hot_display = -9999;
    static int last_trans_temp_cooled_display = -9999;
    static int last_steer_temp_hot_display = -9999;
    static int last_steer_temp_cooled_display = -9999;
    static int last_diff_temp_hot_display = -9999;
    static int last_diff_temp_cooled_display = -9999;
    static int last_fuel_trust_display = -9999;

    // Handle panel opacity reset request (from mode switching)
    if (g_force_panel_reset) {
        // Reset all panels to transparent
        if (ui_OIL_PRESS_Value_Tap_Panel) lv_obj_set_style_bg_opa(ui_OIL_PRESS_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        if (ui_OIL_TEMP_Value_Tap_Panel) lv_obj_set_style_bg_opa(ui_OIL_TEMP_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        if (ui_W_TEMP_Value_Tap_Panel) lv_obj_set_style_bg_opa(ui_W_TEMP_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        if (ui_TRAN_TEMP_Value_Tap_Panel) lv_obj_set_style_bg_opa(ui_TRAN_TEMP_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        if (ui_STEER_TEMP_Value_Tap_Panel) lv_obj_set_style_bg_opa(ui_STEER_TEMP_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        if (ui_DIFF_TEMP_Value_Tap_Panel) lv_obj_set_style_bg_opa(ui_DIFF_TEMP_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        if (ui_FUEL_TRUST_Value_Tap_Panel) lv_obj_set_style_bg_opa(ui_FUEL_TRUST_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        
        // Reset tracking state
        g_oil_press_panel_was_critical = false;
        g_oil_temp_panel_was_critical = false;
        g_water_temp_panel_was_critical = false;
        g_trans_temp_panel_was_critical = false;
        g_steer_temp_panel_was_critical = false;
        g_diff_temp_panel_was_critical = false;
        g_fuel_panel_was_critical = false;
        
        g_force_panel_reset = false;
    }

    // Track last units to force update on unit change
    static PressureUnit last_pressure_unit = (PressureUnit)-1;
    static TempUnit last_oil_temp_unit = (TempUnit)-1;
    static TempUnit last_water_temp_unit = (TempUnit)-1;
    static TempUnit last_trans_temp_unit = (TempUnit)-1;
    static TempUnit last_steer_temp_unit = (TempUnit)-1;
    static TempUnit last_diff_temp_unit = (TempUnit)-1;

    // Check for unit changes - force update if units changed
    bool pressure_unit_changed = (last_pressure_unit != g_pressure_unit);
    bool oil_temp_unit_changed = (last_oil_temp_unit != g_oil_temp_unit);
    bool water_temp_unit_changed = (last_water_temp_unit != g_water_temp_unit);
    bool trans_temp_unit_changed = (last_trans_temp_unit != g_trans_temp_unit);
    bool steer_temp_unit_changed = (last_steer_temp_unit != g_steer_temp_unit);
    bool diff_temp_unit_changed = (last_diff_temp_unit != g_diff_temp_unit);

    // Update unit tracking
    last_pressure_unit = g_pressure_unit;
    last_oil_temp_unit = g_oil_temp_unit;
    last_water_temp_unit = g_water_temp_unit;
    last_trans_temp_unit = g_trans_temp_unit;
    last_steer_temp_unit = g_steer_temp_unit;
    last_diff_temp_unit = g_diff_temp_unit;

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
#if ENABLE_BARS
        if (ui_OIL_PRESS_Bar) {
            // Bar always uses PSI internally for range
            lv_bar_set_value(ui_OIL_PRESS_Bar, pressure_psi, LV_ANIM_OFF);
            // Only update bar color on critical state change
            bool critical = isOilPressureCritical();
            static bool oil_press_bar_was_critical = false;
            if (critical != oil_press_bar_was_critical) {
                lv_obj_set_style_bg_color(ui_OIL_PRESS_Bar,
                    critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                    LV_PART_INDICATOR);
                oil_press_bar_was_critical = critical;
            }
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(0, pressure_psi);
#endif

        // Only update label if value changed OR unit changed
        if (ui_OIL_PRESS_Value && (display_val != last_oil_press_display || pressure_unit_changed)) {
            if (g_pressure_unit == PRESS_KPA) {
                snprintf(buf, sizeof(buf), "%d %s", display_val, pressUnit);
            }
            else if (g_pressure_unit == PRESS_BAR) {
                snprintf(buf, sizeof(buf), "%.1f %s", smooth_oil_pressure, pressUnit);
            }
            else if (g_pressure_unit == PRESS_ATM) {
                snprintf(buf, sizeof(buf), "%.1f %s", smooth_oil_pressure, pressUnit);
            }
            else {
                snprintf(buf, sizeof(buf), "%d %s", display_val, pressUnit);
            }
            lv_label_set_text(ui_OIL_PRESS_Value, buf);
            last_oil_press_display = display_val;

            // Style label text color based on critical
            bool value_critical = isOilPressureCritical();
            if (value_critical != g_oil_press_panel_was_critical) {
                if (value_critical) {
                    // Critical: black text, panel shows red background
                    lv_obj_set_style_text_color(ui_OIL_PRESS_Value, lv_color_hex(0x000000), 0);
                    if (ui_OIL_PRESS_Value_Tap_Panel) {
                        lv_obj_set_style_bg_opa(ui_OIL_PRESS_Value_Tap_Panel, LV_OPA_COVER, 0);
                    }
                }
                else {
                    // Normal: white text, panel transparent
                    lv_obj_set_style_text_color(ui_OIL_PRESS_Value, lv_color_hex(0xFFFFFF), 0);
                    if (ui_OIL_PRESS_Value_Tap_Panel) {
                        lv_obj_set_style_bg_opa(ui_OIL_PRESS_Value_Tap_Panel, LV_OPA_TRANSP, 0);
                    }
                }
                g_oil_press_panel_was_critical = value_critical;
            }
        }

#if ENABLE_VALUE_CRITICAL
        updateCriticalLabel(ui_OIL_PRESS_VALUE_CRITICAL_Label, isOilPressureCritical(),
            &oil_press_was_critical, &oil_press_visible, &oil_press_exit_time, &oil_press_last_blink);
#endif
    }
    
    // Handle transition to invalid state (sensor/modbus failure)
    static bool oil_press_was_data_valid = true;  // Track previous validity
    if (!g_vehicle_data.oil_pressure_valid && oil_press_was_data_valid) {
        // Just became invalid - update UI once to show "---"
        if (ui_OIL_PRESS_Value) {
            snprintf(buf, sizeof(buf), "--- %s", pressUnit);
            lv_label_set_text(ui_OIL_PRESS_Value, buf);
            lv_obj_set_style_text_color(ui_OIL_PRESS_Value, lv_color_hex(0xFFFFFF), 0);
        }
        
        // Reset panel background
        if (ui_OIL_PRESS_Value_Tap_Panel) {
            lv_obj_set_style_bg_opa(ui_OIL_PRESS_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        }
        g_oil_press_panel_was_critical = false;
        
        // Hide critical label immediately
#if ENABLE_VALUE_CRITICAL
        if (ui_OIL_PRESS_VALUE_CRITICAL_Label) {
            lv_obj_set_style_text_opa(ui_OIL_PRESS_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_OIL_PRESS_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
        }
        oil_press_was_critical = false;
        oil_press_visible = false;
        oil_press_exit_time = 0;
#endif
        
#if ENABLE_LIGHTWEIGHT_BARS
        // Reset bar to 0
        updateLightweightBar(0, 0);
#endif
        
        // Reset smoothing and last display value so it updates properly on reconnect
        last_oil_press_display = -9999;
        smooth_oil_pressure = -1.0f;
    }
    oil_press_was_data_valid = g_vehicle_data.oil_pressure_valid;

    // ----- Oil Temperature -----
    if (g_vehicle_data.oil_temp_valid) {
        const char* oilTempUnit = getTempUnitStr(g_oil_temp_unit);
        float temp_pan_disp = tempToDisplay((float)g_vehicle_data.oil_temp_pan_f, g_oil_temp_unit);
        // Single temperature value per gauge

        if (smooth_oil_temp_f < 0) {
            smooth_oil_temp_f = temp_pan_disp;
        }
        else {
            smooth_oil_temp_f = smooth_oil_temp_f * (1.0f - SMOOTH_FACTOR) + temp_pan_disp * SMOOTH_FACTOR;
        }

#if ENABLE_BARS
        if (ui_OIL_TEMP_Bar) {
            lv_bar_set_value(ui_OIL_TEMP_Bar, g_vehicle_data.oil_temp_pan_f, LV_ANIM_OFF);
            bool critical = (g_vehicle_data.oil_temp_pan_f > OIL_TEMP_ValueCriticalF);
            lv_obj_set_style_bg_color(ui_OIL_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(1, g_vehicle_data.oil_temp_pan_f);
#endif

        if (ui_OIL_TEMP_Value) {
            int pan_display_val = (int)(temp_pan_disp + 0.5f);
            if (pan_display_val != last_oil_temp_pan_display || oil_temp_unit_changed) {
                snprintf(buf, sizeof(buf), "%d°%s", pan_display_val, oilTempUnit);
                lv_label_set_text(ui_OIL_TEMP_Value, buf);
                last_oil_temp_pan_display = pan_display_val;
            }

            // Style label text color based on critical
            bool value_critical = (g_vehicle_data.oil_temp_pan_f > OIL_TEMP_ValueCriticalF);
            static bool oil_temp_was_value_critical = false;
            if (value_critical != oil_temp_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_OIL_TEMP_Value, lv_color_hex(0x000000), 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_OIL_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
                }
                oil_temp_was_value_critical = value_critical;
            }
        }

        // Panel background for critical state
        bool oil_temp_critical = (g_vehicle_data.oil_temp_pan_f > OIL_TEMP_ValueCriticalF);
        if (oil_temp_critical != g_oil_temp_panel_was_critical) {
            if (ui_OIL_TEMP_Value_Tap_Panel) {
                lv_obj_set_style_bg_opa(ui_OIL_TEMP_Value_Tap_Panel, 
                    oil_temp_critical ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            }
            g_oil_temp_panel_was_critical = oil_temp_critical;
        }

#if ENABLE_VALUE_CRITICAL
        bool critical = (g_vehicle_data.oil_temp_pan_f > OIL_TEMP_ValueCriticalF);
        updateCriticalLabel(ui_OIL_TEMP_VALUE_CRITICAL_Label, critical,
            &oil_temp_was_critical, &oil_temp_visible, &oil_temp_exit_time, &oil_temp_last_blink);
#endif
    }
    
    // Handle transition to invalid state for oil temperature
    static bool oil_temp_was_data_valid = true;
    if (!g_vehicle_data.oil_temp_valid && oil_temp_was_data_valid) {
        // Just became invalid - update UI once to show "---" with unit
        const char* oilTempUnit = getTempUnitStr(g_oil_temp_unit);
        char buf[32];
        if (ui_OIL_TEMP_Value) {
            snprintf(buf, sizeof(buf), "---°%s", oilTempUnit);
            lv_label_set_text(ui_OIL_TEMP_Value, buf);
            lv_obj_set_style_text_color(ui_OIL_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
        }
        
        // Reset panel background
        if (ui_OIL_TEMP_Value_Tap_Panel) {
            lv_obj_set_style_bg_opa(ui_OIL_TEMP_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        }
        g_oil_temp_panel_was_critical = false;
        
        // Hide critical label immediately
#if ENABLE_VALUE_CRITICAL
        if (ui_OIL_TEMP_VALUE_CRITICAL_Label) {
            lv_obj_set_style_text_opa(ui_OIL_TEMP_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_OIL_TEMP_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
        }
        oil_temp_was_critical = false;
        oil_temp_visible = false;
        oil_temp_exit_time = 0;
#endif
        
#if ENABLE_LIGHTWEIGHT_BARS
        // Reset bar to 0
        updateLightweightBar(1, 0);
#endif
        
        // Reset smoothing and last display value
        last_oil_temp_pan_display = -9999;
        smooth_oil_temp_f = -1.0f;
    }
    oil_temp_was_data_valid = g_vehicle_data.oil_temp_valid;

    // ----- Water Temperature -----
    if (g_vehicle_data.water_temp_valid) {
        const char* waterTempUnit = getTempUnitStr(g_water_temp_unit);
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.water_temp_hot_f, g_water_temp_unit);

        if (ui_W_TEMP_Value) {
            int hot_display_val = (int)(temp_hot_disp + 0.5f);
            if (hot_display_val != last_water_temp_hot_display || water_temp_unit_changed) {
                snprintf(buf, sizeof(buf), "%d°%s", hot_display_val, waterTempUnit);
                lv_label_set_text(ui_W_TEMP_Value, buf);
                last_water_temp_hot_display = hot_display_val;
            }

            // Style label text color based on critical
            bool value_critical = (g_vehicle_data.water_temp_hot_f > W_TEMP_ValueCritical_F);
            static bool water_temp_was_value_critical = false;
            if (value_critical != water_temp_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_W_TEMP_Value, lv_color_hex(0x000000), 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_W_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
                }
                water_temp_was_value_critical = value_critical;
            }
        }

        // Panel background for critical state
        bool water_temp_critical = (g_vehicle_data.water_temp_hot_f > W_TEMP_ValueCritical_F);
        if (water_temp_critical != g_water_temp_panel_was_critical) {
            if (ui_W_TEMP_Value_Tap_Panel) {
                lv_obj_set_style_bg_opa(ui_W_TEMP_Value_Tap_Panel,
                    water_temp_critical ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            }
            g_water_temp_panel_was_critical = water_temp_critical;
        }
#if ENABLE_BARS
        if (ui_W_TEMP_Bar) {
            lv_bar_set_value(ui_W_TEMP_Bar, g_vehicle_data.water_temp_hot_f, LV_ANIM_OFF);
            bool critical = (g_vehicle_data.water_temp_hot_f > W_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_W_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(2, g_vehicle_data.water_temp_hot_f);
#endif

#if ENABLE_VALUE_CRITICAL
        bool critical = (g_vehicle_data.water_temp_hot_f > W_TEMP_ValueCritical_F);
        updateCriticalLabel(ui_W_TEMP_VALUE_CRITICAL_Label, critical,
            &water_temp_was_critical, &water_temp_visible, &water_temp_exit_time, &water_temp_last_blink);
#endif
    }

    // ----- Trans Temperature -----
    if (g_vehicle_data.trans_temp_valid) {
        const char* transTempUnit = getTempUnitStr(g_trans_temp_unit);
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.trans_temp_hot_f, g_trans_temp_unit);

        if (ui_TRAN_TEMP_Value) {
            int hot_display_val = (int)(temp_hot_disp + 0.5f);
            if (hot_display_val != last_trans_temp_hot_display || trans_temp_unit_changed) {
                snprintf(buf, sizeof(buf), "%d°%s", hot_display_val, transTempUnit);
                lv_label_set_text(ui_TRAN_TEMP_Value, buf);
                last_trans_temp_hot_display = hot_display_val;
            }

            // Style label text color based on critical
            bool value_critical = (g_vehicle_data.trans_temp_hot_f > TRAN_TEMP_ValueCritical_F);
            static bool trans_temp_was_value_critical = false;
            if (value_critical != trans_temp_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_TRAN_TEMP_Value, lv_color_hex(0x000000), 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_TRAN_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
                }
                trans_temp_was_value_critical = value_critical;
            }
        }

        // Panel background for critical state
        bool trans_temp_critical = (g_vehicle_data.trans_temp_hot_f > TRAN_TEMP_ValueCritical_F);
        if (trans_temp_critical != g_trans_temp_panel_was_critical) {
            if (ui_TRAN_TEMP_Value_Tap_Panel) {
                lv_obj_set_style_bg_opa(ui_TRAN_TEMP_Value_Tap_Panel,
                    trans_temp_critical ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            }
            g_trans_temp_panel_was_critical = trans_temp_critical;
        }
#if ENABLE_BARS
        if (ui_TRAN_TEMP_Bar) {
            lv_bar_set_value(ui_TRAN_TEMP_Bar, g_vehicle_data.trans_temp_hot_f, LV_ANIM_OFF);
            bool critical = (g_vehicle_data.trans_temp_hot_f > TRAN_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_TRAN_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(3, g_vehicle_data.trans_temp_hot_f);
#endif

#if ENABLE_VALUE_CRITICAL
        bool critical = (g_vehicle_data.trans_temp_hot_f > TRAN_TEMP_ValueCritical_F);
        updateCriticalLabel(ui_TRAN_TEMP_VALUE_CRITICAL_Label, critical,
            &trans_temp_was_critical, &trans_temp_visible, &trans_temp_exit_time, &trans_temp_last_blink);
#endif
    }
    
    // Handle transition to invalid state for trans temperature
    static bool trans_temp_was_data_valid = true;
    if (!g_vehicle_data.trans_temp_valid && trans_temp_was_data_valid) {
        // Just became invalid - update UI once to show "---" with unit
        const char* transTempUnit = getTempUnitStr(g_trans_temp_unit);
        char buf[32];
        if (ui_TRAN_TEMP_Value) {
            snprintf(buf, sizeof(buf), "---°%s", transTempUnit);
            lv_label_set_text(ui_TRAN_TEMP_Value, buf);
            lv_obj_set_style_text_color(ui_TRAN_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
        }
        
        // Reset panel background
        if (ui_TRAN_TEMP_Value_Tap_Panel) {
            lv_obj_set_style_bg_opa(ui_TRAN_TEMP_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        }
        g_trans_temp_panel_was_critical = false;
        
        // Hide critical label immediately
#if ENABLE_VALUE_CRITICAL
        if (ui_TRAN_TEMP_VALUE_CRITICAL_Label) {
            lv_obj_set_style_text_opa(ui_TRAN_TEMP_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_TRAN_TEMP_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
        }
        trans_temp_was_critical = false;
        trans_temp_visible = false;
        trans_temp_exit_time = 0;
#endif
        
#if ENABLE_LIGHTWEIGHT_BARS
        // Reset bar to 0
        updateLightweightBar(3, 0);
#endif
        
        // Reset last display values
        last_trans_temp_hot_display = -9999;
        last_trans_temp_cooled_display = -9999;
    }
    trans_temp_was_data_valid = g_vehicle_data.trans_temp_valid;

    // ----- Steering Temperature -----
    if (g_vehicle_data.steer_temp_valid) {
        const char* steerTempUnit = getTempUnitStr(g_steer_temp_unit);
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.steer_temp_hot_f, g_steer_temp_unit);

        if (ui_STEER_TEMP_Value) {
            int hot_display_val = (int)(temp_hot_disp + 0.5f);
            if (hot_display_val != last_steer_temp_hot_display || steer_temp_unit_changed) {
                snprintf(buf, sizeof(buf), "%d°%s", hot_display_val, steerTempUnit);
                lv_label_set_text(ui_STEER_TEMP_Value, buf);
                last_steer_temp_hot_display = hot_display_val;
            }

            // Style label text color based on critical
            bool value_critical = (g_vehicle_data.steer_temp_hot_f > STEER_TEMP_ValueCritical_F);
            static bool steer_temp_was_value_critical = false;
            if (value_critical != steer_temp_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_STEER_TEMP_Value, lv_color_hex(0x000000), 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_STEER_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
                }
                steer_temp_was_value_critical = value_critical;
            }
        }

        // Panel background for critical state
        bool steer_temp_critical = (g_vehicle_data.steer_temp_hot_f > STEER_TEMP_ValueCritical_F);
        if (steer_temp_critical != g_steer_temp_panel_was_critical) {
            if (ui_STEER_TEMP_Value_Tap_Panel) {
                lv_obj_set_style_bg_opa(ui_STEER_TEMP_Value_Tap_Panel,
                    steer_temp_critical ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            }
            g_steer_temp_panel_was_critical = steer_temp_critical;
        }
#if ENABLE_BARS
        if (ui_STEER_TEMP_Bar) {
            lv_bar_set_value(ui_STEER_TEMP_Bar, g_vehicle_data.steer_temp_hot_f, LV_ANIM_OFF);
            bool critical = (g_vehicle_data.steer_temp_hot_f > STEER_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_STEER_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(4, g_vehicle_data.steer_temp_hot_f);
#endif

#if ENABLE_VALUE_CRITICAL
        bool critical = (g_vehicle_data.steer_temp_hot_f > STEER_TEMP_ValueCritical_F);
        updateCriticalLabel(ui_STEER_TEMP_VALUE_CRITICAL_Label, critical,
            &steer_temp_was_critical, &steer_temp_visible, &steer_temp_exit_time, &steer_temp_last_blink);
#endif
    }
    
    // Handle transition to invalid state for steer temperature
    static bool steer_temp_was_data_valid = true;
    if (!g_vehicle_data.steer_temp_valid && steer_temp_was_data_valid) {
        // Just became invalid - update UI once to show "---" with unit
        const char* steerTempUnit = getTempUnitStr(g_steer_temp_unit);
        char buf[32];
        if (ui_STEER_TEMP_Value) {
            snprintf(buf, sizeof(buf), "---°%s", steerTempUnit);
            lv_label_set_text(ui_STEER_TEMP_Value, buf);
            lv_obj_set_style_text_color(ui_STEER_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
        }
        
        // Reset panel background
        if (ui_STEER_TEMP_Value_Tap_Panel) {
            lv_obj_set_style_bg_opa(ui_STEER_TEMP_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        }
        g_steer_temp_panel_was_critical = false;
        
        // Hide critical label immediately
#if ENABLE_VALUE_CRITICAL
        if (ui_STEER_TEMP_VALUE_CRITICAL_Label) {
            lv_obj_set_style_text_opa(ui_STEER_TEMP_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_STEER_TEMP_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
        }
        steer_temp_was_critical = false;
        steer_temp_visible = false;
        steer_temp_exit_time = 0;
#endif
        
#if ENABLE_LIGHTWEIGHT_BARS
        // Reset bar to 0
        updateLightweightBar(4, 0);
#endif
        
        // Reset last display values
        last_steer_temp_hot_display = -9999;
        last_steer_temp_cooled_display = -9999;
    }
    steer_temp_was_data_valid = g_vehicle_data.steer_temp_valid;

    // ----- Diff Temperature -----
    if (g_vehicle_data.diff_temp_valid) {
        const char* diffTempUnit = getTempUnitStr(g_diff_temp_unit);
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.diff_temp_hot_f, g_diff_temp_unit);

        if (ui_DIFF_TEMP_Value) {
            int hot_display_val = (int)(temp_hot_disp + 0.5f);
            if (hot_display_val != last_diff_temp_hot_display || diff_temp_unit_changed) {
                snprintf(buf, sizeof(buf), "%d°%s", hot_display_val, diffTempUnit);
                lv_label_set_text(ui_DIFF_TEMP_Value, buf);
                last_diff_temp_hot_display = hot_display_val;
            }

            // Style label text color based on critical
            bool value_critical = (g_vehicle_data.diff_temp_hot_f > DIFF_TEMP_ValueCritical_F);
            static bool diff_temp_was_value_critical = false;
            if (value_critical != diff_temp_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_DIFF_TEMP_Value, lv_color_hex(0x000000), 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_DIFF_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
                }
                diff_temp_was_value_critical = value_critical;
            }
        }

        // Panel background for critical state
        bool diff_temp_critical = (g_vehicle_data.diff_temp_hot_f > DIFF_TEMP_ValueCritical_F);
        if (diff_temp_critical != g_diff_temp_panel_was_critical) {
            if (ui_DIFF_TEMP_Value_Tap_Panel) {
                lv_obj_set_style_bg_opa(ui_DIFF_TEMP_Value_Tap_Panel,
                    diff_temp_critical ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            }
            g_diff_temp_panel_was_critical = diff_temp_critical;
        }
#if ENABLE_BARS
        if (ui_DIFF_TEMP_Bar) {
            lv_bar_set_value(ui_DIFF_TEMP_Bar, g_vehicle_data.diff_temp_hot_f, LV_ANIM_OFF);
            bool critical = (g_vehicle_data.diff_temp_hot_f > DIFF_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_DIFF_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(5, g_vehicle_data.diff_temp_hot_f);
#endif

#if ENABLE_VALUE_CRITICAL
        bool critical = (g_vehicle_data.diff_temp_hot_f > DIFF_TEMP_ValueCritical_F);
        updateCriticalLabel(ui_DIFF_TEMP_VALUE_CRITICAL_Label, critical,
            &diff_temp_was_critical, &diff_temp_visible, &diff_temp_exit_time, &diff_temp_last_blink);
#endif
    }
    
    // Handle transition to invalid state for diff temperature
    static bool diff_temp_was_data_valid = true;
    if (!g_vehicle_data.diff_temp_valid && diff_temp_was_data_valid) {
        // Just became invalid - update UI once to show "---" with unit
        const char* diffTempUnit = getTempUnitStr(g_diff_temp_unit);
        char buf[32];
        if (ui_DIFF_TEMP_Value) {
            snprintf(buf, sizeof(buf), "---°%s", diffTempUnit);
            lv_label_set_text(ui_DIFF_TEMP_Value, buf);
            lv_obj_set_style_text_color(ui_DIFF_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
        }
        
        // Reset panel background
        if (ui_DIFF_TEMP_Value_Tap_Panel) {
            lv_obj_set_style_bg_opa(ui_DIFF_TEMP_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        }
        g_diff_temp_panel_was_critical = false;
        
        // Hide critical label immediately
#if ENABLE_VALUE_CRITICAL
        if (ui_DIFF_TEMP_VALUE_CRITICAL_Label) {
            lv_obj_set_style_text_opa(ui_DIFF_TEMP_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_DIFF_TEMP_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
        }
        diff_temp_was_critical = false;
        diff_temp_visible = false;
        diff_temp_exit_time = 0;
#endif
        
#if ENABLE_LIGHTWEIGHT_BARS
        // Reset bar to 0
        updateLightweightBar(5, 0);
#endif
        
        // Reset last display values
        last_diff_temp_hot_display = -9999;
        last_diff_temp_cooled_display = -9999;
    }
    diff_temp_was_data_valid = g_vehicle_data.diff_temp_valid;

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

#if ENABLE_BARS
        if (ui_FUEL_TRUST_Bar) {
            lv_bar_set_value(ui_FUEL_TRUST_Bar, display_fuel, LV_ANIM_OFF);
            bool critical = (fuel < FUEL_TRUST_ValueCritical);
            lv_obj_set_style_bg_color(ui_FUEL_TRUST_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(6, display_fuel);
#endif

        if (ui_FUEL_TRUST_Value) {
            if (display_fuel != last_fuel_trust_display) {
                snprintf(buf, sizeof(buf), "%d %%", display_fuel);
                lv_label_set_text(ui_FUEL_TRUST_Value, buf);
                last_fuel_trust_display = display_fuel;
            }

            // Style label text color based on critical
            bool value_critical = (fuel < FUEL_TRUST_ValueCritical);
            static bool fuel_trust_was_value_critical = false;
            if (value_critical != fuel_trust_was_value_critical) {
                if (value_critical) {
                    lv_obj_set_style_text_color(ui_FUEL_TRUST_Value, lv_color_hex(0x000000), 0);
                }
                else {
                    lv_obj_set_style_text_color(ui_FUEL_TRUST_Value, lv_color_hex(0xFFFFFF), 0);
                }
                fuel_trust_was_value_critical = value_critical;
            }
        }

        // Panel background for critical state
        bool fuel_critical = (fuel < FUEL_TRUST_ValueCritical);
        if (fuel_critical != g_fuel_panel_was_critical) {
            if (ui_FUEL_TRUST_Value_Tap_Panel) {
                lv_obj_set_style_bg_opa(ui_FUEL_TRUST_Value_Tap_Panel,
                    fuel_critical ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            }
            g_fuel_panel_was_critical = fuel_critical;
        }

#if ENABLE_VALUE_CRITICAL
        bool critical = (fuel < FUEL_TRUST_ValueCritical);
        updateCriticalLabel(ui_FUEL_TRUST_VALUE_CRITICAL_Label, critical,
            &fuel_trust_was_critical, &fuel_trust_visible, &fuel_trust_exit_time, &fuel_trust_last_blink);
#endif
    }
}

//=================================================================
// CHART UPDATE FUNCTION - OPTIMIZED WITH SELECTIVE INVALIDATION
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

    // Push to charts every CHART_BUCKET_MS - also update critical flags
    // Charts always advance to maintain time progression (gaps show "no data")
    if ((now - oil_pressure_bucket_start) >= CHART_BUCKET_MS && chart_series_oil_press) {
        // Chart min is 0 for oil pressure - only show bars for values > 0
        if (oil_pressure_samples > 0) {
            int32_t avg = oil_pressure_sum / oil_pressure_samples;
            if (avg < 0) avg = 0;
            if (avg > 150) avg = 150;
            
            if (avg > 0) {
                // Valid data above chart minimum - show bar
                shift_history(oil_press_history, avg);
                lv_chart_set_next_value(ui_OIL_PRESS_CHART, chart_series_oil_press, avg);
            } else {
                // Value at or below chart minimum - show gap
                shift_history(oil_press_history, CHART_NO_DATA);
                lv_chart_set_next_value(ui_OIL_PRESS_CHART, chart_series_oil_press, LV_CHART_POINT_NONE);
            }
        } else {
            // No samples (sensor invalid) - show gap
            shift_history(oil_press_history, CHART_NO_DATA);
            lv_chart_set_next_value(ui_OIL_PRESS_CHART, chart_series_oil_press, LV_CHART_POINT_NONE);
        }
        
        // Check if ANY point in history is critical
        g_chart_has_critical_oil_press = false;
        for (int i = 0; i < CHART_POINTS; i++) {
            if (oil_press_history[i] != CHART_NO_DATA && oil_press_history[i] > 0 &&
                (oil_press_history[i] < OIL_PRESS_ValueCriticalLow ||
                    oil_press_history[i] > OIL_PRESS_ValueCriticalAbsolute)) {
                g_chart_has_critical_oil_press = true;
                break;
            }
        }
        
        oil_pressure_sum = 0;
        oil_pressure_samples = 0;
        oil_pressure_bucket_start = now;
    }

    if ((now - oil_temp_bucket_start) >= CHART_BUCKET_MS && chart_series_oil_temp) {
        // Chart min is OIL_TEMP_Min_F - only show bars for values > min
        if (oil_temp_samples > 0) {
            int32_t avg = oil_temp_sum / oil_temp_samples;
            
            if (avg > OIL_TEMP_Min_F) {
                // Valid data above chart minimum - show bar
                shift_history(oil_temp_history, avg);
                lv_chart_set_next_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, avg);
            } else {
                // Value at or below chart minimum - show gap
                shift_history(oil_temp_history, CHART_NO_DATA);
                lv_chart_set_next_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, LV_CHART_POINT_NONE);
            }
        } else {
            // No samples (sensor invalid) - show gap
            shift_history(oil_temp_history, CHART_NO_DATA);
            lv_chart_set_next_value(ui_OIL_TEMP_CHART, chart_series_oil_temp, LV_CHART_POINT_NONE);
        }
        
        // Check if ANY point in history is critical
        g_chart_has_critical_oil_temp = false;
        for (int i = 0; i < CHART_POINTS; i++) {
            if (oil_temp_history[i] != CHART_NO_DATA && oil_temp_history[i] > OIL_TEMP_ValueCriticalF) {
                g_chart_has_critical_oil_temp = true;
                break;
            }
        }
        
        oil_temp_sum = 0;
        oil_temp_samples = 0;
        oil_temp_bucket_start = now;
    }

    if ((now - water_temp_bucket_start) >= CHART_BUCKET_MS && chart_series_water_temp) {
        // Chart min is W_TEMP_Min_F - only show bars for values > min
        if (water_temp_samples > 0) {
            int32_t avg = water_temp_sum / water_temp_samples;
            
            if (avg > W_TEMP_Min_F) {
                // Valid data above chart minimum - show bar
                shift_history(water_temp_history, avg);
                lv_chart_set_next_value(ui_W_TEMP_CHART, chart_series_water_temp, avg);
            } else {
                // Value at or below chart minimum - show gap
                shift_history(water_temp_history, CHART_NO_DATA);
                lv_chart_set_next_value(ui_W_TEMP_CHART, chart_series_water_temp, LV_CHART_POINT_NONE);
            }
        } else {
            // No samples (sensor invalid) - show gap
            shift_history(water_temp_history, CHART_NO_DATA);
            lv_chart_set_next_value(ui_W_TEMP_CHART, chart_series_water_temp, LV_CHART_POINT_NONE);
        }
        
        // Check if ANY point in history is critical
        g_chart_has_critical_water_temp = false;
        for (int i = 0; i < CHART_POINTS; i++) {
            if (water_temp_history[i] != CHART_NO_DATA && water_temp_history[i] > W_TEMP_ValueCritical_F) {
                g_chart_has_critical_water_temp = true;
                break;
            }
        }
        
        water_temp_sum = 0;
        water_temp_samples = 0;
        water_temp_bucket_start = now;
    }

    if ((now - transmission_temp_bucket_start) >= CHART_BUCKET_MS && chart_series_transmission_temp) {
        // Chart min is TRAN_TEMP_Min_F - only show bars for values > min
        if (transmission_temp_samples > 0) {
            int32_t avg = transmission_temp_sum / transmission_temp_samples;
            
            if (avg > TRAN_TEMP_Min_F) {
                // Valid data above chart minimum - show bar
                shift_history(transmission_temp_history, avg);
                lv_chart_set_next_value(ui_TRAN_TEMP_CHART, chart_series_transmission_temp, avg);
            } else {
                // Value at or below chart minimum - show gap
                shift_history(transmission_temp_history, CHART_NO_DATA);
                lv_chart_set_next_value(ui_TRAN_TEMP_CHART, chart_series_transmission_temp, LV_CHART_POINT_NONE);
            }
        } else {
            // No samples (sensor invalid) - show gap
            shift_history(transmission_temp_history, CHART_NO_DATA);
            lv_chart_set_next_value(ui_TRAN_TEMP_CHART, chart_series_transmission_temp, LV_CHART_POINT_NONE);
        }
        
        // Check if ANY point in history is critical
        g_chart_has_critical_trans_temp = false;
        for (int i = 0; i < CHART_POINTS; i++) {
            if (transmission_temp_history[i] != CHART_NO_DATA && transmission_temp_history[i] > TRAN_TEMP_ValueCritical_F) {
                g_chart_has_critical_trans_temp = true;
                break;
            }
        }
        
        transmission_temp_sum = 0;
        transmission_temp_samples = 0;
        transmission_temp_bucket_start = now;
    }

    if ((now - steering_temp_bucket_start) >= CHART_BUCKET_MS && chart_series_steering_temp) {
        // Chart min is STEER_TEMP_Min_F - only show bars for values > min
        if (steering_temp_samples > 0) {
            int32_t avg = steering_temp_sum / steering_temp_samples;
            
            if (avg > STEER_TEMP_Min_F) {
                // Valid data above chart minimum - show bar
                shift_history(steering_temp_history, avg);
                lv_chart_set_next_value(ui_STEER_TEMP_CHART, chart_series_steering_temp, avg);
            } else {
                // Value at or below chart minimum - show gap
                shift_history(steering_temp_history, CHART_NO_DATA);
                lv_chart_set_next_value(ui_STEER_TEMP_CHART, chart_series_steering_temp, LV_CHART_POINT_NONE);
            }
        } else {
            // No samples (sensor invalid) - show gap
            shift_history(steering_temp_history, CHART_NO_DATA);
            lv_chart_set_next_value(ui_STEER_TEMP_CHART, chart_series_steering_temp, LV_CHART_POINT_NONE);
        }
        
        // Check if ANY point in history is critical
        g_chart_has_critical_steer_temp = false;
        for (int i = 0; i < CHART_POINTS; i++) {
            if (steering_temp_history[i] != CHART_NO_DATA && steering_temp_history[i] > STEER_TEMP_ValueCritical_F) {
                g_chart_has_critical_steer_temp = true;
                break;
            }
        }
        
        steering_temp_sum = 0;
        steering_temp_samples = 0;
        steering_temp_bucket_start = now;
    }

    if ((now - differencial_temp_bucket_start) >= CHART_BUCKET_MS && chart_series_differencial_temp) {
        // Chart min is DIFF_TEMP_Min_F - only show bars for values > min
        if (differencial_temp_samples > 0) {
            int32_t avg = differencial_temp_sum / differencial_temp_samples;
            
            if (avg > DIFF_TEMP_Min_F) {
                // Valid data above chart minimum - show bar
                shift_history(differencial_temp_history, avg);
                lv_chart_set_next_value(ui_DIFF_TEMP_CHART, chart_series_differencial_temp, avg);
            } else {
                // Value at or below chart minimum - show gap
                shift_history(differencial_temp_history, CHART_NO_DATA);
                lv_chart_set_next_value(ui_DIFF_TEMP_CHART, chart_series_differencial_temp, LV_CHART_POINT_NONE);
            }
        } else {
            // No samples (sensor invalid) - show gap
            shift_history(differencial_temp_history, CHART_NO_DATA);
            lv_chart_set_next_value(ui_DIFF_TEMP_CHART, chart_series_differencial_temp, LV_CHART_POINT_NONE);
        }
        
        // Check if ANY point in history is critical
        g_chart_has_critical_diff_temp = false;
        for (int i = 0; i < CHART_POINTS; i++) {
            if (differencial_temp_history[i] != CHART_NO_DATA && differencial_temp_history[i] > DIFF_TEMP_ValueCritical_F) {
                g_chart_has_critical_diff_temp = true;
                break;
            }
        }
        
        differencial_temp_sum = 0;
        differencial_temp_samples = 0;
        differencial_temp_bucket_start = now;
    }

    if ((now - fuel_trust_start) >= CHART_BUCKET_MS && chart_series_fuel_trust) {
        // Chart min is 0 for fuel trust - only show bars for values > 0
        if (fuel_trust_samples > 0) {
            int32_t avg = fuel_trust_sum / fuel_trust_samples;
            
            if (avg > 0) {
                // Valid data above chart minimum - show bar
                shift_history(fuel_trust_history, avg);
                lv_chart_set_next_value(ui_FUEL_TRUST_CHART, chart_series_fuel_trust, avg);
            } else {
                // Value at or below chart minimum - show gap
                shift_history(fuel_trust_history, CHART_NO_DATA);
                lv_chart_set_next_value(ui_FUEL_TRUST_CHART, chart_series_fuel_trust, LV_CHART_POINT_NONE);
            }
        } else {
            // No samples (sensor invalid) - show gap
            shift_history(fuel_trust_history, CHART_NO_DATA);
            lv_chart_set_next_value(ui_FUEL_TRUST_CHART, chart_series_fuel_trust, LV_CHART_POINT_NONE);
        }
        
        // Check if ANY point in history is critical (low fuel)
        g_chart_has_critical_fuel_trust = false;
        for (int i = 0; i < CHART_POINTS; i++) {
            if (fuel_trust_history[i] != CHART_NO_DATA && fuel_trust_history[i] > 0 && 
                fuel_trust_history[i] < FUEL_TRUST_ValueCritical) {
                g_chart_has_critical_fuel_trust = true;
                break;
            }
        }
        
        fuel_trust_sum = 0;
        fuel_trust_samples = 0;
        fuel_trust_start = now;
    }

    // Update blink phase - ONLY invalidate charts that have critical values
    if (now - g_last_blink_toggle >= CHART_BLINK_INTERVAL_MS) {
        g_critical_blink_phase = !g_critical_blink_phase;
        g_last_blink_toggle = now;

        // Selective invalidation - only charts with critical values need redraw
        if (g_chart_has_critical_oil_press && ui_OIL_PRESS_CHART)
            lv_obj_invalidate(ui_OIL_PRESS_CHART);
        if (g_chart_has_critical_oil_temp && ui_OIL_TEMP_CHART)
            lv_obj_invalidate(ui_OIL_TEMP_CHART);
        if (g_chart_has_critical_water_temp && ui_W_TEMP_CHART)
            lv_obj_invalidate(ui_W_TEMP_CHART);
        if (g_chart_has_critical_trans_temp && ui_TRAN_TEMP_CHART)
            lv_obj_invalidate(ui_TRAN_TEMP_CHART);
        if (g_chart_has_critical_steer_temp && ui_STEER_TEMP_CHART)
            lv_obj_invalidate(ui_STEER_TEMP_CHART);
        if (g_chart_has_critical_diff_temp && ui_DIFF_TEMP_CHART)
            lv_obj_invalidate(ui_DIFF_TEMP_CHART);
        if (g_chart_has_critical_fuel_trust && ui_FUEL_TRUST_CHART)
            lv_obj_invalidate(ui_FUEL_TRUST_CHART);
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
        if (history) history[i] = CHART_NO_DATA;
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
    Serial.println("   370zMonitor v5.0 - Dual-Core");
    Serial.println("========================================");

    if (psramFound()) {
        Serial.printf("✓ PSRAM: %u bytes total, %u free\n", ESP.getPsramSize(), ESP.getFreePsram());
    }
    else {
        Serial.println("✗ WARNING: No PSRAM detected!");
    }

    // Load unit preferences from flash
    loadUnitPreferences();

    // Initialize Modbus RS485 sensors
#if ENABLE_MODBUS_SENSORS
    initModbusSensors();
#endif

    // I2C init
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
    Wire.setTimeOut(50);
    delay(50);

    Serial.println("[1/8] IO Expander...");
    g_ioexp_ok = initIOExtension();
    Serial.printf("      %s\n", g_ioexp_ok ? "OK" : "FAILED");

    Serial.println("[2/8] Touch controller...");
    // Hardware reset GT911 before initialization
    // CRITICAL: On cold boot, GT911 needs time to power up before reset sequence
    // Double-reset cycle ensures reliable startup on both cold boot and warm reset
    
    // Step 1: Power-on stabilization (GT911 needs 5-10ms after power-on)
    // On cold boot this is critical; on warm reset it's harmless
    delay(100);  // Allow GT911 power rails to stabilize
    
    // Step 2: First reset cycle (clears any garbage state from power-on)
    exio_set(EXIO_TP_RST, false);  // Assert reset
    delay(20);                      // Hold reset low
    exio_set(EXIO_TP_RST, true);   // Release reset
    delay(100);                     // Let it partially boot
    
    // Step 3: Second reset cycle (ensures clean initialization)
    exio_set(EXIO_TP_RST, false);  // Assert reset again
    delay(50);                      // Hold reset low longer
    exio_set(EXIO_TP_RST, true);   // Release reset
    delay(350);                     // GT911 full boot time after reset
    
    touch.begin();
    delay(150);  // Allow I2C to stabilize
    touch.setRotation(0);
    
    // Create touch mutex for dual-core architecture
    g_touch_mutex = xSemaphoreCreateMutex();
    if (!g_touch_mutex) {
        Serial.println("      FATAL: Touch mutex creation failed!");
        while (1) delay(100);
    }
    Serial.println("      GT911 initialized + mutex created");

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

    // CRITICAL: Allocate LVGL buffers in internal DMA-capable RAM, NOT PSRAM!
    // PSRAM allocation causes screen tearing/shift because both cores fight for
    // PSRAM bandwidth while the RGB panel is scanning (bounce buffer contention).
    disp_draw_buf1 = (uint8_t*)heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!disp_draw_buf1) {
        Serial.println("      FATAL: Buffer 1 alloc failed (need internal DMA RAM)!");
        Serial.printf("      Requested: %u bytes, Free internal: %u\n", buf_bytes, heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        while (1) delay(100);
    }

    disp_draw_buf2 = (uint8_t*)heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!disp_draw_buf2) {
        Serial.println("      FATAL: Buffer 2 alloc failed (need internal DMA RAM)!");
        Serial.printf("      Requested: %u bytes, Free internal: %u\n", buf_bytes, heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        while (1) delay(100);
    }
    
    Serial.printf("      LVGL buffers: 2x %u bytes in internal DMA RAM\n", buf_bytes);
    Serial.printf("      Internal heap remaining: %u bytes (largest block: %u)\n", 
                  heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

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
    
    // Schedule system check after delay (allows sensors to initialize)
    // Note: ui_init() already loads Screen1, so we set up timers directly here
    // rather than using LV_EVENT_SCREEN_LOADED which fires during ui_init()
    Serial.println("[UI] Scheduling system status check...");
    if (g_system_check_timer) {
        lv_timer_delete(g_system_check_timer);
    }
    g_system_check_timer = lv_timer_create(system_check_timer_cb, SYSTEM_CHECK_DELAY_MS, NULL);
    lv_timer_set_repeat_count(g_system_check_timer, 1);
    
    Serial.println("      OK");

    Serial.println("[7/8] Data providers...");
    initSensors();
    initOBD();
    Serial.println("      OK");

#if ENABLE_SD_LOGGING
    Serial.println("[8/8] SD Card...");
    
    // Create SD queue for dual-core architecture
    g_sd_queue = xQueueCreate(SD_QUEUE_SIZE, sizeof(SDLogEntry));
    if (!g_sd_queue) {
        Serial.println("      WARNING: SD queue creation failed!");
    } else {
        Serial.println("      SD queue created");
    }
    
    // Create serial log queue
    g_serial_log_queue = xQueueCreate(SERIAL_LOG_QUEUE_SIZE, sizeof(SerialLogEntry));
    if (!g_serial_log_queue) {
        Serial.println("      WARNING: Serial log queue creation failed!");
    } else {
        Serial.println("      Serial log queue created");
    }
    
    if (sdInit()) {
        sdTestWrite();
        sdStartSession();
    }
    Serial.println("      OK");
    
    // Initialize timekeeping (RTC or WiFi NTP)
    initTimeKeeping();
    
    // Initialize file browser (tap FILES button in utility box to enter)
#if ENABLE_FILE_BROWSER
    fb_init();
    Serial.println("[FB] File browser ready (tap FILES in utility box)");
#endif
#endif

    // Bar animation speeds
    lv_obj_t* bars[] = { ui_OIL_PRESS_Bar, ui_OIL_TEMP_Bar, ui_FUEL_TRUST_Bar,
                        ui_W_TEMP_Bar, ui_TRAN_TEMP_Bar, ui_STEER_TEMP_Bar, ui_DIFF_TEMP_Bar };
    for (int i = 0; i < 7; i++) {
        if (bars[i]) lv_obj_set_style_anim_duration(bars[i], 0, LV_PART_MAIN);
    }

    // Create utility box
    if (ui_Screen1) {
        utility_box = lv_obj_create(ui_Screen1);
#if ENABLE_SD_LOGGING
#if ENABLE_FILE_BROWSER
        lv_obj_set_size(utility_box, 250, 240);
#else
        lv_obj_set_size(utility_box, 250, 192);  // Height for 9 lines: FPS, CPU0, CPU1, SRAM, PSRAM, BRI, SD, TIME, DATE
#endif
#else
#if ENABLE_FILE_BROWSER
        lv_obj_set_size(utility_box, 250, 240);
#else
        lv_obj_set_size(utility_box, 250, 138);  // Height for 6 lines: FPS, CPU0, CPU1, SRAM, PSRAM, BRI
#endif
#endif
        lv_obj_align(utility_box, LV_ALIGN_TOP_LEFT, 5, 5);
        lv_obj_set_style_bg_color(utility_box, lv_color_hex(0x444444), 0);
        lv_obj_set_style_bg_opa(utility_box, LV_OPA_COVER, 0);  // Fully opaque background (no transparency)
        lv_obj_set_style_opa(utility_box, LV_OPA_TRANSP, 0);    // Start hidden (double-tap to reveal)
        lv_obj_set_style_border_color(utility_box, lv_color_hex(0x444444), 0);
        lv_obj_set_style_border_width(utility_box, 1, 0);
        lv_obj_set_style_radius(utility_box, 0, 0);
        lv_obj_set_style_pad_all(utility_box, 5, 0);
        lv_obj_add_flag(utility_box, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_remove_flag(utility_box, LV_OBJ_FLAG_SCROLLABLE);

        // Event callbacks for tap and long-press
        lv_obj_add_event_cb(utility_box, utility_box_tap_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_add_event_cb(utility_box, utility_box_press_cb, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(utility_box, utility_box_release_cb, LV_EVENT_RELEASED, NULL);

        // CPU load hooks register
        esp_register_freertos_idle_hook_for_cpu(idle_hook_core0, 0);
        esp_register_freertos_idle_hook_for_cpu(idle_hook_core1, 1);

#if ENABLE_FILE_BROWSER
        // FILES button at the top of utility box
        files_btn = lv_obj_create(utility_box);
        lv_obj_set_size(files_btn, 235, 44);
        lv_obj_align(files_btn, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_set_style_bg_color(files_btn, lv_color_hex(0xC1CDCD), 0);  // Gray
        lv_obj_set_style_bg_opa(files_btn, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(files_btn, 0, 0);
        lv_obj_set_style_border_width(files_btn, 0, 0);
        lv_obj_set_style_pad_all(files_btn, 2, 0);
        lv_obj_remove_flag(files_btn, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);  // Start disabled (utility box hidden)
        
        // FILES label inside button
        lv_obj_t* files_lbl = lv_label_create(files_btn);
        lv_label_set_text(files_lbl, "FILES");
        lv_obj_set_style_text_color(files_lbl, lv_color_hex(0x000000), 0);  // Black text
        lv_obj_set_style_text_font(files_lbl, &lv_font_unscii_16, 0);  // Smaller font
        lv_obj_center(files_lbl);
        
        // Event callbacks for press/release visual feedback and click
        lv_obj_add_event_cb(files_btn, files_btn_press_cb, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(files_btn, files_btn_release_cb, LV_EVENT_RELEASED, NULL);
        lv_obj_add_event_cb(files_btn, files_btn_click_cb, LV_EVENT_CLICKED, NULL);
#endif

        // FPS/CPU/BRI/SD labels - individual labels for per-line coloring
        const char* init_texts[] = {
            "FPS:  ---",
            "CPU0: ---%",
            "CPU1: ---%",
            "SRAM: ---%",
            "PSRAM:---%",
            "BRI:  ---%"
#if ENABLE_SD_LOGGING
            , "SD:   ---",
            "TIME: ---",
            "      --/--/--"
#endif
        };
        
        int line_height = 16;  // Font height for lv_font_unscii_16
#if ENABLE_FILE_BROWSER
        int label_y_offset = 48;  // Offset for FILES button row
#else
        int label_y_offset = 0;
#endif
        for (int i = 0; i < UTIL_LABEL_COUNT; i++) {
            util_labels[i] = lv_label_create(utility_box);
            lv_label_set_text(util_labels[i], init_texts[i]);
            lv_obj_set_style_text_color(util_labels[i], lv_color_hex(0xffff00), 0);
            lv_obj_set_style_text_font(util_labels[i], &lv_font_unscii_16, 0);
            lv_obj_align(util_labels[i], LV_ALIGN_TOP_LEFT, 0, label_y_offset + i * line_height);
        }

        // Mode indicator (DEMO/LIVE)
        mode_indicator = lv_label_create(utility_box);
        lv_obj_set_style_text_font(mode_indicator, &lv_font_unscii_16, 0);
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
    resetTapPanelOpacity();  // Ensure panels start transparent
    resetUIElements();
    resetCharts();
    
#if ENABLE_LIGHTWEIGHT_BARS
    initLightweightBars();
#endif

    Serial.println("\nForcing initial render...");
    uint32_t t0 = millis();
    lv_refr_now(NULL);
    Serial.printf("Initial render took: %u ms\n", millis() - t0);

    //=================================================================
    // CREATE CORE 0 TASKS
    // These run independently from the main loop on Core 1
    //=================================================================
    Serial.println("\n[CORE0] Creating Core 0 tasks...");
    
    // Touch task on Core 1 - polls GT911 every 10ms
    // Moved to Core 1 to avoid I2C bus contention with RTC/SD operations on Core 0
    BaseType_t touchTaskResult = xTaskCreatePinnedToCore(
        touchTask,           // Task function
        "TouchTask",         // Task name
        4096,                // Stack size (bytes)
        NULL,                // Parameters
        2,                   // Priority (higher than idle)
        &g_touch_task_handle, // Task handle
        1                    // Core 1 (same as LVGL for direct integration)
    );
    if (touchTaskResult == pdPASS) {
        Serial.println("[CORE1] Touch task created on Core 1");
    } else {
        Serial.println("[CORE1] ERROR: Touch task creation failed!");
    }
    
#if ENABLE_SD_LOGGING
    // SD write task on Core 0 - handles all SD card I/O
    if (g_sd_queue && g_sd_state.initialized) {
        // Check available heap before task creation
        size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        Serial.printf("[CORE0] Free internal heap: %u bytes, largest block: %u bytes\n", free_heap, largest_block);
        
        // Use 4KB stack (reduced from 8KB to leave room for other allocations)
        // SD task locals use ~700 bytes, File ops need ~1-2KB, leaves safety margin
        const uint32_t SD_TASK_STACK = 4096;
        
        if (largest_block < SD_TASK_STACK + 1024) {
            Serial.printf("[CORE0] WARNING: Low heap! Need %u bytes for SD task\n", SD_TASK_STACK);
        }
        
        BaseType_t sdTaskResult = xTaskCreatePinnedToCore(
            sdWriteTask,         // Task function
            "SDWriteTask",       // Task name
            SD_TASK_STACK,       // Stack size (bytes)
            NULL,                // Parameters
            1,                   // Priority (lower than touch)
            &g_sd_task_handle,   // Task handle
            0                    // Core 0
        );
        if (sdTaskResult == pdPASS) {
            Serial.println("[CORE0] SD write task created on Core 0");
        } else {
            Serial.printf("[CORE0] ERROR: SD task creation failed! (heap=%u)\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        }
    } else {
        Serial.printf("[CORE0] SD task NOT created: queue=%p, initialized=%d\n", g_sd_queue, g_sd_state.initialized);
    }
#endif

    Serial.println("\n========================================");
    Serial.println("         RUNNING (DUAL-CORE)");
    Serial.printf("  Mode: %s\n", g_demo_mode ? "DEMO" : "LIVE");
    Serial.printf("  Pressure Unit: %s\n", getPressureUnitStr(g_pressure_unit));
    Serial.println("  Temp units: Per-gauge (tap to cycle)");
    Serial.println("  Hold utility box 5s to toggle mode");
    Serial.println("  Core 0: SD I/O + Time Sync");
    Serial.println("  Core 1: LVGL + Touch + Data Processing");
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

    // File browser update (monitors BOOT button for 5s hold)
#if ENABLE_FILE_BROWSER
    fb_update();
    
    // If file browser is active, skip normal UI updates
    if (fb_isActive()) {
        lv_timer_handler();
        delay(5);  // Yield CPU while in file browser
        return;
    }
#endif

    // Status every 1 second
    if (now - last_status >= 1000) {
        uint32_t dt_ms = now - last_status;
        
        // Update time from RTC/NTP
        updateTime();

        // Snapshot and reset frame/flush counts
        uint32_t frames = frame_count;
        uint32_t flushes = flush_count;
        frame_count = 0;
        flush_count = 0;

        // FPS = real frames per second (scaled by actual elapsed time for accuracy)
        int fps = (dt_ms > 0) ? (int)((frames * 1000UL + dt_ms / 2) / dt_ms) : 0;

        // Calculate per-core CPU usage from idle counts
        uint32_t delta0 = g_idle_count_core0 - g_last_idle0;
        uint32_t delta1 = g_idle_count_core1 - g_last_idle1;
        g_last_idle0 = g_idle_count_core0;
        g_last_idle1 = g_idle_count_core1;

        // More idle calls = less busy. Use dynamic calibration based on max observed.
        // Typical idle counts are ~300k-800k per second per core when mostly idle
        static uint32_t max_idle0 = 500000, max_idle1 = 500000;
        if (delta0 > max_idle0) max_idle0 = delta0;
        if (delta1 > max_idle1) max_idle1 = delta1;

        int cpu0_percent = (max_idle0 > 0) ? (100 - ((delta0 * 100) / max_idle0)) : 0;
        int cpu1_percent = (max_idle1 > 0) ? (100 - ((delta1 * 100) / max_idle1)) : 0;
        if (cpu0_percent < 0) cpu0_percent = 0;
        if (cpu1_percent < 0) cpu1_percent = 0;
        if (cpu0_percent > 100) cpu0_percent = 100;
        if (cpu1_percent > 100) cpu1_percent = 100;

        // Update utility box with REAL FPS
        update_utility_label(fps, cpu0_percent, cpu1_percent);

        // Log with both frames and flushes for diagnostics
        Serial.printf("[STATUS] fps=%d (frames=%u flushes=%u) cpu0=%d%% cpu1=%d%% idle0=%u idle1=%u heap=%u mode=%s\n",
            fps, frames, flushes, cpu0_percent, cpu1_percent, delta0, delta1,
            ESP.getFreeHeap(), g_demo_mode ? "DEMO" : "LIVE");
        cpu_busy_time = 0;
        last_status = now;
    }

    // Update data and UI
#if ENABLE_UI_UPDATES

    // DEMO mode throttling: Reduce update rate to prevent CPU1 saturation
    // In demo mode, values change slowly anyway, so 50ms updates are plenty
    uint32_t effective_interval = g_demo_mode ? (UPDATE_INTERVAL_MS > 50 ? UPDATE_INTERVAL_MS : 50) : UPDATE_INTERVAL_MS;
    
    if (now - last_update >= effective_interval) {
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

    lv_timer_handler();

    uint32_t elapsed = millis() - frame_start;
    cpu_busy_time += elapsed;
    if (elapsed < FRAME_TIME_MS) {
        delay(FRAME_TIME_MS - elapsed);
    }
}
