# 370zMonitor - AI Assistant Reference

## Project Overview

**370zMonitor** is a track car data logging and display system for a 2018 Nissan 370Z. It uses an ESP32-S3 microcontroller with a 7" touchscreen to display real-time sensor data during track days.

- **Current Version:** v4.9
- **Hardware:** Waveshare ESP32-S3-Touch-LCD-7 (800×480)
- **Framework:** Arduino with LVGL 9.1.0
- **Architecture:** Dual-core (Core 0: I/O, Core 1: LVGL rendering)

---

## Directory Structure

```
C:\source\370zMonitor\
├── Arduino/
│   └── 370zMonitor/
│       ├── 370zMonitor.ino      # Main sketch (~2500+ lines)
│       ├── file_browser.h       # SD card file browser module
│       ├── ui_ScreenSplash.c    # Splash screen (version label here)
│       ├── ui_Screen1.c         # Main gauge screen (SquareLine export)
│       ├── ui.c/h               # SquareLine Studio UI exports
│       ├── ui_font_*.c          # Orbitron fonts (various sizes)
│       ├── ui_img_*.c           # Image assets
│       ├── Images/              # Source images
│       └── wifi.cfg.example     # WiFi config template
├── SquareLine/
│   ├── GaugeAndChartDesign.spj  # SquareLine Studio project
│   ├── !export/                 # Export destination
│   └── assets/                  # UI assets
└── CLAUDE.md                    # This file
```

---

## Version Sync Checklist

**When updating version number, change BOTH locations:**

1. `Arduino/370zMonitor/370zMonitor.ino` - Header comment block (line ~4)
2. `Arduino/370zMonitor/ui_ScreenSplash.c` - `lv_label_set_text(version_label, "vX.X")` (~line 107)

---

## Hardware Configuration

### Display & Touch
| Component | Details |
|-----------|---------|
| Display | 800×480 RGB666, 14MHz pixel clock |
| Touch | GT911 capacitive (I2C: 0x5D or 0x14) |
| IO Expander | CH422G (I2C: 0x24) |
| Backlight | Digital ON/OFF only (no PWM) |

### Pin Assignments
| Function | GPIO |
|----------|------|
| I2C SDA | 8 |
| I2C SCL | 9 |
| Touch INT | 4 |
| SD SCK | 12 |
| SD MISO | 13 |
| SD MOSI | 11 |
| SD CS | IO Expander bit 4 |
| RS485 TX | 16 |
| RS485 RX | 15 |
| BOOT Button | 0 |

**⚠️ GPIO Conflicts to Avoid:**
- GPIO46 = Display HSYNC (do NOT use for SD card)
- GPIO10 = Display Blue channel (do NOT use for SD)

### RGB Display Pins
```
DE=5, VSYNC=3, HSYNC=46, PCLK=7
R: 1,2,42,41,40  G: 39,0,45,48,47,21  B: 14,38,18,17,10
```

---

## Key Feature Flags

Located near top of `370zMonitor.ino`:

```cpp
#define ENABLE_SD_LOGGING           1   // SD card data logging
#define ENABLE_FILE_BROWSER         1   // In-field log viewer
#define ENABLE_TOUCH                1   // Touch input
#define ENABLE_UI_UPDATES           1   // Master UI switch
#define ENABLE_LIGHTWEIGHT_BARS     1   // Efficient bar rendering
#define ENABLE_CHARTS               1   // Rolling charts
#define ENABLE_USB_MSC              1   // USB Mass Storage mode
#define ENABLE_MODBUS_SENSORS       1   // RS485 sensor reading
#define UPDATE_INTERVAL_MS          25  // UI refresh rate (ms)
```

---

## Data Architecture

### VehicleData Structure
All sensor values stored internally in base units:
- **Temperature:** Fahrenheit (converted to C for display if selected)
- **Pressure:** PSI (converted to Bar/kPa/ATM for display if selected)

Each value has a `_valid` flag indicating sensor health.

### Unit Preferences
Stored in ESP32 Preferences (flash):
- Per-gauge temperature units (oil, water, trans, steer, diff)
- Pressure unit (global)

---

## Dual-Core Architecture

| Core | Responsibilities |
|------|-----------------|
| Core 0 | Touch polling task, SD logging task, Modbus RS485 I/O |
| Core 1 | LVGL rendering, UI updates, main loop |

**FreeRTOS Components:**
- Touch mutex for thread-safe access
- SD write queue (producer/consumer pattern)
- Dedicated tasks pinned to specific cores

---

## SD Card Logging

### File Naming
- Format: `SESS_NNNNNNNN.csv` (8-digit boot counter)
- Boot counter stored in `/.bootcount` on SD card
- Supports up to 99,999,999 sessions

### CSV Columns
```
timestamp_ms, elapsed_s, cpu_percent, mode, oil_press_psi, oil_press_valid,
oil_temp_pan_f, oil_temp_cooled_f, oil_temp_valid, water_temp_hot_f, ...
```

### TeeSerial
Transparent Serial wrapper that automatically mirrors all `Serial.print()` output to SD card log. No code changes needed - just use Serial as normal.

---

## Modbus RS485 Sensors

### Hardware
- Waveshare 8-Ch Analog Acquisition Module (Model B, 0-10V)
- SP3485 transceiver (auto-direction, no DE/RE pin)
- 9600 baud, 8N1

### Channel Mapping
| Channel | Sensor | Signal |
|---------|--------|--------|
| 0 (AI1) | Oil Pressure (PX3AN2BH150PSAAX) | 0.5-4.5V via 10kΩ/22kΩ divider |
| 1 (AI2) | Oil Temperature (PT100 + uxcell transmitter) | 0-10V direct |
| 2-7 | Reserved for future sensors | - |

### Pressure Sensor Calibration (CH1)
```cpp
// PX3AN2BH150PSAAX: 0.5V-4.5V for 0-150 PSI
// Voltage divider: 10kΩ / 22kΩ (ratio 0.6875)
PSI = (raw_mV * 1.4545 - 500) * 0.0375
```

### Temperature Sensor Calibration (CH2)
```cpp
// PT100 RTD with uxcell transmitter (24V powered)
// Transmitter outputs 0-10V linear for -50°C to +200°C
// Waveshare Model B reads 0-10V directly (no divider needed)
temp_C = (raw_mV * 0.025) - 50
// Then converted to Fahrenheit for internal storage
```

### Wiring Diagram
```
[PT100 Probe] ▶ [uxcell Transmitter] ▶ [Waveshare AI2+/AI2-]
  3-wire RTD      24V DC powered         0-10V input mode
                  outputs 0-10V
```

### Error Handling
- 3 consecutive Modbus errors → mark sensors invalid
- CH1: mV < 100 → pressure sensor disconnected
- CH2: mV < 50 → PT100/transmitter disconnected
- UI shows "---" when invalid

---

## User Interactions

### Touch Gestures
| Gesture | Action |
|---------|--------|
| Single tap on gauge | Cycle display units (°F/°C, PSI/Bar/kPa) |
| Double-tap anywhere | Show utility box |
| 5-second hold on utility | Toggle Demo/Live mode |
| Tap FILES button | Enter file browser |

### Special Modes
| Mode | Entry |
|------|-------|
| USB Mass Storage | Hold BOOT during power-on (press RESET, then hold BOOT) |
| Firmware Download | Hold BOOT, then press RESET |
| File Browser | Double-tap → tap FILES in utility box |

---

## UI Color Palette

```cpp
#define PASSION_RED_COLOR   0xA31621    // Nissan Passion Red
#define PASSION_RED_BRIGHT  0xD41F2D    // Highlight red
#define DARK_BACKGROUND     0x0A0A0A    // Near black
#define ACCENT_GRAY         0x333333    // Subtle gray
#define TEXT_WHITE          0xFFFFFF
#define TEXT_GRAY           0xAAAAAA
```

---

## Performance Tuning

### Key Parameters
| Parameter | Default | Purpose |
|-----------|---------|---------|
| UPDATE_INTERVAL_MS | 25 | UI refresh rate |
| I2C_FREQ_HZ | 400000 | Touch controller speed |
| LVGL buffer | 1/10 screen | Render buffer size |
| SD_FLUSH_INTERVAL_MS | 1000 | SD write frequency |

### Optimization Notes
- Use lightweight bars (rectangles) instead of lv_bar widgets
- State-based label updates (only update when value changes)
- Minimize LVGL invalidations
- Chart point limits to prevent memory growth

---

## Build Settings (Arduino IDE)

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | Enabled |
| PSRAM | OPI PSRAM |
| Flash Mode | QIO 80MHz |
| Flash Size | 16MB |
| Partition | 16M Flash (3MB APP/9.9MB FATFS) |
| Upload Speed | 921600 |

---

## Required Libraries

1. **Arduino_GFX** (moononournation)
2. **lvgl** (9.x)
3. **TAMC_GT911** (touch driver)
4. **SD** (built-in)
5. **SPI** (built-in)
6. **Preferences** (built-in)
7. **WiFi** (built-in, for NTP)

---

## Common Tasks

### Adding a New Sensor
1. Add channel constant: `#define MODBUS_CH_NEW_SENSOR N`
2. Add fields to `VehicleData` struct with `_valid` flag
3. Add conversion function if needed
4. Update `updateModbusSensors()` to read and convert
5. Add UI elements in SquareLine Studio
6. Export and update `ui_Screen1.c`
7. Add update logic in main loop

### Adding a New Screen
1. Design in SquareLine Studio
2. Export to `Arduino/370zMonitor/`
3. Add `#include` in main sketch
4. Initialize in `setup()` after `ui_init()`
5. Add navigation logic

### Debugging Tips
- All Serial output goes to both USB and SD card
- Check `SESS_*.csv` files for runtime logs
- Use `[TAG]` prefixes for log filtering
- CPU load displayed in utility box

---

## File Browser Module

Located in `file_browser.h`. Key features:
- Optimized O(25) file listing using boot_count
- CSV grid viewer with frozen headers
- Text file viewer
- Navigate folders, back button exits

Entry: Double-tap → FILES button

---

## Toast Notification System

Real-time system health monitoring with visual feedback. Located in `370zMonitor.ino`.

### Key Components

| Component | Purpose |
|-----------|---------|
| `g_toast_obj` | LVGL toast container object |
| `g_toast_timer` | Auto-hide timer |
| `g_system_monitor_timer` | Background monitoring (10 sec interval) |
| `g_prev_system_status` | Previous state for change detection |

### Timing Constants

```cpp
#define TOAST_SUCCESS_MS        3000   // Green "All Systems Online" (3 sec)
#define TOAST_ERROR_MS         30000   // Red error toast (30 sec)
#define TOAST_RECOVERY_MS      15000   // Green recovery toast (15 sec)
#define SYSTEM_CHECK_DELAY_MS   5000   // Initial check delay after boot
#define SYSTEM_MONITOR_INTERVAL_MS 10000  // Background check interval
```

### Monitored Systems

| System | Check Method | Error Message | Recovery Message |
|--------|--------------|---------------|------------------|
| SD Card | `SD.cardType()` runtime probe + hot-swap recovery | "SD Card offline" | "SD Card back online" |
| Logs Writing | `file_open \|\| log_file_open` | "Logs writing offline" | "Logs writing back online" |
| RTC (HW-084) | I2C probe to DS3231 (0x68) | "Time keeper offline" | "Time keeper back online" |
| Time Sync | `time_available` flag | "Time sync failed" | "Time sync back online" |
| Modbus RTU | `initialized && comm_ok` | "Modbus RTU offline" | "Modbus RTU back online" |
| Oil Pressure | `g_sensor_ch1_connected` | "Sensor Oil Pressure offline" | "Sensor Oil Pressure back online" |
| Oil Temp (PT100) | `g_sensor_ch2_connected` | "Sensor Oil Temp offline" | "Sensor Oil Temp back online" |

### Toast Behavior

1. **Boot sequence:** 5 seconds after main screen loads → full system check → green "All Systems Online" or red error list
2. **Background monitoring:** Every 10 seconds, checks all systems for state changes
3. **Error detection:** Only shows toast when system transitions from OK → FAIL
4. **Recovery detection:** Shows green toast when system transitions from FAIL → OK
5. **Priority:** New errors take priority over recovery toasts

### SD Card Hot-Swap Recovery

The system automatically attempts to recover when an SD card is reinserted:

1. Background monitor detects card is offline (`initialized = false`)
2. Calls `sdTryReinit()` which:
   - Cleans up with `SD.end()`
   - Attempts `SD.begin()` to detect card
   - If successful: restores state, increments boot count, starts new session
3. Recovery toast shows "SD Card back online"
4. New session files created (SESS_XXXXXXXX.csv/.log)

**Note:** This creates a new session with incremented boot count - previous session data from before removal is preserved.

### Adding New Sensors/Devices to Toast System

1. **Add to `g_prev_system_status` struct:**
   ```cpp
   static struct {
       // ... existing fields ...
       bool new_sensor_ok;
   } g_prev_system_status = {true, true, ..., true, false};
   ```

2. **Add check in `checkSystemsAndShowToast()`:** (initial boot check)
   ```cpp
   new_sensor_ok = /* your check */;
   if (!new_sensor_ok) {
       strcat(error_msg, "New Sensor offline");
       error_count++;
   }
   ```

3. **Add check in `backgroundSystemMonitor()`:** (runtime monitoring)
   ```cpp
   // Runtime probe for new sensor (e.g., I2C check)
   Wire.beginTransmission(NEW_SENSOR_ADDR);
   new_sensor_ok = (Wire.endTransmission() == 0);
   
   // Offline detection
   if (!new_sensor_ok && g_prev_system_status.new_sensor_ok) {
       strcat(error_msg, "New Sensor offline");
       new_error_count++;
       Serial.println("[MONITOR] New Sensor went OFFLINE");
   }
   // Recovery detection
   else if (new_sensor_ok && !g_prev_system_status.new_sensor_ok) {
       strcat(recovery_msg, "New Sensor back online");
       recovery_count++;
       Serial.println("[MONITOR] New Sensor RECOVERED");
   }
   
   // Update previous status
   g_prev_system_status.new_sensor_ok = new_sensor_ok;
   ```

### Toast Styling

```cpp
lv_font_montserrat_20           // Font
lv_obj_set_style_pad_hor(..., 30, 0)   // Horizontal padding
lv_obj_set_style_pad_ver(..., 18, 0)   // Vertical padding
lv_obj_set_style_radius(..., 0, 0)     // No rounded corners
lv_obj_set_style_bg_opa(..., LV_OPA_COVER, 0)  // Fully opaque
TOAST_COLOR_SUCCESS  0x2E7D32  // Green
TOAST_COLOR_ERROR    0xB71C1C  // Dark red
```

---

## Known Issues / Gotchas

1. **GPIO46 Conflict:** Cannot use GPIO46 for SD card - it's HSYNC for display
2. **Backlight:** No PWM brightness control on this board variant
3. **SquareLine Exports:** Regenerates `ui_*.c` files - don't edit directly unless adding to non-exported sections
4. **Boot Counter:** Stored on SD card - new card starts at 0
5. **Demo Mode:** Persists until toggled - check `g_demo_mode` flag

---

## Version History

| Version | Key Changes |
|---------|-------------|
| v4.9 | PT100 oil temperature sensor via uxcell transmitter on Modbus CH2 |
| v4.8 | Toast notification system with runtime monitoring, error/recovery detection |
| v4.7 | File browser optimization, 8-digit sessions |
| v4.6 | SD card file browser, CSV/text viewers |
| v4.5 | Sensor failure detection, "---" UI feedback |
| v4.4 | Splash screen with loading animation |
| v4.3 | USB MSC fixes, CSV header bugfix |
| v4.2 | Dual-core architecture |
| v4.1 | Unit conversion, tap-to-cycle units |

---

## Quick Reference

```
Compile: Arduino IDE → Upload (921600 baud)
Serial:  115200 baud
SD Card: FAT32, Class 10+
Logs:    /SESS_NNNNNNNN.csv
Config:  /wifi.cfg (optional)
```
