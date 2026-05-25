# 370zMonitor - AI Assistant Reference

## Project Overview

**370zMonitor** is a track car data logging and display system for a 2018 Nissan 370Z. It uses an ESP32-S3 microcontroller with a 7" touchscreen to display real-time sensor data during track days.

- **Current Version:** v5.8
- **Hardware:** Waveshare ESP32-S3-Touch-LCD-7 (800x480) with onboard TJA1051T CAN transceiver and SP3485 RS485 transceiver
- **Hardware:** Waveshare Industrial 8-Ch Analog Acquisition Module (Model B, 0-10V / 0-20mA / 4-20mA selectable)
- **Hardware:** Crowtail I2C Hub 2.0 (for multiple I2C devices)
- **Hardware:** HW-084 DS3231 RTC module (battery-backed real-time clock)
- **Sensor:** Oil Pressure: PX3AN2BH150PSAAX (Channel 1 / AI1 on Waveshare, Mode 0 = 0-10V)
- **Sensor:** Oil Temperature: PRTXI-1/2N-1/4-4-IO RTD transmitter (Channel 2 / AI2, Mode 3 = 4-20mA)
- **Sensor:** Trans Temperature: PRTXI RTD transmitter (Channel 3 / AI3, Mode 3 = 4-20mA)
- **Sensor:** Power Steering Temperature: PRTXI RTD transmitter (Channel 4 / AI4, Mode 3 = 4-20mA)
- **Sensor:** Differential Temperature: PRTXI RTD transmitter (Channel 5 / AI5, Mode 3 = 4-20mA)
- **Sensor:** Accelerometer: Adafruit LIS3DH (ADA2809) via I2C (0x18 or 0x19)
- **OBD-II:** CAN bus (TWAI) over onboard TJA1051T to vehicle ECU (provides water temp + fuel trims for Fuel Trust)
- **Framework:** Arduino with LVGL 9.1.0
- **Architecture:** Dual-core (Core 0: SD I/O + time sync, Core 1: LVGL + Touch + sensor I/O + main loop)

---

## Directory Structure

```
<repo root>/  (canonical path on the dev machine: C:\source\370zMonitor or your Google Drive clone)
├── Arduino/
│   └── 370zMonitor/
│       ├── 370zMonitor.ino      # Main sketch (~8,500 lines)
│       ├── file_browser.h       # SD card file browser module (~1,200 lines)
│       ├── ui_ScreenSplash.c    # Splash screen (version label hard-coded here)
│       ├── ui_ScreenSplash.h    # Splash screen header
│       ├── ui_Screen1.c         # Main gauge screen (SquareLine export)
│       ├── ui_Screen1.h
│       ├── ui.c / ui.h          # SquareLine Studio UI exports
│       ├── ui_events.h
│       ├── ui_helpers.c / .h
│       ├── ui_comp_hook.c
│       ├── ui_font_Orbitron*.c  # Orbitron fonts (multiple weights and sizes)
│       ├── ui_img_*.c           # Image assets
│       ├── Images/              # Source images (splash variants)
│       ├── wifi.cfg.example     # WiFi config template
│       ├── platformio.ini       # PlatformIO config (optional alternative to Arduino IDE)
│       ├── CMakeLists.txt
│       └── README.md
├── SquareLine/
│   ├── GaugeAndChartDesign.spj  # SquareLine Studio project
│   ├── !export/                 # Export destination
│   └── assets/                  # UI assets
└── CLAUDE.md                    # This file
```

---

## Version Sync Checklist

**When updating version number, change BOTH locations:**

1. `Arduino/370zMonitor/370zMonitor.ino` - Header comment block at top (line ~4: `* 370zMonitor v5.8`)
2. `Arduino/370zMonitor/ui_ScreenSplash.c` - `lv_label_set_text(version_label, "vX.X")` (~line 125)

---

## Hardware Configuration

### Display & Touch
| Component | Details |
|-----------|---------|
| Display | 800x480 RGB666, 14MHz pixel clock |
| Touch | GT911 capacitive (I2C: 0x5D default, 0x14 alternate) |
| IO Expander | CH422G (I2C: 0x24 system, 0x38 IOWR) |
| Backlight | Hardware: digital ON/OFF only (no PWM). Software dimming uses a black LVGL overlay on the top layer (see `setBrightness()`); brightness 250-255 hides it, lower values increase opacity. |

### Pin Assignments
| Function | GPIO |
|----------|------|
| I2C SDA | 8 |
| I2C SCL | 9 |
| Touch INT | 4 |
| SD SCK | 12 |
| SD MISO | 13 |
| SD MOSI | 11 |
| SD CS | IO Expander bit 4 (`EXIO_SD_CS`) — no native GPIO |
| RS485 TX | 16 (Serial1, to SP3485 DI) |
| RS485 RX | 15 (Serial1, from SP3485 RO) |
| CAN TX | 19 (to onboard TJA1051T TXD) |
| CAN RX | 20 (from onboard TJA1051T RXD) |
| BOOT Button | 0 (`USB_MSC_BOOT_PIN`) |

**GPIO Conflicts to Avoid:**
- GPIO46 = Display HSYNC (do NOT use for SD card)
- GPIO10 = Display Blue channel (do NOT use for SD)
- GPIO19/20 are CAN bus — keep clear of any other use

### RGB Display Pins
```
DE=5, VSYNC=3, HSYNC=46, PCLK=7
R: 1,2,42,41,40   G: 39,0,45,48,47,21   B: 14,38,18,17,10
```

### CH422G IO Expander Bits
| Bit | Symbol | Purpose |
|-----|--------|---------|
| 1 | EXIO_TP_RST | Touch panel reset |
| 2 | EXIO_DISP | Display enable / backlight ON-OFF |
| 4 | EXIO_SD_CS | SD card chip select |

---

## Key Feature Flags

Defined near the top of `370zMonitor.ino`. Default values shown:

```cpp
// Top-of-file (above TeeSerial)
#define ENABLE_GSENSOR              1   // LIS3DH accelerometer (line ~180)
#define ENABLE_SD_LOGGING           1   // SD card data logging (line ~236)
#define ENABLE_FILE_BROWSER         1   // In-field log viewer (line ~237)

// Main feature flags block (line ~351)
#define ENABLE_TOUCH                1
#define ENABLE_UI_UPDATES           1   // Master UI switch
#define ENABLE_BARS                 0   // DISABLED - SquareLine bars cause CPU spikes
#define ENABLE_LIGHTWEIGHT_BARS     1   // Rectangle overlays (cheap)
#define ENABLE_CRITICAL_LABEL_BLINK 0   // 0 = static, 1 = blinking
#define ENABLE_VALUE_CRITICAL       1   // "Value Critical" labels
#define ENABLE_CHARTS               1   // Rolling charts
#define ENABLE_USB_MSC              1   // USB Mass Storage mode
#define ENABLE_MODBUS_SENSORS       1   // RS485 sensor reading
#define ENABLE_OBD_CAN              1   // OBD-II via CAN bus (TWAI)
#define UPDATE_INTERVAL_MS          25  // UI refresh rate (ms)
#define USB_MSC_BOOT_PIN            0   // GPIO0 = BOOT button
```

---

## Data Architecture

### VehicleData Structure
All sensor values stored internally in base units:
- **Temperature:** Fahrenheit (converted to/from C at the boundaries)
- **Pressure:** PSI (converted to Bar/kPa/ATM for display if selected)

Each value has a `_valid` flag indicating sensor health. `has_received_data` is a global flag that flips true once any sensor has produced a valid reading (used to decide whether the UI shows "---" or real values).

Fields (see `struct VehicleData` in the sketch):
- `oil_pressure_psi`, `oil_pressure_valid`
- `oil_temp_value_f`, `oil_temp_valid`
- `water_temp_value_f`, `water_temp_valid` (sourced from OBD ECT)
- `trans_temp_value_f`, `trans_temp_valid`
- `steer_temp_value_f`, `steer_temp_valid`
- `diff_temp_value_f`, `diff_temp_valid`
- `fuel_trust_percent`, `fuel_trust_valid` (computed from OBD trims + timing)
- `rpm`, `rpm_valid` (from OBD PID 0x0C)
- `accel_x_g`, `accel_y_g`, `accel_z_g`, `accel_valid` (LIS3DH)

### Gauge Range / Critical Thresholds
Defined in the "Gauges Configuration" region of the sketch:

| Gauge | Min | Max | Critical |
|-------|-----|-----|----------|
| Oil Pressure | 0 PSI | 150 PSI | `<10` PSI or `>120` PSI (also RPM-based low-pressure check) |
| Oil Temp | 150 F | 300 F | `>=260 F` |
| Water Temp | 100 F | 260 F | `>=220 F` |
| Trans Temp | 80 F | 280 F | `>=230 F` |
| Steer Temp | 60 F | 300 F | `>=230 F` |
| Diff Temp | 60 F | 320 F | `>=260 F` |
| Fuel Trust | 0% | 100% | `<=75%` |

### Unit Preferences
Stored in ESP32 Preferences (flash), namespace `units`:
- Per-gauge temperature units (oil, water, trans, steer, diff)
- Pressure unit (global): PSI / Bar / kPa / ATM

A separate namespace `display` stores `auto_bri` (auto-brightness on/off).

---

## Dual-Core Architecture

| Core | Responsibilities |
|------|-----------------|
| Core 0 | SD write task (`sdWriteTask`), time/RTC sync, NTP background work |
| Core 1 | Main loop, LVGL rendering, Touch polling task (`touchTask`), Modbus reads, OBD CAN polling, accelerometer reads |

> Note: although CLAUDE.md previously claimed touch ran on Core 0, the touch task is now explicitly pinned to **Core 1** to avoid I2C bus contention with RTC/SD on Core 0. See `xTaskCreatePinnedToCore(touchTask, ..., 1)` near the bottom of `setup()`.

**FreeRTOS Components:**
- `g_touch_mutex` — thread-safe access to touch state
- `g_sd_queue` — producer/consumer for binary CSV log entries
- `g_serial_log_queue` — producer/consumer for Serial output -> .log file
- `g_time_mutex` — guards the formatted datetime string
- Dedicated tasks pinned to specific cores (Touch=Core1, SDWrite=Core0)

---

## SD Card Logging

### File Naming
- Format: `SESS_NNNNNNNN.csv` and `SESS_NNNNNNNN.log` (8-digit boot counter)
- Boot counter stored as binary file on the SD card at `/BOOTCNT.DAT` with a redundant backup at `/BOOTCNT.BAK` (validated against each other on read)
- Supports up to 99,999,999 sessions
- File browser also recognizes legacy 5-digit `SESS_NNNNN` names

### CSV Columns (header written at session start)
```
datetime,rpm,
oil_press_psi,oil_press_is_critical,
oil_temp_value_f,oil_temp_is_critical,
water_temp_value_f,water_temp_is_critical,
trans_temp_value_f,trans_temp_is_critical,
steer_temp_value_f,steer_temp_is_critical,
diff_temp_value_f,diff_temp_is_critical,
fuel_trust_percent,fuel_trust_is_critical,
accel_x_g,accel_y_g,accel_z_g,
rpm_valid,oil_press_valid,oil_temp_valid,
water_temp_valid,trans_temp_valid,steer_temp_valid,diff_temp_valid,
fuel_trust_valid,accel_valid,
timestamp_ms,elapsed_s,cpu_percent,mode
```

### Free-space Management
- `SD_FREE_SPACE_PERCENT 5` — keep at least 5% free
- `SD_MIN_FREE_BYTES 1 MB` — absolute minimum
- When low, the oldest `SESS_*.csv` (and matching `.log`) is deleted

### TeeSerial
Transparent `Print` wrapper that mirrors all `Serial.print()` output to the per-session `.log` file via a queue. After it is installed, `#define Serial _TeeSerialInstance` swaps the macro so every existing log call also writes to SD with no code changes. The "real" Serial is preserved as `_RealSerial` for use inside the SD task itself.

---

## Modbus RS485 Sensors

### Hardware
- Waveshare 8-Ch Analog Acquisition Module (Model B; 0-10V / 0-20mA / 4-20mA per-channel)
- SP3485 transceiver on the Waveshare ESP32-S3 board (auto-direction, no DE/RE pin)
- 9600 baud, 8N1, slave address 1

### Channel Mapping (5 channels active as of v5.4)
| Channel | Sensor | Signal | Waveshare Mode |
|---------|--------|--------|----------------|
| 0 (AI1) | Oil Pressure (PX3AN2BH150PSAAX) | 0.5-4.5V via 10kΩ/22kΩ divider | Mode 0 (0-10V) |
| 1 (AI2) | Oil Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 2 (AI3) | Transmission Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 3 (AI4) | Power Steering Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 4 (AI5) | Differential Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 5-7 | Reserved for future sensors | - | - |

Channel mode is written at startup via Modbus Function 0x06 to holding registers `0x1000` (CH1) through `0x1004` (CH5). The PRTXI channels are auto-configured to Mode 3 on each boot in `initModbusSensors()`.

### Pressure Sensor Calibration (CH1)
```cpp
// PX3AN2BH150PSAAX: 0.5V-4.5V for 0-150 PSI
// Voltage divider: 10kΩ / 22kΩ — divides by 0.6875 (inverse = 1.4545)
// Effective formula in convertToPSI():
//   PSI = (raw_mV * 1.4545 - 500) * 0.0375
// Clamped to [0, 150]
#define PRESSURE_DIVIDER_RATIO 1.4545f
#define PRESSURE_OFFSET_MV     500.0f
#define PRESSURE_SCALE         0.0375f
```

### Temperature Sensor Calibration (CH2-CH5)
```cpp
// PRTXI-1/2N-1/4-4-IO RTD Temperature Transmitter (4-20mA output)
// Outputs 4-20mA linear for -50°C to +200°C (250°C span)
// Waveshare in Mode 3 (4-20mA) returns microamps (µA) directly via Modbus.
//
//   4000  µA (4mA)    = -50°C
//   8800  µA (8.8mA)  =  25°C  (room temp)
//   12000 µA (12mA)   =  75°C  (midpoint)
//   20000 µA (20mA)   = +200°C
//
// In convertToTempC(), a +5°C calibration offset is added (ice-bath validated,
// conservative margin for metal-surface measurements). Clamped to [-45 .. 205].
//
// Hysteresis: 0.3°C — display only updates when change >= TEMP_HYSTERESIS_C
// (applied per-channel via g_last_*_temp_c state).
```

### Wiring Diagram (PRTXI, confirmed by real-world testing — do not flip)
```
24V+ ────────────────▶ PRTXI Pin 1 (V+)

PRTXI Pin 2 (V-/Signal) ────▶ Waveshare AI(n)+

Waveshare AI(n)- ────────────▶ GND (24V-)

PRTXI Pins 3,4: Not used (IO-Link mode only)
```

### Error Handling
- `MODBUS_RETRY_COUNT = 2` retries on each failed read
- `MODBUS_ERROR_THRESHOLD = 3` consecutive errors mark every channel invalid
- CH1: `mV < SENSOR_MIN_VALID_MV (100)` => pressure sensor disconnected
- CH2-CH5: `µA < PRTXI_MIN_VALID_UA (3000)` => RTD transmitter disconnected
- UI shows "---" when invalid; lightweight bars and tap panels reset to default

### Polling Cadence
- `MODBUS_READ_INTERVAL_MS = 100` (10 Hz)
- Unified one-line summary log printed every ~1 second covering all 5 channels plus OBD water-temp and fuel-trust

---

## OBD-II via CAN Bus (TWAI)

The system reads live data from the vehicle ECU over the OBD-II port using the ESP32's onboard TWAI controller routed through the **onboard TJA1051T transceiver** on the Waveshare board (no external transceiver needed).

### Hardware Wiring
| Board | OBD-II Port |
|-------|-------------|
| J6 Pin 1 (CANL) | OBD-II Pin 14 |
| J6 Pin 2 (CANH) | OBD-II Pin 6 |
| Any board GND | OBD-II Pin 4 or 5 (signal ground) |

No termination resistor needed — the car's ECU provides termination.

### Configuration
```cpp
#define CAN_TX_PIN  GPIO_NUM_19     // to TJA1051T TXD
#define CAN_RX_PIN  GPIO_NUM_20     // from TJA1051T RXD
// Bit rate: 500 kbit/s (OBD-II standard)
// Addressing: ISO 15765-4 11-bit CAN
//   Functional request: 0x7DF
//   ECU responses:      0x7E8 - 0x7EF
```

### Polled PIDs (round-robin, one PID every 200ms)
| PID | Meaning | Formula |
|-----|---------|---------|
| 0x05 | Engine Coolant Temp (ECT) | `A - 40` [°C] |
| 0x0C | Engine RPM | `((A*256)+B)/4` |
| 0x0D | Vehicle Speed | `A` [km/h] |
| 0x0E | Timing Advance | `(A - 64) / 2` [°BTDC] |
| 0x06 | STFT Bank 1 | `(A-128)*100/128` [%] |
| 0x07 | LTFT Bank 1 | `(A-128)*100/128` [%] |
| 0x08 | STFT Bank 2 | `(A-128)*100/128` [%] |
| 0x09 | LTFT Bank 2 | `(A-128)*100/128` [%] |

8 PIDs at 200ms = ~1.6s full cycle. Stale threshold: 3 seconds.

### Fuel Trust Calculation (`computeFuelTrust()`)
A 0-100% confidence score for fuel quality / tune. Starts at 100 and subtracts penalties:

| Condition | Penalty |
|-----------|---------|
| Average abs(STFT) > 5% | 0-10 points (ramps to 10 at 8%) |
| Average abs(LTFT) > 5% | 0-20 points (ramps to 20 at 10%) |
| Bank imbalance > 5% (STFT or LTFT) | 5 points |
| Timing pull > 3° drop under load (speed>30 kph AND rpm>2000) | 1.5 points per pull, max 30 |

Timing-pull counter is reset every `FUEL_TRUST_TIMING_RESET_INTERVAL_MS = 10s` to keep the display responsive. Fuel Trust is only displayed once the engine is warmed up (`ECT >= OBD_WARMUP_TEMP_C = 80°C`).

---

## LIS3DH Accelerometer (G-Sensor)

The system includes a 3-axis accelerometer for logging G-forces during track sessions.

### Hardware
- **Sensor:** Adafruit LIS3DH breakout (ADA2809)
- **Interface:** I2C via Crowtail I2C Hub 2.0
- **Address:** 0x18 (default, SA0 to GND) or 0x19 (SA0 to VCC) — both probed automatically
- **Range:** ±4G (`LIS3DH_RANGE_4_G`)
- **Data Rate:** 100 Hz (`LIS3DH_DATARATE_100_HZ`)
- **Performance:** High-resolution mode

### I2C Hub Connections
The Crowtail I2C Hub 2.0 connects multiple I2C devices on a single bus:
```
ESP32-S3 (SDA=GPIO8, SCL=GPIO9)
    └─── I2C Hub
           ├─── GT911 Touch Controller (0x5D / 0x14)
           ├─── CH422G IO Expander (0x24 system / 0x38 IOWR)
           ├─── DS3231 RTC (HW-084) (0x68)
           └─── LIS3DH Accelerometer (0x18 / 0x19)
```

### Data Output
- **CSV columns:** `accel_x_g,accel_y_g,accel_z_g` plus `accel_valid` flag
- **Units:** G-forces (1g = 9.80665 m/s²); library returns m/s², divided by GRAVITY constant
- **Axes (as mounted):**
  - X: Lateral (positive = right turn)
  - Y: Longitudinal (positive = forward acceleration)
  - Z: Vertical (positive = upward, ~1.0g at rest)

### Demo Mode
In demo mode, accelerometer simulates driving motion (handled inside `provideDemoData()` / related functions).

---

## Auto Brightness (Sunrise/Sunset based dimming)

Uses the NOAA Solar Calculator algorithm (same as Google Maps) to compute local sunrise/sunset times each day. Brightness flips between day and night automatically with a configurable twilight buffer. Implementation lives near the top of the sketch and in the `AUTO BRIGHTNESS` region around line 5112.

### Configuration
```cpp
#define LOCATION_LATITUDE   42.03   // Schaumburg, IL (positive = North)
#define LOCATION_LONGITUDE -88.08   // (negative = West)

#define BRIGHTNESS_DAY      255     // 100% — full brightness
#define BRIGHTNESS_NIGHT    90      // ~35% — dimmed at night

#define TWILIGHT_OFFSET_MINUTES            20      // start transition this early/late
#define AUTO_BRIGHTNESS_CHECK_INTERVAL_MS  30000   // check every 30s
```

### Behavior
- Default: ON (`g_auto_brightness.enabled = true`)
- Preference persisted in Preferences namespace `display`, key `auto_bri`
- Sunrise/sunset cached once per day-of-year (recomputed when date changes)
- "Manual override" engages when the user single-taps the utility-box brightness button — they get the opposite brightness; auto resumes only when the user toggles auto-brightness back on (via the dedicated button)
- Utility-box auto-brightness button label format: `DIM:\nAUTO [HH:MM]` (showing the next transition time)
- Brightness is applied via `setBrightness()` which controls the opacity of a black LVGL overlay on the top layer (no hardware PWM available on this board variant)

---

## RTC and Time Sync

| Component | Details |
|-----------|---------|
| RTC chip | DS3231 (HW-084 module), I2C address `0x68` |
| Detection | I2C probe in `sdDetectRTC()` at boot — populates `g_sd_state.rtc_available` |
| Time read | `readRTC()` — BCD decode, populates `struct tm` |
| Time write | `writeRTC()` — used after successful NTP sync to keep RTC fresh |
| OSF flag | `clearRTCOSFlag()` clears Oscillator Stop Flag after first set |
| Validity check | `isRTCTimeValid()` — fails if OSF set or year < 2024 |

### NTP Background Sync
- Optional WiFi credentials loaded from `/wifi.cfg` on the SD card (`WIFI_CONFIG_FILE`)
- NTP servers: `pool.ntp.org`, `time.nist.gov`, `time.google.com`
- `GMT_OFFSET_SEC = -6 * 3600` (CST); `DAYLIGHT_OFFSET_SEC = 0` (set to 3600 when DST is active)
- WiFi timeout: 10 s; NTP timeout: 3 s per server
- After a successful NTP sync, RTC is updated with the synced time

### Time-state Flags (used by toast monitor)
| Flag | Meaning |
|------|---------|
| `g_time_state.rtc_active` | Currently using RTC for time |
| `g_time_state.wifi_time_active` | Currently using WiFi/NTP for time |
| `g_time_state.time_available` | Any valid time source is producing data |

---

## User Interactions

### Touch Gestures
| Gesture | Action |
|---------|--------|
| Single tap on gauge value | Cycle display units (°F/°C; PSI/Bar/kPa/ATM) |
| Single tap on utility-box brightness button | Manual override (flip to opposite brightness) |
| Double-tap anywhere | Show utility box |
| 5-second hold on utility box | Toggle Demo / Live mode (`DEMO_MODE_TOGGLE_HOLD_MS`) |
| Tap FILES button (in utility box) | Enter file browser |
| Tap AUTO BRI button (in utility box) | Toggle auto-brightness ON/OFF |

### Special Modes
| Mode | Entry |
|------|-------|
| USB Mass Storage | Hold BOOT during power-on (press RESET, then hold BOOT) |
| Firmware Download | Hold BOOT, then press RESET |
| File Browser | Double-tap to open utility box, then tap FILES |

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

Lightweight-bars use `0x32231E` (dark brown) background and `0xFF4500` (orange) fill.

---

## Performance Tuning

### Key Parameters
| Parameter | Default | Purpose |
|-----------|---------|---------|
| `UPDATE_INTERVAL_MS` | 25 | UI refresh rate (raised to 50 in demo mode) |
| `TARGET_FPS` | 50 | Loop pacing target (`FRAME_TIME_MS = 1000/TARGET_FPS`) |
| `I2C_FREQ_HZ` | 400000 | Touch / RTC / G-sensor I2C speed |
| `LVGL_BUFFER_SIZE` | `800 * 30` (~48KB) | Internal-DMA-RAM render buffer (prevents PSRAM contention) |
| `SD_FLUSH_INTERVAL_MS` | 1000 | SD write/flush frequency |
| `SD_WRITE_INTERVAL_MS` | 1000 | Data-row write cadence |
| `SD_SPI_FREQ` | 4 MHz | Conservative SPI clock for reliability |
| `MODBUS_READ_INTERVAL_MS` | 100 | 10 Hz polling of Waveshare module |
| `OBD_PID_REQUEST_PERIOD_MS` | 200 | 5 Hz round-robin per PID |
| `CHART_BUCKET_MS` | 5000 | One chart bar every 5 s |
| `CHART_POINTS` | 24 | Bars per chart |
| `SMOOTH_FACTOR` | 0.3 | UI smoothing |

### Optimization Notes
- Lightweight rectangle bars used instead of `lv_bar` widgets (huge CPU saving)
- State-based label updates (only redraw when value changes)
- LVGL invalidations are minimized
- Chart point limits prevent memory growth
- Demo mode is throttled to >= 50 ms updates to keep CPU1 from saturating

---

## Build Settings (Arduino IDE)

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | Enabled |
| PSRAM | OPI PSRAM |
| Flash Mode | QIO 80MHz |
| Flash Size | 16MB |
| Partition | 16M Flash (3MB APP / 9.9MB FATFS) |
| Upload Speed | 921600 |

A `platformio.ini` is also present for PlatformIO users.

---

## Serial Output & Debugging

**Serial baud rate: 115200**

`Serial.begin(115200)` is called from `setup()`. All `Serial.print*` traffic is captured by the **TeeSerial** wrapper and queued to the per-session `.log` file on the SD card.

**Arduino IDE Serial Monitor:**
- Set baud-rate dropdown to **115200** (bottom-right corner)
- Garbled output -> verify baud rate matches code

**Alternative Serial Tools:**
- PuTTY, TeraTerm, etc.: **115200, 8N1, no flow control**

**Useful log tags** (for grep / filtering):
`[MODBUS] [OBD] [SD] [RTC] [WIFI] [NTP] [G-SENSOR] [TOAST] [MONITOR] [SYSTEM CHECK] [AUTO-BRI] [PREFS] [CORE0/SD] [CORE1] [STATUS] [CHARTS]`

---

## Required Libraries

1. **Arduino_GFX_Library** (moononournation)
2. **lvgl** (9.x)
3. **TAMC_GT911** (touch driver)
4. **Adafruit_LIS3DH** (accelerometer driver)
5. **Adafruit_Sensor** (required by LIS3DH)
6. **SD** (built-in)
7. **SPI** (built-in)
8. **Preferences** (built-in)
9. **WiFi** (built-in, for NTP)
10. **driver/twai.h** (ESP-IDF TWAI/CAN driver — bundled with ESP32 core)
11. **USB / USBMSC** (ESP32-S3 native USB, for Mass Storage mode)

---

## Common Tasks

### Adding a New Modbus Sensor
1. Add channel constant: `#define MODBUS_CH_NEW_SENSOR N`
2. Increase `MODBUS_NUM_CHANNELS` if needed
3. Add fields to `VehicleData` struct with `_valid` flag
4. Add a conversion function if needed (look at `convertToPSI` / `convertToTempC` for patterns)
5. Update `readModbusSensors()` to read and convert, plus state-change logging
6. Add UI elements in SquareLine Studio
7. Export and update `ui_Screen1.c`
8. Add update logic in `updateUI()` and a chart series if relevant
9. Add the sensor to the toast monitor (see "Adding New Sensors/Devices to Toast System" below)
10. Add the column to the CSV header and the format string in `sdWriteTask()`

### Adding a New OBD PID
1. Append the PID byte to `OBD_PID_LIST[]`
2. Add a `case 0xNN:` to `decodeOBD_Mode01Reply()` to populate `g_obd_data`
3. Add staleness tracking in `checkOBDDataStaleness()` if you add a new timestamp
4. Wire it into `updateOBDData()` to update `g_vehicle_data`

### Adding a New Screen
1. Design in SquareLine Studio
2. Export to `Arduino/370zMonitor/`
3. Add `#include` in the main sketch
4. Initialize in `setup()` after `ui_init()`
5. Add navigation logic

### Debugging Tips
- All Serial output goes to both USB and SD card (`SESS_*.log`)
- Per-session CSV (`SESS_*.csv`) holds the raw data
- Use `[TAG]` prefixes for log filtering
- CPU0/CPU1 load and FPS are displayed in the utility box
- `[STATUS]` line is printed once per second with FPS, per-core CPU%, heap, mode

---

## File Browser Module

Located in `file_browser.h` (~1,200 lines). Key features:
- Optimized O(25) recent-session listing using `g_current_boot_count` (no full directory scan)
- Folder list cached after first scan
- Supports both 8-digit (new) and 5-digit (legacy) `SESS_*` filenames
- CSV grid viewer with frozen header row, frozen first column, and frozen corner cell
- Text file viewer (chunked, up to ~100 KB, 4 KB chunks)
- Navigate folders; back from root exits the browser
- Sets `g_fb_pause_sd_writes = true` while active to avoid SD-card contention with the writer task

Entry: Double-tap to open the utility box -> tap FILES.

---

## Toast Notification System

Real-time system health monitoring with visual feedback. Implemented in `370zMonitor.ino` around line 5400+.

### Key Components

| Component | Purpose |
|-----------|---------|
| `g_toast_obj` | LVGL toast container object |
| `g_toast_timer` | Auto-hide timer |
| `g_system_monitor_timer` | Background monitoring (10-second interval) |
| `g_prev_system_status` | Previous state for change detection |

### Timing Constants

```cpp
#define TOAST_SUCCESS_MS           3000    // Green "All Systems Online" (3 s)
#define TOAST_ERROR_MS            30000    // Red error toast (30 s)
#define TOAST_RECOVERY_MS         15000    // Green recovery toast (15 s)
#define SYSTEM_CHECK_DELAY_MS      5000    // Delay after main screen loads
#define SYSTEM_MONITOR_INTERVAL_MS 10000   // Background check interval
```

### Monitored Systems (12 total)

| System | Check | Error Message | Recovery Message |
|--------|-------|---------------|------------------|
| SD Card | `g_sd_state.card_present && g_sd_state.initialized` + hot-swap reinit | "SD Card offline" | "SD Card back online" |
| Logs Writing | `file_open || log_file_open` | "Logs writing offline" | "Logs writing back online" |
| RTC (HW-084) | I2C probe to DS3231 (0x68) | "Time keeper offline" | "Time keeper back online" |
| Time Sync | `g_time_state.time_available` (only reported failed if RTC also failed) | "Time sync failed" | "Time sync back online" |
| Modbus RTU | `g_modbus_initialized && g_modbus_comm_ok` | "Modbus RTU offline" | "Modbus RTU back online" |
| Oil Pressure | `g_sensor_ch1_connected` | "Sensor Oil Pressure offline" | "Sensor Oil Pressure back online" |
| Oil Temp (PRTXI) | `g_sensor_ch2_connected` | "Sensor Oil Temp offline" | "Sensor Oil Temp back online" |
| Trans Temp (PRTXI) | `g_sensor_ch3_connected` | "Sensor Trans Temp offline" | "Sensor Trans Temp back online" |
| Steer Temp (PRTXI) | `g_sensor_ch4_connected` | "Sensor Steer Temp offline" | "Sensor Steer Temp back online" |
| Diff Temp (PRTXI) | `g_sensor_ch5_connected` | "Sensor Diff Temp offline" | "Sensor Diff Temp back online" |
| Accelerometer | `isAccelerometerConnected()` I2C probe to LIS3DH (0x18 / 0x19) | "G-Sensor offline" | "G-Sensor back online" |
| OBD CAN | `g_obd_initialized && g_obd_success_count > 0` | "OBD CAN offline" / "OBD no ECU response" | "OBD CAN back online" |

### Toast Behavior

1. **Boot sequence:** 5 s after main screen loads -> full system check -> green "All Systems Online" or red multi-line error list
2. **Background monitoring:** Every 10 s, checks all systems for state changes
3. **Error detection:** Only shows a new toast when system transitions OK -> FAIL
4. **Recovery detection:** Shows green toast when system transitions FAIL -> OK
5. **Priority:** New errors take priority over recovery toasts

### SD Card Hot-Swap Recovery

When the background monitor detects the card is offline (`initialized = false`), it calls `sdTryReinit()`:
1. `SD.end()` to clean up state
2. `SD.begin()` to detect the card
3. On success: restores state, reads/increments boot count, calls `sdStartSession()`
4. Recovery toast "SD Card back online" fires
5. New `SESS_*.csv/.log` pair is created

This creates a new session with an incremented boot count; previous-session data is preserved.

### Adding New Sensors/Devices to Toast System

1. Add a `bool` field to `g_prev_system_status` struct (and add to the initializer list)
2. In `checkSystemsAndShowToast()`: compute current OK flag, append to `error_msg` and increment `error_count` if not OK, then store into `g_prev_system_status`
3. In `backgroundSystemMonitor()`: compute current OK flag (do any runtime probe like an I2C transaction), then for offline detection check `(!new_ok && g_prev_system_status.new_ok)`, and for recovery check `(new_ok && !g_prev_system_status.new_ok)`; concatenate to `error_msg` / `recovery_msg` and update `g_prev_system_status.new_ok` at the end

### Toast Styling

```cpp
lv_font_montserrat_20            // Font
lv_obj_set_style_pad_hor(..., 30, 0)
lv_obj_set_style_pad_ver(..., 18, 0)
lv_obj_set_style_radius(..., 0, 0)
lv_obj_set_style_bg_opa(..., LV_OPA_COVER, 0)
TOAST_COLOR_SUCCESS  0x2E7D32    // Green
TOAST_COLOR_ERROR    0xB71C1C    // Dark red
```

---

## Known Issues / Gotchas

1. **GPIO46 Conflict:** Cannot use GPIO46 for SD card — it is HSYNC for the display
2. **Hardware backlight has no PWM:** Variable brightness is implemented via a black LVGL overlay on the top layer (see `setBrightness()`); brightness >= 250 hides the overlay completely
3. **SquareLine Exports:** Regenerates `ui_*.c` files — don't edit directly unless you're adding to non-exported sections
4. **Boot Counter:** Stored on SD card at `/BOOTCNT.DAT` (+ `/BOOTCNT.BAK`) — a fresh card starts at 0
5. **Demo Mode:** Persists across reboots only via the `g_demo_mode` global (defaults to LIVE on every boot); 5-second hold on utility box toggles
6. **Touch task pinned to Core 1, not Core 0:** Historical CLAUDE.md docs said Core 0, but the code now pins `touchTask` to Core 1 to avoid I2C contention with RTC/SD operations on Core 0
7. **PRTXI wiring is confirmed correct as documented** (Pin 1 = V+, Pin 2 = Signal). Do NOT flip these in docs even though some PRTXI datasheets diagram them differently.
8. **CAN pins are GPIO19/20, not 17/18:** Some inline comments in the .ino header still reference the older "GPIO17/18" plan from before the onboard TJA1051T transceiver was used. The active configuration is GPIO19/20.

---

## Version History

| Version | Key Changes |
|---------|-------------|
| v5.8 | LIS3DH accelerometer (G-sensor) via I2C, logs X/Y/Z to CSV, toast monitor extended |
| v5.7 | OBD-II via CAN bus (TWAI) using onboard TJA1051T, Fuel Trust calculation, RPM/ECT via PIDs |
| v5.6 | Auto Brightness (sunrise/sunset based dimming) using NOAA solar calculator |
| v5.5 | UI shows "---" on Modbus disconnect, compact 1-line/sec logging, "!" critical indicators |
| v5.4 | 5-channel Modbus: Oil Press + Oil/Trans/Steer/Diff Temp via PRTXI 4-20mA |
| v5.3 | PRTXI +5°C calibration offset, temperature hysteresis (0.3°C) |
| v5.2 | Waveshare mode register fix (0x1001), µA direct reading |
| v5.1 | Oil temp gauge simplification |
| v5.0 | PRTXI-1/2N-1/4-4-IO RTD transmitter on CH2 (4-20mA mode) |
| v4.9 | Oil temperature sensor on Modbus CH2 |
| v4.8 | Toast notification system with runtime monitoring, error/recovery detection |
| v4.7 | File browser optimization, 8-digit session filenames |
| v4.6 | SD card file browser, CSV/text viewers |
| v4.5 | Sensor failure detection, "---" UI feedback |
| v4.4 | Splash screen with loading animation |
| v4.3 | USB MSC fixes, CSV header bugfix (FILE_APPEND -> FILE_WRITE) |
| v4.2 | Dual-core architecture |
| v4.1 | Unit conversion, tap-to-cycle units, persistent preferences |

---

## Quick Reference

```
Compile: Arduino IDE -> Upload (921600 baud)
Serial:  115200 baud
SD Card: FAT32, Class 10+
Logs:    /SESS_NNNNNNNN.csv  (data)
         /SESS_NNNNNNNN.log  (mirrored Serial)
Boot:    /BOOTCNT.DAT (+ /BOOTCNT.BAK)
Config:  /wifi.cfg (optional, for NTP)
```
