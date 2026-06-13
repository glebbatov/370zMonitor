# 370zMonitor - AI Assistant Reference

## Project Overview

**370zMonitor** is a track car data logging and display system for a 2018 Nissan 370Z. It uses an ESP32-S3 microcontroller with a 7" touchscreen to display real-time sensor data during track days.

- **Current Version:** v6.0
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
| CAN TX | 20 (CANTX, to onboard TJA1051T TXD) — shared w/ native USB via EXIO5 mux |
| CAN RX | 19 (CANRX, from onboard TJA1051T RXD) — shared w/ native USB via EXIO5 mux |
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
| 0 (AI1) | Oil Pressure (PX3AN2BH150PSAAX) | 0.5-4.5V direct (no divider, v5.9+) | Mode 0 (0-10V) |
| 1 (AI2) | Oil Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 2 (AI3) | Transmission Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 3 (AI4) | Power Steering Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 4 (AI5) | Differential Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 5-7 | Reserved for future sensors | - | - |

Channel mode is written at startup via Modbus Function 0x06 to holding registers `0x1000` (CH1) through `0x1004` (CH5). The PRTXI channels are auto-configured to Mode 3 on each boot in `initModbusSensors()`.

### Pressure Sensor Calibration (CH1)
```cpp
// PX3AN2BH150PSAAX: 0.5V-4.5V for 0-150 PSI
// v5.9: Direct wiring, no voltage divider. Sensor signal connects straight
// to Waveshare AI1 in Mode 0 (0-10V), which natively handles 0.5-4.5V.
// Effective formula in convertToPSI():
//   PSI = (raw_mV - 500) * 0.0375          (DIVIDER_RATIO is now 1.0)
// Clamped to [0, 150]
#define PRESSURE_DIVIDER_RATIO 1.0f       // No divider (v5.9+)
#define PRESSURE_OFFSET_MV     500.0f
#define PRESSURE_SCALE         0.0375f
```

### Wiring Diagram (PX3 oil pressure — v5.9+ direct, no divider)

**Sensor: 3-wire ratiometric voltage output. Red = +5V (Pin A), Black = GND (Pin B), Yellow = Signal (Pin C).**

```
                ┌──────────────────────────────────┐
                │   Fixed 5V regulator             │
                │   (Recom R-78E5.0-1.0 or equiv;  │
   24V rail ───▶│   NOT a JTAREA LM2596 — see #11) │
   GND      ───▶│   IN+              OUT+ ─────────┼──▶ +5.00V regulated rail
                │   IN−              OUT− ─────────┼──▶ Common GND
                └──────────────────────────────────┘

   +5V rail ─────────────────────▶ PX3 Red    (V+,     Pin A)
   Common GND ───────────────────▶ PX3 Black  (GND,    Pin B)
   PX3 Yellow (Signal, Pin C) ───▶ Waveshare AI1+
   Common GND ───────────────────▶ Waveshare AI1−

   Supply decoupling near sensor: 10 µF + 0.1 µF across +5V/GND
   (short leads, close to the PX3 supply branch)
```

**Confirmed PX3 wire colors (validated 2026-06-11 with multimeter and HANGELL bench supply):**
- **Red = V+** (Metri-Pack 150 Pin A)
- **Black = GND** (Pin B)
- **Yellow = Signal/Output** (Pin C)

The Metri-Pack connector on this particular PX3 has **no "A" stamp/cast** on the body — colors are the only practical key in the field. Verify electrically anyway (see resistance fingerprint below).

**Critical wiring rules:**
- Must run **3 conductors** end-to-end (V+, GND, Signal) — sensor will not function with 2 wires. The sensor body does NOT reliably ground through its 1/8-27 NPT thread; do not assume chassis grounding.
- **Supply must be 4.75 V–5.25 V (5.0 V nominal). 5.25 V is absolute max** per Honeywell datasheet — exceeding it damages the internal ASIC, typically rails the output high regardless of pressure. The June 2026 install cooked one sensor at 8.5 V from a broken JTAREA LM2596 module (see gotcha #11). **Verify supply with a multimeter, under load, before plugging the sensor in. Every time.**
- Shielded 3-conductor cable preferred for the engine-bay run; tie shield to GND **at the box end only**.
- **Verify connector pinout with a multimeter before powering.** Do not trust wire colors blindly. Resistance fingerprint of a healthy unpowered PX3 (2026-06-11 measurements):
  - V+ to GND (red↔black): **2–5 MΩ** (drifts as meter charges internal cap — normal)
  - V+ to Signal (red↔yellow): **2–5 MΩ** (same)
  - GND to Signal (black↔yellow): **~49 kΩ stable** (internal discharge path through output stage)

  The asymmetry — two high-MΩ readings and one stable ~49 kΩ — uniquely identifies V+ as the wire that reads MΩ against both others. The 49 kΩ pair is GND/Signal in some order; powering up at 5 V and looking for 0.50 V on one of them distinguishes which is which (Signal = 0.50 V, GND = bench ground reference).

### Pressure Sensor Install Status (as of 2026-06-11)

**Current state (sensor dead, replacement on order):**

Root cause confirmed: the JTAREA LM2596 buck module ([Amazon B0D2TS7CBN](https://www.amazon.com/dp/B0D2TS7CBN)) had a broken feedback path and was passing 8.5 V (close to its 24 V input) straight through to the sensor — 62 % over the PX3's 5.25 V absolute max. The sensor's ASIC output stage was cooked. Bench test on 2026-06-11 with the HANGELL HPS-3010D supply at 5.00 V on red, GND on black, multimeter on yellow showed **4.06 V at rest** (should be 0.50 V) and **zero response to mouth-blown pressure** — classic dead-ASIC signature.

Secondary issue: the original harness was 2-conductor (V+ and Signal only — GND missing), so even if the sensor had been healthy it couldn't have worked correctly on the car. The previously-installed "Frankenstein" inline resistors/divider in the harness have been removed.

**Actions:**
1. ✅ Confirmed sensor is dead via bench test (HANGELL @ 5.00 V, yellow rails at 4.06 V).
2. ✅ Confirmed Honeywell wire color convention (red/black/yellow → V+/GND/Signal).
3. ⏳ Order replacement PX3AN2BH150PSAAX (Newark, Digi-Key, or Mouser — ~$30–40, see distributor links in [Buck Converter & Supply Notes](#buck-converter--supply-notes) below).
4. ⏳ Replace the JTAREA LM2596 with a known-good fixed 5 V regulator (see options below) — do not reuse the JTAREA module under any circumstances.
5. ⏳ Add the missing GND conductor between the electric box and the sensor (use 3-conductor shielded cable end-to-end).

**Target state (after parts arrive):** Clean 3-conductor shielded harness sensor→box; reliable fixed-output 5.00 V regulator from the 24 V rail; no inline divider; firmware already on `PRESSURE_DIVIDER_RATIO = 1.0` (v5.9). Decoupling caps (10 µF + 0.1 µF) at the sensor V+ pin.

**Install sequence when parts arrive:**
1. Bench-test the new regulator standalone first: 24 V in, verify **5.00 V ± 0.05 V** at the output with a ~50–100 mA dummy load (e.g. 100 Ω resistor → ~50 mA). Confirm regulation holds under no-load too. If output drifts outside 4.9–5.1 V at any time, do not use it.
2. Bench-test the new sensor before installing in the car: HANGELL supply at 5.00 V, red→V+, black→GND, **expect yellow = 500 mV ± 50 mV at atmospheric pressure**. Apply pressure (bike pump or even mouth pressure) and confirm voltage climbs. Resistance fingerprint should also match the values above (~49 kΩ black↔yellow, MΩ for the other pairs).
3. Replace the 2-conductor harness from sensor location to the electric box with 3-conductor shielded cable. Shield to box-end GND only.
4. Inside the box: regulator IN+ from 24 V rail (fused 1 A), regulator OUT+ to sensor V+ wire, regulator OUT- to common GND (tied to Waveshare AI1-). Sensor Signal wire to Waveshare AI1+.
5. Add 10 µF (electrolytic) + 0.1 µF (ceramic) decoupling across V+/GND at the sensor end of the harness.
6. Power-up bench check before screwing the sensor back into the sandwich plate: `[MODBUS]` CH1 raw should read **~500 mV** and display **0 PSI**.
7. Reinstall sensor in oil filter sandwich plate, start engine, watch the `[MODBUS]` per-second log for smooth rise to ~30–60 PSI at idle, climbing with RPM.

### Buck Converter & Supply Notes

**Do not reuse the JTAREA LM2596 module.** Its feedback path is broken — outputs nearly the full input voltage under no-load and almost certainly under load too. This is what killed the original sensor. Common failure mode for cloned LM2596 modules sold on Amazon under unfamiliar brand names.

**Replacement options (all fixed 5 V output, drop-in 3-pin SIP, no trim pot):**

| Part | Input range | Output | Price | Best for |
|------|-------------|--------|-------|----------|
| [Recom R-78E5.0-1.0](https://www.digikey.com/en/products/detail/recom-power/R-78E5-0-1-0/4930585) | 8–28 V | 5 V @ 1 A | ~$7 | Industrial-grade, 3-yr warranty — best choice for automotive |
| [Murata OKI-78SR-5/1.5-W36-C](https://www.digikey.com/en/products/detail/murata-power-solutions-inc/OKI-78SR-5-1.5-W36-C/811-2196-5-ND/2259781) | 7–36 V | 5 V @ 1.5 A | ~$8 | Wider input range, more headroom over 24 V nominal |
| [Pololu D24V5F5](https://www.pololu.com/product/2843) | 5.1–36 V | 5 V @ 500 mA | ~$6 | Hobbyist favorite, board-mount; 500 mA is plenty for 3.5 mA PX3 load |

All three are switching topology (low heat), TO-220-compatible 3-pin form factor, fixed output (no trim — nothing to drift), and from reputable manufacturers (not Amazon-resold knock-offs). The PX3 draws ~3.5 mA, so any of these is massively over-specced — pick on input range and form factor.

**Recommendation:** Recom R-78E5.0-1.0 — industrial, used in EU automotive aftermarket gear, fits 7805 footprint, 3-year warranty. Per memory `component-preferences.md`, this matches the "industrial/enclosed parts from reputable brands" preference.

### Expected Validation Readings (PX3, v5.9 direct wiring)

| Condition | PX3 Signal pin | Waveshare AI1 (raw mV via Modbus) | Firmware shows |
|-----------|---------------|------------------------------------|----------------|
| Engine off, 0 psi, 5.00V supply | ~0.50V | ~500 mV | 0 PSI |
| Engine cold idle (typical) | ~1.0–1.5V | ~1000–1500 mV | ~19–38 PSI |
| Full scale, 150 psi | ~4.50V | ~4500 mV | 150 PSI |
| Pegged at 150 with engine off | sensor signal > 4.5V or supply wrong | > 4500 mV | 150 PSI (clamped) — sensor damaged or wiring fault |
| Stuck near 0 with engine running | signal floating / disconnected | < 100 mV | "---" (sensor offline per `SENSOR_MIN_VALID_MV`) |

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
| CAN terminal CANL | OBD-II Pin 14 (pigtail Brown/White) |
| CAN terminal CANH | OBD-II Pin 6 (pigtail Green) |
| Any board GND | OBD-II Pin 4 + 5 (pigtail Orange + Yellow, signal/chassis ground) |

**Remove the board's onboard 120Ω CAN termination jumper.** The car's CAN-C bus is already terminated at both ends (2×120Ω = 60Ω). Leaving the board terminator in makes it 40Ω, out of spec. See `obd_can_wiring.md` for the full harness + bring-up guide.

**EXIO5 mux gotcha:** GPIO19/20 are shared between native USB and the TJA1051T via an FSUSB42UMX mux selected by CH422G EXIO5 (`EXIO_CAN_SEL`). It must be driven HIGH (done in `initIOExtension()`) or CAN is electrically disconnected. This disables native USB / USB-MSC while CAN is active; flash & serial use the separate UART USB-C port.

### Configuration
```cpp
#define CAN_TX_PIN  GPIO_NUM_20     // CANTX -> TJA1051T TXD (per Waveshare pinout)
#define CAN_RX_PIN  GPIO_NUM_19     // CANRX <- TJA1051T RXD
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

## Vehicle Install — Power Distribution & Auxiliary Hardware

External vehicle hardware that shares the install's 12V distribution with the monitor. Documented here so future wiring/sensor work follows the established topology.

### Power Distribution Topology

```
Battery (+) ──► 60A MIDI/ANL master fuse (within 12" of battery)
            ──► 6 AWG main feed
            ──► Covered +/- bus bars (Blue Sea or equivalent, 100A+ rated)
            ──► Inline ATC/ATO fuse holders (Littelfuse / Blue Sea / Bussmann)
                 ▼  (each fuse holder mounted within ~7" of bus bar — tail wire is unprotected)
            ──► Per-circuit branches (each to its own relay where applicable)

Ground return: 6 AWG from (-) bus bar to clean, paint-stripped chassis stud.
```

### Per-Circuit Fuse Sizing

| Circuit | Fuse | Wire | Notes |
|---------|------|------|-------|
| 370zMonitor (electric box) | 5 A | 18 AWG | **Isolated from motor circuits** — fan inrush spikes must not brown out the monitor |
| Trans cooler fan (SPAL VA07-AP7C-31A) | 20 A | 12 AWG | ~10A continuous, ~17A inrush. Via Pico 5593PT relay (40A SPDT). Setrab thermo switch on coil ground |
| Engine oil cooler fan | 20 A | 12 AWG | Via own relay. Setrab 200°F thermal switch on coil ground |
| Z1 diff cooler **pump** | 10 A | 16 AWG | Tilton 40-524 (datasheet 98-1901). Via own relay |
| Z1 diff cooler **fan** | 7.5 A slow-blow | 18 AWG | SPAL 30103011 (5.2"). Inrush ~15–25 A for 100–300 ms — 5 A ATC nuisance-blows. Via own relay |

Pump and fan get **separate relays and fuses** so a stuck pump motor or shorted fan does not disable both halves of the diff cooler.

### Z1 Motorsports 370Z/G37 Differential Cooler Kit (P/N 21139)

| Component | Part | Spec |
|-----------|------|------|
| Pump | Tilton 40-524 (datasheet 98-1901) | 12 VDC, 2–3 A typical, 6.6 A max under load (8 A worst case per datasheet), Tilton-recommended **10 A inline fuse**, 16 AWG min stranded |
| Fan | SPAL 30103011 | 5.2" low-profile puller, 12 VDC, **2.2 A**, 313–342 CFM |
| Trigger | Setrab 200°F thermo switch (Z1 P/N 46687) | Installs in 1/8 NPT pilot hole on Z1 High Capacity Diff Cover, normally open, closes hot |
| Cooler core | Z1 ProCooler 25-row | — |

**Wiring topology in this install** (deviates from Z1 default — pump is manual, only the fan is auto-triggered):

- **Power distribution at diff**: single 10 AWG +12 V trunk feed from engine-bay bus bar (25 A fused) to a Blue Sea 5025 mini fuse block at the diff. From there, 10 A → pump relay pin 30, **7.5 A slow-blow** → fan relay pin 30.
- **Pump trigger (manual, OR logic)**: two SPST switches in parallel feed the +12 V ignition trigger (washer-tank fuse tap) to pump relay coil pin 86 at "node Y" — (a) Gardner Bender **GSW-49** illuminated rocker in the cabin dashboard (neon lamp stays dark on 12 V DC, switch contacts work fine) and (b) a DPST waterproof inline switch under the car at the diff (one pole wired, other capped). Either switch closes → pump runs. Driver watches diff temp on the 370zMonitor display and toggles manually.
- **Fan trigger (auto)**: Setrab 31-TS200-08 (200 °F NO, opens at 185 °F) inline between the +12 V trigger and fan relay coil pin 86 — **high-side switching**, matches Setrab's own install diagram.
- **Both Pico 5593PT relays**: pin 85 permanently grounded at the diff GND bolt, pin 87a (NC) capped and heat-shrunk.
- **No auto pump mode** — pump must be on for the cooler/fan to do useful work; this is a deliberate trust-the-driver decision (Tilton also warns against running pump on cold heavy gear oil).
- **PRTXI temp sensor (CH5)**: 2-wire 4-20 mA loop from electric box to diff. Belden 8761 shielded twisted pair. Shield grounded at the Waveshare end only; diff end capped and heat-shrunk.

Detailed wire-by-wire plan and SVG schematic: see `diff_cooler_wiring.md` in the repo root.

### Bus Bar Hygiene

- Positive bus bar **must be covered** — exposed +12V studs near each other are a wrench-drop short hazard
- Inline fuse holders located within ~7" of the bus bar (tail wire is unprotected)
- Avoid no-name Amazon fuse holders for sustained current near rating — spring contacts and crimps fail. Stick to Littelfuse / Blue Sea / Bussmann / Eaton

### Setrab TS200 Reference (for trans, oil, and diff fans/pumps)

- Normally open, closes when hot. **~200 °F (~93 °C) close / ~185 °F (~85 °C) open** per Setrab spec sheet (~15 °F hysteresis; verify body stamp on the specific unit)
- **10 A @ 12 V DC** contact rating — switching ~150 mA relay coil is trivial
- Wire **high-side** (between ignition-switched +12 V and relay coil pin 86) per Setrab's own install PDF. Low-side (pin 85 → switch → ground) works electrically too, but high-side keeps installs consistent across all three Setrab-controlled circuits (trans, engine oil, diff)
- The **AN08 variant** (`31-TS200-08`, used on the Z1 diff cover via the SUSA AN-male-to-AN-female adapter) has **2 spade terminals, electrically isolated from the case** — non-polarity-sensitive, either terminal either direction
- **Single-spade Type-22** housing variants ground through the threads into the fitting and need a clean metal-to-metal interface
- **PTFE tape on the 1/8" NPT: 1.5–2 turns max** — more turns can distort the port and throw off the switching temperature

---

## Known Issues / Gotchas

1. **GPIO46 Conflict:** Cannot use GPIO46 for SD card — it is HSYNC for the display
2. **Hardware backlight has no PWM:** Variable brightness is implemented via a black LVGL overlay on the top layer (see `setBrightness()`); brightness >= 250 hides the overlay completely
3. **SquareLine Exports:** Regenerates `ui_*.c` files — don't edit directly unless you're adding to non-exported sections
4. **Boot Counter:** Stored on SD card at `/BOOTCNT.DAT` (+ `/BOOTCNT.BAK`) — a fresh card starts at 0
5. **Demo Mode:** Persists across reboots only via the `g_demo_mode` global (defaults to LIVE on every boot); 5-second hold on utility box toggles
6. **Touch task pinned to Core 1, not Core 0:** Historical CLAUDE.md docs said Core 0, but the code now pins `touchTask` to Core 1 to avoid I2C contention with RTC/SD operations on Core 0
7. **PRTXI wiring is confirmed correct as documented** (Pin 1 = V+, Pin 2 = Signal). Do NOT flip these in docs even though some PRTXI datasheets diagram them differently.
8. **CAN pins: GPIO20=CANTX, GPIO19=CANRX (do not swap), and they are MUXED with native USB.** Two things bit the June 2026 install and produced *zero* CAN data: (a) CH422G **EXIO5 (`EXIO_CAN_SEL`) was never driven HIGH**, so the FSUSB42UMX mux left GPIO19/20 on the native-USB port and the TJA1051T was disconnected from the MCU; (b) **TX/RX were swapped** in firmware (had TX=19/RX=20). Fixed in v6.0. Per Waveshare: GPIO20=CANTX, GPIO19=CANRX. Setting CAN mode disables native USB (USB-MSC can't run with CAN active) — use the separate UART USB-C port for flash/serial. Ignore any older "GPIO17/18" comments.
9. **PX3 oil pressure sensor is 3-wire, not 2-wire:** It outputs ratiometric 0.5–4.5V on a separate Signal pin. Running only 2 conductors (V+/GND) leaves the AI1 input floating and produces a pegged 150 PSI reading. This bit the install in June 2026 — 8 months of bench testing showed 0 PSI because both bench grounds were at the same potential and signal floated low; on the car the floating input read near rail.
10. **PX3 wire colors (this connector): red=V+, black=GND, yellow=Signal.** Confirmed 2026-06-11 by bench test. The Metri-Pack 150 on this unit has no "A" cast/stamp on the body — colors are the only field key. Always cross-check with the resistance fingerprint (red↔others = MΩ, black↔yellow = ~49 kΩ) before applying power.
11. **JTAREA-branded LM2596 modules ([Amazon B0D2TS7CBN](https://www.amazon.com/dp/B0D2TS7CBN)) are unsafe — do not use.** The unit installed in 2026 had a broken feedback path and passed 8.5 V (near its 24 V input) straight through under no load, cooking the original PX3 sensor at 62 % over its 5.25 V absolute max. A working LM2596 should regulate independent of load — if it doesn't, the chip has no feedback. Use fixed-output industrial parts (Recom R-78E5.0-1.0, Murata OKI-78SR, Pololu D24V5F5) instead of any adjustable Amazon-resold LM2596. See [Buck Converter & Supply Notes](#buck-converter--supply-notes).
12. **PX3 absolute max supply is 5.25 V (datasheet), not 18 V.** Earlier CLAUDE.md said "~18V" — that was wrong and contributed to the June 2026 sensor death. The PX3 ASIC is regulated at ~5 V internally; sustained supply above 5.25 V damages the output stage permanently. Symptom of overvoltage damage: signal pin sits near rail (~4 V) at rest, ignores pressure changes, even after correct supply is restored.
13. **Dead-sensor signature (post over-voltage):** Signal pin rails high (~3.5–4.5 V) at atmospheric pressure on a correct 5 V supply, with **no measurable response to applied pressure**. The diaphragm may still be mechanically fine, but the signal-conditioning ASIC is cooked. There is no field repair — replace the sensor. Order from Newark, Digi-Key, or Mouser; avoid eBay/Amazon for this part.
14. **The PDF `PX3AN2BH150PSAAX_ESP32_3V3_Interface_Breakdown.pdf` describes the OLD design** (PX3 → 10kΩ/22kΩ divider → ESP32 GPIO 3.3V ADC directly). The current architecture uses a Waveshare Modbus module in Mode 0 (0-10V), so the divider was redundant and was removed in v5.9. Refer to the PX3 wiring diagram in this CLAUDE.md (not the PDF) for the active design.

---

## Version History

| Version | Key Changes |
|---------|-------------|
| v6.0 | **OBD CAN fixed.** Drive CH422G EXIO5 (`EXIO_CAN_SEL`) HIGH to route the FSUSB42UMX mux to the TJA1051T (GPIO19/20 are shared with native USB); un-swapped CAN pins (now TX=GPIO20, RX=GPIO19). See `obd_can_wiring.md` |
| v5.9 | Removed PX3 voltage divider — direct wiring to Waveshare AI1 (Mode 0 handles 0.5-4.5V natively); `PRESSURE_DIVIDER_RATIO` is now 1.0 |
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
