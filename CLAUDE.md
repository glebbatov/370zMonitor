# 370zMonitor - AI Assistant Reference

## Project Overview

**370zMonitor** is a track car data logging and display system for a 2018 Nissan 370Z. It uses an ESP32-S3 microcontroller with a 7" touchscreen to display real-time sensor data during track days.

- **Current Version:** v6.13
- **Hardware:** Waveshare ESP32-S3-Touch-LCD-7 (800x480) with onboard TJA1051T CAN transceiver and SP3485 RS485 transceiver
- **Hardware:** Waveshare Industrial 8-Ch Analog Acquisition Module (Model B, 0-10V / 0-20mA / 4-20mA selectable)
- **Hardware:** Crowtail I2C Hub 2.0 (for multiple I2C devices)
- **Hardware:** HW-084 DS3231 RTC module (battery-backed real-time clock)
- **Sensor:** Oil Pressure: P51-150-G-B-P-20MA-000-000 (Channel 1 / AI1, Mode 3 = 4-20mA loop) — replaced the PX3 voltage sensor in v6.2
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
├── lv_conf.h                    # Reference copy of the working LVGL v9 config (copy to Arduino libraries root to build)
└── CLAUDE.md                    # This file
```

---

## Version Sync Checklist

**When updating version number, change ALL THREE locations:**

1. `Arduino/370zMonitor/370zMonitor.ino` - Header comment block at top (line ~4: `* 370zMonitor v6.3`)
2. `Arduino/370zMonitor/ui_ScreenSplash.c` - `lv_label_set_text(version_label, "vX.X")` (~line 125)
3. `Arduino/370zMonitor/370zMonitor.ino` - Serial boot banner (~line 8181: `Serial.println("   370zMonitor vX.X - Dual-Core + G-Sensor")`)

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
| Oil Pressure | 0 PSI | 150 PSI | v6.7: RPM-aware — critical below `max(10 psi/1000 rpm, 10 psi idle floor)` while running, or `>120` PSI overpressure (warm-oil assumption; drives Value Critical) |
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
timestamp_ms,elapsed_s,cpu_percent,mode,
[v6.9 appended, 22 cols] throttle_pct,throttle_valid,engine_load_pct,engine_load_valid,
  intake_air_temp_f,intake_air_temp_valid,ambient_temp_f,ambient_temp_valid,
  obd_oil_temp_f,obd_oil_temp_valid,maf_gps,maf_valid,commanded_lambda,lambda_valid,
  module_voltage,module_voltage_valid,fuel_level_pct,fuel_level_valid,baro_kpa,baro_valid,
  fuel_sys_status,fuel_sys_valid,
[v6.10 appended, 12 cols] stft_b1,stft_b2,ltft_b1,ltft_b2,fuel_trim_valid,
  ft_timing_deg,pen_stft,pen_ltft,pen_bank,pen_timing,mil_on,dtc_count
```
Columns are only ever **appended** (v6.9, then v6.10), so every pre-existing column keeps its
position — old parsers still work. Total = 32 base + 22 (v6.9) + 12 (v6.10) = **66 columns**.
A `# 370zMonitor FW vX.Y` comment line is written just under the header (v6.10), and the
actual DTC code strings go to the `.log` on a `[DTC]` line (the CSV logs only `mil_on`/`dtc_count`).

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

### Channel Mapping (as of v6.6 — ORIGINAL 5-channel layout restored)
| Channel | Sensor | Signal | Waveshare Mode |
|---------|--------|--------|----------------|
| 0 (AI1) | **Oil Pressure (P51-150-G-B-P-20MA)** — via isolated 24V DC-DC (see Install Status) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 1 (AI2) | Oil Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 2 (AI3) | Transmission Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 3 (AI4) | Power Steering Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 4 (AI5) | Differential Temperature (PRTXI 4-20mA) | 4-20mA loop-powered | Mode 3 (4-20mA) |
| 5-7 (AI6-AI8) | Reserved for future sensors | - | - |

**v6.6 restores the clean original mapping.** The v6.3–v6.5 channel swaps (oil pressure shuffled CH1→CH6→CH2→CH3) were all chasing the oil-pressure dropout, which turned out to be a **grounding/isolation fault, not any bad channel** — see "Pressure Sensor Install Status" below and `oil_pressure_isolation_saga.md`. All five channels are good. Firmware: `MODBUS_CH_OIL_PRESSURE=0`, `OIL_TEMP=1`, `TRANS_TEMP=2`, `STEER_TEMP=3`, `DIFF_TEMP=4`; `MODBUS_NUM_CHANNELS=5`; boot configures CH1–CH5 for Mode 3.

**All five channel jumpers must be ON (current mode)** — AI1 especially, which spent the saga in voltage mode (the original "CH1 dead" was that jumper-OFF mistake, not a dead channel). The Waveshare per-channel jumper selects **ON = current (sense resistor in) / OFF = voltage (high-Z)**; a 4-20mA sensor needs the jumper ON *and* software Mode 3. A 4-20mA loop into a voltage-mode (high-Z) input rails to full scale (20000 µA / 150 PSI), which is exactly what produced the bogus "dead CH1" read.

**Physical wiring (v6.6):** oil pressure on **AI1**, fed from a dedicated **isolated 24V DC-DC** (converter +24V out → P51 Pin1; Pin3 → AI1+; AI1− → converter −out). The four PRTXI temps on AI2–AI5 wire as before (24V loop, return to their AI+). All AI1–AI5 jumpers ON.

### Pressure Sensor Calibration (CH1)
```cpp
// P51-150-G-B-P-20MA-000-000: 4-20mA loop output, 0-150 PSI gauge.
// v6.2: Replaced the PX3 (0.5-4.5V) with the P51 4-20mA current-loop sensor.
// Waveshare CH1 is now Mode 3 (4-20mA) like CH2-CH5, so the module returns
// microamps (uA) directly. Loop-powered off the 24V rail - no buck, no divider.
//   4000  uA (4mA)  = 0 PSI
//   20000 uA (20mA) = 150 PSI
// Effective formula in convertToPSI():
//   PSI = ((uA - 4000) / 16000) * 150        // clamped to [0, 150]
#define PRESSURE_MIN_CURRENT_UA   4000     // 4mA = 0 PSI
#define PRESSURE_CURRENT_SPAN_UA  16000    // 20mA - 4mA span
#define PRESSURE_FS_PSI           150.0f   // full-scale at 20mA
// Disconnect: CH1 uses PRTXI_MIN_VALID_UA (<3mA = open loop), same as CH2-CH5.
```

### Wiring Diagram (P51 oil pressure — 4-20mA loop, v6.2+)

**2-wire 4-20mA current loop, loop-powered. Function is fixed by connector PIN, not wire color: Pin 1 = Vin (8-30V), Pin 2 = NC (unused on the 4-20mA variant), Pin 3 = loop return.**

> **v6.6 update — power the loop from an ISOLATED 24V DC-DC, not the shared 24V rail.** The "+24V rail" in the diagram below must be the output of a dedicated isolated DC-DC (Mean Well DDR-15G-24 / Traco TRN 3-1215) fed from car 12V, with its −out tied only to AI1−. Powering the loop from the box's shared rail is exactly what caused the long dropout saga (grounding/isolation fault). See "Pressure Sensor Install Status" and `oil_pressure_isolation_saga.md`.

```
   +24V rail --[fuse]--+-----------------> P51 Pin 1 (Vin, 8-30V)
                       |                         |
                  1.5KE36A TVS            (sensor regulates 4-20mA)
                  band(cathode) -> +24V          |
                       |                   P51 Pin 3 (loop return)
                       |                         |
                       |                         v
                       |                   Waveshare AI1+
                       |                         |  (internal sense resistor)
                       |                   Waveshare AI1-
                       |                         |
   GND (24V-) ---------+-------------------------+

   P51 Pin 2 = NC  ->  cut back, cap, heat-shrink
```

**Wire colors on this unit: red / black / yellow. Bench-confirmed mapping (2026-06-14, by powered sweep):**

- **Red -> +24V** (P51 Vin)
- **Black -> Waveshare AI1+** (P51 loop return)
- **Yellow -> UNUSED** — cap it (the old 3-wire *signal* lead; a 2-wire current loop has no use for it)
- **Waveshare AI1- -> GND (24V-)**

Why it lands this way: a 2-wire 4-20mA loop uses only supply (+) and ground/return (-). On this Honeywell-spec pigtail ([Amazon B09G65X1YC](https://www.amazon.com/dp/B09G65X1YC)), red = supply = P51 Vin, black = ground = the loop return, and yellow = the old voltage *output* pin, which a current loop doesn't use. (Earlier datasheet-only guesses — first red=Vin/yellow=AI+, then black=Vin — were both wrong on the details; only the powered sweep settled it.)

**Finding the pins on a replacement — the ohm test does NOT work here:** unpowered, a 4-20mA transmitter is high-impedance, so all three wires read open (useless for ID). Sweep *powered* instead, and **at 12V, not 24V** — reverse-polarity combinations in the sweep otherwise exceed the P51's +/-16V reverse limit and can damage the sensor. Cap one wire, try the other two as the loop in both polarities, and watch for ~4 mA (or a sane PSI on CH1); the combo that draws ~4 mA gives Vin (+) and the return. Once known, only ever wire it forward (red=+24V, black=AI1+) — never reverse it.

Shielded 2-conductor cable for the engine-bay run; shield to GND at the box end only.

### Pressure Sensor Install Status (RESOLVED 2026-06-20 — grounding/isolation, NOT the module · ✅ VALIDATED on-car 2026-07-04)

> **Both earlier verdicts were WRONG** — it was never a defective sensor ("RMA the P51") and never a bad module ("replace the Waveshare"). The real cause was a **grounding/isolation fault**: the P51's 4-20mA loop shared the electric box's power-and-ground, so its 4 mA had a sneak path *around* the Waveshare's 500 Ω sense resistor and the module read ≈0/`---`. **Fix = power the loop from an isolated 24V DC-DC.** Full narrative in **`oil_pressure_isolation_saga.md`**. (Old verdicts kept only in the version history.)

**✅ VALIDATED ON-CAR 2026-07-04 — the fix works.** The isolated DDR-15G-24 is installed (input off car 12V, −out to AI1− only) and oil pressure now reads correctly under power: an 11-minute drive (`SESS_00000405`) logged **99.7% valid, 37–112 PSI**, tracking rpm and warmup, **stable with the engine on and off** — the dropout is gone. Boot system check reports "Oil Pressure Sensor: OK". Grounding/isolation diagnosis confirmed correct. *(Unrelated: the **oil-temp** PRTXI on AI2 has failed — reads offline — and is being replaced; see next steps.)*

**THE SENSOR IS GOOD — proven 2026-06-17.** With the P51 on a fully-verified loop, a voltmeter reads **2.074 V at AI+ (to GND)** at atmosphere. The Waveshare 4-20mA input uses a **500 Ω sense resistor** (its "2–10 V" range = 4–20 mA × 500 Ω), so 2.074 V = **~4.15 mA = ~0 PSI** — a textbook-correct zero reading. The sensor is regulating current correctly. It also responded to applied pressure (values changed) during an earlier bench test on CH6. **Do not RMA the sensor.**

**Why the early "dead CH1 / toast sensor" reads were bogus — the jumper.** The Waveshare has a **per-channel hardware jumper: ON = current mode (4-20mA sense resistor in circuit) / OFF = voltage mode (high-Z 0-10V input)**. A 4-20mA sensor needs the jumper **ON** *and* software Mode 3. **CH1's jumper was left OFF** (a leftover from the PX3 voltage sensor). A 4-20mA loop driven into a voltage-mode (high-Z) input has no sense resistor to flow through, so the sensor's current source rails the input → module reads **full scale = 20000 µA = 150 PSI**, dead-steady. That is exactly the "CH1 is dead / sensor is toast" symptom — but it was the jumper, not a dead channel or sensor. **CH1 was never retested with its jumper ON; it may be fine.** (The user later reproduced this: flipping a channel's jumper OFF → 150 PSI, back ON → real reading.)

**The real problem = grounding/isolation (NOT the module).** The module is fine — a sibling 4-20mA channel (diff temp PRTXI) read rock-steady on it the whole time, and oil pressure read a clean 4 mA whenever its loop was on an *isolated* supply. The dropout followed the **sensor's loop**, never a channel. The defining test: the P51 works on an **isolated/floating** supply (a bench supply dedicated to just the loop) and fails on **box power** (the Victron rail or the car battery, both sharing the system ground). A 4-20mA loop must return its current through the module's sense resistor; sharing the box ground gave the 4 mA a parallel path that bypassed the resistor, so the module read ≈0. (Why the readings were binary 4.00-or-0: a diverted loop reads full current when the sneak path is open and ≈0 when it's active — never an in-between value. The chronic RS485 drops were a separate power-up artifact; the temps prove the bus is fine.)

**Fix:** power the oil-pressure loop from an **ISOLATED 24V DC-DC** (Mean Well **DDR-15G-24** or Traco **TRN 3-1215**), input from **car 12V** (not the box 24V rail — that re-bonds the grounds across the converter and defeats it), output **+24V → P51 red, −out → AI1− only**. 24V output is required for headroom to full-scale 150 PSI through the 500 Ω burden. Do NOT RMA the sensor or replace the module.

**Pin mapping (734-1165-ND harness, confirmed by PIN not color, 2026-06-17):** Pin 1 = +24V (Vin), Pin 2 = **NA / unused** (not a ground — leave open), Pin 3 = loop return → **AI+**; AI− → supply (−). Continuity verified: 12V+↔Pin1 0.1 Ω, Pin3↔AI+ 0.3 Ω, AI−↔12V− 0.2 Ω, Pin2 isolated.

**Current firmware: v6.6** — original 5-channel mapping restored: oil pressure on **AI1** (via the isolated DC-DC), oil/trans/steer/diff temps on **AI2–AI5**. `MODBUS_CH_OIL_PRESSURE=0`, `OIL_TEMP=1`, `TRANS_TEMP=2`, `STEER_TEMP=3`, `DIFF_TEMP=4`. **Status — DONE + validated on-car 2026-07-04:** the isolated DDR-15G-24 is installed and oil pressure reads correctly under power (99.7% valid, 37–112 PSI over an 11-min drive, stable engine on/off). See `oil_pressure_isolation_saga.md` and `ddr_isolator_tvs_wiring.md`.

**How to measure a 4-20mA loop without an inline mA meter** (the user's KAIWEETS HT206D is a **clamp meter** — current only via the 60A/600A clamp jaw, no series mA mode, useless for 4-20mA):
- **Voltmeter trick (no parts):** DC volts at **AI+ to GND ÷ 500 Ω = loop current.** 2.0 V = 4 mA (0 PSI), 10 V = 20 mA (150 PSI). (Cross-check the 500 Ω by measuring a known temp channel: V_AI+ ÷ its known mA.)
- **Resistor substitution (bypass the module):** 24V+ → Pin1; Pin3 → [any 100–500 Ω resistor] → 24V−; measure V across the resistor; current = V ÷ R. Confirms the sensor independent of the Waveshare.

**Open items / next steps:**
1. ✅ **DONE** — oil-pressure loop on the isolated DDR-15G-24, validated on-car 2026-07-04 (99.7% valid, 37–112 PSI, stable engine on/off). Sensor and Waveshare module both confirmed good; the old "replace the module" plan was unnecessary — the real cause was grounding/isolation.
2. ✅ **DONE** — P51 is plumbed in the oil-filter sandwich plate and reading real pressure (~37 PSI warm idle, up to 112 PSI, climbs with rpm).
3. 🔧 **TODO (hardware) — replace the oil-temp PRTXI on AI2.** It has failed: reads open/offline (<3 mA, 0% valid) in every session while trans/steer/diff read clean ~8.8 mA and the module/bus are healthy. Swap in a new PRTXI on AI2 (jumper ON; 24V→Pin1, Pin2→AI2+, AI2−→GND per the PRTXI wiring diagram). Interim: the car's **factory oil-temp gauge** covers oil-temp monitoring until the PRTXI is back, so this is not a track-day blocker.
4. ⏳ **TODO** — add the 1.5KE36A TVS, band(cathode)→+24V, across the **Victron 24V OUTPUT** rail for transient protection (see `ddr_isolator_tvs_wiring.md`).

Why the PX3 died (historical): the JTAREA LM2596 buck module ([Amazon B0D2TS7CBN](https://www.amazon.com/dp/B0D2TS7CBN)) had a broken feedback path and passed 8.5 V straight to the sensor — 62 % over the PX3's 5.25 V absolute max — cooking its ASIC. The 4-20mA P51 removes this entire failure class: loop-powered off 24V, no regulated-5V rail to fail.

### Buck Converter & Supply Notes (historical — no longer used)

**The P51 4-20mA sensor needs no buck converter** — it is loop-powered directly off the 24V rail (8-30V). The buck converter is gone from the oil-pressure circuit; this section is retained only as a caution.

**Do not reuse the JTAREA LM2596 module ([Amazon B0D2TS7CBN](https://www.amazon.com/dp/B0D2TS7CBN)).** Its feedback path is broken — it passed ~8.5 V straight through and cooked the original PX3. A common failure mode for cloned LM2596 modules sold under unfamiliar Amazon brand names. If any *future* circuit needs regulated 5V, use a fixed-output industrial part (Recom R-78E5.0-1.0, Murata OKI-78SR-5, or Pololu D24V5F5) — never an adjustable Amazon-resold LM2596. Matches the "industrial / enclosed parts from reputable brands" preference (memory `component-preferences.md`).

### Expected Validation Readings (P51, 4-20mA, v6.2)

| Condition | P51 loop current | Waveshare AI1 (raw uA via Modbus) | Firmware shows |
|-----------|------------------|------------------------------------|----------------|
| Engine off, 0 psi | 4.0 mA | ~4000 uA | 0 PSI |
| Engine cold idle (~30 psi) | ~7.2 mA | ~7200 uA | ~30 PSI |
| ~60 psi | ~10.4 mA | ~10400 uA | ~60 PSI |
| Full scale, 150 psi | 20.0 mA | ~20000 uA | 150 PSI |
| Open loop / sensor offline | <3 mA | <3000 uA | "---" (offline per `PRTXI_MIN_VALID_UA`) |
| Pegged at 150, engine off | >20 mA | >20000 uA | 150 PSI (clamped) — overpressure or wiring fault |

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
- CH1: `µA < PRTXI_MIN_VALID_UA (3000)` => P51 pressure loop disconnected (4-20mA as of v6.2; was mV-based in the PX3 era)
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

### Polled PIDs (round-robin, one PID every 120ms as of v6.9)
| PID | Meaning | Formula | Since |
|-----|---------|---------|-------|
| 0x05 | Engine Coolant Temp (ECT) | `A - 40` [°C] | — |
| 0x0C | Engine RPM | `((A*256)+B)/4` | — (oversampled ×5) |
| 0x0D | Vehicle Speed | `A` [km/h] | — |
| 0x0E | Timing Advance | `(A - 64) / 2` [°BTDC] | — |
| 0x06/0x07 | STFT/LTFT Bank 1 | `(A-128)*100/128` [%] | — |
| 0x08/0x09 | STFT/LTFT Bank 2 | `(A-128)*100/128` [%] | — |
| 0x11 | Throttle position | `A*100/255` [%] | v6.9 (oversampled ×3) |
| 0x04 | Calculated engine load | `A*100/255` [%] | v6.9 |
| 0x0F | Intake air temp (IAT) | `A - 40` [°C] | v6.9 |
| 0x46 | Ambient air temp | `A - 40` [°C] | v6.9 |
| 0x5C | Engine oil temp | `A - 40` [°C] (Nissan support varies) | v6.9 |
| 0x10 | MAF air flow | `((A*256)+B)/100` [g/s] | v6.9 |
| 0x44 | Commanded lambda | `((A*256)+B)/32768` | v6.9 |
| 0x42 | Module/battery voltage | `((A*256)+B)/1000` [V] | v6.9 |
| 0x2F | Fuel level | `A*100/255` [%] (support varies) | v6.9 |
| 0x33 | Barometric pressure | `A` [kPa] | v6.9 |
| 0x03 | Fuel system status | raw byte A | v6.9 |
| 0x01 | Monitor status (MIL + DTC count) | `A` bit7=MIL, bits0-6=count | v6.10 |

**v6.9:** poll period **120ms** (was 200), stale threshold **5s** (was 3), ~25-slot sequence (RPM ×5, throttle ×3) = ~3s full cycle. Unsupported PIDs stay invalid. New values decode into `g_obd_data` + `g_vehicle_data`, log to the CSV (22 appended columns: `throttle_pct…fuel_sys_valid`), and print on a new `[OBD2]` serial line (value = supported, `-1` = not). OBD oil temp (0x5C) is logged as `obd_oil_temp_f`, **separate** from the (dead) Modbus oil-temp sensor. Logging only — no gauges.

### OBD Mode 03 — Trouble Codes (DTCs) (v6.10)
Beyond the round-robin Mode 01 PIDs, v6.10 also issues a **Mode 03** ("read stored DTCs") request every `DTC_READ_INTERVAL_MS = 15000` (and on-demand when the Fuel Trust popup opens). The reply (service byte `0x43`) is reassembled via ISO-TP in `handleDTCFrame()` — single-frame directly, multi-frame with a Flow-Control frame to the ECU's physical address (`resp_id − 8`), timeout `DTC_ASM_TIMEOUT_MS = 200`. Each 2-byte DTC is decoded to a `Pxxxx/Cxxxx/Bxxxx/Uxxxx` string by `dtcToString()` and stored in `g_dtc_list[]`. Codes print on a `[DTC]` serial line and show in the Fuel Trust popup; the CSV logs only `mil_on` + `dtc_count` (from PID 0x01, single-frame and always reliable). **Assumes the ISO 15765-4 CAN Mode-03 count byte** (`0x43,count,pairs…`) — verify the decoded codes against a scanner on first flash. Failure is benign (empty/partial list; MIL+count still show).

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
- **Axes (car frame, after v6.7 calibration):**
  - X: Lateral (positive = right)
  - Y: Longitudinal (positive = forward acceleration)
  - Z: Vertical (positive = upward, ~1.0g at rest)
- **v6.7 mounting calibration:** the LIS3DH sits at an arbitrary angle, so `readAccelerometer()` rotates the raw sensor axes into the car frame with a fixed matrix (`ACCEL_R_*`, gated by `ENABLE_ACCEL_CALIBRATION`). Derived from 55 mi of 7/5 logs: the gravity vector fixes level/vertical **exactly**; the forward axis (rpm-correlated) is **provisional ~±20°**. Vertical and total-horizontal-G are trustworthy; the lateral-vs-longitudinal split may need a sign flip after an on-track hard-brake / steady-corner check. Re-derive from the 10 Hz data when convenient.

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
- **v6.9: AUTOMATIC DST — no more seasonal reflash.** The DS3231 now stores **UTC**; local time (incl. the spring/fall hour) is derived from a POSIX TZ rule `POSIX_TZ = "CST6CDT,M3.2.0,M11.1.0"` via `readRTCLocal()` / `getLocalTime()`. `GMT_OFFSET_SEC`/`DAYLIGHT_OFFSET_SEC` are now legacy (GMT_OFFSET is used only by the sunrise/sunset calc, which adds the DST hour itself). To move timezones, change only `POSIX_TZ`. **After flashing v6.9, do ONE WiFi/NTP boot** so the RTC is rewritten in UTC — until then the old local-time RTC value reads ~5–6 h off. Helpers: `ensureTimezone()`, `utcTmToEpoch()`, `readRTCLocal()`.
- **v6.8 NTP-sync fix:** `tryNTPSync()` no longer accepts the stale RTC-seeded system clock as a "successful" sync (it seeds an old sentinel and waits for a genuinely fresh SNTP result, year≥2025), so NTP actually corrects the clock instead of re-cementing the RTC's error. Root cause of the old days-behind/non-monotonic timestamps. Also check the DS3231 coin cell.
- **v6.7 file-timestamp fix:** `syncSystemTimeFromRTC()` sets the ESP32 system clock from the DS3231 at boot via `settimeofday()` (now with UTC), so SD/FAT **file timestamps** are correct even with no WiFi/NTP.
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
| Single tap on **Fuel Trust** value | Open the Fuel Trust breakdown popup (both banks' STFT/LTFT, the 4 penalties, meaning, + active DTCs). Tap anywhere to close. (v6.10) |
| Single tap on utility-box brightness button | Manual override (flip to opposite brightness) |
| Double-tap anywhere | Show utility box |
| 5-second hold on utility box | Toggle Demo / Live mode (`DEMO_MODE_TOGGLE_HOLD_MS`). v6.13: LIVE→DEMO is **blocked while the engine is running** (rpm>300) to stop phantom-touch triggers; DEMO→LIVE always allowed. A full-width "DEMO - SIMULATED DATA" banner shows whenever demo is active. |
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
| `SD_FLUSH_INTERVAL_MS` | 1000 | SD flush-to-card frequency (batched, not per row) |
| `SD_WRITE_INTERVAL_MS` | 100 | v6.7: data-row write cadence — **10 Hz** (was 1000 / 1 Hz) to catch sub-second oil-pressure dips |
| `SD_QUEUE_SIZE` | 40 | v6.7: raised from 16 to buffer 10 Hz logging against flush stalls |
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
| USB CDC On Boot | **Disabled** (see note) |
| CPU Frequency | 240MHz (WiFi/BT) |
| PSRAM | OPI PSRAM |
| Flash Mode | QIO 80MHz |
| Flash Size | 16MB (128Mb) |
| Partition | 16M Flash (3MB APP / 9.9MB FATFS) |
| Upload Mode | UART0 / Hardware CDC |
| Upload Speed | 921600 |

**Flashing & serial use the UART USB-C port, NOT the native-USB port.** Reasons:
- `USB CDC On Boot` is **Disabled**, so `Serial` maps to UART0 → the onboard CH343 USB-UART bridge (the "UART" Type-C connector). The board has a **UART-select switch** that must be set to route that port to the console.
- When CAN is enabled (`CAN_MUX_TO_CAN 1`), EXIO5 flips the FSUSB42 mux to CAN mode and the **native USB port goes dark** (GPIO19/20 are shared — see gotcha #8). So always flash/monitor over the UART port.
- To flash when the running app has taken over the port: enter download mode manually — **hold BOOT, tap RESET, release BOOT**, then Upload.

A `platformio.ini` is also present for PlatformIO users (it correctly sets `ARDUINO_USB_CDC_ON_BOOT=0`; update its `flash_size` to 16MB and partition to `default_16MB`/FATFS to match the board).

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
`[MODBUS] [OBD] [SD] [RTC] [TIME] [WIFI] [NTP] [G-SENSOR] [TOAST] [MONITOR] [SYSTEM CHECK] [AUTO-BRI] [PREFS] [CORE0/SD] [CORE1] [STATUS] [SESSION] [CHARTS]`

`[SESSION]` (v6.7): per-second running summary — `min_oilP(>2k)` (lowest oil PSI above 2000 rpm), `peak_oilT`, and `maxG lat/lon/vert`.

---

## Required Libraries

**Versions matter — the code is written against specific major versions. Mismatches cause hundreds of compile errors or runtime failures. See the "LVGL v9 / Display Stack" section below for the full setup.**

1. **GFX Library for Arduino** (moononournation) — installs the header `Arduino_GFX_Library.h`. Tested with **v1.6.6**. NOTE: the Library-Manager package is named "GFX Library for Arduino", not "Arduino_GFX_Library" (searching the header name won't find it).
2. **lvgl** — **must be 9.1.0** (the SquareLine UI export and the sketch use the v9 API). v8 will not compile; v9.2+ may drift. Requires a matching v9 `lv_conf.h` (see below).
3. **TAMC_GT911** (touch driver) — included as `TAMC_GT911.h`
4. **Adafruit_LIS3DH** (accelerometer driver)
5. **Adafruit_Unified_Sensor** (required by LIS3DH)
6. **SD** (built-in — the ESP32 core copy is used; a "Multiple libraries for SD.h" warning is harmless)
7. **SPI** (built-in)
8. **Preferences** (built-in)
9. **WiFi** (built-in, for NTP)
10. **driver/twai.h** (ESP-IDF TWAI/CAN driver — bundled with ESP32 core)
11. **USB / USBMSC** (ESP32-S3 native USB, for Mass Storage mode — conflicts with CAN, see gotcha #8)

---

## LVGL v9 / Display Stack (read before touching the display or upgrading libraries)

The project runs **LVGL 9.1.0** with **GFX Library for Arduino 1.6.6** driving the onboard 800×480 RGB panel. Getting from a fresh library install to a working, tear-free display requires the following — every item here cost real debugging time, so don't skip them.

### lv_conf.h (the v9 config)
- **A known-good copy of this config is checked into the repo root: `lv_conf.h`.** It is a *reference copy only* — LVGL does not read it from the repo. To build, copy it to the Arduino **libraries root** (next to the `lvgl` folder, e.g. `<sketchbook>/libraries/lv_conf.h`). Keep the two in sync if you change either.
- Active location: in the Arduino **libraries root**, *next to* (not inside) the `lvgl` folder — e.g. `<sketchbook>/libraries/lv_conf.h`.
- **Must be the v9 template**, not a v8 one. Generate it by copying `lvgl/lv_conf_template.h` → `lv_conf.h` (or just use the repo's `lv_conf.h`). A leftover v8 `lv_conf.h` "mostly compiles" under v9 but misbehaves (a stray GarageLatch v8 config with `LV_COLOR_DEPTH 32` caused the first failures).
- Required settings:
  - Top guard `#if 0` → **`#if 1`** (the #1 thing people miss — without it the whole file is ignored).
  - `LV_COLOR_DEPTH 16`
  - Fonts on: `LV_FONT_MONTSERRAT_14`, `_20`, `_38`, `LV_FONT_UNSCII_8`, `LV_FONT_UNSCII_16` (the sketch references all of these).
  - **Put LVGL's heap pool in PSRAM** to free scarce internal RAM:
    ```c
    #define LV_MEM_SIZE (128 * 1024U)
    #define LV_MEM_ADR 0
    #define LV_MEM_POOL_INCLUDE "esp_heap_caps.h"
    #define LV_MEM_POOL_ALLOC(size) heap_caps_malloc(size, MALLOC_CAP_SPIRAM)
    ```
    Without this, v9's default builtin pool is a ~64 KB static block in internal RAM. Combined with the DMA draw buffers it leaves only ~11 KB free → the **WiFi task fails to create, SD queue overflows, and the touch task can't run** (looks like "touch broken").

### Draw-buffer sizing (the v9 black-screen trap)
- In v9, `lv_color_t` is a **3-byte RGB888 struct regardless of `LV_COLOR_DEPTH`** (it was 2 bytes in v8). So `LVGL_BUFFER_SIZE * sizeof(lv_color_t)` over-allocates by 50% and the internal-DMA-RAM alloc fails → firmware halts at `[5/8] LVGL init` with a black (backlit) screen.
- Size the buffer by the **render format**: RGB565 = **2 bytes/px**. The sketch uses `buf_bytes = LVGL_BUFFER_SIZE * 2`.
- `LVGL_BUFFER_SIZE = 800 * 30` (two ~48 KB buffers, double-buffered) in internal DMA RAM. Fits comfortably once the LVGL pool is moved to PSRAM (above).

### Arduino_GFX RGB panel constructor (v1.6.6 signature)
The v1.6.6 `Arduino_ESP32RGBPanel(...)` signature ends with:
`(..., pclk_active_neg, prefer_speed, useBigEndian, de_idle_high, pclk_idle_high, bounce_buffer_size_px)`.
An older API had an `auto_flush` arg here; the sketch's trailing args were written for that older API. Correct values for this board:
- **`useBigEndian = true`** — RGB565 byte order. If false/omitted, **all colors invert**.
- `de_idle_high = 0`, `pclk_idle_high = 0`.
- **`bounce_buffer_size_px = 800 * 10`** — this is what **eliminates the screen tearing**. With a single framebuffer (the lib hardcodes `num_fbs = 1`), the bounce buffer decouples the LCD scan from PSRAM reads. Setting it to 0 brings the tearing back.
- True hardware double-buffering / VSYNC isn't available in Arduino_GFX 1.6.6 (single FB). If the bounce buffer ever proves insufficient, the real fix is a double/triple-framebuffer + VSYNC backend (e.g. the installed `ESP32_Display_Panel` library or direct `esp_lcd`).

### The `.S` assembler errors after installing/upgrading LVGL
LVGL 9.x ships ARM assembly files (`lvgl/src/draw/sw/blend/neon/lv_blend_neon.S` and `.../helium/lv_blend_helium.S`) that `#include` a C header **before** their architecture guard. The Arduino builder feeds them to the xtensa assembler, which dies with `Error: unknown opcode or format name 'typedef'` on `stdint.h`. Fix (re-apply after any LVGL reinstall — these live in the library, not the repo):
wrap each file's `#include` (and the NEON body) in `#if defined(__ARM_NEON)` / `#if defined(__ARM_FEATURE_MVE)` so non-ARM targets preprocess them to nothing.

### GT911 touch startup quirk
On most boots the GT911 isn't ready when first probed: log shows `GT911 no ACK` ×5 → `Controller stuck - attempting recovery` → `Reset attempt 1/3` → `GT911 verified`. It self-heals (touch works) but adds ~3 s and log noise. Low priority; a longer pre-init delay would smooth it.

### Internal-RAM budget (current)
After the PSRAM pool move + 2×48 KB draw buffers + 800×10 bounce buffer: ~95 KB internal free at LVGL init, ~46 KB free at runtime. Healthy, but internal RAM is the tightest resource — watch it if adding features.

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
- **Both Pico 5593PT relays**: pin 85 (**white** lead) permanently grounded at the diff GND bolt, coil triggered via pin 86 (**black** lead), pin 87a (NC) capped and heat-shrunk. (Full Pico pigtail color map: see *Pico 5593PT Relay Reference* below.)
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

### Pico 5593PT Relay Reference (all fan/pump relays + the cabin-accessory relay)

**Pigtail lead colors — `black = pin 86` (coil **+**, trigger side), `white = pin 85` (coil **−**, ground side).** Confirmed on the engine + ATF-radiator fan relays, where black runs to the ignition-switched +12 V and white runs to the thermal switch on the coil-ground (low-side) leg. The diff-cooler relays use the **same** color↔pin map but switch high-side (white/85 is a permanent ground, black/86 is fed through the Setrab) — the mapping is a property of the pigtail, not the circuit.

- Matches ISO convention (86 = coil +, 85 = coil −). The **"85"/"86" numbers are molded into the relay body** next to each lead — trust those over color if ever unsure. Colors for 30/87/87a weren't logged; read the molded numbers.
- On a plain relay (no coil diode) 85/86 are electrically interchangeable; the color→number mapping only matters for documentation and for any diode/LED coil (where 85 must be − and 86 the + trigger — black=86/white=85 satisfies it).
- **Plain ignition-switched circuit, no thermal switch (e.g. the cabin-accessory relay):** black (86) → ignition/ACC +12 V, white (85) → ground directly, pin 30 → constant fused 12 V, pin 87 → load. Same coil hookup as the fans, minus the thermal switch in the white leg.

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
9. **Oil pressure is now a P51 4-20mA current loop (v6.2), not the old PX3 voltage sensor.** 2-wire, loop-powered off the 24V rail (8-30V); CH1 is Waveshare Mode 3 like the temp channels. No buck converter, no 5V rail, no divider — which removes the regulated-5V overvoltage failure mode that killed the PX3. Function is set by connector PIN: Pin 1 = Vin, Pin 2 = NC (unused), Pin 3 = loop return -> AI1+.
10. **Oil-pressure pigtail mapping is BENCH-CONFIRMED (2026-06-14): red=+24V (Vin), black=AI1+ (loop return), yellow=capped (unused).** It's a Honeywell-spec pigtail ([Amazon B09G65X1YC](https://www.amazon.com/dp/B09G65X1YC)) on an SSI P51; a 2-wire 4-20mA loop uses only supply (red) and ground/return (black), so the old 3-wire signal lead (yellow) is dead. The **ohm test can't ID the pins** on a 4-20mA sensor (unpowered = high-Z, all read open) — sweep powered, and do it at **12V** so reverse combos don't exceed the P51's +/-16V reverse limit. Never wire it reversed. (Earlier datasheet guesses red=Vin/yellow=AI+ and then black=Vin were both wrong.)
11. **No buck converter in the oil-pressure circuit anymore.** The P51 is loop-powered off 24V directly. The JTAREA-branded LM2596 ([Amazon B0D2TS7CBN](https://www.amazon.com/dp/B0D2TS7CBN)) that cooked the PX3 (broken feedback, passed ~8.5 V) is removed for good. If any *future* circuit needs regulated 5V, use a fixed-output industrial part (Recom R-78E5.0-1.0 / Murata OKI-78SR-5 / Pololu D24V5F5), never an adjustable Amazon-resold LM2596.
12. **P51 supply range is 8-30V; the 24V rail is well within it.** Unlike the PX3 (5.25 V absolute max, easily exceeded), the P51 tolerates the raw rail, so there is no precise-supply check to get wrong. Add a **1.5KE36A TVS across the 24V rail** (band / cathode -> +24V) for load-dump / transient protection; one TVS on the rail covers the P51 and all four PRTXI loops.
13. **P51 reverse-polarity tolerance is +/-16V for ~5 min (datasheet) — not unlimited.** A brief reversed bench hookup at <=16V won't kill it, but sustained reverse at 24V can. The rail TVS + fuse protect against a reversed *rail* (TVS forward-conducts, blows the fuse) but **not** against swapping the sensor's own two leads. Always confirm Red=Vin, Yellow=return before power.
14. **The PDF `PX3AN2BH150PSAAX_ESP32_3V3_Interface_Breakdown.pdf` is obsolete** — it described the PX3 -> divider -> ESP32 ADC design. The active design is the P51 4-20mA loop into the Waveshare in Mode 3. Refer to the P51 wiring diagram in this CLAUDE.md, not that PDF.
15. **`CAN_MUX_TO_CAN` flag (in `initIOExtension`) — now `1` (CAN ON) as of the 2026-06-14 OBD harness work.** It was temporarily `0` during the v6.0 LVGL/display bring-up (to keep native USB alive). It is now `1`, so EXIO5 is driven HIGH and the FSUSB42UMX mux connects GPIO19/20 to the onboard TJA1051T — OBD CAN is live. **Side effect: native USB / USB-MSC is disabled while CAN is active; flash and serial over the separate UART USB-C port.** Set back to `0` only if you specifically need native USB/USB-MSC and can live without CAN. If OBD still logs `CAN active, no ECU response` with the flag at `1`, it's wiring/car-side (see `obd_can_wiring.md`), not this flag.
16. **LVGL v9 has its own set of traps** — wrong library version, v8 `lv_conf.h`, the `sizeof(lv_color_t)`=3 buffer trap (black screen), internal-RAM exhaustion (breaks touch/WiFi/SD), the `.S` assembler errors, and the `useBigEndian`/bounce-buffer constructor args (inverted colors / tearing). All are documented in the **"LVGL v9 / Display Stack"** section above. If the display, touch, or memory misbehaves after a library change, start there.
17. **Modbus dropping out when the car key cycles is NOT a fault.** The electric box is powered from an ignition-switched wire, so it loses power with the car. Repeated `[MODBUS] Read failed` / `Communication LOST` after a key-off is expected; it recovers on the next power-up.
18. **Every gauge in `updateUI()` needs BOTH a valid-update path and a "became-invalid → show `---`" path.** The `if (xxx_valid) { ... }` block only repaints while data is valid; without a matching transition-to-invalid block (using a `static bool xxx_was_data_valid`), the label latches its last value forever when the sensor drops. This bit OBD-fed **Water Temp** and **Fuel Trust** (they had no invalid path and stayed frozen on key-off / OBD unplug) — fixed in v6.1 by copying the Modbus gauges' pattern. When adding any new gauge, add both paths.
19. **Waveshare has a PER-CHANNEL HARDWARE JUMPER (voltage vs current) — separate from the software Mode 3 register. Both must agree.** Jumper **ON = current mode** (4-20mA sense resistor in circuit) / **OFF = voltage mode** (high-Z 0-10V). A 4-20mA sensor needs the jumper **ON** *and* software Mode 3. If the jumper is **OFF** on a 4-20mA loop, the sensor's current source rails the high-Z input → module reads **full scale (20000 µA / 150 PSI), dead-steady**. **This — not a dead channel or sensor — is what produced the long "CH1 is dead / first P51 is toast" misdiagnosis (CH1's jumper was left OFF from the PX3 era).** When a 4-20mA channel reads pegged-150 or flat-0, check the jumper FIRST. Measure loop current with a voltmeter: **V(AI+ to GND) ÷ 500 Ω = mA** (the module's "2-10V" range is 4-20mA through a 500 Ω sense resistor; 2.0 V = 4 mA = 0 PSI).
20. **Oil-pressure dropout = a GROUNDING/ISOLATION fault, not a bad sensor or module (RESOLVED v6.6).** Both P51 sensors are bench-good; the Waveshare module is good (a sibling 4-20mA channel ran clean on it the whole time). The P51's 4-20mA loop shared the electric box's power-and-ground, giving its 4 mA a sneak path around the module's 500 Ω sense resistor → reads ≈0/`---`. It works on an isolated/floating supply and dies on box power. **Fix: power the oil-pressure loop from an isolated 24V DC-DC (Mean Well DDR-15G-24 / Traco TRN 3-1215), input off car 12V, output− to AI1− only.** Do NOT RMA P51 sensors or replace the module. **✅ Installed + validated on-car 2026-07-04: oil pressure 99.7% valid, 37–112 PSI, stable engine on/off.** Full story: `oil_pressure_isolation_saga.md`.

---

## Version History

| Version | Key Changes |
|---------|-------------|
| v6.13 | **Demo-mode phantom-touch guard + WiFi diagnostics (2026-07-30; host-verified logic only, NOT flash-tested; not committed).** On-car, a stray capacitive 5-s touch dropped the display into DEMO mid-drive (session 597, ~5 min in: `[MODE] Demo mode ENABLED (5-second hold)`); the simulated temps climbed to "critical" and the user thought it was real — the old "DEMO" marker lives inside the hidden utility box, so nothing warned them. Fix: (1) `checkUtilityLongPress()` now blocks LIVE→DEMO while `g_vehicle_data.rpm_valid && rpm>300` (engine running), logging `[MODE] Demo toggle IGNORED`; DEMO→LIVE still allowed. (2) New `updateDemoBanner()` shows a full-width always-on-top magenta "DEMO - SIMULATED DATA" banner (top layer) whenever `g_demo_mode`, called from `updateModeIndicator()`. Plus **WiFi diagnostics**: `logWifiScanDiag()` on connect-timeout logs `WiFi.status()` + a 2.4 GHz scan (is the SSID visible? RSSI/ch/enc) to separate band-steering/range from a bad password. Context: user's `NETGEAR68` is one SSID for both bands; ESP32 is 2.4-only. Verified: braces/parens/#if balanced, CSV still 66 cols. |
| v6.12 | **Local WiFi file page (2026-07-30; host-verified logic only, NOT flash-tested; not committed).** Redundancy for v6.11's cloud upload + a no-external-service fallback. The box serves its SD logs over WiFi as a plain web page: `serveFilesWindow()` (in `timeSyncTask`, Core 0, after the cloud upload and before WiFi power-off) starts `WebServer(80)` + mDNS (`z370.local`) and loops `handleClient()` for `FILE_SERVER_MINUTES` (default 15, set in `/wifi.cfg`, 0=off), then stops and lets WiFi power down so its RAM is reclaimed. `fsHandleRoot()` chunk-streams a `SESS_*` listing (name/size/download link); `fsHandleDownload()` validates the name (`fsValidName()` — `SESS_*.csv/.log` only, blocks path traversal) and `streamFile()`s it off the card. SD reads bracketed with `g_fb_pause_sd_writes` (same as the file browser) so they can't collide with the writer. New includes `WebServer.h`/`ESPmDNS.h`; flag `ENABLE_FILE_SERVER`; `[FILESRV]` serial tag. **Design note: bounded window (not always-on) on purpose — holding WiFi all drive would keep ~tens of KB of internal RAM (the board's tightest resource). First-flash risk = RAM while WiFi + full UI coexist; if it glitches, lower `FILE_SERVER_MINUTES`.** |
| v6.11 | **Cloud auto-upload of session logs (2026-07-30; host-verified logic only, NOT flash-tested; not committed).** Kills the pull-the-SD/USB chore. At boot, inside `timeSyncTask` (Core 0) while WiFi is already up for NTP, `uploadPendingSessions()` scans the card and streams any completed `SESS_*.csv/.log` not in `/CLOUDUP.DAT` to a configurable HTTPS endpoint (`cloudUploadFile()` — file streamed as the POST body, metadata `secret`/`folder`/`name` in the query string), records them in the manifest, then WiFi powers off as before. Endpoint-agnostic; bundled **`cloud/CloudUpload.gs`** Apps Script receives them into **Google Drive** (folder `CLOUD_FOLDER`) so they sync to PC/phone. Config in the **same `/wifi.cfg`**: `CLOUD_URL`/`CLOUD_SECRET`/`CLOUD_FOLDER` (template updated). Current (open) session uploads next boot; capped at `CLOUD_MAX_PER_BOOT=8`/boot; idempotent (dedupe by name both ends). `TimeSyncTask` stack **4096→16384** for the TLS handshake; `WiFiClientSecure.setInsecure()`; follows the Apps Script 302 (`HTTPC_STRICT_FOLLOW_REDIRECTS`, accepts 200/301/302). New `[CLOUD]` serial tag. **On first flash verify:** the 302 redirect follows, and TLS fits in RAM/stack. Feature flag `ENABLE_CLOUD_UPLOAD`. |
| v6.10 | **Fuel-trust transparency + OBD trouble-code reading (2026-07-30; host-verified logic only, NOT flash-tested; not committed).** Motivated by a real low-fuel-trust day: LTFT pegged at −13.3% (−20 pts) + high idle STFT (−10) floored the score at 70%, and the pre-v6.9 logs only stored the final % — not the inputs. (1) **CSV now logs the 4 inputs** — `stft_b1/b2, ltft_b1/b2, fuel_trim_valid, ft_timing_deg` + the 4 deductions `pen_stft/pen_ltft/pen_bank/pen_timing` + `mil_on, dtc_count` (12 appended cols → 66 total; positions unchanged). `[OBD]` serial line prints **Bank 2** (ST2/LT2) again. `computeFuelTrust()` publishes the breakdown to global `g_ft`. (2) **FUEL TRUST tap popup** — `ui_FUEL_TRUST_Value_Tap_Panel` → `showFuelTrustPopup()`: both banks' trims, per-penalty points, plain-English meaning, and active DTCs; tap to dismiss. (3) **DTC reading** — Mode 01 PID **0x01** (MIL + count) + a **Mode 03** ISO-TP reader (`sendOBD_Mode03`/`handleDTCFrame`/`finalizeDTCs`/`dtcToString`) that decodes stored codes to `Pxxxx` strings, prints `[DTC]`, and shows them in the popup. Assumes the ISO 15765-4 CAN count-byte format — **verify against a scanner on first flash.** (4) **Firmware version** now stamped into the `.log` header and a `# 370zMonitor FW` CSV comment line (was in neither); new `FW_VERSION` macro is the single source of truth for the banner + logs. Host-tested: `dtcToString` byte-decode + penalty exposure. **TODO: flash + verify DTC decode on-car.** |
| v6.9 | **Automatic DST + expanded OBD logging (2026-07-21; g++/host-verified, NOT flash-tested; not committed).** (1) **Automatic daylight saving** — DS3231 now stores **UTC**; local time (incl. the spring/fall hour) derives from `POSIX_TZ = "CST6CDT,M3.2.0,M11.1.0"` via new `readRTCLocal()`/`utcTmToEpoch()`/`ensureTimezone()`. NTP writes UTC (`gmtime_r`), `configTzTime()` replaces `configTime()`. **No more seasonal reflash.** After flashing, do ONE WiFi/NTP boot to rewrite the RTC in UTC (until then it reads ~5–6 h off). Host-tested: epoch math vs `timegm`, both 2026 transitions (Mar 8 / Nov 1). (2) **Expanded OBD-II** — added 11 PIDs (throttle 0x11, load 0x04, IAT 0x0F, ambient 0x46, oil temp 0x5C, MAF 0x10, lambda 0x44, module voltage 0x42, fuel level 0x2F, baro 0x33, fuel-sys 0x03); poll 200→120ms, stale 3→5s, RPM/throttle oversampled; +22 CSV columns (appended, so old column positions unchanged → 54 total) + `[OBD2]` serial line. Logging only, no gauges. Sunrise/sunset calc now DST-aware. |
| v6.8 | **NTP time-sync fix (superseded by v6.9's DST work but the sync fix carries forward).** `tryNTPSync()` was accepting the stale RTC-seeded system clock as a "successful" sync (getLocalTime returns true once year>2016, and the clock was already seeded from the wrong RTC), then writing that stale time back to the RTC — so NTP re-cemented the error instead of fixing it (root cause of the days-behind/non-monotonic clock). Fix: seed an old sentinel, only accept a fresh SNTP result (year≥2025), restore the clock on failure. Also flagged the DS3231 coin cell. A coolant Value-Critical bump 220→235°F was made then reverted (stays 220). |
| v6.7 | **Track-day "smart oil-pressure monitor" pass (compiled clean 54% flash / 24% RAM; on-car validation pending).** (1) **10 Hz logging** — `SD_WRITE_INTERVAL_MS` 1000→100, `SD_QUEUE_SIZE` 16→40, to catch sub-second oil-pressure/starvation dips (10 Hz is the RS485/Modbus ceiling, not the P51's — the sensor is analog/near-instant). (2) **RPM-aware oil-pressure alarm** — `isOilPressureCritical()` now trips below `max(10 psi/1000 rpm, 10 psi idle floor)` while the engine runs (warm-oil, always armed, no temp gating) plus the `>120` PSI overpressure trip; drives the existing **Value Critical** label (no buzzer, by request). The old fixed `<10` PSI trip is replaced; `isOilPressureCriticalRPM()` is now unused. (3) **Accelerometer calibration** — `readAccelerometer()` rotates raw axes into the car frame (`ACCEL_R_*`, `ENABLE_ACCEL_CALIBRATION`), derived from 55 mi of logs; vertical exact, lat/lon provisional ~±20°. (4) **Timestamp fixes** — `syncSystemTimeFromRTC()`/`settimeofday()` at boot fixes 1979 FAT file dates without WiFi; `DAYLIGHT_OFFSET_SEC` 0→3600 fixes the 1-hour-behind (needs one WiFi boot to rewrite the RTC). (5) **`[SESSION]` summary** — per-second log line (min oil PSI >2k rpm, peak oil temp, max lat/lon/vert G). Oil-temp PRTXI (AI2) still offline/being replaced; the car's factory oil-temp gauge covers it in the meantime. |
| v6.6 | **ROLLBACK — restored the original 5-channel mapping; real root cause found.** The oil-pressure dropout chased through v6.3–v6.5 was never a bad channel — it was a **grounding/isolation fault** (the P51 loop shared the box's power-and-ground, so its 4 mA bypassed the Waveshare's 500 Ω sense resistor and read ≈0/`---`). Proven on the bench: both P51 units read steady ~4 mA on an isolated supply and dropped out only on box power; a sibling 4-20mA channel (diff temp) was clean throughout. **Fix (hardware):** power the oil-pressure loop from an isolated 24V DC-DC (Mean Well DDR-15G-24 / Traco TRN 3-1215), input off car 12V, output− to AI1− only (24V out for full-scale headroom). **Firmware:** reverted all diagnostic swaps — `MODBUS_CH_OIL_PRESSURE` 2→0, `OIL_TEMP` 5→1, `TRANS_TEMP` 1→2; `MODBUS_NUM_CHANNELS` 6→5; boot configures CH1–CH5 Mode 3 (dead-CH1 skip + CH6 spare removed); labels + version (header/splash/banner) → v6.6. **Validated on-car 2026-07-04** (isolated DDR-15G-24 installed; oil pressure 99.7% valid, 37–112 PSI, stable engine on/off; oil-temp PRTXI on AI2 failed, replacement pending). Full narrative: `oil_pressure_isolation_saga.md`. |
| v6.5 | **DIAGNOSTIC A/B swap — oil pressure CH2 ↔ trans temp CH3.** On v6.4 the P51 read flat 0 µA on CH2 while CH3-CH5 temps read perfectly, so oil pressure is moved onto **CH3 (AI3)** — the proven-good trans-temp channel — and trans temp is moved to **CH2 (AI2)**. Physically swap the two loops between AI2/AI3 (jumpers stay ON). If the P51 reads steady 4 mA on CH3, the sensor's good and AI2/its wiring was the bad spot; if it still zeros/flickers on CH3 while trans temp is solid on CH2, the fault is the P51/harness, not the board. Firmware: `MODBUS_CH_OIL_PRESSURE` 1→2, `MODBUS_CH_TRANS_TEMP` 2→1; labels + version (header/splash/banner) → v6.5. **Also flagged:** the old "CH1 dead" reading was taken with CH1's jumper OFF (voltage mode) — likely a jumper mistake, not a dead channel; CH1 may be fine. |
| v6.4 | **Oil pressure moved off INTERMITTENT CH6 -> CH2 (AI2).** CH6's current-sense path is flaky (reads clean 4.00 mA then drops to 0, repeatedly); a channel-swap test proved the P51 sensor + harness + wiring are good, so the fault is the CH6 jumper/terminal/front-end on the module. With CH1 already dead, this module has two bad channels (likely damaged — consider replacing). Firmware: `MODBUS_CH_OIL_PRESSURE` 5→1 (CH2/AI2, a proven-good channel); unused oil-temp slot parked on CH6 (`MODBUS_CH_OIL_TEMP` 1→5); read range unchanged (regs 0..5); boot still configures CH2-CH6 Mode 3, skips dead CH1; serial labels CH6→CH2; version→v6.4 (header + splash + banner). Wiring: move P51 loop to **AI2** (Pin1→24V, Pin3→AI2+, AI2-→GND) and set the **AI2 jumper to current mode**. |
| v6.3 | **Oil pressure moved off DEAD Waveshare CH1 -> CH6 (AI6).** CH1's 4-20mA current input is stuck at full-scale (20000 µA / 150 PSI), dead-steady at atmosphere — confirmed with TWO different P51 sensors on the same module/firmware while CH2-CH5 read correctly (likely AI1 sense-circuit damage from the reverse-polarity event that killed the first P51). Firmware: `MODBUS_CH_OIL_PRESSURE` 0→5, `MODBUS_NUM_CHANNELS` 5→6 (reads regs `0..5`, index 0/CH1 ignored), added `WAVESHARE_CH6_MODE_REG 0x1005`; boot now configures **CH2-CH6** for Mode 3 and **skips the dead CH1**. Serial labels `[OILP]`/CONNECTED now say CH6. Wiring: move P51 loop return AI1+ → **AI6+** (AI6- → GND); leave AI1 unwired. CH1 is electrically dead — not a firmware/sensor issue. |
| v6.2 | **Oil pressure sensor: PX3 voltage -> P51 4-20mA current loop.** Replaced PX3AN2BH150PSAAX (0.5-4.5V, needed a regulated 5V buck) with Amphenol SSI P51-150-G-B-P-20MA (4-20mA loop, loop-powered off the 24V rail, 8-30V) — eliminates the buck-converter overvoltage failure mode that killed the PX3. Firmware: Waveshare CH1 -> Mode 3 (4-20mA) at boot alongside CH2-CH5; `convertToPSI()` rewritten current-based (`PSI = ((uA-4000)/16000)*150`); CH1 disconnect via `PRTXI_MIN_VALID_UA`; var renamed oil_press_mV -> oil_press_uA. Wiring: 24V+ -> Red(Pin1); Yellow(Pin3) -> AI1+; AI1- -> GND; Black(Pin2) unused; 1.5KE36A TVS across the 24V rail. |
| v6.1 | **OBD CAN operational on the car + stale-value UI fix.** Set `CAN_MUX_TO_CAN 1` (EXIO5 HIGH → FSUSB42UMX mux to the onboard TJA1051T); OBD CAN confirmed reading the 2018 370Z ECU (water temp / ECT) over a 2-wire CANH/CANL tap (OBD pins 6/14; ground shared via chassis) — see `obd_can_wiring.md` + `obd_can_harness_diagram.svg`. UI: Water Temp and Fuel Trust were latching their last value when OBD data went stale; added the per-gauge "became-invalid → show `---`" block the Modbus gauges already had, so both clear ~3 s (`OBD_PID_STALE_THRESHOLD_MS`) after the ECU stops responding. Fuel Trust still only shows once ECT ≥ 80 °C. |
| v6.0 | **OBD CAN fix + LVGL v9 migration / display stabilization.** CAN: drive CH422G EXIO5 (`EXIO_CAN_SEL`) HIGH to route the FSUSB42UMX mux to the TJA1051T (GPIO19/20 shared with native USB); un-swapped CAN pins (now TX=GPIO20, RX=GPIO19) — see `obd_can_wiring.md`. (CAN was gated off via `CAN_MUX_TO_CAN 0` during this release while the display was stabilized; turned **on** in v6.1.) Display: migrated to LVGL 9.1.0 — fixed `lv_conf.h` (v9 template, PSRAM heap pool, fonts, color depth), fixed the `sizeof(lv_color_t)` draw-buffer trap, patched LVGL's ARM `.S` files for xtensa, restored `useBigEndian` + added bounce buffer (colors + tearing). Full detail in "LVGL v9 / Display Stack". Build: `USB CDC On Boot` Disabled; flash/monitor over the UART port. |
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
