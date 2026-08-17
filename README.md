<p align="center">
  <img src="docs/images/hero-autobahn.jpg" alt="Red 2018 Nissan 370Z at speed on track at Autobahn Country Club, SCCA Track Night in America, August 2026" width="100%">
</p>

# 370zMonitor

A comprehensive real-time automotive data acquisition and monitoring system purpose-built for track day use on a 2018 Nissan 370Z (works on Nissan/Infiniti vehicles). Powered by a Waveshare 7" ESP32-S3 — an all-in-one development board with an integrated 800×480 capacitive touch display — the system provides live sensor telemetry, interactive gauges, automated data logging, and critical alerts, all in a compact, self-contained package.

Designed around the demands of regular track use, 370zMonitor gives you real-time visibility into fluid temperatures, oil pressure, and G-forces, helping you monitor vehicle health and catch dangerous conditions before they become expensive problems.

---

## Demo Mode

[![370zMonitor — Demo Mode](https://img.youtube.com/vi/LVQcww5BEp8/hqdefault.jpg)](https://www.youtube.com/watch?v=LVQcww5BEp8)

▶ **[Watch the demo on YouTube](https://www.youtube.com/watch?v=LVQcww5BEp8)**

---

## Gallery

### ESP32 unit

<p align="center">
  <img src="docs/images/000.jpg" alt="Waveshare ESP32-S3-Touch-LCD-7 specifications: LX7 dual-core processor, 2.4 GHz Wi-Fi, BLE 5, onboard antenna, 7 inch 800x480 5-point touch display, 65K RGB" width="49%">
  <img src="docs/images/001.jpg" alt="Waveshare ESP32-S3-Touch-LCD-7 board interfaces: UART port, USB port, sensor, CAN, I2C and RS485 headers" width="49%">
</p>

### ESP32: mounted into the interior of the car

<p align="center">
  <img src="docs/images/01.jpg" alt="370zMonitor display suction-mounted to the windshield, showing the live gauge screen with oil pressure, oil temp, water temp, trans temp, steering temp, diff temp and fuel trust" width="80%">
</p>

### Electric box: ignition relay, 12V to 24V converter, Modbus RTU

<p align="center">
  <img src="docs/images/02.jpg" alt="Electric box open, viewed from above: Victron Orion-Tr 12/24-5 isolated DC-DC converter, Waveshare Modbus RTU module, ignition relay and labelled wiring" width="49%">
  <img src="docs/images/03.jpg" alt="Electric box open on the bench, three-quarter view showing the internal loom and labelled connector pigtails" width="49%">
</p>

<p align="center">
  <img src="docs/images/04.jpg" alt="Sealed electric box closed, clear lid showing the wiring inside and labelled Deutsch-style connector pigtails exiting the gland" width="34%">
</p>

### Electric box: under car's bumper mount

<p align="center">
  <img src="docs/images/05.jpg" alt="370Z front end with the bumper removed, electric box mounted to the crash bar alongside the oil and transmission coolers" width="49%">
  <img src="docs/images/06.jpg" alt="Front view of the 370Z with bumper removed showing the fuse block, harness routing and cooler ducting on the crash bar" width="49%">
</p>

---

## Features

- **LVGL Touch Interface** — Custom UI with real-time gauges, live charts, and a splash screen with version display on the integrated 800×480 touchscreen
- **Dual-Core Architecture** — UI rendering runs on ESP32-S3 Core 1 while sensor polling and data acquisition run on Core 0 for responsive, non-blocking performance
- **Modbus RTU Sensor Monitoring** — Waveshare 8-Ch Modbus AI (B) module reading five sensor channels:
  - CH1: Oil pressure (PX3 sensor, 0–10V)
  - CH2: Oil temperature (PRTXI, 4–20mA)
  - CH3: Transmission temperature (PRTXI, 4–20mA)
  - CH4: Steering fluid temperature (PRTXI, 4–20mA)
  - CH5: Differential temperature (PRTXI, 4–20mA)
  - PRTXI sensors include a +5°C calibration offset and 0.3°C hysteresis applied in software
- **CAN Bus OBD-II Integration** — Reads vehicle data directly from the vehicle's OBD-II port via CAN bus
- **Accelerometer** — LIS3DH for G-force logging during cornering, braking, and acceleration
- **RTC Timekeeping** — DS3231 real-time clock for accurate session timestamps and time sync
- **SD Card Data Logging** — Automated session-based logging to SD card with structured data files. All important data points get logged
- **Toast Notification System** — On-screen alerts for sensor status, SD card events, log activity, RTC status, and time sync
- **Critical Value Alerts** — Configurable warnings for high temperature and pressure conditions to protect the drivetrain during sustained track driving

---

## Hardware

- Waveshare 7" ESP32-S3 all-in-one board (800×480 capacitive touch display with integrated ESP32-S3)
- Waveshare 8-Ch Modbus AI (B) analog input module for the sensors
- PX3 oil pressure sensor (0–10V output)
- 4× PRTXI-1/2N-1/4-4-IO temperature sensors (4–20mA output)
- LIS3DH 3-axis accelerometer
- DS3231 precision RTC module
- LM2596 buck converters for 12V automotive power regulation

---

## Data Sources

### Modbus RTU Sensors (via Waveshare 8-Ch AI)

| Channel | Measurement | Sensor | Signal |
|---|---|---|---|
| CH1 | Oil Pressure | PX3 | 0–10V |
| CH2 | Oil Temperature | PRTXI | 4–20mA |
| CH3 | Transmission Temperature | PRTXI | 4–20mA |
| CH4 | Steering Fluid Temperature | PRTXI | 4–20mA |
| CH5 | Differential Temperature | PRTXI | 4–20mA |

### OBD-II CAN Bus (from ECU)

- Coolant Temperature — PID 0x05
- Fuel Trust — Fuel system status

### I2C Sensors

- G-Force (X/Y/Z) — LIS3DH 3-axis accelerometer

---

## Use Case

Built for a track-driven 370Z participating in approximately 20 events per year at midwest circuits including Road America, Autobahn Country Club, Blackhawk Farms, and Gingerman Raceway. The system monitors fluid temperatures and oil pressure in real time to catch dangerous conditions before they cause damage during sustained high-speed driving.

---

## Wiring Diagram

![370zMonitor Wiring Diagram](https://github.com/user-attachments/assets/7bb15eca-8a60-40c7-aba1-2f2afad45fcd)
