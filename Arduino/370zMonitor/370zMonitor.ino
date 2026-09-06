//-----------------------------------------------------------------

/*
 * 370zMonitor v6.15
 * Supports Demo Mode (animated values) and Live Mode (sensors data/OBD data)
 * ESP32-S3 with PSRAM, LVGL, GT911 Touch
 *
 * v6.15 Changes (HEAT DERATE MONITOR — "is the ECU pulling power because the car is hot?"):
 *   Background: the 7/24 logs showed PID 0x0E (timing advance) refreshing only every 3-5 s
 *   (26-PID round-robin x 120 ms), i.e. 2-3 samples per WOT pull, and the Fuel Trust
 *   "timing pull" term comparing samples 5 s / thousands of rpm apart (it fired in 24.5% of
 *   rows on a street drive). See heat_derate_research.md in the repo root for the research.
 * - POWER WINDOWS. While throttle >= 70% and rpm >= 3000 (or pedal pinned) the OBD scheduler
 *   switches to a short burst list (OBD_PID_LIST_POWER, 60 ms) so timing refreshes ~8 Hz and
 *   rpm ~3 Hz; the full 26-PID list resumes 1.5 s after the throttle lifts. Slow PIDs get a
 *   longer staleness timeout during a window so temps don't blink "---" on a long straight.
 * - LEARNED COOL BASELINE. Timing/load/pedal-gap at WOT are averaged per 500-rpm bin
 *   (2000-7999) while the engine is warm-but-cool (ECT 160-205F, IAT < 120F, oil < 230F when
 *   known), persisted in NVS namespace "timbase". Every later WOT sample is compared to its
 *   bin: tim_delta = live - cool. rpm is extrapolated to the timing sample time so the bin is
 *   right even in low gears. Reset: long-press RESET BASELINE in the popup, or
 *   PWR_BASELINE_RESET=1 in /wifi.cfg (one boot).
 * - DERATE DETECTORS -> one POWER state (worst wins): REV CAP (rpm plateau < 7000 at WOT while
 *   oil is hot, or a jagged limiter-style plateau), FUEL? (rpm stumble + recovery at WOT in a
 *   corner), TIMING (delta <= -3 amber / -6 red, held 1 s), THROTTLE (pedal-vs-plate gap grows
 *   vs cool), LIFT (calculated load below the cool baseline = VVEL lift cut), AIR (SAE J1349
 *   density loss from IAT + baro; banner only at >= 6%). Each is time-qualified and latched.
 * - UI. A top banner (same top-layer pattern as the DEMO banner; amber/red; hidden when OK)
 *   shows the state + reason, e.g. "PWR  REV CAP 6050 - OIL 281F". Tapping it acknowledges
 *   (hides until a worse state) and opens the popup. The FUEL TRUST popup gained a POWER
 *   block (delta, bin, gap, load, rev cap, air, attribution, TCM ATF) + a long-press
 *   RESET BASELINE button.
 * - PEDAL POSITION. PID 0x49 is polled; if the ECU doesn't answer it, the pedal comes from the
 *   passively-heard CAN frame 0x180 byte F (= F/255*100, per the Z34 reverse-engineering).
 *   pedal_src in the CSV says which (0 none, 1 PID, 2 CAN).
 * - TCM ATF TEMPS + TCC SLIP via Mode 21 LID 0x20 to the TCM (7E1 -> 7E9, ISO-TP). Byte
 *   offsets are PROVISIONAL (TCM_ATF1_IDX etc.) — the raw record is dumped on a [TCM] line so
 *   the offsets can be confirmed against the PRTXI trans temp on the first drive.
 * - DISCOVERY (one-time, logged): supported-PID bitmaps 0x00/0x20/0x40/0x60/0x80 at boot
 *   ([OBD-DISC]); a passive CAN ID census 30 s after boot ([CAN-DISC]); optional Nissan
 *   Mode 22 DID sweep 0x1100-0x12FF at idle when OBD_DID_SWEEP=1 in /wifi.cfg ([DID]).
 * - FUEL TRUST timing term now uses the baseline delta (FUEL_TRUST_TIMING_MODE 1; 0 = legacy
 *   consecutive-sample counter). Same max weight (30 pts). This changes the score on purpose.
 * - CSV: 14 columns appended (66 -> 80): tim_base_deg, tim_delta_deg, tim_bin_n, pedal_pct,
 *   pedal_src, thr_gap_pct, load_delta_pct, rev_cap_rpm, air_loss_pct, pwr_state, pwr_sev,
 *   atf1_f, atf2_f, tcc_slip_rpm. New [PWR] serial line every 2 s; [SESSION] line extended.
 * - Bus etiquette unchanged in spirit: <= 17 requests/s even in a power window; TWAI RX queue
 *   32 -> 64. HOST-VERIFIED only, NOT flash-tested. Validation steps are in CLAUDE.md.
 *
 * v6.14 Changes (OBD oil temp via Mode 22 + watchdog/power-loss hardening):
 * - OBD OIL TEMP (Nissan Mode 22). The generic PID 0x5C is unsupported on the VQ37, but the 370Z
 *   exposes oil temp via Nissan enhanced DID 0x111F over Mode 0x22 (physical addr 7E0, °C = A-50).
 *   Added sendOBD_Mode22()/decodeOBD_Mode22Reply(), polled ~1 Hz. When the (dead) Modbus PRTXI
 *   oil-temp sensor is offline, mergeOilTempSource() drives the oil-temp DIAL from this OBD value —
 *   so oil temp is back on screen in software. Validate the -50 offset vs the factory gauge on first
 *   drive. If the PRTXI sensor is replaced later, its reading wins automatically.
 * - TASK WATCHDOG. esp_task_wdt armed on the main loop (12 s, version-guarded for IDF 4.x/5.x); a
 *   hang now auto-resets the board instead of a frozen screen. Fed each loop iteration. No hardware.
 * - POWER-LOSS FLUSH (optional, OFF by default). ENABLE_POWER_FAIL_DETECT + a 12V-sense divider on
 *   POWER_SENSE_PIN: on a power good->failing transition it emergency-flushes the SD so a mid-write
 *   key-off can't corrupt the card. Dormant until wired + enabled. NOT flash-tested.
 *
 * v6.13 Changes (demo-mode phantom-touch guard + WiFi diagnostics):
 * - DEMO-MODE SAFETY. A stray capacitive touch once held the utility box for 5 s mid-drive and
 *   dropped the display into DEMO, whose simulated temps climbed to "critical" and looked like a
 *   real emergency (the old DEMO marker lived inside the hidden utility box, so nothing showed).
 *   Now: (1) LIVE->DEMO is blocked whenever the engine is running (rpm_valid && rpm>300) — blocked
 *   attempts log "[MODE] Demo toggle IGNORED"; DEMO->LIVE is always allowed. (2) A full-width
 *   always-on-top "DEMO - SIMULATED DATA" banner shows whenever demo is active, so it can never be
 *   mistaken for live readings again.
 * - WIFI DIAGNOSTICS. On a connection timeout the box now logs WiFi.status() and scans 2.4 GHz,
 *   reporting whether the configured SSID is even visible (RSSI/channel/enc) — distinguishing a
 *   5 GHz-only / band-steering / out-of-range problem from a wrong password. Helps pin down why
 *   NTP/cloud/file-page WiFi won't connect.
 *
 * v6.12 Changes (local WiFi file page — download logs from a browser):
 * - LOCAL FILE SERVER. The box now also serves its SD logs over WiFi as a simple web page, so you
 *   can browse and download SESS_*.csv/.log from a laptop/phone on the same network — redundancy for
 *   the v6.11 cloud upload, and a no-external-service fallback. Reach it at http://<box-ip>/ or
 *   http://z370.local/ (mDNS). SD reads are bracketed with the same g_fb_pause_sd_writes flag the
 *   file browser uses, so they can't collide with the logging writer; downloads stream straight off
 *   the card. Filenames are validated (SESS_*.csv/.log only) to block path traversal.
 * - BOUNDED SERVE WINDOW. To avoid holding WiFi's RAM for the whole drive (internal RAM is the
 *   tightest resource), it serves for FILE_SERVER_MINUTES after boot (default 15, set in /wifi.cfg,
 *   0 = off) inside timeSyncTask, then powers WiFi down and reclaims the RAM exactly as before.
 *   NOT flash-tested — main risk is internal RAM while WiFi + the full UI run together.
 *
 * v6.11 Changes (cloud auto-upload of session logs):
 * - CLOUD UPLOAD. At boot, while WiFi is already up for NTP, the box now streams any completed
 *   SESS_*.csv/.log that haven't been uploaded yet to a configurable HTTPS endpoint, then powers
 *   WiFi off as before. With the bundled Google Apps Script (cloud/CloudUpload.gs) the files land
 *   in your Google Drive (folder CLOUD_FOLDER), so they sync to your PC/phone — no more pulling the
 *   SD card or plugging in USB. Endpoint-agnostic: point CLOUD_URL at Dropbox/S3/your own server
 *   and only /wifi.cfg changes. Uploaded sessions are tracked in /CLOUDUP.DAT (never re-sent),
 *   capped at CLOUD_MAX_PER_BOOT per boot; the current (open) session uploads on the next boot.
 * - Config via the SAME /wifi.cfg: CLOUD_URL / CLOUD_SECRET / CLOUD_FOLDER (added to the template).
 *   The TimeSyncTask stack was raised 4096->16384 for the TLS handshake. NOT flash-tested: verify
 *   the Apps Script 302 redirect follows and that TLS fits in RAM on first flash.
 *
 * v6.10 Changes (fuel-trust transparency + OBD trouble-code reading):
 * - FUEL-TRUST BREAKDOWN LOGGED. The four inputs to the Fuel Trust score are now written to the
 *   CSV (12 appended columns): stft_b1/b2, ltft_b1/b2, fuel_trim_valid, ft_timing_deg, and the
 *   four deductions pen_stft/pen_ltft/pen_bank/pen_timing, plus mil_on + dtc_count. The [OBD]
 *   serial line now prints Bank 2 (ST2/LT2) again, so the .log shows both banks.
 * - FUEL TRUST TAP POPUP. Tapping the Fuel Trust value opens a breakdown panel: both banks'
 *   STFT/LTFT, timing, each of the four penalties, a plain-English "what it means" line, and the
 *   active trouble codes. Tap anywhere to close.
 * - OBD TROUBLE-CODE (DTC) READING. Added Mode 01 PID 0x01 (MIL lamp + stored count) and a
 *   Mode 03 reader (ISO-TP single- and multi-frame) that decodes stored DTCs to Pxxxx strings,
 *   prints a [DTC] serial line, and shows them in the popup — no scanner needed.
 * - FIRMWARE VERSION now stamped into both logs: the .log header line and a "# 370zMonitor FW"
 *   comment line at the top of each CSV (previously the version was in neither).
 * - HOST-VERIFIED logic only (DTC byte decode, penalty exposure). NOT flash-tested yet. On first
 *   flash, verify the popup's decoded DTCs against your scanner (the Mode 03 count-byte format is
 *   assumed per ISO 15765-4).
 *
 * v6.9 Changes (automatic DST + expanded OBD logging):
 * - AUTOMATIC DAYLIGHT SAVING (no more seasonal reflash). The DS3231 now stores UTC and local
 *   time is derived from a POSIX TZ rule (POSIX_TZ = "CST6CDT,M3.2.0,M11.1.0"), so the
 *   spring-forward (2nd Sun of March) / fall-back (1st Sun of November) hour is applied
 *   automatically. NTP writes UTC to the RTC; readRTCLocal() converts UTC->local on read.
 *   NOTE: after flashing, do ONE WiFi/NTP sync so the RTC is re-written in UTC (until then the
 *   old local-time RTC value will read ~5-6 h off).
 * - EXPANDED OBD-II LOGGING: added throttle(0x11), engine load(0x04), IAT(0x0F), ambient(0x46),
 *   engine oil temp(0x5C), MAF(0x10), commanded lambda(0x44), module/battery voltage(0x42),
 *   fuel level(0x2F), barometric pressure(0x33), fuel-system status(0x03). RPM/throttle are
 *   oversampled to stay responsive; poll period 200->120ms, stale 3000->5000ms. New values are
 *   logged to the CSV (22 appended columns) and printed on a new [OBD2] serial line. Any PID the
 *   ECU doesn't support simply stays invalid/-1. Logging only — no new gauges.
 *
 * v6.8 Changes (post-track-day analysis follow-ups):
 * - TIME-SYNC FIX: tryNTPSync() no longer accepts the stale RTC-seeded system clock as a
 *   successful sync. It seeds an old sentinel, waits for SNTP to push the clock to a real
 *   date (year>=2025), and restores the prior clock on failure. Root cause of the
 *   "clock is days behind / non-monotonic" problem: getLocalTime() returned the wrong RTC
 *   time before the real NTP response landed, then that wrong time was written back to the
 *   RTC — so NTP re-cemented the error instead of fixing it. (Also verify the DS3231 coin
 *   cell; do one WiFi/NTP boot near a known network to rewrite the RTC after flashing.)
 *   (Coolant Value-Critical threshold left at 220F — a 235F bump was considered then reverted.)
 *
 * v6.7 Changes (track-day quick pass - smart oil-pressure monitor):
 * - Logging raised to 10 Hz (SD_WRITE_INTERVAL_MS 1000->100, queue 16->40) to catch
 *   sub-second oil-pressure dips.
 * - isOilPressureCritical() is now RPM-aware (warm-oil floor ~10 psi/1000 rpm + idle
 *   floor) instead of a fixed <10 psi trip; still drives the "Value Critical" label.
 * - LIS3DH accelerometer calibrated from 55 mi of logs: raw axes rotated into car frame
 *   (X=lateral, Y=longitudinal, Z=vertical). Vertical exact; lat/lon provisional (~+/-20deg).
 * - Timestamps: system clock set from the DS3231 at boot (settimeofday) so SD file dates
 *   are correct without WiFi; DST fixed (DAYLIGHT_OFFSET_SEC 0->3600).
 * - Added a per-second [SESSION] summary line (min oil PSI >2k rpm, peak oil temp, max G).
 *
 * v6.6 Changes (ROLLBACK: restore original 5-channel mapping; real root cause found):
 * - The oil-pressure dropout chased through v6.3-v6.5 was NEVER a bad channel. It was a
 *   grounding/isolation fault: the P51 loop shared the electric box's power-and-ground,
 *   so its 4 mA had a sneak path around the Waveshare's sense resistor and read 0/---.
 * - Proven on the bench: both P51 units read a rock-steady 4 mA on an isolated supply
 *   (HANGELL), and dropped out only on box power. A sibling 4-20mA channel (diff temp)
 *   ran fine on the same module/box the whole time.
 * - FIX (hardware): power the oil-pressure loop from an ISOLATED 24V DC-DC (Mean Well
 *   DDR-15G-24 or Traco TRN 3-1215) fed off car 12V; output- ties to AI1- only. This
 *   gives the loop its own floating reference, exactly like the bench supply did.
 * - FIRMWARE: reverted all diagnostic swaps to the clean original layout —
 *   AI1 Oil Pressure, AI2 Oil Temp, AI3 Trans, AI4 Steer, AI5 Diff. NUM_CHANNELS 6->5,
 *   boot configures CH1-CH5 Mode 3 (dead-CH1 skip + CH6 spare removed). All jumpers ON.
 *
 * v6.5 Changes (DIAGNOSTIC A/B swap: oil pressure CH2 <-> trans temp CH3):
 * - On v6.4 (oil pressure on CH2) the P51 read a flat 0 uA while the temp
 *   sensors on CH3-CH5 read perfectly. To isolate sensor-vs-channel, swap the
 *   two loops: oil pressure -> CH3 (AI3, where trans temp just worked flawlessly),
 *   trans temp -> CH2 (AI2, where oil pressure read flat 0).
 *   MODBUS_CH_OIL_PRESSURE 1->2, MODBUS_CH_TRANS_TEMP 2->1.
 * - Physically swap the loops between AI2 and AI3 (both jumpers already ON/current).
 *   Expected: P51 steady ~4 mA on CH3 + trans temp fine on CH2 => sensor good,
 *   AI2 wiring was the problem. P51 still flickers/0 on CH3 => P51/harness is bad.
 * - Diagnostic config; revert (or keep, if CH3 proves good) after the test.
 *
 * v6.4 Changes (oil pressure moved off INTERMITTENT CH6 -> CH2):
 * - CH6 (AI6) current-sense path is intermittent: reads a clean 4.00 mA then
 *   drops to 0, repeatedly (jumper/terminal contact). Sensor + harness proven
 *   good by a channel-swap test, so the fault is the CH6 channel on the module.
 *   This module now has TWO bad channels: CH1 dead, CH6 intermittent.
 * - Oil pressure (P51 4-20mA) relocated CH6 -> CH2 (AI2), a proven-good current
 *   channel. MODBUS_CH_OIL_PRESSURE 5 -> 1. The unused oil-temp slot (no physical
 *   sensor) is parked on the freed CH6 (MODBUS_CH_OIL_TEMP 1 -> 5).
 * - Read range unchanged (regs 0..5). Boot still configures CH2-CH6 for Mode 3,
 *   skips dead CH1. Serial labels updated CH6 -> CH2 for oil pressure.
 * - Physical: move the P51 loop to AI2 (24V+ -> Pin1; Pin3 -> AI2+; AI2- -> GND)
 *   and set the AI2 jumper to current mode (ON), like the temp channels.
 *
 * v6.3 Changes (oil pressure moved off DEAD Waveshare CH1 -> CH6):
 * - The Waveshare CH1 (AI1) current input is DEAD: stuck at full-scale
 *   (20000 uA / 150 PSI), dead-steady at atmosphere, confirmed with TWO
 *   different P51 sensors on the same module/firmware (CH2-CH5 read fine).
 *   Most likely collateral damage to AI1's sense circuit from the
 *   reverse-polarity event that killed the first P51.
 * - Oil pressure (P51 4-20mA) is now read from CH6 (AI6). MODBUS_CH_OIL_PRESSURE
 *   = 5, MODBUS_NUM_CHANNELS = 6 (read registers 0..5, index 0/CH1 ignored).
 * - CH1 is EXCLUDED from mode config: boot now configures CH2-CH6 for Mode 3
 *   (4-20mA), not CH1-CH5. Nothing reads CH1's value.
 * - Wiring change: move the P51 loop return from AI1+ to AI6+ (AI6- -> GND).
 *   CH6 register/mode: data input reg 0x0005, mode reg 0x1005.
 *
 * v6.2 Changes (oil pressure: PX3 voltage sensor -> P51 4-20mA current loop):
 * - Replaced PX3AN2BH150PSAAX (0.5-4.5V ratiometric, needed a regulated 5V
 *   buck supply) with Amphenol SSI P51-150-G-B-P-20MA (4-20mA loop). The P51
 *   is loop-powered straight off the 24V rail (8-30V) — no buck converter,
 *   which removes the 5V-overvoltage failure mode that killed the PX3.
 * - Waveshare CH1 now configured to Mode 3 (4-20mA) at boot alongside CH2-CH5;
 *   module returns uA directly. convertToPSI() rewritten to current-based:
 *   PSI = ((uA - 4000) / 16000) * 150, clamped [0,150].
 * - CH1 disconnect detection now uses PRTXI_MIN_VALID_UA (<3mA = open loop)
 *   instead of the legacy mV threshold.
 * - Wiring: 24V+ -> P51 Red(Pin1 Vin); P51 White(Pin3) -> Waveshare AI1+;
 *   AI1- -> GND. P51 Black(Pin2) unused. 1.5KE36A TVS across 24V rail.
 *
 * v6.1 Changes (OBD CAN operational on-car + stale-value UI fix):
 * - CAN_MUX_TO_CAN set to 1 — EXIO5 driven HIGH at boot, so the FSUSB42UMX mux
 *   connects GPIO19/20 to the onboard TJA1051T. OBD CAN confirmed reading the
 *   2018 370Z ECU (water temp / ECT) on the car over a 2-wire CANH/CANL tap.
 * - FIXED: Water Temp and Fuel Trust gauges latched their last value when OBD
 *   data went stale (they had no "became-invalid -> show ---" handler). Added the
 *   same invalid-transition block the Modbus gauges use; both now clear to "---"
 *   ~3 s (OBD_PID_STALE_THRESHOLD_MS) after the ECU stops responding.
 * - NOTE: Fuel Trust still only displays once ECT >= OBD_WARMUP_TEMP_C (80 C).
 *
 * v6.0 Changes (OBD CAN finally works):
 * - FIXED: EXIO_CAN_SEL (CH422G EXIO5) now driven HIGH in initIOExtension().
 *   On this board GPIO19/20 are shared between native USB and the TJA1051T CAN
 *   transceiver via an FSUSB42UMX mux; EXIO5 selects which. It was never set,
 *   so the transceiver was disconnected from the MCU and CAN produced no data.
 * - FIXED: CAN TX/RX pins were swapped. Waveshare routes GPIO20=CANTX, GPIO19=CANRX;
 *   firmware had them reversed. Now CAN_TX_PIN=GPIO20, CAN_RX_PIN=GPIO19.
 * - NOTE: CAN mode disables native USB (USB-MSC cannot run while CAN is active).
 *   Flash/serial use the separate UART USB-C port.
 * - See obd_can_wiring.md for harness colors, termination jumper, and bring-up.
 *
 * v5.9 Changes:
 * - REMOVED: 10kΩ/22kΩ voltage divider on PX3AN2BH150PSAAX oil pressure signal
 * - Sensor now wires directly to Waveshare AI1 (Mode 0 = 0-10V natively handles 0.5-4.5V)
 * - PRESSURE_DIVIDER_RATIO changed from 1.4545 to 1.0 — Modbus mV is sensor mV directly
 * - Restored proper 3-conductor wiring (V+, GND, Signal) to PX3 sensor
 *
 * v5.8 Changes:
 * - ADDED: LIS3DH accelerometer support via I2C (ADA2809 breakout)
 * - G-sensor logs X/Y/Z in g-forces to CSV and .log files
 * - Toast system monitors accelerometer connection status
 * - Demo mode simulates accelerometer values (gentle sway pattern)
 * - Accelerometer connected via I2C hub with RTC and touch controller
 *
 * v5.7 Changes:
 * - ADDED: OBD-II via CAN bus (TWAI) for Water/Coolant Temperature (ECT)
 * - ADDED: Fuel Trust calculation from OBD fuel trims and timing advance
 * - Fuel Trust: Penalizes high STFT/LTFT, bank imbalance, and timing pulls
 * - Fuel Trust only displayed when engine warmed up (ECT >= 80°C)
 * - OBD polling: Round-robin 8 PIDs at 5Hz each (~1.6s full cycle)
 * - CAN pins configurable (default GPIO17/18) for SN65HVD230 transceiver
 *
 * v5.6 Changes:
 * - ADDED: Auto Brightness feature (sunrise/sunset based screen dimming)
 * - Uses NOAA Solar Calculator algorithm (same as Google Maps)
 * - Location: Schaumburg, IL (configurable via LOCATION_LATITUDE/LONGITUDE)
 * - Day: 100% brightness, Night: 35% brightness, 20-min twilight buffer
 * - New "Auto Brightness: ON/OFF" button in utility box
 * - Single-tap on utility box now toggles manual override (opposite brightness)
 * - Auto brightness preference saved to flash (persists across reboots)
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
#include <WiFiClientSecure.h>        // v6.11: HTTPS client for cloud log upload
#include <HTTPClient.h>              // v6.11: HTTP(S) POST for cloud log upload
#include <WebServer.h>              // v6.12: local WiFi file page (browse/download logs)
#include <ESPmDNS.h>                // v6.12: advertise http://z370.local for the file page
#include <time.h>                   // Time functions
#include <stdarg.h>                 // For va_list in SerialLogf
#include <vector>                   // For file browser
#include "driver/twai.h"            // ESP32 CAN (TWAI) controller for OBD-II
#include "esp_task_wdt.h"           // v6.14: task watchdog (auto-reset on a hang)
#include <math.h>                   // For fabsf, fminf in Fuel Trust calculation
#include <Adafruit_LIS3DH.h>         // LIS3DH accelerometer (ADA2809)
#include <Adafruit_Sensor.h>         // Required by Adafruit LIS3DH
#include <sys/time.h>                // v6.7: settimeofday() to set system clock from RTC

//-----------------------------------------------------------------

//=================================================================
// FEATURE FLAG - G-SENSOR (LIS3DH Accelerometer)
//=================================================================
#define ENABLE_GSENSOR              1   // Enable LIS3DH accelerometer logging

//=================================================================
// AUTO BRIGHTNESS - Sunrise/Sunset based screen dimming
// Uses NOAA Solar Calculator algorithm (same as Google Maps)
//=================================================================

// Location: Schaumburg, IL
// Find your coordinates at: https://www.latlong.net/
#define LOCATION_LATITUDE   42.03   // Degrees North (negative for South)
#define LOCATION_LONGITUDE -88.08   // Degrees West (negative for West)

// Brightness levels (0-255)
#define BRIGHTNESS_DAY      255     // 100% - full brightness during daytime
#define BRIGHTNESS_NIGHT    90      // ~35% - dimmed at night

// Twilight offset (minutes before sunrise / after sunset to start transition)
#define TWILIGHT_OFFSET_MINUTES  20

// How often to check time (sunrise/sunset times cached daily)
#define AUTO_BRIGHTNESS_CHECK_INTERVAL_MS  30000  // Check every 30 seconds

// Auto brightness state
static struct {
    bool enabled;               // User preference (saved to flash)
    bool initialized;
    int last_check_day;         // Day of year when times were last calculated
    float sunrise_hours;        // Sunrise time in hours (e.g., 6.5 = 6:30 AM)
    float sunset_hours;         // Sunset time in hours (e.g., 19.5 = 7:30 PM)
    bool is_daytime;            // Current calculated state
    bool manual_override;       // True if user manually changed brightness
    uint32_t last_check_ms;     // Last time we checked
    char sunrise_str[8];        // "HH:MM" format for display
    char sunset_str[8];         // "HH:MM" format for display
} g_auto_brightness = {
    .enabled = true,            // ON by default
    .initialized = false,
    .last_check_day = -1,
    .sunrise_hours = 6.0f,
    .sunset_hours = 18.0f,
    .is_daytime = true,
    .manual_override = false,
    .last_check_ms = 0
};

// Forward declarations for auto brightness
void autoBrightnessInit();
bool autoBrightnessUpdate();
void autoBrightnessForceUpdate();
void saveAutoBrightnessPreference();
void loadAutoBrightnessPreference();
void updateAutoBrightnessButtonText();

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
#define ENABLE_OBD_CAN                  1       // Enable OBD-II via CAN bus (TWAI)
#ifndef FW_VERSION
#define FW_VERSION                      "v6.15" // single source of truth for the serial banner + .log/.csv stamps
#endif
// v6.15: Fuel Trust timing term. 1 = baseline delta from the Heat Derate Monitor (timing at WOT
// vs the learned cool map, same rpm bin); 0 = legacy consecutive-sample "pull" counter, which
// compared samples 3-5 s and thousands of rpm apart and fired during gentle driving.
#define FUEL_TRUST_TIMING_MODE          1
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

// Save/Load auto brightness preference
void saveAutoBrightnessPreference() {
    g_prefs.begin("display", false);
    g_prefs.putBool("auto_bri", g_auto_brightness.enabled);
    g_prefs.end();
    Serial.printf("[PREFS] Auto brightness saved: %s\n", g_auto_brightness.enabled ? "ON" : "OFF");
}

void loadAutoBrightnessPreference() {
    g_prefs.begin("display", true);
    g_auto_brightness.enabled = g_prefs.getBool("auto_bri", true);  // Default ON
    g_prefs.end();
    Serial.printf("[PREFS] Auto brightness loaded: %s\n", g_auto_brightness.enabled ? "ON" : "OFF");
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
    int oil_temp_value_f;
    bool oil_temp_valid;

    // Water/Coolant Temperature (stored in Fahrenheit internally)
    int water_temp_value_f;
    bool water_temp_valid;

    // Transmission Temperature (stored in Fahrenheit internally)
    int trans_temp_value_f;
    bool trans_temp_valid;

    // Power Steering Temperature (stored in Fahrenheit internally)
    int steer_temp_value_f;
    bool steer_temp_valid;

    // Differential Temperature (stored in Fahrenheit internally)
    int diff_temp_value_f;
    bool diff_temp_valid;

    // Fuel Trust (confidence percentage)
    int fuel_trust_percent;
    bool fuel_trust_valid;

    // OBD Data
    int rpm;
    bool rpm_valid;

    // Accelerometer (LIS3DH) - values in g-forces
    float accel_x_g;        // X-axis acceleration in g (lateral, positive = right)
    float accel_y_g;        // Y-axis acceleration in g (longitudinal, positive = forward)
    float accel_z_g;        // Z-axis acceleration in g (vertical, positive = up)
    bool accel_valid;       // Accelerometer data validity flag

    // v6.9 extended OBD-II data (logged to CSV; not shown on gauges) — value + valid each
    int   throttle_pct;        bool throttle_valid;         // 0x11 [%]
    int   engine_load_pct;     bool engine_load_valid;      // 0x04 [%]
    int   intake_air_temp_f;   bool intake_air_temp_valid;  // 0x0F [°F]
    int   ambient_temp_f;      bool ambient_temp_valid;     // 0x46 [°F]
    int   obd_oil_temp_f;      bool obd_oil_temp_valid;     // 0x5C [°F] (separate from the Modbus oil-temp sensor)
    float maf_gps;             bool maf_valid;              // 0x10 [g/s]
    float commanded_lambda;    bool lambda_valid;           // 0x44 [lambda ratio]
    float module_voltage;      bool module_voltage_valid;   // 0x42 [V]
    int   fuel_level_pct;      bool fuel_level_valid;       // 0x2F [%]
    int   baro_kpa;            bool baro_valid;             // 0x33 [kPa]
    int   fuel_sys_status;     bool fuel_sys_valid;         // 0x03 [raw status byte]

    // v6.10 fuel-trust transparency: raw trims, the four score deductions, and DTC status
    float stft_b1, stft_b2, ltft_b1, ltft_b2; bool fuel_trim_valid;  // live trims [%]
    float ft_timing_deg;                                             // timing advance [°]
    float ft_pen_stft, ft_pen_ltft, ft_pen_bank, ft_pen_timing;      // deductions from 100
    bool  mil_on;              // MIL (check-engine) lamp commanded on (PID 0x01)
    int   dtc_count;           // stored DTC count (PID 0x01; -1 = unknown)
    bool  dtc_valid;           // PID 0x01 answered

    // v6.15 Heat Derate Monitor ("POWER") — logged to CSV (14 appended columns)
    float tim_base_deg;        // cool-baseline timing for the current rpm bin (0 = no baseline yet)
    float tim_delta_deg;       // last WOT timing sample minus baseline (0 = none)
    int   tim_bin_n;           // samples in that baseline bin
    int   pedal_pct;           // accelerator pedal [%] (PID 0x49 or CAN 0x180)
    int   pedal_src;           // 0 none, 1 = PID 0x49, 2 = CAN 0x180 byte F
    int   thr_gap_pct;         // pedal - throttle plate at WOT, minus the cool gap (0 = n/a)
    int   load_delta_pct;      // calculated load minus cool baseline at WOT (0 = n/a)
    int   rev_cap_rpm;         // rpm plateau flagged as a rev cap this session (0 = none)
    float air_loss_pct;        // SAE J1349 density loss vs 77F/990mb from IAT + baro [%]
    int   pwr_state;           // 0 OK,1 AIR,2 LIFT,3 THROTTLE,4 TIMING,5 FUEL,6 REVCAP
    int   pwr_sev;             // 0 ok, 1 amber, 2 red
    int   atf1_f, atf2_f;      // TCM ATF temps [F] via Mode 21 (provisional decode)
    bool  atf_valid;
    int   tcc_slip_rpm;        // torque-converter slip [rpm] (provisional decode)

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
// v6.6: ORIGINAL MAPPING RESTORED. The v6.3-v6.5 channel swaps were all chasing the
// oil-pressure dropout, which turned out to be a grounding/isolation fault (the sensor
// loop shared the electric box's power-and-ground), NOT any bad channel. Root cause and
// fix proven on the bench: the P51 (both units) read a rock-steady 4 mA on an isolated
// supply and dropped out on box power. Fix = power the oil-pressure loop from an
// ISOLATED 24V DC-DC (Mean Well DDR-15G-24 / Traco TRN 3-1215) fed off car 12V, output-
// to AI1- only. So every channel is good; we revert to the clean 5-channel layout.
//   AI1 = Oil Pressure (P51, via isolated DC-DC) | AI2 = Oil Temp | AI3 = Trans Temp
//   AI4 = Steer Temp | AI5 = Diff Temp.  All jumpers ON (current mode), all Mode 3.
#define MODBUS_CH_OIL_PRESSURE  0   // Channel 1 (AI1) - P51 4-20mA loop (via isolated DC-DC)
#define MODBUS_CH_OIL_TEMP      1   // Channel 2 (AI2) - PRTXI RTD transmitter (4-20mA)
#define MODBUS_CH_TRANS_TEMP    2   // Channel 3 (AI3) - PRTXI RTD transmitter (4-20mA)
#define MODBUS_CH_STEER_TEMP    3   // Channel 4 (AI4) - PRTXI RTD transmitter (4-20mA)
#define MODBUS_CH_DIFF_TEMP     4   // Channel 5 (AI5) - PRTXI RTD transmitter (4-20mA)

// Number of channels to read (registers 0..4 = CH1..CH5, all populated)
#define MODBUS_NUM_CHANNELS     5   // oil pressure (CH1) + oil/trans/steer/diff temps (CH2-CH5)

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

// Pressure Sensor Calibration (P51-150-G-B-P-20MA-000-000, 4-20mA loop)
// v6.2: Replaced PX3 (0.5-4.5V) with Amphenol SSI P51 4-20mA current-loop
// sensor. Waveshare CH1 is now set to Mode 3 (4-20mA) just like the PRTXI
// temp channels, so the module returns microamps (µA) directly over Modbus.
// Loop-powered straight off the 24V rail (8-30V) — no buck converter, no divider.
//
//   4000  µA (4mA)  = 0 PSI
//   12000 µA (12mA) = 75 PSI   (midpoint)
//   20000 µA (20mA) = 150 PSI
//
#define PRESSURE_MIN_CURRENT_UA   4000     // 4000 µA (4mA) = 0 PSI
#define PRESSURE_CURRENT_SPAN_UA  16000    // 20000 - 4000 = 16000 µA span
#define PRESSURE_FS_PSI           150.0f   // Full-scale pressure at 20mA

// --- OIL-PRESSURE DEBUG STREAM ---
// Set to 1 to stream CH1's raw loop current off the ESP32 so you can watch the
// actual mA (and whether it moves when you apply pressure) without a meter in
// mA mode. Handy for validating the isolated-supply fix; set to 0 for production.
#define DEBUG_OIL_PRESSURE        1        // 1 = print [OILP] CH1 raw uA/mA at ~5 Hz

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
// v6.2: CH1 (oil pressure) is now a 4-20mA loop like CH2-CH5, so it uses the
// PRTXI_MIN_VALID_UA threshold (below ~3mA = open loop = disconnected).
// SENSOR_MIN_VALID_MV is legacy (PX3 voltage era) and no longer referenced.
#define SENSOR_MIN_VALID_MV     100     // Legacy (PX3 voltage); unused as of v6.2
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
// convertToPSI() - Convert Modbus µA reading to PSI
//-----------------------------------------------------------------------------
// Sensor: P51-150-G-B-P-20MA-000-000 (0-150 PSI gauge, 4-20mA loop output)
//
// v6.2: Replaced the PX3 voltage sensor with the P51 4-20mA current-loop
// sensor. Waveshare CH1 is configured to Mode 3 (4-20mA), so the module
// returns microamps (µA) directly — identical path to the PRTXI temp
// channels. Loop-powered off the 24V rail; no buck converter, no divider.
//
//   4000  µA (4mA)  = 0 PSI
//   20000 µA (20mA) = 150 PSI
//
// Linear: PSI = ((µA - 4000) / 16000) * 150
//
static float convertToPSI(uint16_t modbus_uA) {
    float psi = (((float)modbus_uA - PRESSURE_MIN_CURRENT_UA) / PRESSURE_CURRENT_SPAN_UA) * PRESSURE_FS_PSI;
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
            // Liveness check reads register 0 (CH1 = oil pressure). It confirms the
            // module is answering on the bus; real per-sensor state is set on the
            // first poll in readModbusSensors().
            Serial.printf("[MODBUS] SUCCESS! comms OK (reg0/CH1 raw=%d)\n", value);
            g_modbus_initialized = true;
            g_modbus_comm_ok = true;

            // Oil pressure lives on CH1; real connected state is set on the
            // first poll in readModbusSensors().
            g_sensor_ch1_connected = false;

            // ========== CONFIGURE CURRENT-LOOP CHANNELS FOR 4-20mA ==========
            // v6.6: all five channels carry loop-powered 4-20mA sensors — oil pressure
            // on CH1 (via the isolated DC-DC), oil/trans/steer/diff temps on CH2-CH5.
            // Set every channel to Mode 3 (4-20mA) at boot. Each channel's jumper must
            // be ON (current mode) for the reading to be valid.
            Serial.println("[MODBUS] Configuring CH1-CH5 for 4-20mA mode (Mode 3)...");

            // Channel configuration array: {register, name}
            struct ChannelConfig {
                uint16_t reg;
                const char* name;
            };
            ChannelConfig currentChannels[] = {
                {WAVESHARE_CH1_MODE_REG, "CH1 (Oil Pressure)"},
                {WAVESHARE_CH2_MODE_REG, "CH2 (Oil Temp)"},
                {WAVESHARE_CH3_MODE_REG, "CH3 (Trans Temp)"},
                {WAVESHARE_CH4_MODE_REG, "CH4 (Steer Temp)"},
                {WAVESHARE_CH5_MODE_REG, "CH5 (Diff Temp)"}
            };

            for (int i = 0; i < (int)(sizeof(currentChannels) / sizeof(currentChannels[0])); i++) {
                uint16_t currentMode = 0;
                if (modbusReadHoldingRegister(MODBUS_SLAVE_ADDR, currentChannels[i].reg, &currentMode)) {
                    Serial.printf("[MODBUS] %s current mode: %d\n", currentChannels[i].name, currentMode);
                }
                
                if (modbusWriteHoldingRegister(MODBUS_SLAVE_ADDR, currentChannels[i].reg, WAVESHARE_MODE_4_20MA)) {
                    Serial.printf("[MODBUS] %s configured for 4-20mA (Mode 3)\n", currentChannels[i].name);
                } else {
                    Serial.printf("[MODBUS] WARNING: Failed to configure %s!\n", currentChannels[i].name);
                    Serial.println("[MODBUS]   Check: Is jumper set to 'I' or 'mA' position?");
                }
                // Read the mode back to confirm the write actually took
                uint16_t verifyMode = 0xFFFF;
                if (modbusReadHoldingRegister(MODBUS_SLAVE_ADDR, currentChannels[i].reg, &verifyMode)) {
                    Serial.printf("[MODBUS] %s readback mode = %u %s\n", currentChannels[i].name,
                                  verifyMode, (verifyMode == WAVESHARE_MODE_4_20MA) ? "(OK 4-20mA)" : "(NOT Mode 3!)");
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
// Forward declarations for critical thresholds (defined in Gauges Configuration section)
extern int OIL_PRESS_ValueCriticalAbsolute;
extern int OIL_PRESS_ValueCriticalLow;
extern int OIL_TEMP_ValueCriticalF;
extern int W_TEMP_ValueCritical_F;
extern int TRAN_TEMP_ValueCritical_F;
extern int STEER_TEMP_ValueCritical_F;
extern int DIFF_TEMP_ValueCritical_F;
extern int FUEL_TRUST_ValueCritical;

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
        
        // Channel 1: Oil Pressure (P51 4-20mA transmitter, via isolated DC-DC)
        // Waveshare configured to Mode 3 (4-20mA) - returns µA directly
        uint16_t oil_press_uA = g_modbus_channel_values[MODBUS_CH_OIL_PRESSURE];
        bool sensor_connected = (oil_press_uA >= PRTXI_MIN_VALID_UA);

#if DEBUG_OIL_PRESSURE
        // Stream raw CH1 current so the actual loop mA is visible (pump test):
        //   ~4 mA / ~0 PSI, rising with applied pressure = healthy sensor
        //   pegged >20 mA / >150 PSI and NOT moving with pressure = not regulating (toast)
        //   offline(<3mA) = open loop / wrong wires / no 24V
        static uint32_t lastOilDbg = 0;
        if (millis() - lastOilDbg >= 200) {   // ~5 Hz
            lastOilDbg = millis();
            float mA  = oil_press_uA / 1000.0f;
            float psi = ((float)oil_press_uA - PRESSURE_MIN_CURRENT_UA) / PRESSURE_CURRENT_SPAN_UA * PRESSURE_FS_PSI;
            Serial.printf("[OILP] CH1 raw=%u uA = %.2f mA -> %.1f PSI(unclamped) | %s\n",
                          oil_press_uA, mA, psi, sensor_connected ? "in-loop" : "offline(<3mA)");
        }
#endif

        // Log sensor state changes
        if (sensor_connected != g_sensor_ch1_connected) {
            if (sensor_connected) {
                Serial.printf("[MODBUS] CH1 (Oil Pressure): Sensor CONNECTED (%d µA)\n", oil_press_uA);
            } else {
                Serial.printf("[MODBUS] CH1 (Oil Pressure): Sensor DISCONNECTED (%d µA < %d µA threshold)\n",
                             oil_press_uA, PRTXI_MIN_VALID_UA);
            }
            g_sensor_ch1_connected = sensor_connected;
        }
        
        if (sensor_connected) {
            // Sensor connected and reading valid
            float oil_press_psi = convertToPSI(oil_press_uA);
            
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
                Serial.printf("[MODBUS] CH2 (Oil Temp): PRTXI Sensor CONNECTED (%d uA = %.2f mA)\n",
                             oil_temp_uA, oil_temp_uA / 1000.0f);
            } else {
                Serial.printf("[MODBUS] CH2 (Oil Temp): PRTXI Sensor DISCONNECTED (%d uA < %d uA threshold)\n",
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
            g_vehicle_data.oil_temp_value_f = (int)(oil_temp_f + 0.5f);
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
            
            g_vehicle_data.trans_temp_value_f = (int)(trans_temp_f + 0.5f);
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
            
            g_vehicle_data.steer_temp_value_f = (int)(steer_temp_f + 0.5f);
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
            
            g_vehicle_data.diff_temp_value_f = (int)(diff_temp_f + 0.5f);
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
            char logBuf[512];  // Increased for "(VALUE CRITICAL)" indicators
            int pos = 0;
            
            // CH1: Oil Pressure
            if (sensor_connected) {
                int psi = (int)(convertToPSI(oil_press_uA) + 0.5f);
                bool crit = (psi < OIL_PRESS_ValueCriticalLow) || (psi > OIL_PRESS_ValueCriticalAbsolute);
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, "Oil-Press:%dPSI%s", 
                               psi, crit ? " (VALUE CRITICAL)" : "");
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, "Oil-Press:n/a");
            }
            
            // CH2: Oil Temp
            if (temp_sensor_connected) {
                bool crit = (g_vehicle_data.oil_temp_value_f > OIL_TEMP_ValueCriticalF);
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Oil-Temp:%.0f°F%s", 
                               celsiusToFahrenheit(g_last_oil_temp_c), crit ? " (VALUE CRITICAL)" : "");
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Oil-Temp:n/a");
            }
            
            // CH3: Trans Temp
            if (trans_sensor_connected) {
                bool crit = (g_vehicle_data.trans_temp_value_f > TRAN_TEMP_ValueCritical_F);
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Trans-Temp:%.0f°F%s", 
                               celsiusToFahrenheit(g_last_trans_temp_c), crit ? " (VALUE CRITICAL)" : "");
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Trans-Temp:n/a");
            }
            
            // CH4: Steer Temp
            if (steer_sensor_connected) {
                bool crit = (g_vehicle_data.steer_temp_value_f > STEER_TEMP_ValueCritical_F);
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Steer-Temp:%.0f°F%s", 
                               celsiusToFahrenheit(g_last_steer_temp_c), crit ? " (VALUE CRITICAL)" : "");
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Steer-Temp:n/a");
            }
            
            // CH5: Diff Temp
            if (diff_sensor_connected) {
                bool crit = (g_vehicle_data.diff_temp_value_f > DIFF_TEMP_ValueCritical_F);
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Diff-Temp:%.0f°F%s", 
                               celsiusToFahrenheit(g_last_diff_temp_c), crit ? " (VALUE CRITICAL)" : "");
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Diff-Temp:n/a");
            }
            
            // OBD: Water/Coolant Temp
            if (g_vehicle_data.water_temp_valid) {
                bool crit = (g_vehicle_data.water_temp_value_f > W_TEMP_ValueCritical_F);
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | W-Temp:%d°F%s", 
                               g_vehicle_data.water_temp_value_f, crit ? " (VALUE CRITICAL)" : "");
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | W-Temp:n/a");
            }
            
            // OBD: Fuel Trust
            if (g_vehicle_data.fuel_trust_valid) {
                bool crit = (g_vehicle_data.fuel_trust_percent < FUEL_TRUST_ValueCritical);
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Fuel-Trust:%d%%%s", 
                               g_vehicle_data.fuel_trust_percent, crit ? " (VALUE CRITICAL)" : "");
            } else {
                pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, " | Fuel-Trust:n/a");
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

//=================================================================
// LIS3DH ACCELEROMETER (G-SENSOR)
// Connected via I2C hub (Crowtail I2C Hub 2.0)
// ADA2809 breakout board (Adafruit LIS3DH)
//=================================================================

#if ENABLE_GSENSOR

#define LIS3DH_I2C_ADDR     0x18    // Default address (SA0 to GND)
#define LIS3DH_I2C_ADDR_ALT 0x19    // Alternate address (SA0 to VCC)

// Global accelerometer object
static Adafruit_LIS3DH g_accel = Adafruit_LIS3DH();
static bool g_accel_initialized = false;
static bool g_accel_connected = false;

// Initialize accelerometer - called during setup
bool initAccelerometer() {
    Serial.println("[G-SENSOR] Initializing LIS3DH accelerometer...");
    
    // Try default address first
    if (g_accel.begin(LIS3DH_I2C_ADDR)) {
        Serial.printf("[G-SENSOR] Found LIS3DH at 0x%02X\n", LIS3DH_I2C_ADDR);
        g_accel_initialized = true;
    } 
    // Try alternate address
    else if (g_accel.begin(LIS3DH_I2C_ADDR_ALT)) {
        Serial.printf("[G-SENSOR] Found LIS3DH at 0x%02X\n", LIS3DH_I2C_ADDR_ALT);
        g_accel_initialized = true;
    }
    else {
        Serial.println("[G-SENSOR] LIS3DH not found!");
        g_accel_initialized = false;
        g_accel_connected = false;
        return false;
    }
    
    // Configure accelerometer for track use
    // +/- 4G range is good for street/track driving
    g_accel.setRange(LIS3DH_RANGE_4_G);
    
    // Set data rate to 100 Hz (good balance of responsiveness vs power)
    g_accel.setDataRate(LIS3DH_DATARATE_100_HZ);
    
    // High-resolution mode for better precision
    g_accel.setPerformanceMode(LIS3DH_MODE_HIGH_RESOLUTION);
    
    g_accel_connected = true;
    Serial.printf("[G-SENSOR] LIS3DH configured: Range=+/-%dG, Rate=100Hz\n",
                  (g_accel.getRange() == LIS3DH_RANGE_2_G) ? 2 :
                  (g_accel.getRange() == LIS3DH_RANGE_4_G) ? 4 :
                  (g_accel.getRange() == LIS3DH_RANGE_8_G) ? 8 : 16);
    
    return true;
}

// v6.7: LIS3DH mounting calibration - rotate raw sensor axes into car frame.
// Derived from 55 mi of 7/5 logs (gravity vector + rpm-correlated forward axis).
// Vertical/level is EXACT; lateral & longitudinal are PROVISIONAL (~+/-20 deg) and
// will be re-derived from the cleaner 10 Hz data. If a known hard brake or a steady
// corner reads on the wrong axis or sign, flip the offending row. Set flag 0 for raw.
#define ENABLE_ACCEL_CALIBRATION 1
#define ACCEL_R_LAT_X   0.5296f
#define ACCEL_R_LAT_Y   0.0450f
#define ACCEL_R_LAT_Z  -0.8470f
#define ACCEL_R_LON_X  -0.8471f
#define ACCEL_R_LON_Y   0.0783f
#define ACCEL_R_LON_Z  -0.5256f
#define ACCEL_R_VRT_X   0.0427f
#define ACCEL_R_VRT_Y   0.9959f
#define ACCEL_R_VRT_Z   0.0796f

// Read accelerometer values - updates g_vehicle_data
void readAccelerometer() {
    if (!g_accel_initialized) {
        g_vehicle_data.accel_valid = false;
        return;
    }
    
    // Get new sensor event
    sensors_event_t event;
    if (!g_accel.getEvent(&event)) {
        // Read failed - sensor may be disconnected
        static uint32_t last_fail_log = 0;
        if (millis() - last_fail_log > 5000) {
            Serial.println("[G-SENSOR] Read failed - sensor may be disconnected");
            last_fail_log = millis();
        }
        g_accel_connected = false;
        g_vehicle_data.accel_valid = false;
        return;
    }
    
    g_accel_connected = true;
    
    // Convert from m/s² to g (divide by 9.80665)
    // The Adafruit library returns acceleration in m/s²
    const float GRAVITY = 9.80665f;
    float rax = event.acceleration.x / GRAVITY;
    float ray = event.acceleration.y / GRAVITY;
    float raz = event.acceleration.z / GRAVITY;
#if ENABLE_ACCEL_CALIBRATION
    // Rotate raw sensor axes into car frame: X=lateral(+right), Y=longitudinal(+fwd accel), Z=vertical(+up)
    g_vehicle_data.accel_x_g = ACCEL_R_LAT_X * rax + ACCEL_R_LAT_Y * ray + ACCEL_R_LAT_Z * raz;
    g_vehicle_data.accel_y_g = ACCEL_R_LON_X * rax + ACCEL_R_LON_Y * ray + ACCEL_R_LON_Z * raz;
    g_vehicle_data.accel_z_g = ACCEL_R_VRT_X * rax + ACCEL_R_VRT_Y * ray + ACCEL_R_VRT_Z * raz;
#else
    g_vehicle_data.accel_x_g = rax;
    g_vehicle_data.accel_y_g = ray;
    g_vehicle_data.accel_z_g = raz;
#endif
    g_vehicle_data.accel_valid = true;
    g_vehicle_data.has_received_data = true;
}

// Check if accelerometer is connected via I2C probe
bool isAccelerometerConnected() {
    if (!g_accel_initialized) return false;
    
    // Quick I2C probe
    Wire.beginTransmission(LIS3DH_I2C_ADDR);
    if (Wire.endTransmission() == 0) return true;
    
    Wire.beginTransmission(LIS3DH_I2C_ADDR_ALT);
    return (Wire.endTransmission() == 0);
}

#endif // ENABLE_GSENSOR

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
#define EXIO_CAN_SEL  5   // FSUSB42UMX USB mux select: HIGH = CAN mode (GPIO19/20 -> TJA1051T),
                          //                            LOW  = USB mode (GPIO19/20 -> native USB D-/D+).
                          // Must be HIGH for OBD CAN to work. NOTE: disables native USB (incl. USB-MSC) while set.
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
// v6.7: smart RPM-aware low-pressure floor (warm oil, track use). Critical when oil
// PSI < max(RPM * PSI_PER_1000 / 1000, idle floor). ~10 psi/1000 rpm is the motorsport
// minimum; VQ37 healthy warm sits well above it, so this flags genuine low pressure
// (and hard corner-surge dips) without false-alarming in normal running.
int OIL_PRESS_PSI_PER_1000RPM = 10;   // slope of the min-pressure floor
int OIL_PRESS_IdleFloorPSI    = 10;   // floor at/below idle (factory warm min ~14 psi)
int OIL_PRESS_RPM_ACTIVE      = 500;  // below this rpm, use the idle floor

// OIL TEMP: 150-300°F, Critical: >=260°F
int OIL_TEMP_Min_F = 150;
int OIL_TEMP_Max_F = 300;
int OIL_TEMP_ValueCriticalF = 260;

// WATER TEMP: 100-260°F, Critical: >=220°F
int W_TEMP_Min_F = 100;
int W_TEMP_Max_F = 260;
int W_TEMP_ValueCritical_F = 220;

// TRANS TEMP: 80-280°F, Critical: >=230°F
int TRAN_TEMP_Min_F = 80;
int TRAN_TEMP_Max_F = 280;
int TRAN_TEMP_ValueCritical_F = 230;

// STEER TEMP: 60-300°F, Critical: >=230°F
int STEER_TEMP_Min_F = 60;
int STEER_TEMP_Max_F = 300;
int STEER_TEMP_ValueCritical_F = 230;

// DIFF TEMP: 60-320°F, Critical: >=260°F
int DIFF_TEMP_Min_F = 60;
int DIFF_TEMP_Max_F = 320;
int DIFF_TEMP_ValueCritical_F = 260;

// FUEL TRUST: 0-100%, Critical: <=75%
int FUEL_TRUST_Min = 0;
int FUEL_TRUST_Max = 100;
int FUEL_TRUST_ValueCritical = 75;

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
    1,          // pclk_active_neg
    14000000,   // prefer_speed (pixel clock)
    true,       // useBigEndian — RGB565 byte order. MUST be true on this panel or colors invert.
                //   (This is the arg the old sketch's mislabeled 'auto_flush' true was actually setting.)
    0,          // de_idle_high
    0,          // pclk_idle_high
    (800 * 10)  // bounce_buffer_size_px: decouples LCD DMA from PSRAM reads (helps flicker; not VSYNC tearing)
);
Arduino_RGB_Display* gfx = new Arduino_RGB_Display(800, 480, rgbpanel, 0, true);

//-----------------------------------------------------------------

// LVGL
#define LVGL_BUFFER_SIZE (800 * 30)  // 30 lines, double-buffered in internal DMA RAM (~48KB each @ RGB565).
                                     // Restored to original size after moving LVGL's heap pool to PSRAM (lv_conf.h)
                                     // freed internal RAM. Larger buffer = fewer partial flushes = less tearing.
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
#define SD_QUEUE_SIZE 40   // v6.7: deeper queue for 10 Hz logging (absorbs flush stalls, was 16)
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
static lv_obj_t* mode_indicator = NULL;  // Shows "DEMO" or "LIVE" (inside the hidden utility box)
static lv_obj_t* g_demo_banner = NULL;   // v6.13: always-visible DEMO banner (top layer)
static lv_obj_t* auto_bri_btn = NULL;      // Auto Brightness toggle button
static lv_obj_t* auto_bri_lbl = NULL;      // Label inside auto brightness button
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
    bool accel_ok;            // LIS3DH accelerometer
    bool obd_can_ok;          // OBD-II CAN bus (TWAI)
    bool initialized;  // Set true after first check
} g_prev_system_status = {true, true, true, true, true, true, true, true, true, true, true, true, false};

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
    g_vehicle_data.oil_temp_value_f = calcDemoValue(OIL_TEMP_Min_F, OIL_TEMP_Max_F, DEMO_CYCLE_OIL_TEMP_MS);
    g_vehicle_data.oil_temp_valid = true;

    // ----- Water Temperature: Simple sine wave 100-260°F -----
    g_vehicle_data.water_temp_value_f = calcDemoValue(W_TEMP_Min_F, W_TEMP_Max_F, DEMO_CYCLE_WATER_TEMP_MS);
    g_vehicle_data.water_temp_valid = true;

    // ----- Trans Temperature: Simple sine wave 80-280°F -----
    g_vehicle_data.trans_temp_value_f = calcDemoValue(TRAN_TEMP_Min_F, TRAN_TEMP_Max_F, DEMO_CYCLE_TRANS_TEMP_MS);
    g_vehicle_data.trans_temp_valid = true;

    // ----- Steering Temperature: Simple sine wave 60-300°F -----
    g_vehicle_data.steer_temp_value_f = calcDemoValue(STEER_TEMP_Min_F, STEER_TEMP_Max_F, DEMO_CYCLE_STEER_TEMP_MS);
    g_vehicle_data.steer_temp_valid = true;

    // ----- Diff Temperature: Simple sine wave 60-320°F -----
    g_vehicle_data.diff_temp_value_f = calcDemoValue(DIFF_TEMP_Min_F, DIFF_TEMP_Max_F, DEMO_CYCLE_DIFF_TEMP_MS);
    g_vehicle_data.diff_temp_valid = true;

    // ----- Fuel Trust: Simple sine wave 0-100% -----
    g_vehicle_data.fuel_trust_percent = calcDemoValue(FUEL_TRUST_Min, FUEL_TRUST_Max, DEMO_CYCLE_FUEL_TRUST_MS);
    g_vehicle_data.fuel_trust_valid = true;

    // ----- RPM: Simple sine wave 700-6500 -----
    g_vehicle_data.rpm = calcDemoValue(700, 6500, DEMO_CYCLE_RPM_MS);
    g_vehicle_data.rpm_valid = true;

    // ----- Accelerometer: Gentle sway pattern simulating cornering -----
    // Use different cycle periods for each axis to create natural-looking motion
    uint32_t elapsed = millis() - g_demo_state.start_time;
    // X (lateral): ±0.8g with 4 second period (cornering)
    g_vehicle_data.accel_x_g = 0.8f * sinf((float)elapsed * 2.0f * M_PI / 4000.0f);
    // Y (longitudinal): ±0.5g with 3 second period (accel/brake)
    g_vehicle_data.accel_y_g = 0.5f * sinf((float)elapsed * 2.0f * M_PI / 3000.0f);
    // Z (vertical): 1.0g base ±0.2g with 2.5 second period (bumps)
    g_vehicle_data.accel_z_g = 1.0f + 0.2f * sinf((float)elapsed * 2.0f * M_PI / 2500.0f);
    g_vehicle_data.accel_valid = true;
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
    // Handles: Oil Pressure (CH1), Oil Temp (CH2), Trans Temp (CH3), Steer Temp (CH4), Diff Temp (CH5)
    readModbusSensors();
    
    // Note: Water temp and Fuel trust are handled by updateOBDData() (via CAN bus)
#else
    // No Modbus sensors configured - show "---" on Modbus gauges
    g_vehicle_data.oil_pressure_valid = false;
    g_vehicle_data.oil_temp_valid = false;
    g_vehicle_data.trans_temp_valid = false;
    g_vehicle_data.steer_temp_valid = false;
    g_vehicle_data.diff_temp_valid = false;
    // Note: Water temp and Fuel trust are handled by updateOBDData() (via CAN bus)
#endif

#if ENABLE_GSENSOR
    // Read accelerometer (LIS3DH via I2C hub)
    readAccelerometer();
#else
    g_vehicle_data.accel_valid = false;
#endif
}

#pragma endregion Sensor Data Provider

//=================================================================
// OBD DATA PROVIDER
// Reads from OBD-II via CAN bus using ESP32 TWAI controller
// Provides: Water/Coolant Temperature (ECT) and Fuel Trust calculation
//=================================================================

#pragma region OBD Data Provider

#if ENABLE_OBD_CAN

//-----------------------------------------------------------------------------
// CAN BUS CONFIGURATION (Waveshare ESP32-S3-Touch-LCD-7)
//-----------------------------------------------------------------------------
// The Waveshare board has an ONBOARD TJA1051T CAN transceiver (U7).
// No external transceiver needed — but GPIO19/20 are SHARED with native USB
// through an FSUSB42UMX mux selected by CH422G EXIO5 (see EXIO_CAN_SEL).
// EXIO_CAN_SEL must be driven HIGH (done in initIOExtension) or CAN is dead.
//
// PER WAVESHARE PINOUT (do not swap):
//   GPIO20 (CANTX) -> TJA1051T TXD   (ESP32 transmit)
//   GPIO19 (CANRX) <- TJA1051T RXD   (ESP32 receive)
//   TJA1051T CANH/CANL -> CAN PH2.0 terminal (J6)
//
// WIRING TO OBD-II PORT (370Z CAN-C, 500 kbps):
//   CANH -> OBD-II Pin 6   (pigtail Green)
//   CANL -> OBD-II Pin 14  (pigtail Brown/White)
//   GND  -> OBD-II Pin 4 + 5 (pigtail Orange + Yellow)
//
// TERMINATION: remove the board's onboard 120R jumper. The car's bus is already
// terminated (2x120R = 60R); the board terminator would make it 40R (out of spec).
//-----------------------------------------------------------------------------

#define CAN_TX_PIN  GPIO_NUM_20     // ESP32 CANTX -> onboard TJA1051T TXD (per Waveshare pinout)
#define CAN_RX_PIN  GPIO_NUM_19     // ESP32 CANRX <- onboard TJA1051T RXD

// OBD-II addressing (ISO 15765-4, 11-bit CAN)
#define OBD_FUNCTIONAL_REQ_ID  0x7DF    // Broadcast request address
#define OBD_MIN_RESP_ID        0x7E8    // ECU response range start
#define OBD_MAX_RESP_ID        0x7EF    // ECU response range end
#define OBD_PHYS_ECU_REQ_ID    0x7E0    // v6.14: physical request to ECU #1 (used for Mode 22)
#define OBD_DID_OIL_TEMP       0x111F   // v6.14: Nissan enhanced oil-temp DID (Mode 0x22); °C = A - 50

// v6.15: transmission control module (RE7R01A 7AT) — Mode 21 (ReadDataByLocalIdentifier)
// LID 0x20 returns a multi-frame record that ScanGauge's 370Z X-Gauges decode as two ATF
// temperatures (°C = raw - 55) and the torque-converter slip (16-bit rpm). The byte offsets
// inside the record are PROVISIONAL — the whole record is dumped on a [TCM] line so they can
// be confirmed against the PRTXI trans-temp sensor. Indices count from the 0x61 service byte.
#define OBD_PHYS_TCM_REQ_ID    0x7E1
#define OBD_TCM_RESP_ID        0x7E9
#define TCM_LID_TRANS_DATA     0x20
#define TCM_ATF1_IDX           4        // provisional: record byte holding ATF temp 1 (raw - 55 = °C)
#define TCM_ATF2_IDX           5        // provisional: ATF temp 2
#define TCM_TCC_IDX            7        // provisional: 16-bit big-endian torque-converter slip [rpm]
#define TCM_ATF_OFFSET_C       55       // °C = raw - 55 (from the X-Gauge math: F = raw*9/5 - 67)
#define TCM_POLL_INTERVAL_MS   1000

// v6.15: passively-heard Z34 broadcast frames (no request needed; from the 2010 370Z
// reverse-engineering, verify on this 2018 car via the [CAN180] log line)
#define CAN_ID_ENGINE_180      0x180    // bytes A,B = rpm (scale unverified), byte F = pedal (F/255*100)
#define CAN_ID_ECT_551         0x551    // byte A ≈ ECT + 40 (°C)
#define CAN_ID_VDC_354         0x354    // byte E: TCS (0 on / 64 off), byte G: brake (4 off / 20 pressed)
#define CAN_PEDAL_STALE_MS     500      // pedal from 0x180 is 100 Hz; older than this = not heard

//-----------------------------------------------------------------------------
// OBD PID POLLING CONFIGURATION
//-----------------------------------------------------------------------------
// Round-robin polling: one PID every OBD_PID_REQUEST_PERIOD_MS
// For 10 PIDs at 200ms each = 2 seconds full cycle (~0.5 Hz per PID)
//-----------------------------------------------------------------------------

#define OBD_PID_REQUEST_PERIOD_MS  120   // v6.9: poll one PID every 120ms (bigger list now; ~3s full cycle)
#define OBD_PID_STALE_THRESHOLD_MS 5000  // v6.9: raised from 3000 to cover the longer round-robin cycle
#define OBD_WARMUP_TEMP_C          80    // ECT threshold before showing Fuel Trust

// PIDs to poll (Mode 01 - Current Data).
// v6.9: expanded from 8 to a fuller set. The two fast-moving signals (RPM 0x0C, throttle 0x11)
// appear multiple times so they refresh ~5x faster than the slow once-per-cycle PIDs.
// Any PID the ECU doesn't support simply returns nothing and stays invalid (harmless).
static const uint8_t OBD_PID_LIST[] = {
    0x0C,  // Engine RPM: ((A*256)+B)/4 [rpm]
    0x11,  // Throttle position: A*100/255 [%]
    0x01,  // v6.10: Monitor status — MIL lamp + stored DTC count (once/cycle)
    0x05,  // Engine Coolant Temperature (ECT): A - 40 [°C]
    0x0D,  // Vehicle Speed: A [km/h]
    0x0C,  // RPM (repeat — fast mover)
    0x0E,  // Timing Advance: (A - 64) / 2 [° BTDC]
    0x04,  // Calculated engine load: A*100/255 [%]
    0x11,  // Throttle (repeat — fast mover)
    0x06,  // Short Term Fuel Trim Bank 1: (A-128)*100/128 [%]
    0x07,  // Long Term Fuel Trim Bank 1: (A-128)*100/128 [%]
    0x0C,  // RPM (repeat)
    0x08,  // Short Term Fuel Trim Bank 2: (A-128)*100/128 [%]
    0x09,  // Long Term Fuel Trim Bank 2: (A-128)*100/128 [%]
    0x0F,  // Intake air temp (IAT): A - 40 [°C]
    0x0C,  // RPM (repeat)
    0x10,  // MAF air flow: ((A*256)+B)/100 [g/s]
    0x44,  // Commanded equivalence ratio (lambda): ((A*256)+B)/32768
    0x11,  // Throttle (repeat)
    0x42,  // Control module (battery) voltage: ((A*256)+B)/1000 [V]
    0x03,  // Fuel system status (raw byte A: 1=OL, 2=CL, 4=OL-drive, 8=OL-fault, 16=CL-fault)
    0x0C,  // RPM (repeat)
    0x2F,  // Fuel level: A*100/255 [%]  (Nissan support varies)
    0x5C,  // Engine oil temperature: A - 40 [°C]  (Nissan support varies — free oil-temp source if present)
    0x46,  // Ambient air temp: A - 40 [°C]
    0x33,  // Barometric pressure (absolute): A [kPa]
    0x49,  // v6.15: Accelerator pedal position D: A*100/255 [%] (skipped if the bitmap says unsupported)
};
#define OBD_NUM_PIDS (sizeof(OBD_PID_LIST) / sizeof(OBD_PID_LIST[0]))

// v6.15: POWER-WINDOW burst list. While the driver is hard on the throttle (see
// pwrUpdateWindow()) the scheduler polls THIS list at OBD_PID_REQUEST_PERIOD_POWER_MS instead
// of the full list, so timing advance refreshes ~8 Hz, rpm ~3 Hz, throttle/load/pedal ~1.7 Hz.
// That is what makes a 2-7 s pull measurable (the full list gave 2-3 timing samples per pull).
// Temperatures don't move in 5 s, so they simply wait for the window to end. Total request
// rate stays <= 17/s (still well below what ELM-class loggers put on the bus).
static const uint8_t OBD_PID_LIST_POWER[] = {
    0x0E, 0x0C, 0x0E, 0x11, 0x0E, 0x04, 0x0E, 0x0C, 0x0E, 0x49,
};
#define OBD_NUM_PIDS_POWER (sizeof(OBD_PID_LIST_POWER) / sizeof(OBD_PID_LIST_POWER[0]))
#define OBD_PID_REQUEST_PERIOD_POWER_MS  60     // one request per 60 ms inside a power window
#define OBD_PID_STALE_THRESHOLD_POWER_MS 12000  // slow PIDs may legitimately go quiet for a whole straight
#define PWR_WINDOW_THROTTLE_PCT          70     // plate (PID 0x11) >= this ...
#define PWR_WINDOW_RPM_MIN               3000   // ... and rpm >= this opens a power window
#define PWR_WINDOW_EXIT_MS               1500   // window closes this long after the condition clears

//-----------------------------------------------------------------------------
// OBD STATE VARIABLES
//-----------------------------------------------------------------------------

static bool g_obd_initialized = false;
static int g_obd_pid_index = 0;                  // Round-robin index
static uint32_t g_obd_last_request_ms = 0;       // Last PID request time
static uint32_t g_obd_success_count = 0;
static uint32_t g_obd_error_count = 0;
static uint32_t g_obd_tx_error_count = 0;        // Transmit failures
static uint32_t g_obd_tx_total_count = 0;        // Total TX attempts
static bool g_obd_boot_status_logged = false;    // Post-boot status flag

// Live OBD values with timestamps for staleness detection
static struct {
    // Engine Coolant Temperature
    int coolant_c;                    // [°C]
    uint32_t coolant_timestamp;
    bool coolant_valid;
    
    // RPM
    float rpm;                        // [rpm]
    uint32_t rpm_timestamp;
    bool rpm_valid;
    
    // Vehicle Speed
    int speed_kph;                    // [km/h]
    uint32_t speed_timestamp;
    bool speed_valid;
    
    // Timing Advance
    float timing_deg;                 // [degrees BTDC]
    uint32_t timing_timestamp;
    bool timing_valid;
    
    // Fuel Trims
    float stft_b1;                    // Short Term Fuel Trim Bank 1 [%]
    float ltft_b1;                    // Long Term Fuel Trim Bank 1 [%]
    float stft_b2;                    // Short Term Fuel Trim Bank 2 [%]
    float ltft_b2;                    // Long Term Fuel Trim Bank 2 [%]
    uint32_t fuel_trim_timestamp;
    bool fuel_trim_valid;

    // v6.9 extended PIDs — each value + its own timestamp + valid flag (independent staleness)
    int      throttle_pct;      uint32_t throttle_ts;      bool throttle_valid;       // 0x11
    int      load_pct;          uint32_t load_ts;          bool load_valid;           // 0x04
    int      iat_c;             uint32_t iat_ts;           bool iat_valid;            // 0x0F
    int      ambient_c;         uint32_t ambient_ts;       bool ambient_valid;        // 0x46
    int      oil_temp_c;        uint32_t oil_temp_ts;      bool oil_temp_valid;       // 0x5C (may be unsupported)
    float    maf_gps;           uint32_t maf_ts;           bool maf_valid;            // 0x10
    float    lambda;            uint32_t lambda_ts;        bool lambda_valid;         // 0x44
    float    module_voltage;    uint32_t module_voltage_ts;bool module_voltage_valid; // 0x42
    int      fuel_level_pct;    uint32_t fuel_level_ts;    bool fuel_level_valid;     // 0x2F (may be unsupported)
    int      baro_kpa;          uint32_t baro_ts;          bool baro_valid;           // 0x33
    int      fuel_sys_status;   uint32_t fuel_sys_ts;      bool fuel_sys_valid;       // 0x03 (raw status byte A)

    // v6.15: accelerator pedal from PID 0x49 (D). If the ECU never answers it, the
    // passively-heard CAN 0x180 byte F takes over (see g_can180 below).
    int      pedal_pct;         uint32_t pedal_ts;         bool pedal_valid;          // 0x49
    // v6.15: previous rpm sample, used to extrapolate rpm to a timing sample's timestamp
    float    rpm_prev;          uint32_t rpm_prev_ts;
} g_obd_data = {0};

// ---------------------------------------------------------------------------
// v6.15: power-window flag (drives the burst scheduler + relaxed staleness)
// ---------------------------------------------------------------------------
static bool     g_pwr_window = false;        // true while the driver is hard on the throttle
static uint32_t g_pwr_window_cond_ms = 0;    // last time the open-condition was true
static int      g_obd_pwr_index = 0;         // round-robin index into OBD_PID_LIST_POWER

// ---------------------------------------------------------------------------
// v6.15: supported-PID discovery (Mode 01 PIDs 0x00/0x20/0x40/0x60/0x80 bitmaps)
// ---------------------------------------------------------------------------
static uint32_t g_pid_bitmap[5]   = {0, 0, 0, 0, 0};  // bits for PIDs 0x01-0x20, 0x21-0x40, ...
static bool     g_pid_bitmap_ok[5] = {false, false, false, false, false};
static int      g_disc_next_block = 0;        // next bitmap block to request (0..4); 5 = done
static uint32_t g_disc_last_req_ms = 0;
static int      g_disc_retries = 0;
static bool     g_pid49_supported = true;     // assume yes until the 0x40 bitmap says otherwise

static inline bool obdPidSupported(uint8_t pid) {
    if (pid == 0) return true;
    int block = (pid - 1) / 32;               // 0x01-0x20 -> block 0, 0x21-0x40 -> 1, ...
    if (block < 0 || block > 4 || !g_pid_bitmap_ok[block]) return true;  // unknown = assume yes
    int bit = 31 - ((pid - 1) % 32);          // MSB of byte A = first PID of the block
    return (g_pid_bitmap[block] >> bit) & 1;
}

// ---------------------------------------------------------------------------
// v6.15: TCM Mode 21 (LID 0x20) ISO-TP reassembly + decoded ATF temps / TCC slip
// ---------------------------------------------------------------------------
#define TCM_ASM_BUF_LEN 64
static uint8_t  g_tcm_asm_buf[TCM_ASM_BUF_LEN];
static int      g_tcm_asm_len = 0, g_tcm_asm_got = 0;
static bool     g_tcm_asm_active = false;
static uint32_t g_tcm_asm_start_ms = 0;
static uint32_t g_tcm_last_poll_ms = 0;
static uint32_t g_tcm_last_dump_ms = 0;
static int      g_tcm_atf1_c = 0, g_tcm_atf2_c = 0, g_tcm_tcc_slip = 0;
static bool     g_tcm_valid = false;
static uint32_t g_tcm_ts = 0;
static uint32_t g_tcm_reply_count = 0;

// ---------------------------------------------------------------------------
// v6.15: passive CAN — frames the car broadcasts anyway (zero added bus load)
// ---------------------------------------------------------------------------
static struct {
    uint8_t  pedal_raw;  uint16_t rpm_raw;  uint32_t ts;  uint32_t count;   // 0x180
    uint8_t  ect_raw;    uint32_t ect_ts;                                   // 0x551 byte A
    uint8_t  vdc_e, vdc_g; uint32_t vdc_ts;                                 // 0x354 bytes E,G
} g_can180 = {0};
#define CAN_CENSUS_SLOTS 40
static struct { uint16_t id; uint32_t count; } g_can_census[CAN_CENSUS_SLOTS];
static int      g_can_census_n = 0;
static bool     g_can_census_printed = false;
static uint32_t g_can_census_start_ms = 0;

// ---------------------------------------------------------------------------
// v6.15: optional Nissan Mode 22 DID sweep (OBD_DID_SWEEP=1 in /wifi.cfg). Walks
// 0x1100-0x12FF at idle, one request per 60 ms, logging every DID that answers.
// ---------------------------------------------------------------------------
static bool     g_did_sweep_enabled = false;   // from /wifi.cfg
static bool     g_did_sweep_done = false;
static bool     g_did_sweep_running = false;
static uint16_t g_did_sweep_next = 0x1100;
static uint32_t g_did_sweep_last_ms = 0;
static int      g_did_sweep_hits = 0;
static bool     g_pwr_baseline_reset_req = false;  // PWR_BASELINE_RESET=1 in /wifi.cfg

// ---------------------------------------------------------------------------
// v6.15: HEAT DERATE MONITOR ("POWER") — state. Functions live after updateOBDData().
// ---------------------------------------------------------------------------
#define PWR_RPM_BIN_MIN      2000
#define PWR_RPM_BIN_SIZE     500
#define PWR_RPM_BINS         12         // 2000-2499 ... 7500-7999
#define PWR_WOT_LOAD_PCT     85         // calculated load >= this (with throttle >= PWR_WINDOW_THROTTLE_PCT) = WOT sample
#define PWR_COOL_ECT_MIN_F   160        // baseline learns only when warmed up ...
#define PWR_COOL_ECT_MAX_F   205        // ... but not hot
#define PWR_COOL_IAT_MAX_F   120
#define PWR_COOL_OIL_MAX_F   230        // applied only when an oil temp is known
#define PWR_BIN_MIN_N        5          // bin usable for deltas after this many cool samples
#define PWR_EMA_ALPHA        0.10f
#define PWR_ALIGN_RPM_MS     400        // rpm sample must be at most this old to place a timing sample
#define PWR_ALIGN_LOAD_MS    800
#define PWR_TIM_AMBER_DEG    -3.0f
#define PWR_TIM_RED_DEG      -6.0f
#define PWR_TIM_HOLD_MS      1000       // delta must stay past the threshold this long
#define PWR_TIM_LATCH_MS     10000      // ... and the state stays up this long after the last bad sample
#define PWR_GAP_PTS          10         // pedal-vs-throttle gap growth vs cool [points]
#define PWR_GAP_HOLD_MS      500
#define PWR_LOAD_DEFICIT_PCT 8          // load below cool baseline [%]
#define PWR_LOAD_HOLD_MS     500
#define PWR_REVCAP_BAND_RPM  150        // plateau = rpm stays within +/- this ...
#define PWR_REVCAP_HOLD_MS   500        // ... for this long at >= 85% throttle
#define PWR_REVCAP_MAX_RPM   7000       // plateaus above this are just the normal limiter
#define PWR_REVCAP_OIL_F     250        // "oil is hot" co-condition for the plateau
#define PWR_REVCAP_JAG_RPM   80         // or: limiter-style jaggedness inside the plateau
#define PWR_REVCAP_LATCH_MS  30000
#define PWR_FUEL_DROP_MIN    300        // rpm stumble at WOT: drop between MIN and MAX ...
#define PWR_FUEL_DROP_MAX    1000       // ... (bigger = an upshift, not fuel)
#define PWR_FUEL_RECOVER_MS  1000       // ... that recovers within this = FUEL?
#define PWR_FUEL_LAT_G       0.8f
#define PWR_FUEL_LATCH_MS    30000
#define PWR_AIR_BANNER_PCT   6.0f       // AIR shows on the banner only past this loss
#define PWR_LATCH_MS         10000      // THROTTLE / LIFT latch
#define PWR_BASELINE_SAVE_MS 120000     // NVS write at most this often (only when changed)
#define PWR_BASELINE_VERSION 2   // v6.15: bumped when PwrBin gained mean-rpm (slope interpolation)

enum PwrState { PWR_OK = 0, PWR_AIR, PWR_LIFT, PWR_THROTTLE, PWR_TIMING, PWR_FUEL, PWR_REVCAP };
static const char* const PWR_STATE_NAME[] = { "OK", "AIR", "LIFT", "THROTTLE", "TIMING", "FUEL?", "REV CAP" };

struct PwrBin { float tim; float load; float gap; float rpm; uint16_t n; uint16_t n_gap; };
struct PwrBaseline { uint8_t version; uint8_t pad[3]; PwrBin bin[PWR_RPM_BINS]; };
static PwrBaseline g_pwr_base = {0};
static bool     g_pwr_base_dirty = false;
static uint32_t g_pwr_base_saved_ms = 0;

struct PwrMonitor {
    // last processed WOT timing sample
    uint32_t last_tim_ts;         // timestamp of the last timing sample we looked at
    int      bin;                 // rpm bin of the last WOT sample (-1 = none)
    float    tim_live, tim_base, tim_delta;  // last WOT sample, its baseline, the difference
    int      bin_n;
    bool     delta_valid;         // bin had >= PWR_BIN_MIN_N cool samples
    uint32_t delta_ts;            // when tim_delta was last computed
    float    delta_worst;         // most negative delta inside the current latch window
    // timing alarm timing
    uint32_t tim_bad_since;       // 0 = delta not currently past amber
    uint32_t tim_last_bad_ms;
    uint32_t tim_red_since;       // 0 = delta not currently past red (separate sustained breach)
    uint32_t tim_last_red_ms;
    int      tim_sev;             // 0/1/2 (amber/red) currently latched
    // pedal / throttle gap
    int      pedal_pct, pedal_src;   // src: 0 none, 1 PID 0x49, 2 CAN 0x180
    int      pedal_max_seen;         // session max pedal (pedal is "pinned" within 5 pts of it)
    int      gap_live, gap_delta;    // pedal - throttle now; minus cool gap
    uint32_t gap_bad_since, gap_last_bad_ms;
    // load deficit
    int      load_delta;
    uint32_t load_bad_since, load_last_bad_ms;
    // rev cap (plateau detector)
    uint32_t last_rpm_ts;         // last rpm sample folded into the plateau tracker
    float    plat_ref, plat_min, plat_max;
    uint32_t plat_since;
    int      rev_cap_rpm;         // 0 = none this session
    int      rev_cap_count;
    uint32_t rev_cap_ms;
    // fuel stumble
    float    fuel_pre_rpm; uint32_t fuel_drop_ms; bool fuel_pending; float fuel_lat_g;
    uint32_t fuel_ms;
    // air density
    float    air_cf, air_loss_pct;
    // resulting state
    int      state, sev;
    char     reason[48];
    uint32_t state_since;
    // session stats for [SESSION]
    float    sess_min_delta; int sess_max_iat_f; float sess_max_air_loss; uint16_t sess_states_seen;
    // banner acknowledge
    int      ack_state, ack_sev;
    bool     banner_shown;
} g_pwr = {0};

// Fuel Trust calculation state
static float g_obd_last_timing_deg = NAN;        // For timing pull detection
static unsigned g_obd_timing_pull_count = 0;     // Timing pull event counter
static uint32_t g_obd_timing_pull_reset_ms = 0;  // Reset counter periodically
#define FUEL_TRUST_TIMING_RESET_INTERVAL_MS 10000 // Reset timing pulls every 10s

// ---------------------------------------------------------------------------
// v6.10: Fuel-Trust transparency + OBD trouble-code (DTC) reading
// (FW_VERSION is defined unconditionally near the top with the feature flags,
//  because the .log/.csv/banner use it even when ENABLE_OBD_CAN is 0.)
// ---------------------------------------------------------------------------

// Live breakdown of the Fuel Trust score, populated at the end of
// computeFuelTrust(). Read by the SD logger (via the g_vehicle_data copy) and
// by the FUEL TRUST tap popup so the number is never a black box.
struct FuelTrustBreakdown {
    float st1, st2, lt1, lt2;                 // live trims [%] (0 when OBD invalid)
    float timing_deg;                         // live timing advance [°]
    float avgAbsST, avgAbsLT;                 // averaged magnitudes actually penalized
    float deltaST, deltaLT;                   // bank-to-bank spreads
    float penST, penLT, penBank, penTiming;   // the four deductions from 100
    int   timingPulls;                        // timing-pull events in the window
    int   score;                              // resulting 0-100
    bool  valid;                              // true when trims were valid at compute
};
static FuelTrustBreakdown g_ft = {0};

// MIL lamp + DTC count from Mode 01 PID 0x01 (single-frame, always reliable)
static bool g_mil_on       = false;
static int  g_dtc_count_01 = 0;
static bool g_dtc01_valid  = false;

// Stored trouble codes from Mode 03 (ISO-TP; may span multiple CAN frames)
#define MAX_STORED_DTCS 12
static char     g_dtc_list[MAX_STORED_DTCS][6];   // decoded strings, e.g. "P010B"
static int      g_dtc_list_count = 0;
static bool     g_dtc_list_valid = false;
static uint32_t g_dtc_last_read_ms = 0;
#define DTC_READ_INTERVAL_MS 15000                // re-read stored codes every 15 s

// ISO-TP reassembly state for the Mode 03 (0x43) response
static bool     g_dtc_asm_active   = false;
static uint8_t  g_dtc_asm_buf[2 + MAX_STORED_DTCS * 2 + 8];
static int      g_dtc_asm_len      = 0;   // total payload bytes expected
static int      g_dtc_asm_got      = 0;   // payload bytes collected so far
static uint8_t  g_dtc_asm_next_sn  = 1;   // next expected consecutive-frame seq #
static uint32_t g_dtc_asm_start_ms = 0;   // start time (for timeout)
static uint32_t g_dtc_asm_txid     = 0;   // ECU physical addr for the flow-control frame
#define DTC_ASM_TIMEOUT_MS 200                    // give up on a stuck reassembly

//-----------------------------------------------------------------------------
// startCAN() - Initialize ESP32 TWAI driver
//-----------------------------------------------------------------------------
static bool startCAN() {
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    general.rx_queue_len = 64;   // v6.15: 32 -> 64, the drain now also reads broadcast frames
    general.tx_queue_len = 8;
    
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();  // 500 kbit/s for OBD-II
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Accept all frames
    
    esp_err_t err = twai_driver_install(&general, &timing, &filter);
    if (err != ESP_OK) {
        Serial.printf("[OBD] TWAI driver install failed: 0x%X\n", err);
        return false;
    }
    
    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("[OBD] TWAI start failed: 0x%X\n", err);
        twai_driver_uninstall();
        return false;
    }
    
    return true;
}

//-----------------------------------------------------------------------------
// sendOBD_Mode01() - Send Mode 01 PID request
//-----------------------------------------------------------------------------
static void sendOBD_Mode01(uint8_t pid) {
    twai_message_t tx = {};
    tx.identifier = OBD_FUNCTIONAL_REQ_ID;
    tx.extd = 0;              // 11-bit ID
    tx.rtr = 0;               // Data frame
    tx.data_length_code = 8;
    tx.data[0] = 0x02;        // 2 data bytes follow
    tx.data[1] = 0x01;        // Mode 01 (current data)
    tx.data[2] = pid;         // Requested PID
    tx.data[3] = 0x00;
    tx.data[4] = 0x00;
    tx.data[5] = 0x00;
    tx.data[6] = 0x00;
    tx.data[7] = 0x00;
    
    g_obd_tx_total_count++;
    esp_err_t err = twai_transmit(&tx, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        g_obd_tx_error_count++;
        // Log first few TX errors and then periodically
        if (g_obd_tx_error_count <= 3 || (g_obd_tx_error_count % 50 == 0)) {
            Serial.printf("[OBD] TX error 0x%X for PID 0x%02X (err#%lu/%lu)\n",
                err, pid, g_obd_tx_error_count, g_obd_tx_total_count);
        }
    }
}

//-----------------------------------------------------------------------------
// v6.10: OBD Mode 03 — read stored Diagnostic Trouble Codes (DTCs)
//-----------------------------------------------------------------------------
// The Mode 03 reply (service byte 0x43) can arrive as one CAN frame or as an
// ISO-TP multi-frame sequence, so frames are reassembled in handleDTCFrame()
// and turned into strings by finalizeDTCs(). Failure is benign: the worst case
// is an empty/partial list, and the popup still shows MIL + count from PID 0x01.
//
// CAN Mode 03 payload is assumed to be: [0x43][count][DTC1_hi][DTC1_lo]...
// (the ISO 15765-4 count byte). Verify the decoded codes against a scanner on
// first flash; if they are off by one byte, the count-byte assumption is wrong.
static void sendOBD_Mode03() {
    twai_message_t tx = {};
    tx.identifier = OBD_FUNCTIONAL_REQ_ID;   // 0x7DF functional request
    tx.extd = 0; tx.rtr = 0; tx.data_length_code = 8;
    tx.data[0] = 0x01;   // 1 data byte follows
    tx.data[1] = 0x03;   // Mode 03 (request stored DTCs)
    g_obd_tx_total_count++;
    twai_transmit(&tx, pdMS_TO_TICKS(10));
    // Reset reassembly state for this new request
    g_dtc_asm_active = false;
    g_dtc_asm_len = g_dtc_asm_got = 0;
    g_dtc_asm_next_sn = 1;
}

//-----------------------------------------------------------------------------
// v6.14: OBD Mode 0x22 (ReadDataByIdentifier) — Nissan enhanced PIDs
//-----------------------------------------------------------------------------
// The generic oil-temp PID (0x5C) is unsupported on the VQ37, but the 370Z exposes
// oil temp via the Nissan enhanced DID 0x111F over Mode 0x22 (physical addr 7E0):
//   request  7E0: 03 22 11 1F ...
//   response 7E8: xx 62 11 1F A ...   ->  oil_temp_C = A - 50
// NOT flash-tested — validate the offset against the factory oil-temp gauge on first
// drive; if it reads ~50 low/high, the -50 constant is the thing to tweak.
static void sendOBD_Mode22(uint16_t did) {
    twai_message_t tx = {};
    tx.identifier = OBD_PHYS_ECU_REQ_ID;   // 0x7E0 (physical, not the 0x7DF broadcast)
    tx.extd = 0; tx.rtr = 0; tx.data_length_code = 8;
    tx.data[0] = 0x03;                     // 3 data bytes follow
    tx.data[1] = 0x22;                     // Mode 22 (ReadDataByIdentifier)
    tx.data[2] = (did >> 8) & 0xFF;        // DID high
    tx.data[3] = did & 0xFF;               // DID low
    g_obd_tx_total_count++;
    twai_transmit(&tx, pdMS_TO_TICKS(10));
}

// Decode a Mode 22 positive response (service byte 0x62).
static void decodeOBD_Mode22Reply(const twai_message_t& rx) {
    if (rx.data_length_code < 5 || rx.data[1] != 0x62) return;
    uint16_t did = ((uint16_t)rx.data[2] << 8) | rx.data[3];
    uint8_t A = rx.data[4];
    uint32_t now = millis();
    if (did == OBD_DID_OIL_TEMP) {
        g_obd_data.oil_temp_c = (int)A - 50;   // Nissan: °C = A - 50
        g_obd_data.oil_temp_ts = now;
        g_obd_data.oil_temp_valid = true;
        g_obd_success_count++;
    }
}

// ISO-TP Flow Control (Clear-To-Send) to a specific ECU physical address.
static void sendFlowControl(uint32_t txid) {
    twai_message_t tx = {};
    tx.identifier = txid;
    tx.extd = 0; tx.rtr = 0; tx.data_length_code = 8;
    tx.data[0] = 0x30;   // FC: ContinueToSend
    tx.data[1] = 0x00;   // Block size: all remaining frames
    tx.data[2] = 0x00;   // STmin: 0 ms
    twai_transmit(&tx, pdMS_TO_TICKS(10));
}

// Decode a 2-byte DTC into a 5-char string (e.g. 0x01,0x0B -> "P010B").
static void dtcToString(uint8_t b1, uint8_t b2, char* out /* buffer >= 6 */) {
    static const char sysLetter[4] = {'P', 'C', 'B', 'U'};
    static const char* hex = "0123456789ABCDEF";
    out[0] = sysLetter[(b1 >> 6) & 0x03];   // 2 MSB pick P/C/B/U
    out[1] = (char)('0' + ((b1 >> 4) & 0x03)); // next 2 bits: 0-3
    out[2] = hex[b1 & 0x0F];
    out[3] = hex[(b2 >> 4) & 0x0F];
    out[4] = hex[b2 & 0x0F];
    out[5] = '\0';
}

// Convert the reassembled payload ([0x43][count][pairs...]) into strings.
static void finalizeDTCs() {
    g_dtc_list_count = 0;
    if (g_dtc_asm_got >= 2 && g_dtc_asm_buf[0] == 0x43) {
        int count = g_dtc_asm_buf[1];
        for (int i = 0; i < count && g_dtc_list_count < MAX_STORED_DTCS; i++) {
            int off = 2 + i * 2;
            if (off + 1 >= g_dtc_asm_got) break;      // ran out of bytes
            uint8_t b1 = g_dtc_asm_buf[off], b2 = g_dtc_asm_buf[off + 1];
            if (b1 == 0 && b2 == 0) continue;          // padding / empty slot
            dtcToString(b1, b2, g_dtc_list[g_dtc_list_count]);
            g_dtc_list_count++;
        }
    }
    g_dtc_list_valid = true;
    // Human-readable summary into the .log
    char codes[MAX_STORED_DTCS * 7 + 1]; codes[0] = '\0';
    for (int i = 0; i < g_dtc_list_count; i++) {
        strncat(codes, g_dtc_list[i], sizeof(codes) - strlen(codes) - 1);
        if (i < g_dtc_list_count - 1) strncat(codes, " ", sizeof(codes) - strlen(codes) - 1);
    }
    Serial.printf("[DTC] MIL:%s stored=%d codes: %s\n",
                  g_mil_on ? "ON" : "off", g_dtc_list_count,
                  g_dtc_list_count ? codes : "(none)");
}

// Feed one CAN frame belonging to a Mode 03 (0x43) reply into the ISO-TP
// reassembler. Returns true if the frame was consumed as DTC data (so the
// caller should NOT hand it to the Mode 01 decoder).
static bool handleDTCFrame(const twai_message_t& rx) {
    if (rx.data_length_code < 2) return false;
    uint8_t pci = rx.data[0] & 0xF0;

    if (pci == 0x00) {                         // Single Frame
        if (rx.data[1] != 0x43) return false;
        int len = rx.data[0] & 0x0F;           // payload byte count
        if (len < 1) len = 1;
        g_dtc_asm_got = 0;
        for (int i = 0; i < len && g_dtc_asm_got < (int)sizeof(g_dtc_asm_buf); i++)
            g_dtc_asm_buf[g_dtc_asm_got++] = rx.data[1 + i];
        g_dtc_asm_active = false;
        finalizeDTCs();
        return true;
    }
    else if (pci == 0x10) {                    // First Frame (multi-frame reply)
        if (rx.data_length_code < 3 || rx.data[2] != 0x43) return false;
        g_dtc_asm_len = ((rx.data[0] & 0x0F) << 8) | rx.data[1];  // total payload len
        g_dtc_asm_got = 0;
        for (int i = 0; i < 6 && g_dtc_asm_got < (int)sizeof(g_dtc_asm_buf); i++)
            g_dtc_asm_buf[g_dtc_asm_got++] = rx.data[2 + i];       // 0x43,count,4 bytes
        g_dtc_asm_active   = true;
        g_dtc_asm_next_sn  = 1;
        g_dtc_asm_start_ms = millis();
        g_dtc_asm_txid     = rx.identifier - 8;   // 0x7E8 -> 0x7E0 physical addr
        sendFlowControl(g_dtc_asm_txid);
        return true;
    }
    else if (pci == 0x20) {                    // Consecutive Frame
        if (!g_dtc_asm_active) return false;
        for (int i = 1; i < 8 && g_dtc_asm_got < g_dtc_asm_len &&
                        g_dtc_asm_got < (int)sizeof(g_dtc_asm_buf); i++)
            g_dtc_asm_buf[g_dtc_asm_got++] = rx.data[i];
        if (g_dtc_asm_got >= g_dtc_asm_len) {
            g_dtc_asm_active = false;
            finalizeDTCs();
        }
        return true;
    }
    return false;
}

//-----------------------------------------------------------------------------
// decodeOBD_Mode01Reply() - Parse Mode 01 response and update values
//-----------------------------------------------------------------------------
static void decodeOBD_Mode01Reply(const twai_message_t& rx) {
    if (rx.data_length_code < 3) return;
    if (rx.data[1] != 0x41) return;  // 0x41 = Mode 01 response
    
    uint8_t pid = rx.data[2];
    uint32_t now = millis();
    
    switch (pid) {
        case 0x01: {  // Monitor status since DTCs cleared: MIL lamp + stored DTC count
            if (rx.data_length_code >= 4) {
                uint8_t A = rx.data[3];
                g_mil_on = (A & 0x80) != 0;   // bit 7 = MIL commanded on
                g_dtc_count_01 = A & 0x7F;    // bits 0-6 = number of stored DTCs
                g_dtc01_valid = true;
                g_obd_success_count++;
            }
        } break;

        case 0x05: {  // Engine Coolant Temperature: A - 40 [°C]
            if (rx.data_length_code >= 4) {
                g_obd_data.coolant_c = (int)rx.data[3] - 40;
                g_obd_data.coolant_timestamp = now;
                g_obd_data.coolant_valid = true;
                g_obd_success_count++;
            }
        } break;
        
        case 0x0C: {  // Engine RPM: ((A*256)+B)/4 [rpm]
            if (rx.data_length_code >= 5) {
                uint16_t A = rx.data[3];
                uint16_t B = rx.data[4];
                // v6.15: keep the previous sample so a timing sample can be placed at the
                // rpm the engine was actually at (rpm moves ~600 rpm in 0.3 s in 2nd gear)
                if (g_obd_data.rpm_valid) {
                    g_obd_data.rpm_prev    = g_obd_data.rpm;
                    g_obd_data.rpm_prev_ts = g_obd_data.rpm_timestamp;
                }
                g_obd_data.rpm = ((A << 8) | B) / 4.0f;
                g_obd_data.rpm_timestamp = now;
                g_obd_data.rpm_valid = true;
            }
        } break;

        // ---- v6.15: supported-PID bitmaps (discovery) ----
        case 0x00: case 0x20: case 0x40: case 0x60: case 0x80: {
            if (rx.data_length_code >= 7) {
                int block = pid / 0x20;
                g_pid_bitmap[block] = ((uint32_t)rx.data[3] << 24) | ((uint32_t)rx.data[4] << 16) |
                                      ((uint32_t)rx.data[5] << 8)  |  (uint32_t)rx.data[6];
                g_pid_bitmap_ok[block] = true;
                g_obd_success_count++;
                // Decode into a readable list so the .log answers "what does this ECU support?"
                char list[200]; int m = 0;
                for (int i = 0; i < 32 && m < (int)sizeof(list) - 4; i++)
                    if ((g_pid_bitmap[block] >> (31 - i)) & 1)
                        m += snprintf(list + m, sizeof(list) - m, "%02X ", pid + 1 + i);
                Serial.printf("[OBD-DISC] PIDs %02X-%02X bitmap=%08lX supported: %s\n",
                              pid + 1, pid + 0x20, (unsigned long)g_pid_bitmap[block], m ? list : "(none)");
                if (block == 2) {
                    g_pid49_supported = obdPidSupported(0x49);
                    Serial.printf("[OBD-DISC] pedal 0x49:%s 0x4A:%s cmd-throttle 0x4C:%s rel-throttle 0x45:%s rel-pedal 0x5A:%s\n",
                        obdPidSupported(0x49) ? "yes" : "no", obdPidSupported(0x4A) ? "yes" : "no",
                        obdPidSupported(0x4C) ? "yes" : "no", obdPidSupported(0x45) ? "yes" : "no",
                        obdPidSupported(0x5A) ? "yes" : "no");
                } else if (block == 3) {
                    Serial.printf("[OBD-DISC] torque 0x61:%s 0x62:%s 0x63:%s  MAF-B 0x66:%s  fuel-rate 0x5E:%s\n",
                        obdPidSupported(0x61) ? "yes" : "no", obdPidSupported(0x62) ? "yes" : "no",
                        obdPidSupported(0x63) ? "yes" : "no", obdPidSupported(0x66) ? "yes" : "no",
                        obdPidSupported(0x5E) ? "yes" : "no");
                }
                // Chain to the next block only if the ECU says it exists (last bit = "next block supported")
                if (block < 4 && (g_pid_bitmap[block] & 1)) g_disc_next_block = block + 1;
                else g_disc_next_block = 5;   // done
                g_disc_retries = 0;
            }
        } break;

        case 0x49: {  // v6.15: Accelerator pedal position D: A*100/255 [%]
            if (rx.data_length_code >= 4) {
                g_obd_data.pedal_pct = (int)((float)rx.data[3] * 100.0f / 255.0f + 0.5f);
                g_obd_data.pedal_ts = now; g_obd_data.pedal_valid = true;
            }
        } break;
        
        case 0x0D: {  // Vehicle Speed: A [km/h]
            if (rx.data_length_code >= 4) {
                g_obd_data.speed_kph = (int)rx.data[3];
                g_obd_data.speed_timestamp = now;
                g_obd_data.speed_valid = true;
            }
        } break;
        
        case 0x0E: {  // Timing Advance: (A - 64) / 2 [degrees BTDC]
            if (rx.data_length_code >= 4) {
                g_obd_data.timing_deg = ((float)rx.data[3] - 64.0f) / 2.0f;
                g_obd_data.timing_timestamp = now;
                g_obd_data.timing_valid = true;
            }
        } break;
        
        case 0x06: {  // STFT Bank 1: (A-128)*100/128 [%]
            if (rx.data_length_code >= 4) {
                g_obd_data.stft_b1 = ((float)rx.data[3] - 128.0f) * 100.0f / 128.0f;
                g_obd_data.fuel_trim_timestamp = now;
                g_obd_data.fuel_trim_valid = true;
            }
        } break;
        
        case 0x07: {  // LTFT Bank 1: (A-128)*100/128 [%]
            if (rx.data_length_code >= 4) {
                g_obd_data.ltft_b1 = ((float)rx.data[3] - 128.0f) * 100.0f / 128.0f;
            }
        } break;
        
        case 0x08: {  // STFT Bank 2: (A-128)*100/128 [%]
            if (rx.data_length_code >= 4) {
                g_obd_data.stft_b2 = ((float)rx.data[3] - 128.0f) * 100.0f / 128.0f;
            }
        } break;
        
        case 0x09: {  // LTFT Bank 2: (A-128)*100/128 [%]
            if (rx.data_length_code >= 4) {
                g_obd_data.ltft_b2 = ((float)rx.data[3] - 128.0f) * 100.0f / 128.0f;
            }
        } break;

        // ---- v6.9 extended PIDs ----
        case 0x11: {  // Throttle position: A*100/255 [%]
            if (rx.data_length_code >= 4) {
                g_obd_data.throttle_pct = (int)((float)rx.data[3] * 100.0f / 255.0f + 0.5f);
                g_obd_data.throttle_ts = now; g_obd_data.throttle_valid = true;
            }
        } break;
        case 0x04: {  // Calculated engine load: A*100/255 [%]
            if (rx.data_length_code >= 4) {
                g_obd_data.load_pct = (int)((float)rx.data[3] * 100.0f / 255.0f + 0.5f);
                g_obd_data.load_ts = now; g_obd_data.load_valid = true;
            }
        } break;
        case 0x0F: {  // Intake air temp: A - 40 [°C]
            if (rx.data_length_code >= 4) {
                g_obd_data.iat_c = (int)rx.data[3] - 40;
                g_obd_data.iat_ts = now; g_obd_data.iat_valid = true;
            }
        } break;
        case 0x46: {  // Ambient air temp: A - 40 [°C]
            if (rx.data_length_code >= 4) {
                g_obd_data.ambient_c = (int)rx.data[3] - 40;
                g_obd_data.ambient_ts = now; g_obd_data.ambient_valid = true;
            }
        } break;
        case 0x5C: {  // Engine oil temp: A - 40 [°C]
            if (rx.data_length_code >= 4) {
                g_obd_data.oil_temp_c = (int)rx.data[3] - 40;
                g_obd_data.oil_temp_ts = now; g_obd_data.oil_temp_valid = true;
            }
        } break;
        case 0x10: {  // MAF air flow: ((A*256)+B)/100 [g/s]
            if (rx.data_length_code >= 5) {
                uint16_t A = rx.data[3], B = rx.data[4];
                g_obd_data.maf_gps = (float)((A << 8) | B) / 100.0f;
                g_obd_data.maf_ts = now; g_obd_data.maf_valid = true;
            }
        } break;
        case 0x44: {  // Commanded equivalence ratio (lambda): ((A*256)+B)/32768
            if (rx.data_length_code >= 5) {
                uint16_t A = rx.data[3], B = rx.data[4];
                g_obd_data.lambda = (float)((A << 8) | B) / 32768.0f;
                g_obd_data.lambda_ts = now; g_obd_data.lambda_valid = true;
            }
        } break;
        case 0x42: {  // Control module (battery) voltage: ((A*256)+B)/1000 [V]
            if (rx.data_length_code >= 5) {
                uint16_t A = rx.data[3], B = rx.data[4];
                g_obd_data.module_voltage = (float)((A << 8) | B) / 1000.0f;
                g_obd_data.module_voltage_ts = now; g_obd_data.module_voltage_valid = true;
            }
        } break;
        case 0x2F: {  // Fuel level: A*100/255 [%]
            if (rx.data_length_code >= 4) {
                g_obd_data.fuel_level_pct = (int)((float)rx.data[3] * 100.0f / 255.0f + 0.5f);
                g_obd_data.fuel_level_ts = now; g_obd_data.fuel_level_valid = true;
            }
        } break;
        case 0x33: {  // Barometric pressure (absolute): A [kPa]
            if (rx.data_length_code >= 4) {
                g_obd_data.baro_kpa = (int)rx.data[3];
                g_obd_data.baro_ts = now; g_obd_data.baro_valid = true;
            }
        } break;
        case 0x03: {  // Fuel system status: raw status byte A
            if (rx.data_length_code >= 4) {
                g_obd_data.fuel_sys_status = (int)rx.data[3];
                g_obd_data.fuel_sys_ts = now; g_obd_data.fuel_sys_valid = true;
            }
        } break;

        default:
            break;
    }
}

//-----------------------------------------------------------------------------
// computeFuelTrust() - Calculate Fuel Trust score (0-100%)
//-----------------------------------------------------------------------------
// Penalizes:
//   - Large fuel trim magnitudes (STFT/LTFT > 5%)
//   - Bank-to-bank imbalance (difference > 5%)
//   - Timing pulls under load (drops > 3° when speed>30 km/h & rpm>2000)
//
// Returns: 0-100 score (higher = more confidence in fuel quality/tune)
//-----------------------------------------------------------------------------
static float computeFuelTrust() {
    // Get fuel trim values (default to 0 if not available)
    float st1 = g_obd_data.fuel_trim_valid ? g_obd_data.stft_b1 : 0.0f;
    float st2 = g_obd_data.fuel_trim_valid ? g_obd_data.stft_b2 : 0.0f;
    float lt1 = g_obd_data.fuel_trim_valid ? g_obd_data.ltft_b1 : 0.0f;
    float lt2 = g_obd_data.fuel_trim_valid ? g_obd_data.ltft_b2 : 0.0f;
    
    // Calculate averages and deltas
    float avgAbsST = 0.5f * (fabsf(st1) + fabsf(st2));
    float avgAbsLT = 0.5f * (fabsf(lt1) + fabsf(lt2));
    float deltaBanksST = fabsf(st1 - st2);
    float deltaBanksLT = fabsf(lt1 - lt2);
    
    // Penalties for trim magnitudes (tuned gently)
    // STFT > 5%: 0-10 point penalty
    float pST = (avgAbsST <= 5.0f) ? 0.0f : 
                (avgAbsST <= 8.0f) ? (avgAbsST - 5.0f) * 1.0f : 10.0f;
    
    // LTFT > 5%: 0-20 point penalty (more severe - indicates persistent issue)
    float pLT = (avgAbsLT <= 5.0f) ? 0.0f : 
                (avgAbsLT <= 10.0f) ? (avgAbsLT - 5.0f) * 1.5f : 20.0f;
    
    // Bank imbalance > 5%: 5 point penalty
    float pDelta = (deltaBanksST > 5.0f || deltaBanksLT > 5.0f) ? 5.0f : 0.0f;
    
    float pTiming = 0.0f;
#if FUEL_TRUST_TIMING_MODE
    // v6.15: timing term = how far WOT timing sits below the learned cool baseline at the
    // same rpm bin (Heat Derate Monitor). 0 pts down to -1°, then 3 pts per degree, max 30
    // (-3° -> 6, -6° -> 15, -11° -> 30). Uses the latest delta from the last 2 minutes.
    if (g_pwr.delta_valid && (millis() - g_pwr.delta_ts) < 120000UL) {
        float below = -g_pwr.tim_delta - 1.0f;
        if (below > 0.0f) pTiming = fminf(30.0f, below * 3.0f);
    }
    g_obd_timing_pull_count = (unsigned)(pTiming / 1.5f + 0.5f);   // keeps the popup/CSV field meaningful
#else
    // Legacy: count drops > 3° between consecutive samples when under load. (Compares
    // samples 3-5 s and thousands of rpm apart — fires during gentle driving.)
    if (g_obd_data.timing_valid && g_obd_data.speed_valid && g_obd_data.rpm_valid) {
        if (g_obd_data.speed_kph > 30 && g_obd_data.rpm > 2000.0f) {
            if (!isnan(g_obd_last_timing_deg)) {
                float drop = g_obd_data.timing_deg - g_obd_last_timing_deg;
                if (drop < -3.0f) {
                    g_obd_timing_pull_count++;
                }
            }
        }
    }
    g_obd_last_timing_deg = g_obd_data.timing_valid ? g_obd_data.timing_deg : NAN;

    // Timing pull penalty: 1.5 points per pull, max 30 points
    pTiming = fminf(30.0f, g_obd_timing_pull_count * 1.5f);
#endif
    
    // Calculate final score
    float score = 100.0f - (pST + pLT + pDelta + pTiming);
    if (score < 0.0f) score = 0.0f;
    if (score > 100.0f) score = 100.0f;

    // v6.10: publish the full breakdown so the CSV and the FUEL TRUST popup
    // can show exactly how the score was reached (transparency).
    g_ft.st1 = st1; g_ft.st2 = st2; g_ft.lt1 = lt1; g_ft.lt2 = lt2;
    g_ft.timing_deg = g_obd_data.timing_valid ? g_obd_data.timing_deg : NAN;
    g_ft.avgAbsST = avgAbsST; g_ft.avgAbsLT = avgAbsLT;
    g_ft.deltaST = deltaBanksST; g_ft.deltaLT = deltaBanksLT;
    g_ft.penST = pST; g_ft.penLT = pLT; g_ft.penBank = pDelta; g_ft.penTiming = pTiming;
    g_ft.timingPulls = (int)g_obd_timing_pull_count;
    g_ft.score = (int)(score + 0.5f);
    g_ft.valid = g_obd_data.fuel_trim_valid;

#if !FUEL_TRUST_TIMING_MODE
    // Periodic reset of timing pull counter (keeps display responsive)
    uint32_t now = millis();
    if (now - g_obd_timing_pull_reset_ms > FUEL_TRUST_TIMING_RESET_INTERVAL_MS) {
        g_obd_timing_pull_count = 0;
        g_obd_timing_pull_reset_ms = now;
    }
#endif

    return score;
}

//-----------------------------------------------------------------------------
// checkOBDDataStaleness() - Mark data invalid if not updated recently
//-----------------------------------------------------------------------------
static void checkOBDDataStaleness() {
    uint32_t now = millis();
    // v6.15: inside a power window the slow PIDs are deliberately not polled, so give them
    // a longer leash; the fast movers (rpm/timing/throttle/load/pedal) keep the normal one.
    const uint32_t slow_ms = g_pwr_window ? OBD_PID_STALE_THRESHOLD_POWER_MS : OBD_PID_STALE_THRESHOLD_MS;
    const uint32_t fast_ms = OBD_PID_STALE_THRESHOLD_MS;

    if (g_obd_data.coolant_valid &&
        (now - g_obd_data.coolant_timestamp > slow_ms)) {
        g_obd_data.coolant_valid = false;
    }

    if (g_obd_data.rpm_valid &&
        (now - g_obd_data.rpm_timestamp > fast_ms)) {
        g_obd_data.rpm_valid = false;
    }

    if (g_obd_data.speed_valid &&
        (now - g_obd_data.speed_timestamp > slow_ms)) {
        g_obd_data.speed_valid = false;
    }

    if (g_obd_data.timing_valid &&
        (now - g_obd_data.timing_timestamp > fast_ms)) {
        g_obd_data.timing_valid = false;
    }

    if (g_obd_data.fuel_trim_valid &&
        (now - g_obd_data.fuel_trim_timestamp > slow_ms)) {
        g_obd_data.fuel_trim_valid = false;
    }

    // v6.9 extended PIDs — independent staleness
    #define OBD_STALE(v, ts) if (g_obd_data.v && (now - g_obd_data.ts > slow_ms)) g_obd_data.v = false;
    #define OBD_STALE_FAST(v, ts) if (g_obd_data.v && (now - g_obd_data.ts > fast_ms)) g_obd_data.v = false;
    OBD_STALE_FAST(pedal_valid,     pedal_ts)
    #undef OBD_STALE_FAST
    OBD_STALE(throttle_valid,       throttle_ts)
    OBD_STALE(load_valid,           load_ts)
    OBD_STALE(iat_valid,            iat_ts)
    OBD_STALE(ambient_valid,        ambient_ts)
    OBD_STALE(oil_temp_valid,       oil_temp_ts)
    OBD_STALE(maf_valid,            maf_ts)
    OBD_STALE(lambda_valid,         lambda_ts)
    OBD_STALE(module_voltage_valid, module_voltage_ts)
    OBD_STALE(fuel_level_valid,     fuel_level_ts)
    OBD_STALE(baro_valid,           baro_ts)
    OBD_STALE(fuel_sys_valid,       fuel_sys_ts)
    #undef OBD_STALE
}

//=============================================================================
// v6.15: POWER WINDOW — when to switch the OBD scheduler into burst mode
//=============================================================================
// Open while the plate is >= 70% (PID 0x11) and rpm >= 3000, or the pedal is pinned
// (within 5 points of the session max, when a pedal source exists). Closes 1.5 s after
// the condition clears so a short lift between corners doesn't drop the burst list.
static void pwrUpdateWindow(uint32_t now) {
    bool cond = false;
    if (g_obd_data.rpm_valid && g_obd_data.rpm >= PWR_WINDOW_RPM_MIN) {
        if (g_obd_data.throttle_valid && g_obd_data.throttle_pct >= PWR_WINDOW_THROTTLE_PCT) cond = true;
        if (g_pwr.pedal_src != 0 && g_pwr.pedal_max_seen >= 60 &&
            g_pwr.pedal_pct >= g_pwr.pedal_max_seen - 5) cond = true;
    }
    if (cond) {
        g_pwr_window_cond_ms = now;
        if (!g_pwr_window) {
            g_pwr_window = true;
            g_obd_pwr_index = 0;
        }
    } else if (g_pwr_window && (now - g_pwr_window_cond_ms) > PWR_WINDOW_EXIT_MS) {
        g_pwr_window = false;
    }
}

//=============================================================================
// v6.15: TCM Mode 21 LID 0x20 — ATF temperatures + torque-converter slip
//=============================================================================
static void sendOBD_Mode21_TCM(uint8_t lid) {
    twai_message_t tx = {};
    tx.identifier = OBD_PHYS_TCM_REQ_ID;   // 0x7E1 (physical: the TCM)
    tx.extd = 0; tx.rtr = 0; tx.data_length_code = 8;
    tx.data[0] = 0x02;                     // 2 data bytes follow
    tx.data[1] = 0x21;                     // Mode 21 (ReadDataByLocalIdentifier)
    tx.data[2] = lid;
    g_obd_tx_total_count++;
    twai_transmit(&tx, pdMS_TO_TICKS(10));
    g_tcm_asm_active = false;
    g_tcm_asm_len = g_tcm_asm_got = 0;
}

// Reassembled record = [0x61][LID][d0][d1]... . Decode with the PROVISIONAL indices and
// dump the raw bytes every 30 s so the real offsets can be confirmed on the car.
static void finalizeTCMRecord() {
    g_tcm_reply_count++;
    uint32_t now = millis();
    int need = TCM_TCC_IDX + 2;
    if (g_tcm_asm_got >= need && g_tcm_asm_buf[0] == 0x61 && g_tcm_asm_buf[1] == TCM_LID_TRANS_DATA) {
        int a1 = (int)g_tcm_asm_buf[TCM_ATF1_IDX] - TCM_ATF_OFFSET_C;
        int a2 = (int)g_tcm_asm_buf[TCM_ATF2_IDX] - TCM_ATF_OFFSET_C;
        int slip = ((int)g_tcm_asm_buf[TCM_TCC_IDX] << 8) | g_tcm_asm_buf[TCM_TCC_IDX + 1];
        // plausibility: ATF between -40 and 180 C, slip below 5000 rpm
        if (a1 > -40 && a1 < 180 && a2 > -40 && a2 < 180 && slip >= 0 && slip < 5000) {
            g_tcm_atf1_c = a1; g_tcm_atf2_c = a2; g_tcm_tcc_slip = slip;
            g_tcm_valid = true; g_tcm_ts = now;
        }
    }
    if (now - g_tcm_last_dump_ms > 30000 || g_tcm_reply_count <= 3) {
        g_tcm_last_dump_ms = now;
        char hex[TCM_ASM_BUF_LEN * 3 + 1]; int m = 0;
        for (int i = 0; i < g_tcm_asm_got && m < (int)sizeof(hex) - 3; i++)
            m += snprintf(hex + m, sizeof(hex) - m, "%02X ", g_tcm_asm_buf[i]);
        Serial.printf("[TCM] LID 0x%02X record (%d bytes): %s| provisional ATF1=%dC ATF2=%dC slip=%d%s\n",
                      TCM_LID_TRANS_DATA, g_tcm_asm_got, hex, g_tcm_atf1_c, g_tcm_atf2_c, g_tcm_tcc_slip,
                      g_tcm_valid ? "" : " (not plausible yet)");
    }
}

// Feed a 0x7E9 frame into the TCM reassembler. Returns true if consumed.
static bool handleTCMFrame(const twai_message_t& rx) {
    if (rx.data_length_code < 2) return false;
    uint8_t pci = rx.data[0] & 0xF0;
    if (pci == 0x00) {                                  // single frame
        if (rx.data[1] != 0x61) return false;
        int len = rx.data[0] & 0x0F;
        g_tcm_asm_got = 0;
        for (int i = 0; i < len && g_tcm_asm_got < TCM_ASM_BUF_LEN && (1 + i) < 8; i++)
            g_tcm_asm_buf[g_tcm_asm_got++] = rx.data[1 + i];
        g_tcm_asm_active = false;
        finalizeTCMRecord();
        return true;
    }
    if (pci == 0x10) {                                  // first frame
        if (rx.data_length_code < 3 || rx.data[2] != 0x61) return false;
        g_tcm_asm_len = ((rx.data[0] & 0x0F) << 8) | rx.data[1];
        if (g_tcm_asm_len > TCM_ASM_BUF_LEN) g_tcm_asm_len = TCM_ASM_BUF_LEN;
        g_tcm_asm_got = 0;
        for (int i = 0; i < 6 && g_tcm_asm_got < g_tcm_asm_len; i++)
            g_tcm_asm_buf[g_tcm_asm_got++] = rx.data[2 + i];
        g_tcm_asm_active = true;
        g_tcm_asm_start_ms = millis();
        sendFlowControl(OBD_PHYS_TCM_REQ_ID);
        return true;
    }
    if (pci == 0x20) {                                  // consecutive frame
        if (!g_tcm_asm_active) return false;
        for (int i = 1; i < 8 && g_tcm_asm_got < g_tcm_asm_len; i++)
            g_tcm_asm_buf[g_tcm_asm_got++] = rx.data[i];
        if (g_tcm_asm_got >= g_tcm_asm_len) {
            g_tcm_asm_active = false;
            finalizeTCMRecord();
        }
        return true;
    }
    return false;
}

//=============================================================================
// v6.15: DISCOVERY — supported-PID bitmaps at boot, optional Mode 22 DID sweep
//=============================================================================
static void obdDiscoveryStep(uint32_t now) {
    if (g_disc_next_block > 4) return;                          // done
    // first request ~3 s after CAN comes up (ECU settles), then one request per 700 ms
    const uint32_t wait_ms = (g_disc_next_block == 0 && g_disc_retries == 0) ? 3000 : 700;
    if (now - g_disc_last_req_ms < wait_ms) return;             // one request, wait for the answer
    if (g_disc_retries >= 3) {                                  // ECU silent for this block: give up on it
        Serial.printf("[OBD-DISC] no answer for bitmap block %d after 3 tries\n", g_disc_next_block);
        g_disc_next_block = 5;
        return;
    }
    g_disc_last_req_ms = now;
    g_disc_retries++;
    sendOBD_Mode01((uint8_t)(g_disc_next_block * 0x20));
}

static void didSweepStep(uint32_t now) {
    if (!g_did_sweep_enabled || g_did_sweep_done) return;
    if (!g_did_sweep_running) {
        // start only at a warm idle, 20 s after boot, outside a power window
        if (now < 20000 || g_pwr_window) return;
        if (!g_obd_data.rpm_valid || g_obd_data.rpm < 300 || g_obd_data.rpm > 1200) return;
        g_did_sweep_running = true;
        g_did_sweep_next = 0x1100;
        g_did_sweep_hits = 0;
        Serial.println("[DID] Mode 22 sweep 0x1100-0x12FF starting (one request / 60 ms, ~30 s)");
    }
    if (now - g_did_sweep_last_ms < 60) return;
    g_did_sweep_last_ms = now;
    if (g_did_sweep_next > 0x12FF) {
        g_did_sweep_running = false; g_did_sweep_done = true;
        Serial.printf("[DID] sweep done: %d DIDs answered (see [DID] lines above)\n", g_did_sweep_hits);
        return;
    }
    if (g_did_sweep_next != OBD_DID_OIL_TEMP) sendOBD_Mode22(g_did_sweep_next);
    g_did_sweep_next++;
}

// A positive Mode 22 reply for a DID we don't decode (sweep hit) — log the bytes.
static void logSweepDID(const twai_message_t& rx) {
    if (!g_did_sweep_running) return;
    g_did_sweep_hits++;
    char hex[40]; int m = 0;
    for (int i = 0; i < rx.data_length_code && m < (int)sizeof(hex) - 3; i++)
        m += snprintf(hex + m, sizeof(hex) - m, "%02X ", rx.data[i]);
    Serial.printf("[DID] %s\n", hex);   // frame as received: [len][62][DIDhi][DIDlo][A][B]... or [10][len][62]... for multi-frame
}

//=============================================================================
// v6.15: PASSIVE CAN — frames the car broadcasts anyway
//=============================================================================
static void canCensusAdd(uint16_t id) {
    for (int i = 0; i < g_can_census_n; i++)
        if (g_can_census[i].id == id) { g_can_census[i].count++; return; }
    if (g_can_census_n < CAN_CENSUS_SLOTS) { g_can_census[g_can_census_n].id = id; g_can_census[g_can_census_n].count = 1; g_can_census_n++; }
}

static void handlePassiveCanFrame(const twai_message_t& rx, uint32_t now) {
    if (rx.extd) return;
    if (!g_can_census_printed) canCensusAdd((uint16_t)rx.identifier);
    switch (rx.identifier) {
        case CAN_ID_ENGINE_180:
            if (rx.data_length_code >= 6) {
                g_can180.rpm_raw   = ((uint16_t)rx.data[0] << 8) | rx.data[1];
                g_can180.pedal_raw = rx.data[5];
                g_can180.ts = now; g_can180.count++;
            }
            break;
        case CAN_ID_ECT_551:
            if (rx.data_length_code >= 1) { g_can180.ect_raw = rx.data[0]; g_can180.ect_ts = now; }
            break;
        case CAN_ID_VDC_354:
            if (rx.data_length_code >= 7) { g_can180.vdc_e = rx.data[4]; g_can180.vdc_g = rx.data[6]; g_can180.vdc_ts = now; }
            break;
        default: break;
    }
}

// One-shot ID census ~30 s after CAN came up: which IDs are on this bus and how busy.
static void canCensusStep(uint32_t now) {
    if (g_can_census_printed) return;
    if (g_can_census_start_ms == 0) { g_can_census_start_ms = now; return; }
    if (now - g_can_census_start_ms < 30000) return;
    g_can_census_printed = true;
    // sort by count (tiny insertion sort)
    for (int i = 1; i < g_can_census_n; i++) {
        auto t = g_can_census[i]; int j = i - 1;
        while (j >= 0 && g_can_census[j].count < t.count) { g_can_census[j + 1] = g_can_census[j]; j--; }
        g_can_census[j + 1] = t;
    }
    char line[CAN_CENSUS_SLOTS * 14 + 1]; int m = 0;
    for (int i = 0; i < g_can_census_n && m < (int)sizeof(line) - 14; i++)
        m += snprintf(line + m, sizeof(line) - m, "%03X:%lu ", g_can_census[i].id, (unsigned long)(g_can_census[i].count / 30));
    Serial.printf("[CAN-DISC] %d IDs heard in 30 s (id:frames/s): %s\n", g_can_census_n, line);
    Serial.printf("[CAN-DISC] 0x180 %s, 0x551 %s, 0x354 %s\n",
                  g_can180.count ? "heard (pedal/rpm source available)" : "NOT heard",
                  g_can180.ect_ts ? "heard" : "NOT heard", g_can180.vdc_ts ? "heard" : "NOT heard");
}

#endif // ENABLE_OBD_CAN

//-----------------------------------------------------------------------------
// initOBD() - Initialize OBD-II CAN communication
//-----------------------------------------------------------------------------
void initOBD() {
#if ENABLE_OBD_CAN
    Serial.println("[OBD] Initializing CAN bus...");
    Serial.printf("[OBD] Config: TX=GPIO%d, RX=GPIO%d, 500kbps\n", CAN_TX_PIN, CAN_RX_PIN);
    
    if (startCAN()) {
        g_obd_initialized = true;
        g_obd_last_request_ms = millis();
        Serial.println("[OBD] CAN bus initialized successfully");
        Serial.println("[OBD] Polling: ECT, RPM, Speed, Timing, STFT/LTFT B1/B2,");
        Serial.println("[OBD]   +v6.9: Throttle, Load, IAT, Ambient, OilTemp(0x5C), MAF, Lambda, Voltage, FuelLevel, Baro, FuelSys");
        Serial.println("[OBD]   +v6.15: Pedal(0x49), power-window burst list (timing ~8 Hz at WOT), TCM Mode 21 ATF, PID bitmaps, passive 0x180/0x551/0x354");
        g_disc_next_block = 0; g_disc_retries = 0; g_disc_last_req_ms = millis();   // v6.15: bitmaps ~3 s after CAN is up
        g_can_census_start_ms = 0; g_can_census_printed = false; g_can_census_n = 0;
    } else {
        g_obd_initialized = false;
        Serial.println("[OBD] CAN bus initialization FAILED!");
        Serial.println("[OBD] Check: TX/RX wiring, transceiver power, OBD-II connection");
    }
#else
    Serial.println("[OBD] OBD-II CAN disabled (ENABLE_OBD_CAN=0)");
#endif
}

//-----------------------------------------------------------------------------
// updateOBDData() - Poll PIDs and update vehicle data
//-----------------------------------------------------------------------------
// Called from main loop. Performs:
//   1. Round-robin PID requests (one per OBD_PID_REQUEST_PERIOD_MS)
//   2. Receives and decodes any pending CAN frames
//   3. Updates g_vehicle_data with water temp and fuel trust
//   4. Marks stale data as invalid
//-----------------------------------------------------------------------------
void updateOBDData() {
#if ENABLE_OBD_CAN
    if (!g_obd_initialized) {
        // Try to reinitialize periodically
        static uint32_t lastRetry = 0;
        if (millis() - lastRetry > 5000) {
            lastRetry = millis();
            initOBD();
        }
        g_vehicle_data.water_temp_valid = false;
        g_vehicle_data.fuel_trust_valid = false;
        g_vehicle_data.rpm_valid = false;
        return;
    }
    
    uint32_t now = millis();

    // 1. Send next PID request (round-robin).
    //    v6.15: inside a POWER WINDOW the short burst list runs at 60 ms (timing ~8 Hz);
    //    otherwise the full list runs at 120 ms exactly as before.
    {
        const uint32_t period = g_pwr_window ? OBD_PID_REQUEST_PERIOD_POWER_MS : OBD_PID_REQUEST_PERIOD_MS;
        if (now - g_obd_last_request_ms >= period) {
            g_obd_last_request_ms = now;
            uint8_t pid;
            if (g_pwr_window) {
                pid = OBD_PID_LIST_POWER[g_obd_pwr_index];
                g_obd_pwr_index = (g_obd_pwr_index + 1) % OBD_NUM_PIDS_POWER;
            } else {
                pid = OBD_PID_LIST[g_obd_pid_index];
                g_obd_pid_index = (g_obd_pid_index + 1) % OBD_NUM_PIDS;
            }
            // pedal PID skipped once the bitmap says the ECU doesn't have it (CAN 0x180 covers it)
            if (pid == 0x49 && !g_pid49_supported) pid = 0x0C;
            sendOBD_Mode01(pid);
        }
    }

    // 1b. v6.10: periodically read stored trouble codes (Mode 03), and give up
    //     on any ISO-TP reassembly that stalls (benign — leaves the last list).
    //     v6.15: housekeeping requests (DTC, TCM, discovery, DID sweep) stay out of power
    //     windows so the burst budget is all timing/rpm.
    if (!g_pwr_window && now - g_dtc_last_read_ms >= DTC_READ_INTERVAL_MS) {
        g_dtc_last_read_ms = now;
        sendOBD_Mode03();
    }

    // v6.14: poll Nissan Mode 22 oil temp (~1 Hz). Generic PID 0x5C is unsupported on
    // the VQ37, so this is the working oil-temp source over OBD.
    static uint32_t lastMode22 = 0;
    if (now - lastMode22 >= 1000) {
        lastMode22 = now;
        sendOBD_Mode22(OBD_DID_OIL_TEMP);
    }
    if (g_dtc_asm_active && (now - g_dtc_asm_start_ms > DTC_ASM_TIMEOUT_MS)) {
        g_dtc_asm_active = false;
    }

    // v6.15: TCM ATF/TCC record (~1 Hz), supported-PID bitmaps (boot), optional DID sweep
    if (!g_pwr_window) {
        if (now - g_tcm_last_poll_ms >= TCM_POLL_INTERVAL_MS) {
            g_tcm_last_poll_ms = now;
            sendOBD_Mode21_TCM(TCM_LID_TRANS_DATA);
        }
        obdDiscoveryStep(now);
        didSweepStep(now);
    }
    if (g_tcm_asm_active && (now - g_tcm_asm_start_ms > DTC_ASM_TIMEOUT_MS)) {
        g_tcm_asm_active = false;
    }

    // 2. Receive any pending CAN frames (non-blocking)
    twai_message_t rx;
    while (twai_receive(&rx, pdMS_TO_TICKS(1)) == ESP_OK) {
        if (rx.extd) continue;
        if (rx.identifier == OBD_TCM_RESP_ID) {
            // v6.15: TCM (0x7E9): Mode 21 record first, else its Mode 03 DTC reply
            if (!handleTCMFrame(rx)) handleDTCFrame(rx);
        }
        else if (rx.identifier >= OBD_MIN_RESP_ID && rx.identifier <= OBD_MAX_RESP_ID) {
            // Filter for OBD-II responses (0x7E8-0x7EF)
            // Mode 03 (DTC) frames are ISO-TP framed; route them to the reassembler
            // first, then Mode 22 (0x62) responses, else it's a Mode 01 reply.
            if (handleDTCFrame(rx)) {
                // consumed as DTC data
            } else if (rx.data_length_code >= 2 && rx.data[1] == 0x62) {
                decodeOBD_Mode22Reply(rx);   // v6.14: Mode 22 (Nissan oil temp)
                if (rx.data_length_code >= 4 && ((((uint16_t)rx.data[2] << 8) | rx.data[3]) != OBD_DID_OIL_TEMP))
                    logSweepDID(rx);         // v6.15: any other DID answering = sweep hit
            } else if (rx.data_length_code >= 3 && rx.data[0] == 0x10 && rx.data[2] == 0x62) {
                logSweepDID(rx);             // v6.15: multi-frame Mode 22 reply (first frame only)
            } else if (rx.data_length_code >= 2 && rx.data[1] == 0x7F) {
                // negative response (e.g. 7F 22 31 = DID not supported) — expected during the sweep
            } else {
                decodeOBD_Mode01Reply(rx);
            }
        }
        else {
            handlePassiveCanFrame(rx, now);   // v6.15: 0x180 pedal/rpm, 0x551 ECT, 0x354 VDC, ID census
        }
    }
    canCensusStep(now);

    // 3. Check for stale data
    checkOBDDataStaleness();

    // 3b. v6.15: pedal source (PID 0x49 first, else the 100 Hz CAN 0x180 byte) and the
    //     power-window flag that steers the scheduler on the next call.
    {
        int pct = 0, src = 0;
        if (g_obd_data.pedal_valid) { pct = g_obd_data.pedal_pct; src = 1; }
        else if (g_can180.ts && (now - g_can180.ts) < CAN_PEDAL_STALE_MS) {
            pct = (int)((float)g_can180.pedal_raw * 100.0f / 255.0f + 0.5f); src = 2;
        }
        g_pwr.pedal_pct = pct; g_pwr.pedal_src = src;
        if (src && pct > g_pwr.pedal_max_seen) g_pwr.pedal_max_seen = pct;
        pwrUpdateWindow(now);
    }
    
    // 4. Update Water Temperature from ECT
    if (g_obd_data.coolant_valid) {
        // Convert Celsius to Fahrenheit for internal storage
        float water_temp_f = celsiusToFahrenheit((float)g_obd_data.coolant_c);
        g_vehicle_data.water_temp_value_f = (int)(water_temp_f + 0.5f);
        g_vehicle_data.water_temp_valid = true;
        g_vehicle_data.has_received_data = true;
    } else {
        g_vehicle_data.water_temp_valid = false;
    }
    
    // 5. Update RPM
    if (g_obd_data.rpm_valid) {
        g_vehicle_data.rpm = (int)(g_obd_data.rpm + 0.5f);
        g_vehicle_data.rpm_valid = true;
    } else {
        g_vehicle_data.rpm_valid = false;
    }
    
    // 6. Calculate and update Fuel Trust
    // Only show Fuel Trust when engine is warmed up (ECT >= 80°C)
    bool show_fuel_trust = g_obd_data.coolant_valid && 
                           g_obd_data.coolant_c >= OBD_WARMUP_TEMP_C;
    
    if (show_fuel_trust && g_obd_data.fuel_trim_valid) {
        float trust = computeFuelTrust();
        g_vehicle_data.fuel_trust_percent = (int)(trust + 0.5f);
        g_vehicle_data.fuel_trust_valid = true;
        g_vehicle_data.has_received_data = true;
    } else {
        g_vehicle_data.fuel_trust_valid = false;
    }

    // 6b. v6.9 extended OBD PIDs -> vehicle data (for CSV logging). Any PID the ECU
    //     doesn't answer simply stays invalid. Temperatures convert °C -> °F.
    #define OBD_COPY(dst, dvalid, src, svalid)  do { \
        if (g_obd_data.svalid) { g_vehicle_data.dst = g_obd_data.src; g_vehicle_data.dvalid = true; } \
        else { g_vehicle_data.dvalid = false; } } while(0)
    OBD_COPY(throttle_pct,     throttle_valid,        throttle_pct,   throttle_valid);
    OBD_COPY(engine_load_pct,  engine_load_valid,     load_pct,       load_valid);
    OBD_COPY(maf_gps,          maf_valid,             maf_gps,        maf_valid);
    OBD_COPY(commanded_lambda, lambda_valid,          lambda,         lambda_valid);
    OBD_COPY(module_voltage,   module_voltage_valid,  module_voltage, module_voltage_valid);
    OBD_COPY(fuel_level_pct,   fuel_level_valid,      fuel_level_pct, fuel_level_valid);
    OBD_COPY(baro_kpa,         baro_valid,            baro_kpa,       baro_valid);
    OBD_COPY(fuel_sys_status,  fuel_sys_valid,        fuel_sys_status,fuel_sys_valid);
    #undef OBD_COPY
    if (g_obd_data.iat_valid) {
        g_vehicle_data.intake_air_temp_f = (int)(celsiusToFahrenheit((float)g_obd_data.iat_c) + 0.5f);
        g_vehicle_data.intake_air_temp_valid = true;
    } else g_vehicle_data.intake_air_temp_valid = false;
    if (g_obd_data.ambient_valid) {
        g_vehicle_data.ambient_temp_f = (int)(celsiusToFahrenheit((float)g_obd_data.ambient_c) + 0.5f);
        g_vehicle_data.ambient_temp_valid = true;
    } else g_vehicle_data.ambient_temp_valid = false;
    if (g_obd_data.oil_temp_valid) {
        g_vehicle_data.obd_oil_temp_f = (int)(celsiusToFahrenheit((float)g_obd_data.oil_temp_c) + 0.5f);
        g_vehicle_data.obd_oil_temp_valid = true;
    } else g_vehicle_data.obd_oil_temp_valid = false;

    // 6c. v6.10: copy fuel-trim, score-breakdown, and DTC status for CSV logging.
    //     Raw trims are copied whenever valid (even before warm-up gating), so the
    //     log captures the relearn even while the on-screen Fuel Trust is hidden.
    g_vehicle_data.fuel_trim_valid = g_obd_data.fuel_trim_valid;
    if (g_obd_data.fuel_trim_valid) {
        g_vehicle_data.stft_b1 = g_obd_data.stft_b1;
        g_vehicle_data.stft_b2 = g_obd_data.stft_b2;
        g_vehicle_data.ltft_b1 = g_obd_data.ltft_b1;
        g_vehicle_data.ltft_b2 = g_obd_data.ltft_b2;
    }
    g_vehicle_data.ft_timing_deg = g_obd_data.timing_valid ? g_obd_data.timing_deg : 0.0f;
    g_vehicle_data.ft_pen_stft   = g_ft.penST;      // 0 unless computeFuelTrust ran this cycle
    g_vehicle_data.ft_pen_ltft   = g_ft.penLT;
    g_vehicle_data.ft_pen_bank   = g_ft.penBank;
    g_vehicle_data.ft_pen_timing = g_ft.penTiming;
    g_vehicle_data.mil_on        = g_mil_on;
    g_vehicle_data.dtc_count     = g_dtc01_valid ? g_dtc_count_01 : -1;
    g_vehicle_data.dtc_valid     = g_dtc01_valid;

    // 6d. v6.15: pedal + TCM values for the CSV (the POWER fields are filled by updatePowerMonitor)
    g_vehicle_data.pedal_pct = g_pwr.pedal_pct;
    g_vehicle_data.pedal_src = g_pwr.pedal_src;
    if (g_tcm_valid && (now - g_tcm_ts) < 5000) {
        g_vehicle_data.atf1_f = (int)(celsiusToFahrenheit((float)g_tcm_atf1_c) + 0.5f);
        g_vehicle_data.atf2_f = (int)(celsiusToFahrenheit((float)g_tcm_atf2_c) + 0.5f);
        g_vehicle_data.tcc_slip_rpm = g_tcm_tcc_slip;
        g_vehicle_data.atf_valid = true;
    } else {
        g_vehicle_data.atf_valid = false;
    }

    // 7. Periodic logging (ALWAYS logs status, even when no ECU data received)
    static uint32_t lastOBDLog = 0;
    if (now - lastOBDLog > 5000) {  // Log every 5 seconds
        lastOBDLog = now;
        
        if (g_obd_data.coolant_valid || g_obd_data.rpm_valid || g_obd_data.fuel_trim_valid) {
            Serial.printf("[OBD] ECT:%d°C RPM:%.0f Spd:%dkph Tim:%.1f° ST1:%.1f%% LT1:%.1f%% ST2:%.1f%% LT2:%.1f%% Trust:%d%%\n",
                g_obd_data.coolant_valid ? g_obd_data.coolant_c : -99,
                g_obd_data.rpm_valid ? g_obd_data.rpm : 0,
                g_obd_data.speed_valid ? g_obd_data.speed_kph : -1,
                g_obd_data.timing_valid ? g_obd_data.timing_deg : 0,
                g_obd_data.fuel_trim_valid ? g_obd_data.stft_b1 : 0,
                g_obd_data.fuel_trim_valid ? g_obd_data.ltft_b1 : 0,
                g_obd_data.fuel_trim_valid ? g_obd_data.stft_b2 : 0,   // v6.10: Bank 2 restored to the log
                g_obd_data.fuel_trim_valid ? g_obd_data.ltft_b2 : 0,
                g_vehicle_data.fuel_trust_valid ? g_vehicle_data.fuel_trust_percent : -1);
            // v6.9 extended PIDs. A real value = ECU supports it; -1 = unsupported/no-response yet.
            Serial.printf("[OBD2] Thr:%d%% Load:%d%% IATf:%d Ambf:%d OilTf:%d MAF:%.1f Lam:%.3f Volt:%.2f Fuel:%d%% Baro:%d FSys:0x%02X\n",
                g_obd_data.throttle_valid      ? g_obd_data.throttle_pct : -1,
                g_obd_data.load_valid          ? g_obd_data.load_pct : -1,
                g_obd_data.iat_valid           ? (int)(celsiusToFahrenheit((float)g_obd_data.iat_c) + 0.5f) : -1,
                g_obd_data.ambient_valid       ? (int)(celsiusToFahrenheit((float)g_obd_data.ambient_c) + 0.5f) : -1,
                g_obd_data.oil_temp_valid      ? (int)(celsiusToFahrenheit((float)g_obd_data.oil_temp_c) + 0.5f) : -1,
                g_obd_data.maf_valid           ? g_obd_data.maf_gps : -1.0f,
                g_obd_data.lambda_valid        ? g_obd_data.lambda : -1.0f,
                g_obd_data.module_voltage_valid? g_obd_data.module_voltage : -1.0f,
                g_obd_data.fuel_level_valid    ? g_obd_data.fuel_level_pct : -1,
                g_obd_data.baro_valid          ? g_obd_data.baro_kpa : -1,
                g_obd_data.fuel_sys_valid      ? g_obd_data.fuel_sys_status : 0);
            // v6.15: pedal source + raw passive-CAN words for on-car scale validation
            // (compare rpmRaw180 with RPM above, pedalRaw180/255 with the pedal PID, ect551-40 with ECT).
            Serial.printf("[CAN180] pedal:%d%%(src %d) frames:%lu rpmRaw180:%u pedalRaw180:%u ect551:%d vdcE:%u vdcG:%u window:%d ATF:%d/%dC slip:%d\n",
                g_pwr.pedal_pct, g_pwr.pedal_src, (unsigned long)g_can180.count,
                g_can180.rpm_raw, g_can180.pedal_raw,
                g_can180.ect_ts ? (int)g_can180.ect_raw - 40 : -99,
                g_can180.vdc_e, g_can180.vdc_g, g_pwr_window ? 1 : 0,
                g_tcm_valid ? g_tcm_atf1_c : -99, g_tcm_valid ? g_tcm_atf2_c : -99, g_tcm_valid ? g_tcm_tcc_slip : -1);
        } else {
            // No valid data from ECU - log bus state so SD logs show OBD is alive but silent
            Serial.printf("[OBD] CAN active, no ECU response (TX=GPIO%d RX=GPIO%d) pid_idx=%d\n",
                CAN_TX_PIN, CAN_RX_PIN, g_obd_pid_index);
        }
    }
    
#else
    // OBD disabled - mark data invalid
    g_vehicle_data.water_temp_valid = false;
    g_vehicle_data.fuel_trust_valid = false;
    g_vehicle_data.rpm_valid = false;
#endif
}

#pragma endregion OBD Data Provider

//=================================================================
// v6.15 HEAT DERATE MONITOR ("POWER")
//=================================================================
// Answers "is the ECU pulling power because the car is hot?" from the OBD tap alone.
//
//   1. Power windows (pwrUpdateWindow, OBD section) make timing measurable at WOT (~8 Hz).
//   2. A cool baseline of WOT timing / load / pedal-gap per 500-rpm bin is learned on the
//      first warm-but-cool laps and kept in NVS ("timbase").
//   3. Every later WOT timing sample is compared to its bin -> tim_delta. rpm is
//      extrapolated to the timing sample's timestamp so low-gear pulls bin correctly.
//   4. Detectors: TIMING (delta), THROTTLE (pedal-vs-plate gap vs cool), LIFT (load below
//      cool), REV CAP (rpm plateau < 7000 at WOT while oil is hot / limiter-jagged),
//      FUEL? (rpm stumble that recovers, in a corner), AIR (SAE J1349 density loss).
//   5. Worst active state -> banner + popup + CSV + [PWR] / [SESSION] lines.
//
// Everything is time-qualified (must persist) and latched (stays up long enough to read).
// Runs from loop() after updateOBDData()/mergeOilTempSource(), i.e. with the merged oil temp.
//=================================================================
#pragma region Heat Derate Monitor
#if ENABLE_OBD_CAN

static void pwrLoadBaseline() {
    g_pwr.bin = -1;
    g_pwr.sess_min_delta = 0.0f;
    memset(&g_pwr_base, 0, sizeof(g_pwr_base));
    g_prefs.begin("timbase", true);
    size_t len = g_prefs.getBytesLength("bins");
    if (len == sizeof(PwrBaseline)) g_prefs.getBytes("bins", &g_pwr_base, sizeof(g_pwr_base));
    g_prefs.end();
    if (g_pwr_base.version != PWR_BASELINE_VERSION) {
        memset(&g_pwr_base, 0, sizeof(g_pwr_base));
        g_pwr_base.version = PWR_BASELINE_VERSION;
        Serial.println("[PWR] baseline: none stored yet (learns on the first cool WOT pulls)");
        return;
    }
    int ready = 0; char bins[PWR_RPM_BINS * 12 + 1]; int m = 0;
    for (int i = 0; i < PWR_RPM_BINS; i++) {
        if (g_pwr_base.bin[i].n >= PWR_BIN_MIN_N) {
            ready++;
            m += snprintf(bins + m, sizeof(bins) - m, "%d:%.1f ", PWR_RPM_BIN_MIN + i * PWR_RPM_BIN_SIZE, g_pwr_base.bin[i].tim);
        }
    }
    Serial.printf("[PWR] baseline loaded from NVS: %d/%d bins ready (rpm:timing) %s\n", ready, PWR_RPM_BINS, m ? bins : "");
}

static void pwrSaveBaseline(bool force) {
    uint32_t now = millis();
    if (!force) {
        if (!g_pwr_base_dirty) return;
        if (now - g_pwr_base_saved_ms < PWR_BASELINE_SAVE_MS) return;
    }
    g_prefs.begin("timbase", false);
    g_prefs.putBytes("bins", &g_pwr_base, sizeof(g_pwr_base));
    g_prefs.end();
    g_pwr_base_dirty = false;
    g_pwr_base_saved_ms = now;
    Serial.println("[PWR] baseline saved to NVS");
}

static void pwrResetBaseline(const char* why) {
    memset(&g_pwr_base, 0, sizeof(g_pwr_base));
    g_pwr_base.version = PWR_BASELINE_VERSION;
    g_pwr.delta_valid = false; g_pwr.tim_delta = 0; g_pwr.tim_base = 0; g_pwr.bin_n = 0; g_pwr.bin = -1;
    g_pwr.gap_delta = 0; g_pwr.load_delta = 0;
    g_pwr.tim_bad_since = 0; g_pwr.gap_bad_since = 0; g_pwr.load_bad_since = 0;
    pwrSaveBaseline(true);
    Serial.printf("[PWR] baseline RESET (%s)\n", why);
}

// SAE J1349 correction factor from inlet temp + absolute pressure (dry-air approximation:
// no humidity sensor yet). Loss% = 1 - 1/cf. 77F/990mb reference -> 0%.
static float pwrAirLoss(int iat_f, int baro_kpa, float* cf_out) {
    float Tc = ((float)iat_f - 32.0f) * 5.0f / 9.0f;
    float Pd = (float)baro_kpa * 10.0f;                       // kPa -> mbar
    if (Pd < 600.0f) Pd = 600.0f;
    float cf = 1.18f * (990.0f / Pd) * sqrtf((Tc + 273.0f) / 298.0f) - 0.18f;
    if (cf < 0.5f) cf = 0.5f;
    if (cf_out) *cf_out = cf;
    float loss = (1.0f - 1.0f / cf) * 100.0f;
    if (loss < -20.0f) loss = -20.0f;
    if (loss >  30.0f) loss =  30.0f;
    return loss;
}

// Which temperature "explains" a timing pull right now (for the reason string).
static const char* pwrAttribution(int oil_f, int iat_f, int ect_f) {
    if (oil_f >= PWR_REVCAP_OIL_F) return "OIL";
    if (iat_f >= PWR_COOL_IAT_MAX_F) return "IAT";
    if (ect_f >= 215) return "ECT";
    return "no temp excuse (knock?)";
}

static void updatePowerMonitor() {
    uint32_t now = millis();
    if (g_demo_mode || !g_obd_initialized) {
        g_pwr.state = PWR_OK; g_pwr.sev = 0; g_pwr.reason[0] = '\0';
        g_vehicle_data.pwr_state = 0; g_vehicle_data.pwr_sev = 0;
        return;
    }
    if (g_pwr_baseline_reset_req) { g_pwr_baseline_reset_req = false; pwrResetBaseline("PWR_BASELINE_RESET in /wifi.cfg"); }

    // ---- temperatures the detectors reason about (merged oil temp = PRTXI or Mode 22) ----
    const int ect_f = g_vehicle_data.water_temp_valid      ? g_vehicle_data.water_temp_value_f : -1;
    const int iat_f = g_vehicle_data.intake_air_temp_valid ? g_vehicle_data.intake_air_temp_f  : -1;
    const int oil_f = g_vehicle_data.oil_temp_valid        ? g_vehicle_data.oil_temp_value_f   : -1;

    // ---- AIR: density loss from IAT + baro ----
    if (iat_f >= 0 && g_vehicle_data.baro_valid && g_vehicle_data.baro_kpa > 0) {
        g_pwr.air_loss_pct = pwrAirLoss(iat_f, g_vehicle_data.baro_kpa, &g_pwr.air_cf);
    } else { g_pwr.air_loss_pct = 0.0f; g_pwr.air_cf = 1.0f; }

    // ---- new WOT timing sample? ----
    if (g_obd_data.timing_valid && g_obd_data.timing_timestamp != g_pwr.last_tim_ts) {
        const uint32_t ts = g_obd_data.timing_timestamp;
        g_pwr.last_tim_ts = ts;

        bool rpm_ok = g_obd_data.rpm_valid && (uint32_t)labs((long)ts - (long)g_obd_data.rpm_timestamp) <= PWR_ALIGN_RPM_MS;
        bool thr_ok = g_obd_data.throttle_valid && (uint32_t)labs((long)ts - (long)g_obd_data.throttle_ts) <= PWR_ALIGN_LOAD_MS;
        bool load_ok = g_obd_data.load_valid && (uint32_t)labs((long)ts - (long)g_obd_data.load_ts) <= PWR_ALIGN_LOAD_MS;
        bool wot = thr_ok && load_ok &&
                   g_obd_data.throttle_pct >= PWR_WINDOW_THROTTLE_PCT && g_obd_data.load_pct >= PWR_WOT_LOAD_PCT;

        if (rpm_ok && wot) {
            // extrapolate rpm to the timing sample's timestamp using the last two rpm samples
            float rpm_at = g_obd_data.rpm;
            if (g_obd_data.rpm_prev_ts && g_obd_data.rpm_timestamp > g_obd_data.rpm_prev_ts) {
                uint32_t dtp = g_obd_data.rpm_timestamp - g_obd_data.rpm_prev_ts;
                if (dtp >= 50 && dtp <= 1500) {
                    float rate = (g_obd_data.rpm - g_obd_data.rpm_prev) / (float)dtp;      // rpm per ms
                    float corr = rate * ((float)ts - (float)g_obd_data.rpm_timestamp);
                    if (corr >  600.0f) corr =  600.0f;
                    if (corr < -600.0f) corr = -600.0f;
                    rpm_at += corr;
                }
            }
            int bin = (int)((rpm_at - PWR_RPM_BIN_MIN) / PWR_RPM_BIN_SIZE);
            if (rpm_at >= PWR_RPM_BIN_MIN && bin >= 0 && bin < PWR_RPM_BINS) {
                PwrBin& b = g_pwr_base.bin[bin];
                const float tim = g_obd_data.timing_deg;
                const float load = (float)g_obd_data.load_pct;
                const bool pedal_pinned = g_pwr.pedal_src != 0 && g_pwr.pedal_max_seen >= 60 &&
                                          g_pwr.pedal_pct >= g_pwr.pedal_max_seen - 5;
                const float gap = (float)(g_pwr.pedal_pct - g_obd_data.throttle_pct);

                // cool? (warm engine, but none of the heat drivers elevated)
                const bool cool = ect_f >= PWR_COOL_ECT_MIN_F && ect_f <= PWR_COOL_ECT_MAX_F &&
                                  (iat_f < 0 || iat_f < PWR_COOL_IAT_MAX_F) &&
                                  (oil_f < 0 || oil_f < PWR_COOL_OIL_MAX_F);
                if (cool) {
                    float a = (b.n < PWR_BIN_MIN_N) ? 1.0f / (float)(b.n + 1) : PWR_EMA_ALPHA;   // fast start, then EMA
                    b.tim  = (b.n == 0) ? tim    : b.tim  + a * (tim    - b.tim);
                    b.load = (b.n == 0) ? load   : b.load + a * (load   - b.load);
                    b.rpm  = (b.n == 0) ? rpm_at : b.rpm  + a * (rpm_at - b.rpm);   // mean rpm for slope interpolation
                    if (b.n < 65000) b.n++;
                    if (pedal_pinned) {
                        float ag = (b.n_gap < PWR_BIN_MIN_N) ? 1.0f / (float)(b.n_gap + 1) : PWR_EMA_ALPHA;
                        b.gap = (b.n_gap == 0) ? gap : b.gap + ag * (gap - b.gap);
                        if (b.n_gap < 65000) b.n_gap++;
                    }
                    g_pwr_base_dirty = true;
                }

                g_pwr.bin = bin; g_pwr.tim_live = tim; g_pwr.bin_n = b.n;
                if (b.n >= PWR_BIN_MIN_N) {
                    // Interpolate the baseline to the exact rpm using a local slope from ready
                    // neighbour bins. The map climbs ~2.5 deg per 500-rpm bin, so comparing a
                    // sample against its bin mean alone would read up to ~2.5 deg of phantom pull
                    // at a bin edge; the slope term removes that within-bin gradient.
                    float base = b.tim; float slope = 0.0f; bool have = false;
                    if (bin + 1 < PWR_RPM_BINS && g_pwr_base.bin[bin + 1].n >= PWR_BIN_MIN_N &&
                        bin - 1 >= 0 && g_pwr_base.bin[bin - 1].n >= PWR_BIN_MIN_N) {
                        float dr = g_pwr_base.bin[bin + 1].rpm - g_pwr_base.bin[bin - 1].rpm;
                        if (dr > 1.0f) { slope = (g_pwr_base.bin[bin + 1].tim - g_pwr_base.bin[bin - 1].tim) / dr; have = true; }
                    }
                    if (!have && bin + 1 < PWR_RPM_BINS && g_pwr_base.bin[bin + 1].n >= PWR_BIN_MIN_N) {
                        float dr = g_pwr_base.bin[bin + 1].rpm - b.rpm;
                        if (dr > 1.0f) { slope = (g_pwr_base.bin[bin + 1].tim - b.tim) / dr; have = true; }
                    }
                    if (!have && bin - 1 >= 0 && g_pwr_base.bin[bin - 1].n >= PWR_BIN_MIN_N) {
                        float dr = b.rpm - g_pwr_base.bin[bin - 1].rpm;
                        if (dr > 1.0f) slope = (b.tim - g_pwr_base.bin[bin - 1].tim) / dr;
                    }
                    // clamp so a noisy neighbour can't invent a wild correction (map is < ~0.02 deg/rpm)
                    if (slope >  0.05f) slope =  0.05f;
                    if (slope < -0.05f) slope = -0.05f;
                    base += slope * (rpm_at - b.rpm);
                    g_pwr.tim_base = base;
                    g_pwr.tim_delta = tim - base;
                    g_pwr.delta_valid = true;
                    g_pwr.delta_ts = now;
                    if (g_pwr.tim_delta < g_pwr.sess_min_delta) g_pwr.sess_min_delta = g_pwr.tim_delta;
                    // TIMING: amber must persist past amber for PWR_TIM_HOLD_MS; small hysteresis
                    // on release. RED is a SEPARATE sustained breach (tim_red_since) so a single
                    // noisy bin-edge sample at -6 can't latch red — it only reads amber.
                    if (g_pwr.tim_delta <= PWR_TIM_AMBER_DEG) {
                        if (!g_pwr.tim_bad_since) { g_pwr.tim_bad_since = now; g_pwr.delta_worst = g_pwr.tim_delta; }
                        if (g_pwr.tim_delta < g_pwr.delta_worst) g_pwr.delta_worst = g_pwr.tim_delta;
                        g_pwr.tim_last_bad_ms = now;
                    } else if (g_pwr.tim_delta > PWR_TIM_AMBER_DEG + 0.5f) {
                        g_pwr.tim_bad_since = 0;
                    }
                    if (g_pwr.tim_delta <= PWR_TIM_RED_DEG) {
                        if (!g_pwr.tim_red_since) g_pwr.tim_red_since = now;
                        g_pwr.tim_last_red_ms = now;
                    } else if (g_pwr.tim_delta > PWR_TIM_RED_DEG + 0.5f) {
                        g_pwr.tim_red_since = 0;
                    }
                    // LIFT: calculated load below the cool baseline
                    g_pwr.load_delta = (int)(load - b.load + (load >= b.load ? 0.5f : -0.5f));
                    if (g_pwr.load_delta <= -PWR_LOAD_DEFICIT_PCT) {
                        if (!g_pwr.load_bad_since) g_pwr.load_bad_since = now;
                        g_pwr.load_last_bad_ms = now;
                    } else g_pwr.load_bad_since = 0;
                } else {
                    g_pwr.delta_valid = false; g_pwr.tim_base = 0; g_pwr.tim_delta = 0;
                    g_pwr.tim_bad_since = 0; g_pwr.load_bad_since = 0; g_pwr.load_delta = 0;
                }
                // THROTTLE: pedal pinned but the plate opens less than it did cool
                if (pedal_pinned && b.n_gap >= PWR_BIN_MIN_N) {
                    g_pwr.gap_live = (int)gap;
                    g_pwr.gap_delta = (int)(gap - b.gap + 0.5f);
                    if (g_pwr.gap_delta >= PWR_GAP_PTS) {
                        if (!g_pwr.gap_bad_since) g_pwr.gap_bad_since = now;
                        g_pwr.gap_last_bad_ms = now;
                    } else g_pwr.gap_bad_since = 0;
                } else { g_pwr.gap_bad_since = 0; g_pwr.gap_delta = 0; g_pwr.gap_live = 0; }
            }
        }
    }

    // ---- REV CAP + FUEL? : per new rpm sample, inside a power window ----
    if (g_pwr_window && g_obd_data.rpm_valid && g_obd_data.rpm_timestamp != g_pwr.last_rpm_ts) {
        g_pwr.last_rpm_ts = g_obd_data.rpm_timestamp;
        const float rpm = g_obd_data.rpm;
        const bool pedal_pinned = g_pwr.pedal_src != 0 && g_pwr.pedal_max_seen >= 60 &&
                                  g_pwr.pedal_pct >= g_pwr.pedal_max_seen - 5;
        const bool thr85 = (g_obd_data.throttle_valid && g_obd_data.throttle_pct >= 85) || pedal_pinned;

        // plateau tracker
        if (!thr85) {
            g_pwr.plat_since = 0;
        } else if (g_pwr.plat_since == 0 || fabsf(rpm - g_pwr.plat_ref) > PWR_REVCAP_BAND_RPM) {
            g_pwr.plat_ref = rpm; g_pwr.plat_min = rpm; g_pwr.plat_max = rpm; g_pwr.plat_since = now;
        } else {
            if (rpm < g_pwr.plat_min) g_pwr.plat_min = rpm;
            if (rpm > g_pwr.plat_max) g_pwr.plat_max = rpm;
            // A rev cap is a SUSTAINED, FLAT ceiling at WOT below the normal limiter, with hot
            // oil (the documented oil-temp protection). "Flat" (narrow band) is what separates a
            // real ceiling from a slow climb or a fuel stumble; hot oil is what separates it from
            // a drag/gear-limited top-speed plateau on a cool engine. A brief plateau (< HOLD) or
            // a wobble > the band resets the tracker above, so neither reaches here.
            if (now - g_pwr.plat_since >= PWR_REVCAP_HOLD_MS && g_pwr.plat_ref < PWR_REVCAP_MAX_RPM &&
                rpm >= PWR_WINDOW_RPM_MIN && oil_f >= PWR_REVCAP_OIL_F &&
                (g_pwr.plat_max - g_pwr.plat_min) <= PWR_REVCAP_BAND_RPM) {
                if (!g_pwr.rev_cap_ms || now - g_pwr.rev_cap_ms > 5000) g_pwr.rev_cap_count++;
                g_pwr.rev_cap_rpm = (int)((g_pwr.plat_min + g_pwr.plat_max) * 0.5f + 0.5f);
                g_pwr.rev_cap_ms = now;
            }
        }

        // fuel stumble: a 300-1000 rpm drop between consecutive samples at WOT in a corner,
        // that recovers within 1 s (an upshift drops more and doesn't come back)
        if (thr85 && g_obd_data.rpm_prev_ts && (g_obd_data.rpm_timestamp - g_obd_data.rpm_prev_ts) <= 400) {
            float drop = g_obd_data.rpm_prev - rpm;
            float lat = g_vehicle_data.accel_valid ? g_vehicle_data.accel_x_g : 0.0f;
            if (!g_pwr.fuel_pending && drop >= PWR_FUEL_DROP_MIN && drop <= PWR_FUEL_DROP_MAX && fabsf(lat) >= PWR_FUEL_LAT_G) {
                g_pwr.fuel_pending = true; g_pwr.fuel_pre_rpm = g_obd_data.rpm_prev;
                g_pwr.fuel_drop_ms = now; g_pwr.fuel_lat_g = lat;
            }
        }
        if (g_pwr.fuel_pending) {
            if (now - g_pwr.fuel_drop_ms > PWR_FUEL_RECOVER_MS) g_pwr.fuel_pending = false;      // never recovered: not a stumble
            else if (rpm >= g_pwr.fuel_pre_rpm - 150.0f && now - g_pwr.fuel_drop_ms >= 150) {   // recovered: FUEL?
                g_pwr.fuel_pending = false; g_pwr.fuel_ms = now;
            }
        }
    }
    if (!g_pwr_window) { g_pwr.plat_since = 0; g_pwr.fuel_pending = false; }

    // ---- resolve the POWER state (worst active wins) ----
    int state = PWR_OK, sev = 0;
    char reason[48] = "";
    const bool timing_hold = g_pwr.tim_bad_since && (now - g_pwr.tim_bad_since) >= PWR_TIM_HOLD_MS;
    const bool red_hold    = g_pwr.tim_red_since && (now - g_pwr.tim_red_since) >= PWR_TIM_HOLD_MS;
    static uint32_t tim_latch_until = 0, gap_latch_until = 0, load_latch_until = 0;
    static int tim_latch_sev = 0;
    if (timing_hold) {
        tim_latch_until = g_pwr.tim_last_bad_ms + PWR_TIM_LATCH_MS;
        // red only when the -6 breach itself was sustained; a lone edge spike stays amber
        tim_latch_sev = red_hold ? 2 : 1;
    }
    if (g_pwr.gap_bad_since  && (now - g_pwr.gap_bad_since)  >= PWR_GAP_HOLD_MS)  gap_latch_until  = g_pwr.gap_last_bad_ms  + PWR_LATCH_MS;
    if (g_pwr.load_bad_since && (now - g_pwr.load_bad_since) >= PWR_LOAD_HOLD_MS) load_latch_until = g_pwr.load_last_bad_ms + PWR_LATCH_MS;

    const bool revcap_active = g_pwr.rev_cap_ms && (now - g_pwr.rev_cap_ms) < PWR_REVCAP_LATCH_MS;
    const bool fuel_active   = g_pwr.fuel_ms    && (now - g_pwr.fuel_ms)    < PWR_FUEL_LATCH_MS;
    const bool timing_active = (int32_t)(tim_latch_until - now) > 0;
    const bool gap_active    = (int32_t)(gap_latch_until - now) > 0;
    const bool load_active   = (int32_t)(load_latch_until - now) > 0;
    const bool air_active    = g_pwr.air_loss_pct >= PWR_AIR_BANNER_PCT;

    if (fuel_active) {
        state = PWR_FUEL; sev = 2;
        snprintf(reason, sizeof(reason), "FUEL? stumble %.1fg %s", fabsf(g_pwr.fuel_lat_g), g_pwr.fuel_lat_g >= 0 ? "R" : "L");
    } else if (revcap_active) {
        state = PWR_REVCAP; sev = 2;
        snprintf(reason, sizeof(reason), "REV CAP %d", g_pwr.rev_cap_rpm);
        if (oil_f >= 0) snprintf(reason + strlen(reason), sizeof(reason) - strlen(reason), " - OIL %dF", oil_f);
    } else if (timing_active) {
        state = PWR_TIMING; sev = tim_latch_sev;
        snprintf(reason, sizeof(reason), "TIMING %.1f deg - %s", g_pwr.delta_worst, pwrAttribution(oil_f, iat_f, ect_f));
    } else if (gap_active) {
        state = PWR_THROTTLE; sev = 1;
        snprintf(reason, sizeof(reason), "THROTTLE gap +%d vs cool", g_pwr.gap_delta);
    } else if (load_active) {
        state = PWR_LIFT; sev = 1;
        snprintf(reason, sizeof(reason), "LIFT load %d%% vs cool", g_pwr.load_delta);
    } else if (air_active) {
        state = PWR_AIR; sev = 1;
        snprintf(reason, sizeof(reason), "AIR %.1f%% - IAT %dF", -g_pwr.air_loss_pct, iat_f);
    }
    if (state != g_pwr.state) {
        g_pwr.state_since = now;
        if (state != PWR_OK) Serial.printf("[PWR] state -> %s (%s)\n", PWR_STATE_NAME[state], reason);
        else Serial.println("[PWR] state -> OK");
        if (state == PWR_OK) { g_pwr.ack_state = PWR_OK; g_pwr.ack_sev = 0; }   // re-arm the banner
    }
    g_pwr.state = state; g_pwr.sev = sev;
    strncpy(g_pwr.reason, reason, sizeof(g_pwr.reason) - 1); g_pwr.reason[sizeof(g_pwr.reason) - 1] = '\0';

    // ---- session stats ----
    if (state != PWR_OK) g_pwr.sess_states_seen |= (uint16_t)(1u << state);
    if (iat_f > g_pwr.sess_max_iat_f) g_pwr.sess_max_iat_f = iat_f;
    if (g_pwr.air_loss_pct > g_pwr.sess_max_air_loss) g_pwr.sess_max_air_loss = g_pwr.air_loss_pct;

    // ---- publish for the CSV ----
    g_vehicle_data.tim_base_deg   = g_pwr.delta_valid ? g_pwr.tim_base : 0.0f;
    g_vehicle_data.tim_delta_deg  = g_pwr.delta_valid ? g_pwr.tim_delta : 0.0f;
    g_vehicle_data.tim_bin_n      = g_pwr.bin_n;
    g_vehicle_data.thr_gap_pct    = g_pwr.gap_delta;
    g_vehicle_data.load_delta_pct = g_pwr.delta_valid ? g_pwr.load_delta : 0;
    g_vehicle_data.rev_cap_rpm    = g_pwr.rev_cap_rpm;
    g_vehicle_data.air_loss_pct   = g_pwr.air_loss_pct;
    g_vehicle_data.pwr_state      = state;
    g_vehicle_data.pwr_sev        = sev;

    // ---- housekeeping: persist the baseline (rate-limited), 2 s status line ----
    pwrSaveBaseline(false);
    static uint32_t last_line = 0;
    if (now - last_line >= 2000) {
        last_line = now;
        Serial.printf("[PWR] %s%s%s | dTim %+.1f (bin %d n=%d base %.1f) | gap %+d | load %+d%% | revcap %d x%d | air %+.1f%% | win %d | oil %d iat %d ect %d\n",
            PWR_STATE_NAME[state], reason[0] ? " " : "", reason,
            g_pwr.delta_valid ? g_pwr.tim_delta : 0.0f,
            g_pwr.bin >= 0 ? PWR_RPM_BIN_MIN + g_pwr.bin * PWR_RPM_BIN_SIZE : 0, g_pwr.bin_n,
            g_pwr.delta_valid ? g_pwr.tim_base : 0.0f,
            g_pwr.gap_delta, g_pwr.delta_valid ? g_pwr.load_delta : 0,
            g_pwr.rev_cap_rpm, g_pwr.rev_cap_count, -g_pwr.air_loss_pct, g_pwr_window ? 1 : 0,
            oil_f, iat_f, ect_f);
    }
}

#else
static void updatePowerMonitor() {}
#endif // ENABLE_OBD_CAN
#pragma endregion Heat Derate Monitor

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
#define SD_WRITE_INTERVAL_MS 100  // v6.7: 10 Hz logging to catch sub-second oil-pressure dips (was 1000)
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

// v6.11: cloud auto-upload of completed session logs (Google Drive via an Apps
// Script webhook — see cloud/CloudUpload.gs). All endpoint-agnostic: the box just
// HTTPS-POSTs each file to CLOUD_URL with a shared secret, so you can point it at
// Dropbox/S3/your own server later by changing only /wifi.cfg. Runs at boot while
// WiFi is already up for NTP, then powers WiFi back off.
#define ENABLE_CLOUD_UPLOAD 1
static char g_cloud_url[192]   = "";                 // CLOUD_URL   in /wifi.cfg (Apps Script /exec URL)
static char g_cloud_secret[64] = "";                 // CLOUD_SECRET in /wifi.cfg (must match the script)
static char g_cloud_folder[48] = "370zMonitor_logs"; // CLOUD_FOLDER in /wifi.cfg (Drive folder name)
#define CLOUD_MANIFEST_FILE "/CLOUDUP.DAT"            // records already-uploaded session base names
#define CLOUD_MAX_PER_BOOT  8                         // cap uploads per boot to bound the WiFi window

// v6.12: local WiFi file page — the box serves its SD logs over WiFi so you can
// browse/download them from a laptop/phone on the same network (redundancy for the
// cloud upload; works with no external service). To keep WiFi's RAM from being held
// for the whole drive (internal RAM is the board's tightest resource), it serves for
// a bounded window after boot, then powers WiFi down and reclaims the RAM as usual.
// Reach it at http://<box-ip>/ or http://z370.local/ .
#define ENABLE_FILE_SERVER 1
#define FILE_SERVER_DEFAULT_MINUTES 15                // serve this long after boot; 0 = disabled
static int g_fileserver_minutes = FILE_SERVER_DEFAULT_MINUTES;  // overridable via /wifi.cfg FILE_SERVER_MINUTES

// v6.14: reliability — task watchdog (no hardware) + optional power-fail SD flush.
#define WDT_TIMEOUT_MS 12000            // auto-reset the board if the main loop hangs this long
static bool g_wdt_armed = false;
#define ENABLE_POWER_FAIL_DETECT 0      // 0 = OFF. Needs a 12V-sense divider on POWER_SENSE_PIN first.
#define POWER_SENSE_PIN 6               // candidate spare ADC1 pin; wire ignition-12V via a divider (~12V -> ~2.7V)
#define POWER_SENSE_GOOD_RAW 1800       // analogRead above this = power present (establishes "power good")
#define POWER_SENSE_FAIL_RAW 900        // dropping below this after "good" = power failing -> emergency flush

#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.nist.gov"
#define NTP_SERVER_3 "time.google.com"
#define GMT_OFFSET_SEC (-6 * 3600)       // CST = UTC-6 (now only used by the sunrise/sunset calc)
#define DAYLIGHT_OFFSET_SEC 3600         // (legacy, no longer used for the clock — DST is automatic, see POSIX_TZ)
// v6.9: AUTOMATIC US-Central daylight saving. The RTC now stores UTC; local time — including
// the spring-forward / fall-back hour — is derived from this POSIX TZ rule, so no seasonal
// reflash is needed. "CST6CDT" = std UTC-6 / daylight UTC-5; "M3.2.0" = 2nd Sunday of March
// 02:00 (spring forward), "M11.1.0" = 1st Sunday of November 02:00 (fall back). Only change
// this string if you move to a different timezone.
#define POSIX_TZ "CST6CDT,M3.2.0,M11.1.0"
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
    // Health tracking (#3)
    uint32_t consecutive_write_errors;  // Reset on success, for early warning
    uint32_t total_write_errors;        // Lifetime errors (never reset)
    uint32_t last_sync_marker_ms;       // For periodic sync markers (#4)
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
void syncSystemTimeFromRTC();  // v6.7: set ESP32 clock from RTC for correct SD file timestamps
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

    // v6.7: set the system clock from the RTC now, BEFORE the session file is created,
    // so SD file timestamps are correct without needing WiFi/NTP.
    syncSystemTimeFromRTC();

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
    g_sd_state.consecutive_write_errors = 0;  // Reset health tracking (#3)
    g_sd_state.last_sync_marker_ms = millis(); // First sync marker in 5min (#4)

    const char* header = "datetime,rpm,"
        "oil_press_psi,oil_press_is_critical,"
        "oil_temp_value_f,oil_temp_is_critical,"
        "water_temp_value_f,water_temp_is_critical,"
        "trans_temp_value_f,trans_temp_is_critical,"
        "steer_temp_value_f,steer_temp_is_critical,"
        "diff_temp_value_f,diff_temp_is_critical,"
        "fuel_trust_percent,fuel_trust_is_critical,"
        "accel_x_g,accel_y_g,accel_z_g,"
        "rpm_valid,oil_press_valid,oil_temp_valid,"
        "water_temp_valid,trans_temp_valid,steer_temp_valid,diff_temp_valid,"
        "fuel_trust_valid,accel_valid,"
        "timestamp_ms,elapsed_s,cpu_percent,mode,"
        // v6.9 extended OBD-II columns (appended so existing column positions are unchanged)
        "throttle_pct,throttle_valid,engine_load_pct,engine_load_valid,"
        "intake_air_temp_f,intake_air_temp_valid,ambient_temp_f,ambient_temp_valid,"
        "obd_oil_temp_f,obd_oil_temp_valid,maf_gps,maf_valid,"
        "commanded_lambda,lambda_valid,module_voltage,module_voltage_valid,"
        "fuel_level_pct,fuel_level_valid,baro_kpa,baro_valid,fuel_sys_status,fuel_sys_valid,"
        // v6.10 fuel-trust transparency columns (appended so v6.9 positions are unchanged)
        "stft_b1,stft_b2,ltft_b1,ltft_b2,fuel_trim_valid,"
        "ft_timing_deg,pen_stft,pen_ltft,pen_bank,pen_timing,"
        "mil_on,dtc_count,"
        // v6.15 Heat Derate Monitor columns (appended so v6.10 positions are unchanged; 80 total)
        "tim_base_deg,tim_delta_deg,tim_bin_n,pedal_pct,pedal_src,thr_gap_pct,load_delta_pct,"
        "rev_cap_rpm,air_loss_pct,pwr_state,pwr_sev,atf1_f,atf2_f,tcc_slip_rpm\n";

    size_t written = g_sd_state.data_file.print(header);
    if (written == 0) {
        g_sd_state.data_file.close();
        g_sd_state.file_open = false;
        return false;
    }

    g_sd_state.bytes_written += written;
    // v6.10: stamp firmware version into the CSV as a #-comment line
    // (same shape as the periodic "# SYNC" lines, so parsers already skip it).
    {
        char vbuf[64];
        snprintf(vbuf, sizeof(vbuf), "# 370zMonitor FW %s\n", FW_VERSION);
        g_sd_state.bytes_written += g_sd_state.data_file.print(vbuf);
    }
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
    snprintf(header, sizeof(header), "=== 370zMonitor %s - Serial Log - Session %lu ===\n", FW_VERSION, g_sd_state.boot_count);
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
    char line[700];  // v6.9: widened for datetime + extended OBD columns; v6.10: +12 fuel-trust fields; v6.15: +14 POWER fields (~370 chars worst case, fits)
    
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
                // Get datetime for timestamp prefix
                char datetime_str[24] = "N/A";
                if (g_time_mutex && xSemaphoreTake(g_time_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    strncpy(datetime_str, g_time_state.time_string, sizeof(datetime_str) - 1);
                    datetime_str[sizeof(datetime_str) - 1] = '\0';
                    xSemaphoreGive(g_time_mutex);
                }
                
                // Format: [datetime] [timestamp_ms] message
                char logLine[SERIAL_LOG_MAX_MSG_LEN + 64];
                int len = snprintf(logLine, sizeof(logLine), "[%s] [%lu] %s\n", 
                                   datetime_str, logEntry.timestamp_ms, logEntry.message);
                
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
            
            // Calculate critical flags for CSV logging
            int oil_press_crit = (entry.data.oil_pressure_psi < OIL_PRESS_ValueCriticalLow || 
                                  entry.data.oil_pressure_psi > OIL_PRESS_ValueCriticalAbsolute) ? 1 : 0;
            int oil_temp_crit = (entry.data.oil_temp_value_f > OIL_TEMP_ValueCriticalF) ? 1 : 0;
            int water_temp_crit = (entry.data.water_temp_value_f > W_TEMP_ValueCritical_F) ? 1 : 0;
            int trans_temp_crit = (entry.data.trans_temp_value_f > TRAN_TEMP_ValueCritical_F) ? 1 : 0;
            int steer_temp_crit = (entry.data.steer_temp_value_f > STEER_TEMP_ValueCritical_F) ? 1 : 0;
            int diff_temp_crit = (entry.data.diff_temp_value_f > DIFF_TEMP_ValueCritical_F) ? 1 : 0;
            int fuel_trust_crit = (entry.data.fuel_trust_percent < FUEL_TRUST_ValueCritical) ? 1 : 0;
            
            int len = snprintf(line, sizeof(line),
                "%s,%d,"
                "%d,%d,"
                "%d,%d,"
                "%d,%d,"
                "%d,%d,"
                "%d,%d,"
                "%d,%d,"
                "%d,%d,"
                "%.3f,%.3f,%.3f,"
                "%d,%d,%d,"
                "%d,%d,%d,%d,"
                "%d,%d,"
                "%lu,%.2f,%.1f,%s,"
                // v6.9 extended OBD columns (11 value+valid pairs = 22 fields)
                "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
                "%.2f,%d,%.3f,%d,%.2f,%d,"
                "%d,%d,%d,%d,%d,%d,"
                // v6.10 fuel-trust transparency columns
                "%.1f,%.1f,%.1f,%.1f,%d,"
                "%.1f,%.1f,%.1f,%.1f,%.1f,"
                "%d,%d,"
                // v6.15 Heat Derate Monitor columns (14)
                "%.1f,%.1f,%d,%d,%d,%d,%d,"
                "%d,%.1f,%d,%d,%d,%d,%d\n",
                datetime_copy, entry.data.rpm,  // datetime, rpm
                entry.data.oil_pressure_psi, oil_press_crit,  // oil pressure + critical
                entry.data.oil_temp_value_f, oil_temp_crit,  // oil temp + critical
                entry.data.water_temp_value_f, water_temp_crit,  // water temp + critical
                entry.data.trans_temp_value_f, trans_temp_crit,  // trans temp + critical
                entry.data.steer_temp_value_f, steer_temp_crit,  // steer temp + critical
                entry.data.diff_temp_value_f, diff_temp_crit,  // diff temp + critical
                entry.data.fuel_trust_percent, fuel_trust_crit,  // fuel trust + critical
                entry.data.accel_x_g, entry.data.accel_y_g, entry.data.accel_z_g,  // accelerometer X/Y/Z
                entry.data.rpm_valid ? 1 : 0, entry.data.oil_pressure_valid ? 1 : 0, entry.data.oil_temp_valid ? 1 : 0,  // validity flags part 1
                entry.data.water_temp_valid ? 1 : 0, entry.data.trans_temp_valid ? 1 : 0,
                entry.data.steer_temp_valid ? 1 : 0, entry.data.diff_temp_valid ? 1 : 0,  // validity flags part 2
                entry.data.fuel_trust_valid ? 1 : 0, entry.data.accel_valid ? 1 : 0,  // fuel_trust_valid, accel_valid
                entry.timestamp_ms, entry.elapsed_s, entry.cpu_pct,
                entry.demo_mode ? "DEMO" : "LIVE",  // metadata
                // v6.9 extended OBD columns (value, valid) — order matches the header exactly
                entry.data.throttle_pct,      entry.data.throttle_valid ? 1 : 0,
                entry.data.engine_load_pct,   entry.data.engine_load_valid ? 1 : 0,
                entry.data.intake_air_temp_f, entry.data.intake_air_temp_valid ? 1 : 0,
                entry.data.ambient_temp_f,    entry.data.ambient_temp_valid ? 1 : 0,
                entry.data.obd_oil_temp_f,    entry.data.obd_oil_temp_valid ? 1 : 0,
                entry.data.maf_gps,           entry.data.maf_valid ? 1 : 0,
                entry.data.commanded_lambda,  entry.data.lambda_valid ? 1 : 0,
                entry.data.module_voltage,    entry.data.module_voltage_valid ? 1 : 0,
                entry.data.fuel_level_pct,    entry.data.fuel_level_valid ? 1 : 0,
                entry.data.baro_kpa,          entry.data.baro_valid ? 1 : 0,
                entry.data.fuel_sys_status,   entry.data.fuel_sys_valid ? 1 : 0,
                // v6.10 fuel-trust transparency (order matches the header exactly)
                entry.data.stft_b1, entry.data.stft_b2, entry.data.ltft_b1, entry.data.ltft_b2,
                entry.data.fuel_trim_valid ? 1 : 0,
                entry.data.ft_timing_deg, entry.data.ft_pen_stft, entry.data.ft_pen_ltft,
                entry.data.ft_pen_bank,   entry.data.ft_pen_timing,
                entry.data.mil_on ? 1 : 0, entry.data.dtc_count,
                // v6.15 Heat Derate Monitor (order matches the header exactly)
                entry.data.tim_base_deg, entry.data.tim_delta_deg, entry.data.tim_bin_n,
                entry.data.pedal_pct, entry.data.pedal_src, entry.data.thr_gap_pct, entry.data.load_delta_pct,
                entry.data.rev_cap_rpm, entry.data.air_loss_pct, entry.data.pwr_state, entry.data.pwr_sev,
                entry.data.atf_valid ? entry.data.atf1_f : -1, entry.data.atf_valid ? entry.data.atf2_f : -1,
                entry.data.atf_valid ? entry.data.tcc_slip_rpm : -1
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
                g_sd_state.consecutive_write_errors = 0;  // Reset on success (#3)
            } else {
                g_sd_state.error_count++;
                g_sd_state.consecutive_write_errors++;    // Track consecutive (#3)
                g_sd_state.total_write_errors++;          // Track lifetime (#3)
                
                // Early warning at 3 consecutive errors (#3)
                if (g_sd_state.consecutive_write_errors == 3) {
                    _RealSerial.println("[SD/CORE0] WARNING: 3 consecutive write failures - card may be failing");
                }
                
                if (g_sd_state.error_count >= 10) {
                    g_sd_state.logging_enabled = false;
                    _RealSerial.printf("[SD/CORE0] Too many errors (total=%lu), logging disabled\n", 
                                       g_sd_state.total_write_errors);
                }
            }
            
            // Time-based flush
            uint32_t now = millis();
            if ((now - g_sd_state.last_flush_ms) >= SD_FLUSH_INTERVAL_MS) {
                g_sd_state.data_file.flush();
                g_sd_state.last_flush_ms = now;
            }
            
            // Periodic sync marker every 5 minutes (#4)
            // Helps identify how much data was valid if power dies mid-session
            #define SYNC_MARKER_INTERVAL_MS 300000  // 5 minutes
            if ((now - g_sd_state.last_sync_marker_ms) >= SYNC_MARKER_INTERVAL_MS) {
                char sync_line[64];
                snprintf(sync_line, sizeof(sync_line), "# SYNC %lu writes=%lu bytes=%lu\n", 
                         now, g_sd_state.write_count, g_sd_state.bytes_written);
                g_sd_state.data_file.print(sync_line);
                g_sd_state.data_file.flush();
                g_sd_state.last_sync_marker_ms = now;
                _RealSerial.printf("[SD/CORE0] Sync marker written at %lu ms\n", now);
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

// v6.7: lightweight running session summary (printed once/sec on the [SESSION] line).
// Sampled every main-loop call so it catches peaks the 1 Hz status print would miss.
struct SessionStats {
    int   min_oil_psi;   // lowest oil PSI seen while rpm > 2000 (9999 = none yet)
    int   peak_oil_f;    // highest oil temp seen
    float max_lat_g;     // peak |lateral| g
    float max_lon_g;     // peak |longitudinal| g
    float max_vert_g;    // peak |vertical - 1g| (bump/load)
};
static SessionStats g_sess = { 9999, 0, 0.0f, 0.0f, 0.0f };

static void updateSessionStats() {
    if (g_vehicle_data.oil_pressure_valid && g_vehicle_data.rpm_valid &&
        g_vehicle_data.rpm > 2000) {
        if (g_vehicle_data.oil_pressure_psi < g_sess.min_oil_psi)
            g_sess.min_oil_psi = g_vehicle_data.oil_pressure_psi;
    }
    if (g_vehicle_data.oil_temp_valid) {
        int t = (int)g_vehicle_data.oil_temp_value_f;
        if (t > g_sess.peak_oil_f) g_sess.peak_oil_f = t;
    }
    if (g_vehicle_data.accel_valid) {
        float la = fabsf(g_vehicle_data.accel_x_g);
        float lo = fabsf(g_vehicle_data.accel_y_g);
        float ve = fabsf(g_vehicle_data.accel_z_g - 1.0f);
        if (la > g_sess.max_lat_g)  g_sess.max_lat_g  = la;
        if (lo > g_sess.max_lon_g)  g_sess.max_lon_g  = lo;
        if (ve > g_sess.max_vert_g) g_sess.max_vert_g = ve;
    }
}

// Queue data for SD logging - called from main loop (Core 1)
// Non-blocking: if queue is full, data is dropped
void sdLogData() {
    if (!g_sd_state.file_open || !g_sd_state.logging_enabled) return;
    if (!g_sd_queue) return;

    uint32_t now = millis();

    updateSessionStats();  // v6.7: sample session peaks every loop

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
// BOOT COUNTER MANAGEMENT - WITH CORRUPTION PROTECTION
// Uses dual-file backup with checksum validation to survive power loss
// Primary: BOOTCNT.DAT  Backup: BOOTCNT.BAK
// Format: "count\nCHKSUM:sum" where sum = sum of ASCII digit values
//=================================================================

#define BOOT_COUNT_PRIMARY  "/BOOTCNT.DAT"
#define BOOT_COUNT_BACKUP   "/BOOTCNT.BAK"

// Calculate checksum as sum of digit values (simple but effective)
static uint32_t bootCountChecksum(uint32_t count) {
    uint32_t sum = 0;
    if (count == 0) return 0;  // Special case: checksum of 0 is 0
    while (count > 0) {
        sum += count % 10;
        count /= 10;
    }
    return sum;
}

// Validate boot count file format and checksum
// Returns true if valid, stores count in *out_count
static bool validateBootCountFile(const char* filename, uint32_t* out_count) {
    File f = SD.open(filename, FILE_READ);
    if (!f) {
        return false;
    }
    
    char buf[64] = { 0 };
    size_t len = f.readBytes(buf, sizeof(buf) - 1);
    f.close();
    
    if (len == 0) {
        Serial.printf("[SD] %s: Empty file\n", filename);
        return false;
    }
    
    // Parse format: "count\nCHKSUM:sum"
    char* newline = strchr(buf, '\n');
    if (!newline) {
        Serial.printf("[SD] %s: Missing checksum line\n", filename);
        return false;
    }
    
    *newline = '\0';  // Null-terminate the count string
    uint32_t count = (uint32_t)atol(buf);
    
    // Parse checksum
    char* chksum_line = newline + 1;
    if (strncmp(chksum_line, "CHKSUM:", 7) != 0) {
        Serial.printf("[SD] %s: Invalid checksum format\n", filename);
        return false;
    }
    
    uint32_t stored_checksum = (uint32_t)atol(chksum_line + 7);
    uint32_t calculated_checksum = bootCountChecksum(count);
    
    if (stored_checksum != calculated_checksum) {
        Serial.printf("[SD] %s: Checksum mismatch (stored=%lu calc=%lu)\n", 
                      filename, stored_checksum, calculated_checksum);
        return false;
    }
    
    *out_count = count;
    return true;
}

// Scan SD card for highest session number (catastrophic recovery)
static uint32_t scanSDForHighestSession() {
    Serial.println("[SD] Scanning for highest session number...");
    uint32_t max_session = 0;
    
    File root = SD.open("/");
    if (!root) {
        Serial.println("[SD] Cannot open root for scan");
        return 0;
    }
    
    uint32_t files_scanned = 0;
    File entry;
    while ((entry = root.openNextFile())) {
        const char* name = entry.name();
        files_scanned++;
        
        // Look for SESS_NNNNNNNN.csv or SESS_NNNNNNNN.log
        if (strncmp(name, "SESS_", 5) == 0 && strlen(name) >= 13) {
            // Extract session number (8 digits after SESS_)
            char numstr[9] = { 0 };
            strncpy(numstr, name + 5, 8);
            uint32_t num = (uint32_t)atol(numstr);
            if (num > max_session) {
                max_session = num;
            }
        }
        entry.close();
    }
    root.close();
    
    Serial.printf("[SD] Scanned %lu files, highest session: %lu\n", files_scanned, max_session);
    return max_session;
}

uint32_t sdReadBootCount() {
    uint32_t count = 0;
    
    // Try primary file first
    if (validateBootCountFile(BOOT_COUNT_PRIMARY, &count)) {
        Serial.printf("[SD] Boot count: %lu (from primary)\n", count);
        return count;
    }
    
    // Primary corrupt or missing - try backup
    Serial.println("[SD] Primary boot count invalid, trying backup...");
    if (validateBootCountFile(BOOT_COUNT_BACKUP, &count)) {
        Serial.printf("[SD] Boot count: %lu (recovered from backup!)\n", count);
        return count;
    }
    
    // Both files corrupt - catastrophic recovery via SD scan
    Serial.println("[SD] BOTH boot count files corrupt! Scanning SD...");
    count = scanSDForHighestSession();
    Serial.printf("[SD] Boot count: %lu (recovered from SD scan!)\n", count);
    return count;
}

void sdWriteBootCount(uint32_t count) {
    uint32_t checksum = bootCountChecksum(count);
    char content[32];
    snprintf(content, sizeof(content), "%lu\nCHKSUM:%lu", count, checksum);
    
    // Step 1: Backup current primary to backup file (if primary exists)
    // This preserves the last known good value before we modify primary
    if (SD.exists(BOOT_COUNT_PRIMARY)) {
        SD.remove(BOOT_COUNT_BACKUP);  // Remove old backup
        
        // Read primary content
        File src = SD.open(BOOT_COUNT_PRIMARY, FILE_READ);
        if (src) {
            char backup_content[64] = { 0 };
            size_t len = src.readBytes(backup_content, sizeof(backup_content) - 1);
            src.close();
            
            // Write to backup
            if (len > 0) {
                File dst = SD.open(BOOT_COUNT_BACKUP, FILE_WRITE);
                if (dst) {
                    dst.write((const uint8_t*)backup_content, len);
                    dst.flush();
                    dst.close();
                }
            }
        }
    }
    
    // Step 2: Write new value to primary file
    File f = SD.open(BOOT_COUNT_PRIMARY, FILE_WRITE);
    if (!f) {
        Serial.println("[SD] Warning: Could not write boot count");
        return;
    }
    
    f.print(content);
    f.flush();
    f.close();
    
    Serial.printf("[SD] Boot count written: %lu (checksum=%lu)\n", count, checksum);
    
    // Verify write succeeded by reading back (#2)
    uint32_t verify = 0;
    if (!validateBootCountFile(BOOT_COUNT_PRIMARY, &verify) || verify != count) {
        Serial.println("[SD] WARNING: Boot count write verification FAILED!");
        Serial.printf("[SD]   Expected: %lu, Got: %lu\n", count, verify);
        // Attempt one retry
        f = SD.open(BOOT_COUNT_PRIMARY, FILE_WRITE);
        if (f) {
            f.print(content);
            f.flush();
            f.close();
            Serial.println("[SD] Boot count retry write attempted");
        }
    }
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
            templateFile.println("# v6.11 optional cloud upload (see cloud/CloudUpload.gs):");
            templateFile.println("# CLOUD_URL=https://script.google.com/macros/s/XXXX/exec");
            templateFile.println("# CLOUD_SECRET=your_long_random_secret");
            templateFile.println("# CLOUD_FOLDER=370zMonitor_logs");
            templateFile.println("# v6.12 local WiFi file page: minutes to serve after boot (0=off, default 15):");
            templateFile.println("# FILE_SERVER_MINUTES=15");
            templateFile.println("# v6.15 Heat Derate Monitor: sweep Nissan Mode 22 DIDs 0x1100-0x12FF once at idle (logs [DID] lines):");
            templateFile.println("# OBD_DID_SWEEP=1");
            templateFile.println("# v6.15: wipe the learned cool timing baseline at boot (remove the line afterwards):");
            templateFile.println("# PWR_BASELINE_RESET=1");
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
            // v6.11: optional cloud-upload settings (same /wifi.cfg file)
            else if (key.equalsIgnoreCase("CLOUD_URL")) {
                strncpy(g_cloud_url, value.c_str(), sizeof(g_cloud_url) - 1);
                g_cloud_url[sizeof(g_cloud_url) - 1] = '\0';
                Serial.printf("[CLOUD] Upload URL loaded (%d chars)\n", (int)strlen(g_cloud_url));
            }
            else if (key.equalsIgnoreCase("CLOUD_SECRET")) {
                strncpy(g_cloud_secret, value.c_str(), sizeof(g_cloud_secret) - 1);
                g_cloud_secret[sizeof(g_cloud_secret) - 1] = '\0';
                Serial.println("[CLOUD] Upload secret loaded: ********");
            }
            else if (key.equalsIgnoreCase("CLOUD_FOLDER")) {
                strncpy(g_cloud_folder, value.c_str(), sizeof(g_cloud_folder) - 1);
                g_cloud_folder[sizeof(g_cloud_folder) - 1] = '\0';
                Serial.printf("[CLOUD] Upload folder: %s\n", g_cloud_folder);
            }
            // v6.12: how long (minutes) to keep the local WiFi file page up after boot (0 = off)
            else if (key.equalsIgnoreCase("FILE_SERVER_MINUTES")) {
                g_fileserver_minutes = value.toInt();
                Serial.printf("[FILESRV] Serve window: %d min\n", g_fileserver_minutes);
            }
#if ENABLE_OBD_CAN
            // v6.15: Heat Derate Monitor options
            else if (key.equalsIgnoreCase("OBD_DID_SWEEP")) {
                g_did_sweep_enabled = (value.toInt() != 0);
                Serial.printf("[DID] Mode 22 DID sweep: %s\n", g_did_sweep_enabled ? "ENABLED (runs once at idle)" : "off");
            }
            else if (key.equalsIgnoreCase("PWR_BASELINE_RESET")) {
                g_pwr_baseline_reset_req = (value.toInt() != 0);
                if (g_pwr_baseline_reset_req) Serial.println("[PWR] baseline reset requested by /wifi.cfg (remove the line afterwards)");
            }
#endif
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
// v6.9: timezone / DST helpers ------------------------------------------------
// Install the POSIX TZ rule once so localtime_r()/getLocalTime() apply US-Central DST
// automatically (spring-forward / fall-back). Idempotent — safe to call from anywhere.
static void ensureTimezone() {
    static bool tz_done = false;
    if (tz_done) return;
    setenv("TZ", POSIX_TZ, 1);
    tzset();
    tz_done = true;
}

// Portable "UTC broken-down time -> epoch seconds" (a timegm() that doesn't rely on the
// platform providing timegm). Howard Hinnant's days-from-civil algorithm. The DS3231 now
// stores UTC, so this turns an RTC read into a real time_t that localtime_r() can localize.
static time_t utcTmToEpoch(const struct tm* t) {
    int year  = t->tm_year + 1900;
    int month = t->tm_mon + 1;            // 1..12
    int day   = t->tm_mday;
    int y = year - (month <= 2 ? 1 : 0);
    int era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153u * (unsigned)(month + (month > 2 ? -3 : 9)) + 2u) / 5u + (unsigned)(day - 1);
    unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    long long days = (long long)era * 146097LL + (long long)doe - 719468LL;
    return (time_t)(days * 86400LL + t->tm_hour * 3600 + t->tm_min * 60 + t->tm_sec);
}

// Read RTC (UTC since v6.9) and return LOCAL broken-down time with automatic DST applied.
bool readRTCLocal(struct tm* local_out) {
    struct tm utc;
    if (!readRTC(&utc)) return false;
    ensureTimezone();
    time_t epoch = utcTmToEpoch(&utc);
    localtime_r(&epoch, local_out);
    return true;
}

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

// v6.7: Push the DS3231 RTC's local time into the ESP32 system clock so that SD/FAT file
// timestamps (get_fattime) are correct even with NO WiFi/NTP. Without this, no-WiFi
// sessions get 1979/1980 file dates while the in-log datetime (read from the RTC) is fine.
void syncSystemTimeFromRTC() {
    if (!g_sd_state.rtc_available || !isRTCTimeValid()) return;
    ensureTimezone();
    struct tm utc;
    if (!readRTC(&utc)) return;                  // v6.9: RTC stores UTC
    time_t t = utcTmToEpoch(&utc);
    if (t < 1735689600) return;  // sanity: reject anything before 2025-01-01
    struct timeval tv;
    tv.tv_sec = t;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);                      // system clock = UTC; TZ (POSIX_TZ) applies DST on read
    struct tm local;
    localtime_r(&t, &local);                      // for the log line only
    Serial.printf("[TIME] System clock set from RTC (UTC, local=%s): %02d/%02d/%04d %02d:%02d:%02d\n",
        local.tm_isdst > 0 ? "CDT" : "CST",
        local.tm_mon + 1, local.tm_mday, local.tm_year + 1900,
        local.tm_hour, local.tm_min, local.tm_sec);
}

// Try to sync time from a specific NTP server.
// v6.8 FIX: the old version called getLocalTime() right after configTime() and returned
// true as soon as the clock read any year > 2016 — but the system clock was already seeded
// from the (possibly wrong) RTC at boot, so getLocalTime() handed back the STALE RTC time
// before the real SNTP response ever arrived. That stale time was then written back to the
// RTC, so NTP could never actually correct the clock (it just re-cemented the RTC's error).
// Fix: seed the clock to an old sentinel, then only accept the sync once SNTP has pushed the
// clock forward to a genuinely fresh date (year >= 2025). Restore the prior clock on failure
// so SD/FAT file timestamps stay correct when NTP is unavailable.
bool tryNTPSync(const char* server) {
    Serial.printf("[TIME] Trying NTP server: %s\n", server);
    time_t before = time(nullptr);           // current (RTC-seeded) clock
    struct timeval sentinel;                 // 2024-01-01 UTC sentinel
    sentinel.tv_sec = 1704067200;
    sentinel.tv_usec = 0;
    settimeofday(&sentinel, NULL);           // force clock "old" until SNTP lands
    ensureTimezone();
    configTzTime(POSIX_TZ, server);          // v6.9: TZ-aware — SNTP sets UTC, POSIX_TZ applies DST

    struct tm timeinfo;
    uint32_t start_ms = millis();
    while ((millis() - start_ms) < NTP_SYNC_TIMEOUT_MS) {
        // A real SNTP response lands at 2025+; the sentinel is 2024, so year>=2025 proves the
        // clock was actually updated by NTP rather than left at the seed (or the stale RTC time).
        if (getLocalTime(&timeinfo, 100) && (timeinfo.tm_year + 1900) >= 2025) {
            Serial.printf("[TIME] NTP sync successful from %s\n", server);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // No fresh response — restore the pre-sync clock so file timestamps remain sane.
    struct timeval restore;
    restore.tv_sec = before;
    restore.tv_usec = 0;
    settimeofday(&restore, NULL);
    Serial.printf("[TIME] NTP server %s timeout (no fresh SNTP response)\n", server);
    return false;
}

//=================================================================
// v6.11: Cloud auto-upload of completed session logs (runs on Core 0)
//=================================================================
// Called from timeSyncTask() while WiFi is already up for NTP, just before it
// powers WiFi back off. Streams each not-yet-uploaded SESS_*.csv/.log to
// CLOUD_URL (HTTPS POST — file as the body, metadata in the query string) and
// records success in /CLOUDUP.DAT so nothing is ever re-sent. Endpoint-agnostic;
// with the bundled Apps Script (cloud/CloudUpload.gs) files land in Google Drive.
//
// NOT flash-tested. On first flash verify: (1) the Apps Script 302 redirect is
// followed (we accept 200/301/302), and (2) TLS fits in RAM/stack — mbedTLS
// alloc or stack-overflow errors mean the heap/task-stack is too small.
#if ENABLE_CLOUD_UPLOAD
static bool cloudAlreadyUploaded(const char* base) {
    File m = SD.open(CLOUD_MANIFEST_FILE, FILE_READ);
    if (!m) return false;
    bool found = false;
    while (m.available()) {
        String line = m.readStringUntil('\n'); line.trim();
        if (line == base) { found = true; break; }
    }
    m.close();
    return found;
}

static void cloudMarkUploaded(const char* base) {
    File m = SD.open(CLOUD_MANIFEST_FILE, FILE_APPEND);
    if (m) { m.println(base); m.close(); }
}

// Stream one SD file to the endpoint as the POST body. True on success.
static bool cloudUploadFile(const char* path, const char* remoteName) {
    File f = SD.open(path, FILE_READ);
    if (!f) { Serial.printf("[CLOUD] open failed: %s\n", path); return false; }
    size_t sz = f.size();

    WiFiClientSecure client;
    client.setInsecure();                 // uploading our own logs; skip cert pinning
    HTTPClient http;
    char url[420];
    snprintf(url, sizeof(url), "%s?secret=%s&folder=%s&name=%s",
             g_cloud_url, g_cloud_secret, g_cloud_folder, remoteName);
    if (!http.begin(client, url)) { f.close(); return false; }
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);  // Apps Script 302 -> googleusercontent
    http.setTimeout(20000);
    http.addHeader("Content-Type", "text/plain");

    int code = http.sendRequest("POST", &f, sz);            // stream file, no big RAM buffer
    String resp = (code > 0) ? http.getString() : String("");
    http.end();
    f.close();

    bool ok = (code == 200 && resp.indexOf("OK") >= 0) || code == 301 || code == 302;
    Serial.printf("[CLOUD] %-22s HTTP %d  %s  (%u bytes)\n",
                  remoteName, code, ok ? "OK" : "FAIL", (unsigned)sz);
    return ok;
}

// Scan the card and upload any completed sessions not yet uploaded.
void uploadPendingSessions() {
    if (strlen(g_cloud_url) < 8 || strlen(g_cloud_secret) < 1) return;   // not configured
    if (!g_sd_state.initialized) return;
    Serial.println("[CLOUD] Scanning card for sessions to upload...");

    // Pass 1: collect candidate base names (avoid nested SD ops during iteration).
    String bases[40]; int nb = 0;
    File root = SD.open("/");
    if (root) {
        File entry;
        while (nb < 40 && (entry = root.openNextFile())) {
            bool isDir = entry.isDirectory();
            String fn = entry.name();
            entry.close();
            if (isDir) continue;
            int slash = fn.lastIndexOf('/'); if (slash >= 0) fn = fn.substring(slash + 1);
            String low = fn; low.toLowerCase();                    // FAT may report .CSV uppercase
            if (low.startsWith("sess_") && low.endsWith(".csv"))
                bases[nb++] = fn.substring(0, fn.length() - 4);   // strip ".csv", keep original case
        }
        root.close();
    }

    // Pass 2: upload csv + log for each pending session.
    int done = 0;
    for (int i = 0; i < nb && done < CLOUD_MAX_PER_BOOT; i++) {
        const char* base = bases[i].c_str();
        if (strstr(g_sd_state.current_filename, base)) continue;   // skip the open session
        if (cloudAlreadyUploaded(base)) continue;

        char csvPath[48], logPath[48], csvName[44], logName[44];
        snprintf(csvPath, sizeof(csvPath), "/%s.csv", base);
        snprintf(logPath, sizeof(logPath), "/%s.log", base);
        snprintf(csvName, sizeof(csvName), "%s.csv", base);
        snprintf(logName, sizeof(logName), "%s.log", base);

        bool okCsv = cloudUploadFile(csvPath, csvName);
        bool okLog = SD.exists(logPath) ? cloudUploadFile(logPath, logName) : true;
        if (okCsv && okLog) { cloudMarkUploaded(base); done++; }
    }
    Serial.printf("[CLOUD] Uploaded %d new session(s) this boot.\n", done);
}
#else
void uploadPendingSessions() {}
#endif // ENABLE_CLOUD_UPLOAD

//=================================================================
// v6.12: Local WiFi file page (browse/download logs from a browser)
//=================================================================
// Runs from timeSyncTask() (Core 0) for a bounded window after boot, reusing the
// WiFi that's already up for NTP/upload. SD reads are bracketed with the same
// g_fb_pause_sd_writes flag the file browser uses, so they can't collide with the
// logging writer. After the window, WiFi powers off and its RAM is reclaimed.
// NOT flash-tested. Main risk: internal RAM while WiFi + the full UI run together —
// if the box glitches during the window, lower FILE_SERVER_MINUTES (or set 0).
#if ENABLE_FILE_SERVER
static WebServer* g_web = nullptr;   // valid only during the serve window

// Only allow our own log files to be fetched (blocks path traversal).
static bool fsValidName(const String& n) {
    if (n.length() < 6 || n.length() > 32) return false;
    String low = n; low.toLowerCase();
    if (!low.startsWith("sess_")) return false;
    if (!(low.endsWith(".csv") || low.endsWith(".log"))) return false;
    for (size_t i = 0; i < n.length(); i++) {
        char c = n[i];
        if (!(isalnum((unsigned char)c) || c == '_' || c == '.')) return false;  // no '/', no ".."
    }
    return true;
}

// Root: chunked listing of SESS_* files with sizes + download links.
static void fsHandleRoot() {
    g_fb_pause_sd_writes = true; vTaskDelay(pdMS_TO_TICKS(150));
    g_web->setContentLength(CONTENT_LENGTH_UNKNOWN);
    g_web->send(200, "text/html", "");
    g_web->sendContent(F(
        "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>370Z logs</title><style>body{font-family:system-ui;background:#0a0a0a;color:#eee;margin:0;padding:16px}"
        "h1{color:#d41f2d;font-size:19px;margin:0 0 12px}a{color:#4da6ff;text-decoration:none}"
        "table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:6px 9px;border-bottom:1px solid #222}"
        "th{color:#888;text-align:left}.s{font-family:monospace;text-align:right;color:#9a9a9a}"
        ".d{text-align:right}</style>"
        "<h1>370Z session logs</h1><table><tr><th>File</th><th class=s>Size</th><th class=d></th></tr>"));
    File root = SD.open("/");
    if (root) {
        File e;
        while ((e = root.openNextFile())) {
            String n = e.name(); bool dir = e.isDirectory(); size_t sz = e.size(); e.close();
            int sl = n.lastIndexOf('/'); if (sl >= 0) n = n.substring(sl + 1);
            if (dir) continue;
            String low = n; low.toLowerCase();
            if (low.startsWith("sess_") && (low.endsWith(".csv") || low.endsWith(".log"))) {
                String row = "<tr><td>" + n + "</td><td class=s>" + String((sz + 1023) / 1024) +
                             " KB</td><td class=d><a href='/dl?f=" + n + "'>download</a></td></tr>";
                g_web->sendContent(row);
            }
        }
        root.close();
    }
    String foot = "</table><p style='color:#888;font-size:12px'>Serving for ~" + String(g_fileserver_minutes) +
                  " min after boot — restart the car to reopen. Firmware " FW_VERSION "</p>";
    g_web->sendContent(foot);
    g_web->sendContent("");            // end chunked response
    g_fb_pause_sd_writes = false;
}

// Download one validated session file (streamed straight off the SD card).
static void fsHandleDownload() {
    if (!g_web->hasArg("f")) { g_web->send(400, "text/plain", "missing f"); return; }
    String f = g_web->arg("f");
    if (!fsValidName(f)) { g_web->send(400, "text/plain", "bad name"); return; }
    String path = "/" + f;
    g_fb_pause_sd_writes = true; vTaskDelay(pdMS_TO_TICKS(150));
    File file = SD.open(path.c_str(), FILE_READ);
    if (!file) { g_fb_pause_sd_writes = false; g_web->send(404, "text/plain", "not found"); return; }
    g_web->sendHeader("Content-Disposition", "attachment; filename=" + f);
    g_web->streamFile(file, "application/octet-stream");
    file.close();
    g_fb_pause_sd_writes = false;
}

// Bring the server up, serve for the configured window, then tear it down.
static void serveFilesWindow() {
    if (g_fileserver_minutes <= 0) return;
    WebServer server(80);
    g_web = &server;
    server.on("/", fsHandleRoot);
    server.on("/dl", fsHandleDownload);
    server.onNotFound([]() { g_web->send(404, "text/plain", "404 - try /"); });
    server.begin();
    if (MDNS.begin("z370")) MDNS.addService("http", "tcp", 80);
    Serial.printf("[FILESRV] Ready: http://%s/  (or http://z370.local/) for ~%d min\n",
                  WiFi.localIP().toString().c_str(), g_fileserver_minutes);

    uint32_t start = millis();
    uint32_t windowMs = (uint32_t)g_fileserver_minutes * 60000UL;
    while (millis() - start < windowMs) {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    server.stop();
    MDNS.end();
    g_web = nullptr;
    Serial.println("[FILESRV] Serve window closed; powering WiFi down");
}
#else
static void serveFilesWindow() {}
#endif // ENABLE_FILE_SERVER

// v6.13: on a WiFi failure, scan 2.4 GHz and report whether the configured SSID is
// even visible. The ESP32 radio is 2.4 GHz only, so if a scan does NOT see the SSID,
// it's 5 GHz-only here (band steering) or out of range — not a password problem.
static void logWifiScanDiag() {
    Serial.printf("[WIFI] status=%d (3=connected,1=no-SSID-found,4=connect/auth-fail,6=disconnected)\n",
                  (int)WiFi.status());
    Serial.println("[WIFI] Scanning 2.4 GHz for the configured SSID...");
    int n = WiFi.scanNetworks();
    bool found = false;
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) == String(g_wifi_ssid)) {
            found = true;
            Serial.printf("[WIFI]   FOUND '%s'  rssi=%d dBm  ch=%d  enc=%d\n",
                          g_wifi_ssid, WiFi.RSSI(i), WiFi.channel(i), (int)WiFi.encryptionType(i));
        }
    }
    if (!found) {
        Serial.printf("[WIFI]   '%s' NOT visible on 2.4 GHz (saw %d networks). Likely 5 GHz-only here "
                      "(band steering) or out of range — give 2.4 GHz its own SSID, e.g. %s-2G.\n",
                      g_wifi_ssid, n, g_wifi_ssid);
    } else {
        Serial.println("[WIFI]   SSID is visible on 2.4 GHz — if connect still fails it's the password.");
    }
    WiFi.scanDelete();
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
            logWifiScanDiag();   // v6.13: is the SSID even visible on 2.4 GHz?
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
        // v6.8: guard — only trust/write a genuinely fresh NTP time (year >= 2025).
        if (getLocalTime(&timeinfo) && (timeinfo.tm_year + 1900) >= 2025) {
            Serial.printf("[TIME/CORE0] NTP time (local %s): %02d/%02d/%04d %02d:%02d:%02d\n",
                timeinfo.tm_isdst > 0 ? "CDT" : "CST",
                timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_year + 1900,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

            // *** WRITE NTP TIME TO RTC AS UTC (v6.9: RTC stores UTC, DST applied on read) ***
            if (g_sd_state.rtc_available) {
                time_t now_utc = time(NULL);
                struct tm utc;
                gmtime_r(&now_utc, &utc);
                if (writeRTC(&utc)) {
                    clearRTCOSFlag();  // Mark time as valid
                    Serial.println("[TIME/CORE0] RTC updated from NTP (stored as UTC)");
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
    
    // v6.11: while WiFi is up, push any completed sessions to the cloud
    uploadPendingSessions();

    // v6.12: then keep WiFi up and serve the local file page for a bounded window
    serveFilesWindow();

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
        
        // Priority 1: DS3231 RTC (stores UTC since v6.9; readRTCLocal applies POSIX_TZ + DST)
        if (g_sd_state.rtc_available) {
            if (readRTCLocal(&timeinfo)) {
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
            // RTC has valid time - use it immediately (RTC is UTC; convert to local w/ DST)
            // WiFi sync will run in background and update if successful
            struct tm timeinfo;
            if (readRTCLocal(&timeinfo)) {
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
        16384,                  // v6.11: bumped 4096->16384; the TLS handshake for cloud upload
                                //        needs a much deeper stack than NTP alone (was overflowing).
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
    int psi = g_vehicle_data.oil_pressure_psi;
    // Overpressure (blocked cooler / bad reading) - always critical.
    if (psi > OIL_PRESS_ValueCriticalAbsolute) return true;
    // v6.7: RPM-aware warm-oil low-pressure floor (replaces the fixed <10 psi trip).
    // Assumes warm oil (track use); only armed while the engine is running.
    if (g_vehicle_data.rpm_valid && g_vehicle_data.rpm >= OIL_PRESS_RPM_ACTIVE) {
        int floor_psi = (g_vehicle_data.rpm * OIL_PRESS_PSI_PER_1000RPM) / 1000;
        if (floor_psi < OIL_PRESS_IdleFloorPSI) floor_psi = OIL_PRESS_IdleFloorPSI;
        return (psi < floor_psi);
    }
    // Engine idling / RPM unknown: fall back to the low absolute floor.
    return (psi < OIL_PRESS_ValueCriticalLow);
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

//=================================================================
// AUTO BRIGHTNESS - Sunrise/Sunset Calculation (NOAA Algorithm)
//=================================================================

// Helper: Convert degrees to radians
static inline float degToRad(float deg) {
    return deg * M_PI / 180.0f;
}

// Helper: Convert radians to degrees
static inline float radToDeg(float rad) {
    return rad * 180.0f / M_PI;
}

// Calculate day of year (1-365/366)
static int getDayOfYear(int year, int month, int day) {
    static const int days_before_month[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int doy = days_before_month[month - 1] + day;
    
    // Add 1 for leap year if after February
    if (month > 2) {
        bool is_leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
        if (is_leap) doy++;
    }
    
    return doy;
}

// Calculate sunrise and sunset times for a given date and location
// Returns times in hours (e.g., 6.5 = 6:30 AM) in UTC
static void calculateSunriseSunset(int year, int month, int day, 
                                    float latitude, float longitude,
                                    float* sunrise_out, float* sunset_out) {
    
    // Day of year
    int N = getDayOfYear(year, month, day);
    
    // Fractional year (gamma) in radians
    float gamma = (2.0f * M_PI / 365.0f) * (N - 1);
    
    // Equation of time (in minutes)
    float eqtime = 229.18f * (0.000075f 
                   + 0.001868f * cos(gamma) 
                   - 0.032077f * sin(gamma)
                   - 0.014615f * cos(2.0f * gamma) 
                   - 0.040849f * sin(2.0f * gamma));
    
    // Solar declination (in radians)
    float decl = 0.006918f 
               - 0.399912f * cos(gamma) 
               + 0.070257f * sin(gamma)
               - 0.006758f * cos(2.0f * gamma) 
               + 0.000907f * sin(2.0f * gamma)
               - 0.002697f * cos(3.0f * gamma) 
               + 0.001480f * sin(3.0f * gamma);
    
    // Hour angle at sunrise/sunset
    float lat_rad = degToRad(latitude);
    float cos_ha = (cos(degToRad(90.833f)) / (cos(lat_rad) * cos(decl))) 
                   - tan(lat_rad) * tan(decl);
    
    // Clamp to valid range (handles polar day/night)
    if (cos_ha > 1.0f) cos_ha = 1.0f;
    if (cos_ha < -1.0f) cos_ha = -1.0f;
    
    float ha = radToDeg(acos(cos_ha));  // Hour angle in degrees
    
    // Solar noon (in minutes from midnight, UTC)
    float solar_noon = 720.0f - 4.0f * longitude - eqtime;
    
    // Sunrise and sunset times (in minutes from midnight, UTC)
    float sunrise_min = solar_noon - ha * 4.0f;
    float sunset_min = solar_noon + ha * 4.0f;
    
    // Convert to hours
    *sunrise_out = sunrise_min / 60.0f;
    *sunset_out = sunset_min / 60.0f;
}

// Apply timezone offset to UTC hours
static float applyTimezoneOffset(float utc_hours, float tz_offset_hours) {
    float local = utc_hours + tz_offset_hours;
    
    // Wrap around midnight
    while (local < 0) local += 24.0f;
    while (local >= 24.0f) local -= 24.0f;
    
    return local;
}

// Convert hours to "HH:MM" string
static void hoursToTimeString(float hours, char* buf, size_t buf_size) {
    int h = (int)hours;
    int m = (int)((hours - h) * 60.0f + 0.5f);
    
    if (m >= 60) { m -= 60; h++; }
    if (h >= 24) h -= 24;
    
    snprintf(buf, buf_size, "%02d:%02d", h, m);
}

// Initialize auto brightness
void autoBrightnessInit() {
    g_auto_brightness.initialized = true;
    g_auto_brightness.last_check_day = -1;  // Force recalculation
    g_auto_brightness.last_check_ms = 0;
    g_auto_brightness.manual_override = false;
    
    Serial.println("[AUTO-BRI] Initialized - sunrise/sunset auto-dimming");
    Serial.printf("[AUTO-BRI] Location: %.2f°N, %.2f°W (Schaumburg, IL)\n", 
                  LOCATION_LATITUDE, fabs(LOCATION_LONGITUDE));
    Serial.printf("[AUTO-BRI] Day: %d%%, Night: %d%%, Enabled: %s\n",
                  (BRIGHTNESS_DAY * 100) / 255, (BRIGHTNESS_NIGHT * 100) / 255,
                  g_auto_brightness.enabled ? "YES" : "NO");
}

// Update auto brightness button text (two lines: "DIM:" and mode/time)
void updateAutoBrightnessButtonText() {
    if (auto_bri_lbl) {
        if (g_auto_brightness.enabled) {
            // Show next toggle time if times are calculated
            if (g_auto_brightness.last_check_day != -1) {
                char btn_text[24];
                if (g_auto_brightness.is_daytime) {
                    // Daytime - show when it will dim (sunset)
                    snprintf(btn_text, sizeof(btn_text), "DIM:\nAUTO [%s]", g_auto_brightness.sunset_str);
                } else {
                    // Nighttime - show when it will brighten (sunrise)
                    snprintf(btn_text, sizeof(btn_text), "DIM:\nAUTO [%s]", g_auto_brightness.sunrise_str);
                }
                lv_label_set_text(auto_bri_lbl, btn_text);
            } else {
                lv_label_set_text(auto_bri_lbl, "DIM:\nAUTO");
            }
        } else {
            lv_label_set_text(auto_bri_lbl, "DIM:\nMANUAL");
        }
    }
}

// Main update function - call periodically
// Returns true if brightness was changed
bool autoBrightnessUpdate() {
    if (!g_auto_brightness.enabled || !g_auto_brightness.initialized) {
        return false;
    }
    
    // Skip if manual override is active
    if (g_auto_brightness.manual_override) {
        return false;
    }
    
    // Rate limit checks
    uint32_t now = millis();
    if (now - g_auto_brightness.last_check_ms < AUTO_BRIGHTNESS_CHECK_INTERVAL_MS) {
        return false;
    }
    g_auto_brightness.last_check_ms = now;
    
    // Get current time
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 100)) {
        return false;
    }
    
    // Calculate day of year for cache check
    int day_of_year = getDayOfYear(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
    
    // Recalculate sunrise/sunset if date changed
    if (day_of_year != g_auto_brightness.last_check_day) {
        float sunrise_utc, sunset_utc;
        
        calculateSunriseSunset(
            timeinfo.tm_year + 1900,
            timeinfo.tm_mon + 1,
            timeinfo.tm_mday,
            LOCATION_LATITUDE,
            LOCATION_LONGITUDE,
            &sunrise_utc,
            &sunset_utc
        );
        
        // Apply timezone offset (CST = UTC-6), + 1h automatically while DST is active (CDT)
        // so sunrise/sunset brightness transitions track wall-clock time year-round.
        float tz_hours = (float)GMT_OFFSET_SEC / 3600.0f + (timeinfo.tm_isdst > 0 ? 1.0f : 0.0f);

        g_auto_brightness.sunrise_hours = applyTimezoneOffset(sunrise_utc, tz_hours);
        g_auto_brightness.sunset_hours = applyTimezoneOffset(sunset_utc, tz_hours);
        
        // Apply twilight offset
        float twilight_hours = TWILIGHT_OFFSET_MINUTES / 60.0f;
        g_auto_brightness.sunrise_hours -= twilight_hours;
        g_auto_brightness.sunset_hours += twilight_hours;
        
        // Generate display strings
        hoursToTimeString(g_auto_brightness.sunrise_hours, g_auto_brightness.sunrise_str, sizeof(g_auto_brightness.sunrise_str));
        hoursToTimeString(g_auto_brightness.sunset_hours, g_auto_brightness.sunset_str, sizeof(g_auto_brightness.sunset_str));
        
        g_auto_brightness.last_check_day = day_of_year;
        
        Serial.printf("[AUTO-BRI] Date: %04d-%02d-%02d | Sunrise: %s, Sunset: %s (local)\n",
                      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                      g_auto_brightness.sunrise_str, g_auto_brightness.sunset_str);
        
        // Update button to show calculated times
        updateAutoBrightnessButtonText();
    }
    
    // Current time in hours
    float current_hours = timeinfo.tm_hour + timeinfo.tm_min / 60.0f + timeinfo.tm_sec / 3600.0f;
    
    // Determine if it's daytime
    bool was_daytime = g_auto_brightness.is_daytime;
    
    if (g_auto_brightness.sunrise_hours < g_auto_brightness.sunset_hours) {
        // Normal case: sunrise before sunset (same day)
        g_auto_brightness.is_daytime = (current_hours >= g_auto_brightness.sunrise_hours && 
                                         current_hours < g_auto_brightness.sunset_hours);
    } else {
        // Edge case: polar regions
        g_auto_brightness.is_daytime = (current_hours >= g_auto_brightness.sunrise_hours || 
                                         current_hours < g_auto_brightness.sunset_hours);
    }
    
    // Change brightness if state changed
    if (g_auto_brightness.is_daytime != was_daytime) {
        uint8_t new_brightness = g_auto_brightness.is_daytime ? BRIGHTNESS_DAY : BRIGHTNESS_NIGHT;
        setBrightness(new_brightness);
        
        Serial.printf("[AUTO-BRI] %s -> %s (brightness: %d%%)\n",
                      was_daytime ? "Day" : "Night",
                      g_auto_brightness.is_daytime ? "Day" : "Night",
                      (new_brightness * 100) / 255);
        
        // Update button to show next toggle time
        updateAutoBrightnessButtonText();
        
        return true;
    }
    
    return false;
}

// Force immediate brightness update
void autoBrightnessForceUpdate() {
    if (!g_auto_brightness.enabled || !g_auto_brightness.initialized) return;
    
    g_auto_brightness.last_check_ms = 0;
    g_auto_brightness.last_check_day = -1;
    g_auto_brightness.manual_override = false;
    autoBrightnessUpdate();
    
    // Ensure button text is updated with calculated times
    updateAutoBrightnessButtonText();
}

#pragma region Utility Box Callbacks

#define DOUBLE_TAP_TIMEOUT_MS 400  // 400ms window for double-tap detection

// Update mode indicator text and color
// v6.13: an always-visible banner so DEMO mode can never be mistaken for live data.
// The small "DEMO" label above lives inside the utility box, which is hidden by
// default — that's why a mid-drive demo trigger looked like real climbing temps.
static void updateDemoBanner() {
    if (!g_demo_mode) {
        if (g_demo_banner) lv_obj_add_flag(g_demo_banner, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (!g_demo_banner) {
        lv_obj_t* top = lv_layer_top();
        g_demo_banner = lv_label_create(top);
        lv_obj_set_style_bg_color(g_demo_banner, lv_color_hex(0xFF00FF), 0);   // magenta = fake
        lv_obj_set_style_bg_opa(g_demo_banner, LV_OPA_COVER, 0);
        lv_obj_set_style_text_color(g_demo_banner, lv_color_hex(0x000000), 0);
        lv_obj_set_style_text_font(g_demo_banner, &lv_font_montserrat_20, 0);
        lv_obj_set_style_pad_ver(g_demo_banner, 5, 0);
        lv_obj_set_style_pad_hor(g_demo_banner, 40, 0);
        lv_label_set_text(g_demo_banner, "DEMO - SIMULATED DATA (not live)");
        lv_obj_set_width(g_demo_banner, lv_pct(100));
        lv_obj_set_style_text_align(g_demo_banner, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(g_demo_banner, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_add_flag(g_demo_banner, LV_OBJ_FLAG_IGNORE_LAYOUT);
    }
    lv_obj_clear_flag(g_demo_banner, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(g_demo_banner);   // keep it above gauges/overlays
}

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
    updateDemoBanner();   // v6.13: always-visible DEMO banner
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
    bool accel_ok = true;
    bool obd_can_ok = true;
    
    // Build list of failures
    char error_msg[320] = "";
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

#if ENABLE_GSENSOR
    // Check Accelerometer (LIS3DH)
    accel_ok = g_accel_initialized && g_accel_connected;
    Serial.printf("[SYSTEM CHECK] G-Sensor (LIS3DH): %s (init=%d, conn=%d)\n", 
                  accel_ok ? "OK" : "FAIL", g_accel_initialized, g_accel_connected);
    if (!accel_ok) {
        if (error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "G-Sensor offline");
        error_count++;
    }
#else
    Serial.println("[SYSTEM CHECK] G-Sensor disabled - skipping accelerometer check");
#endif

#if ENABLE_OBD_CAN
    // Check OBD-II CAN bus (TWAI -> TJA1051T -> ECU)
    if (!g_obd_initialized) {
        obd_can_ok = false;
        Serial.println("[SYSTEM CHECK] OBD CAN: FAIL (not initialized)");
        if (error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "OBD CAN offline");
        error_count++;
    } else if (g_obd_success_count == 0) {
        obd_can_ok = false;
        Serial.printf("[SYSTEM CHECK] OBD CAN: FAIL (init=1, ecu_resp=0, tx_err=%lu/%lu)\n",
                      g_obd_tx_error_count, g_obd_tx_total_count);
        if (error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "OBD no ECU response");
        error_count++;
    } else {
        Serial.printf("[SYSTEM CHECK] OBD CAN: OK (resp=%lu, tx_err=%lu/%lu)\n",
                      g_obd_success_count, g_obd_tx_error_count, g_obd_tx_total_count);
    }
#else
    Serial.println("[SYSTEM CHECK] OBD CAN disabled - skipping");
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
    g_prev_system_status.accel_ok = accel_ok;
    g_prev_system_status.obd_can_ok = obd_can_ok;
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
    bool accel_ok = true;
    bool obd_can_ok = true;
    
    // Build list of NEW failures (things that just went offline)
    char error_msg[320] = "";
    int new_error_count = 0;
    int recovery_count = 0;  // Track recoveries to show green toast
    char recovery_msg[320] = "";
    
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

#if ENABLE_GSENSOR
    // Accelerometer runtime probe
    accel_ok = isAccelerometerConnected();
    g_accel_connected = accel_ok;  // Update global state
    
    if (!accel_ok && g_prev_system_status.accel_ok) {
        if (new_error_count > 0) strcat(error_msg, "\n");
        strcat(error_msg, "G-Sensor offline");
        new_error_count++;
        Serial.println("[MONITOR] G-Sensor went OFFLINE");
    } else if (accel_ok && !g_prev_system_status.accel_ok) {
        if (recovery_count > 0) strcat(recovery_msg, "\n");
        strcat(recovery_msg, "G-Sensor back online");
        recovery_count++;
        Serial.println("[MONITOR] G-Sensor RECOVERED");
    }
#endif

#if ENABLE_OBD_CAN
    // OBD-II CAN bus monitoring
    obd_can_ok = g_obd_initialized && (g_obd_success_count > 0);
    if (!obd_can_ok && g_prev_system_status.obd_can_ok) {
        if (new_error_count > 0) strcat(error_msg, "\n");
        if (!g_obd_initialized) {
            strcat(error_msg, "OBD CAN offline");
        } else {
            strcat(error_msg, "OBD no ECU response");
        }
        new_error_count++;
        Serial.println("[MONITOR] OBD CAN went OFFLINE");
    } else if (obd_can_ok && !g_prev_system_status.obd_can_ok) {
        if (recovery_count > 0) strcat(recovery_msg, "\n");
        strcat(recovery_msg, "OBD CAN back online");
        recovery_count++;
        Serial.println("[MONITOR] OBD CAN RECOVERED");
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
    g_prev_system_status.accel_ok = accel_ok;
    g_prev_system_status.obd_can_ok = obd_can_ok;
    
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

    // Manual brightness toggle - switches to opposite of current state
    // Also sets manual_override flag to prevent auto-brightness from changing it
    if (g_brightness_level >= 200) {
        // Currently bright -> go dim
        setBrightness(BRIGHTNESS_NIGHT);
        g_auto_brightness.manual_override = true;
        Serial.printf("[UI] Single-tap: Manual override -> Night (%d%%)\n", (BRIGHTNESS_NIGHT * 100) / 255);
    }
    else {
        // Currently dim -> go bright
        setBrightness(BRIGHTNESS_DAY);
        g_auto_brightness.manual_override = true;
        Serial.printf("[UI] Single-tap: Manual override -> Day (%d%%)\n", (BRIGHTNESS_DAY * 100) / 255);
    }
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
                // Re-enable buttons when utility box is visible
                if (files_btn) {
                    lv_obj_add_flag(files_btn, LV_OBJ_FLAG_CLICKABLE);
                }
                if (auto_bri_btn) {
                    lv_obj_add_flag(auto_bri_btn, LV_OBJ_FLAG_CLICKABLE);
                }
            }
            else {
                lv_obj_set_style_opa(utility_box, LV_OPA_TRANSP, 0);
                // Disable buttons when utility box is hidden
                if (files_btn) {
                    lv_obj_remove_flag(files_btn, LV_OBJ_FLAG_CLICKABLE);
                }
                if (auto_bri_btn) {
                    lv_obj_remove_flag(auto_bri_btn, LV_OBJ_FLAG_CLICKABLE);
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

//=================================================================
// AUTO BRIGHTNESS BUTTON CALLBACKS
//=================================================================
static void auto_bri_btn_press_cb(lv_event_t* e) {
    LV_UNUSED(e);
    // Visual feedback: change to darker gray on press
    if (auto_bri_btn) {
        lv_obj_set_style_bg_color(auto_bri_btn, lv_color_hex(0x8B9A9A), 0);
    }
}

static void auto_bri_btn_release_cb(lv_event_t* e) {
    LV_UNUSED(e);
    // Visual feedback: restore original gray
    if (auto_bri_btn) {
        lv_obj_set_style_bg_color(auto_bri_btn, lv_color_hex(0xC1CDCD), 0);
    }
}

static void auto_bri_btn_click_cb(lv_event_t* e) {
    LV_UNUSED(e);
    
    // Toggle auto brightness
    g_auto_brightness.enabled = !g_auto_brightness.enabled;
    
    // Save preference
    saveAutoBrightnessPreference();
    
    if (g_auto_brightness.enabled) {
        // Just enabled - clear manual override and apply current time-based setting
        g_auto_brightness.manual_override = false;
        autoBrightnessForceUpdate();  // This calculates times AND updates button text
        Serial.println("[UI] Auto Brightness button tapped - ENABLED");
    } else {
        // Just disabled - set to full brightness
        setBrightness(BRIGHTNESS_DAY);
        updateAutoBrightnessButtonText();  // Show "DIM: MANUAL"
        Serial.println("[UI] Auto Brightness button tapped - DISABLED (100%)");
    }
}

// Called periodically to check for long press
static void checkUtilityLongPress() {
    if (g_utility_press_start > 0 && !g_utility_long_press_triggered) {
        uint32_t held_time = millis() - g_utility_press_start;

        if (held_time >= DEMO_MODE_TOGGLE_HOLD_MS) {
            // v6.13: PHANTOM-TOUCH GUARD. Demo mode is a bench-only feature; a stray
            // 5-second capacitive touch on the road once dropped the display into DEMO
            // mid-drive, showing simulated temps climbing to critical — looked like a
            // real emergency. Never allow LIVE->DEMO while the engine is turning; still
            // allow DEMO->LIVE so you can always get back to real data.
            bool engine_running = g_vehicle_data.rpm_valid && g_vehicle_data.rpm > 300;
            if (!g_demo_mode && engine_running) {
                g_utility_long_press_triggered = true;   // consume the hold, do nothing
                Serial.printf("[MODE] Demo toggle IGNORED - engine running (rpm=%d); likely a phantom touch\n",
                              g_vehicle_data.rpm);
                return;
            }
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

// ---------------------------------------------------------------------------
// v6.10: FUEL TRUST tap popup — shows exactly how the 0-100% score is derived
// (both banks' trims, the four penalties, plain-English meaning, and DTCs).
// Tap the Fuel Trust value to open; tap anywhere to close.
// ---------------------------------------------------------------------------
#if ENABLE_OBD_CAN
static lv_obj_t* g_ft_popup = NULL;

static void ft_popup_dismiss_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (g_ft_popup) { lv_obj_delete(g_ft_popup); g_ft_popup = NULL; }
}

// Short descriptions for the trouble codes this catless/tuned VQ37 tends to set.
// Unknown codes just show the raw code (still fully transparent).
static const char* dtcDescription(const char* c) {
    static const struct { const char* code; const char* desc; } tbl[] = {
        {"P010B","MAF 'B' range/perf"}, {"P0101","MAF range/perf"},
        {"P0102","MAF flow low"},       {"P0103","MAF flow high"},
        {"P0171","Sys too lean B1"},    {"P0174","Sys too lean B2"},
        {"P0172","Sys too rich B1"},    {"P0175","Sys too rich B2"},
        {"P0037","O2 htr low B1S2"},    {"P0038","O2 htr high B1S2"},
        {"P0057","O2 htr low B2S2"},    {"P0058","O2 htr high B2S2"},
        {"P0130","O2 circuit B1S1"},    {"P0150","O2 circuit B2S1"},
        {"P0138","O2 high B1S2"},       {"P0158","O2 high B2S2"},
        {"P2096","Post-cat lean B1"},   {"P2098","Post-cat lean B2"},
        {"P2097","Post-cat rich B1"},   {"P2099","Post-cat rich B2"},
        {"P0420","Catalyst eff B1"},    {"P0430","Catalyst eff B2"},
    };
    for (unsigned i = 0; i < sizeof(tbl) / sizeof(tbl[0]); i++)
        if (strcmp(c, tbl[i].code) == 0) return tbl[i].desc;
    return "";
}

// v6.15: long-press on the popup's RESET BASELINE button wipes the learned cool map.
// Closes the popup and confirms with a toast so the reset is unmistakable.
static void pwr_reset_longpress_cb(lv_event_t* e) {
    LV_UNUSED(e);
    pwrResetBaseline("long-press in popup");
    if (g_ft_popup) { lv_obj_delete(g_ft_popup); g_ft_popup = NULL; }
    showToast("POWER baseline reset - relearns on cool WOT", TOAST_COLOR_SUCCESS, TOAST_SUCCESS_MS);
}

static void showFuelTrustPopup() {
    if (!ui_Screen1) return;
    if (g_ft_popup) { lv_obj_delete(g_ft_popup); g_ft_popup = NULL; }

    // Full-screen dark backdrop on the top layer; a tap anywhere dismisses it.
    lv_obj_t* top = lv_layer_top();
    g_ft_popup = lv_obj_create(top);
    lv_obj_remove_style_all(g_ft_popup);
    lv_obj_set_size(g_ft_popup, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_set_pos(g_ft_popup, 0, 0);
    lv_obj_set_style_bg_color(g_ft_popup, lv_color_hex(0x0A0A0A), 0);
    lv_obj_set_style_bg_opa(g_ft_popup, LV_OPA_90, 0);
    lv_obj_add_flag(g_ft_popup, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(g_ft_popup, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(g_ft_popup, ft_popup_dismiss_cb, LV_EVENT_CLICKED, NULL);

    // Title + hint
    int score = g_vehicle_data.fuel_trust_valid ? g_vehicle_data.fuel_trust_percent : g_ft.score;
    lv_obj_t* title = lv_label_create(g_ft_popup);
    { char t[48]; snprintf(t, sizeof(t), "FUEL TRUST   %d%%", score); lv_label_set_text(title, t); }
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

    lv_obj_t* hint = lv_label_create(g_ft_popup);
    lv_label_set_text(hint, "tap to close");
    lv_obj_set_style_text_color(hint, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 40);

    // Left column: live trims + score math + meaning
    char b[900];
    if (g_ft.valid) {
        float avgLTsigned = 0.5f * (g_ft.lt1 + g_ft.lt2);
        const char* dir = (avgLTsigned < -0.05f) ? "rich (ECU pulling fuel out)"
                        : (avgLTsigned >  0.05f) ? "lean (ECU adding fuel)"
                        : "balanced";
        snprintf(b, sizeof(b),
            "LIVE FUEL TRIMS (OBD)\n"
            "  STFT   B1 %+.1f%%   B2 %+.1f%%\n"
            "  LTFT   B1 %+.1f%%   B2 %+.1f%%\n"
            "  Timing %.1f deg\n"
            "\n"
            "SCORE = 100 - penalties\n"
            "  STFT avg %.1f%%  ->  -%.0f\n"
            "  LTFT avg %.1f%%  ->  -%.0f\n"
            "  Bank split ST %.1f / LT %.1f  ->  -%.0f\n"
#if FUEL_TRUST_TIMING_MODE
            "  Timing vs cool %+.1f deg  ->  -%.0f\n"
#else
            "  Timing pulls x%d  ->  -%.0f\n"
#endif
            "  --------------------------\n"
            "  = %d%%\n"
            "\n"
            "MEANING\n"
            "  Avg LTFT %+.1f%% -> running %s.\n"
            "  Past +/-10%% is worth a look.",
            g_ft.st1, g_ft.st2, g_ft.lt1, g_ft.lt2,
            isnan(g_ft.timing_deg) ? 0.0f : g_ft.timing_deg,
            g_ft.avgAbsST, g_ft.penST,
            g_ft.avgAbsLT, g_ft.penLT,
            g_ft.deltaST, g_ft.deltaLT, g_ft.penBank,
#if FUEL_TRUST_TIMING_MODE
            g_pwr.delta_valid ? g_pwr.tim_delta : 0.0f, g_ft.penTiming,
#else
            g_ft.timingPulls, g_ft.penTiming,
#endif
            g_ft.score, avgLTsigned, dir);
    } else {
        snprintf(b, sizeof(b),
            "LIVE FUEL TRIMS (OBD)\n"
            "  Waiting for warmed-up data.\n"
            "\n"
            "  Fuel Trust appears once coolant\n"
            "  is >= 80 C and the ECU reports\n"
            "  fuel trims. The score = 100 minus\n"
            "  penalties for STFT, LTFT, bank\n"
            "  imbalance, and timing pulls.");
    }
    lv_obj_t* body = lv_label_create(g_ft_popup);
    lv_label_set_text(body, b);
    lv_obj_set_style_text_color(body, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(body, &lv_font_montserrat_14, 0);
    lv_obj_align(body, LV_ALIGN_TOP_LEFT, 36, 72);

    // Right column: check-engine lamp + stored trouble codes
    char d[600]; int m = 0;
    m += snprintf(d + m, sizeof(d) - m, "CHECK-ENGINE\n  MIL %s",
                  g_dtc01_valid ? (g_mil_on ? "ON" : "off") : "?");
    if (g_dtc01_valid) m += snprintf(d + m, sizeof(d) - m, ",  %d stored", g_dtc_count_01);
    m += snprintf(d + m, sizeof(d) - m, "\n\n");
    if (g_dtc_list_valid && g_dtc_list_count > 0) {
        for (int i = 0; i < g_dtc_list_count && i < 8; i++) {
            const char* desc = dtcDescription(g_dtc_list[i]);
            m += snprintf(d + m, sizeof(d) - m, "  %s %s\n", g_dtc_list[i], desc);
        }
    } else if (g_dtc01_valid && g_dtc_count_01 == 0) {
        m += snprintf(d + m, sizeof(d) - m, "  no stored codes");
    } else {
        m += snprintf(d + m, sizeof(d) - m, "  reading codes...");
    }
    lv_obj_t* dtc = lv_label_create(g_ft_popup);
    lv_label_set_text(dtc, d);
    lv_obj_set_style_text_color(dtc, lv_color_hex(0xFFC04D), 0);   // amber, like a MIL
    lv_obj_set_style_text_font(dtc, &lv_font_montserrat_14, 0);
    lv_obj_align(dtc, LV_ALIGN_TOP_LEFT, 470, 72);

    // v6.15: POWER block (Heat Derate Monitor) under the trouble codes
    {
        char p[700]; int n = 0;
        const int oil_f = g_vehicle_data.oil_temp_valid ? g_vehicle_data.oil_temp_value_f : -1;
        const int iat_f = g_vehicle_data.intake_air_temp_valid ? g_vehicle_data.intake_air_temp_f : -1;
        const int ect_f = g_vehicle_data.water_temp_valid ? g_vehicle_data.water_temp_value_f : -1;
        int ready = 0;
        for (int i = 0; i < PWR_RPM_BINS; i++) if (g_pwr_base.bin[i].n >= PWR_BIN_MIN_N) ready++;
        n += snprintf(p + n, sizeof(p) - n, "POWER   %s%s%s\n", PWR_STATE_NAME[g_pwr.state],
                      g_pwr.reason[0] ? "  " : "", g_pwr.reason);
        if (g_pwr.delta_valid)
            n += snprintf(p + n, sizeof(p) - n, "  timing %+.1f vs cool  (bin %d n=%d: %.1f now / %.1f cool)\n",
                          g_pwr.tim_delta, PWR_RPM_BIN_MIN + g_pwr.bin * PWR_RPM_BIN_SIZE, g_pwr.bin_n,
                          g_pwr.tim_live, g_pwr.tim_base);
        else
            n += snprintf(p + n, sizeof(p) - n, "  timing: baseline %d/%d bins ready, learning on cool WOT\n", ready, PWR_RPM_BINS);
        if (g_pwr.pedal_src)
            n += snprintf(p + n, sizeof(p) - n, "  pedal %d%% (%s)  plate %d%%  gap %+d vs cool\n",
                          g_pwr.pedal_pct, g_pwr.pedal_src == 1 ? "PID 49" : "CAN 180",
                          g_vehicle_data.throttle_valid ? g_vehicle_data.throttle_pct : -1, g_pwr.gap_delta);
        else
            n += snprintf(p + n, sizeof(p) - n, "  pedal: no source (PID 0x49 / CAN 0x180 silent)\n");
        n += snprintf(p + n, sizeof(p) - n, "  load %+d%% vs cool   rev cap %s%d (x%d)\n",
                      g_pwr.delta_valid ? g_pwr.load_delta : 0, g_pwr.rev_cap_rpm ? "" : "none ",
                      g_pwr.rev_cap_rpm, g_pwr.rev_cap_count);
        n += snprintf(p + n, sizeof(p) - n, "  air %+.1f%%  IAT %dF  baro %d kPa\n", -g_pwr.air_loss_pct, iat_f,
                      g_vehicle_data.baro_valid ? g_vehicle_data.baro_kpa : -1);
        n += snprintf(p + n, sizeof(p) - n, "  temps  oil %d  IAT %d  ECT %d  -> %s\n", oil_f, iat_f, ect_f,
                      pwrAttribution(oil_f, iat_f, ect_f));
        if (g_vehicle_data.atf_valid)
            n += snprintf(p + n, sizeof(p) - n, "  ATF (TCM, verify) %d / %dF  slip %d rpm", g_vehicle_data.atf1_f,
                          g_vehicle_data.atf2_f, g_vehicle_data.tcc_slip_rpm);
        else
            n += snprintf(p + n, sizeof(p) - n, "  ATF (TCM): no reply yet");
        lv_obj_t* pw = lv_label_create(g_ft_popup);
        lv_label_set_text(pw, p);
        lv_obj_set_style_text_color(pw, g_pwr.sev == 2 ? lv_color_hex(0xFF6F5E) :
                                        g_pwr.sev == 1 ? lv_color_hex(0xF2A93B) : lv_color_hex(0x9FD8FF), 0);
        lv_obj_set_style_text_font(pw, &lv_font_montserrat_14, 0);
        lv_obj_align(pw, LV_ALIGN_TOP_LEFT, 470, 236);

        // long-press (1.5 s) button: wipe the learned baseline. A plain tap does nothing,
        // so a stray touch can't erase it.
        lv_obj_t* btn = lv_btn_create(g_ft_popup);   // same API the file browser uses
        lv_obj_set_size(btn, 200, 34);
        lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT, -24, -16);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x333333), 0);
        lv_obj_add_event_cb(btn, pwr_reset_longpress_cb, LV_EVENT_LONG_PRESSED, NULL);
        lv_obj_t* bl = lv_label_create(btn);
        lv_label_set_text(bl, "hold: RESET BASELINE");
        lv_obj_set_style_text_font(bl, &lv_font_montserrat_14, 0);
        lv_obj_center(bl);
    }

    Serial.println("[UI] Fuel Trust popup opened");
}

static void fuel_trust_tap_cb(lv_event_t* e) {
    LV_UNUSED(e);
    // Kick a fresh DTC read so the codes shown are current, then open the popup.
    if (g_obd_initialized) { sendOBD_Mode03(); g_dtc_last_read_ms = millis(); }
    showFuelTrustPopup();
}

// ---------------------------------------------------------------------------
// v6.15: POWER banner — top-layer strip (same pattern as the DEMO banner), shown only
// when the Heat Derate Monitor state is not OK. Amber = AIR/LIFT/THROTTLE/TIMING(-3),
// red = TIMING(-6)/FUEL?/REV CAP. Tap = acknowledge (hides until a worse state or a
// different one) + opens the popup. Re-arms automatically when the state returns to OK.
// ---------------------------------------------------------------------------
static lv_obj_t* g_pwr_banner = NULL;

static void pwr_banner_tap_cb(lv_event_t* e) {
    LV_UNUSED(e);
    g_pwr.ack_state = g_pwr.state; g_pwr.ack_sev = g_pwr.sev;
    if (g_pwr_banner) lv_obj_add_flag(g_pwr_banner, LV_OBJ_FLAG_HIDDEN);
    g_pwr.banner_shown = false;
    Serial.printf("[PWR] banner acknowledged (%s)\n", PWR_STATE_NAME[g_pwr.state]);
    showFuelTrustPopup();
}

static void updatePowerBanner() {
    const bool want = !g_demo_mode && g_pwr.state != PWR_OK &&
                      !(g_pwr.state == g_pwr.ack_state && g_pwr.sev <= g_pwr.ack_sev);
    if (!want) {
        if (g_pwr_banner && g_pwr.banner_shown) { lv_obj_add_flag(g_pwr_banner, LV_OBJ_FLAG_HIDDEN); g_pwr.banner_shown = false; }
        return;
    }
    if (!g_pwr_banner) {
        lv_obj_t* top = lv_layer_top();
        g_pwr_banner = lv_label_create(top);
        lv_obj_set_style_bg_opa(g_pwr_banner, LV_OPA_COVER, 0);
        lv_obj_set_style_text_font(g_pwr_banner, &lv_font_montserrat_20, 0);
        lv_obj_set_style_pad_ver(g_pwr_banner, 5, 0);
        lv_obj_set_style_pad_hor(g_pwr_banner, 40, 0);
        lv_obj_set_width(g_pwr_banner, lv_pct(100));
        lv_obj_set_style_text_align(g_pwr_banner, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(g_pwr_banner, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_add_flag(g_pwr_banner, LV_OBJ_FLAG_IGNORE_LAYOUT);
        lv_obj_add_flag(g_pwr_banner, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(g_pwr_banner, pwr_banner_tap_cb, LV_EVENT_CLICKED, NULL);
    }
    static int last_state = -1, last_sev = -1; static char last_reason[48] = "";
    if (last_state != g_pwr.state || last_sev != g_pwr.sev || strcmp(last_reason, g_pwr.reason) != 0) {
        last_state = g_pwr.state; last_sev = g_pwr.sev;
        strncpy(last_reason, g_pwr.reason, sizeof(last_reason) - 1); last_reason[sizeof(last_reason) - 1] = '\0';
        char t[80];
        snprintf(t, sizeof(t), "PWR  %s", g_pwr.reason[0] ? g_pwr.reason : PWR_STATE_NAME[g_pwr.state]);
        lv_label_set_text(g_pwr_banner, t);
        if (g_pwr.sev >= 2) {
            lv_obj_set_style_bg_color(g_pwr_banner, lv_color_hex(0xE03A2E), 0);
            lv_obj_set_style_text_color(g_pwr_banner, lv_color_hex(0xFFFFFF), 0);
        } else {
            lv_obj_set_style_bg_color(g_pwr_banner, lv_color_hex(0xF2A93B), 0);
            lv_obj_set_style_text_color(g_pwr_banner, lv_color_hex(0x000000), 0);
        }
    }
    if (!g_pwr.banner_shown) {
        lv_obj_clear_flag(g_pwr_banner, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(g_pwr_banner);
        g_pwr.banner_shown = true;
    }
}
#else
static void fuel_trust_tap_cb(lv_event_t* e) { LV_UNUSED(e); }  // OBD disabled: no-op
#endif // ENABLE_OBD_CAN

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
    // v6.10: FUEL TRUST tap opens the score-breakdown popup (not unit cycling)
    configureTapPanel(ui_FUEL_TRUST_Value_Tap_Panel, fuel_trust_tap_cb);

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
    // EXIO_CAN_SEL HIGH routes the FSUSB42UMX mux to the onboard TJA1051T CAN transceiver.
    // Setting it HIGH disconnects the NATIVE USB port (GPIO19/20).
    // CAN_MUX_TO_CAN 1 = drive EXIO5 HIGH so the FSUSB42UMX mux connects GPIO19/20 to the
    // onboard TJA1051T CAN transceiver. REQUIRED for OBD CAN to produce data. Side effect:
    // native USB goes dark (USB-MSC can't run while CAN is active) — flash/serial over the
    // separate UART USB-C port. Set back to 0 only if you need native USB/USB-MSC and can
    // live without CAN.
    #define CAN_MUX_TO_CAN 1
#if CAN_MUX_TO_CAN
    g_exio_state = (1u << EXIO_TP_RST) | (1u << EXIO_SD_CS) | (1u << EXIO_CAN_SEL);
#else
    g_exio_state = (1u << EXIO_TP_RST) | (1u << EXIO_SD_CS);   // USB mode (CAN off)
#endif
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
    // Note: 3000ms needed for reliable cold-boot initialization
    vTaskDelay(pdMS_TO_TICKS(10000));
    
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
        float temp_pan_disp = tempToDisplay((float)g_vehicle_data.oil_temp_value_f, g_oil_temp_unit);
        // Single temperature value per gauge

        if (smooth_oil_temp_f < 0) {
            smooth_oil_temp_f = temp_pan_disp;
        }
        else {
            smooth_oil_temp_f = smooth_oil_temp_f * (1.0f - SMOOTH_FACTOR) + temp_pan_disp * SMOOTH_FACTOR;
        }

#if ENABLE_BARS
        if (ui_OIL_TEMP_Bar) {
            lv_bar_set_value(ui_OIL_TEMP_Bar, g_vehicle_data.oil_temp_value_f, LV_ANIM_OFF);
            bool critical = (g_vehicle_data.oil_temp_value_f > OIL_TEMP_ValueCriticalF);
            lv_obj_set_style_bg_color(ui_OIL_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(1, g_vehicle_data.oil_temp_value_f);
#endif

        if (ui_OIL_TEMP_Value) {
            int pan_display_val = (int)(temp_pan_disp + 0.5f);
            if (pan_display_val != last_oil_temp_pan_display || oil_temp_unit_changed) {
                snprintf(buf, sizeof(buf), "%d°%s", pan_display_val, oilTempUnit);
                lv_label_set_text(ui_OIL_TEMP_Value, buf);
                last_oil_temp_pan_display = pan_display_val;
            }

            // Style label text color based on critical
            bool value_critical = (g_vehicle_data.oil_temp_value_f > OIL_TEMP_ValueCriticalF);
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
        bool oil_temp_critical = (g_vehicle_data.oil_temp_value_f > OIL_TEMP_ValueCriticalF);
        if (oil_temp_critical != g_oil_temp_panel_was_critical) {
            if (ui_OIL_TEMP_Value_Tap_Panel) {
                lv_obj_set_style_bg_opa(ui_OIL_TEMP_Value_Tap_Panel, 
                    oil_temp_critical ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            }
            g_oil_temp_panel_was_critical = oil_temp_critical;
        }

#if ENABLE_VALUE_CRITICAL
        bool critical = (g_vehicle_data.oil_temp_value_f > OIL_TEMP_ValueCriticalF);
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
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.water_temp_value_f, g_water_temp_unit);

        if (ui_W_TEMP_Value) {
            int hot_display_val = (int)(temp_hot_disp + 0.5f);
            if (hot_display_val != last_water_temp_hot_display || water_temp_unit_changed) {
                snprintf(buf, sizeof(buf), "%d°%s", hot_display_val, waterTempUnit);
                lv_label_set_text(ui_W_TEMP_Value, buf);
                last_water_temp_hot_display = hot_display_val;
            }

            // Style label text color based on critical
            bool value_critical = (g_vehicle_data.water_temp_value_f > W_TEMP_ValueCritical_F);
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
        bool water_temp_critical = (g_vehicle_data.water_temp_value_f > W_TEMP_ValueCritical_F);
        if (water_temp_critical != g_water_temp_panel_was_critical) {
            if (ui_W_TEMP_Value_Tap_Panel) {
                lv_obj_set_style_bg_opa(ui_W_TEMP_Value_Tap_Panel,
                    water_temp_critical ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            }
            g_water_temp_panel_was_critical = water_temp_critical;
        }
#if ENABLE_BARS
        if (ui_W_TEMP_Bar) {
            lv_bar_set_value(ui_W_TEMP_Bar, g_vehicle_data.water_temp_value_f, LV_ANIM_OFF);
            bool critical = (g_vehicle_data.water_temp_value_f > W_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_W_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(2, g_vehicle_data.water_temp_value_f);
#endif

#if ENABLE_VALUE_CRITICAL
        bool critical = (g_vehicle_data.water_temp_value_f > W_TEMP_ValueCritical_F);
        updateCriticalLabel(ui_W_TEMP_VALUE_CRITICAL_Label, critical,
            &water_temp_was_critical, &water_temp_visible, &water_temp_exit_time, &water_temp_last_blink);
#endif
    }

    // Handle transition to invalid state for water temperature (OBD stale / disconnected)
    static bool water_temp_was_data_valid = true;
    if (!g_vehicle_data.water_temp_valid && water_temp_was_data_valid) {
        // Just became invalid - update UI once to show "---" with unit
        const char* waterTempUnit = getTempUnitStr(g_water_temp_unit);
        char buf[32];
        if (ui_W_TEMP_Value) {
            snprintf(buf, sizeof(buf), "---°%s", waterTempUnit);
            lv_label_set_text(ui_W_TEMP_Value, buf);
            lv_obj_set_style_text_color(ui_W_TEMP_Value, lv_color_hex(0xFFFFFF), 0);
        }

        // Reset panel background
        if (ui_W_TEMP_Value_Tap_Panel) {
            lv_obj_set_style_bg_opa(ui_W_TEMP_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        }
        g_water_temp_panel_was_critical = false;

        // Hide critical label immediately
#if ENABLE_VALUE_CRITICAL
        if (ui_W_TEMP_VALUE_CRITICAL_Label) {
            lv_obj_set_style_text_opa(ui_W_TEMP_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_W_TEMP_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
        }
        water_temp_was_critical = false;
        water_temp_visible = false;
        water_temp_exit_time = 0;
#endif

#if ENABLE_LIGHTWEIGHT_BARS
        // Reset bar to 0
        updateLightweightBar(2, 0);
#endif

        // Reset last display value
        last_water_temp_hot_display = -9999;
    }
    water_temp_was_data_valid = g_vehicle_data.water_temp_valid;

    // ----- Trans Temperature -----
    if (g_vehicle_data.trans_temp_valid) {
        const char* transTempUnit = getTempUnitStr(g_trans_temp_unit);
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.trans_temp_value_f, g_trans_temp_unit);

        if (ui_TRAN_TEMP_Value) {
            int hot_display_val = (int)(temp_hot_disp + 0.5f);
            if (hot_display_val != last_trans_temp_hot_display || trans_temp_unit_changed) {
                snprintf(buf, sizeof(buf), "%d°%s", hot_display_val, transTempUnit);
                lv_label_set_text(ui_TRAN_TEMP_Value, buf);
                last_trans_temp_hot_display = hot_display_val;
            }

            // Style label text color based on critical
            bool value_critical = (g_vehicle_data.trans_temp_value_f > TRAN_TEMP_ValueCritical_F);
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
        bool trans_temp_critical = (g_vehicle_data.trans_temp_value_f > TRAN_TEMP_ValueCritical_F);
        if (trans_temp_critical != g_trans_temp_panel_was_critical) {
            if (ui_TRAN_TEMP_Value_Tap_Panel) {
                lv_obj_set_style_bg_opa(ui_TRAN_TEMP_Value_Tap_Panel,
                    trans_temp_critical ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            }
            g_trans_temp_panel_was_critical = trans_temp_critical;
        }
#if ENABLE_BARS
        if (ui_TRAN_TEMP_Bar) {
            lv_bar_set_value(ui_TRAN_TEMP_Bar, g_vehicle_data.trans_temp_value_f, LV_ANIM_OFF);
            bool critical = (g_vehicle_data.trans_temp_value_f > TRAN_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_TRAN_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(3, g_vehicle_data.trans_temp_value_f);
#endif

#if ENABLE_VALUE_CRITICAL
        bool critical = (g_vehicle_data.trans_temp_value_f > TRAN_TEMP_ValueCritical_F);
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
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.steer_temp_value_f, g_steer_temp_unit);

        if (ui_STEER_TEMP_Value) {
            int hot_display_val = (int)(temp_hot_disp + 0.5f);
            if (hot_display_val != last_steer_temp_hot_display || steer_temp_unit_changed) {
                snprintf(buf, sizeof(buf), "%d°%s", hot_display_val, steerTempUnit);
                lv_label_set_text(ui_STEER_TEMP_Value, buf);
                last_steer_temp_hot_display = hot_display_val;
            }

            // Style label text color based on critical
            bool value_critical = (g_vehicle_data.steer_temp_value_f > STEER_TEMP_ValueCritical_F);
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
        bool steer_temp_critical = (g_vehicle_data.steer_temp_value_f > STEER_TEMP_ValueCritical_F);
        if (steer_temp_critical != g_steer_temp_panel_was_critical) {
            if (ui_STEER_TEMP_Value_Tap_Panel) {
                lv_obj_set_style_bg_opa(ui_STEER_TEMP_Value_Tap_Panel,
                    steer_temp_critical ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            }
            g_steer_temp_panel_was_critical = steer_temp_critical;
        }
#if ENABLE_BARS
        if (ui_STEER_TEMP_Bar) {
            lv_bar_set_value(ui_STEER_TEMP_Bar, g_vehicle_data.steer_temp_value_f, LV_ANIM_OFF);
            bool critical = (g_vehicle_data.steer_temp_value_f > STEER_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_STEER_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(4, g_vehicle_data.steer_temp_value_f);
#endif

#if ENABLE_VALUE_CRITICAL
        bool critical = (g_vehicle_data.steer_temp_value_f > STEER_TEMP_ValueCritical_F);
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
        float temp_hot_disp = tempToDisplay((float)g_vehicle_data.diff_temp_value_f, g_diff_temp_unit);

        if (ui_DIFF_TEMP_Value) {
            int hot_display_val = (int)(temp_hot_disp + 0.5f);
            if (hot_display_val != last_diff_temp_hot_display || diff_temp_unit_changed) {
                snprintf(buf, sizeof(buf), "%d°%s", hot_display_val, diffTempUnit);
                lv_label_set_text(ui_DIFF_TEMP_Value, buf);
                last_diff_temp_hot_display = hot_display_val;
            }

            // Style label text color based on critical
            bool value_critical = (g_vehicle_data.diff_temp_value_f > DIFF_TEMP_ValueCritical_F);
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
        bool diff_temp_critical = (g_vehicle_data.diff_temp_value_f > DIFF_TEMP_ValueCritical_F);
        if (diff_temp_critical != g_diff_temp_panel_was_critical) {
            if (ui_DIFF_TEMP_Value_Tap_Panel) {
                lv_obj_set_style_bg_opa(ui_DIFF_TEMP_Value_Tap_Panel,
                    diff_temp_critical ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
            }
            g_diff_temp_panel_was_critical = diff_temp_critical;
        }
#if ENABLE_BARS
        if (ui_DIFF_TEMP_Bar) {
            lv_bar_set_value(ui_DIFF_TEMP_Bar, g_vehicle_data.diff_temp_value_f, LV_ANIM_OFF);
            bool critical = (g_vehicle_data.diff_temp_value_f > DIFF_TEMP_ValueCritical_F);
            lv_obj_set_style_bg_color(ui_DIFF_TEMP_Bar,
                critical ? lv_color_hex(hexRed) : lv_color_hex(hexOrange),
                LV_PART_INDICATOR);
        }
#endif
#if ENABLE_LIGHTWEIGHT_BARS
        if (update_bars_this_frame) updateLightweightBar(5, g_vehicle_data.diff_temp_value_f);
#endif

#if ENABLE_VALUE_CRITICAL
        bool critical = (g_vehicle_data.diff_temp_value_f > DIFF_TEMP_ValueCritical_F);
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

    // Handle transition to invalid state for fuel trust
    // (OBD stale/disconnected, OR engine not yet warmed to OBD_WARMUP_TEMP_C)
    static bool fuel_trust_was_data_valid = true;
    if (!g_vehicle_data.fuel_trust_valid && fuel_trust_was_data_valid) {
        // Just became invalid - show "---" once
        if (ui_FUEL_TRUST_Value) {
            lv_label_set_text(ui_FUEL_TRUST_Value, "--- %");
            lv_obj_set_style_text_color(ui_FUEL_TRUST_Value, lv_color_hex(0xFFFFFF), 0);
        }

        // Reset panel background
        if (ui_FUEL_TRUST_Value_Tap_Panel) {
            lv_obj_set_style_bg_opa(ui_FUEL_TRUST_Value_Tap_Panel, LV_OPA_TRANSP, 0);
        }
        g_fuel_panel_was_critical = false;

        // Hide critical label immediately
#if ENABLE_VALUE_CRITICAL
        if (ui_FUEL_TRUST_VALUE_CRITICAL_Label) {
            lv_obj_set_style_text_opa(ui_FUEL_TRUST_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_FUEL_TRUST_VALUE_CRITICAL_Label, 0, LV_PART_MAIN);
        }
        fuel_trust_was_critical = false;
        fuel_trust_visible = false;
        fuel_trust_exit_time = 0;
#endif

#if ENABLE_LIGHTWEIGHT_BARS
        // Reset bar to 0
        updateLightweightBar(6, 0);
#endif

        // Reset smoothing + last display value
        smooth_fuel_trust = -1.0f;
        last_fuel_trust_display = -9999;
    }
    fuel_trust_was_data_valid = g_vehicle_data.fuel_trust_valid;
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
        oil_temp_sum += g_vehicle_data.oil_temp_value_f;
        oil_temp_samples++;
    }
    if (g_vehicle_data.water_temp_valid) {
        water_temp_sum += g_vehicle_data.water_temp_value_f;
        water_temp_samples++;
    }
    if (g_vehicle_data.trans_temp_valid) {
        transmission_temp_sum += g_vehicle_data.trans_temp_value_f;
        transmission_temp_samples++;
    }
    if (g_vehicle_data.steer_temp_valid) {
        steering_temp_sum += g_vehicle_data.steer_temp_value_f;
        steering_temp_samples++;
    }
    if (g_vehicle_data.diff_temp_valid) {
        differencial_temp_sum += g_vehicle_data.diff_temp_value_f;
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
    Serial.printf("   370zMonitor %s - Dual-Core + G-Sensor\n", FW_VERSION);
    Serial.println("========================================");

    if (psramFound()) {
        Serial.printf("✓ PSRAM: %u bytes total, %u free\n", ESP.getPsramSize(), ESP.getFreePsram());
    }
    else {
        Serial.println("✗ WARNING: No PSRAM detected!");
    }

    // Load unit preferences from flash
    loadUnitPreferences();

    // Load auto brightness preference from flash
    loadAutoBrightnessPreference();

#if ENABLE_OBD_CAN
    // v6.15: learned cool timing baseline (Heat Derate Monitor) from flash
    pwrLoadBaseline();
#endif

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

    // LVGL v9: lv_color_t is RGB888 (3 bytes) regardless of LV_COLOR_DEPTH, but this
    // display renders RGB565 (2 bytes/px). Size the DMA buffer by the render format,
    // NOT sizeof(lv_color_t) — using sizeof here over-allocates 50% (the v9 black-screen bug).
    size_t buf_bytes = LVGL_BUFFER_SIZE * 2;  // RGB565 = 2 bytes per pixel

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

    // Create serial log queue EARLY so initOBD() and initSensors() output reaches SD logs
    // (was previously in [8/8] SD Card section, causing all [OBD] init messages to be dropped)
    g_serial_log_queue = xQueueCreate(SERIAL_LOG_QUEUE_SIZE, sizeof(SerialLogEntry));
    if (!g_serial_log_queue) {
        Serial.println("      WARNING: Serial log queue creation failed!");
    } else {
        Serial.println("      Serial log queue created (early init)");
    }

    Serial.println("[7/8] Data providers...");
    initSensors();
    initOBD();
#if ENABLE_GSENSOR
    if (initAccelerometer()) {
        Serial.println("      G-Sensor: OK");
    } else {
        Serial.println("      G-Sensor: NOT FOUND");
    }
#endif
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
    
    // NOTE: g_serial_log_queue already created before [7/8] Data providers
    // so initOBD()/initSensors() output reaches SD logs
    
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
        lv_obj_set_size(utility_box, 250, 288);  // Added 48px for Auto Brightness button
#else
        lv_obj_set_size(utility_box, 250, 240);  // Height for 9 lines + Auto Brightness button
#endif
#else
#if ENABLE_FILE_BROWSER
        lv_obj_set_size(utility_box, 250, 288);  // Added 48px for Auto Brightness button
#else
        lv_obj_set_size(utility_box, 250, 186);  // Height for 6 lines + Auto Brightness button
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

        // AUTO BRIGHTNESS button (below FILES button or at top if no file browser)
        auto_bri_btn = lv_obj_create(utility_box);
        lv_obj_set_size(auto_bri_btn, 235, 44);
#if ENABLE_FILE_BROWSER
        lv_obj_align(auto_bri_btn, LV_ALIGN_TOP_LEFT, 0, 46);  // Below FILES button
#else
        lv_obj_align(auto_bri_btn, LV_ALIGN_TOP_LEFT, 0, 0);   // At top if no file browser
#endif
        lv_obj_set_style_bg_color(auto_bri_btn, lv_color_hex(0xC1CDCD), 0);  // Same gray as FILES
        lv_obj_set_style_bg_opa(auto_bri_btn, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(auto_bri_btn, 0, 0);
        lv_obj_set_style_border_width(auto_bri_btn, 0, 0);
        lv_obj_set_style_pad_all(auto_bri_btn, 2, 0);
        lv_obj_remove_flag(auto_bri_btn, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);  // Start disabled
        
        // Auto Brightness label inside button (two lines, centered)
        auto_bri_lbl = lv_label_create(auto_bri_btn);
        lv_label_set_text(auto_bri_lbl, g_auto_brightness.enabled ? "DIM:\nAUTO" : "DIM:\nMANUAL");
        lv_obj_set_style_text_color(auto_bri_lbl, lv_color_hex(0x000000), 0);  // Black text
        lv_obj_set_style_text_font(auto_bri_lbl, &lv_font_unscii_16, 0);
        lv_obj_set_style_text_align(auto_bri_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(auto_bri_lbl);
        
        // Event callbacks for auto brightness button
        lv_obj_add_event_cb(auto_bri_btn, auto_bri_btn_press_cb, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(auto_bri_btn, auto_bri_btn_release_cb, LV_EVENT_RELEASED, NULL);
        lv_obj_add_event_cb(auto_bri_btn, auto_bri_btn_click_cb, LV_EVENT_CLICKED, NULL);

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
        int label_y_offset = 96;  // Offset for FILES + Auto Brightness buttons (48 + 48)
#else
        int label_y_offset = 48;  // Offset for Auto Brightness button only
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
    
    // Initialize auto brightness (will start working once RTC/NTP time is available)
    autoBrightnessInit();

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

    // v6.14: arm the task watchdog on the main loop — a hang now auto-resets the board
    // instead of a frozen screen. Version-guarded (the TWDT API differs across ESP-IDF).
#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t twdt_cfg = { .timeout_ms = WDT_TIMEOUT_MS, .idle_core_mask = 0, .trigger_panic = true };
    esp_task_wdt_reconfigure(&twdt_cfg);
#else
    esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
#endif
    if (esp_task_wdt_add(NULL) == ESP_OK) {
        g_wdt_armed = true;
        Serial.printf("[WDT] Task watchdog armed (%d s)\n", WDT_TIMEOUT_MS / 1000);
    } else {
        Serial.println("[WDT] Task watchdog add failed (continuing without it)");
    }

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

// v6.14: if the Modbus PRTXI oil-temp sensor is dead, drive the oil-temp dial from
// the OBD (Mode 22) oil temp instead. Runs between data-gather and UI update so it's
// order-independent. When the physical sensor is healthy, its value wins.
static void mergeOilTempSource() {
#if ENABLE_OBD_CAN
    if (!g_demo_mode && !g_vehicle_data.oil_temp_valid && g_vehicle_data.obd_oil_temp_valid) {
        g_vehicle_data.oil_temp_value_f = g_vehicle_data.obd_oil_temp_f;
        g_vehicle_data.oil_temp_valid   = true;
        g_vehicle_data.has_received_data = true;
    }
#endif
}

// v6.14: optional power-loss flush. OFF by default — needs a resistor divider from
// ignition-switched 12V to POWER_SENSE_PIN (pin sees ~2.7V when 12V is present, and
// collapses on key-off before the 3.3V rail sags). When it sees power go good -> failing
// it flushes the SD once so a mid-write key-off can't corrupt the card. Unwired/floating
// with the flag off = never runs. The watchdog needs no hardware; this is the extra layer.
static void checkPowerFail() {
#if ENABLE_POWER_FAIL_DETECT
    static bool power_was_good = false;
    static bool flushed = false;
    int raw = analogRead(POWER_SENSE_PIN);
    if (raw > POWER_SENSE_GOOD_RAW) { power_was_good = true; flushed = false; }
    else if (power_was_good && !flushed && raw < POWER_SENSE_FAIL_RAW) {
        flushed = true;
        _RealSerial.println("[PWR] Power failing - emergency SD flush");
        g_fb_pause_sd_writes = true;   // halt the writer, then push buffered data to the card
        sdSafeFlush();
    }
#endif
}

//=================================================================
// MAIN LOOP
//=================================================================

void loop() {
    static uint32_t frame_start = 0;
    frame_start = millis();

    if (g_wdt_armed) esp_task_wdt_reset();   // v6.14: feed the watchdog each iteration
    checkPowerFail();                        // v6.14: emergency SD flush on power loss (if enabled)

    static uint32_t last_tick = 0;
    static uint32_t last_status = 0;
    static uint32_t last_update = 0;

    uint32_t now = millis();

    // LVGL tick
    if (last_tick == 0) last_tick = now;
    lv_tick_inc(now - last_tick);
    last_tick = now;

    loop_count++;

    // Auto brightness update (sunrise/sunset based dimming)
    autoBrightnessUpdate();

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
        // v6.7: running session summary line (v6.15: + Heat Derate Monitor peaks)
#if ENABLE_OBD_CAN
        Serial.printf("[SESSION] min_oilP(>2k)=%dpsi peak_oilT=%dF maxG lat=%.2f lon=%.2f vert=%.2f | min_dTiming=%.1f rev_cap=%d max_IAT=%dF max_air_loss=%.1f%% states=0x%02X\n",
            (g_sess.min_oil_psi >= 9999) ? 0 : g_sess.min_oil_psi,
            g_sess.peak_oil_f, g_sess.max_lat_g, g_sess.max_lon_g, g_sess.max_vert_g,
            g_pwr.sess_min_delta, g_pwr.rev_cap_rpm, g_pwr.sess_max_iat_f, g_pwr.sess_max_air_loss,
            g_pwr.sess_states_seen);
#else
        Serial.printf("[SESSION] min_oilP(>2k)=%dpsi peak_oilT=%dF maxG lat=%.2f lon=%.2f vert=%.2f\n",
            (g_sess.min_oil_psi >= 9999) ? 0 : g_sess.min_oil_psi,
            g_sess.peak_oil_f, g_sess.max_lat_g, g_sess.max_lon_g, g_sess.max_vert_g);
#endif
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

        // Step 1.5 (v6.14): OBD oil temp fills in for the dead Modbus oil-temp sensor
        mergeOilTempSource();

        // Step 1.6 (v6.15): Heat Derate Monitor — needs the merged oil temp, feeds the banner + CSV
        updatePowerMonitor();
#if ENABLE_OBD_CAN
        updatePowerBanner();
#endif

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
