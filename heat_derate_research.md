# Heat Derate Monitor — research & design brief

*370zMonitor · 2018 370Z (Z34, VQ37VHR, RE7R01A 7AT) · 2026-09-06*
*Status: research + proposal. No firmware changed. Features below need per-feature authorization.*

---

## 0. Bottom line

1. **"Pulling timing" is only one of four things that make the Z slower when hot, and probably not the biggest one.** In order of likely impact on this car: (a) **air density** — at the 130–145 °F intake temps the box logged on 7/24, ~5–7 % of power is gone before the ECU does anything; (b) **oil-temp "engine protection mode"** — Nissan documents it for MY2009–2019; the FSM says the ECM *restricts VVEL lift and lowers the rev limit* (owners: cap ≈6,000–6,500 rpm around 280 °F on the gauge, then ≈3,500–4,500); (c) **ignition retard** from knock feedback and temperature-correction tables (this is the "pulling timing" part); (d) **transmission protection** — the TCM has its own fail-safe/protection control keyed to ATF temperature.
2. **Yes, it can be built into the app with the tap you already have — no new hardware for the core.** The box already reads timing advance (PID 0x0E), IAT, ECT, load, throttle, rpm, baro, and (v6.14, unflashed) oil temp via Mode 22. What's missing is *how* timing is sampled, a *baseline* to compare against, and two cheap signals (pedal position, TCM ATF temp) that are readable over the same wire.
3. **Today's timing channel can't see a heat pull.** Measured from the 7/24 logs: timing updates once every 3–5 s (0.20–0.25 Hz). A WOT pull lasts 2–7 s → 2–3 samples per pull, paired with load values up to 3 s stale. And the existing `pen_timing` "timing pull" penalty fires in 24.5 % of rows of a 12-minute street drive with only six short pulls (and 9 % of rows in a drive that never exceeded 35 % throttle) — it is measuring operating-point changes 3–5 s apart, not retard.
4. **The design (Part 3):** burst-poll timing at ~8 Hz only during power windows; learn the car's own cool-morning timing map per rpm bin; show Δtiming vs that baseline; add three more derate detectors (rev ceiling, pedal-vs-throttle gap, load deficit) plus an air-density readout so "hot air" and "ECU pulling" are separated; surface it as an event banner + a POWER popup + appended CSV columns + a per-lap/session summary.
5. **What else (Part 4):** fuel-pressure sender on spare AI6 (catches the saddle-tank starvation that mimics heat loss *and* is the physical fuel-pressure test you already planned), TCM ATF temps + torque-converter slip (free), pedal + wheel speeds + steering from passive CAN (free), 10–25 Hz GPS for per-lap "how much slower" numbers, wideband on AI7, BME280 for real density altitude.

---

## 1. What the ECU actually does when the car gets hot

Confidence: **HIGH** = Nissan document read directly · **MED** = FSM text quoted by independent sources, or tuner documentation · **LOW** = owner reports.

| Mechanism | Trigger (confidence) | What changes | How the driver feels it | Signal that reveals it over OBD/CAN |
|---|---|---|---|---|
| **Oil-temp engine protection** ("torque cut control at high engine oil temperature") | Nissan's driver action line is **280 °F** on the gauge, MY2009–2019 (HIGH that it exists; LOW for the ECM's exact trip point). Owners: stage 1 ≈280 °F, stage 2 ≈290 °F (LOW) | ECM **restricts the VVEL operating angle** (less lift → less torque) and **lowers max engine speed** (MED). Owners report caps of ≈6,000–6,500 then ≈3,500–4,500 rpm; clears after cooling + key cycle (LOW) | Weak pull, then a soft limiter well below 7,500 | Oil temp (Mode 22 DID 0x111F, v6.14); rpm ceiling during a pedal-pinned pull; calculated load at fixed rpm+pedal dropping |
| **Ignition retard — knock control** | Knock sensors (2), per-cylinder feedback + learned retard; UpRev also documents a **"High Det" table switch** (ECU swaps to a lower-timing ignition table and a different VVEL table after too many knock events) (MED). Hot oil/coolant/intake air raise knock tendency | Final timing lower than the base map at the same rpm/load | Flat, hesitant WOT | **PID 0x0E** timing advance vs a cool baseline at the same rpm/load bin. Knock counts are NOT on standard PIDs |
| **Ignition retard — temperature corrections** | UpRev/EcuTek expose oil-temp, coolant-temp and IAT ignition-correction tables (MED). Whether the *stock* calibration populates them isn't settled in public docs; the FSM lists corrections at start/warm-up/idle/low voltage/acceleration/knock | Timing reduced as the temperature rises, before any knock | Gradual loss through a session | Same PID 0x0E Δ vs baseline, attributed by which temperature is elevated |
| **Air density** (physics, not ECU) | IAT ↑, baro ↓ | Less air per cycle. SAE J1349 vs 77 °F: **−2.4 % at 100 °F, −4.4 % at 120 °F, −6.0 % at 137 °F, −6.8 % at 145 °F** intake temp | Slower everywhere, no ECU involvement | IAT 0x0F + baro 0x33 (already logged) → correction factor |
| **Coolant** | **No ECT-triggered torque cut is documented** for the VQ37VHR — the FSM fail-safe chart has no high-ECT row; P1217 "engine over temperature" is a DTC/MIL only (HIGH). Hot-engine fuel enrichment exists | Nothing explicit; enrichment | — | ECT 0x05, commanded lambda 0x44, fuel-system status 0x03, MIL 0x01 |
| **Transmission (RE7R01A TCM)** | FSM TM section has "Fail-Safe" (TM-285) and "Protection Control" (TM-287) (HIGH that they exist; text not retrieved). Nissan's sibling TCMs reduce "gear shift permission maximum revolution and the maximum torque" when fluid is hot (MED, analog). Owner: limp at ~270 °F ATF (LOW) | Torque-down request to ECM, shift/rev restriction, lock-up changes | A/T CHECK lamp; held gears | **TCM ATF temps via Mode 21 LID 0x20 on 0x7E1** (ScanGauge X-Gauges prove the request works on the 370Z); TCM DTCs on 0x7E9 |
| **Other torque management** | Drive-by-wire: TCS/VDC throttle close, shift torque-down, rev/speed fuel cut, low-battery timing correction; **CAN-comm loss (U1000 etc.) → 3,500 rpm VVEL limp** (HIGH) | Throttle plate < pedal; fuel cut | Pedal ≠ response | Pedal (0x49/0x4A or CAN 0x180 byte F) vs throttle 0x11; rpm plateau; voltage 0x42 |

Points that matter for this car specifically:

- **Oil temp is the mechanism Nissan itself documents.** 2009 owner's manual: *"When the engine oil temperature is high, the engine protection mode, which helps reduce the chance of engine damage, could activate. The engine protection mode automatically decreases engine power."* 2018/2019 QRGs repeat it and set 280 °F as the "reduce engine speed" line. The FSM passage (quoted by two independent posters): *"To avoid VVEL performance, ECM performs the engine torque cut control at high engine oil temperature… This control is to control the VVEL operating angle… engine performance will decrease, then maximum engine speed is reduced a little."* The gauge, the protection logic and DID 0x111F all read the same sensor (the ECM feeds the gauge over CAN).
- **You currently have no oil temp on track.** The PRTXI AI2 sensor is dead and v6.14's Mode-22 fallback is not flashed yet. Flashing v6.14 is prerequisite #1 for any of this.
- **The IAT you log is the bank-1 MAF-housing temperature** (the ECM ignores the bank-2 IAT). On 7/24 it ran 97–145 °F with the "ambient" PID at 70–118 °F — genuine heat soak.
- **Coolant at ~230 °F on 7/6 was not triggering an ECU derate** (none is documented), though it raises knock tendency.
- **Knock counts are not reachable from the box.** EcuTek/UpRev/HP Tuners log them as ROM-address "memory parameters"; no public Mode 22 DID exists. Δtiming vs baseline is the observable proxy.
- **Risk to keep in mind:** the ESP32 is a node on the powertrain CAN. The FSM lists CAN-communication-loss DTCs as a trigger for the 3,500 rpm VVEL limp. Keep request rates bounded and never loop bus-off recovery.

---

## 2. What the box can see today (evidence from the 7/24 logs)

Sessions 592–596, FW v6.12, 10 Hz CSV.

| Finding | Number | Why it matters |
|---|---|---|
| Timing update rate | 26-entry round-robin × 120 ms = 3.1 s cycle; **measured 0.20–0.25 Hz** value changes (one new value per 3–5 s) | A pull lasts 2–7 s → 2–3 samples per pull |
| Samples inside real pulls (sess 596: 6 pulls of 2.0–6.7 s, rpm to 7,413) | e.g. `t=387.8 s, 6.7 s, 3338→4988 rpm: timing (0.0 s, 30°, load 17 %) (1.9 s, 58°, load 17 %) (5.4 s, 34°, load 100 %)` | Row values are **time-misaligned**: rpm is oversampled, but the load/timing in the same row can be 3 s stale — "load 17 %" at 4,463 rpm and ≥70 % throttle is a stale value, not a reading |
| `pen_timing` firing | **24.5 %** of rows in 596 (12.7 min, six short pulls); **9.2 %** in 594 (max throttle 35 %); **3.0 %** in 592 (max throttle 63 %) | The penalty compares consecutive timing samples 3–5 s apart at different rpm/load. It fires during gentle driving; it is not measuring retard |
| Absolute 0x0E values | 41° BTDC warm idle; 73–88° at light cruise; 34–62° at WOT (36° at 3,500 rpm → 55–62° at 5,500–7,500) | Wild for a conventional engine, plausible for VVEL at low lift, **unverified** — check once against a Cipher/EcuTek/Consult "IGN TIMING" reading. Detection needs only deltas, so the scale doesn't block the design |
| Heat soak | IAT 97–145 °F (median 111 °F in sess 595), ambient PID 70–118 °F, ECT ≤ 201 °F | ≈−3 % to −7 % from density alone across that range |
| Open loop share | `fuel_sys_status = 4` in 13 % of rows | STFT/LTFT are blind at WOT — Fuel Trust cannot see fuel starvation while it happens |
| Throttle PID at WOT | max 89 % at 7,563 rpm | 0x11 is the plate, not the pedal; pedal is not logged at all today |

---

## 3. The Heat Derate Monitor — what exactly, and how it works

### 3.1 Fast timing sampling in power windows (fixes finding #1)

- Add a second PID list, `OBD_PID_LIST_POWER[] = {0x0E, 0x0C, 0x0E, 0x11, 0x0E, 0x04, 0x0E, 0x0C}` at `OBD_PID_REQUEST_PERIOD_POWER_MS = 60`. Timing ≈ 8 Hz, rpm ≈ 4 Hz, throttle/load ≈ 2 Hz.
- Enter power mode when `throttle ≥ 70 % && rpm ≥ 3000` (or pedal ≥ 80 % once pedal exists); leave 1.5 s after the condition clears. Outside it, the existing 26-PID list runs unchanged (temperatures don't move fast).
- While in power mode, exempt the slow PIDs from the 5 s staleness timeout (or raise it to 10 s) so temps don't flicker "---" during a long straight.
- Each fast sample is stored with its own `millis()` and paired with the nearest rpm/load samples (the per-PID timestamps already exist in `g_obd_data`), so bins are built from aligned values, not from CSV rows.
- Where: `processOBD()` scheduler (`g_obd_pid_index`, `OBD_PID_REQUEST_PERIOD_MS`), `decodeOBD_Mode01Reply` case 0x0E. Bus load stays ≤ 17 req/s — well inside what ELM-class loggers do.

### 3.2 A learned cool baseline (the thing `pen_timing` never had)

- `g_tim_base[12]`: rpm bins of 500 from 2,000 to 8,000, WOT only (`throttle ≥ 70 % && load ≥ 85 %`). Each bin keeps an EMA (α≈0.1) and a count.
- A sample updates the baseline only while the engine is **cool**: oil < 230 °F, IAT < 120 °F, ECT < 205 °F. First cool laps of the day = the reference. Persisted in `Preferences` namespace `timbase` (written once at session end) so it survives reboots and sharpens over track days. Reset by long-press in the POWER popup.
- `Δtiming = live − base[bin]`, valid once the bin has ≥ 5 samples. Time-qualified alarm: Δ ≤ −3° held ≥ 1 s → amber; ≤ −6° → red.
- Attribution string from which temperature is elevated at that moment: `IAT` (IAT > 120 °F), `OIL` (oil > 250 °F), `ECT` (> 215 °F), else `KNOCK/FUEL?` (retard with no temperature excuse).

### 3.3 Four more derate detectors

| Detector | Uses | Logic | Catches |
|---|---|---|---|
| **Rev ceiling** | rpm @ 4 Hz, throttle | In a power window, track the rpm plateau: rpm within ±75 for ≥ 0.5 s while throttle ≥ 85 %. Plateau < 7,000 → `REV CAP ≈6,0xx`; < 5,000 → stage 2. A 7AT upshift is a sharp drop, not a plateau, so it doesn't false-trigger | Oil-temp protection stage 1/2 |
| **Pedal-vs-throttle gap** | pedal (PID 0x49/0x4A if supported, else CAN 0x180 byte F), throttle 0x11 | Normalize throttle to its own WOT max (~89 %); learn the cool gap per rpm bin; gap grows > 10 pts for ≥ 0.5 s outside a shift (rpm drop > 500 in 0.3 s) → `THROTTLE` | Any drive-by-wire torque management: protection, TCS, TCM torque-down |
| **Load deficit** | load 0x04 @ WOT | Cool baseline per rpm bin; live ≤ baseline − 8 % for ≥ 0.5 s → `LIFT` | VVEL lift restriction (airflow falls before the rev cap shows) |
| **Air density** | IAT 0x0F, baro 0x33 (BME280 humidity later) | `cf = 1.18·(990/Pd)·√((T_C+273)/298) − 0.18` → show `AIR −4 %` | Separates "hot air" from "ECU pulling" |
| **Fuel-cut flag** (bonus) | rpm, throttle, lateral g | rpm drop > 300 in 0.2 s at throttle ≥ 85 % and \|lat g\| > 0.8 → `FUEL?` latched 30 s with the g-sign and fuel % | Saddle-tank starvation in right-handers (see Part 4) |

The five feed one **POWER state**: `OK` / `AIR` / `TIMING` / `THROTTLE` / `LIFT` / `REV CAP` / `FUEL?` — worst active wins, with a reason string (`TIMING −4° · IAT 138 °F`).

### 3.4 What the driver sees (UI)

- **Event banner** (same top-layer pattern as the v6.13 DEMO banner): hidden when `OK`; amber/red strip across the top with the state + reason, e.g. `PWR ▼  REV CAP 6,050 · OIL 281 °F`. Colour is the message; the number is small. Persists until acknowledged by a tap; re-arms at a worse value instead of nagging (MoTeC-style auto-increment).
- **POWER popup**, opened by tapping the banner or extending the existing FUEL TRUST value tap (`showFuelTrustPopup` → two columns "FUEL / POWER"): Δtiming and its bin (e.g. `5,500–5,999: 55.0° now vs 58.5° cool`), samples in bin, pedal/throttle gap, load deficit, rev ceiling seen this session, air factor, oil/IAT/ECT at last alarm, and the attribution. Long-press = reset baseline.
- **Sensor-fault colour** (magenta, distinct from red) when pedal/timing sources are invalid, so "no data" never looks like "no problem".
- Optional: swap the FUEL TRUST chart to plot Δtiming during power windows. Not required.

### 3.5 Logging & post-session

- CSV: **append** columns (66 → ~78; existing positions unchanged, same convention as v6.9/v6.10): `tim_base_deg, tim_delta_deg, tim_bin_n, pedal_pct, pedal_valid, thr_gap_pct, load_delta_pct, rev_cap_rpm, air_cf, pwr_state, pwr_reason, atf1_f, atf2_f, tcc_slip_rpm`.
- Serial: `[PWR]` once per second; `[SESSION]` gains `min_dTiming_WOT`, `rev_cap`, `max_IAT`, `min_air_cf`.
- Session/lap report (the cloud dashboard consumes it): per lap (per session until GPS exists) — `min Δtiming @WOT, mean Δtiming, IAT max, ECT max, oil max, ATF max, rev cap, air cf, POWER states seen, min oilP > 2k, max lat/long g` + one verdict string (`HEAT: Δtiming −4.1° (IAT 128 °F) · AIR −4 %`).

### 3.6 Fuel Trust fix (same class of bug)

`computeFuelTrust()`'s timing term compares each 0x0E sample with the previous one, 3–5 s and thousands of rpm apart. Replace it with the baseline Δ from 3.2, counted only inside power windows (keep the 1.5 pts/event, max 30 weighting). This is not a softening — the current term is noise — but it will change the score, so it's flagged as a decision.

### 3.7 One-time discovery modes (on-car, no guessing)

| Mode | Request | Gives | Notes |
|---|---|---|---|
| **Supported-PID bitmaps** at boot | `01 00 / 20 / 40 / 60 / 80` → `[OBD-DISC]` line | Which of 0x45, 0x47–0x4C (pedal D/E, commanded throttle), 0x5A, 0x61–0x63 (demand/actual torque), 0x66 (MAF B1/B2) the 2018 ECU supports | Ends the guesswork that cost 0x10/0x5C/0x2F |
| **TCM ATF + TCC slip** | `7E1 21 20` (ISO-TP multi-frame; reuse the Mode 03 reassembler; flow control to 0x7E1) | ATF temp 1, ATF temp 2 (adjacent bytes, °C ≈ raw − 55 per ScanGauge's math), TCC slip 16-bit | Validate against the PRTXI trans sensor you already log — a built-in cross-check |
| **Mode 22 DID sweep** (menu toggle, idle only) | `7E0 22 1100…12FF` at 60 ms (~30 s); optionally after `10 C0` + `3E 01` keep-alive every 2.5 s | A 370Z owner found 224 responding DIDs this way. Candidates from the Nissan table (formulas unverified): 0x1107–0x110A / 0x112D / 0x113C ignition timing, 0x120D/0x120E accel sensors, 0x120F/0x1210 throttle sensors, 0x1204/0x1205 MAF B1/B2 (replaces unsupported PID 0x10), 0x1123–0x1126 A/F alpha | Decode by correlating with the standard PIDs |
| **Passive CAN sniff** | Filter is already ACCEPT_ALL; the RX drain drops non-0x7E8–0x7EF frames — add a dispatch | 0x180: rpm (A,B) + **pedal % = F/255** at 100 Hz; 0x551: ECT ≈ A − 40; 0x354: TCS off / brake; 0x002: steering angle; 0x284/0x285: wheel speeds (scale unverified); 0x1F9 rpm | Zero added bus load; raise `rx_queue_len` 32→64 and drain every loop |

### 3.8 On-car validation plan

1. Flash v6.14 → confirm the Mode-22 oil-temp offset against the factory gauge (you finally have oil temp on track).
2. Boot → read the bitmap line; add whichever of 0x49/0x4A/0x4C/0x61–0x63 answer.
3. At idle → TCM `21 20` dump; match ATF1/ATF2 to the PRTXI trans reading.
4. First drive → confirm 0x180 pedal/rpm scale against 0x0C and 0x11.
5. Track day: morning session builds the baseline; afternoon shows Δ. Compare the verdict strings with how the car felt.

### 3.9 Risks

- CPU/RAM: negligible (a second PID list, ~1 KB of bins, one popup). Passive sniffing raises RX interrupt load — keep the drain tight.
- Bus etiquette: ≤ 17 req/s, watch TWAI alerts, stop on bus-off rather than auto-recover in a loop (FSM: CAN-loss DTC → 3,500 rpm limp).
- NVS wear: write the baseline at most once per session.
- The 0x0E absolute scale: unverified but irrelevant to Δ; verify once for the popup's absolute number.

---

## 4. More transparency on track — prioritized

Box constraints: 3 spare Waveshare channels AI6–AI8 (0–10 V / 0–20 mA / 4–20 mA), spare I2C ports on the hub, one free UART, TWAI on the OBD CAN, 10 Hz SD logging, cloud upload.

| Pri | Add | What it reveals | Hardware | Plugs in | Cost / effort |
|---|---|---|---|---|---|
| 1 | **Heat Derate Monitor** (Part 3) | ECU/thermal power loss, attributed | none | `processOBD`, new popup, banner, CSV cols | $0 / medium |
| 1 | **Fuel pressure** | Right-hander starvation (saddle tank, pump on the passenger side; can occur up to ¾ tank in long sweepers — Z1), pump fade with heat, and it *is* the physical fuel-pressure test for the −12 % rich trims. STFT can't see it (open loop at WOT) | WIKA A-10 0–100 psi **4–20 mA** (industrial, ~$150) → AI6 Mode 3, same PRTXI wiring pattern; or AEM 30-2130-100 0.5–4.5 V → 0–10 V mode. Tap at the OEM rail damper (aftermarket kits mount there; nominal 51 psi, returnless) | new reading in the FUEL popup + `FUEL?` latch upgraded to a real alarm (`< 40 psi ≥ 0.3 s at rpm > 3k`) + CSV | ~$150–200 / low |
| 1 | **TCM ATF temps + TCC slip** | The transmission's *own* protection input; converter slip = heat generation and lock-up behaviour on track | none (Mode 21) | TRAN TEMP popup "TCM says…", CSV | $0 / low |
| 1 | **Pedal position** | Required for the gap detector; 100 Hz | none (CAN 0x180 or PID 0x49) | `g_obd_data.pedal` | $0 / low |
| 2 | **GPS 10–25 Hz** | Lap/sector times, track map, and the actual "how much slower" number: per-lap acceleration in a reference straight → `P_wheel ≈ m·a·v + ½ρ·CdA·v³` (Race Technology / AiM math), density-normalized. Turns "feels slow" into "lap 8: −0.6 s on the straight, Δtiming −3°, IAT 131 °F" | u-blox NEO-M9N (25 Hz, 0.05 m/s velocity, ~$30–70) on the free UART (UBX binary), or RaceBox Mini | new `[LAP]` line, per-lap report CSV, cloud dashboard track map | ~$50 / medium |
| 2 | **Passive CAN: wheel speeds, steering, brake, TCS** | Slip/lock-up flags, understeer channel (steering vs lat-g vs speed), better speed than PID 0x0D | none | RX dispatch, CSV | $0 / low–medium (verify scaling) |
| 2 | **Wideband O2** | Real WOT AFR on a catless untuned car; lean spikes = starvation; enrichment vs heat | AEM 30-0300 X-series 0–5 V (~$230) → AI7 0–10 V mode; budget: 14Point7 Spartan 3 Lite (~$85–135). Bung in the test pipe | FUEL popup, CSV, lean-spike latch | ~$100–250 / low–medium |
| 3 | **BME280** | Humidity → proper density altitude; "thin air today" vs "engine bay cooking" | $15, I2C hub, mount at the cowl | `air_cf` uses dry-air pressure | $15 / low |
| 3 | **Brake pressure** | Braking technique (threshold, trail), brake vs scrub in the g trace | any 0–2,000 psi 0–5 V transducer + tee at the master cylinder (~$150) → AI8 | CSV, lap report | ~$150 / medium |
| 3 | **Coolant pressure** | Cap/leak/head-gasket early warning | 150 psi 0.5–4.5 V; needs a channel | CSV alarm | ~$50 / medium |
| — | Tire IR arrays / rotor IR / EGT | Camber balance / pad window / per-bank mixture | per-corner MCUs; rotor IR needs ≥ 1,000 °C sensors (MLX90614 tops at 382 °C); EGT low value on a stock NA ECU | — | skip for now |
| — | Knock counts directly | The direct heat-power signal | Only via an EcuTek/UpRev tune (RaceROM also adds oil-temp/fuel-pressure/knock failsafes and a knock-warning CEL flash) | not a box channel | tuning decision |

Presentation rules worth adopting from AiM/MoTeC/RaceCapture practice: every alarm gets a time qualifier and an rpm/speed gate (you already did this for oil pressure); tiered colours (amber → red); magenta for sensor faults; single-tap acknowledge with auto-increment re-arm; colour readable at a glance, number secondary.

---

## Sources

**Nissan / FSM**
- 2009 370Z owner's manual (engine protection mode, 280 °F): https://owners.nissanusa.com/content/techpub/ManualsAndGuides/ZCoupe/2009/2009-ZCoupe-owner-manual.pdf
- 2018 370Z Quick Reference Guide: https://owners.nissanusa.com/content/techpub/ManualsAndGuides/ZCoupe/2018/2018-ZCoupe-quick-reference-guide.pdf
- 2019 370Z Quick Reference Guide: https://owners.nissanusa.com/content/techpub/ManualsAndGuides/ZCoupe/2019/2019-ZCoupe-quick-reference-guide.pdf
- FSM "torque cut control at high engine oil temperature" passage (quoted): https://bobistheoilguy.com/forums/threads/new-370z-overheating-with-mild-track-time.101331/ and https://my350z.com/forum/2009-370z/414330-370z-has-overheating-issues-on-road-courses.html
- VQ37VHR FSM (2008 G37, same ECM family): fail-safe chart https://zinref.ru/avtomobili/Infiniti/040_010_00_Infiniti_G37_Cupe_service_manual_2008_ENGLISH/592.htm · ignition corrections …/466.htm · IAT sensor in MAF …/500.htm · fuel enrichment/fuel cut …/463.htm · P1217 …/550.htm
- 2012 370Z FSM LAN (CAN signal chart) https://www.nicoclub.com/service-manual?fsm_download=370Z%2FCoupe%2F2012%2FLAN.pdf · TM (Fail-Safe TM-285, Protection Control TM-287) https://www.nicoclub.com/service-manual?fsm_download=370Z%2FCoupe%2F2012%2FTM.pdf

**Tuner documentation**
- UpRev Nissan Tuning Guide (temperature corrections, Knock Strength, High Det, torque tables): https://uprev.com/wp-content/uploads/2023/05/UpRev-Nissan-Tuning-Guide-Rev2.pdf
- EcuTek 370Z ProECU Tuning Guide: https://ecutek.atlassian.net/wiki/spaces/SUPPORT/pages/7143430/370Z+ProECU+Tuning+Guide
- EcuTek 370Z knock control / octane judgement: https://ecutek.atlassian.net/wiki/spaces/SUPPORT/pages/1653997569/370Z+Knock+Control,+dynamic+advance+and+Octane+Judgement
- EcuTek 370Z RaceROM supplement (failsafes): https://ecutek.atlassian.net/wiki/spaces/SUPPORT/pages/6881638/370Z+ProECU+RaceROM+Supplement
- EcuTek 370Z FAQ (oil-temp failsafe/CEL): https://ecutek.atlassian.net/wiki/spaces/SUPPORT/pages/7635028/Nissan+370Z+FAQ
- HP Tuners forum (knock as ROM memory parameters): https://forum.hptuners.com/showthread.php?91041-370Z-turbo-tuning&p=647261

**Owner reports (oil-temp protection behaviour)**
- MotoIQ: https://motoiq.com/nissan-370z-bad-news-buzz-high-engine-diff-oil-temps-the-300-oil-change-engine-clatter-and-more-hype-or-reality/
- Edmunds 2009 thread: https://forums.edmunds.com/discussion/11381/nissan/370z/nissan-370z
- my350z "dealer says no problem…": https://my350z.com/forum/2009-370z/492809-dealer-says-no-problem-with-high-running-temperatures-370z.html
- my350z ATF temperature/limp: https://my350z.com/forum/engine-and-drivetrain/472200-automatic-transmission-fluid-temperature.html
- my350z heat soak: https://my350z.com/forum/autocross-road/565115-350z-heat-soak-causes.html · IAT numbers: https://my350z.com/forum/engine-and-drivetrain/440115-air-intake-temperature.html

**OBD / CAN**
- OBD-II PIDs (formulas, support bitmaps): https://en.wikipedia.org/wiki/OBD-II_PIDs
- 2010 370Z CAN reverse engineering (0x180 rpm+pedal, 0x551, 0x354, 0x002, 0x284/0x285): https://github.com/Knio/carhack/blob/master/Cars/Nissan.markdown
- AiM Nissan 370Z OEM protocol (channel list): https://www.aim-sportline.com/download/ecu/stock/nissan/Nissan370Z_104_eng.pdf
- Racelogic VBOX 370Z CAN database: https://www.racelogic.co.uk/_downloads/vbox/Vehicles/Other/Docs/Nissan-370Z.pdf
- ProjectBytes 370Z custom PIDs (224 DIDs, session C0): https://projectbytes.wordpress.com/2014/06/29/nissan-370z-custom-pids/
- Nissan Mode 22 DID table (Torque forum, formulas unverified): https://torque-bhp.com/community/main-forum/nissan-titan-support-thread/
- ScanGauge 370Z X-Gauges — ATF 1: https://www.scangauge.com/xgauge/transmission-fluid-temperature-1-f-370z/ · ATF 2: https://www.scangauge.com/xgauge/transmission-fluid-temperature-2-f-nissan-370z/ · TCC slip: https://www.scangauge.com/xgauge/torque-converter-clutch-slip-speed-rpm-nissan-370z/
- niscan (370Z Arduino CAN): https://github.com/rampage128/niscan · G37 DBC: https://github.com/icecube45/Dash_InfinitiG37/blob/master/InfinitiG37.dbc

**Methods & sensors**
- SAE J1349 correction: https://www.ajdesigner.com/dyno-correction-factor/
- Race Technology power from data: https://www.race-technology.com/wiki/index.php/AnalysisTechnical/MeasuringPowerAndTorque · AiM Math Channels 101: https://www.aim-sportline.com/download/software/doc/Math_Channels_101_eng.pdf
- Z1 anti-starvation kit text (reseller copy): https://www.tarmacsportz.co.uk/z1-motorsports-nissan-350z-03-09-370z-09-20-fuel-anti-starvation-kit.html · Radium 370Z fuel hanger: https://www.radiumauto.com/products/fuel-hanger-surge-tank-nissan-370z
- Starvation threads: https://my350z.com/forum/autocross-road/544501-power-fuel-cut-off-on-track.html · https://my350z.com/forum/autocross-road/590134-fuel-starvation-issues-2.html · https://my350z.com/forum/autocross-road/556381-does-anybody-experience-fuel-starvation-in-right-handers.html
- Fuel pressure test point (51 psi, returnless): https://my350z.com/forum/maintenance-and-repair/585387-fuel-pressure-test-point.html · BTW Tuning damper-mount kit: https://btwtuning.com/products/370z-g37-q50-3-7-fuel-pressure-monitoring-kit
- WIKA A-10: https://www.wika.com/en-us/a_10.WIKA · AEM 30-2130-100: https://www.aemelectronics.com/products/sensors/pressure_sensors/parts/30-2130-100
- AEM 30-0300 wideband sheet: https://documents.aemelectronics.com/30-0300_for_x-series_fae_uego_gauge_kit.pdf · 14Point7 Spartan 3 Lite: https://www.14point7.com/products/spartan-3-lite-v2
- u-blox NEO-M9N datasheet: https://content.u-blox.com/sites/default/files/NEO-M9N-00B_DataSheet_UBX-19014285.pdf · RaceChrono GPS FAQ: https://racechrono.com/article/faq/which-external-gps-should-i-buy
- BME280: https://www.adafruit.com/product/2652 · Brake pressure sensors: https://grassrootsmotorsports.com/forum/grm/brake-pressure-sensor-for-data-acquisition/70215/page1/ · Coolant pressure: https://www.autosportlabs.com/how-to-add-a-coolant-pressure-sensor-and-avoid-a-rookie-mistake/
- Rotor temps vs IR sensor limits: https://www.hpacademy.com/forum/general-tuning-discussion/show/desired-rotor-temp/ · DIY tire IR: https://github.com/MagnusThome/RejsaRubberTrac

**Alarm/presentation practice**
- MoTeC C125 manual (time-qualified alarms, auto-increment): https://www.motec.com.au/hessian/uploads/C125_User_Manual_1f4a1c6bf4.pdf
- Rush SR AiM dash alarms (tiered oil-pressure/rpm ratio, magenta = sensor fault): https://manual.rush.sr/at-the-track/aim-dashes-and-smartycams/dash-alarms
- Rennlist AiM MXS alarm discussion: https://rennlist.com/forums/data-acquisition-and-analysis-for-racing-and-de/909669-this-mxs-alarm-pop-up-feature-want.html

*Unreachable during research (claims from them not used): the370z.com, myg37.com, z1motorsports.com product text, the 2009+ 370Z EC/TM PDF bodies.*
