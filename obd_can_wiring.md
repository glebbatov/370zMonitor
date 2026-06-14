# OBD-II CAN Wiring & Bring-up — Waveshare ESP32-S3-Touch-LCD-7 ↔ 2018 Nissan 370Z

**Status: ✅ RESOLVED (2026-06-14, v6.1).** OBD CAN is reading the 2018 370Z ECU on the car — water temp / ECT confirmed in the serial log (`[OBD] ECT:25°C ...` + `OBD CAN back online`). The working harness is a **2-wire CANH/CANL tap** (OBD pin 6 → CANH, pin 14 → CANL); no separate OBD ground was needed because the electric box already shares chassis ground with the car. Fix was firmware (`CAN_MUX_TO_CAN 1` + the v6.0 pin/EXIO5 work), not wiring.

**What was wrong (history):** No CAN data at all. Two firmware bugs (TX/RX swapped + the EXIO5 USB/CAN mux never selected), both fixed in v6.0, plus the `CAN_MUX_TO_CAN` debug gate left at `0` during display bring-up — flipped to `1` on 2026-06-14 (v6.1).

> **Note on the 3rd wire:** earlier drafts of this doc recommended running OBD pins 4/5 (ground) as well. In practice the 2-conductor CANH/CANL tap works because the box's 12 V supply ground is bonded to chassis, the same reference as the ECU. Add the OBD ground wire only if you ever see flaky/no comms.

**Visual:** see `obd_can_harness_diagram.svg` in this repo for the pigtail-color → board-terminal map.

---

## TL;DR — why you got zero CAN data

The Waveshare ESP32-S3-Touch-LCD-7 does **not** give the ESP32 a dedicated CAN pin pair. GPIO19/GPIO20 are **shared** between the chip's native USB (D-/D+) and the CAN transceiver, and a hardware USB mux (**FSUSB42UMX**) picks which one is connected. That mux is controlled by **CH422G EXIO5 (CAN_SEL / USB_SEL)**.

Confirmed from three sources that all agree:
- Waveshare wiki pinout: `GPIO20 = CANTX`, `GPIO19 = CANRX`, and `EXIO5 CAN_SEL — pull **up** to CAN mode, otherwise USB mode`.
- The board schematic (`ESP32-S3-Touch-LCD-7-Sch.pdf`, page 1): U7 = `TJA1051T` transceiver; `EXIO5 / USB_SEL` drives a `FSUSB42UMX` 2:1 USB mux on the D+/D- lines.
- Your firmware (v6.0): the original no-data was caused by (1) **EXIO5 never set**, so the mux stayed in USB mode and the TJA1051T was disconnected from the MCU, and (2) **TX/RX swapped**. Both are **now fixed** — `CAN_TX_PIN = GPIO20`, `CAN_RX_PIN = GPIO19`, and EXIO5 is driven HIGH at boot. The last thing gating it, the debug flag `CAN_MUX_TO_CAN`, was left at `0` during the LVGL/display bring-up and is **now `1`** (2026-06-14).

So the firmware is ready. The remaining job is a clean 3-wire-plus-ground tap to the OBD port — see "Firmware status" and "The harness" below.

---

## Firmware status (done — nothing to change)

Every firmware item that caused the old no-data is already in the code as of v6.0 + the 2026-06-14 flag flip:

| Item | State | Where |
|---|---|---|
| `CAN_TX_PIN` / `CAN_RX_PIN` | `GPIO20` / `GPIO19` — correct, un-swapped | ~line 2746 |
| `EXIO_CAN_SEL` (EXIO5) | defined as bit 5, driven HIGH at boot | ~line 1668 + `initIOExtension()` |
| `CAN_MUX_TO_CAN` gate | **`1`** — set 2026-06-14 (was `0` during display bring-up) | `initIOExtension()`, ~line 6485 |

With `CAN_MUX_TO_CAN 1`, `initIOExtension()` sets `g_exio_state = EXIO_TP_RST | EXIO_SD_CS | EXIO_CAN_SEL`, flipping the FSUSB42UMX mux so GPIO19/20 reach the onboard TJA1051T. On boot the log should print `[OBD] Config: TX=GPIO20, RX=GPIO19, 500kbps` and the TWAI driver should install without `0x…` errors. **Flash this build before testing the harness.**

### Side effect to know about

Putting EXIO5 in CAN mode **disconnects the ESP32-S3 native USB** (the FSUSB42 routes D+/D- to USB *or* CAN, not both). On this board that's fine for normal operation:
- Flashing and the serial monitor run through the **separate "UART" USB-C port** (CH343/USB-UART bridge) at 115200. Keep using that port.
- **USB Mass Storage (`ENABLE_USB_MSC`) uses native USB and cannot coexist with CAN.** To pull SD logs over USB-MSC, set `CAN_MUX_TO_CAN 0` and reflash; CAN will be off in that mode (expected).

---

## The harness — pigtail colors → board

Pigtail: **XMSJSIY OBD-II 16-pin male, open-end bare wire, 1 m** (Amazon B0CSK7FRG6). Color map is the manufacturer's own table on the listing photo; it matches standard OBD-II pin functions. **Verify each wire with a multimeter continuity check to the connector pin before trusting the color** — cheap pigtails do occasionally ship mis-labeled.

| OBD-II pin | Function | Pigtail wire color | Goes to |
|---|---|---|---|
| **6** | CAN High (CAN-C, 500 kbps) | **Green** | Board CAN terminal **CANH** |
| **14** | CAN Low (CAN-C, 500 kbps) | **Brown/White** | Board CAN terminal **CANL** |
| **4** | Chassis ground | **Orange** | Common GND (board GND / supply −) |
| **5** | Signal ground | **Yellow** | Common GND (tie with pin 4) |
| 16 | Battery +12 V (constant) | **Green/White** | **Not used for CAN** — cap it (see note) |

Unused pigtail wires (1,2,3,7,8,9,10,11,12,13,15) — cut back, individually heat-shrink, bundle. Don't leave bare copper near the +12 V (pin 16) wire.

### Notes on the connections

- **CANH/CANL polarity matters.** Match the board's silkscreen labels (`CANH`, `CANL` on the PH2.0 CAN terminal, item 9). Green→CANH (pin 6), Brown/White→CANL (pin 14). If H and L are swapped the differential pair is inverted and the controller won't sync — no data, no errors that obviously point at it.
- **Ground is not optional.** The CAN transceiver needs a common reference with the car. Tie OBD pins 4 and 5 together to the board/supply ground. In practice the box already shares chassis ground through its 12 V feed, but run the OBD ground anyway — a floating or high-impedance CAN ground is a classic "intermittent / no comms" cause.
- **Pin 16 (+12 V):** Your install powers the electric box from its own 5 A-fused 12 V branch (per CLAUDE.md / install-architecture), so you do **not** need OBD pin 16. Cap and heat-shrink the Green/White wire. (Pin 16 is *constant* battery on most cars — if you ever did use it, it would not key off with ignition.)

---

## Board hardware setup

1. **CAN termination jumper (item 13) — REMOVE / set to NC.**
   The board ships with its 120 Ω CAN terminator jumpered ON. The car's CAN-C bus is **already terminated at both ends (2 × 120 Ω = 60 Ω)**. You're a *tap* on an existing, terminated bus — adding the board's 120 Ω drops the bus to 60 ‖ 120 = **40 Ω**, out of spec and extra load on the ECU's drivers. Pull the jumper to the NC position so the board does **not** add termination. (RS485's separate terminator jumper is unrelated — leave it however your RS485 sensor chain needs.)

2. **CAN terminal connector type:** the CAN header is a **2-pin JST PH 2.0 mm** (item 9). The board comes with PH2.0→2.54 mm pigtails in the box — use one of those to land the OBD Green / Brown-White wires, or crimp a 2-pin PH2.0 housing. Don't solder directly to the board pads.

3. **Serial monitor:** use the **UART** USB-C port (not the one nearest the native-USB silkscreen). 115200 8N1. This keeps working while CAN mode owns the native-USB pins.

---

## Bring-up / validation sequence

Do this with the **engine running** (or at least ignition in RUN). On the 370Z the powertrain CAN on pins 6/14 is only active and the ECU only answers when the car is awake — a no-data result at key-off is normal and not a wiring fault.

1. **Bench/firmware check first.** Flash the current build (with `CAN_MUX_TO_CAN 1`). On boot, the serial log should print `[OBD] Config: TX=GPIO20, RX=GPIO19, 500kbps` and the TWAI driver should install/start without `0x...` errors.
2. **Continuity-verify the pigtail** (key out): meter from each OBD socket pin to the bare wire end. Confirm pin 6 = Green, pin 14 = Brown/White, pin 4/5 = Orange/Yellow. Re-label if the factory colors are wrong.
3. **Resistance sanity check at the OBD plug** (key out, car asleep): CANH↔CANL should read **~60 Ω** (the two factory terminators in parallel). If you read ~120 Ω something's off; if you read 40 Ω your board terminator is still jumpered in — go fix step 1 of hardware setup.
4. **Plug in, key to RUN / start engine.** Watch the once-per-second OBD/`[OBD]` log. You should see successful PID replies; the toast monitor should clear "OBD CAN offline."
5. **If still nothing:** the fastest disambiguation is to **swap CANH/CANL** (Green↔Brown-White) at the board terminal and retry. Inverted polarity is the most common remaining cause once the mux and TX/RX are correct.

### Quick fault tree if it's still dead after the fixes

- TWAI installs but `RX = 0`, bus-error counter climbing → polarity (swap CANH/CANL) or termination (40 Ω = board terminator still in).
- TWAI installs, totally silent, no errors → car asleep (key off) or ground not connected.
- TWAI driver `install failed` → EXIO5 not actually high, or pin numbers wrong. Confirm `exio_set(EXIO_CAN_SEL,true)` ran *before* `startCAN()`.
- Works on bench with a USB-CAN tool but not on the car → confirm the 370Z exposes powertrain CAN on the diagnostic port (it does on 2009-2020 Z34; some OBD readers need engine running to wake the bus).

---

## Parts list (researched 2026-06-14)

You already have the only must-buy item (the pigtail). Everything else is connectors/cable/consumables — **nothing active is needed**, since the transceiver, mux, and termination are all on the Waveshare board.

| Item | Spec / part | Why | ~Price |
|---|---|---|---|
| **OBD-II pigtail** (have it) | XMSJSIY 16-pin male, open-end, 1 m — [Amazon B0CSK7FRG6](https://www.amazon.com/dp/B0CSK7FRG6) | OBD plug → bare wires | $9 |
| **Board CAN lead** | JST **PH2.0 2-pin** — ships in the board box (PH2.0→2.54 mm leads). Spare: JST **PHR-2** housing + **SPH-002T-P0.5S** crimps, or a pre-made PH2.0 2-pin lead | Land Green / Brown-White into CAN terminal (item 9) without soldering the pads | $0–6 |
| **CAN cable, OBD→box** | 120 Ω twisted shielded pair — **Belden 3105A** (J1939/11, 22 AWG) or **L-com** per-foot CAN cable (24 AWG, 120 Ω, double-shielded) | Twisted CANH/CANL run; shield to GND at the box end only. Run is short (~1–3 ft) so even plain shielded twisted pair works — the twist matters more than the exact P/N | $1–2/ft |
| **Capping consumables** | Adhesive-lined heat-shrink + crimp ferrules | Cap the 11 unused pigtail wires (esp. pin 16 +12 V) | $8 kit |
| **(Optional) USB-CAN analyzer** | Waveshare **USB-CAN-A** (their own recommended bench tool) | Bench-test the board's CAN with demo `06_TWAItransmit` / `07_TWAIreceive` before going to the car — isolates the board from car-side issues | $12 |
| **(Optional) OBD-II Y-splitter** | 16-pin male → dual female (VIMVIP / iKKEGOL) | Keep a port free for a scan tool. Note: only one device can actively talk at a time | $8 |

Bottom line: the only fix beyond a clean 3-wire-plus-ground tap was firmware (EXIO5 + TX/RX + the `CAN_MUX_TO_CAN` gate), and that's done.

---

## Sources

- [Waveshare ESP32-S3-Touch-LCD-7 Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-7) — CAN pinout (GPIO20=CANTX, GPIO19=CANRX), EXIO5 CAN_SEL/USB_SEL mux, PH2.0 CAN terminal, 120 Ω termination jumper.
- `ESP32-S3-Touch-LCD-7-Sch.pdf` (in repo) page 1 — U7 TJA1051T transceiver, EXIO5/USB_SEL → FSUSB42UMX USB mux on D+/D-.
- [Racelogic Nissan 370Z CAN database](https://www.racelogic.co.uk/_downloads/vbox/Vehicles/Other/Docs/Nissan-370Z.pdf) and [AiM Nissan 370Z notes](https://www.aim-sportline.com/download/ecu/stock/nissan/Nissan370Z_104_eng.pdf) — 370Z OBD-II CAN on pins 6/14, 500 kbps.
- [XMSJSIY pigtail listing](https://www.amazon.com/dp/B0CSK7FRG6) — pin↔color table (verify with meter).
