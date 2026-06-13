# OBD-II CAN Wiring & Bring-up — Waveshare ESP32-S3-Touch-LCD-7 ↔ 2018 Nissan 370Z

**Status:** New harness, fresh start. Goal: get the onboard TWAI/CAN reading the car's ECU (water temp, RPM, fuel trims) over the OBD-II port.

**Symptom on the old attempt:** No CAN data at all. Two firmware bugs explain this completely (see below). Fix those first — the wiring is the easy part.

---

## TL;DR — why you got zero CAN data

The Waveshare ESP32-S3-Touch-LCD-7 does **not** give the ESP32 a dedicated CAN pin pair. GPIO19/GPIO20 are **shared** between the chip's native USB (D-/D+) and the CAN transceiver, and a hardware USB mux (**FSUSB42UMX**) picks which one is connected. That mux is controlled by **CH422G EXIO5 (CAN_SEL / USB_SEL)**.

Confirmed from three sources that all agree:
- Waveshare wiki pinout: `GPIO20 = CANTX`, `GPIO19 = CANRX`, and `EXIO5 CAN_SEL — pull **up** to CAN mode, otherwise USB mode`.
- The board schematic (`ESP32-S3-Touch-LCD-7-Sch.pdf`, page 1): U7 = `TJA1051T` transceiver; `EXIO5 / USB_SEL` drives a `FSUSB42UMX` 2:1 USB mux on the D+/D- lines.
- Your firmware: `g_exio_state` is initialized to only `EXIO_TP_RST | EXIO_SD_CS` — **EXIO5 is never set**, so the mux stays in USB mode and the TJA1051T is disconnected from the MCU.

So even with perfect wiring, the transceiver was never electrically connected to the ESP32. **This is the primary bug.**

**Bug #2:** TX and RX are swapped in firmware. Waveshare routes `GPIO20→CANTX` and `GPIO19→CANRX`, but the sketch defines `CAN_TX_PIN = GPIO19` and `CAN_RX_PIN = GPIO20`. Backwards.

Fix both and the wiring below will work.

---

## Firmware fixes (required — do these before wiring matters)

> The repo memory note says don't touch firmware unless asked. These two changes are the reason CAN never worked, so they're in scope for "make CAN work." Apply them (or ask me to).

### Fix 1 — drive EXIO5 to CAN mode at boot

In the IO-expander defines block (near line ~1654), add a bit for EXIO5:

```cpp
#define EXIO_TP_RST   1
#define EXIO_DISP     2
#define EXIO_SD_CS    4
#define EXIO_CAN_SEL  5   // FSUSB42UMX mux: HIGH = CAN mode, LOW = USB mode
```

In `exio_init()` (near line ~6457), include EXIO5 in the initial state so the mux selects CAN:

```cpp
// was: g_exio_state = (1u << EXIO_TP_RST) | (1u << EXIO_SD_CS);
g_exio_state = (1u << EXIO_TP_RST) | (1u << EXIO_SD_CS) | (1u << EXIO_CAN_SEL);
```

(or call `exio_set(EXIO_CAN_SEL, true);` right after `exio_init()` succeeds, before `startCAN()`.)

### Fix 2 — un-swap the CAN pins

At ~line 2725:

```cpp
// was:
// #define CAN_TX_PIN  GPIO_NUM_19
// #define CAN_RX_PIN  GPIO_NUM_20
#define CAN_TX_PIN  GPIO_NUM_20   // ESP32 CANTX -> TJA1051T TXD  (per Waveshare pinout)
#define CAN_RX_PIN  GPIO_NUM_19   // ESP32 CANRX <- TJA1051T RXD
```

Also fix the stale header comment at ~line 2713 (it currently claims GPIO19=CANTX, which is the source of the confusion).

### Side effect to know about

Putting EXIO5 in CAN mode **disconnects the ESP32-S3 native USB** (the FSUSB42 can only route D+/D- to one place). On this board that's fine for normal operation because:
- Flashing and the serial monitor run through the **separate "UART" USB-C port** (the CH343/USB-UART bridge), not the native USB. Keep using that port at 115200.
- **USB Mass Storage mode (`ENABLE_USB_MSC`) uses the native USB and therefore cannot coexist with CAN mode.** When you boot into MSC to pull SD logs, CAN will be off — expected. If you want MSC, set EXIO5 low (or just don't start CAN) in that mode.

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

1. **Bench/firmware check first.** Flash the two fixes. On boot, the serial log should print `[OBD] Config: TX=GPIO20, RX=GPIO19, 500kbps` and the TWAI driver should install/start without `0x...` errors.
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

## Parts list

| Item | Why | Notes |
|---|---|---|
| XMSJSIY OBD-II 16-pin male pigtail (you have it) | OBD plug → bare wires | [Amazon B0CSK7FRG6](https://www.amazon.com/dp/B0CSK7FRG6). Verify colors with a meter. |
| JST PH 2.0 mm 2-pin pigtail/housing | Land Green/Brown-White into the board CAN terminal | Ships with the board; spares are cheap (JST PHR-2 + crimps, or pre-made PH2.0 leads). |
| 3-conductor shielded cable (engine-bay run) | CANH + CANL + drain, OBD plug → box | Twisted pair preferred for CANH/CANL. Tie shield to GND at the box end only. |
| Heat-shrink + crimp ferrules | Cap unused pigtail wires (esp. pin 16 +12 V) | — |
| (Optional) OBD-II Y-splitter | Keep a port free for a scan tool | Only if you still want to plug in a code reader without unplugging the monitor. |

No new active parts needed — the transceiver, mux, and termination are all already on the Waveshare board. The fix is firmware (EXIO5 + TX/RX) plus a clean 3-wire-plus-ground tap.

---

## Sources

- [Waveshare ESP32-S3-Touch-LCD-7 Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-7) — CAN pinout (GPIO20=CANTX, GPIO19=CANRX), EXIO5 CAN_SEL/USB_SEL mux, PH2.0 CAN terminal, 120 Ω termination jumper.
- `ESP32-S3-Touch-LCD-7-Sch.pdf` (in repo) page 1 — U7 TJA1051T transceiver, EXIO5/USB_SEL → FSUSB42UMX USB mux on D+/D-.
- [Racelogic Nissan 370Z CAN database](https://www.racelogic.co.uk/_downloads/vbox/Vehicles/Other/Docs/Nissan-370Z.pdf) and [AiM Nissan 370Z notes](https://www.aim-sportline.com/download/ecu/stock/nissan/Nissan370Z_104_eng.pdf) — 370Z OBD-II CAN on pins 6/14, 500 kbps.
- [XMSJSIY pigtail listing](https://www.amazon.com/dp/B0CSK7FRG6) — pin↔color table (verify with meter).
