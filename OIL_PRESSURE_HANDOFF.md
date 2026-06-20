# Oil Pressure Debug — Handoff (2026-06-17)

Carry this into the next chat. Full detail lives in `CLAUDE.md` → "Pressure Sensor Install Status" and Gotchas #19/#20.

## TL;DR verdict
- **The P51 oil-pressure sensor and its harness are GOOD.** Stop replacing/RMA-ing sensors.
- **The Waveshare 8-channel analog module is the bad actor.** It won't reliably report a 4-20mA current that is *measurably present* on its own sense resistor.
- The months-long "CH1 is dead / first P51 is toast" story was a **jumper mistake**, not dead hardware.

## How we proved the sensor is good
With the loop fully verified, a voltmeter reads **2.074 V at AI+ (to GND)** at atmosphere.
The Waveshare 4-20mA input = a **500 Ω sense resistor** (its "2-10 V" range is 4-20 mA × 500 Ω), so:

> **mA = V(AI+ to GND) ÷ 500 Ω** → 2.074 V = **~4.15 mA = ~0 PSI** (textbook-correct zero).

The sensor also responded to applied pressure in an earlier bench test. Continuity is perfect: 12V+↔Pin1 = 0.1 Ω, Pin3↔AI+ = 0.3 Ω, AI−↔12V− = 0.2 Ω, Pin2 isolated.

## The jumper insight (root of the whole saga)
The Waveshare has a **per-channel hardware jumper**, separate from the software Mode-3 register:
- **ON = current mode** (4-20mA sense resistor in circuit) — what a 4-20mA sensor needs.
- **OFF = voltage mode** (high-Z 0-10V input).

**CH1's jumper was left OFF** (leftover from the old PX3 voltage sensor). A 4-20mA loop into a high-Z voltage input has no sense resistor to flow through, so the sensor rails the input → module reads **20000 µA / 150 PSI, dead-steady**. That "stuck at 150 / dead channel" symptom was the jumper — **CH1 may actually be fine** (never retested with jumper ON). A 4-20mA channel needs jumper **ON + Mode 3**.

## Why it's the module, not the sensor
Temps (PRTXI) read flawlessly on every channel; the P51 never sustains on any channel even though it delivers ~4 mA:
- CH1 (AI1): jumper OFF → railed 20000 (not a real test).
- CH6 (AI6): jumper ON → clean 4.00 mA then drops to 0, repeatedly (intermittent).
- CH2 (AI2): jumper ON → flat 0 µA.
- CH3 (AI3, the channel that read trans-temp perfectly): **2.074 V present at AI+ (~4 mA) but module reports 0 µA.**
- Plus chronic RS485 `Communication LOST` / `Got 0 bytes` in nearly every log.

## Current firmware = v6.5 (diagnostic A/B swap)
- Oil pressure → **CH3 (AI3)**: `MODBUS_CH_OIL_PRESSURE = 2`
- Trans temp → **CH2 (AI2)**: `MODBUS_CH_TRANS_TEMP = 1`
- CH1 excluded (dead/jumper), oil-temp slot parked on CH6 (no sensor)
- Version bumped in all three spots: `.ino` header, `ui_ScreenSplash.c`, serial boot banner.

## Pin mapping (734-1165-ND harness — by PIN, not color)
- **Pin 1 = +24V (Vin)**
- **Pin 2 = NA / unused** (not a ground — leave open)
- **Pin 3 = loop return → Waveshare AI+**
- **AI− → supply (−)**

## Next steps (in order)
1. **Free shot:** pull/clean/firmly reseat the **CH3 jumper (ON)** and re-torque the **AI3+ screw terminal**; watch whether `[OILP] CH3` starts reporting the ~4 mA that's already there.
2. **If it still won't read → replace the Waveshare 8-ch analog module (~$30).** Keep the same P51 + harness; they're good. (Three misbehaving channels + chronic RS485 drops = damaged module.)
3. After a steady 0 PSI / ~4 mA at rest that climbs under a pump, reinstall in the oil-filter sandwich plate; expect ~30-60 PSI at idle.

## Test tools / gotchas
- **Meter is a clamp meter (KAIWEETS HT206D)** — current only via the 60A/600A clamp jaw, **no series-mA mode**, useless for 4-20mA. Use **DC Volts + Continuity**.
- **Read loop current with the voltmeter:** V(AI+ to GND) ÷ 500 Ω. 2.0 V = 4 mA (0 PSI), 10 V = 20 mA (150 PSI).
- **Resistor substitution (bypass the module):** 24V+ → Pin1; Pin3 → [100-500 Ω] → 24V−; current = V_across_R ÷ R.
- Do bench polarity sweeps at **12V** (P51 reverse limit is ±16V); never wire it reversed.

## Don't
- Don't RMA more P51 sensors. Don't keep channel-hopping expecting a different result. Don't commit (user gates all commits).
