# Oil Pressure Sensor — The Grounding/Isolation Saga (and the fix)

**TL;DR:** The oil-pressure P51 kept reading `---` / 0 PSI (jumping, then dead). After a long hunt through "dead channels" and "bad sensors," the real cause was a **grounding/isolation fault**: the P51's 4-20 mA loop shared the electric box's power-and-ground, so its 4 mA had a sneak path *around* the Waveshare's sense resistor. **Both P51 sensors are good; the Waveshare module is good; no channel was ever bad.** Fix = power the oil-pressure loop from an **isolated 24 V DC-DC** (Mean Well DDR-15G-24 or Traco TRN 3-1215) fed off car 12 V, with the converter's output− tied to AI1− only. Firmware reverted to the clean original 5-channel mapping in **v6.6**.

---

## The symptom

On key-on (engine off), oil pressure jumped between `0 PSI` and `---` for under a minute, then settled to `---` forever. Off a couple minutes, restart → it jumps again, then dies. Restart too soon → stays `---`. In the logs the channel read **only two values, ever**: a clean ~4.00 mA (4000–4005 µA = exactly 0 PSI) or exactly 0 — binary, never anything in between, ~150 connect/disconnect toggles in 8 minutes. The temp sensors on the same module sat rock-steady the whole time.

## What it was NOT (every dead end, in order)

1. **"First P51 is defective — RMA it."** Wrong. The original 20000 µA / pegged-150 reading was a **jumper mistake**: CH1's hardware jumper was left in voltage mode (OFF) from the old PX3 sensor. A 4-20 mA loop driven into a high-Z voltage input rails to full scale. Not a dead sensor.
2. **"The Waveshare module is the bad actor — replace it."** Wrong. This was the standing verdict for a while (CH1 "dead," CH6 "intermittent," CH2 "flat zero"). All of those were the grounding fault showing up on whatever channel oil pressure happened to be on.
3. **Channel A/B swaps (firmware v6.3 → v6.4 → v6.5).** Oil pressure was moved CH1 → CH6 → CH2 → CH3 trying to find a "good" channel. The fault followed the sensor's loop every time — which, read correctly, means it is **not** the channel.
4. **"It's signal noise — add a filter / cap / ferrite."** Wrong. The failure is a clean drop to **exactly 0** (an open/diverted current), not jitter around 4 mA. You can't filter a signal that isn't flowing. And a sibling 4-20 mA channel (diff temp) was perfectly clean on the same module/box/converter — no noise present.
5. **"It's a PTC fuse tripping from a short / overcurrent."** Wrong. A PTC trips on *over*-current; the logs never show current above ~4.005 mA. A 2-wire 4-20 mA transmitter is a current *regulator* — it physically can't source a fuse-tripping overcurrent unless its own regulator dies short, and then it couldn't keep returning to a perfect 4.00 mA (which it did, hundreds of times).

## The breakthrough (how it was actually localized)

**Bench-isolate the sensor.** Off the car: 12 V supply → P51, a 300 Ω resistor as the load, voltmeter across the resistor (mA = V ÷ R; the KAIWEETS is a clamp meter and can't do series mA). Result: a dead-steady ~4.05 mA for 10+ minutes — no jumping, no dropout under heat-gun or wiggle. **Both P51 units passed.** The sensor and its pigtail are good, full stop.

**Then the decisive comparison:**
- P51 on the **HANGELL bench supply** (isolated, floating, dedicated) → rock-steady 0 PSI, read correctly by the module, even sitting inside the box.
- P51 on the **electric box** (Victron Orion-Tr, shared with the module) → jumps then `---`. Same on 12 V or 24 V from the box.
- P51 on the **car battery** (chassis ground) → `---`.
- **Diff temp (another 4-20 mA loop) ran perfectly on box power the entire time.**

So the module is fine (diff temp proves it), the sensor is fine (bench proves it), and the channel is fine (the swap proved the fault follows the sensor's *loop*, not the channel). The **only** variable that flipped pass/fail was the **power source's grounding**: a floating/dedicated supply worked; any supply sharing the box's ground did not.

## Root cause

A 4-20 mA current loop must be one closed circuit whose current returns through the receiver's sense resistor. The Waveshare measures AI+ → [internal ~500 Ω] → AI−. When the P51 was powered from the box's shared rail, the loop's ground was common with the module (and the rest of the system) at more than one point, giving the 4 mA a **sneak path back to supply− that bypassed the sense resistor**. The module then saw ≈0 through its resistor and called the sensor offline — even though the sensor was faithfully sourcing 4 mA. The Victron Orion-Tr is an **isolated** converter, so the module's measurement ground floats relative to chassis; the chassis-bound battery and the shared box rail both fought that reference, while the mains-isolated HANGELL (dedicated to just the loop) gave it a single clean return. The temp PRTXIs tolerated the shared grounding; the P51, sitting at the very bottom of the range (4 mA), did not.

> Why the signature was binary "4.00-or-0": a diverted/sneak-path loop reads the full current when the sneak path is open and ≈0 when it's active — never an intermediate value. Same fingerprint as an intermittent open. That's exactly why it never looked like noise.

## The fix

Give the oil-pressure loop its own **floating reference** — what the bench supply did — permanently:

- **Part:** an *isolated* 24 V DC-DC converter. Chosen: **Mean Well DDR-15G-24** (DIN-rail, 9–36 V in → 24 V out, 4 kV isolated) as primary; **Traco TRN 3-1215** (board-mount, 9–18 V → 24 V, isolated) as backup. A plain non-isolated 12→24 V boost converter will **not** work — it shares ground in-to-out.
- **Output must be 24 V** (not 12 V): at 150 PSI the P51 pulls 20 mA through the module's 500 Ω = 10 V burden, plus the sensor's 8 V minimum = an 18 V floor. A 12 V supply would brown the sensor out above ~35 PSI.
- **Power the converter's INPUT from car 12 V**, *not* the box's 24 V rail. The module runs on the isolated Victron output, so feeding the converter from that same rail would short input− to output− across the isolation barrier and undo the fix. Car 12 V (chassis) keeps the two grounds separate.
- **Wiring:**
  ```
  Car 12V  → converter input (+ / −)        (chassis side)
  +24V out → P51 red (Vin)
  P51 black → Waveshare AI1+
  AI1−      → −24V out                       (output− ties ONLY to AI1−)
  P51 yellow → capped
  ```

## Firmware (v6.6)

Reverted all the diagnostic channel swaps to the clean original 5-channel layout, since no channel was ever bad:

| Waveshare input | Sensor | Constant |
|---|---|---|
| AI1 | Oil Pressure (P51, via isolated DC-DC) | `MODBUS_CH_OIL_PRESSURE = 0` |
| AI2 | Oil Temp (PRTXI) | `MODBUS_CH_OIL_TEMP = 1` |
| AI3 | Trans Temp (PRTXI) | `MODBUS_CH_TRANS_TEMP = 2` |
| AI4 | Steer Temp (PRTXI) | `MODBUS_CH_STEER_TEMP = 3` |
| AI5 | Diff Temp (PRTXI) | `MODBUS_CH_DIFF_TEMP = 4` |

`MODBUS_NUM_CHANNELS` 6→5; boot configures CH1–CH5 for Mode 3 (the dead-CH1 skip and the CH6 spare are removed). All five channel jumpers must be **ON (current mode)** — AI1 especially, which spent the saga in voltage mode. No change to the PSI/temp math.

## Lessons for next time

- **A binary "perfect-or-zero" reading is an open or a diverted/short path — never noise.** Don't reach for filters/caps.
- **A 2-wire 4-20 mA transmitter is a current regulator.** It can't make an overcurrent; "find the short that trips the fuse" is usually a ghost.
- **Swap the suspect part to localize.** When the fault follows the sensor across channels, it isn't the channel. When a sibling sensor works on the same hardware, it isn't the module/supply.
- **"Works on a bench supply, dies wired into the system" = a grounding/isolation problem,** not the sensor. The cure is an isolated, dedicated supply (or a 4-20 mA loop isolator), not a better sensor.
- **Isolated converters float their output from their input** — only useful if you don't re-bond the two grounds externally. Power from a different ground than the receiver senses against.

## Validate after install

1. **Bench rig first:** car 12 V → converter → P51 → AI1+ → 300 Ω → AI1− → converter−. Steady ~4 mA, no jumping.
2. **In the box:** AI1 jumper ON; `[OILP] CH1` logs a steady ~4000 µA / 0 PSI at rest, no toggling.
3. **Apply pressure** (pump or running engine): the reading climbs smoothly (expect ~30–60 PSI at idle, more under load).
4. Confirm AI2–AI5 (oil / trans / steer / diff temps) all read real values, no `n/a`.

---
*Recorded after the v6.6 rollback. Supersedes the earlier "RMA the sensor / replace the module" verdicts, both of which were wrong.*
