# Electric Box — DDR-15G-24 Isolator + 1.5KE36A TVS Wiring

Install reference for the two parts that close out the oil-pressure fix (see `oil_pressure_isolation_saga.md` for *why*). The **Mean Well DDR-15G-24** gives the P51 oil-pressure loop its own floating 24 V island so its 4–20 mA can't sneak around the Waveshare sense resistor. The **Littelfuse 1.5KE36A** is a surge clamp / reverse-polarity fail-safe across the 24 V sensor rail.

**TL;DR:**
- DDR-15G-24 **input** ← car 12 V (fused 1 A). **Not** the box 24 V rail — that re-bonds the grounds and defeats the isolation.
- DDR-15G-24 **output** → powers only the P51 loop. **−Vo ties to AI1− and nothing else.**
- 1.5KE36A goes **across the 24 V rail**, **band (cathode) → +24 V**. It's unidirectional — backwards it shorts and blows the rail fuse.

---

## Part-spec facts (datasheet-confirmed)

| Item | Confirmed | Source |
|---|---|---|
| Mean Well DDR-15G-24 | DIN-rail DC-DC. **In 9–36 V → out 24 V / 0.63 A (15 W)**. **I/O isolation 4 kVdc**, >100 MΩ. Internal **reverse-polarity protection** (auto-recovers). UVLO: on ≥ 9 V, off ≤ 8.5 V. **Terminals: pin 1 = +Vo, pin 2 = −Vo, pin 3 = −Vin, pin 4 = +Vin.** | [DDR-15 datasheet](https://www.meanwell.com/Upload/PDF/DDR-15/DDR-15-SPEC.PDF) |
| Littelfuse 1.5KE36A | **Unidirectional** 1500 W TVS, DO-201AE axial. **Standoff (V_RWM) 30.8 V**, breakdown 34.2–37.8 V, **max clamp 49.9 V @ 30.1 A**. **Banded end = cathode.** | [1.5KE36A — Newark/Littelfuse](https://www.newark.com/littelfuse/1-5ke36a/tvs-diode-1-5kw-36v-do-201/dp/17H2647) |
| P51-150-G-B-P-20MA | 2-wire 4–20 mA loop, 8–30 V. **Pin 1 = Vin (red), Pin 3 = loop return (black), Pin 2 = unused (yellow, cap).** | CLAUDE.md (bench-confirmed 2026-06-14/17) |

Why 24 V out (not 12): at 150 PSI the P51 pulls 20 mA through the module's 500 Ω = 10 V burden, plus the sensor's ~8 V floor = ~18 V minimum. A 12 V supply browns out above ~35 PSI.

---

## 1. DDR-15G-24 isolator — the oil-pressure loop

```
     CHASSIS SIDE (car ground)          ║        FLOATING 24 V ISLAND (loop only)
                                        ║
  car +12V ─[1A fuse]─► pin 4  +Vin ─┐  ║  ┌─► pin 1  +Vo ─────────► P51 Pin 1  (red, Vin)
                                     │  ║  │                                 │
                       ┌─────────────┴──╫──┴─────────────┐          (P51 regulates 4–20 mA)
                       │           DDR-15G-24             │                  │
                       │      9–36 V → 24 V, 4 kV iso     │          P51 Pin 3 (black)
                       └─────────────┬──╫──┬─────────────┘                  │
                                     │  ║  │                                ▼
  car GND  ───────────► pin 3  −Vin ─┘  ║  └─► pin 2  −Vo ◄──────── Waveshare AI1−
                                        ║                                   ▲
                              isolation barrier              [ internal 500 Ω sense ]
                              (no shared ground)                            │
                                        ║              Waveshare AI1+ ◄──────┘

  P51 Pin 2 (yellow) ─► capped, heat-shrunk (unused on a 2-wire loop)
```

Current path (one closed loop, all through the sense resistor):
`+Vo → P51 Vin → [P51 sets 4–20 mA] → P51 return → AI1+ → 500 Ω → AI1− → −Vo`

### Step-by-step

1. **Input:** car switched +12 V through a 1 A inline fuse → **pin 4 (+Vin)**; car ground → **pin 3 (−Vin)**. Do **not** feed the input from the box 24 V rail.
2. **Output +:** **pin 1 (+Vo)** → P51 **red (Pin 1)**.
3. **Loop:** P51 **black (Pin 3)** → Waveshare **AI1+**.
4. **Return:** Waveshare **AI1−** → **pin 2 (−Vo)**.
5. **Cap** the P51 **yellow (Pin 2)** — unused.
6. Confirm the **AI1 channel jumper is ON (current mode)** and firmware is **Mode 3** (both already true in v6.6).

> **The one rule:** `−Vo` connects to `AI1−` and to nothing else. No chassis, no box ground, no other terminal. That single dedicated return is the entire fix — it forces every microamp of the loop through the 500 Ω sense resistor.

The DDR's 9–36 V input covers 12–14.4 V car power with margin, and its built-in reverse-polarity protection means a momentary input miswire won't kill it. Mount it on the DIN rail next to the Waveshare module.

---

## 2. 1.5KE36A TVS — 24 V rail protection

```
   +24 V rail ──[rail fuse]──┬───────────────────────► to Waveshare + 4× PRTXI temp loops
                             │
                            ═╪═   ◄── banded end (cathode) to +24 V
                             ▽        1.5KE36A  (1500 W unidirectional TVS)
                             │
   GND rail ─────────────────┴───────────────────────►
```

- **Placement:** straight across the **24 V sensor rail** (the one feeding the four PRTXI temp loops), downstream of the rail fuse, physically close to the Waveshare module.
- **Polarity (critical — it's unidirectional):** **banded end (cathode) → +24 V**, plain end → GND. Backwards, it conducts at 24 V and blows the rail fuse (or cooks itself).
- **What it does:** invisible below 30.8 V; on a transient above ~34 V it avalanches and clamps the rail to ~50 V max, absorbing the spike. If the rail is ever reversed it forward-conducts like a short and blows the fuse — a fail-safe, not a failure.
- **Why the 36 V value on a 24 V rail:** 30.8 V standoff sits safely above 24 V so it never conducts in normal use, yet clamps well below anything that would damage the module. **Do not** put a 36 V TVS on the 12 V input — it wouldn't clamp until 34 V, too high to protect 12 V parts (that line needs a lower-voltage TVS).
- The isolated P51 island (section 1) does **not** need its own TVS — the DDR's regulation + isolation shield it, and this rail TVS covers the module and temp channels.

---

## Fuse sizing

| Circuit | Fuse | Notes |
|---|---|---|
| DDR-15G-24 input (car 12 V) | **1 A** | Module draws < 0.1 A; 1 A is protection for the wire, not the load |
| 24 V sensor rail | existing rail fuse | TVS sits downstream of it so a clamp event blows this fuse |

---

## Validate after install

1. **Bench first:** car 12 V → DDR → P51 → AI1+ → 300 Ω → AI1− → DDR −Vo. Voltmeter across the 300 Ω should read a steady ~1.2 V (≈ 4 mA), no jumping. *(Loop math: mA = V ÷ R.)*
2. **Voltmeter trick in the box:** V(AI1+ to GND) ÷ 500 Ω = loop mA. ~2.0 V = 4 mA = 0 PSI at rest, dead-steady, no toggling to `---`.
3. **Apply pressure** (pump or running engine): reading climbs smoothly — expect ~30–60 PSI at idle, more under load.
4. Confirm AI2–AI5 (oil / trans / steer / diff temps) still read real values.
5. **TVS check:** at rest the rail sits at 24 V and the TVS does nothing (correct). If the rail fuse blows on power-up, the TVS is in backwards — flip it (band to +24 V).

---

## Related docs

- `oil_pressure_isolation_saga.md` — the full diagnosis (why box power failed and an isolated supply fixes it)
- `CLAUDE.md` → "Pressure Sensor Install Status" and gotchas #19/#20
- `OIL_PRESSURE_HANDOFF.md` — prior handoff notes

*DDR-15G-24 installed and ✅ VALIDATED on-car 2026-07-04 — oil pressure 99.7% valid, 37–112 PSI, stable engine on/off. 1.5KE36A TVS still to install: band (cathode) → +24 V across the Victron 24 V output rail.*
