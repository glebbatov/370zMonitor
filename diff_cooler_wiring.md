# Z1 Diff Cooler — Electrical Wiring Plan

Part-verified plan for the differential cooler side of the 370zMonitor install. Pump = Tilton 40-524, fan = SPAL 30103011, thermo switch = Setrab SUSA 31-TS200-08, relays = Pico 5593PT (×2), temp sensor = PRTXI on Modbus CH5.

---

## Part-spec facts (research-confirmed)

| Item | Confirmed | Source |
|---|---|---|
| Tilton 40-524 pump | **Red = +12V, Black = ground**. 2–3A typical, **6.6A max, 8A worst case**. **10A inline fuse + 16 AWG min** (Tilton-mandated). Self-priming up to 8 ft. Polarity-sensitive (wrong polarity = no flow, no damage). Mount with motor ABOVE ports; feed (from cooler) on TOP port, return (to diff) on BOTTOM port. | [Tilton 98-1901 PDF](https://tiltonracing.com/wp-content/uploads/98-1901-Transmission-Differential-Oil-Cooler-Pump-web.pdf), [Z1 21139-IM Rev C](https://cdn.z1motorsports.com/upload/Install_Z1_Differential_Cooler_Kit.pdf) |
| SPAL 30103011 fan | **Two unmarked black leads**; polarity printed on housing (`+`/`-`). **Reversing leads reverses rotation** → blade becomes inefficient pusher. **2.2A nominal, up to 5A behind a restrictive core, 15–25A inrush 100–300ms**. Recommend **7.5A slow-blow ATC + 18 AWG** (or 10A + 16 AWG for margin). Bench-test polarity with 9V battery before final install. | [SPAL VA31-A101-46A](https://www.spalautomotive.com/en/axial-fans/va31-a101-46a/33089) |
| Setrab 31-TS200-08 | **2 spade terminals, non-grounding (isolated)**. NO, closes at 200°F, opens at 185°F (15°F hysteresis). **10A @ 12VDC contact rating**. **Not polarity sensitive** — either terminal either way. PTFE tape **1.5–2 turns max** (more distorts the port). | [Setrab install PDF](https://cdn.shopify.com/s/files/1/0257/2599/3014/files/susa_instructions_31-TS-AN_rev20250611e.pdf?v=1749678878) |
| Pico 5593PT | Standard ISO 7588 pinout: **85/86 = coil**, **30 = common**, **87 = NO**, **87a = NC**. Coil ~85Ω, **~150 mA at 12V**. Pigtail has 2× 18 AWG (coil) and 3× 12 AWG (load). **Pico does NOT publish wire colors — beep continuity to each terminal stub before crimping**. **No inline fuse in the harness** (external only). | [Summit PCO-5593PT](https://www.summitracing.com/parts/pco-5593pt) |
| PRTXI 4-20mA loop | Already proven on your CH2–CH4 — same scheme for CH5. **Belden 8761** (or 9841) **shielded twisted pair, 22 AWG**. **Shield grounded at the Waveshare end only** (cap and heat-shrink the diff end). Keep ≥12" from spark plug wires, alternator B+, fuel pump feed. Cross power wires at 90°. | [PRTXI datasheet](https://rspsupply.com/images/downloads/Omega/P/Omega%20PRTXI-1-4N-1-4-4-IO/PRTXI_spec.pdf), [Waveshare wiki](https://www.waveshare.com/wiki/Modbus_RTU_Analog_Input_8CH) |

---

## Topology

```
  ENGINE BAY (electric box)                                            DIFF AREA
  ─────────────────────────                                            ──────────
                                                                       
  [Battery bus +]──FUSE 25A──┐                                         ┌─[Mini fuse block]
                             │  WIRE A: 10 AWG +12V battery feed       │     │
                             └─────────────────────────────────────────►│     ├─FUSE 10A──┐
                                                                       │     │            │       ┌────────────┐
                                                                       │     │            ├──[30]─┤            │
                                                                       │     │            │  [87]─┤ PUMP RELAY │──► Tilton RED (+)
                                                                       │     │            │       │   (Pico)   │
                                                                       │     │            │       └─[85][86]───┘
                                                                       │     │                       │    │
                                                                       │     ├─FUSE 7.5A──┐         GND   │
                                                                       │     │            │      (diff bolt)
                                                                       │                  │       ┌────────────┐
                                                                       │                  ├──[30]─┤            │
                                                                       │                  │  [87]─┤ FAN RELAY  │──► SPAL (+)
                                                                       │                  │       │   (Pico)   │
                                                                       │                  │       └─[85][86]───┘
                                                                       │                                │    │
                                                                       │                              GND   │
                                                                       │                                    │
  [Washer-tank fuse]──┐                                                │                                    │
   (ignition-sw +12V) │  WIRE B: 18 AWG trigger feed                   │                                    │
                      └───────────────────────────────────────────────►│  ┌─► Setrab spade A              │
                                                                       │  │                                 │
                                                                       │  └─► Under-car inline switch IN    │
                                                                       │                                    │
                                                                       ▼                                    │
                                                                  Setrab spade B ──────────────────────────┘ (fan coil 86)
                                                                       
                                                                  Under-car switch OUT ──┐
                                                                                         │
  CABIN                                                                                  ▼
  ─────                                                                              NODE Y ──► pump coil 86
  Tap WIRE B in cabin ──► Cabin rocker (SPST) ─── WIRE C: 18 AWG ─────────────────────►┘
                                                                       
  ELECTRIC BOX                                                         
  ────────────                                                         
  Waveshare AI5+ ◄── (signal)  CABLE D: Belden 8761 shielded pair ──── PRTXI Pin 2 (signal/V-)
  +24V loop supply ─► (V+)                                             PRTXI Pin 1 (V+)
  Waveshare AI5-  ─► (return to 24V GND at electric box)
  Shield → chassis ground at electric box ONLY                         Shield: cap & heat-shrink
```

**Key logic decisions:**
- **Pump trigger = "OR" logic** = two switches feeding pump coil 86 from +12V trigger, in parallel. Cabin rocker output and under-car switch output both terminate on node Y → coil 86. Either switch closing energizes coil. Coil 85 is permanently grounded at the diff bolt. (High-side switching is automotive convention and keeps coil ground clean.)
- **Fan trigger = Setrab only**, on the +12V side of fan coil 86 (matches Setrab's own diagram). Coil 85 permanently grounded.
- **Cap and heat-shrink the 87a (NC) lead on both relays** — you only want load energized when the coil is on.
- **Common ground at diff** = one paint-stripped chassis bolt, star washer, anti-seize on threads, dielectric grease on terminals. Both relay coil 85s, pump black lead, fan `-` lead all terminate here.

---

## Wire-by-wire list

**Long runs (engine bay ↔ diff and cabin ↔ diff):**

| # | Wire | Gauge | From | To | Notes |
|---|---|---|---|---|---|
| A | +12V battery feed | **10 AWG** | Engine-bay bus bar (via 25A fuse) | Mini fuse block at diff | Sole high-current trunk feed. Sized for ~12A worst case + voltage drop over ~12 ft. |
| B | +12V ignition trigger | **18 AWG** | Washer-tank fuse tap | Diff area (junction) | Branches at diff to Setrab and under-car switch; also taps in cabin to feed rocker. |
| C | Cabin rocker output | **18 AWG** | Cabin rocker output spade | Pump relay coil 86 (node Y) | Carries ~150 mA when active, floats when off. |
| D | PRTXI loop pair | **22 AWG shielded twisted pair** (Belden 8761) | Waveshare AI5+/AI5- + 24V loop supply at electric box | PRTXI Pin 1 (V+) and Pin 2 (signal) at diff | Shield grounded at electric box end ONLY. Cap shield at diff end. |

**Short runs at the diff (all stay in the diff area):**

| # | Wire | Gauge | From | To |
|---|---|---|---|---|
| 1 | Mini-fuse block in | 10 AWG | Wire A | Mini fuse block + bus |
| 2 | Pump fused +12V | 16 AWG | Mini fuse block (10A fuse) | Pump relay pin 30 |
| 3 | Fan fused +12V | 18 AWG (or 16 with 10A) | Mini fuse block (7.5A fuse) | Fan relay pin 30 |
| 4 | Pump load out | 16 AWG | Pump relay pin 87 | Tilton RED |
| 5 | Pump ground | 16 AWG | Tilton BLACK | Diff ground bolt |
| 6 | Fan load out | 18 AWG | Fan relay pin 87 | SPAL `+` lead |
| 7 | Fan ground | 18 AWG | SPAL `-` lead | Diff ground bolt |
| 8 | Pump coil ground | 18 AWG (pigtail) | Pump relay pin 85 | Diff ground bolt |
| 9 | Fan coil ground | 18 AWG (pigtail) | Fan relay pin 85 | Diff ground bolt |
| 10 | Setrab in | 18 AWG | Wire B branch (trigger feed) | Setrab spade (either) |
| 11 | Setrab out → fan coil | 18 AWG | Setrab other spade | Fan relay pin 86 |
| 12 | Under-car switch in | 18 AWG | Wire B branch (trigger feed) | Inline switch pole 1 input |
| 13 | Under-car switch out | 18 AWG | Inline switch pole 1 output | Pump relay pin 86 (node Y, joins Wire C) |
| 14 | 87a NC pump | — | Pump relay pin 87a | **CAP OFF, heat-shrink** |
| 15 | 87a NC fan | — | Fan relay pin 87a | **CAP OFF, heat-shrink** |

**Cabin:**

| # | Wire | Gauge | From | To |
|---|---|---|---|---|
| 16 | Rocker switch in | 18 AWG | Wire B branch (trigger feed tapped in cabin) | Cabin rocker pole 1 |
| 17 | Rocker switch out | 18 AWG | Cabin rocker pole 1 out | Wire C (going back to diff) |

**Wire count totals:**
- **4 long runs** (Wires A, B, C, D — though D is one cable carrying two conductors + shield)
- **~15 short wires** at the diff (most are pigtails already attached to relays/pump/fan)
- **2 short wires** in the cabin

---

## What you missed / suggestions

1. **No mini fuse block at the diff = ugly wiring.** Rather than running two separate fused feeds from the engine bay, run **one beefier feed (Wire A, 10 AWG / 25A fuse)** to a small distribution block at the diff (Blue Sea 5025 or similar — 4–6 circuits, covered). Branch out from there with the 10A and 7.5A fuses. Cleaner, single point of disconnect, room for future expansion (e.g., trans cooler pump if you add one).

2. **Cap 87a on both relays.** Easy to forget — uncapped NC leads at 12V at rest are a short hazard.

3. **Bench-test fan polarity before final install.** SPAL 30103011 ships bare-lead with two black wires. Use a 9V battery, watch the blade — confirm the blade pulls air *through the cooler in the direction printed on the housing*. Then label `+` and `-` with heat-shrink, permanently.

4. **Bench-test pump priming and rotation.** Tilton 40-524 is polarity-sensitive (wrong polarity = motor spins but no flow). Prime with discharge line open into a catch can; confirm flow direction. Z1 manual: feed = TOP port, return = BOTTOM port.

5. **Inrush margin on the fan fuse.** 5A ATC will nuisance-blow on the SPAL's ~15–25A startup spike. Use **7.5A or 10A slow-blow ATC** ("MAXI" or time-delay ATM variants). Your CLAUDE.md currently says 5A — bump it.

6. **Flyback diode across relay coils.** Pico 5593PT has an internal resistor for suppression, but a 1N4007 diode across each coil (cathode → 86, anode → 85) gives belt-and-suspenders protection for the small switch contacts on the Setrab and the cabin rocker. Optional but standard race-install practice.

7. **Auto-mode mismatch:** Fan is auto (Setrab), pump is manual. If diff overheats and driver doesn't notice the dashboard temp, fan runs but pump doesn't → fan does almost nothing (no fluid flowing through cooler core). Two options:
   - **Trust the driver** (current plan — fine, but rely on the 370zMonitor display + critical-temp toast at 260°F).
   - **Tie fan to pump-on as well**: tap pump's +12V (relay pin 87 output) to fan coil 86 via a diode, in parallel with the Setrab path. Then fan runs any time pump runs OR temp >200°F. This is the safer config but adds wires.

8. **Pump cold-start protection.** Tilton explicitly warns against running on cold heavy gear oil. You already have manual control, which satisfies this — just discipline yourself to not turn the pump on until water temp is at operating range. If you ever want firmware-side protection, a one-liner that blinks a warning on the pump's dashboard tile when ECT < 80°C would do it — but per your memory, no firmware changes during install phase.

9. **PRTXI sensor mounting.** Where does the diff PRTXI probe go? It's a 1/4" NPT probe. Z1 high-cap diff cover may only have 1× 1/8" NPT port (for the Setrab). Confirm you have a 1/4" NPT bung available — common option is a brass tee in the cooler return line, or a second port in the cover if drilled/tapped.

10. **Under-car inline switch is DPST but you only need SPST.** The product you linked switches two independent circuits. Wire just one pole (pole 1). Cap the other pole's leads. Don't accidentally use the second pole for anything safety-critical — those bargain switches sometimes have mediocre internal isolation.

11. **Label every wire at both ends.** Cheap heat-shrink labels + Brother PT label gun. In a year when you're crawling under the car with one hand on a multimeter, you will not remember which 18 AWG black wire goes where.

12. **Routing.** Run the PRTXI shielded pair (Cable D) along the factory chassis harness path, NOT alongside the +12V battery feed or the cabin trigger wire. Keep it ≥12" from spark plugs, alternator B+, fuel pump feed, and the starter cable. Use split loom for chafe protection. Use grommets at every body panel pass-through.

13. **Diff ground bolt.** Should be a clean, paint-stripped chassis bolt (subframe or diff cover bolt with no rubber isolator), star washer, anti-seize on the threads, dielectric grease on the ring terminals. Don't ground to a body panel or thin sheet metal.

14. **Future-proof the cabin rocker.** If you ever want "OFF / AUTO / FORCE-ON" you'll want an SPDT (3-position) rocker. The DaierTek SPST you specified is fine for your current "manual only" plan, but worth noting.

---

## Quick verification checklist (before powering up)

- [ ] Continuity test every Pico pigtail wire to its relay terminal — write down the color↔pin mapping
- [ ] Bench-test pump on 9V or 12V supply — confirm flow direction (RED = +)
- [ ] Bench-test fan on 9V — confirm pulls in printed direction
- [ ] Confirm Setrab spades isolated from body with a meter (>1 MΩ to case)
- [ ] Confirm PRTXI loop reads ~4 mA at room temp, climbs as you heat the probe
- [ ] Heat-shrink and dielectric grease every crimp
- [ ] Test cabin rocker → pump runs; release → pump stops
- [ ] Test under-car inline → pump runs; release → pump stops
- [ ] Heat Setrab probe (heat gun, briefly) → fan runs; cool → fan stops (with ~15°F hysteresis)
- [ ] Confirm 87a leads are capped on both relays
- [ ] Confirm fuses match plan (25A trunk, 10A pump, 7.5A fan)

---
