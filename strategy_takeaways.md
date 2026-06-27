# Solar Car Race Strategy — SSCP Takeaways

Actionable conclusions for SSCP: model gaps, what code exists, optimizer constraints, and open questions.
See [research_notes.md](research_notes.md) for the underlying literature.

---

## 1. Literature Motivation

### 1.1 Why a simulation-based optimizer, not an analytical solution

The Pudney & Howlett line (University of South Australia, 1990s–2000s) proves closed-form results about optimal driving modes: constant speed is optimal on flat roads with a simple battery, and a "critical speed" strategy (two alternating speeds) is optimal on undulating roads once battery nonlinearity is accounted for. These results are elegant and exactly correct within their assumptions.

We didn't follow this line because our model violates its key assumptions:
- **Grade sensitivity:** the motor is highly sensitive to slope (flagged in 1994 SSCP notes; confirmed by the WaveSculptor datasheet). Pudney's proofs assume grade effects are separable. In practice, a steep climb at v* is very different from flat driving at v*.
- **Weather variability:** real-race solar input **changes hour-to-hour.** Pudney's steady-state model fixes GHI; ours varies it per segment.
- **Crosswind drag:** `CdA(β)` varies with yaw angle, which changes continuously along a real route. Not representable in the analytical framework.

The Michigan (Betancur/Yesil) line takes the alternative approach: build a detailed simulation and run a numerical optimizer over it. Michigan ran multiple simulations in parallel with different weather inputs mid-race and used the spread to judge forecast uncertainty. This is the practical approach for real racing — re-runnable in minutes when conditions change, and naturally handles all the nonlinearities.

**Our choice:** simulate each segment with the full drivetrain equation (Betancur, research_notes.md §2.3), then minimize total time with SLSQP (gradient-based, fast), seeded by the Pudney critical speed v\*. Same philosophy as Michigan; different optimizer.

### 1.2 Why SLSQP as the primary optimizer

SLSQP is the gradient-based workhorse: ~7 seconds to converge on a 3000-segment route. This matters for live race use — re-running the optimizer mid-race after a cloud update needs to complete before the car has driven past the relevant segment.

**Literature validation:** Betancur et al. compare GA, BB-BC (Big-Bang Big-Crunch), and exhaustive search on the same model. Exhaustive search won but is impractical live; BB-BC was fastest with comparable results. We use SLSQP instead, which is faster still for smooth differentiable objectives and is well-validated in the scipy stack.

### 1.3 Why these specific model terms

| Term | Literature source | Rationale |
|---|---|---|
| Drivetrain: v³ aero + linear rolling | Betancur Eq. 1; confirmed by Pudney | Dominant power consumption terms. Rolling resistance linear; aero cubic — small speed increases are expensive. |
| `Ps = Ii·Ai·ηs·sin(φ)` | Betancur solar sub-model | Standard flat-panel model. GHI from Solcast replaces the irradiance model Betancur uses internally. |
| `ηb` round-trip efficiency | Betancur | Simple and well-validated; battery quadratic loss is small relative to drivetrain at cruise. |
| `I(b)` quadratic battery curve | Pudney | Captures voltage sag near discharge limit; relevant near Eb_min where the optimizer presses hardest. |
| Per-segment speed vector | Michigan / Betancur / Pudney all use it | Flexible: captures speed limit changes, grade variation, overnight stops at segment boundaries. |
| Overnight SoC constraint | Novel application | Not explicit in any paper (single-day races); natural extension — battery floor at end of each race day. |

### 1.4 What the literature does not cover (gaps we have to fill)

- **Multi-day race structure** — all reviewed papers model single-day or continuous races. ASC's 9 am–6 pm tour-day structure with overnight charging is an SSCP addition.
- **Live telemetry integration** — Michigan describes the workflow (chase car strategists re-running simulations) but not the software architecture. Our UDP-based live input loop is original.
---

## 2. Model Selection

**Well-sourced from literature (Betancur, Pudney):**
- All terms in the drivetrain equation
- `Ps = Ii·Ai·ηs·sin(φ)` — solar power
- `I(b) = c1·b + c2·b²` — battery current model (Pudney)
- `ηb` round-trip battery efficiency (Betancur)

**Synthesized / extended here (physically sound but not explicitly in papers):**
- `I(b, T, SoC)` — extending Pudney's static curve to depend on temperature and SoC. Standard in battery modeling; enabled by SSCP's live per-cell voltage and pack temperature telemetry.
- Using live MPPT V×I as a direct `Ps` measurement rather than computing from irradiance model.
- Solcast + GPX road heading + `CdA(β)` yaw curve for per-segment crosswind drag.

**Known gaps — real physical effects not currently in the model:**

| Effect | Impact | Priority | Notes |
|---|---|---|---|
| Motor efficiency varies with operating point (RPM × torque) | Moderate–significant | V2 | '94 notes flag motor sensitivity to grade. At high torque / low speed (steep climbs) efficiency drops significantly. Current model uses constant ηm. Source from WaveSculptor datasheet or bench test. |
| Panel efficiency drops with temperature | Small–moderate | V2 | ~0.3–0.5% per °C above rated test temperature. Betancur bakes convective cooling into a fixed ηs. Panel temp needs a sensor or thermal model. |
| Auxiliary power draw | Small, systematic | V1 | Electronics, comms, displays draw ~50–100 W constantly. Add as a constant parasitic load in `energy_delta_Wh`. |
| Battery voltage sag under high discharge | Small | V2 | Pudney's `I(b)` quadratic captures this implicitly; Betancur's constant `ηb` does not. Only relevant near discharge limits. |
| Spatial wind variation | Small in open terrain | Low | Solcast gives per-segment wind but wind is locally variable. Fine for open outback; worse through channeling terrain. |
| Motor top speed / torque-speed curve | Small in practice | Low | WaveSculptor has a rated peak RPM; above it the motor enters flux-weakening. `v_max` is currently a hard 100 km/h clip, not derived from motor physics. Fix: compute from WaveSculptor peak RPM + gear ratio + wheel diameter. `max_speed_from_discharge_limit()` already handles the climb case. |

---

## 3. Existing Strategy Code

### 3.1 Battery SoC — yes, but primitive

- [`motherboard/src/bms/state.rs`](sunstruck_onboard_rust_impl/motherboard/src/bms/state.rs): SoC is a simple linear interpolation of minimum cell voltage (2.5 V → 0%, 4.2 V → 100%). No coulomb counting, no model-based estimation.
- [`motherboard/src/bms/sensors/current.rs`](sunstruck_onboard_rust_impl/motherboard/src/bms/sensors/current.rs): Energy (Wh) and charge (Ah) are integrated at 100 Hz from a current shunt. These accumulators exist and are broadcast over telemetry — this is coulomb counting data, it's just not being fed back into the SoC estimate.

### 3.2 Motor power / speed control — yes, but no grade sensitivity or efficiency map

- [`motherboard/src/vehicle/drive.rs`](sunstruck_onboard_rust_impl/motherboard/src/vehicle/drive.rs): Generates motor drive commands. Three cruise modes: constant RPM, constant bus current, off. Currently no PID — cruise is constant-speed + 100% torque. **Known open issue.**
- Motor messages (`messages/src/motor/mod.rs`) use Tritium WaveSculptor protocol: velocity setpoint in RPM, current as a fraction 0.0–1.0. No efficiency map or grade-aware power estimation anywhere in the codebase.

### 3.3 Solar array telemetry — logged, not modeled

- MPPT messages (`messages/src/mppt/mod.rs`): 6 channels (3 units × 2 ch), each reports array voltage, array current, battery voltage, and temperature. All data is broadcast over CAN and logged. No irradiance model, no power forecasting.

### 3.4 GPS + elevation — tool exists, not integrated

- [`offboard/elevationProfiler/ElevationProfiler.go`](sunstruck-code/offboard/elevationProfiler/ElevationProfiler.go): A standalone Go tool that loads SRTM3 topographic tiles and processes a CSV route to produce an elevation profile. A KML→CSV converter is also included. This is exactly the Michigan pre-race route survey workflow — but the output is not connected to anything in the vehicle firmware or any optimizer.
- Live GPS (NEO-F9P / ZED-F9P with RTK corrections from EarthScope P221) is implemented in firmware and outputs `GpsPosition`, `GpsStatus`, `GpsTime` messages. **Known issue: the GNSS receiver may have a hardware problem as of June 2026.**

### 3.5 Telemetry logging — robust

- [`motherboard/src/comms/mod.rs`](sunstruck_onboard_rust_impl/motherboard/src/comms/mod.rs): UDP broadcast every 100 ms to off-site proxy. All live data (voltage, current, MPPT, GPS, SoC, Wh, Ah, motor commands) is on the wire.
- [`motherboard/src/sd.rs`](sunstruck_onboard_rust_impl/motherboard/src/sd.rs): SD card circular log at 100 ms cadence. 59 CAN message slots + per-cell voltage + per-cell temperature arrays per record. Capacity ~18 days at 100 ms intervals. This is the raw material for any strategy model.
- [`offboard/telem/`](sunstruck-code/offboard/telem/): Python tools to receive telemetry UDP, write to InfluxDB + CSV, and replay from log files.

### 3.6 Max's model — prior SSCP optimizer (last race)

MATLAB-based car model used at the most recent race. Direct predecessor to `python/optimize.py`. Find it before re-deriving physics from scratch.

---

## 4. Strategy Gap Summary

### 4.1 Pre-race / offline inputs

| Capability | Status |
|---|---|
| Route elevation profile | **Have it** — GPX + SRTM3 tool in repo. Needs smoothing. |
| Speed limits along route | **Partial** — not in GPX; pull from OpenStreetMap or mark manually |
| CdA | **Can get it** — from aero model. Combine with Solcast wind + GPX heading for per-segment crosswind drag. |
| Crr | **Unknown** — needs rolldown test. Literature estimate ~0.002–0.004. |
| Vehicle mass | **Unknown** — weigh with driver + full pack |
| Battery model I(b, T, SoC) | **Have raw data** — telemetry (V×I, pack temp, Ah) + cell cycler logs. Needs MCP3913 calibration fix first. |
| Motor / regen efficiency | **Unknown** — source from WaveSculptor datasheet or bench test |
| Solar / wind / cloud forecast | **Have it** — Solcast API (GHI, cloud opacity, 10m wind). Confirm access tier fields. |
| Cell balancing | **Currently not implemented** in Rust firmware |

### 4.2 Live inputs (chase car, updated continuously during race)

| Capability | Status |
|---|---|
| Solar power harvested right now | **Have it** — MPPT V×I summed across 6 channels = live Ps directly; logged at 100 ms. No irradiance model needed for live use. |
| Battery SoC / energy remaining | **Have it** — Wh/Ah integrated at 100 Hz, broadcast over UDP |
| Vehicle speed | **Have it** — motor RPM on telemetry stream |
| GPS position | **Have it** — NEO-F9P with RTK (hardware issue TBD as of June 2026) |
| Road grade at current position | **Not implemented** — needs live GPS position matched to pre-surveyed elevation profile |
| Wind speed / direction (live) | **Partial** — no anemometer on car; Solcast nowcast gives a short-horizon estimate at current position but isn't real-time measured |
| Short-horizon cloud forecast | **Have it** — Solcast queried live on ~30 min horizon gives cloud opacity ahead on route |
| Chase car internet connectivity | **Hypothetical** — Starlink discussed as an option; would enable live Solcast queries and telemetry sync |


### 4.4 Key telemetry wire units

| Quantity | Wire unit | Notes |
|---|---|---|
| Pack voltage | 0.1 V (u16) | Sum of all 32 cells |
| Cell voltage | mV (u16) | 2.5–4.2 V range |
| Pack current | 0.01 A (i16) | Positive = discharge |
| Battery energy | Wh (f32) | Integrated at 100 Hz |
| Battery charge | Ah (f32) | Integrated at 100 Hz |
| SoC | % (u8) | Linear in min cell voltage |
| Array voltage | 0.01 V (u16) | Per MPPT channel |
| Array current | 0.001 A (u16) | Per MPPT channel |
| GPS lat/lon | degrees (f32) | WGS84, RTK available |

### 4.5 Vehicle parameter status

All placeholders until measured. High-impact parameters will shift optimizer output significantly.

| Parameter | Value | Impact | Source needed |
|---|---|---|---|
| Mass `m` | 300 kg | High | Weigh with driver + full pack |
| `CdA_flat` | 0.20 m² | High | Aero model |
| `Crr` | 0.005 | High | Rolldown test |
| Panel area `Ai` | 4.0 m² | High | Panel layout |
| `η_s` (panel+MPPT) | 0.20 | High | Datasheet |
| `Eb_max` | 5000 Wh | High | Cell spec |
| `η_b` (round-trip) | 0.95 | Medium | Cell cycler / telemetry fit |
| `η_m` (motor) | 0.97 | Medium | WaveSculptor datasheet |
| `η_regen` | 0.65 | Low–medium | WaveSculptor or descent test |
| `I_discharge_max` | 150 A | Low at cruise | BMS config |

---

## 5. Optimizer Constraints

The optimizer minimizes `Σ d_i / v_i` (total race time) subject to these constraints.

| Constraint | Equation | Implementation | Notes |
|---|---|---|---|
| Battery never runs out | `Eb_i ≥ Eb_min` for all i | ✅ Active — 250 Wh floor | |
| Battery never overcharges | `Eb_i ≤ Eb_max` for all i | ✅ Active — 5000 Wh ceiling | Modeled as a constraint on the unclipped cumsum; does not capture lost solar when clipping is active (see Known Optimizer Limitations) |
| Finish with buffer | `Eb_N ≥ Eb_finish_min` | ✅ Active — 250 Wh | |
| Speed limits (upper) | `v_i ≤ vlimit_i` for all i | ✅ Active — 65 mph / posted limit per segment | Speed limits read from GPX where tagged; default 65 mph elsewhere |
| Minimum speed | `v_i ≥ vmin` | ✅ Active — 20 mph (8.94 m/s) | ASC 2026 §12.22 |
| Discharge rate limit | `Pb_i / V_pack ≤ I_discharge_limit` | ✅ Active — 150 A / 100 V nominal | Encoded in per-segment speed upper bound via `max_speed_from_discharge_limit()` |
| Per-checkpoint SoC | `Eb_checkpoint_k ≥ Eb_checkpoint_min` | ⚙️ Framework built, no checkpoints loaded | Awaiting Route Book for locations and times |
| Checkpoint time windows | `t_open_k ≤ Σt_i ≤ t_close_k` at checkpoint k | ⚙️ Framework built, no checkpoints loaded | Awaiting Route Book |
| Daily overnight SoC | `Eb_end_of_day_k ≥ Eb_overnight_min` | ✅ Active — 500 Wh floor per overnight stop | |
| Regen ceiling clip on descents | `Eb_i ≤ Eb_max` when `Pm_i < 0` | ❌ Not enforced as a constraint | Regen efficiency modeled (η_regen = 0.65, placeholder) but no constraint prevents overcharge on steep descents; irrelevant on flat synthetic route |

### 5.1 Race rules — ASC 2026 (confirmed from Regs Rev C, May 2026)

| Rule | Value | Source |
|---|---|---|
| Tour Day start | 9:00 am (nominal) | §12.10.A.1 |
| Tour Day end | 6:00 pm (nominal) | §12.10.A.1 |
| Overnight grace period | up to 15 min early or 30 min late from 6:00 pm | §12.13 |
| Morning resume | 9:00 am from same overnight location | §12.13 |
| Impound hours | 8:00 pm – 7:00 am | §12.17.B.1 |
| Maximum speed | 65 mph (104.6 km/h / 29.1 m/s) or posted, whichever is lower | §12.21 |
| Minimum speed | 20 mph (32.2 km/h / 8.9 m/s) when posted limit ≥ 60 mph | §12.22 |
| Checkpoint hold time | ~30 minutes (mandatory, at open checkpoints) | §12.14.A |
| Trailering | Defined as failing to complete a Base Leg or Loop under own power | §12.27 |
| Finish requirement | None specified beyond completing Base Route | §12.3.A |

**Ranking for Single-Occupant vehicles (§12.3.B):** highest Official Distance driven wins; elapsed time is tiebreaker. This means the optimizer should maximize distance if the base route can't be completed, not minimize time — though completing the route is the first priority.

**Still need from Route Book (not in regs):**
- Checkpoint locations (mile markers) and open/close times
- Stage Point locations
- Loop options and distances

---

## 6. Known Optimizer Limitations

### 6.1 SLSQP is a local optimizer — morning sprint problem

On days where the battery starts full (or near-full), the true optimal strategy is to **sprint hard in the morning** to drain below Eb_max quickly, freeing headroom to capture solar through midday, then sprint again in the late afternoon to drain to the overnight floor. This produces a "W"-shaped battery curve.

SLSQP doesn't find this because:
- The fast optimizer path (`cumulative_energy_vec`) uses an **unclipped** cumsum with a `soc_ceil` constraint. In the optimizer's world, energy at the ceiling is still "carried" and available later — it doesn't model the physical reality that solar above Eb_max is permanently lost.
- The `soc_ceil` Jacobian is computed only at the single argmax segment. SLSQP sees the constraint as satisfied (slack = 0) and doesn't get a gradient signal pointing toward "drain harder in the morning."
- The symptom: battery pinned at Eb_max for ~110 km (segments 61–82) on day 1, wasting solar that could have been captured by going faster.

**Multi-seed experiment result (10 seeds, flat synthetic route):** all seeds from 32 km/h to 104.6 km/h (v_max) converge to the same solution. This means either:
1. The solution is truly globally optimal on this flat route — v_max is not fast enough to drain the battery before the solar peak; or
2. There is a better solution with a morning sprint, but the gradient landscape around the current solution doesn't point toward it from any of the tested starting points.

To distinguish these, the next step is a **targeted morning sprint seed**: v_max for the first ~28 segments of each day (before the solar peak), v* for the remainder. This tests whether SLSQP can find a better solution from a starting point that already has the sprint shape, without a hardcoded rule.

**Fixes (in order of effort):**
1. **Morning sprint seed** — set seed speed to v_max for segments where projected Eb would exceed ~95% of Eb_max. Tests whether SLSQP can escape the current basin from a better-shaped starting point. More principled than a hardcoded heuristic.
2. Warm-start from the previous optimizer run after re-estimating arrival times.
3. **Global method (DP or stochastic search)** — models battery clipping exactly and guarantees a global optimum within discretization, at higher cost. A DP version was prototyped and removed to keep the codebase SLSQP-only; revisit it (or CMA-ES / simulated annealing) only if better seeding fails to close the gap.

**Priority:** V1 — this is a correctness issue, not just a refinement.

---

## 7. Open Questions / To Investigate

**Testing (critical path):**
- Fix MCP3913 calibration on BMS 2–5 boards before collecting battery model fit data.
- Run controlled speed drives with comms active; use as both calibration data and mock race day practice.
- Weigh car with driver + full pack; get CdA from aero model; run rolldown test for Crr.

**Infrastructure:**
- Confirm which Solcast fields are available under SSCP's free access tier (GHI vs. DNI vs. DHI).
- Confirm GNSS hardware issue (NEO-F9P / ZED-F9P as of June 2026).
- Fix cruise control PID in `drive.rs`.

**Race planning:**
- Get Route Book when available (checkpoint locations, stage points, loop options).
- Coordinate with aero subteam — the optimizer can quantify race time saved per unit CdA reduction, giving them a concrete target.
