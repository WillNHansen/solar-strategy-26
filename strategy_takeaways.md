# Solar Car Race Strategy — SSCP Takeaways

Actionable conclusions for SSCP: model gaps, what code exists, optimizer constraints, and open questions.
See [research_notes.md](research_notes.md) for the underlying literature.

---

## 1. Model Equations

### 1.1 Full problem

We optimise over both speeds **v[0..N−1]** and battery states **E_b[0..N−1]** (one per segment) jointly — the *shooting formulation*:

$$\min_{\mathbf{v},\,\mathbf{E}_b} \sum_{i=0}^{N-1} \frac{d_i}{v_i}$$

subject to the **energy dynamics** (equality constraints):

$$E_b[0] = E_b^{\text{start}} + \Delta E_0 + Q_0, \qquad E_b[i] = E_b[i-1] + \Delta E_i + Q_i \quad \text{for } i \geq 1$$

and **variable bounds**:

$$E_b^{\min} \leq E_b[i] \leq E_b^{\max}, \qquad v_{\min} \leq v_i \leq v_{\text{limit},i}$$

with tighter lower bounds at overnight stops ($E_b[i_{\text{night}}] \geq E_b^{\text{night}}$) and finish ($E_b[N-1] \geq E_b^{\text{finish}}$).

$Q_i$ is the overnight solar charge added at segment $i$ (non-zero only at the first segment of each new race day). Each segment's energy increment expands in layers:

$$\Delta E_i = \Delta E_i^{\text{ss}} - \Delta E_i^{\text{kin}}$$

Expanding the steady-state and kinetic terms:

$$\Delta E_i^{\text{ss}} = (P_{s,i} - P_{b,i})\,\frac{d_i}{3600\,v_i} \cdot \begin{cases} \sqrt{\eta_b} & P_{s,i} \geq P_{b,i} \;\text{(net charging)} \\ 1/\sqrt{\eta_b} & P_{s,i} < P_{b,i} \;\text{(net discharging)} \end{cases}$$

$$\Delta E_i^{\text{kin}} = \Delta KE_i \cdot \text{eff}_{\text{kin}}, \quad \Delta KE_i = \frac{m(v_i^2 - v_{i-1}^2)}{7200}$$

Expanding the power terms:

$$P_{s,i} = GHI_i \cdot A_i \cdot \eta_s \qquad P_{b,i} = \begin{cases} P_{m,i} / \eta_m & P_{m,i} \geq 0 \\ P_{m,i} \cdot \eta_{\text{regen}} & P_{m,i} < 0 \end{cases}$$

Expanding mechanical power and headwind:

$$P_{m,i} = v_i \!\left(\tfrac{1}{2}\rho\;CdA\;(v_i - v_{w,i})^2 + C_{rr}mg + mg\cdot\text{grade}_i\right) \qquad v_{w,i} = w_i \cos(\theta_{w,i} - \theta_{\text{road},i})$$

### 1.2 Term breakdown

**1. Steady-state term** $\Delta E_i^{\text{ss}}$ — net power (solar minus drivetrain draw) integrated over segment time $t_i = d_i/(3600\,v_i)$, scaled by the battery's one-way efficiency. The battery loses energy on both directions: $\sqrt{\eta_b}$ when charging (less energy stored than produced) and $1/\sqrt{\eta_b}$ when discharging (less energy delivered than drawn).

**2. Solar power** $P_{s,i}$ is GHI times panel area times the combined panel+MPPT efficiency — a flat-panel model that assumes the array is always normal to the sky hemisphere:

$$P_{s,i} = GHI_i \cdot A_i \cdot \eta_s$$

**3. Battery power** $P_{b,i}$ splits by direction because the motor and drivetrain have different efficiency paths motoring vs. regenerating:

$$P_{b,i} = \begin{cases} P_{m,i} / \eta_m & P_{m,i} \geq 0 \;\text{(motoring — motor draws more than shaft delivers)} \\ P_{m,i} \cdot \eta_{\text{regen}} & P_{m,i} < 0 \;\text{(regen — battery receives less than shaft produces)} \end{cases}$$

**4. Mechanical power** $P_{m,i}$ is the sum of three forces times speed. Drag grows as $(v - v_w)^2$ — cubic in speed, the dominant term at cruise. Rolling resistance is constant. Grade is linear in $\sin\theta$ and flips sign on descents, making $P_{m,i}$ negative when the car is being pushed faster than drag and rolling resistance alone would allow (i.e. regen opportunity):

$$P_{m,i} = v_i \left(\underbrace{\tfrac{1}{2}\rho\; CdA\;(v_i - v_{w,i})^2}_{\text{drag}} + \underbrace{C_{rr}\,m\,g}_{\text{rolling}} + \underbrace{m\,g\cdot\text{grade}_i}_{\text{grade}}\right)$$

**5. Headwind component** $v_{w,i}$ projects the wind vector onto the road axis. A pure tailwind reduces effective airspeed; a pure crosswind contributes nothing (though it increases yaw angle and therefore $CdA$ — not yet modelled):

$$v_{w,i} = w_i \cos(\theta_{w,i} - \theta_{\text{road},i})$$

Finally, the battery itself loses energy on both charge and discharge. Modelling round-trip efficiency as $\eta_b$ means each direction costs $\sqrt{\eta_b}$: the $\sqrt{\eta_b}$ factor in **1** applies when net power is positive (charging); it flips to $1/\sqrt{\eta_b}$ when net power is negative (discharging).

**6. Kinetic term** — energy cost of changing speed between segments, smoothly blended across the regen/motor boundary:

$$\Delta E_i^{\text{kin}} = -\underbrace{\frac{m(v_i^2 - v_{i-1}^2)}{7200}}_{\Delta KE_i\;\text{(Wh)}} \cdot \underbrace{\left[\alpha_i \cdot \frac{1}{\eta_m \sqrt{\eta_b}} + (1-\alpha_i)\cdot \eta_{\text{regen}}\sqrt{\eta_b}\right]}_{\text{eff}_\text{kin}}, \qquad \alpha_i = \frac{1}{2}\!\left(1 + \tanh\frac{\Delta KE_i}{0.1}\right)$$

$\alpha_i = 1$ (full motor draw) when speeding up, $\alpha_i = 0$ (full regen recovery) when slowing down. The 0.1 Wh half-width keeps the function C² through $\Delta KE \approx 0$.

The same tanh blending is applied to the regen/motor boundary in $P_{b,i}$ and to the charge/discharge boundary in $\Delta E_i^{\text{ss}}$, replacing hard `if-else` branches with smooth approximations (half-width 5 W). This is required for IPOPT's exact Hessian to remain well-conditioned at switching points.

The overnight charges $Q_i$ are non-zero only at the first segment after each overnight stop (charging windows 18:00–20:00 and 07:00–09:00). The full constraint set also includes per-stop overnight floors and checkpoint time windows — see §6.

### 1.3 Parameters

| Symbol | Code name | Description |
|---|---|---|
| $\rho$ | `rho` | Air density (kg/m³) |
| $CdA$ | `CdA_flat` | Drag area (m²) |
| $C_{rr}$ | `Crr` | Rolling resistance coefficient |
| $m$ | `m` | Vehicle + driver mass (kg) |
| $g$ | — | 9.81 m/s² |
| $\eta_m$ | `eta_m` | Motor efficiency |
| $\eta_{\text{regen}}$ | `eta_regen` | Regenerative braking efficiency |
| $\eta_s$ | `eta_s` | Solar panel + MPPT efficiency |
| $\eta_b$ | `eta_b` | Battery round-trip efficiency ($\sqrt{\eta_b}$ per direction) |
| $A_i$ | `Ai` | Solar panel area (m²) |
| $E_b^{\max}$, $E_b^{\min}$ | `Eb_max`, `Eb_min` | Battery energy limits (Wh) |
| $E_b^{\text{start}}$ | `Eb_start` | Starting battery energy (Wh) |
| $E_b^{\text{finish}}$ | `Eb_finish_min` | Finish SoC requirement (Wh) |
| $E_b^{\text{night}}$ | `Eb_overnight_min` | Overnight SoC floor (Wh) |
| $v_{\min}$, $v_{\max}$ | `v_min`, `v_max` | Global speed bounds (m/s) |
| $d_i$ | `distance_m` | Segment length (m) |
| $\text{grade}_i$ | `grade` | Road grade, sin(θ) |
| $GHI_i$ | `GHI` | Global horizontal irradiance (W/m²) |
| $w_i$, $\theta_{w,i}$ | `wind_speed`, `wind_dir` | Wind speed (m/s) and meteorological direction (°) |
| $\theta_{\text{road},i}$ | `heading_deg` | Road heading (°) |
| $v_{\text{limit},i}$ | `speed_limit_ms` | Posted speed limit (m/s) |
| $v_{\text{disch},i}$ | — | Max speed from battery discharge current limit |

---

## 2. Literature Motivation

### 2.1 Why a simulation-based optimizer, not an analytical solution

The Pudney & Howlett line (University of South Australia, 1990s–2000s) proves closed-form results about optimal driving modes: constant speed is optimal on flat roads with a simple battery, and a "critical speed" strategy (two alternating speeds) is optimal on undulating roads once battery nonlinearity is accounted for. These results are elegant and exactly correct within their assumptions.

We didn't follow this line because our model violates its key assumptions:
- **Grade sensitivity:** the motor is highly sensitive to slope (flagged in 1994 SSCP notes; confirmed by the WaveSculptor datasheet). Pudney's proofs assume grade effects are separable. In practice, a steep climb at v* is very different from flat driving at v*.
- **Weather variability:** real-race solar input **changes hour-to-hour.** Pudney's steady-state model fixes GHI; ours varies it per segment.
- **Crosswind drag:** `CdA(β)` varies with yaw angle, which changes continuously along a real route. Not representable in the analytical framework.

The Michigan (Betancur/Yesil) line takes the alternative approach: build a detailed simulation and run a numerical optimizer over it. Michigan ran multiple simulations in parallel with different weather inputs mid-race and used the spread to judge forecast uncertainty. This is the practical approach for real racing — re-runnable in minutes when conditions change, and naturally handles all the nonlinearities.

**Our choice:** simulate each segment with the full drivetrain equation (Betancur, research_notes.md §2.3), then minimize total time with IPOPT (via CasADi), seeded by the Pudney critical speed v\*. Same philosophy as Michigan; we use an interior-point NLP solver instead of their heuristic search.

### 2.2 How the optimizer works

The optimizer is built from three pieces: a problem formulation (shooting), a solver (IPOPT), and a symbolic framework (CasADi). They're separable concerns — it helps to understand each one before seeing why they fit together.

#### The shooting formulation

The most natural way to write the optimization problem is: choose speeds v[0..N-1], define battery state as the running sum $E_b[i] = E_b^\text{start} + \sum_{j \leq i} \Delta E_j$, and add inequality constraints $E_b[i] \geq E_b^\text{min}$ for all i. Call this the *direct* formulation.

The problem with it: $E_b[j]$ is a function of *every* speed v[0] through v[j]. So the constraint Jacobian (the matrix of partial derivatives $\partial E_b[j] / \partial v[i]$) has a nonzero entry in every position where $i \leq j$ — a full lower-triangular N×N matrix. Every inequality constraint is coupled to every upstream speed decision. Interior-point solvers handle this poorly because they need to factor a dense KKT system at each step, and the barrier penalties from 499 simultaneous inequality constraints create a stiff, poorly conditioned landscape.

The *shooting* formulation avoids this by making $E_b[i]$ an explicit decision variable and writing the energy equation as a *local* equality constraint:

$$E_b[i] = E_b[i-1] + \Delta E_i + Q_i$$

Now the constraint Jacobian is bidiagonal: each row touches only four variables ($v[i]$, $v[i-1]$, $E_b[i]$, $E_b[i-1]$). The SoC bounds become simple variable bounds on $E_b[i]$, which the solver handles separately and efficiently. The problem has 2N decision variables (v and Eb) and N equality constraints, but the structure is clean enough that each Newton step is fast.

The name "shooting" comes from optimal control: you're "shooting" forward through the dynamics (integrating $E_b[i+1] = f(E_b[i], v[i])$) while the optimizer adjusts both states and inputs simultaneously.

#### IPOPT — interior-point optimization

IPOPT (Interior Point OPTimizer) solves NLPs of the form: minimize $f(x)$ subject to $g(x) = 0$ (equality constraints) and $l \leq x \leq u$ (variable bounds). It's the standard open-source solver for smooth medium-scale NLPs (tens to hundreds of thousands of variables).

The algorithm: IPOPT converts the variable bounds into a *barrier function* — instead of hard constraints $x \geq l$, it adds $-\mu \sum_i \log(x_i - l_i)$ to the objective. For large $\mu$ this barrier is steep and pushes solutions away from the boundary; as $\mu \to 0$ the barrier vanishes and the solution converges to the true constrained optimum. IPOPT solves a sequence of these barrier subproblems with decreasing $\mu$.

Each barrier subproblem is an equality-constrained NLP (objective + barrier function, equality constraints from $g(x) = 0$). IPOPT solves it with a **Newton step** on the first-order KKT conditions:

$$\begin{pmatrix} H_L & A^T \\ A & 0 \end{pmatrix} \begin{pmatrix} \Delta x \\ \Delta \lambda \end{pmatrix} = -\begin{pmatrix} \nabla_x L \\ g(x) \end{pmatrix}$$

where $H_L$ is the Hessian of the Lagrangian (objective + constraint penalties), $A$ is the constraint Jacobian, and $\lambda$ are the Lagrange multipliers. This linear system is solved by MUMPS (a sparse direct solver). For our problem the system is ~1000×1000 with bidiagonal structure — MUMPS factors it in O(N) time and each Newton step completes in milliseconds.

With the shooting formulation, IPOPT converges in ~50–200 Newton steps. Total solve time per outer iteration: ~0.3–1 second.

#### CasADi — symbolic differentiation and compilation

CasADi is a framework for symbolic computation on NLPs. You write the objective and constraints as symbolic expressions (using `ca.MX.sym` variables), and CasADi automatically derives:
- the objective gradient $\nabla f$
- the constraint Jacobian $A$
- the Lagrangian Hessian $H_L$

using reverse-mode automatic differentiation (not finite differences — exact to machine precision). These are what IPOPT needs at every iteration, and they're the reason no hand-coded Jacobians are required.

CasADi then generates C code for the NLP, JIT-compiles it once, and links it to IPOPT. The compiled function is ~10× faster to evaluate than interpreted Python. The key optimization: **parametric NLP**. Weather data (GHI, wind) and overnight charges are declared as *parameters* rather than constants baked into the graph. The solver is compiled once on the first call and reused across all 5 outer boundary-update iterations — only the parameter values and variable bounds (overnight SoC floors) change between calls. Compilation takes ~2 seconds; subsequent solves take ~0.3 seconds each.

#### The outer boundary-update loop

There is a chicken-and-egg problem: overnight stop locations (where the car is at 18:00) depend on the speed profile, but the speed profile depends on the overnight stop locations (which segments get the overnight charge, and which Eb variables get tighter lower bounds). Neither can be computed without the other.

We break the cycle with a fixed-point iteration:

1. Start with a rough speed estimate (22 m/s).
2. Simulate arrival times → find where 18:00 occurs on the route → identify day boundaries.
3. Run IPOPT with those boundaries, warm-started from the previous solution.
4. Use the optimized speed profile as the new estimate. Go to 2.
5. Stop when boundaries don't change (typically 2–4 outer iterations).

The inner IPOPT solve (step 3) is the expensive part, but thanks to the compiled parametric solver and warm-starting, each outer iteration takes ~0.5 seconds. Total wall time for the full run: ~3 seconds on a 2600 km route with 499 segments.

### 2.3 Why these specific model terms

| Term | Literature source | Rationale |
|---|---|---|
| Drivetrain: v³ drag + linear rolling | Betancur Eq. 1; confirmed by Pudney | Dominant power consumption terms. Rolling resistance linear; drag cubic — small speed increases are expensive. |
| `Ps = Ii·Ai·ηs·sin(φ)` | Betancur solar sub-model | Standard flat-panel model. GHI from Solcast replaces the irradiance model Betancur uses internally. |
| `ηb` round-trip efficiency | Betancur | Simple and well-validated; battery quadratic loss is small relative to drivetrain at cruise. |
| `I(b)` quadratic battery curve | Pudney | Captures voltage sag near discharge limit; relevant near Eb_min where the optimizer presses hardest. |
| Per-segment speed vector | Michigan / Betancur / Pudney all use it | Flexible: captures speed limit changes, grade variation, overnight stops at segment boundaries. |
| Overnight SoC constraint | Novel application | Not explicit in any paper (single-day races); natural extension — battery floor at end of each race day. |

### 2.4 What the literature does not cover (gaps we have to fill)

- **Multi-day race structure** — all reviewed papers model single-day or continuous races. ASC's 9 am–6 pm tour-day structure with overnight charging is an SSCP addition.
- **Live telemetry integration** — Michigan describes the workflow (chase car strategists re-running simulations) but not the software architecture. Our UDP-based live input loop is original.

---

## 3. Model Selection

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

## 4. Existing Strategy Code

### 4.1 Battery SoC — yes, but primitive

- [`motherboard/src/bms/state.rs`](sunstruck_onboard_rust_impl/motherboard/src/bms/state.rs): SoC is a simple linear interpolation of minimum cell voltage (2.5 V → 0%, 4.2 V → 100%). No coulomb counting, no model-based estimation.
- [`motherboard/src/bms/sensors/current.rs`](sunstruck_onboard_rust_impl/motherboard/src/bms/sensors/current.rs): Energy (Wh) and charge (Ah) are integrated at 100 Hz from a current shunt. These accumulators exist and are broadcast over telemetry — this is coulomb counting data, it's just not being fed back into the SoC estimate.

### 4.2 Motor power / speed control — yes, but no grade sensitivity or efficiency map

- [`motherboard/src/vehicle/drive.rs`](sunstruck_onboard_rust_impl/motherboard/src/vehicle/drive.rs): Generates motor drive commands. Three cruise modes: constant RPM, constant bus current, off. Currently no PID — cruise is constant-speed + 100% torque. **Known open issue.**
- Motor messages (`messages/src/motor/mod.rs`) use Tritium WaveSculptor protocol: velocity setpoint in RPM, current as a fraction 0.0–1.0. No efficiency map or grade-aware power estimation anywhere in the codebase.

### 4.3 Solar array telemetry — logged, not modeled

- MPPT messages (`messages/src/mppt/mod.rs`): 6 channels (3 units × 2 ch), each reports array voltage, array current, battery voltage, and temperature. All data is broadcast over CAN and logged. No irradiance model, no power forecasting.

### 4.4 GPS + elevation — tool exists, not integrated

- [`offboard/elevationProfiler/ElevationProfiler.go`](sunstruck-code/offboard/elevationProfiler/ElevationProfiler.go): A standalone Go tool that loads SRTM3 topographic tiles and processes a CSV route to produce an elevation profile. A KML→CSV converter is also included. This is exactly the Michigan pre-race route survey workflow — but the output is not connected to anything in the vehicle firmware or any optimizer.
- Live GPS (NEO-F9P / ZED-F9P with RTK corrections from EarthScope P221) is implemented in firmware and outputs `GpsPosition`, `GpsStatus`, `GpsTime` messages. **Known issue: the GNSS receiver may have a hardware problem as of June 2026.**

### 4.5 Telemetry logging — robust

- [`motherboard/src/comms/mod.rs`](sunstruck_onboard_rust_impl/motherboard/src/comms/mod.rs): UDP broadcast every 100 ms to off-site proxy. All live data (voltage, current, MPPT, GPS, SoC, Wh, Ah, motor commands) is on the wire.
- [`motherboard/src/sd.rs`](sunstruck_onboard_rust_impl/motherboard/src/sd.rs): SD card circular log at 100 ms cadence. 59 CAN message slots + per-cell voltage + per-cell temperature arrays per record. Capacity ~18 days at 100 ms intervals. This is the raw material for any strategy model.
- [`offboard/telem/`](sunstruck-code/offboard/telem/): Python tools to receive telemetry UDP, write to InfluxDB + CSV, and replay from log files.

### 4.6 Max's model — prior SSCP optimizer (last race)

MATLAB-based car model used at the most recent race. Direct predecessor to `python/optimize.py`. Find it before re-deriving physics from scratch.

---

## 5. Strategy Gap Summary

### 5.1 Pre-race / offline inputs

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

### 5.2 Live inputs (chase car, updated continuously during race)

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


### 5.3 Key telemetry wire units

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

### 5.4 Vehicle parameter status

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

## 6. Optimizer Constraints

The optimizer minimizes `Σ d_i / v_i` (total race time) subject to these constraints.

| Constraint | Equation | Implementation | Notes |
|---|---|---|---|
| Battery never runs out | `Eb_i ≥ Eb_min` for all i | ✅ Active — 250 Wh floor | Variable lower bound on Eb[i]; IPOPT handles natively |
| Battery never overcharges | `Eb_i ≤ Eb_max` for all i | ✅ Active — 5000 Wh ceiling | Variable upper bound on Eb[i] |
| Finish with buffer | `Eb[N-1] ≥ Eb_finish_min` | ✅ Active — 250 Wh | Tighter lower bound on Eb[N-1] |
| Speed limits (upper) | `v_i ≤ vlimit_i` for all i | ✅ Active — 65 mph / posted limit per segment | Speed limits read from GPX where tagged; default 65 mph elsewhere |
| Minimum speed | `v_i ≥ vmin` | ✅ Active — 20 mph (8.94 m/s) | ASC 2026 §12.22 |
| Discharge rate limit | `Pb_i / V_pack ≤ I_discharge_limit` | ✅ Active — 150 A / 100 V nominal | Encoded in per-segment speed upper bound via `max_speed_from_discharge_limit()` |
| Per-checkpoint SoC | `Eb[cp_k] ≥ Eb_checkpoint_min` | ⚙️ Framework built, no checkpoints loaded | Awaiting Route Book for locations and times |
| Checkpoint time windows | `t_open_k ≤ Σt_i ≤ t_close_k` at checkpoint k | ⚙️ Framework built, no checkpoints loaded | Awaiting Route Book |
| Daily overnight SoC | `Eb[i_night_k] ≥ Eb_overnight_min` | ✅ Active — 500 Wh floor per overnight stop | Tighter lower bound on the overnight-stop segment's Eb variable |
| Regen ceiling clip on descents | `Eb_i ≤ Eb_max` when `Pm_i < 0` | ✅ Enforced — ceiling is a hard variable bound | Eb_max is a variable upper bound on every Eb[i]; overcharge on regen descents is prevented |

### 6.1 Race rules — ASC 2026 (confirmed from Regs Rev C, May 2026)

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

## 7. Known Optimizer Limitations

### 7.1 Non-convex NLP — morning sprint problem

On days where the battery starts full (or near-full), the true optimal strategy is to **sprint hard in the morning** to drain below Eb_max quickly, freeing headroom to capture solar through midday, then sprint again in the late afternoon to drain to the overnight floor. This produces a "W"-shaped battery curve.

IPOPT (like any local NLP solver) may not find this because the problem is **non-convex**: when net power is positive (solar exceeds load), $\Delta E_i$ is a concave function of $v_i$, making the SoC constraint $E_b[i] \geq E_b^{\min}$ non-convex. Multiple local optima can exist — in particular, "drive steady at the constraint boundary all day" and "sprint in the morning, coast in the afternoon" can both be KKT points.

The shooting formulation + exact Hessian makes IPOPT find the KKT point efficiently, but the specific local optimum found depends on the initial seed. With a v\* seed (near-constant speed), IPOPT converges to the near-constant-speed solution.

**Mitigation:** warm-starting from the previous outer iteration's solution helps maintain the solution basin across boundary updates. A targeted morning-sprint seed (v_max for segments where projected Eb would hit the ceiling, v\* elsewhere) would test whether a better local optimum exists on the real route.

**Fixes (in order of effort):**
1. **Morning sprint seed** — v_max for segments where the v\* trajectory would pin Eb at the ceiling, v\* for the rest. Tests whether IPOPT finds a better local optimum from a sprint-shaped starting point.
2. **Global method (DP)** — guarantees the global optimum within discretization; models battery clipping exactly. Revisit if the sprint seed finds meaningfully better solutions.

**Priority:** V1 on a real route — on the flat synthetic route the battery doesn't pin at the ceiling at race speeds, so this hasn't been an active issue in testing.

---

## 8. Open Questions / To Investigate

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
