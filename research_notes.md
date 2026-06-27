# Solar Car Race Strategy — Research Notes

## 1. Background
Good strategy begins with understanding key parameters and limits of the vehicle: I–V characterization of solar power generation, battery capacity, motor power, vehicle mass.
Utilize benchmarks/records from the past (or test/derive your own) to identify these. For example, the 2012–2013 SSCP team expected:
- Typical array output: 1–1.3 kW at solar noon; may drop below 80 W when overcast
- Power consumption: 1.1–1.5 kW at target speed 90 kph (56 mph)

**Most basic model:** considers live irradiance (and perhaps battery charge) to set a speed. These kinds of models probably already exist in parts of SSCP's codebase.
**More complicated models** use further meteorological data (humidity, temperature, crosswind, cloud cover) to more precisely analyze impacts on car performance:
- Forecasts can be utilized for predictive race modeling and live-updating suggestions
- Not trivial to incorporate crosswind simulation data, thermal performance curves, etc.
- Elevation / road gradient
- Road conditions (e.g., Australian bitumen), expected slowing/stopping along route
- Previous race performance: is the car moving better or worse than expected?
- Route, shading conditions

---

## 2. Conventional Wisdom
Two common strategies:
1. Match a constant, optimal speed
2. Match a desired battery SoC

SoC should matter for long-term race strategy; maintaining a constant speed is a short-term goal given losses due to acceleration/deceleration and the cubic relationship between power and drag.

Power production and power consumption are the two sides of the equation. Over the course of the race, we want to consume all of the power produced to the maximum.

---

## 3. Notes from '94
- It is also a task to determine good drag values to use
- Battery performance does vary as well
- Motors are very sensitive to % grade; there is some algorithm in telemetry that evaluates power consumption

---

## 4. Strategy Considerations
- **Cloud cover approach:** speed up when there is some cloud cover to reach a sunny area. A judgement call — depends how wide the cloud cover is and the cloud/car relative velocity. Maybe too complex to put in a model.
- **Hill strategies:** accelerate uphill quickly? Needs more research into motor heat/torque/velocity.
- **Regen vs. frictional braking:** there may be select racing scenarios only where regen makes sense.
- Suggestion of a bracketed race analysis providing best- and worst-case scenarios.

---

## 5. Luminos Strategy Notes

From the 2012–2013 SSCP wiki pages on Luminos (Stanford's prior competitive solar car):

**Optimization approach:** Constrained optimization of speed using a ~3000-dimensional velocity vector (one value per race segment), with battery SoC > 0 as the constraint. This is exactly the architecture now implemented in `strategy/optimize.py`.

**SoC modeling:** Luminos used a complementary filter to identify battery SoC — likely a blend of voltage-based estimation and Ah integration. The wiki notes this is "probably important down the line, or has been done already in telemetry." SSCP's current firmware integrates Ah at 100 Hz (coulomb counting) but doesn't feed it back into the SoC estimate (still linear in min cell voltage).

**Strategy planning insight:** Constant speed is not optimal. Losses vary at different velocities and conditions; the road is not perfectly flat; the motor is highly sensitive to grade. The Luminos wiki pointed to the Pudney (Adelaide) and Betancur papers as the references to explore — both are now in §7.

**Competitive positioning:** SSCP should plan to "race our own race" — don't expect to be competitive with WSC-class vehicles running 6 m² panels. The strategy model and optimization are still valuable for maximizing our own performance within our class.

---

## 6. Literature Synthesis: The Two Schools of Thought
All solar car strategy research falls into two camps:

**1. Mathematical / Optimal Control (Pudney, Howlett line)**
- Closed-form proofs about optimal driving modes
- Key finding progression: speedholding (flat road, simple battery) → modified two-speed holding (undulating road) → critical speed strategy (realistic battery model)
- Elegant and exact, but requires simplifying assumptions; breaks down with complex nonlinearities

**2. Heuristic / Evolutionary (Betancur, Yesil, Michigan line)**
- Build a detailed simulation model (drivetrain + solar panel + battery + climate)
- Run a search algorithm (GA, BB-BC, exhaustive) to find the optimal velocity vector
- Computationally heavier but handles nonlinearities, weather variability, grade sensitivity naturally
- Michigan's approach: run multiple heuristic simulations simultaneously with different weather inputs; trust the one that seems closest to reality

**Practical recommendation from the literature:** for actual race use, heuristic simulation is preferred because you can re-run it in <3 min mid-race when conditions change.

---

## 6. Physics Reference: Drivetrain Power Equation

From Betancur et al. Equation (1) — the fundamental equation governing what speed you can maintain:
```
Pm = v * [m*a + (1/2)*CdA*ρ*(v - vw)^2 + Crr*m*g + m*g*sin(θ)]
```
At constant speed on flat road with no wind (a=0, θ=0, vw=0), this simplifies to:
```
Pm = v * [(1/2)*CdA*v^2 + Crr*m*g]
   = (1/2)*CdA*ρ*v^3 + Crr*m*g*v
```
The v³ aerodynamic drag term is why power consumption is so nonlinear with speed, and why small speed reductions can save significant power on a flat road. Rolling resistance is the other term (linear in v). Both matter for SSCP's power sensitivity analysis.

---

## 7. Paper Notes

### 7.1 "Follow the Sun: GNSS and the Great Solar Car Race" — InsideGNSS, Sep/Oct 2008

**Full text available:** https://insidegnss.com/auto/sepoct08-solarcar.pdf

**Subject:** University of Michigan's winning strategy at the North American Solar Challenge 2008 (2,400 mi, Plano TX → Calgary AB). Car: Continuum (9th Michigan solar car). Average speed ~46 mph over 51h 41m. Beat 2nd place (Principia College Ra 7) by 10 hours.

**Vehicle specs mentioned:**
- 650 lbs total weight
- 2,700 triple-junction gallium-arsenide solar cells (same type as satellite solar panels)
- Lithium-polymer battery pack
- Three-wheel layout; front wheel steers and drives
- Hall Effect sensor in motor for speed measurement
- Regenerative braking + custom disk brakes
- Cruise control command that could be sent to driver from chase car; driver accepts/rejects via steering wheel button

**Pre-race GPS route survey:**
- ~2 months before the race, team drove the full route at 10 Hz sampling rate
- Used two donated 20-channel L1 survey GPS receivers + commercial satellite-based DGPS service (OmniSTAR HP, spec'd at 10cm real-time accuracy)
- Recorded: lat, lon, GPS signal quality, heading
- Marked locations of hills, stop signs, speed limit changes
- Data post-processed with custom team-written software
- Precision mattered more than absolute accuracy: "If everything is off by one meter in the same direction, it doesn't matter. We care about the differences in elevation."
- GPS receiver output over RS-232 to laptop; not integrated with recording computer

**Simulation during the race:**
- Three strategists rode in the chase car running real-time simulations continuously throughout the race
- Simulation variables: radiation levels, crosswinds, road grade (from pre-surveyed map), battery SoC, vehicle weight
- Multiple simulations run simultaneously with *different weather patterns* as inputs
- Head strategist Alex Dowling used **two different simulation/optimization programs** with different algorithms and different runtimes; he used knowledge of each program's strengths and shortcomings to judge which was closer to reality — a real-world hedge against model uncertainty

**Cloud strategy — validated and decisive:**
- Michigan drove *fast* under cloud cover to reach sunny areas sooner
- Other teams drove slowly under clouds and remained under them longer
- This was likely a significant differentiator in a 10-hour winning margin

**Support vehicle structure:**
- **Weather:** meteorology student ~30 min ahead transmitting forecasts
- **Scout:** ~10 miles ahead with two relief drivers; cleared road of debris
- **Lead:** directly in front, carried support engineers
- **Chase:** directly behind, carried three strategists + race manager Jeff Ferman + crew chief Doug Lambert

**Telemetry:**
- Chase and Lead shared the same live telemetry: speed, voltages, currents, battery pack reading
- "Any power consumptions that we can read, we do read"
- Driver communicated via radio and yes/no steering wheel buttons

**Tools:** OmniSTAR VBS DGPS (1m accuracy during race), Microsoft MapPoint for mapping, simulations written in MATLAB and Microsoft Visual Studio

**Relevance for SSCP:** Pre-surveyed elevation profile is a must-have. The two-simulator hedge is a practical idea. The cloud speed-up heuristic is well-validated and simple to implement. Live telemetry sharing between all support vehicles is worth replicating.

---

### 7.2 "Winning Solar Races with Interface Design" — Hilliard & Jamieson, *Ergonomics in Design*, 2008

**Full text:** https://journals.sagepub.com/doi/pdf/10.1518/106480407X312374 (paywalled; access via Stanford Library). DOI: 10.1518/106480407X312374

**Authors:** Antony Hilliard (M.A.Sc. industrial engineering, U of Toronto) and Greg A. Jamieson (associate professor, mechanical and industrial engineering, U of Toronto). Grew out of an earlier 2007 IEEE Systems, Man and Cybernetics conference paper: "Ecological Interface Design for Solar Car Strategy: From State Equations to Visual Relations."

**Subject:** Design and prototype of an integrated race strategy planning and monitoring interface for the University of Toronto BlueSky Solar Car Racing team, developed over three months using Ecological Interface Design (EID) methodology. BlueSky had been racing since 1997; paper published after their 10th year.

**Context:** BlueSky's current (2008) car used gallium arsenide solar cells, lithium-ion batteries, high-efficiency motors. Strategy team rode in a chase van monitoring telemetry, weather reports, racecourse maps, and hand-built physics models. Problem: the environment is highly coupled and unpredictable — exactly when and where a developing storm will cross the racecourse cannot be planned far in advance.

---

#### 7.2.1 Analysis Framework

Work Domain Analysis (WDA) divided the system into three areas:

| Area | Functional purpose |
|---|---|
| Car | Complete race safely in minimum possible time |
| Natural Environment | Conservation of energy and mass, entropy generation |
| Race Environment | Provide competition structure, ensure safety of persons on roadway |

Key insight from the WDA: physical laws and legal constraints **cannot** be redesigned; team roles, training methods, and computer systems can. Design should make invisible constraints visible.

The analysis also flagged data gaps — they recommended adding **3-axis accelerometers and suspension travel gauges** to the solar car telemetry (not present at the time) to support energy and safety reasoning.

---

#### 7.2.2 Four Display Views

**1. Maintenance view** — voltages, currents, temperatures at generalized/physical function level.

**2. Energy view — Motor efficiency map**
BlueSky's motor had an adjustable rotor-stator air gap (manually set by driver on the fly to optimize electrical efficiency). Old support tool: a set of graphs, one per gap setting, requiring effortful interpolation. New design: motor efficiency plotted as **contours on axes of gap × motor power × car speed** (two linked 2D plots). The current operating point is marked on both and connected by a line (display proximity principle). Higher-efficiency gap settings appear as lighter contours above/below the current point — perceptually direct rather than cognitively inferred. The display also makes visible the effect of wind or road grade changes on motor efficiency before they happen.

**3. Safety view — Car-handling map**
Safe handling envelope plotted on axes of **speed × inverse corner radius**. Envelope shrinks in wet weather or at high speed (aerodynamic effects). Upcoming corners from the pre-surveyed route are marked on the plot; if current speed exceeds safe cornering speed for an upcoming bend, high-salience colors flag the danger. Enables supervisory control of the driver from the chase van.

**4. Navigation view — Time-distance space** *(most important)*
The key design insight of the paper. Racecourse distance on horizontal axis (fixed, one-dimensional). Time of day on vertical axis below. The planned **velocity profile** (horizontal) is integrated to produce a line through the time-distance space — a visual trajectory of where the car will be at what time. **Cloud cover forecasts are mapped onto the same time-distance space** as shaded regions. The overlap between the car's trajectory and cloud patches is immediately visible, and the effect of small speed changes on cloud avoidance is directly perceptible without calculation. Figure 4 shows a scenario where slightly reducing speed at 10:30 would let the car slip between two cloud systems rather than being caught under the first one.

---

#### 7.2.3 Key Findings and Quotes

- "Exactly when and where a developing storm will cross the racecourse, and what actions the driver should take must be estimated and frequently reevaluated."
- Active contextual advice systems (even simple ones) have been shown to reduce fuel consumption in conventional vehicles by **14%**.
- The BlueSky team accepted the prototype and planned to implement it for the 2008 racing season.
- The time-distance space display is broadly applicable beyond solar racing — rail scheduling, military route planning, marine navigation in restricted waterways.

---

#### 7.2.4 Relevance for SSCP

The **time-distance space** is the most directly applicable concept. For our chase car display: route distance on x-axis, time of day on y-axis, planned speed profile integrated to show trajectory. Cloud cover (from a forecast API) mapped onto the same space. This makes the Michigan "speed up under clouds" heuristic into something the strategist can see directly rather than reason about abstractly.

The **motor efficiency map** is relevant if SSCP has measured efficiency as a function of operating point — worth checking whether the WaveSculptor controller or any test data produces a (torque × RPM → efficiency) surface.

The EID framework's core principle — make the physics directly visible so the strategist perceives constraint violations rather than inferring them — should guide any chase car dashboard design. The question is not just "what does the optimizer output" but "how does the strategist read it under stress in a moving van."

Key quote on representation aiding: *"Task performance can be improved by using interfaces that provide a faithful representation of relevant real-world constraints in a form compatible with human perceptual and cognitive abilities."* This is the design principle behind the time-distance space display.

---

### 7.3 "Heuristic Optimization for the Energy Management and Race Strategy of a Solar Car" — Betancur, Osorio-Gómez & Rivera, *Sustainability*, 2017

**Full text open-access:** https://www.mdpi.com/2071-1050/9/10/1576/pdf — also on ResearchGate: https://www.researchgate.net/publication/320049866_Heuristic_Optimization_for_the_Energy_Management_and_Race_Strategy_of_a_Solar_Car

**Subject:** Racing strategy for the EPM-EAFIT solar car at the World Solar Challenge 2015 (Darwin → Adelaide, 3022 km).

---

#### 7.3.1 The Race Model

Four coupled sub-models. This is the canonical architecture:

**Drivetrain (energy consumption)**

Instantaneous wheel power:
```
Pm = v * [m*a + (1/2)*CdA*ρ*(v - vw)^2 + Crr*m*g + m*g*sin(θ)]
```
Where:
- `v` = instantaneous velocity
- `m` = vehicle mass
- `a` = acceleration
- `CdA` = drag area coefficient
- `ρ` = air density
- `vw` = wind velocity component in forward direction
- `Crr` = tyre roll coefficient
- `g` = gravity
- `θ` = road slope

For constant-slope, constant-speed sections, consumed energy per segment:
```
Ei = Pm * ti / ηm
```
Where `ηm` is drivetrain efficiency under those conditions.

**Solar Panel (energy input)**
```
Ps = Ii * Ai * ηs * sin(φ)
```
Where:
- `Ii` = solar irradiance at ground level
- `Ai` = panel effective area (accounting for canopy shadows)
- `ηs` = panel + MPPT efficiency (experimentally determined, includes forced convection cooling from vehicle motion)
- `φ` = sun elevation angle

**Battery**

Overall battery efficiency from charge/discharge tests:
```
ηb = Eout / Ein
```
Battery SoC dynamics:
```
dEb/dt = sqrt(ηb) * (Ps - Pm)    if (Ps - Pm) > 0   [charging]
       = (1/sqrt(ηb)) * (Ps - Pm) if (Ps - Pm) ≤ 0   [discharging]
```
(Charge and discharge efficiencies both assumed equal to sqrt(ηb))

**Climate**
- Solar irradiance calculated using Beer–Bouguer–Lambert law (atmospheric transmittance):
```
Ii = I0 * exp(-τa * AM)
AM = 1 / sin(φ)
```
Where `I0` = extraterrestrial solar radiation, `τa` = atmospheric extinction coefficient, `AM` = air mass factor. Cloudless sky assumed; validated experimentally.
- Wind: monthly averages from Australian Bureau of Meteorology. Stochasticity removed from both irradiance and wind for repeatability.

**Main optimization input:** velocity set point vector (one integer km/h value per race segment). Bounded by road speed limits.

---

#### 7.3.2 Optimization Methods Compared

**Exhaustive Search (ES)** — brute force; feasible only for 1D, 2D, 3D velocity vectors.

**Genetic Algorithms (GA)** — standard GA with:
1. Fitness evaluation (race simulation) for each candidate
2. Selection of best half
3. Crossover: random pairs → four offspring via linear combination
4. Mutation: ±10 km/h uniform random addition to 10% of population
5. 50 iterations; early convergence criterion

**Big Bang-Big Crunch (BB-BC)** — iteratively generates random individuals around a weighted center of mass; search radius shrinks each iteration.

**Algorithm Hybridization** — GA or BB-BC + Local Search (LS) post-processing: one-directional perturbations around the evolutionary result to check for small improvements nearby.

All run at 720 candidates, 50 iterations. One race simulation = 4–6 ms. Full optimization run = <3 min.

---

#### 7.3.3 Results

**Clear sky case:**

| Method | Vector Size | Race Time (h) | Compute Time (s) |
|---|---|---|---|
| Exhaustive Search (1D) | 1 | 38.189 | 0.28 |
| Exhaustive Search (2D) | 2 | 38.189 | 14.85 |
| Exhaustive Search (3D) | 3 | 38.077 | 926.46 |
| Genetic Algorithms | 10 | 38.081 | 137.29 |
| GA + Local Search | 10 | **38.068** | 145.16 |
| Big Bang-Big Crunch | 10 | 38.116 | 141.34 |
| BB-BC + Local Search | 10 | 38.088 | 150.65 |

**Cloudy day case** (one day at 60% irradiance):

| Method | Vector Size | Race Time (h) | Compute Time (s) |
|---|---|---|---|
| Exhaustive Search (1D) | 1 | 40.176 | 0.39 |
| Exhaustive Search (3D) | 3 | 39.792 | 1284.1 |
| Genetic Algorithms | 10 | **39.771** | 116.44 |
| GA + Local Search | 10 | 39.781 | 120.89 |
| Big Bang-Big Crunch | 10 | 39.851 | 197.96 |

**Key findings:**
- GA outperformed BB-BC in both convergence speed and final result quality
- Going from 1D → 10D velocity vector saved ~3% (~7 min) of race time
- 40% irradiance reduction for one full day = ~2h added race time (38.068h → 39.771h)
- The optimal 10 velocities for clear sky all fell between 78–83 km/h; cloudy day 75–84 km/h. In practice these are nearly constant speed — a manual driver would not perceive the difference.
- Optimal strategy depletes battery to near-empty at the finish line
- The method is fast enough to re-run mid-race: new optimal strategy in <3 min after a weather change or deviation from predicted performance

**Conclusions:** GA+LS is the recommended method for this problem size. Heuristic methods are necessary once search space exceeds ~3D (exhaustive becomes intractable). The race model and optimizer are independent modules — vehicle specs are just a parameter input.

---

### 7.4 Pudney & Howlett — Mathematical Optimization Line (University of South Australia)

**Paper progression:**

**Howlett, Pudney, Tarnopolskaya & Gates (1997)** — *"Optimal driving strategy for a solar car on a level road"*, IMA Journal of Mathematics Applied in Business and Industry, vol. 8, pp. 59–81.
- Used simplified battery model
- Proved: optimal strategy on a flat road is essentially **speed-holding**

**Howlett & Pudney (1998)** — *"An optimal driving strategy for a solar powered car on an undulating road"*, Dynamics of Continuous, Discrete and Impulsive Systems, vol. 4, pp. 553–567.
- Extended to undulating road
- Found: **modified speedholding with upper and lower holding speeds**

**Pudney, P.J. (2000)** — *"Optimal energy management strategies for solar-powered cars"*, Ph.D. Thesis, University of South Australia. https://searchlibrary.adelaide.edu.au/discovery/fulldisplay/alma9915959939701831/61USOUTHAUS_INST:ROR

**Pudney & Howlett (2002)** — *"Critical speed control of a solar car"*, Optimization and Engineering, vol. 3, pp. 97–107. https://link.springer.com/article/10.1023/A:1020907101234 (paywalled; access via Stanford Library)

**Boland, Gaitsgory, Howlett & Pudney (2001)** — *"Stochastic optimal control of a solar powered car"*, in Progress in Optimisation III (Kluwer).
- Extension to stochastic weather

---

#### 7.4.1 Pudney & Howlett (2002) — Detailed Notes

**Context:** Used to derive strategy for the *Aurora 101*, winner of the 1999 World Solar Challenge (Darwin → Adelaide, 3000 km). The Aurora team used the University of South Australia's Scheduling and Control Group for strategy. The daily solar radiation was estimated with a Markov model; the driving strategy described in this paper was used in the short-term.

**Problem statement:** Maximize distance traveled in a day, given known solar radiation and specified initial and final battery charge. (Not minimize time over fixed distance — the formulation is per-day distance maximization.)

**Vehicle dynamics:**

Force at wheels: `F = p/v`, where `p` is motor power, `v` is speed.

Resistive force: `R = R(v)` — modeled as a convex quadratic in `v`. For Aurora 101 specifically:
```
R(v) = r0 + r1*v + r2*v²
r0 = 12.936, r1 = 0.156, r2 = 0.066
```

Equations of motion:
```
dx/dt = v
dv/dt = (1/m) * [p/v - R(v)]
```

**Battery model (the key upgrade over prior papers):**

Earlier papers used a simplified battery model. This paper fits a **quadratic current-power model** to experimental silver-zinc cell data (Aurora used silver-zinc cells; tested by Aurora + CSIRO with repeated discharge cycles logged at 20s intervals):
```
I(b) = c1*b + c2*b²
```
Least-squares fit gave (single cell):
```
I(b) = 0.609*b + 0.00324*b²
```
For `n` cells in series:
```
I(b) = 0.609*(b/n) + 0.00324*(b/n)²
```
Capacity efficiency of each cell: ~97%. This model was validated in March 1997 when Aurora drove 250 km at 100 km/h on a no-sun day; the model predicted 253 km. It was also used for Aurora's 1999 WSC win.

Battery state equation:
```
dq/dt = (-1) * I(b)
```
with boundary conditions `q(0) = q0`, `q(T) = qT`.

**Hamiltonian and optimal control:**

Hamiltonian (after normalization):
```
H = v + η[s + b - φ(v)] - C*I(b)
```
where `φ(v) = v*R(v)` (convex), `η = π2/(mv)`, `C = π3/A`.

Maximizing H over `b` gives optimal battery power:
```
b* = (η - C*c1) / (2*C*c2)
```

**The critical speed:**

The optimal trajectory has a unique **saddle point** `(v*, η*)` where `dv/dt = 0` and `dη/dt = 0`. This saddle point defines the **critical speed** `v*`.

At the critical point:
```
s + b*(v*) = φ(v*)        [motor power = resistive power at v*]
η* = 1/φ'(v*)
```

Critical speed `v*` **increases with solar power** `s`. Figure 5 in the paper shows this relationship for Aurora: `v*` ranges from ~86 km/h at s=0 W to ~91 km/h at s=1500 W — a relatively narrow band.

**Why any long journey must stay near v*:**

Phase portrait analysis (Figure 6) shows the saddle point is unstable — trajectories either diverge to very high speed or collapse to v=0. For a journey lasting more than a few minutes, the trajectory must pass close to the saddle point for nearly the entire journey.

Practical implication (Section 8): any optimal race strategy has three phases:
1. **Power phase** (at most a few minutes): accelerate from 0 to near v*
2. **Hold phase** (almost the entire journey): travel at the critical speed v*, which drifts slowly as solar power s changes through the day
3. **Brake phase** (at most a few minutes): decelerate from v* to 0

Figure 7 confirms this: even a trajectory at 99.99% of the optimal Hamiltonian value converges to v* within ~220 seconds. Any race lasting hours must hold at v* for essentially all of it.

**Relationship to earlier speedholding strategies:**

The earlier papers (1997, 1998) found speedholding to be optimal under a simplified battery model. With the realistic quadratic battery model, the optimal strategy is *critical speed*, not simple speedholding. However, when solar power is constant, the two strategies are nearly identical — the critical speed is essentially the same as the holding speed.

**Core message:** The optimal strategy is always "travel at a speed determined by current solar power." The critical speed encodes the correct answer given the battery model and drag characteristics. If you know s (solar power) and have the vehicle parameters, you can compute v* analytically.

**Relevance for SSCP:**
- The critical speed formula gives a sanity check on any optimizer output: if s ≈ constant and the optimizer recommends wildly varying speeds, the model has a bug.
- The three-phase structure (power → hold → brake) means the interesting strategic question is just "what is v* right now, and how is it changing?" — everything else follows.
- Aurora's v* ranged only ~5 km/h over the full range of solar power (0–1500 W). This suggests the optimal speed is much less sensitive to irradiance than intuition suggests; the battery absorbs the variation.

**Fitting the battery model for SSCP:**

SSCP's model is `I(b, T, SoC)` — a richer version of Pudney's single curve, enabled by live telemetry:

*Sources:*
- **Live telemetry** (preferred primary source): pack voltage × pack current = `b`; pack current = `I`. Every steady-speed, approximately constant-grade moment gives a clean `(b, I, T, SoC)` data point. Hundreds of these accumulate in the first hour of driving. Better than cell cycler data because it reflects the whole pack under real driving conditions.
- **Cell cycler data** (`offboard/cellCycler/`): useful as a sanity check and for temperature characterization at controlled conditions. Secondary source.

*Fitting procedure:*
1. From telemetry, filter to steady-state segments (low acceleration, known grade from GPX, stable speed)
2. For each segment, compute `b = V_pack × I_pack` and record `(b, I, T, SoC)` where T is pack temperature and SoC is from the Ah accumulator
3. Bin by temperature (e.g. 5°C bins) and by SoC range (e.g. 20% bins)
4. Within each bin, fit `I = c1·b + c2·b²` via least squares
5. Result: a lookup table of `(c1, c2)` per `(T, SoC)` bin

*During the race:*
- Live layer: uses current T and SoC from telemetry to look up the right `(c1, c2)` — no model needed, just the table
- GA / optimizer: uses expected T and SoC trajectory per segment to look up coefficients, producing a more accurate simulation than a single static curve

*Error sources to watch:*
- **MCP3913 calibration** — stale coefficients on BMS 2–5 boards introduce systematic current measurement error that biases c1 directly. Fix this before fitting.
- **Cell aging** — re-fit from recent telemetry before each race, not from data collected months earlier
- **Pack imbalance** — visible in per-cell voltage spread on telemetry. Large spread means the weakest cell hits limits before the pack-level model predicts; the BMS discharge limit will drop unexpectedly. Monitor min/max cell voltage delta as a real-time imbalance indicator.

---

### 7.5 Supplementary: Other Referenced Works (from Betancur et al. reference list)

**Shimizu et al. (1998)** — Honda Dream solar car strategy (WSC 1990, 1993, 1996). Divided strategy into three topics: supervision support system, cruising simulation program, and power/speed optimizing control algorithm. First published strategy system for a competitive solar car team.

**Yesil et al. (2013)** — *"Strategy optimization of a solar car for a long-distance race using Big Bang-Big Crunch optimization"*, WSC 2013. https://www.researchgate.net/publication/235956111 First published heuristic application to solar car strategy. No comparison baseline or experimental validation. BB-BC later shown by Betancur to be worse than GA for this problem.

**Guerrero-Merino & Duarte-Mermoud (2016)** — *"Online energy management for a solar car using pseudospectral methods for optimal control"*, Optimal Control Applications and Methods. Applies pseudospectral methods (a numerical optimal control approach) with online power prediction. A hybrid between mathematical and numerical methods.

**Train optimal control literature (Chang & Sim 1997, and subsequent):** GA was first applied to train energy optimization in 1997. The train problem is nearly identical to solar cars: minimize energy (or time) between two points subject to speed limits and energy constraints. Two decades of results from the train field transfer directly to solar cars.

---

### 7.6 "Race Simulation and Energy Management System for a Solar Car" — ELECO 2025

**Full text available:** https://www.eleco.org.tr/ELECO2025/Eleco2025-Papers/174.pdf

**Subject:** Most recent paper found (2025). Builds a lap-based race simulation for a circuit solar car. Key contribution: uses a **surface-mounted PMSM motor model with iq-based torque control and a full efficiency-map power path** — the most detailed motor model in any paper reviewed here. Also includes a pack-side battery model with current and SoC update per time step.

**Validation:** Model predicted 57.278 Wh vs. 58.154 Wh measured on a representative lap (1.5% error). Across 11 laps: mean signed difference −1.19%, MAPE 2.81%, RMSE 1.95 Wh. Sufficient fidelity for pacing and pit planning.

**Motor efficiency map — the key contribution:**
The model replaces a constant ηm with a 2D lookup table: `(torque, RPM) → efficiency`. This is implemented as a PMSM efficiency map, the same motor type as SSCP's setup. At high torque / low speed (steep climbs), efficiency drops — this is what the '94 notes were observing when they said "motors are very sensitive to % grade." A constant ηm systematically underestimates power consumption on climbs.

**When this matters for SSCP:**
- On flat terrain at constant speed: motor operates near one point, constant ηm ≈ fine. WaveSculptor maintains ~97–98% across a wide operating range, so map variation may only be 2–3 percentage points.
- On sustained grades (≥ 5% for several km): motor runs at high torque / low RPM, potentially 93–94% efficiency. A 4–5% error in ηm on these segments compounds into meaningful SoC prediction error.
- **Rule of thumb:** Check the GPX elevation profile for sustained climbs. If grades stay below ~3%, skip the map for v1 and use constant ηm. If the route has significant climbs (WSC has several — the escarpment near Glendambo, ranges near Port Augusta), add the map in v2.
- Implementation cost is low: a 2D NumPy lookup table, interpolated bilinearly. Negligible GA runtime impact.

---

### 7.7 Open-Source Strategy Code: Midnight Sun (U. Waterloo)

**GitHub:** https://github.com/uw-midsun/strategy_msxvi (ASC/FSGP 2024–2025, most recent) and https://github.com/uw-midsun/strategy_xv (prior iteration)

University of Waterloo's Midnight Sun Solar Rayce Car team. Python-based, PostgreSQL-backed, used at ASC 2024. The architecture maps almost directly onto what SSCP needs.

**Architecture:**
- `db/` — PostgreSQL with local + cloud deployment. Local instance runs in the chase car for offline operation; syncs bidirectionally with cloud. Tables: `route_model` (GPX-derived waypoints, elevation, grade), `irradiance` / `irradiance_archive` (Solcast GHI, live + historical).
- `src/simulation.py` — physics simulation loop integrating power flows over timesteps
- `src/optimize.py` — SLSQP optimizer (SciPy) over velocity profile
- `src/overview.py` — elevation and irradiance visualization

**Their physics model (`simulation.py`):**
```python
P_rr  = (M·g·C_r1 + 4·C_r2·v) · v   # two-term rolling resistance
P_drag = 0.5·ρ·Cd·A·v³               # aerodynamic drag (no wind term)
P_grad = max(0, M·g·sin(θ)·v)        # grade (downhill clipped to zero — no regen)
P_solar = A_solar · GHI · η_solar    # solar power
```
Their vehicle parameters: M=300 kg, Cd=0.13, A=1.357 m² → CdA=0.176 (higher than WSC-class; ASC race category).

**Their optimizer (`optimize.py`):**
Uses **SLSQP** (Sequential Least Squares Programming) via `scipy.optimize.minimize` rather than GA. Gradient-based, faster convergence than GA when the problem is smooth, but more sensitive to initial guess and can get stuck in local optima. They note it "requires tuning." Velocity bounds 10–20 m/s, SOC floor constraint ≥ 20%.

**What's better in their implementation vs. Betancur:**
- Two-term rolling resistance (C_r1 + C_r2·v) is more physically accurate than single Crr constant
- GPX pipeline using `gpxpy` library — directly usable by SSCP
- Solcast integration already built — same API SSCP has access to
- PostgreSQL schema for route + irradiance data is a ready-made reference

**What SSCP's model would improve on:**
- No wind term in drag (`vw` missing entirely)
- No battery current model (I(b)) — energy tracked in joules per timestep only
- No motor efficiency map — constant ηm
- Downhill grade clipped to zero — no regen modeled
- No live telemetry integration (listed as future work)
- No mid-race re-optimization triggered by SoC drift

**Optimizer choice — GA vs. SLSQP:**
SLSQP is faster per run and fine when you have a good initial guess (e.g. seed with v*). GA is more robust when the search space is noisy or multimodal, which is more likely when weather uncertainty is high. For SSCP: start with SLSQP seeded by v*, fall back to GA if convergence is unreliable. Both are available in SciPy.

**Key libraries used:**
- `gpxpy` — GPX parsing
- `scipy.optimize.minimize` — SLSQP optimizer
- `psycopg2` — PostgreSQL
- `pvlib` — (implied, for sun position; they use GHI directly from Solcast)
- `numpy`, `pandas`, `matplotlib`

---

### 7.8 Reference Data: CdA and Crr for WSC-Class Cars — Scientific Gems Blog

**URL:** https://scientificgems.wordpress.com/2023/11/11/solar-cars-rolling-resistance-drag/ and https://scientificgems.wordpress.com/2022/02/18/solar-racing-basics-revisited/

**Author:** Tony Nuñez, a well-known analyst/commentator on solar car racing with deep technical knowledge of WSC-class vehicles. Posts extensively on physics and strategy.

**Key reference values for WSC-class cars:**

- **CdA** (= Cd × frontal area A, units m²) — the number that drives the v³ aero drag term. Important: Cd and CdA are different things.
  - World-class WSC teams: CdA ≈ 0.05 m² (Nuna 3: Cd=0.07, A≈0.7 m² → CdA≈0.05 m²)
  - Good student teams: CdA ≈ 0.08–0.15 m²
  - Mid-tier student teams: CdA ≈ 0.15–0.30 m²
  - **SSCP optimizer default: 0.20 m²** (mid-tier, Cd≈0.15 × A≈1.3 m²). Replace with aero model value when available.

- **Crr** (rolling resistance coefficient, dimensionless):
  - Good competition tyres on smooth tarmac: ~0.002
  - Less good tyres or rougher surface: ~0.005
  - **SSCP optimizer default: 0.005** (conservative until rolldown test done)

**Key insight:** At WSC race speeds (~80–90 km/h), aerodynamic drag dominates over rolling resistance for world-class cars (CdA≈0.05). For a mid-tier car (CdA≈0.20), aero is still dominant but rolling resistance is proportionally more significant. Either way, CdA is the more important parameter to get right.

**Relevance for SSCP:** Get the aero model CdA value into the optimizer as soon as possible — it has the largest single impact on predicted power consumption. Crr can be estimated at 0.005 conservatively and refined from a rolldown test.

---

### 7.9 pvlib-python — Solar Position and Irradiance Library

**GitHub / Docs:** https://pvlib-python.readthedocs.io

**Install:** `pip install pvlib`

**Do we actually need this?** Probably not for the production system. The key question is what Solcast returns:

- **If Solcast returns GHI** (Global Horizontal Irradiance) — the sun angle is already baked in. GHI is the irradiance actually hitting a horizontal surface at that location and time, accounting for both atmospheric conditions *and* sun geometry. In that case the solar power equation simplifies to:
  ```
  Ps = GHI · Ai · ηs
  ```
  No `sin(φ)` needed, no pvlib needed. Midnight Sun's implementation confirms this — they use `P_solar = A_solar · GHI · η_solar` directly from Solcast with no sun angle calculation.

- **If Solcast returns DNI** (Direct Normal Irradiance — measured perpendicular to the sun beam) — you'd need to convert to horizontal using sun angle, which is where pvlib earns its keep.

**Check which fields your Solcast access tier returns** (GHI, DNI, and DHI are all standard outputs but confirm). If GHI is available, use it directly and skip pvlib in production.

**Where pvlib is still useful:**
- Development and testing — generate synthetic irradiance for any route/time without an API call, useful before Solcast is wired up
- Validation — cross-check Solcast GHI values against a geometric model to sanity-check obvious errors
- Panel tilt correction — if SSCP's panels are significantly non-horizontal, you'd need the angle of incidence between sun and panel surface; pvlib computes this via `pvlib.irradiance.get_total_irradiance()`
- `pvlib.solarposition.get_solarposition(time, lat, lon)` — computes `φ` (sun elevation) at any point; useful for the pre-race segment table even if just for reference

---

## 8. Access Notes + Internal Resources

- **SSCP Google Sites wiki** (all strategy subpages): behind Google account auth wall — need to be logged in as a Stanford Solar Car team member to access
- **SSCP Google Doc** (7/21/2025 strategy sketches): https://docs.google.com/document/d/1O_CNMQn5NZTiBuV36BSLR_ElW1Zb2zPzpPlOI8Hi-v4/edit (behind auth wall). Key content captured below (§8.1).
- **West rolldown analysis** (Google Doc): https://docs.google.com/document/d/1CzS0pgpmk_cq5IV7uU9NpsL39HTybiMnpHxVLeeAybQ/edit#heading=h.8c584gltcwv3 — prior power sensitivity work and rolldown data. Key input for Crr and CdA calibration.
- **Arctan test driving data (2014–2015)**: https://sites.google.com/stanfordsolarcar.com/sscp/home/sscp-2014-2015/strategy-2014-2015/arctan-test-driving-data-and-analysis (behind auth wall). Controlled speed runs → power-to-drive curves (roughly cubic fit), battery voltage and SoC estimation.
- **Google Drive folder** (test procedures and papers): https://drive.google.com/drive/u/0/folders/13Mx0kvlZaGFw-UIYkyE6C9jmjgD845kb — includes Betancur paper and likely other references.
### 8.1 7/21/2025 Strategy Sketch — Key Notes

**Max's model:** The race strategy at SSCP's last race was executed using "Max's model" — a MATLAB-based car model with a "Do math" button that pulled fresh weather data on demand. This is the direct predecessor to what we're building now. The model and any documentation should be tracked down before too much is re-derived from scratch.

**Technology upgrades available now vs. then:**
- Solcast replaces manual weather data pulls — this is the "Do math" button done properly
- Starlink on the chase car enables live weather/telemetry sync that wasn't feasible before (partnership available)
- A Python app replaces the MATLAB dependency — lower barrier, faster iteration

**Two team outputs for this cycle:**
1. **Car model** — accuracy to real-world performance is the success metric. Requires testing: comms driving in the local area, Central Valley, eventually an ASC-style mock race day.
2. **Aero coordination** — strategy team feeds drag/efficiency constraints to the aero subteam for the new aerobody design.

**Design/build timeline:** WSC regs expected ~June 2026. No molds before then. This year = design. Next year = build. Systems should be developed in parallel; whoever finishes a subsystem first sets constraints for others — needs coordination. Workspace access is a gating factor.

**Cross-subteam communication:** Strategists need to understand CFD; mech needs to consider electrical/battery. Open dialogue via #general is critical so design decisions don't blindside other subteams.

---

- **Sagepub (Hilliard & Jamieson 2008)**: full text obtained via Stanford Library — `hilliard-jamieson-2008-winning-solar-races-with-interface-design.pdf`
- **Springer (Pudney & Howlett 2002)**: full text obtained via Stanford Library — `A_1020907101234.pdf`
- **MDPI (Betancur et al. 2017)**: open access, full PDF available
- **InsideGNSS 2008**: open access PDF

---

## 9. References
- [InsideGNSS 2008 — Michigan NASC](https://insidegnss.com/auto/sepoct08-solarcar.pdf)
- [Hilliard & Jamieson 2008 — Interface Design](https://journals.sagepub.com/doi/abs/10.1518/106480407X312374) *(paywalled; access via Stanford Library)*
- [Betancur et al. 2017 — Heuristic Optimization](https://www.mdpi.com/2071-1050/9/10/1576/pdf) *(open access PDF)*
- [Pudney & Howlett 2002 — Critical Speed Control](https://link.springer.com/article/10.1023/A:1020907101234) *(paywalled; access via Stanford Library)*
- [SSCP Strategy Wiki 2012–2013](https://sites.google.com/stanfordsolarcar.com/sscp/home/sscp-2012-2013/strategy-2012-2013/) *(requires team Google account)*
