# Solar Car Race Strategy — Research Notes

## 1. Past Solar Car Notes (SSCP)

### 1.1 Conventional Wisdom

Two common strategies:

1. Match a constant, optimal speed
2. Match a desired battery SoC

SoC should drive long-term race strategy; holding a constant speed is a short-term goal given acceleration/deceleration losses and the cubic relationship between power and drag. Power production and consumption are the two sides of the equation — over the race, consume all the power produced.

### 1.2 Luminos (2012–2013) — Strategy Notes

From the SSCP wiki pages on Luminos (Stanford's prior competitive car):

- **Optimization approach:** constrained optimization over a ~3000-dimensional velocity vector (one speed per race segment), with battery SoC > 0 as the constraint. This is exactly the architecture now in `python/optimize.py`.
- **SoC modeling:** Luminos used a complementary filter — likely a blend of voltage-based estimation and Ah integration. SSCP's current firmware integrates Ah at 100 Hz (coulomb counting) but doesn't feed it back into the SoC estimate (still linear in min cell voltage).
- **Planning insight:** constant speed is not optimal — losses vary with velocity and conditions, the road isn't flat, and the motor is highly grade-sensitive. The wiki pointed to the Pudney (Adelaide) and Betancur papers as the references to explore; both are in §2.

### 1.3 Xenith (2011 WSC) — Real Parameters & Race Debrief

Concrete numbers from a real Stanford car — calibration anchors and sanity checks.

- **Array:** SunPower C60, 390 cells, **5.97 m²**. ~1300 W STC → **~1150 W working** after derating (~97% tracker eff); measured ~1130 W at Darwin.
- **Battery:** 455× Panasonic NCR18650, **35S13P**, 2.91 Ah/cell measured → **~4.8 kWh**. Internal resistance **~0.06 Ω** (to ~0.12 Ω below 20% SOC).
- **MPPT telemetry (CAN):** device 24, messages 0–3, each four 2-byte ints — array V (0.01 V), current (mA), battery V (0.01 V), temp (0.01 °C). Direct ancestor of today's MPPT message format.
- **Debrief:** weather underestimated → get custom forecasts + a **pyranometer/reference cell on scout** for ground-truth irradiance ahead of the car; comms with scout failed (use **sat phone + BGAN**); validate strategy by predicting power *before* test drives then checking; MPPT resweep under tree-shadow flicker killed production some afternoons; haze + AR coating may hurt at low sun angles.

### 1.4 Luminos (2013) — Test-Driving Lessons

Cruise power-vs-speed points taken where speed is steady (low stddev over 30 s); controlled runs in 10 kph steps, random order so wind can't masquerade as a speed effect.

- **Towns cost time:** 55–60 min/leg vs. 47 expected → forces higher cruise elsewhere; model control-stop / town slowdowns.
- Morning insolation cut by roadside trees — plan for local shading, not just cloud.
- Car ran **more efficient than bench characterization** (wheel covers, road surface, sealing) — characterize on the real road surface.
- **GPS lock unreliable** → manual route-index fallback needed; an indexing bug burned power to ~5% SOC.
- **Daylight headlights drew ~35 W** more than expected — model parasitic/aux loads ([strategy_takeaways.md](strategy_takeaways.md) §2).
- **SOC fluctuates** while driving — use multiple estimators + a human check before the solver (reinforces §1.2).
- A full day on the final array wiring produced **~8.3 kWh** — a real daily-yield anchor.

### 1.5 Weather & Tooling Pipeline (2013)

The 2013 stack manually fetched NOAA GFS + WeatherZone, parsed via the NOAA Weather & Climate Toolkit, and optimized in **AMPL**.

- **Takeaway:** Solcast (GHI + cloud + wind from one API) collapses that whole manual pipeline; the optimizer is now Python/scipy.
- WeatherZone's FTP split cloud into total % + low/medium/high layers — useful granularity to ask of any forecast provider.

---

## 2. Paper Notes

### 2.1 "Follow the Sun" — Michigan's NASC 2008 win (InsideGNSS)

https://insidegnss.com/auto/sepoct08-solarcar.pdf

- **Drive fast under clouds** to reach sun sooner — likely decisive in their 10-hour margin.
- Run **two independent optimizers** and judge which tracks reality — a hedge against model error.
- They ran **continuous real-time sims in the chase car** (inputs: radiation, crosswind, pre-surveyed grade, SoC, weight) — the live re-optimization loop we're building.
- **Pre-race route survey at 10 Hz**, marking hills, stop signs, and speed-limit changes — the elevation + speed-limit profile is a must-have optimizer input.
- **Cruise command pushed to the driver**, accepted/rejected via a steering-wheel button — a clean driver-interface pattern; share one live telemetry feed across chase + lead.

### 2.2 "Winning Solar Races with Interface Design" — Hilliard & Jamieson 2008

https://journals.sagepub.com/doi/pdf/10.1518/106480407X312374 *(paywalled; Stanford Library)*

- Build the chase-car display as a **time-distance space**: distance × time-of-day, the planned speed profile integrated into a trajectory, with the **cloud forecast overlaid**. Turns "speed up under clouds" into something the strategist *sees* rather than computes.
- Two other useful views: a **motor-efficiency map** (efficiency contours vs operating point) and a **car-handling safety map** (speed × inverse corner radius, shrinking when wet or fast).
- Core EID principle: **make the physics visible so the strategist perceives constraint violations rather than inferring them.** (Contextual-advice displays cut fuel use ~14% in conventional vehicles — UI is worth investing in.)

### 2.3 "Heuristic Optimization…" — Betancur et al. 2017 (WSC 2015)

https://www.mdpi.com/2071-1050/9/10/1576/pdf *(open access)*

Source of our drivetrain model:

```
Pm = v * [m*a + ½*CdA*ρ*(v - vw)² + Crr*m*g + m*g*sin(θ)]
```

- Canonical **four-sub-model architecture** — drivetrain + solar + battery + climate — with the optimizer as a *separate* module (vehicle specs are just parameters). This is our architecture.
- Solar `Ps = Ii·Ai·ηs·sin(φ)`; battery uses an **asymmetric √ηb round-trip** (charge ×√ηb, discharge ÷√ηb) — exactly what `simulate.py` implements.
- Optimal speeds sit in a **narrow band** (~78–83 km/h) — nearly constant; a driver wouldn't feel the difference. The optimum **drains the battery to ~empty at the finish**.
- A 1D→10D speed vector saves only **~3%**; a **40% irradiance cut for one full day ≈ +2 h** race time — quantifies weather sensitivity.
- GA+LS beats BB-BC; heuristic search becomes necessary past ~3D (exhaustive is intractable), and it's fast enough to **re-run mid-race (<3 min)**.

### 2.4 Pudney & Howlett 2002 — Critical Speed Control (Aurora, WSC 1999)

https://link.springer.com/article/10.1023/A:1020907101234 *(paywalled; Stanford Library)*

- Optimal race = **power → hold → brake**: hold the **critical speed v\*** for essentially the entire race.
- v\* rises only slowly with sun (~86→91 km/h over 0→1500 W) — speed is far less sun-sensitive than intuition; the battery absorbs the variation.
- Their v\* is **battery-aware** (folds in the I(b) curve + the marginal value of charge); our seed's v\* only solves solar = demand. Sanity check: constant sun but wildly varying optimizer speeds ⇒ a bug.
- Battery modeled as a **quadratic current-power fit** `I(b) = c1·b + c2·b²` — the ancestor of our `I(b)` curve; validated to ~1% (predicted 253 vs 250 km driven on a no-sun day).
- Formulated as **per-day distance maximization** given start/end charge, not time minimization — a useful alternate framing for "can't finish the stage under power" cases (cf. ASC ranking on Official Distance).

### 2.5 "Race Simulation & EMS" — ELECO 2025

https://www.eleco.org.tr/ELECO2025/Eleco2025-Papers/174.pdf

- A **(torque, RPM) → efficiency motor map** matters on sustained climbs (>3% grade, where ηm drops to ~93–94%); a constant ηm underestimates climb consumption. Cheap 2D lookup — add in v2.
- Uses a **PMSM model — the same motor type as ours** — and validated to ~1.5% per lap, so a map is credibly worth the effort.

### 2.6 Midnight Sun (U. Waterloo) — open-source strategy code

https://github.com/uw-midsun/strategy_msxvi

- Closest analog to our stack: **SLSQP over a velocity profile, with Solcast + gpxpy + PostgreSQL**. Borrow their **two-term rolling resistance** and route/irradiance schema.
- **Local DB in the chase car** for offline operation, syncing to cloud — good architecture for spotty connectivity.
- Their params: M=300 kg, Cd=0.13, A=1.357 m² → **CdA=0.176** — a comparison anchor for an ASC-class car.
- We go further: wind term in drag, an I(b) battery model, a motor map, regen, live telemetry, and mid-race re-optimization on SoC drift. Pitfall to avoid copying: they **clip downhill grade to zero (no regen)**.

### 2.7 CdA / Crr Reference Data — Scientific Gems (Nuñez)

https://scientificgems.wordpress.com/2023/11/11/solar-cars-rolling-resistance-drag/

- **CdA:** world-class ~0.05, mid-tier student ~0.15–0.30, **our default 0.20 m²**. **Crr:** ~0.002–0.005, **our default 0.005**.
- At race speed (~80–90 km/h) **aero dominates rolling resistance**, so **CdA is the single most impactful parameter** — get the aero value in ASAP; refine Crr by rolldown test.

---

## 3. SSCP Internal Resources

Require a team Google account / FTP / SVN:

- [Strategy Wiki 2012–2013](https://sites.google.com/stanfordsolarcar.com/sscp/home/sscp-2012-2013/strategy-2012-2013) — source of §1.2–1.5. Many leaf pages point to attached PDFs/spreadsheets/MATLAB on the team FTP/SVN.
- [7/21/2025 strategy doc](https://docs.google.com/document/d/1O_CNMQn5NZTiBuV36BSLR_ElW1Zb2zPzpPlOI8Hi-v4/edit)
- [West rolldown analysis](https://docs.google.com/document/d/1CzS0pgpmk_cq5IV7uU9NpsL39HTybiMnpHxVLeeAybQ/edit) — prior power-sensitivity and rolldown data; key input for Crr/CdA calibration.
- [Arctan test-driving data (2014–2015)](https://sites.google.com/stanfordsolarcar.com/sscp/home/sscp-2014-2015/strategy-2014-2015/arctan-test-driving-data-and-analysis)
- [Google Drive folder (test procedures and papers)](https://drive.google.com/drive/u/0/folders/13Mx0kvlZaGFw-UIYkyE6C9jmjgD845kb)
