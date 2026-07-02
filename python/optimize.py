"""
Pre-race velocity optimizer — CasADi + IPOPT (shooting formulation).

The NLP is built once per (N, n_checkpoints) and cached as a compiled CasADi
Function.  Between outer boundary-update iterations, only parameter values and
variable bounds change — the compiled graph is reused.

Formulation
-----------
Variables:  x = [v (N), Eb (N)]
Parameters: p = [GHI (N), wind_speed (N), wind_dir (N), charge_at (N)]
Objective:  minimize Σ d_i / v_i              [total driving time, s]
Constraints (equality):
    Eb[0] = Eb_start + dE[0] + p_charge[0]
    Eb[j] = Eb[j-1] + dE[j] + p_charge[j]   for j = 1..N-1
Bounds (variable):
    v_min  ≤ v[j]  ≤ v_max_j                 [speed limits + discharge]
    Eb_min ≤ Eb[j] ≤ Eb_max                  [SoC bounds as variable bounds]
    Eb[on_idx] ≥ Eb_overnight_min             [tighter lb at overnight stops]
    Eb[N-1]   ≥ Eb_finish_min                 [finish buffer]
Optional inequality constraints:
    t_open ≤ t_cum[cp] ≤ t_close             [checkpoint time windows]
    Eb[cp] ≥ Eb_min_cp                        [checkpoint SoC]

Why shooting?  Expressing SoC as explicit variables turns 499 stacked
inequality constraints into equality constraints + box bounds.  IPOPT
handles box bounds natively (they appear as simple bound multipliers,
not barrier terms in the general constraint space), and the resulting KKT
system has a bidiagonal Jacobian that MUMPS factors in O(N) time.  The
cumulative-sum formulation created a near-rank-deficient N×N Jacobian that
prevented the barrier parameter from decreasing.
"""

from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
import casadi as ca

from python.params import VehicleParams, RaceParams, ModelFeatures
from python.route import RouteSegment
from python.weather import SegmentWeather
from python.physics import critical_speed_ms, max_speed_from_discharge_limit
from python.simulate import RouteArrays, make_arrays, simulate, SimResult


@dataclass
class SolverStats:
    success:       bool
    n_iter:        int
    return_status: str


@dataclass
class OptimizerResult:
    v_opt:    np.ndarray
    sim:      SimResult
    seed:     np.ndarray
    seed_sim: SimResult
    stats:    SolverStats
    x0:       np.ndarray   # [v_opt, Eb_opt] for warm-starting the next call


# ── Compiled solver cache ─────────────────────────────────────────────────────
# Keyed by the NLP structure determinants.  Route geometry and vehicle physics
# that don't change mid-run are baked into the graph at first build.

_solver_cache: dict[tuple, ca.Function] = {}


# ── Bounds ────────────────────────────────────────────────────────────────────

def _build_v_bounds(
    segments: list[RouteSegment],
    weather:  list[SegmentWeather],
    vehicle:  VehicleParams,
) -> list[tuple[float, float]]:
    bounds = []
    for seg, wx in zip(segments, weather):
        v_upper = min(
            seg.speed_limit_ms,
            vehicle.v_max,
            max_speed_from_discharge_limit(seg, wx, vehicle),
        )
        v_upper = max(v_upper, vehicle.v_min)
        bounds.append((vehicle.v_min, v_upper))
    return bounds


# ── Seed ──────────────────────────────────────────────────────────────────────

def _seed_v_star(
    segments: list[RouteSegment],
    weather:  list[SegmentWeather],
    vehicle:  VehicleParams,
    bounds:   list[tuple[float, float]],
) -> np.ndarray:
    return np.array([
        np.clip(critical_speed_ms(seg, wx, vehicle), lo, hi)
        for seg, wx, (lo, hi) in zip(segments, weather, bounds)
    ])


# ── Feature zeroing ───────────────────────────────────────────────────────────

def _apply_features(
    arrays:   RouteArrays,
    vehicle:  VehicleParams,
    features: ModelFeatures,
) -> tuple[RouteArrays, VehicleParams]:
    import copy, dataclasses
    arrays = copy.copy(arrays)
    veh_kw = {}
    if not features.headwind: arrays.wind_speed   = np.zeros(arrays.N)
    if not features.grade:    arrays.grade        = np.zeros(arrays.N)
    if not features.rolling:  veh_kw["Crr"]       = 0.0
    if not features.kinetic:  arrays.skip_kinetic = True
    if not features.solar:    arrays.GHI          = np.zeros(arrays.N)
    if not features.regen:    arrays.skip_regen   = True
    if not features.drag:     veh_kw["CdA_flat"]  = 0.0
    vehicle = dataclasses.replace(vehicle, **veh_kw) if veh_kw else vehicle
    return arrays, vehicle


# ── Symbolic energy (parametric weather) ─────────────────────────────────────

_KIN_BLEND_WH = 0.1  # Wh — transition width for kinetic smoothing
_P_BLEND_W    = 5.0  # W  — transition width for regen/bat-efficiency smoothing


def _energy_sym(
    v:            ca.MX,
    p_GHI:        ca.MX,
    p_wind_speed: ca.MX,
    p_wind_dir:   ca.MX,
    arrays:       RouteArrays,
    vehicle:      VehicleParams,
) -> ca.MX:
    """
    Symbolic energy delta (Wh) per segment.

    GHI and wind are parameters (reusable across outer iterations).
    All if-else branches use tanh blending to keep the function C², so
    IPOPT can compute accurate exact Hessians and converge in O(10s) of
    iterations rather than 1000+.
    """
    N = arrays.N

    heading_rad  = ca.DM(np.radians(arrays.heading))
    wind_dir_rad = p_wind_dir * (ca.pi / 180.0)
    relative     = wind_dir_rad - heading_rad
    vw           = p_wind_speed * ca.cos(relative)

    F_drag  = 0.5 * vehicle.rho * vehicle.CdA_flat * (v - vw) ** 2
    F_roll  = vehicle.Crr * vehicle.m * vehicle.g
    F_grade = ca.DM(vehicle.m * vehicle.g * arrays.grade)

    Pm = v * (F_drag + F_roll + F_grade)

    alpha_pm = 0.5 * (1.0 + ca.tanh(Pm / _P_BLEND_W))
    if arrays.skip_regen:
        Pb = alpha_pm * Pm / vehicle.eta_m
    else:
        Pb = (alpha_pm / vehicle.eta_m + (1.0 - alpha_pm) * vehicle.eta_regen) * Pm

    net_W = p_GHI * vehicle.Ai * vehicle.eta_s - Pb
    t_h   = ca.DM(arrays.d) / v / 3600.0

    eta_sqrt  = float(np.sqrt(vehicle.eta_b))
    alpha_net = 0.5 * (1.0 + ca.tanh(net_W / _P_BLEND_W))
    dE        = (alpha_net * eta_sqrt + (1.0 - alpha_net) / eta_sqrt) * net_W * t_h

    if arrays.skip_kinetic:
        return dE

    v_prev   = ca.vertcat(v[0], v[:N - 1])
    dKE_Wh   = 0.5 * vehicle.m * (v ** 2 - v_prev ** 2) / 3600.0
    alpha_ke = 0.5 * (1.0 + ca.tanh(dKE_Wh / _KIN_BLEND_WH))
    eff_kin  = alpha_ke / vehicle.eta_m / eta_sqrt + (1.0 - alpha_ke) * vehicle.eta_regen * eta_sqrt

    return dE + (-dKE_Wh * eff_kin)


# ── NLP builder (called once, result cached) ──────────────────────────────────

def _build_solver(
    arrays:   RouteArrays,
    vehicle:  VehicleParams,
    race:     RaceParams,
    opts:     dict,
) -> ca.Function:
    """
    Shooting-method NLP.  Eb is an explicit decision variable.
    Energy dynamics are equality constraints; SoC bounds are variable bounds.
    """
    N = arrays.N
    v    = ca.MX.sym('v',  N)
    Eb_v = ca.MX.sym('Eb', N)
    x    = ca.vertcat(v, Eb_v)

    p             = ca.MX.sym('p', 4 * N)
    p_GHI         = p[0:N]
    p_wind_speed  = p[N:2 * N]
    p_wind_dir    = p[2 * N:3 * N]
    p_charge_at   = p[3 * N:4 * N]   # incremental charge at each segment

    d_dm = ca.DM(arrays.d)
    obj  = ca.sum1(d_dm / v)

    dE = _energy_sym(v, p_GHI, p_wind_speed, p_wind_dir, arrays, vehicle)

    # Energy dynamics: Eb[j] = Eb[j-1] + dE[j] + charge[j]
    Eb_prev = ca.vertcat(race.Eb_start, Eb_v[:N - 1])
    g_eq    = Eb_v - Eb_prev - dE - p_charge_at   # == 0

    g_list: list[ca.MX] = [g_eq]

    if race.checkpoints:
        t_cum = ca.cumsum(d_dm / v)
        for cp in race.checkpoints:
            idx = cp.segment_index
            g_list += [t_cum[idx], Eb_v[idx]]

    g   = ca.vertcat(*g_list)
    nlp = {'x': x, 'p': p, 'f': obj, 'g': g}
    return ca.nlpsol('S', 'ipopt', nlp, opts)


# ── Main entry point ──────────────────────────────────────────────────────────

def run_optimizer(
    segments: list[RouteSegment],
    weather:  list[SegmentWeather],
    vehicle:  VehicleParams,
    race:     RaceParams,
    features: ModelFeatures = None,
    max_iter: int   = 1000,
    tol:      float = 1e-6,
    verbose:  bool  = False,
    x0:       np.ndarray | None = None,   # warm-start: [v, Eb] from previous solve
) -> OptimizerResult:
    if features is None:
        features = ModelFeatures()
    arrays, vehicle = _apply_features(make_arrays(segments, weather), vehicle, features)
    v_bounds = _build_v_bounds(segments, weather, vehicle)
    N        = arrays.N

    opts = {
        'ipopt.max_iter':    max_iter,
        'ipopt.print_level': 5 if verbose else 0,
        'ipopt.tol':         tol,
        'print_time':        0,
    }

    cache_key = (N, arrays.skip_kinetic, arrays.skip_regen,
                 vehicle.CdA_flat, vehicle.Crr, vehicle.m,
                 vehicle.eta_m, vehicle.eta_regen,
                 race.Eb_start, vehicle.Eb_max,
                 len(race.checkpoints), max_iter, tol)
    if cache_key not in _solver_cache:
        print("  Compiling NLP (first run for this problem size)...")
        _solver_cache[cache_key] = _build_solver(arrays, vehicle, race, opts)
    solver = _solver_cache[cache_key]

    # Incremental charges (non-cumulative)
    charge_at = np.zeros(N)
    for idx, charge in zip(race.overnight_segment_indices, race.overnight_charge_Wh):
        if idx + 1 < N:
            charge_at[idx + 1] += charge

    p_val = np.concatenate([
        arrays.GHI, arrays.wind_speed, arrays.wind_dir, charge_at,
    ])

    # Variable bounds: v (per segment) and Eb (SoC with overnight/finish floors)
    lbv = np.array([lo for lo, _ in v_bounds])
    ubv = np.array([hi for _, hi in v_bounds])

    lbE = np.full(N, vehicle.Eb_min)
    ubE = np.full(N, vehicle.Eb_max)
    for on_idx in race.overnight_segment_indices:
        if on_idx < N:
            lbE[on_idx] = max(lbE[on_idx], race.Eb_overnight_min)
    lbE[N - 1] = max(lbE[N - 1], race.Eb_finish_min)

    lbx = np.concatenate([lbv, lbE])
    ubx = np.concatenate([ubv, ubE])

    # Equality constraints: g_eq == 0
    lbg: list[float] = [0.0] * N
    ubg: list[float] = [0.0] * N
    for cp in race.checkpoints:
        lbg += [cp.t_open_s,  cp.Eb_min_Wh]
        ubg += [cp.t_close_s, vehicle.Eb_max]

    # Seed and initial x0
    seed     = _seed_v_star(segments, weather, vehicle, v_bounds)
    seed_sim = simulate(seed, segments, weather, vehicle, race, arrays=arrays)
    x0_seed  = np.concatenate([seed, seed_sim.Eb])

    x0_use = x0 if x0 is not None else x0_seed

    if verbose:
        print(f"  Seed v*: {seed_sim.total_time_s / 3600:.2f} h, "
              f"min SoC = {seed_sim.min_Eb_Wh:.0f} Wh")

    sol = solver(
        x0  = x0_use,
        p   = p_val,
        lbx = lbx, ubx = ubx,
        lbg = lbg, ubg = ubg,
    )

    s             = solver.stats()
    return_status = s['return_status']
    success       = return_status in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')

    x_opt  = np.array(sol['x']).flatten()
    v_opt  = np.clip(x_opt[:N], lbv, ubv)
    Eb_opt = x_opt[N:]
    sim    = simulate(v_opt, segments, weather, vehicle, race, arrays=arrays)

    return OptimizerResult(
        v_opt=v_opt,
        sim=sim,
        seed=seed,
        seed_sim=seed_sim,
        stats=SolverStats(
            success=success,
            n_iter=int(s.get('iter_count', -1)),
            return_status=return_status,
        ),
        x0=x_opt,   # [v_opt, Eb_opt] ready for warm-starting next call
    )
