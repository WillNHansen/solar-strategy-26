"""
Pre-race velocity optimizer.

SLSQP seeded with per-segment critical speed v* (the Pudney equilibrium speed).

Performance design:
  - RouteArrays precomputed once — no Python list iteration in the hot path
  - energy_deltas_vec() is pure numpy — ~0.1ms per call for N=6000 segments
  - Aggregate SoC constraints (2 constraints, not 2N) — reduces Jacobian from
    O(N²) to O(N) columns, cutting runtime from hours to ~30s for N=6000

Objective:  minimize Σ (d_i / v_i)          [total race time]
Variables:  v[0..N-1]                        [speed m/s per segment]
Bounds:     v_min ≤ v_i ≤ min(speed_limit_i, v_max_discharge_i)
Constraints (all expressed as fun(v) ≥ 0):
  - min(Eb) ≥ Eb_min                         [SoC floor, aggregate]
  - Eb_max - max(Eb) ≥ 0                     [SoC ceiling, aggregate]
  - Eb[-1] ≥ Eb_finish_min                   [finish buffer]
  - t_arr[k] ≥ t_open[k]   per checkpoint    [arrive after open]
  - t_close[k] - t_arr[k] ≥ 0               [arrive before close]
  - Eb[k] ≥ Eb_min[k]      per checkpoint    [SoC at checkpoint]
  - Eb[overnight] ≥ Eb_overnight_min         [nightly buffer]
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.optimize import minimize, OptimizeResult

from params import VehicleParams, RaceParams
from route import RouteSegment
from weather import SegmentWeather
from physics import critical_speed_ms, max_speed_from_discharge_limit
from simulate import (
    RouteArrays, make_arrays,
    cumulative_energy_vec, cumulative_time_vec,
    energy_deltas_grad_vec,
    simulate, SimResult,
)


@dataclass
class OptimizerResult:
    v_opt:         np.ndarray
    sim:           SimResult
    scipy_result:  OptimizeResult
    seed:          np.ndarray


# ── Bounds ────────────────────────────────────────────────────────────────────

def _build_bounds(
    segments: list[RouteSegment],
    weather: list[SegmentWeather],
    vehicle: VehicleParams,
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


# ── Constraints ───────────────────────────────────────────────────────────────

def _build_constraints(
    arrays: RouteArrays,
    vehicle: VehicleParams,
    race: RaceParams,
) -> list[dict]:
    """
    Build SLSQP constraint dicts with analytical Jacobians.

    Each constraint provides a 'jac' key with an exact gradient, eliminating
    the N×m finite-difference calls that dominated runtime. Total constraint
    Jacobian cost drops from O(N² × m) to O(N × m) per SLSQP iteration.

    Key identity: since Eb[j] = Eb_start + Σ_{k≤j} dE[k] and dE[k] depends
    only on v[k], the Jacobian of Eb w.r.t. v is lower-triangular.
    For aggregate constraints (min, max, final value), only a prefix or the
    full diagonal of dE_grad is needed — all computable in O(N).
    """
    constraints = []
    N = arrays.N
    idx_arr = np.arange(N)

    def _prefix_jac(diag: np.ndarray, sub: np.ndarray, k: int) -> np.ndarray:
        """
        Jacobian of Eb[k] w.r.t. all v[j].

        With kinetic transitions, dE[i] depends on v[i] (diagonal) and v[i-1]
        (subdiagonal). So d(Eb[k])/d(v[j]) = diag[j] * (j≤k) + sub[j+1] * (j+1≤k).
        """
        jac = np.where(idx_arr <= k, diag, 0.0)
        if k > 0:
            jac[:k] += sub[1:k + 1]
        return jac

    # ── SoC floor: min(Eb) - Eb_min ≥ 0 ──────────────────────────────────────
    def soc_floor(v: np.ndarray) -> float:
        return float(cumulative_energy_vec(v, arrays, vehicle, race).min()) - vehicle.Eb_min

    def soc_floor_jac(v: np.ndarray) -> np.ndarray:
        Eb           = cumulative_energy_vec(v, arrays, vehicle, race)
        diag, sub    = energy_deltas_grad_vec(v, arrays, vehicle)
        return _prefix_jac(diag, sub, int(np.argmin(Eb)))

    # ── SoC ceiling: Eb_max - max(Eb) ≥ 0 ────────────────────────────────────
    def soc_ceil(v: np.ndarray) -> float:
        return vehicle.Eb_max - float(cumulative_energy_vec(v, arrays, vehicle, race).max())

    def soc_ceil_jac(v: np.ndarray) -> np.ndarray:
        Eb           = cumulative_energy_vec(v, arrays, vehicle, race)
        diag, sub    = energy_deltas_grad_vec(v, arrays, vehicle)
        return -_prefix_jac(diag, sub, int(np.argmax(Eb)))

    # ── Finish buffer: Eb[-1] - Eb_finish_min ≥ 0 ────────────────────────────
    def finish_soc(v: np.ndarray) -> float:
        return float(cumulative_energy_vec(v, arrays, vehicle, race)[-1]) - race.Eb_finish_min

    def finish_soc_jac(v: np.ndarray) -> np.ndarray:
        diag, sub = energy_deltas_grad_vec(v, arrays, vehicle)
        return _prefix_jac(diag, sub, N - 1)

    constraints += [
        {"type": "ineq", "fun": soc_floor,  "jac": soc_floor_jac},
        {"type": "ineq", "fun": soc_ceil,   "jac": soc_ceil_jac},
        {"type": "ineq", "fun": finish_soc, "jac": finish_soc_jac},
    ]

    # ── Checkpoints ───────────────────────────────────────────────────────────
    for cp in race.checkpoints:
        cp_idx = cp.segment_index

        def cp_open(v: np.ndarray, i: int = cp_idx, t_open: float = cp.t_open_s) -> float:
            return float(cumulative_time_vec(v, arrays)[i]) - t_open

        def cp_open_jac(v: np.ndarray, i: int = cp_idx) -> np.ndarray:
            # d(t_cum[i])/d(v[k]) = -d[k]/v[k]² if k ≤ i, else 0
            g = np.zeros(N)
            g[:i + 1] = -arrays.d[:i + 1] / v[:i + 1] ** 2
            return g

        def cp_close(v: np.ndarray, i: int = cp_idx, t_close: float = cp.t_close_s) -> float:
            return t_close - float(cumulative_time_vec(v, arrays)[i])

        def cp_close_jac(v: np.ndarray, i: int = cp_idx) -> np.ndarray:
            g = np.zeros(N)
            g[:i + 1] = arrays.d[:i + 1] / v[:i + 1] ** 2
            return g

        def cp_soc(v: np.ndarray, i: int = cp_idx, Eb_min: float = cp.Eb_min_Wh) -> float:
            return float(cumulative_energy_vec(v, arrays, vehicle, race)[i]) - Eb_min

        def cp_soc_jac(v: np.ndarray, i: int = cp_idx) -> np.ndarray:
            diag, sub = energy_deltas_grad_vec(v, arrays, vehicle)
            return _prefix_jac(diag, sub, i)

        constraints += [
            {"type": "ineq", "fun": cp_open,  "jac": cp_open_jac},
            {"type": "ineq", "fun": cp_close, "jac": cp_close_jac},
            {"type": "ineq", "fun": cp_soc,   "jac": cp_soc_jac},
        ]

    # ── Overnight stops ───────────────────────────────────────────────────────
    for on_idx in race.overnight_segment_indices:
        def overnight(v: np.ndarray, i: int = on_idx) -> float:
            return float(cumulative_energy_vec(v, arrays, vehicle, race)[i]) - race.Eb_overnight_min

        def overnight_jac(v: np.ndarray, i: int = on_idx) -> np.ndarray:
            diag, sub = energy_deltas_grad_vec(v, arrays, vehicle)
            return _prefix_jac(diag, sub, i)

        constraints.append({"type": "ineq", "fun": overnight, "jac": overnight_jac})

    return constraints


# ── Objective + analytical gradient ──────────────────────────────────────────

def _objective(v: np.ndarray, arrays: RouteArrays) -> float:
    """Total race time (s). Divide by v elementwise — pure numpy, no Python loop."""
    return float(np.sum(arrays.d / v))


def _obj_gradient(v: np.ndarray, arrays: RouteArrays) -> np.ndarray:
    """
    Analytical gradient of total race time w.r.t. each segment speed.
    d(Σ d_i/v_i)/d(v_i) = -d_i / v_i²

    Providing this eliminates N finite-difference calls per SLSQP iteration,
    dropping gradient computation from O(N²) to O(N). All constraint Jacobians
    are also analytical (see _build_constraints), so no finite differences are used.
    """
    return -arrays.d / v ** 2


# ── Seed ──────────────────────────────────────────────────────────────────────

def _seed_v_star(
    segments: list[RouteSegment],
    weather: list[SegmentWeather],
    vehicle: VehicleParams,
    bounds: list[tuple[float, float]],
) -> np.ndarray:
    """
    Per-segment critical speed v* as the SLSQP initial guess.
    v* is the Pudney equilibrium: solar power exactly covers motor demand.
    Already close to optimal on flat terrain; SLSQP corrects for hills + clouds.
    """
    return np.array([
        np.clip(critical_speed_ms(seg, wx, vehicle), lo, hi)
        for seg, wx, (lo, hi) in zip(segments, weather, bounds)
    ])


# ── Dynamic Programming optimizer ────────────────────────────────────────────

def _dp_energy_delta_vec(
    v_grid: np.ndarray,
    seg_idx: int,
    arrays: RouteArrays,
    vehicle: VehicleParams,
) -> np.ndarray:
    """
    Energy delta (Wh) for each speed in v_grid at a single segment.

    Kinetic energy transitions (½mv²) are omitted — including them requires
    tracking v_prev as a second state dimension, which multiplies the state
    space by n_v. Their contribution is small (~1–2%) vs. steady-state terms.
    """
    vw      = arrays.wind_speed[seg_idx] * np.cos(np.radians(arrays.wind_dir[seg_idx] - arrays.heading[seg_idx]))
    F_aero  = 0.5 * vehicle.rho * vehicle.CdA_flat * (v_grid - vw) ** 2
    F_roll  = vehicle.Crr * vehicle.m * vehicle.g
    F_grade = vehicle.m * vehicle.g * arrays.grade[seg_idx]
    Pm      = v_grid * (F_aero + F_roll + F_grade)
    Pb      = np.where(Pm >= 0, Pm / vehicle.eta_m, Pm * vehicle.eta_regen)
    Ps      = arrays.GHI[seg_idx] * vehicle.Ai * vehicle.eta_s
    net_W   = Ps - Pb
    t_h     = arrays.d[seg_idx] / v_grid / 3600.0
    eta_sqrt = vehicle.eta_b ** 0.5
    return np.where(net_W >= 0, net_W * t_h * eta_sqrt, net_W * t_h / eta_sqrt)


def run_dp(
    segments: list[RouteSegment],
    weather: list[SegmentWeather],
    vehicle: VehicleParams,
    race: RaceParams,
    n_Eb: int = 200,
    n_v:  int = 50,
) -> OptimizerResult:
    """
    Dynamic Programming optimizer — global optimum within grid resolution.

    State:      (segment index, Eb level j)
    Control:    speed v, discretized to n_v levels per segment
    Transition: Eb_next = clip(Eb + dE(v, segment), 0, Eb_max)

    Unlike SLSQP, battery clipping is modeled exactly in the state transition.
    Solar energy above Eb_max is permanently lost, so the DP naturally
    prescribes a morning sprint on full-battery days to prevent hitting the
    ceiling — the behaviour SLSQP misses due to its unclipped cumsum formulation.

    Complexity: O(N × n_Eb × n_v). At N=500, n_Eb=200, n_v=50 ≈ 5M ops, ~seconds.
    """
    from scipy.optimize import OptimizeResult

    N      = len(segments)
    arrays = make_arrays(segments, weather)
    bounds = _build_bounds(segments, weather, vehicle)

    Eb_grid = np.linspace(vehicle.Eb_min, vehicle.Eb_max, n_Eb)

    overnight_set = set(race.overnight_segment_indices)
    overnight_charge_map = {
        idx + 1: charge
        for idx, charge in zip(race.overnight_segment_indices, race.overnight_charge_Wh)
        if idx + 1 < N
    }

    INF = np.inf

    # Terminal value function: 0 cost if SoC meets finish minimum, else infeasible.
    V_next = np.where(Eb_grid >= race.Eb_finish_min, 0.0, INF)

    # policy[i, j] = index into v_grid for segment i at SoC level j.
    policy  = np.zeros((N, n_Eb), dtype=np.int32)
    v_grids = []   # per-segment speed grids, built backwards then reversed

    # ── Backward pass ─────────────────────────────────────────────────────────
    for i in range(N - 1, -1, -1):
        lo, hi = bounds[i]
        v_grid = np.linspace(lo, hi, n_v)
        v_grids.append(v_grid)

        # Energy delta for every speed at this segment: shape (n_v,)
        dE_v = _dp_energy_delta_vec(v_grid, i, arrays, vehicle)

        # Eb after driving segment i: shape (n_Eb, n_v)
        Eb_after = np.clip(Eb_grid[:, None] + dE_v[None, :], 0.0, vehicle.Eb_max)

        # Apply overnight charge before the next segment, if applicable.
        # The state entering segment i+1 is Eb_after + charge, clipped.
        if (i + 1) in overnight_charge_map:
            Eb_for_next = np.clip(Eb_after + overnight_charge_map[i + 1], 0.0, vehicle.Eb_max)
        else:
            Eb_for_next = Eb_after

        # Look up future cost from next-segment value function
        j_next = np.searchsorted(Eb_grid, Eb_for_next).clip(0, n_Eb - 1)
        total  = arrays.d[i] / v_grid[None, :] + V_next[j_next]   # (n_Eb, n_v)

        # Global SoC floor: Eb after every segment must be >= Eb_min.
        # A half-cell buffer absorbs quantization error when the continuous
        # simulate() re-runs the extracted policy.
        Eb_floor = vehicle.Eb_min + 0.5 * (Eb_grid[1] - Eb_grid[0])
        total[Eb_after < Eb_floor] = INF

        # Overnight floor: Eb at end of an overnight stop must be >= Eb_overnight_min
        if i in overnight_set:
            total[Eb_after < race.Eb_overnight_min] = INF

        best_k     = np.argmin(total, axis=1)                    # (n_Eb,)
        policy[i]  = best_k
        V_next     = total[np.arange(n_Eb), best_k]             # (n_Eb,)

    v_grids.reverse()   # built N-1→0, reverse to index by segment

    # ── Forward pass ──────────────────────────────────────────────────────────
    v_opt   = np.empty(N)
    Eb_curr = float(race.Eb_start)

    for i in range(N):
        if i in overnight_charge_map:
            Eb_curr = float(np.clip(Eb_curr + overnight_charge_map[i], 0.0, vehicle.Eb_max))

        j  = int(np.searchsorted(Eb_grid, Eb_curr).clip(0, n_Eb - 1))
        vi = float(v_grids[i][policy[i, j]])
        v_opt[i] = vi

        dE      = float(_dp_energy_delta_vec(np.array([vi]), i, arrays, vehicle)[0])
        Eb_curr = float(np.clip(Eb_curr + dE, vehicle.Eb_min, vehicle.Eb_max))

    sim = simulate(v_opt, segments, weather, vehicle, race)

    result = OptimizeResult(
        x=v_opt, success=True,
        message=f"DP global optimum (n_Eb={n_Eb}, n_v={n_v})",
        fun=float(np.sum(arrays.d / v_opt)), nit=N,
    )
    return OptimizerResult(v_opt=v_opt, sim=sim, scipy_result=result, seed=v_opt)


# ── DP + SLSQP hybrid optimizer ───────────────────────────────────────────────

def run_dp_then_slsqp(
    segments: list[RouteSegment],
    weather: list[SegmentWeather],
    vehicle: VehicleParams,
    race: RaceParams,
    n_Eb: int = 2000,
    n_v: int = 400,
    max_iter: int = 2000,
    tol: float = 1e-6,
) -> OptimizerResult:
    """
    Two-stage hybrid optimizer:
      1. DP finds the globally optimal energy allocation shape (morning sprint,
         correct per-day budget), but produces a jerky/discretized speed profile.
      2. SLSQP is warm-started from the DP solution to refine it into a smooth,
         continuous, fully-feasible profile.

    Returns the SLSQP-refined result tagged with the DP solution as its seed.
    """
    dp_result = run_dp(segments, weather, vehicle, race, n_Eb=n_Eb, n_v=n_v)

    arrays      = make_arrays(segments, weather)
    bounds      = _build_bounds(segments, weather, vehicle)
    constraints = _build_constraints(arrays, vehicle, race)
    seed        = np.clip(dp_result.v_opt, [lo for lo, _ in bounds], [hi for _, hi in bounds])

    refined = minimize(
        fun=_objective, jac=_obj_gradient, x0=seed,
        args=(arrays,), method="SLSQP", bounds=bounds, constraints=constraints,
        options={"maxiter": max_iter, "ftol": tol, "disp": False},
    )
    v_opt = np.clip(refined.x, [lo for lo, _ in bounds], [hi for _, hi in bounds])
    sim   = simulate(v_opt, segments, weather, vehicle, race)
    return OptimizerResult(v_opt=v_opt, sim=sim, scipy_result=refined, seed=dp_result.v_opt)


# ── Multi-seed parallel optimizer ────────────────────────────────────────────

def run_multi_seed(
    segments: list[RouteSegment],
    weather: list[SegmentWeather],
    vehicle: VehicleParams,
    race: RaceParams,
    seeds: list[np.ndarray],
    max_iter: int = 2000,
    tol: float = 1e-6,
) -> list[OptimizerResult]:
    """
    Run SLSQP from multiple starting points in parallel and return all results.

    Uses ThreadPoolExecutor — scipy's SLSQP releases the GIL during numerical
    work, so threads parallelize without pickling overhead.
    """
    from concurrent.futures import ThreadPoolExecutor, as_completed

    arrays      = make_arrays(segments, weather)
    bounds      = _build_bounds(segments, weather, vehicle)
    constraints = _build_constraints(arrays, vehicle, race)

    def _run_one(seed: np.ndarray) -> OptimizerResult:
        seed_clipped = np.clip(seed, [lo for lo, _ in bounds], [hi for _, hi in bounds])
        result = minimize(
            fun=_objective,
            jac=_obj_gradient,
            x0=seed_clipped,
            args=(arrays,),
            method="SLSQP",
            bounds=bounds,
            constraints=constraints,
            options={"maxiter": max_iter, "ftol": tol, "disp": False},
        )
        v_opt = np.clip(result.x, [lo for lo, _ in bounds], [hi for _, hi in bounds])
        sim   = simulate(v_opt, segments, weather, vehicle, race)
        return OptimizerResult(v_opt=v_opt, sim=sim, scipy_result=result, seed=seed_clipped)

    results = [None] * len(seeds)
    with ThreadPoolExecutor(max_workers=len(seeds)) as pool:
        futures = {pool.submit(_run_one, s): i for i, s in enumerate(seeds)}
        for future in as_completed(futures):
            results[futures[future]] = future.result()

    return results


def make_constant_seed(
    speed_ms: float,
    bounds: list[tuple[float, float]],
) -> np.ndarray:
    """Constant-speed seed clipped to per-segment bounds."""
    return np.array([np.clip(speed_ms, lo, hi) for lo, hi in bounds])


# ── Main entry point ──────────────────────────────────────────────────────────

def run_optimizer(
    segments: list[RouteSegment],
    weather: list[SegmentWeather],
    vehicle: VehicleParams,
    race: RaceParams,
    max_iter: int = 1000,
    tol: float = 1e-6,
    verbose: bool = False,
) -> OptimizerResult:
    arrays      = make_arrays(segments, weather)
    bounds      = _build_bounds(segments, weather, vehicle)
    constraints = _build_constraints(arrays, vehicle, race)
    seed        = _seed_v_star(segments, weather, vehicle, bounds)

    if verbose:
        seed_sim = simulate(seed, segments, weather, vehicle, race)
        print(f"Seed v*: {seed_sim.total_time_s / 3600:.2f} h, "
              f"min SoC = {seed_sim.min_Eb_Wh:.0f} Wh, feasible = {seed_sim.feasible}")

    result = minimize(
        fun=_objective,
        jac=_obj_gradient,
        x0=seed,
        args=(arrays,),
        method="SLSQP",
        bounds=bounds,
        constraints=constraints,
        options={"maxiter": max_iter, "ftol": tol, "disp": verbose},
    )

    if not result.success and verbose:
        print(f"Warning: SLSQP did not fully converge: {result.message}")

    v_opt = np.clip(result.x, [lo for lo, _ in bounds], [hi for _, hi in bounds])
    sim   = simulate(v_opt, segments, weather, vehicle, race)

    return OptimizerResult(
        v_opt=v_opt,
        sim=sim,
        scipy_result=result,
        seed=seed,
    )
