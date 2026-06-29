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

from python.params import VehicleParams, RaceParams, ModelFeatures
from python.route import RouteSegment
from python.weather import SegmentWeather
from python.physics import critical_speed_ms, max_speed_from_discharge_limit
from python.simulate import (
    RouteArrays, make_arrays,
    cumulative_energy_vec, cumulative_time_vec,
    energy_deltas_grad_vec,
    simulate, SimResult,
)


@dataclass
class OptimizerResult:
    v_opt:         np.ndarray
    sim:           SimResult
    seed:          np.ndarray
    seed_sim:      SimResult
    scipy_result:  OptimizeResult


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


# ── Main entry point ──────────────────────────────────────────────────────────

def _apply_features(
    arrays: RouteArrays, vehicle: VehicleParams, features: ModelFeatures
) -> tuple[RouteArrays, VehicleParams]:
    """
    Return (arrays, vehicle) with disabled terms zeroed out.
    Copies are made so the caller's originals are never mutated.
    """
    import copy, dataclasses
    arrays = copy.copy(arrays)
    veh_kw = {}

    if not features.headwind:
        arrays.wind_speed = np.zeros(arrays.N)
    if not features.grade:
        arrays.grade = np.zeros(arrays.N)
    if not features.rolling:
        veh_kw["Crr"] = 0.0
    if not features.kinetic:
        arrays.skip_kinetic = True
    if not features.solar:
        arrays.GHI = np.zeros(arrays.N)
    if not features.regen:
        arrays.skip_regen = True
    if not features.aero:
        veh_kw["CdA_flat"] = 0.0

    vehicle = dataclasses.replace(vehicle, **veh_kw) if veh_kw else vehicle
    return arrays, vehicle


def run_optimizer(
    segments: list[RouteSegment],
    weather: list[SegmentWeather],
    vehicle: VehicleParams,
    race: RaceParams,
    features: ModelFeatures = None,
    max_iter: int = 1000,
    tol: float = 1e-6,
    verbose: bool = False,
) -> OptimizerResult:
    if features is None:
        features = ModelFeatures()
    arrays, vehicle = _apply_features(make_arrays(segments, weather), vehicle, features)
    bounds      = _build_bounds(segments, weather, vehicle)
    constraints = _build_constraints(arrays, vehicle, race)
    seed        = _seed_v_star(segments, weather, vehicle, bounds)

    seed_sim = simulate(seed, segments, weather, vehicle, race, arrays=arrays)

    if verbose:
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

    # status 8 = "Positive directional derivative for linesearch" — SLSQP is stuck
    # at a constraint boundary, usually because the problem is infeasible.
    if not result.success and verbose:
        print(f"Warning: SLSQP did not fully converge: {result.message}")

    v_opt = np.clip(result.x, [lo for lo, _ in bounds], [hi for _, hi in bounds])
    sim   = simulate(v_opt, segments, weather, vehicle, race, arrays=arrays)

    return OptimizerResult(
        v_opt=v_opt,
        sim=sim,
        seed=seed,
        seed_sim=seed_sim,
        scipy_result=result,
    )
