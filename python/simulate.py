"""
Forward simulation: given a velocity profile v[], compute the SoC trajectory,
segment times, and cumulative race time.

Two paths:
  - RouteArrays + energy_deltas_vec()  — fast numpy path used by the optimizer
  - simulate()                         — full simulation with clipping, used for
                                         final result and diagnostics

Why two paths? The optimizer calls the inner loop thousands of times. Vectorising
the energy computation with numpy makes each call ~100× faster than a Python loop.
The trade-off: the fast path omits battery clipping (trusting constraints to keep
SoC in bounds), which is an exact match when the optimizer is working correctly.
The final simulate() applies real clipping for the result you actually report.
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from strategy.python.params import VehicleParams, RaceParams
from strategy.python.route import RouteSegment
from strategy.python.weather import SegmentWeather
from strategy.python.physics import energy_delta_Wh


# ── Precomputed route + weather arrays ────────────────────────────────────────

@dataclass
class RouteArrays:
    """
    Numpy arrays pre-extracted from segments + weather.
    Build once with make_arrays(); pass to energy_deltas_vec() on every
    optimizer iteration instead of iterating Python lists each time.
    """
    d:           np.ndarray   # segment distances (m)
    grade:       np.ndarray   # sin(θ), positive = uphill
    heading:     np.ndarray   # road heading (degrees)
    speed_limit: np.ndarray   # m/s (inf where no limit)
    GHI:         np.ndarray   # W/m²
    wind_speed:  np.ndarray   # m/s
    wind_dir:    np.ndarray   # degrees (meteorological)
    N:           int


def make_arrays(segments: list[RouteSegment], weather: list[SegmentWeather]) -> RouteArrays:
    return RouteArrays(
        d           = np.array([s.distance_m    for s in segments]),
        grade       = np.array([s.grade         for s in segments]),
        heading     = np.array([s.heading_deg   for s in segments]),
        speed_limit = np.array([s.speed_limit_ms for s in segments]),
        GHI         = np.array([wx.GHI          for wx in weather]),
        wind_speed  = np.array([wx.wind_speed   for wx in weather]),
        wind_dir    = np.array([wx.wind_dir     for wx in weather]),
        N           = len(segments),
    )


# ── Fast vectorised forward pass (optimizer hot path) ─────────────────────────

def energy_deltas_vec(v: np.ndarray, arrays: RouteArrays, vehicle: VehicleParams) -> np.ndarray:
    """
    Compute energy delta (Wh) for every segment simultaneously using numpy.

    No battery clipping — the optimizer's SoC constraints keep energy in bounds,
    so clipping never fires in a feasible solution. Unclipped cumsum is then
    differentiable everywhere, which helps SLSQP converge cleanly.

    Includes kinetic energy transitions: when speed changes between segments,
    the battery pays for acceleration (or recovers via regen on deceleration).
    This correctly penalises speed changes in addition to the steady-state
    aero and rolling resistance terms.
    """
    # Wind projection onto road axis
    relative = np.radians(arrays.wind_dir - arrays.heading)
    vw = arrays.wind_speed * np.cos(relative)    # headwind component (m/s)

    F_aero  = 0.5 * vehicle.rho * vehicle.CdA_flat * (v - vw) ** 2
    F_roll  = vehicle.Crr * vehicle.m * vehicle.g                   # scalar, broadcasts
    F_grade = vehicle.m * vehicle.g * arrays.grade

    Pm = v * (F_aero + F_roll + F_grade)   # motor shaft power (W); negative = regen

    # Battery power: positive = discharging, negative = charging
    Pb = np.where(Pm >= 0, Pm / vehicle.eta_m, Pm * vehicle.eta_regen)
    Ps = arrays.GHI * vehicle.Ai * vehicle.eta_s   # solar input (W)

    net_W  = Ps - Pb
    t_h    = arrays.d / v / 3600.0

    eta_sqrt = np.sqrt(vehicle.eta_b)
    dE = np.where(net_W >= 0,
                  net_W * t_h * eta_sqrt,    # charging: lose energy going in
                  net_W * t_h / eta_sqrt)    # discharging: lose energy coming out

    # Kinetic energy transitions between segments.
    # dKE_Wh[i] = ½m(v[i]² - v[i-1]²) / 3600  (Wh; positive = speeding up)
    # Acceleration draws from battery through motor; deceleration recovers via regen.
    dKE_Wh = np.zeros(arrays.N)
    dKE_Wh[1:] = 0.5 * vehicle.m * (v[1:] ** 2 - v[:-1] ** 2) / 3600.0
    # Use a smooth blend near zero to avoid gradient discontinuity at the
    # acceleration/deceleration boundary, which causes SLSQP linesearch failures.
    _kin_scale = 0.1  # Wh — blend width (small relative to typical dKE)
    _alpha = 0.5 * (1.0 + np.tanh(dKE_Wh / _kin_scale))  # 0=full regen, 1=full motor
    _eff_kin = _alpha / vehicle.eta_m / eta_sqrt + (1 - _alpha) * vehicle.eta_regen * eta_sqrt
    dE_kin = -dKE_Wh * _eff_kin

    return dE + dE_kin


def cumulative_energy_vec(
    v: np.ndarray,
    arrays: RouteArrays,
    vehicle: VehicleParams,
    race: RaceParams,
) -> np.ndarray:
    """
    Battery energy (Wh) at end of each segment — no clipping.
    O(N) numpy cumsum; this is what the optimizer's constraint functions call.

    Overnight charges are constant offsets (zero gradient w.r.t. v), so they
    shift the Eb trajectory without affecting constraint Jacobians in optimize.py.
    """
    dE = energy_deltas_vec(v, arrays, vehicle)
    Eb = race.Eb_start + np.cumsum(dE)
    for idx, charge in zip(race.overnight_segment_indices, race.overnight_charge_Wh):
        if idx + 1 < arrays.N:
            Eb[idx + 1:] += charge
    return Eb


def cumulative_time_vec(v: np.ndarray, arrays: RouteArrays) -> np.ndarray:
    """Cumulative race time (s) at end of each segment."""
    return np.cumsum(arrays.d / v)


def energy_deltas_grad_vec(
    v: np.ndarray, arrays: RouteArrays, vehicle: VehicleParams
) -> tuple[np.ndarray, np.ndarray]:
    """
    Analytical gradient of energy_deltas w.r.t. v.

    Returns (diag, sub) where:
      diag[i] = d(dE[i])/d(v[i])    — diagonal of the Jacobian
      sub[i]  = d(dE[i])/d(v[i-1]) — subdiagonal (kinetic term); sub[0] = 0

    With kinetic energy transitions, dE[i] depends on both v[i] and v[i-1],
    making the Jacobian lower-bidiagonal rather than purely diagonal.

    Constraint Jacobians use:
        d(Eb[k])/d(v[j]) = diag[j]   (if j ≤ k)
                         + sub[j+1]  (if j+1 ≤ k, i.e. j ≤ k-1)

    Steady-state derivation (Pm ≥ 0 case):
        Pm  = v * (F_aero + F_roll + F_grade)
        dPm/dv = (F_aero + F_roll + F_grade) + rho*CdA*v*(v-vw)
        Pb  = Pm / eta_m   →   dPb/dv = dPm/dv / eta_m
        net = Ps - Pb      →   dnet/dv = -dPb/dv
        t_h = d / (v*3600) →   dt_h/dv = -d / (v²*3600)
        dE  = net * t_h * eta_factor
        d(dE)/dv = eta_factor * d/3600/v * (dnet/dv - net/v)
    """
    N = arrays.N
    relative = np.radians(arrays.wind_dir - arrays.heading)
    vw = arrays.wind_speed * np.cos(relative)

    F_aero  = 0.5 * vehicle.rho * vehicle.CdA_flat * (v - vw) ** 2
    F_roll  = vehicle.Crr * vehicle.m * vehicle.g
    F_grade = vehicle.m * vehicle.g * arrays.grade

    Pm = v * (F_aero + F_roll + F_grade)

    dPm_dv = (F_aero + F_roll + F_grade) + vehicle.rho * vehicle.CdA_flat * v * (v - vw)
    dPb_dv = np.where(Pm >= 0, dPm_dv / vehicle.eta_m, dPm_dv * vehicle.eta_regen)

    Ps    = arrays.GHI * vehicle.Ai * vehicle.eta_s
    Pb    = np.where(Pm >= 0, Pm / vehicle.eta_m, Pm * vehicle.eta_regen)
    net_W = Ps - Pb

    eta_sqrt   = np.sqrt(vehicle.eta_b)
    eta_factor = np.where(net_W >= 0, eta_sqrt, 1.0 / eta_sqrt)
    dnet_dv    = -dPb_dv

    # Steady-state diagonal
    ss_diag = eta_factor * arrays.d / 3600.0 / v * (dnet_dv - net_W / v)

    # Kinetic energy gradient — use the same smooth blend as energy_deltas_vec.
    dKE_Wh = np.zeros(N)
    dKE_Wh[1:] = 0.5 * vehicle.m * (v[1:] ** 2 - v[:-1] ** 2) / 3600.0
    _kin_scale = 0.1
    _alpha     = 0.5 * (1.0 + np.tanh(dKE_Wh / _kin_scale))
    _eff_kin   = _alpha / vehicle.eta_m / eta_sqrt + (1 - _alpha) * vehicle.eta_regen * eta_sqrt
    # d(dE_kin[i])/d(v[i]): dKE_Wh[i] = ½m(v[i]²-v[i-1]²)/3600 → d/dv[i] = m*v[i]/3600
    kin_diag      = -_eff_kin * vehicle.m * v / 3600.0
    kin_diag[0]   = 0.0
    # d(dE_kin[i])/d(v[i-1]): d/dv[i-1] = -m*v[i-1]/3600
    sub        = np.zeros(N)
    sub[1:]    = _eff_kin[1:] * vehicle.m * v[:-1] / 3600.0

    return ss_diag + kin_diag, sub


# ── Full simulation (final result, diagnostics) ───────────────────────────────

@dataclass
class SimResult:
    v:            np.ndarray
    Eb:           np.ndarray   # Wh at end of each segment (with clipping)
    t_seg:        np.ndarray   # s per segment
    t_cum:        np.ndarray   # s cumulative
    total_time_s: float
    feasible:     bool
    min_Eb_Wh:    float


def simulate(
    v: np.ndarray,
    segments: list[RouteSegment],
    weather: list[SegmentWeather],
    vehicle: VehicleParams,
    race: RaceParams,
) -> SimResult:
    """
    Full forward simulation with battery clipping.

    Used for the final result and mid-race re-simulation — not in the optimizer
    hot path. Clipping correctly handles the rare case where regen into a full
    battery or discharge below zero would otherwise violate physics.
    """
    N = len(segments)
    Eb    = np.empty(N)
    t_seg = np.empty(N)
    t_cum = np.empty(N)

    overnight_charge_map = {
        idx + 1: charge
        for idx, charge in zip(race.overnight_segment_indices, race.overnight_charge_Wh)
        if idx + 1 < N
    }

    energy  = race.Eb_start
    elapsed = 0.0
    v_prev  = float(v[0])

    for i, (seg, wx) in enumerate(zip(segments, weather)):
        if i in overnight_charge_map:
            energy = float(np.clip(energy + overnight_charge_map[i], 0.0, vehicle.Eb_max))
        vi = float(np.clip(v[i], vehicle.v_min, seg.speed_limit_ms))

        dE = energy_delta_Wh(vi, seg, wx, vehicle)

        # Kinetic energy cost of speed change from previous segment
        if i > 0:
            eta_sqrt = vehicle.eta_b ** 0.5
            dKE_Wh = 0.5 * vehicle.m * (vi ** 2 - v_prev ** 2) / 3600.0
            _alpha = 0.5 * (1.0 + np.tanh(dKE_Wh / 0.1))
            _eff   = _alpha / vehicle.eta_m / eta_sqrt + (1 - _alpha) * vehicle.eta_regen * eta_sqrt
            dE    += -dKE_Wh * _eff

        energy = float(np.clip(energy + dE, 0.0, vehicle.Eb_max))
        v_prev = vi
        ti     = seg.distance_m / vi
        elapsed += ti

        Eb[i]    = energy
        t_seg[i] = ti
        t_cum[i] = elapsed

    eps = 1.0  # 1 Wh tolerance for floating-point drift between unclipped and clipped paths
    feasible = bool(
        np.all(Eb >= vehicle.Eb_min - eps)
        and Eb[-1] >= race.Eb_finish_min - eps
        and _checkpoints_ok(t_cum, Eb, race)
    )

    return SimResult(
        v=v, Eb=Eb, t_seg=t_seg, t_cum=t_cum,
        total_time_s=elapsed,
        feasible=feasible,
        min_Eb_Wh=float(np.min(Eb)),
    )


def _checkpoints_ok(t_cum: np.ndarray, Eb: np.ndarray, race: RaceParams) -> bool:
    for cp in race.checkpoints:
        idx = cp.segment_index
        if idx >= len(t_cum):
            continue
        if not (cp.t_open_s <= t_cum[idx] <= cp.t_close_s):
            return False
        if Eb[idx] < cp.Eb_min_Wh:
            return False
    return True
