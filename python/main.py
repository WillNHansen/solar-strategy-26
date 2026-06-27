"""
Pre-race optimizer entry point.

Usage:
    python main.py --gpx route.gpx --start "2025-10-13 08:00" [--solcast-key KEY]

Without --solcast-key, synthetic weather is used (good for development).
"""

from __future__ import annotations
import argparse
import json
import math
from datetime import datetime, timedelta, timezone

import numpy as np

from strategy.python.params import VehicleParams, RaceParams, Checkpoint
from strategy.python.route import load_gpx, smooth_grade, total_distance_km, RouteSegment
from strategy.python.weather import fetch_solcast, synthetic_weather, SegmentWeather
from strategy.python.optimize import run_optimizer
from strategy.python.simulate import simulate

# ASC 2026 tour hours (Reg 12.10.A.1 — nominal; adjusted per-team based on stage start time)
RACE_START_HOUR = 9    # 09:00 local
RACE_STOP_HOUR  = 18   # 18:00 local
UTC_OFFSET_H    = 0.0  # set per-race; ASC route crosses multiple US time zones


def _compute_arrival_times(
    segments: list[RouteSegment],
    race_start: datetime,
    v_estimate: float,
    race_start_hour: float = RACE_START_HOUR,
    race_stop_hour: float  = RACE_STOP_HOUR,
) -> list[datetime]:
    """
    Estimated arrival time at each segment, accounting for overnight stops.

    When the estimated arrival crosses race_stop_hour, the clock jumps to
    race_start_hour the following morning before continuing. This correctly
    models a multi-day race where the car parks each evening.
    """
    arrivals = []
    current  = race_start

    for seg in segments:
        hour = current.hour + current.minute / 60.0
        if hour >= race_stop_hour:
            current = (current + timedelta(days=1)).replace(
                hour=int(race_start_hour), minute=0, second=0, microsecond=0
            )
        arrivals.append(current)
        current = current + timedelta(seconds=seg.distance_m / v_estimate)

    return arrivals


def _overnight_charge_Wh(
    vehicle: VehicleParams,
    peak_ghi: float = 900.0,
    impound_start_h: float = 20.0,
    impound_end_h: float   = 7.0,
    race_restart_h: float  = RACE_START_HOUR,
    end_of_day_h: float    = RACE_STOP_HOUR,
) -> float:
    """
    Estimate solar energy (Wh) collected at an overnight stop during non-impound hours.

    ASC 2026 impound: 20:00–07:00 (Reg 12.17.B.1).
    Charging windows: end-of-day (18:00) → impound (20:00), plus impound-end (07:00) → race restart (09:00).
    Uses the same synthetic sin-curve GHI model as synthetic_weather().
    """
    def ghi_at_hour(h: float) -> float:
        hour_frac = h / 24.0
        if 0.25 <= hour_frac <= 0.75:
            return peak_ghi * math.sin(math.pi * (hour_frac - 0.25) / 0.5)
        return 0.0

    def integrate_ghi(t_start: float, t_end: float, n: int = 60) -> float:
        """Trapezoid integration of GHI over a time window (hours), returns Wh/m²."""
        dt = (t_end - t_start) / n
        total = sum(ghi_at_hour(t_start + k * dt) for k in range(n + 1))
        total -= 0.5 * (ghi_at_hour(t_start) + ghi_at_hour(t_end))  # trapezoid correction
        return total * dt

    Wh_per_m2  = integrate_ghi(end_of_day_h, impound_start_h)   # evening window
    Wh_per_m2 += integrate_ghi(impound_end_h, race_restart_h)   # morning window

    return Wh_per_m2 * vehicle.Ai * vehicle.eta_s * (vehicle.eta_b ** 0.5)


def _find_day_boundaries(arrivals: list[datetime]) -> list[int]:
    """
    Return the index of the last segment driven on each day (except the final day).
    Used to place overnight SoC minimum constraints.
    """
    boundaries = []
    for i in range(len(arrivals) - 1):
        if arrivals[i + 1].date() != arrivals[i].date():
            boundaries.append(i)
    return boundaries


def main() -> None:
    parser = argparse.ArgumentParser(description="SSCP pre-race velocity optimizer")
    parser.add_argument("--gpx",         required=True,  help="Path to route GPX file")
    parser.add_argument("--start",       required=True,
                        help="Race start datetime (UTC), e.g. '2025-10-13 08:00'")
    parser.add_argument("--solcast-key", default=None,
                        help="Solcast API key (omit for synthetic weather)")
    parser.add_argument("--segment-m",   type=float, default=2000.0,
                        help="Optimizer segment length in metres (default 2000). "
                             "Larger = faster: scipy SLSQP scales O(N²) internally. "
                             "500m ~2 min, 2000m ~7s for a 3000km route.")
    parser.add_argument("--smooth",      type=int, default=5,
                        help="Grade smoothing window (default 5)")
    parser.add_argument("--max-iter",    type=int, default=2000,
                        help="SLSQP iteration limit (default 2000)")
    parser.add_argument("--verbose",     action="store_true")
    parser.add_argument("--output",      default=None,
                        help="Write result JSON to this path")
    args = parser.parse_args()

    # Treat --start as local race time (Darwin, UTC+9:30 for WSC).
    # All internal time calculations are relative offsets from this point,
    # so the absolute timezone doesn't matter — only the local hour does.
    race_start = datetime.fromisoformat(args.start)

    # ── Load route ────────────────────────────────────────────────────────────
    print(f"Loading route from {args.gpx}...")
    segments = load_gpx(args.gpx, segment_length_m=args.segment_m)
    segments = smooth_grade(segments, window=args.smooth)
    print(f"  {len(segments)} segments, {total_distance_km(segments):.1f} km total")

    # ── Vehicle parameters ────────────────────────────────────────────────────
    vehicle = VehicleParams()

    # ── Compute multi-day arrival times ──────────────────────────────────────
    # Arrival times must be known before the optimizer runs so that weather()
    # assigns the correct local hour (GHI) to each segment, and so we can
    # locate the overnight SoC constraint at the right segment index.
    #
    # Chicken-and-egg: arrivals depend on the speed profile we're about to
    # optimize. We break the cycle with a constant-speed estimate — a 20%
    # error shifts a day boundary by ~10–20 segments (~100 km), which is
    # acceptable imprecision for constraint placement.
    initial_speed  = 22.0  # m/s — rough seed; only affects day-boundary placement
    arrivals       = _compute_arrival_times(segments, race_start, initial_speed)
    day_boundaries = _find_day_boundaries(arrivals)
    n_days = len(day_boundaries) + 1
    print(f"  {n_days} race days, day-end boundaries at segments: {day_boundaries}")

    # ── Weather ───────────────────────────────────────────────────────────────
    if args.solcast_key:
        print("Fetching Solcast weather forecasts...")
        weather = fetch_solcast(segments, race_start, initial_speed, args.solcast_key)
    else:
        print("Using synthetic weather (no Solcast key provided)")
        # Pass pre-computed arrivals so GHI reflects the correct local time per day
        weather = synthetic_weather(segments, race_start, initial_speed,
                                    arrival_times=arrivals)

    charge_per_night = _overnight_charge_Wh(vehicle)
    print(f"  Overnight solar charge: {charge_per_night:.0f} Wh per stop")

    race = RaceParams(
        Eb_start=vehicle.Eb_max,
        Eb_finish_min=250.0,
        overnight_segment_indices=day_boundaries,
        Eb_overnight_min=500.0,
        overnight_charge_Wh=[charge_per_night] * len(day_boundaries),
        # TODO: add checkpoints from race rulebook, e.g.:
        # checkpoints=[
        #     Checkpoint("Glendambo", segment_index=180, t_open_s=8*3600, t_close_s=16*3600),
        # ],
    )

    # ── Optimize ──────────────────────────────────────────────────────────────
    print(f"Running SLSQP optimizer ({len(segments)} segments, {n_days} days)...")
    result = run_optimizer(
        segments, weather, vehicle, race,
        max_iter=args.max_iter,
        verbose=args.verbose,
    )

    sim      = simulate(result.v_opt, segments, weather, vehicle, race)
    seed_sim = simulate(result.seed,  segments, weather, vehicle, race)

    # ── Report ────────────────────────────────────────────────────────────────
    # Compute driving-hours only (exclude night segments at v_min)
    print()
    print("=" * 50)
    print("OPTIMIZATION RESULT")
    print("=" * 50)
    print(f"  Converged:        {result.scipy_result.success}")
    print(f"  Iterations:       {result.scipy_result.nit}")
    print()
    print(f"  Seed (v*):        {seed_sim.total_time_s / 3600:.2f} h driving")
    print(f"  Optimized:        {sim.total_time_s / 3600:.2f} h driving")
    print(f"  Time saved:       {(seed_sim.total_time_s - sim.total_time_s) / 60:.1f} min")
    print()
    print(f"  Speed range:      {result.v_opt.min() * 3.6:.1f} – "
          f"{result.v_opt.max() * 3.6:.1f} km/h")
    print(f"  Mean speed:       {result.v_opt.mean() * 3.6:.1f} km/h")
    print()
    print(f"  Min battery:      {sim.min_Eb_Wh:.0f} Wh  (floor: {vehicle.Eb_min:.0f} Wh)")
    print(f"  Finish battery:   {sim.Eb[-1]:.0f} Wh")
    print(f"  Feasible:         {sim.feasible}")
    print("=" * 50)

    if not result.scipy_result.success:
        print(f"\nWarning: {result.scipy_result.message}")

    # ── Optional output ───────────────────────────────────────────────────────
    if args.output:
        out = {
            "total_time_h":    sim.total_time_s / 3600,
            "feasible":        sim.feasible,
            "segments": [
                {
                    "index":      seg.index,
                    "lat":        seg.lat,
                    "lon":        seg.lon,
                    "distance_m": seg.distance_m,
                    "grade":      seg.grade,
                    "day_of_race": next((d for d, b in enumerate(day_boundaries) if i <= b), n_days - 1) + 1,
                    "v_opt_kmh":  float(result.v_opt[i] * 3.6),
                    "Eb_Wh":      float(sim.Eb[i]),
                    "t_cum_s":    float(sim.t_cum[i]),
                }
                for i, seg in enumerate(segments)
            ],
        }
        with open(args.output, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nResult written to {args.output}")


if __name__ == "__main__":
    main()
