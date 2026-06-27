"""
Pre-race optimizer entry point.

Usage:
    python -m strategy.python.main --gpx route.gpx --start "2026-07-13 09:00" [--solcast-key KEY]

Without --solcast-key, synthetic weather is used (good for development).
"""

from __future__ import annotations
import argparse
import json
from datetime import datetime

from strategy.python.params import VehicleParams, RaceParams
from strategy.python.route import load_gpx, smooth_grade, total_distance_km
from strategy.python.weather import fetch_solcast, synthetic_weather
from strategy.python.optimize import run_optimizer
from strategy.python.simulate import simulate
from strategy.python.schedule import (
    compute_arrival_times, find_day_boundaries, overnight_charge_Wh,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="SSCP pre-race velocity optimizer")
    parser.add_argument("--gpx",         required=True,  help="Path to route GPX file")
    parser.add_argument("--start",       required=True,
                        help="Race start datetime (local), e.g. '2026-07-13 09:00'")
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
    # Arrivals must be known before optimizing so weather() assigns the correct
    # local hour (GHI) per segment and overnight constraints land on the right
    # segment. Estimated from a constant speed (see compute_arrival_times).
    initial_speed  = 22.0  # m/s — only affects day-boundary placement
    arrivals       = compute_arrival_times(segments, race_start, initial_speed)
    day_boundaries = find_day_boundaries(arrivals)
    n_days = len(day_boundaries) + 1
    print(f"  {n_days} race days, day-end boundaries at segments: {day_boundaries}")

    # ── Weather ───────────────────────────────────────────────────────────────
    if args.solcast_key:
        print("Fetching Solcast weather forecasts...")
        weather = fetch_solcast(segments, race_start, initial_speed,
                                args.solcast_key, arrival_times=arrivals)
    else:
        print("Using synthetic weather (no Solcast key provided)")
        weather = synthetic_weather(segments, race_start, initial_speed,
                                    arrival_times=arrivals)

    charge_per_night = overnight_charge_Wh(vehicle)
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
