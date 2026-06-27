"""
Multi-seed parallel optimizer experiment.

Runs SLSQP from 10 starting points simultaneously:
  - Seed 0:   v* (Pudney critical speed — current baseline)
  - Seeds 1–9: constant speeds evenly spaced from v_min to v_max

Writes results to multi_seed_results.json for plotting.
"""

from __future__ import annotations
import argparse
import json
import math
import time
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

from strategy.python.params import VehicleParams, RaceParams
from strategy.python.route import load_gpx, smooth_grade, total_distance_km
from strategy.python.weather import synthetic_weather
from strategy.python.optimize import (
    run_multi_seed, make_constant_seed, _build_bounds, _seed_v_star,
)

RACE_START_HOUR = 9
RACE_STOP_HOUR  = 18


def _compute_arrival_times(segments, race_start, v_estimate):
    arrivals = []
    current  = race_start
    for seg in segments:
        hour = current.hour + current.minute / 60.0
        if hour >= RACE_STOP_HOUR:
            current = (current + timedelta(days=1)).replace(
                hour=RACE_START_HOUR, minute=0, second=0, microsecond=0
            )
        arrivals.append(current)
        current = current + timedelta(seconds=seg.distance_m / v_estimate)
    return arrivals


def _find_day_boundaries(arrivals):
    return [i for i in range(len(arrivals) - 1) if arrivals[i+1].date() != arrivals[i].date()]


def _overnight_charge_Wh(vehicle, peak_ghi=900.0):
    def ghi(h):
        f = h / 24.0
        return peak_ghi * math.sin(math.pi * (f - 0.25) / 0.5) if 0.25 <= f <= 0.75 else 0.0

    def integrate(t0, t1, n=60):
        dt = (t1 - t0) / n
        s = sum(ghi(t0 + k * dt) for k in range(n + 1))
        s -= 0.5 * (ghi(t0) + ghi(t1))
        return s * dt

    Wh_m2 = integrate(18, 20) + integrate(7, 9)
    return Wh_m2 * vehicle.Ai * vehicle.eta_s * (vehicle.eta_b ** 0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gpx",       required=True)
    parser.add_argument("--start",     required=True)
    parser.add_argument("--segment-m", type=float, default=5000.0)
    parser.add_argument("--n-seeds",   type=int,   default=10)
    parser.add_argument("--max-iter",  type=int,   default=2000)
    parser.add_argument("--output",    default=str(Path(__file__).resolve().parents[1] / "results" / "multi_seed_results.json"))
    args = parser.parse_args()

    race_start = datetime.fromisoformat(args.start)

    print(f"Loading route from {args.gpx}...")
    segments = load_gpx(args.gpx, segment_length_m=args.segment_m)
    segments = smooth_grade(segments)
    print(f"  {len(segments)} segments, {total_distance_km(segments):.1f} km total")

    vehicle = VehicleParams()
    arrivals       = _compute_arrival_times(segments, race_start, 22.0)
    day_boundaries = _find_day_boundaries(arrivals)
    n_days         = len(day_boundaries) + 1
    print(f"  {n_days} race days, boundaries at segments: {day_boundaries}")

    weather = synthetic_weather(segments, race_start, 22.0, arrival_times=arrivals)

    charge = _overnight_charge_Wh(vehicle)
    race   = RaceParams(
        Eb_start=vehicle.Eb_max,
        Eb_finish_min=250.0,
        overnight_segment_indices=day_boundaries,
        Eb_overnight_min=500.0,
        overnight_charge_Wh=[charge] * len(day_boundaries),
    )

    bounds   = _build_bounds(segments, weather, vehicle)
    v_star   = _seed_v_star(segments, weather, vehicle, bounds)
    v_lo     = vehicle.v_min
    v_hi     = vehicle.v_max

    # 10 seeds: v* plus evenly spaced constant speeds
    constant_speeds = np.linspace(v_lo, v_hi, args.n_seeds - 1)
    seeds = [v_star] + [make_constant_seed(s, bounds) for s in constant_speeds]
    labels = ["v* (Pudney)"] + [f"{s*3.6:.1f} km/h constant" for s in constant_speeds]

    print(f"\nRunning {len(seeds)} seeds in parallel (max_iter={args.max_iter})...")
    t0      = time.perf_counter()
    results = run_multi_seed(segments, weather, vehicle, race, seeds, max_iter=args.max_iter)
    elapsed = time.perf_counter() - t0
    print(f"Done in {elapsed:.1f}s\n")

    # ── Report ────────────────────────────────────────────────────────────────
    print(f"{'Seed':<25} {'Time (h)':>8} {'Feasible':>9} {'Min Eb':>8} {'Converged':>10}")
    print("-" * 65)
    for label, r in zip(labels, results):
        print(f"{label:<25} {r.sim.total_time_s/3600:>8.2f} {str(r.sim.feasible):>9} "
              f"{r.sim.min_Eb_Wh:>8.0f} {str(r.scipy_result.success):>10}")

    # ── Serialise ─────────────────────────────────────────────────────────────
    cum = 0
    dists = []
    for seg in segments:
        cum += seg.distance_m / 1000
        dists.append(round(cum, 3))

    out = {
        "dists_km":      dists,
        "day_boundaries": day_boundaries,
        "overnight_charge_Wh": charge,
        "runs": [
            {
                "label":       label,
                "total_time_h": r.sim.total_time_s / 3600,
                "feasible":    r.sim.feasible,
                "converged":   r.scipy_result.success,
                "min_Eb_Wh":  float(r.sim.min_Eb_Wh),
                "v_kmh":      [round(float(vi) * 3.6, 2) for vi in r.v_opt],
                "Eb_Wh":      [round(float(e), 1) for e in r.sim.Eb],
            }
            for label, r in zip(labels, results)
        ],
    }
    with open(args.output, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nResults written to {args.output}")


if __name__ == "__main__":
    main()
