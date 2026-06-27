"""
Race-day scheduling helpers: per-segment arrival times, overnight-stop
locations, and solar energy collected at each stop.

Used by main.py to place weather queries and overnight constraints by day.
"""

from __future__ import annotations
import math
from datetime import datetime, timedelta

from python.params import VehicleParams
from python.route import RouteSegment

# ASC 2026 tour hours (Reg 12.10.A.1, nominal).
RACE_START_HOUR = 9    # 09:00 local
RACE_STOP_HOUR  = 18   # 18:00 local


def compute_arrival_times(
    segments: list[RouteSegment],
    race_start: datetime,
    v_estimate: float,
    start_hour: float = RACE_START_HOUR,
    stop_hour: float = RACE_STOP_HOUR,
) -> list[datetime]:
    """
    Estimated arrival datetime at each segment, accounting for overnight stops.

    When an arrival crosses stop_hour, the clock jumps to start_hour the next
    morning — modelling a multi-day race where the car parks each evening.

    A constant v_estimate breaks the chicken-and-egg dependency (arrivals depend
    on the speed profile we're about to optimize). A 20% speed error shifts a day
    boundary by ~100 km, which is fine for placing constraints and weather times.
    """
    arrivals = []
    current = race_start
    for seg in segments:
        if current.hour + current.minute / 60.0 >= stop_hour:
            current = (current + timedelta(days=1)).replace(
                hour=int(start_hour), minute=0, second=0, microsecond=0
            )
        arrivals.append(current)
        current += timedelta(seconds=seg.distance_m / v_estimate)
    return arrivals


def find_day_boundaries(arrivals: list[datetime]) -> list[int]:
    """Index of the last segment driven on each day except the last — the overnight-stop locations."""
    return [i for i in range(len(arrivals) - 1) if arrivals[i + 1].date() != arrivals[i].date()]


def overnight_charge_Wh(
    vehicle: VehicleParams,
    peak_ghi: float = 900.0,
    impound_start_h: float = 20.0,
    impound_end_h: float = 7.0,
    restart_h: float = RACE_START_HOUR,
    day_end_h: float = RACE_STOP_HOUR,
) -> float:
    """
    Solar energy (Wh) collected at an overnight stop during non-impound hours.

    ASC 2026 impound is 20:00–07:00 (Reg 12.17.B.1), leaving two charging windows:
    day-end→impound (18:00–20:00) and impound-end→restart (07:00–09:00). Uses the
    same sine GHI model as synthetic_weather().
    """
    def ghi(h: float) -> float:
        f = h / 24.0
        return peak_ghi * math.sin(math.pi * (f - 0.25) / 0.5) if 0.25 <= f <= 0.75 else 0.0

    def integrate(t0: float, t1: float, n: int = 60) -> float:
        dt = (t1 - t0) / n
        s = sum(ghi(t0 + k * dt) for k in range(n + 1)) - 0.5 * (ghi(t0) + ghi(t1))
        return s * dt  # Wh/m²

    Wh_per_m2 = integrate(day_end_h, impound_start_h) + integrate(impound_end_h, restart_h)
    return Wh_per_m2 * vehicle.Ai * vehicle.eta_s * (vehicle.eta_b ** 0.5)
