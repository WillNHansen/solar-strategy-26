"""
Weather data: GHI irradiance and wind per route segment.

In production, call fetch_solcast() which hits the Solcast API for each
segment's lat/lon at the expected arrival time. For development or offline
use, call synthetic_weather() which generates physically plausible test data.

Solcast returns GHI (Global Horizontal Irradiance) directly, which already
has the sun angle baked in — so no pvlib / sin(φ) calculation is needed.
If your Solcast tier only provides DNI, see the note in research_notes.md.
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Optional
import requests

from strategy.python.route import RouteSegment


@dataclass
class SegmentWeather:
    GHI: float          # W/m² — solar irradiance on a horizontal surface
    wind_speed: float   # m/s at 10m height
    wind_dir: float     # degrees (meteorological: 0 = wind from north)


def fetch_solcast(
    segments: list[RouteSegment],
    race_start: datetime,
    initial_speed_ms: float,
    api_key: str,
    arrival_times: Optional[list[datetime]] = None,
) -> list[SegmentWeather]:
    """
    Fetch GHI and wind forecasts from Solcast for each segment.

    Args:
        segments:          Route segment list from route.load_gpx()
        race_start:        UTC datetime of race start
        initial_speed_ms:  Average speed for naive arrival estimation (m/s), used
                           only when arrival_times is None.
        api_key:           Solcast API key.
        arrival_times:     Pre-computed per-segment arrivals accounting for overnight
                           stops (from schedule.compute_arrival_times). Strongly
                           preferred for multi-day races; the naive fallback ignores
                           overnight stops and queries the wrong time of day.

    Returns:
        One SegmentWeather per segment, in order.

    TODO: Solcast rate-limits free-tier accounts. Cache responses to disk and/or
          fetch at coarser spatial resolution and interpolate, rather than one
          call per segment.
    """
    weather: list[SegmentWeather] = []
    elapsed_s = 0.0

    for i, seg in enumerate(segments):
        arrival = arrival_times[i] if arrival_times is not None \
            else race_start + timedelta(seconds=elapsed_s)
        arrival_utc = arrival.astimezone(timezone.utc)

        # Solcast radiation + weather endpoint
        url = "https://api.solcast.com.au/radiation/forecasts"
        params = {
            "latitude":  round(seg.lat, 6),
            "longitude": round(seg.lon, 6),
            "hours":     1,
            "period":    "PT30M",
            "format":    "json",
            "api_key":   api_key,
        }
        resp = requests.get(url, params=params, timeout=10)
        resp.raise_for_status()
        data = resp.json()

        # Take the forecast period closest to expected arrival time
        forecasts = data.get("forecasts", [])
        if not forecasts:
            raise ValueError(f"No Solcast forecast for segment {seg.index} at {arrival_utc}")

        closest = min(
            forecasts,
            key=lambda f: abs(
                datetime.fromisoformat(f["period_end"].replace("Z", "+00:00")) - arrival_utc
            ),
        )

        weather.append(SegmentWeather(
            GHI=float(closest.get("ghi", 0.0)),
            wind_speed=float(closest.get("wind_speed_10m", 0.0)),
            wind_dir=float(closest.get("wind_direction_10m", 0.0)),
        ))

        elapsed_s += seg.distance_m / initial_speed_ms

    return weather


def synthetic_weather(
    segments: list[RouteSegment],
    race_start: datetime,
    initial_speed_ms: float,
    peak_ghi: float = 900.0,
    wind_speed: float = 5.0,
    wind_dir: float = 270.0,
    arrival_times: Optional[list[datetime]] = None,
) -> list[SegmentWeather]:
    """
    Generate synthetic weather for development and testing.

    GHI follows a solar day curve (sine, peaks at solar noon).
    Wind is constant. No cloud cover modeled.

    arrival_times: pre-computed per-segment arrival datetimes accounting for
    overnight stops. If None, computed naively from constant speed (ignores
    overnight stops — incorrect for multi-day races).
    """
    result: list[SegmentWeather] = []
    elapsed_s = 0.0

    for i, seg in enumerate(segments):
        if arrival_times is not None:
            arrival = arrival_times[i]
        else:
            arrival = race_start + timedelta(seconds=elapsed_s)

        hour_frac = (arrival.hour + arrival.minute / 60) / 24.0
        sun_angle = math.pi * (hour_frac - 0.25) / 0.5
        ghi = max(0.0, peak_ghi * math.sin(sun_angle)) if 0.25 <= hour_frac <= 0.75 else 0.0

        result.append(SegmentWeather(
            GHI=ghi,
            wind_speed=wind_speed,
            wind_dir=wind_dir,
        ))
        elapsed_s += seg.distance_m / initial_speed_ms

    return result
