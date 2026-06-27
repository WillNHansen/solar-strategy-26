"""
GPX route parsing → list of RouteSegments.

Each segment is a straight-line approximation between two GPX waypoints,
with precomputed distance, grade, heading, and speed limit.

Usage:
    segments = load_gpx("route.gpx")
    segments = smooth_grade(segments, window=5)
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Optional
import gpxpy
import gpxpy.gpx
import numpy as np


@dataclass
class RouteSegment:
    index: int
    lat: float          # midpoint latitude (degrees)
    lon: float          # midpoint longitude (degrees)
    distance_m: float   # segment length (m)
    grade: float        # sin(θ) — positive = uphill, negative = downhill
    heading_deg: float  # road heading 0–360° (north = 0, east = 90)
    speed_limit_ms: float = math.inf  # m/s; inf where no limit applies


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two WGS84 points."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from point 1 to point 2 (degrees, 0 = north)."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def load_gpx(path: str, segment_length_m: float = 500.0) -> list[RouteSegment]:
    """
    Parse a GPX file into fixed-length RouteSegments.

    GPX waypoints are first linearised into a dense point list, then
    resampled into segments of approximately `segment_length_m` metres.
    This gives the optimizer a uniform resolution regardless of GPX
    waypoint density.

    Elevation from GPX is used for grade. If GPX lacks elevation data,
    grade is set to 0 (flat); substitute SRTM3 data in that case.
    """
    with open(path) as f:
        gpx = gpxpy.parse(f)

    # Flatten all tracks/segments into a single ordered point list
    points: list[tuple[float, float, Optional[float]]] = []
    for track in gpx.tracks:
        for seg in track.segments:
            for pt in seg.points:
                points.append((pt.latitude, pt.longitude, pt.elevation))
    if not points:
        for wpt in gpx.waypoints:
            points.append((wpt.latitude, wpt.longitude, wpt.elevation))

    if len(points) < 2:
        raise ValueError(f"GPX file {path!r} contains fewer than 2 points")

    # Resample into fixed-length segments
    segments: list[RouteSegment] = []
    seg_idx = 0

    # Walk the point list, accumulating distance and elevation
    cum_dist = 0.0
    seg_start_lat, seg_start_lon = points[0][0], points[0][1]
    seg_start_ele = points[0][2] or 0.0

    for i in range(1, len(points)):
        lat1, lon1, ele1 = points[i - 1]
        lat2, lon2, ele2 = points[i]
        ele1 = ele1 or 0.0
        ele2 = ele2 or 0.0

        d = _haversine_m(lat1, lon1, lat2, lon2)
        cum_dist += d

        if cum_dist >= segment_length_m or i == len(points) - 1:
            mid_lat = (seg_start_lat + lat2) / 2
            mid_lon = (seg_start_lon + lon2) / 2
            ele_end = ele2
            elevation_change = ele_end - seg_start_ele
            horizontal = max(cum_dist, 1.0)
            grade = elevation_change / horizontal  # sin(θ) ≈ tan(θ) for small angles

            heading = _bearing_deg(seg_start_lat, seg_start_lon, lat2, lon2)

            segments.append(RouteSegment(
                index=seg_idx,
                lat=mid_lat,
                lon=mid_lon,
                distance_m=cum_dist,
                grade=grade,
                heading_deg=heading,
            ))
            seg_idx += 1
            cum_dist = 0.0
            seg_start_lat, seg_start_lon = lat2, lon2
            seg_start_ele = ele_end

    return segments


def smooth_grade(segments: list[RouteSegment], window: int = 5) -> list[RouteSegment]:
    """
    Apply a moving-average smoothing to segment grades.

    Raw GPX elevation data has GPS noise that produces unrealistic grade spikes.
    Smoothing before feeding into the optimizer prevents it from planning
    wildly varying speeds in response to noise rather than real hills.
    """
    grades = np.array([s.grade for s in segments])
    kernel = np.ones(window) / window
    smoothed = np.convolve(grades, kernel, mode="same")
    for s, g in zip(segments, smoothed):
        s.grade = float(g)
    return segments


def total_distance_km(segments: list[RouteSegment]) -> float:
    return sum(s.distance_m for s in segments) / 1000.0
