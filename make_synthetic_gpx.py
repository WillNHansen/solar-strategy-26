"""
Generate a synthetic 2484 km GPX file approximating the WSC route
Darwin → Adelaide, with a realistic elevation profile.

Elevation profile (approximate, from north to south):
  Darwin area:       ~30m, flat
  Stuart Highway:    ~200m, gently rolling
  Tennant Creek:     ~380m
  Alice Springs:     ~550m (highest point of route)
  Coober Pedy:       ~250m
  Port Augusta area: ~50m
  Adelaide:          ~50m

Usage:
    python make_synthetic_gpx.py --output wsc_synthetic.gpx
"""

import argparse
import math


# Reference points along the Stuart Highway (lat, lon, elevation_m, cum_dist_km)
# Approximate; good enough for optimizer testing
WAYPOINTS = [
    (-12.46, 130.84,  30,    0),     # Darwin
    (-13.82, 131.83,  80,  200),     # Pine Creek area
    (-14.47, 132.26, 120,  285),     # Katherine
    (-16.50, 133.40, 210,  530),     # Daly Waters
    (-19.65, 134.19, 380,  900),     # Tennant Creek
    (-21.14, 134.22, 450, 1090),     # Wauchope
    (-23.70, 133.87, 550, 1350),     # Alice Springs
    (-25.24, 133.43, 420, 1530),     # Erldunda
    (-26.27, 133.18, 350, 1640),     # Marla
    (-29.00, 134.75, 250, 1940),     # Coober Pedy
    (-31.11, 136.46, 150, 2180),     # Pimba / Woomera
    (-32.50, 137.76,  60, 2350),     # Port Augusta
    (-34.93, 138.60,  50, 2484),     # Adelaide
]


def _haversine_km(lat1, lon1, lat2, lon2):
    R = 6371.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2)**2
    return 2 * R * math.asin(math.sqrt(a))


def _interpolate(waypoints, n_points):
    """
    Linearly interpolate lat, lon, elevation along the cumulative distance axis.
    Uses the waypoints' stated cumulative distances (road distances) for
    interpolation, then places points at equal spacing along that axis.
    The resulting lat/lon positions are straight-line approximations between
    waypoints — good enough for a synthetic test route.
    Returns list of (lat, lon, ele) tuples.
    """
    total_km = waypoints[-1][3]
    step_km  = total_km / (n_points - 1)

    result = []
    for i in range(n_points):
        d = i * step_km

        for j in range(len(waypoints) - 1):
            d0, d1 = waypoints[j][3], waypoints[j + 1][3]
            if d0 <= d <= d1:
                t = (d - d0) / (d1 - d0) if d1 > d0 else 0.0
                lat = waypoints[j][0] + t * (waypoints[j + 1][0] - waypoints[j][0])
                lon = waypoints[j][1] + t * (waypoints[j + 1][1] - waypoints[j][1])
                ele = waypoints[j][2] + t * (waypoints[j + 1][2] - waypoints[j][2])
                result.append((lat, lon, ele))
                break

    return result


def _add_terrain_noise(points, amplitude_m=15.0, wavelength_km=25.0):
    """
    Add low-frequency sinusoidal noise to elevation to simulate rolling terrain.
    Amplitude and wavelength are tunable.
    """
    total_km = WAYPOINTS[-1][3]
    out = []
    for i, (lat, lon, ele) in enumerate(points):
        d_km = i / (len(points) - 1) * total_km
        noise = amplitude_m * math.sin(2 * math.pi * d_km / wavelength_km)
        out.append((lat, lon, max(0.0, ele + noise)))
    return out


def _scale_to_target(points, target_km):
    """
    Scale lon coordinates so the haversine total distance equals target_km.
    The straight-line interpolation between waypoints produces a slightly
    different haversine total than the road distances we parameterised on.
    Scaling lon stretches/compresses the route east-west to hit the target.
    """
    # Measure current haversine total
    actual_km = sum(
        _haversine_km(points[i][0], points[i][1], points[i+1][0], points[i+1][1])
        for i in range(len(points) - 1)
    )
    if actual_km < 1.0:
        return points
    scale = target_km / actual_km
    lat0, lon0 = points[0][0], points[0][1]
    scaled = []
    for lat, lon, ele in points:
        lon_new = lon0 + (lon - lon0) * scale
        lat_new = lat0 + (lat - lat0) * scale
        scaled.append((lat_new, lon_new, ele))
    return scaled


def write_gpx(points, path):
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<gpx version="1.1" creator="make_synthetic_gpx.py"',
        '     xmlns="http://www.topografix.com/GPX/1/1">',
        '  <trk>',
        '    <name>WSC Synthetic Route — Darwin to Adelaide</name>',
        '    <trkseg>',
    ]
    for lat, lon, ele in points:
        lines.append(
            f'      <trkpt lat="{lat:.6f}" lon="{lon:.6f}">'
            f'<ele>{ele:.1f}</ele></trkpt>'
        )
    lines += [
        '    </trkseg>',
        '  </trk>',
        '</gpx>',
    ]
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"Written {len(points)} waypoints to {path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output",     default="wsc_synthetic.gpx")
    parser.add_argument("--n-points",   type=int, default=5000,
                        help="Number of GPX waypoints (default 5000 → ~500m spacing)")
    parser.add_argument("--noise",      type=float, default=15.0,
                        help="Terrain noise amplitude in metres (default 15)")
    parser.add_argument("--wavelength", type=float, default=25.0,
                        help="Terrain noise wavelength in km (default 25)")
    args = parser.parse_args()

    total_km = WAYPOINTS[-1][3]
    points = _interpolate(WAYPOINTS, args.n_points)
    points = _add_terrain_noise(points, args.noise, args.wavelength)
    points = _scale_to_target(points, total_km)
    write_gpx(points, args.output)

    actual_km = sum(
        _haversine_km(points[i][0], points[i][1], points[i+1][0], points[i+1][1])
        for i in range(len(points) - 1)
    )
    print(f"Route: Darwin → Adelaide")
    print(f"Target: {total_km} km  |  Actual haversine: {actual_km:.1f} km")
    print(f"Elevation range: {min(e for _,_,e in points):.0f}m – {max(e for _,_,e in points):.0f}m")


if __name__ == "__main__":
    main()
