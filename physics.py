"""
Physics model: power equations, battery update, critical speed seed.

All quantities in SI units (m, s, W, Wh, kg) unless noted.
"""

from __future__ import annotations
import math
from scipy.optimize import brentq

from params import VehicleParams
from route import RouteSegment
from weather import SegmentWeather


def headwind_ms(wind_speed: float, wind_dir_deg: float, road_heading_deg: float) -> float:
    """
    Headwind component (m/s) along the road heading.

    Positive = wind in face (opposing motion).
    Negative = tailwind (assisting motion).

    wind_dir_deg: meteorological convention — direction FROM which wind blows.
    road_heading_deg: direction the car is travelling.
    """
    # Angle between wind source and road direction of travel
    relative = math.radians(wind_dir_deg - road_heading_deg)
    return wind_speed * math.cos(relative)


def yaw_angle_deg(wind_speed: float, wind_dir_deg: float, road_heading_deg: float) -> float:
    """
    Crosswind yaw angle (degrees) for CdA(β) lookup.

    β is the angle between the wind vector and the car's axis of travel.
    """
    relative = math.radians(wind_dir_deg - road_heading_deg)
    return abs(math.degrees(math.sin(relative) * wind_speed))


def motor_power_W(
    v: float,
    seg: RouteSegment,
    wx: SegmentWeather,
    p: VehicleParams,
) -> float:
    """
    Net mechanical power at the motor shaft (W).

    Positive → motor is driving (consuming battery power).
    Negative → car is on a descent and regen is available.

    The acceleration term (m·a) is omitted — the optimizer holds constant speed
    within each segment. Speed changes between segments are handled separately
    as kinetic energy transitions in energy_deltas_vec().
    """
    vw = headwind_ms(wx.wind_speed, wx.wind_dir, seg.heading_deg)
    beta = yaw_angle_deg(wx.wind_speed, wx.wind_dir, seg.heading_deg)

    F_aero  = 0.5 * p.rho * p.CdA(beta) * (v - vw) ** 2
    F_roll  = p.Crr * p.m * p.g
    F_grade = p.m * p.g * seg.grade      # sin(θ), positive = uphill

    return v * (F_aero + F_roll + F_grade)


def battery_power_W(Pm: float, p: VehicleParams) -> float:
    """
    Net power flow into/out of the battery (W) given motor shaft power Pm.

    Positive Pm → driving: battery discharges at Pm / eta_m.
    Negative Pm → descent: motor regenerates at |Pm| * eta_regen back to battery.
    Returns battery discharge power (positive = discharging, negative = charging).
    """
    if Pm >= 0:
        return Pm / p.eta_m
    else:
        return Pm * p.eta_regen   # Pm negative → this is negative (charging)


def solar_power_W(wx: SegmentWeather, p: VehicleParams) -> float:
    """
    Solar array output (W).

    GHI from Solcast already incorporates sun elevation angle, so no sin(φ)
    multiplication is needed. If your Solcast tier provides DNI instead of GHI,
    you must convert: GHI = DNI * sin(elevation) + DHI.
    """
    return wx.GHI * p.Ai * p.eta_s


def energy_delta_Wh(
    v: float,
    seg: RouteSegment,
    wx: SegmentWeather,
    p: VehicleParams,
) -> float:
    """
    Change in battery energy (Wh) for one segment at speed v.

    Positive → net charging (solar surplus).
    Negative → net discharging.

    Battery round-trip efficiency eta_b is applied asymmetrically:
      charging: multiply by sqrt(eta_b)   (energy lost going in)
      discharging: divide by sqrt(eta_b)  (energy lost coming out)
    This correctly models the round-trip loss without double-counting.
    """
    t_s = seg.distance_m / v          # time in segment (seconds)
    t_h = t_s / 3600.0                # convert to hours for Wh

    Pm = motor_power_W(v, seg, wx, p)
    Ps = solar_power_W(wx, p)
    Pb = battery_power_W(Pm, p)       # positive = draining battery

    net_W = Ps - Pb                   # positive = net into battery

    eta_sqrt = math.sqrt(p.eta_b)
    if net_W >= 0:
        return net_W * t_h * eta_sqrt
    else:
        return net_W * t_h / eta_sqrt


def max_speed_from_discharge_limit(
    seg: RouteSegment,
    wx: SegmentWeather,
    p: VehicleParams,
) -> float:
    """
    Maximum physically achievable speed on this segment given the BMS discharge
    current limit. Returns p.v_max if the limit is not binding.

    The optimizer respects speed limits from race rules; this enforces the
    additional constraint that the BMS cannot supply more than I_discharge_max
    amps regardless of what the optimizer requests.
    """
    P_max = p.I_discharge_max * p.V_pack_nominal * p.eta_m  # max motor shaft power

    def net_power(v: float) -> float:
        Pm = motor_power_W(v, seg, wx, p)
        # Solar offsets some demand; only the deficit draws from battery
        Ps = solar_power_W(wx, p)
        return max(0.0, Pm - Ps) - P_max

    if net_power(p.v_max) <= 0:
        return p.v_max  # limit not binding at any legal speed

    try:
        return brentq(net_power, p.v_min, p.v_max)
    except ValueError:
        return p.v_min


def critical_speed_ms(
    seg: RouteSegment,
    wx: SegmentWeather,
    p: VehicleParams,
) -> float:
    """
    Equilibrium speed where solar power exactly covers motor demand (W).

    This is the Pudney critical speed v* — the speed the car should hold when
    running at energy balance. Used as the initial guess (seed) for SLSQP.

    If solar power exceeds demand at v_max, return v_max (speed-limited).
    If solar power is less than demand at v_min, return v_min (need battery).
    """
    def surplus(v: float) -> float:
        Pm = motor_power_W(v, seg, wx, p)
        Ps = solar_power_W(wx, p)
        return Ps - Pm / p.eta_m

    low = surplus(p.v_min)
    high = surplus(p.v_max)

    if low < 0 and high < 0:
        return p.v_min   # solar never covers demand — drive at minimum

    if low > 0 and high > 0:
        return p.v_max   # solar always surplus — drive at maximum (speed-limited)

    return brentq(surplus, p.v_min, p.v_max)
