"""
Vehicle and race parameters.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Callable, Optional


@dataclass
class VehicleParams:
    # ── Physical constants ────────────────────────────────────────────────────
    rho: float = 1.225      # air density kg/m³ (sea level, 15°C; varies ~1% per 1000m)
    g:   float = 9.81       # gravity m/s²

    # ── Mass ─────────────────────────────────────────────────────────────────
    m: float = 290        # TODO: weigh car + driver + full battery pack (kg)

    # ── Aerodynamics ─────────────────────────────────────────────────────────
    # From aero model. If a full CdA(β) yaw curve is available, pass it as
    # CdA_yaw and it will be used automatically; otherwise CdA_flat is used.
    CdA_flat: float = 0.20                          # drag area at zero yaw (m²) — mid-tier student team estimate (Cd≈0.15, A≈1.3m²)
                                                    # TODO: replace with value from SSCP aero model
    CdA_yaw: Optional[Callable[[float], float]] = None  # beta_deg → CdA (m²)

    def CdA(self, beta_deg: float) -> float:
        """Effective drag area at yaw angle beta (degrees)."""
        if self.CdA_yaw is not None:
            return self.CdA_yaw(beta_deg)
        return self.CdA_flat

    # ── Rolling resistance ────────────────────────────────────────────────────
    Crr: float = 0.005      # TODO: rolldown test or early-race telemetry fit — conservative estimate

    # ── Solar array ──────────────────────────────────────────────────────────
    Ai:    float = 4.0      # TODO: confirm effective panel area (m²)
    eta_s: float = 0.20     # TODO: panel + MPPT efficiency (confirm from datasheet)
                            # Midnight Sun uses 0.16; high-end GaAs cells reach 0.30+

    # ── Drivetrain ───────────────────────────────────────────────────────────
    # v1: constant motor efficiency. v2: replace with 2D (torque × RPM) lookup.
    eta_m: float = 0.97     # TODO: source from WaveSculptor datasheet or bench test

    # Round-trip efficiency of kinetic/potential → electrical → stored (regen)
    eta_regen: float = 0.65  # TODO: source from WaveSculptor datasheet or descent test
                              # Literature range: 0.60–0.75

    # ── Battery ──────────────────────────────────────────────────────────────
    Eb_max: float = 5180.0  # TODO: confirm usable capacity (Wh)
    Eb_min: float = 250.0   # 5% floor; model error means running to true zero is unsafe

    # Battery charging/discharging round-trip efficiency (charge at √η, discharge at √η)
    eta_b: float = 0.95     # TODO: fit from cell cycler or telemetry

    # BMS discharge current limit (A). Optimizer enforces this so it never plans
    # a speed that requires more current than the pack can deliver (e.g. on steep climbs).
    I_discharge_max: float = 150.0   # TODO: confirm from BMS config
    V_pack_nominal:  float = 100.0   # TODO: confirm nominal pack voltage (V)

    # ── Speed bounds — ASC 2026 (Regs §12.21–12.22) ─────────────────────────
    v_min: float = 8.94  # m/s = 20 mph — minimum in zones with posted limit ≥ 60 mph
    v_max: float = 29.06 # m/s = 65 mph — ASC maximum (or posted limit if lower)


@dataclass
class ModelFeatures:
    """
    Toggle individual physics terms on/off for sensitivity analysis.

    Each flag zeroes the corresponding arrays before they enter the optimizer,
    so the equations themselves are unchanged — a disabled term simply has no
    magnitude. All flags default to True (full model).
    """
    headwind: bool = True   # wind projection onto road axis (v_w term in aero drag)
    grade:    bool = True   # road grade force (m·g·sin θ)
    rolling:  bool = True   # rolling resistance (C_rr·m·g)
    kinetic:  bool = True   # kinetic energy transitions between segments (½mv² term)
    solar:    bool = True   # solar power income (GHI·Ai·η_s)
    regen:    bool = True   # energy recovery on descents (η_regen path)
    aero:     bool = True   # aerodynamic drag (½ρ·CdA·(v−v_w)²)


@dataclass
class Checkpoint:
    """A mandatory checkpoint the car must pass through within a time window."""
    name: str
    segment_index: int   # index into the route segment list
    t_open_s: float      # seconds from race start when checkpoint opens
    t_close_s: float     # seconds from race start when checkpoint closes
    Eb_min_Wh: float = 0.0  # optional minimum SoC required at this checkpoint


@dataclass
class RaceParams:
    Eb_start: float = 5000.0    # battery energy at race start (Wh) — usually full

    # Minimum energy at finish line. Ensures the car crosses under its own power
    # and provides a buffer against model error accumulating over the full route.
    Eb_finish_min: float = 250.0  # TODO: check race rules for finish-line requirements

    checkpoints: list[Checkpoint] = field(default_factory=list)

    # Overnight stops (multi-day races). Solar charging stops; car must arrive
    # with enough energy to reach cruising speed before solar ramps up next morning.
    overnight_segment_indices: list[int] = field(default_factory=list)
    Eb_overnight_min: float = 500.0  # TODO: determine from next-day route analysis

    # Energy (Wh) collected by solar panels during non-impound hours at each overnight stop.
    # One value per overnight stop, matching overnight_segment_indices.
    # Computed in main.py from the non-impound charging windows (18:00–20:00, 07:00–09:00).
    # Zero gradient w.r.t. v — constraint Jacobians are unaffected.
    overnight_charge_Wh: list[float] = field(default_factory=list)
