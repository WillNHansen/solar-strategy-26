"""
Plot optimizer output: velocity profile and battery (SoC) trajectory vs. distance.

Saves PNGs — no interactive display needed. Two entry points:
  - plot_result()      one optimizer run (optionally overlaying its seed)
  - plot_comparison()  several runs on shared axes (e.g. SLSQP vs DP vs hybrid)

Both stack velocity (top) over battery (bottom) sharing a distance x-axis, with
overnight stops marked and the battery floor/ceiling drawn as reference lines.
"""

from __future__ import annotations
from typing import Optional

import numpy as np
import matplotlib
matplotlib.use("Agg")          # file output only; no GUI backend required
import matplotlib.pyplot as plt

from python.params import VehicleParams, RaceParams
from python.route import RouteSegment
from python.optimize import OptimizerResult


def _cumulative_km(segments: list[RouteSegment]) -> np.ndarray:
    """Distance (km) at the end of each segment."""
    return np.cumsum([s.distance_m for s in segments]) / 1000.0


def _mark_overnights(ax, dist_km: np.ndarray, race: RaceParams) -> None:
    """Dashed vertical line at each overnight stop, labelled once for the legend."""
    for n, idx in enumerate(race.overnight_segment_indices):
        if 0 <= idx < len(dist_km):
            ax.axvline(
                dist_km[idx], color="0.6", ls="--", lw=0.8,
                label="overnight stop" if n == 0 else None,
            )


def plot_result(
    segments: list[RouteSegment],
    result: OptimizerResult,
    vehicle: VehicleParams,
    race: RaceParams,
    path: str,
    title: Optional[str] = None,
    show_seed: bool = False,
) -> str:
    """
    Plot a single optimizer run's velocity and battery trajectory, save to `path`.

    Set show_seed=True to overlay the seed speed profile (e.g. Pudney v*) for
    comparison against the optimized result. Returns `path`.
    """
    dist = _cumulative_km(segments)
    fig, (ax_v, ax_b) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))

    # ── Velocity ──────────────────────────────────────────────────────────────
    if show_seed and result.seed is not None:
        ax_v.plot(dist, np.asarray(result.seed) * 3.6, color="0.7", lw=1.0,
                  label="seed (v*)")
    ax_v.plot(dist, result.v_opt * 3.6, color="C0", lw=1.3, label="optimized")
    ax_v.set_ylabel("speed (km/h)")
    ax_v.grid(True, alpha=0.3)
    _mark_overnights(ax_v, dist, race)
    ax_v.legend(loc="upper right", fontsize=8)

    # ── Battery ───────────────────────────────────────────────────────────────
    ax_b.plot(dist, result.sim.Eb, color="C1", lw=1.3, label="battery")
    ax_b.axhline(vehicle.Eb_min, color="r", ls=":", lw=1.0, label=f"floor {vehicle.Eb_min:.0f} Wh")
    ax_b.axhline(vehicle.Eb_max, color="g", ls=":", lw=1.0, label=f"ceiling {vehicle.Eb_max:.0f} Wh")
    ax_b.set_ylabel("battery energy (Wh)")
    ax_b.set_xlabel("distance (km)")
    ax_b.grid(True, alpha=0.3)
    _mark_overnights(ax_b, dist, race)
    ax_b.legend(loc="upper right", fontsize=8)

    sim = result.sim
    head = title or "Race strategy"
    fig.suptitle(
        f"{head} — {sim.total_time_s / 3600:.2f} h driving, "
        f"min SoC {sim.min_Eb_Wh:.0f} Wh, "
        f"feasible={sim.feasible}",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def plot_comparison(
    segments: list[RouteSegment],
    results: list[OptimizerResult],
    labels: list[str],
    vehicle: VehicleParams,
    race: RaceParams,
    path: str,
    title: Optional[str] = None,
) -> str:
    """
    Overlay several optimizer runs on shared velocity + battery axes, save to `path`.

    One colour per run; useful for SLSQP vs DP vs hybrid, or multi-seed spreads.
    Returns `path`.
    """
    dist = _cumulative_km(segments)
    fig, (ax_v, ax_b) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(results)))

    for r, label, c in zip(results, labels, colors):
        hrs = r.sim.total_time_s / 3600.0
        ax_v.plot(dist, r.v_opt * 3.6, color=c, lw=1.1,
                  label=f"{label} ({hrs:.2f} h)")
        ax_b.plot(dist, r.sim.Eb, color=c, lw=1.1, label=label)

    ax_v.set_ylabel("speed (km/h)")
    ax_v.grid(True, alpha=0.3)
    _mark_overnights(ax_v, dist, race)
    ax_v.legend(loc="upper right", fontsize=8)

    ax_b.axhline(vehicle.Eb_min, color="r", ls=":", lw=1.0, label=f"floor {vehicle.Eb_min:.0f} Wh")
    ax_b.axhline(vehicle.Eb_max, color="g", ls=":", lw=1.0, label=f"ceiling {vehicle.Eb_max:.0f} Wh")
    ax_b.set_ylabel("battery energy (Wh)")
    ax_b.set_xlabel("distance (km)")
    ax_b.grid(True, alpha=0.3)
    _mark_overnights(ax_b, dist, race)
    ax_b.legend(loc="upper right", fontsize=8)

    fig.suptitle(title or "Optimizer comparison", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path
