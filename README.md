# solar-strategy-26

Pre-race velocity optimizer and race simulation for SSCP's 2026 solar car campaign.

Given a GPX route and a race start time, the optimizer finds the speed profile that minimizes total race time subject to battery, speed limit, and overnight stop constraints. It uses SLSQP (fast, gradient-based) seeded with the Pudney critical speed v\*, with analytical objective and constraint Jacobians.

## Repo layout

```
gpx/          Route GPX files (real segments + synthetic test routes)
python/       All source code
results/      Generated optimizer outputs (gitignored)
research_notes.md     Literature review and references
strategy_takeaways.md Actionable conclusions, constraint status, open questions
```

## Setup

Requires Python 3.10+.

```bash
pip install -r requirements.txt
```

All scripts run as modules **from the repo root** (the directory containing `python/`, `gpx/`, and `results/`):

```bash
cd solar-strategy-26   # the repo root
python -m python.main ...
```

## Usage

### Run the optimizer

```bash
python -m python.main \
  --gpx gpx/Seg1A.gpx \
  --start "2026-07-13 09:00" \
  --output results/plan.json
```

With live Solcast weather (requires API key):

```bash
python -m python.main \
  --gpx gpx/Seg1A.gpx \
  --start "2026-07-13 09:00" \
  --solcast-key YOUR_KEY \
  --output results/plan.json
```

Without `--solcast-key`, synthetic weather is used — good for development and testing.

| Flag | Default | Description |
|---|---|---|
| `--gpx` | *(required)* | Path to route GPX file |
| `--start` | *(required)* | Race start datetime, `"YYYY-MM-DD HH:MM"` |
| `--solcast-key` | None | Solcast API key; omit to use synthetic weather |
| `--segment-m` | 2000 | Segment length in metres |
| `--smooth` | 5 | Grade smoothing window (segments) |
| `--max-iter` | 2000 | SLSQP iteration limit |
| `--output` | None | JSON output path |
| `--plot` | None | Save a velocity + battery plot (PNG) to this path |
| `--verbose` | False | Print per-segment details |

The `--plot` PNG stacks the optimized speed profile (top) over the battery trajectory (bottom) against distance, with the seed (v\*) overlaid, overnight stops marked, and the battery floor/ceiling drawn in:

```bash
python -m python.main --gpx gpx/Seg1A.gpx --start "2026-07-13 09:00" --plot results/strategy.png
```

### Generate a synthetic route

Creates a synthetic GPX for development and testing when the real route isn't available:

```bash
python -m python.make_synthetic_gpx
```

Output goes to `gpx/wsc_synthetic.gpx` by default.

| Flag | Default | Description |
|---|---|---|
| `--output` | `gpx/wsc_synthetic.gpx` | Output GPX path |
| `--n-points` | 5000 | Number of waypoints (~500m spacing) |
| `--noise` | 15.0 | Terrain elevation noise amplitude (m) |
| `--wavelength` | 25.0 | Terrain noise wavelength (km) |

### Plotting from your own scripts

Beyond the `--plot` flag, `python/plot.py` exposes:

- `plot_result(segments, result, vehicle, race, path, show_seed=True)` — a single run's velocity + battery profile.
- `plot_comparison(segments, results, labels, vehicle, race, path)` — overlay several `OptimizerResult`s on shared axes (e.g. comparing two parameter sets).

```python
from python.optimize import run_optimizer
from python.plot import plot_result

result = run_optimizer(segments, weather, vehicle, race)
plot_result(segments, result, vehicle, race, "results/strategy.png", show_seed=True)
```

## Vehicle and race parameters

Edit `python/params.py` to set vehicle parameters (`m`, `CdA_flat`, `Crr`, `Eb_max`, etc.) and race rules (`Eb_start`, overnight charge amounts, checkpoint constraints). All values are currently placeholders — see `strategy_takeaways.md §3.5` for the measurement plan.
