# solar-strategy-26

Pre-race velocity optimizer and race simulation for SSCP's 2026 solar car campaign.

Given a GPX route and a race start time, the optimizer finds the speed profile that minimizes total race time subject to battery, speed limit, and overnight stop constraints. Supports SLSQP (fast, gradient-based), Dynamic Programming (globally optimal energy allocation), and a DP→SLSQP hybrid.

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

All scripts are run as modules from the repo root (`solar-strategy-26/`):

```bash
cd solar-strategy-26
```

## Usage

### Run the optimizer

```bash
python -m strategy.python.main \
  --gpx gpx/Seg1A.gpx \
  --start "2026-07-13 09:00" \
  --output results/plan.json
```

With live Solcast weather (requires API key):

```bash
python -m strategy.python.main \
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
| `--verbose` | False | Print per-segment details |

### Multi-seed convergence experiment

Runs the SLSQP optimizer from 10 different starting points in parallel to test for local optima:

```bash
python -m strategy.python.multi_seed \
  --gpx gpx/Seg1A.gpx \
  --start "2026-07-13 09:00"
```

Output goes to `results/multi_seed_results.json` by default.

| Flag | Default | Description |
|---|---|---|
| `--gpx` | *(required)* | Path to route GPX file |
| `--start` | *(required)* | Race start datetime |
| `--n-seeds` | 10 | Number of starting points |
| `--segment-m` | 5000 | Segment length in metres |
| `--max-iter` | 2000 | SLSQP iteration limit per seed |
| `--output` | `results/multi_seed_results.json` | JSON output path |

### Generate a synthetic route

Creates a synthetic GPX for development and testing when the real route isn't available:

```bash
python -m strategy.python.make_synthetic_gpx
```

Output goes to `gpx/wsc_synthetic.gpx` by default.

| Flag | Default | Description |
|---|---|---|
| `--output` | `gpx/wsc_synthetic.gpx` | Output GPX path |
| `--n-points` | 5000 | Number of waypoints (~500m spacing) |
| `--noise` | 15.0 | Terrain elevation noise amplitude (m) |
| `--wavelength` | 25.0 | Terrain noise wavelength (km) |

## Vehicle and race parameters

Edit `python/params.py` to set vehicle parameters (`m`, `CdA_flat`, `Crr`, `Eb_max`, etc.) and race rules (`Eb_start`, overnight charge amounts, checkpoint constraints). All values are currently placeholders — see `strategy_takeaways.md §3.5` for the measurement plan.
