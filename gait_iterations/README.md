# Gait Iterations

Each version of the gait controller as we tuned the bot.

| File | What changed | Speed | Stability |
|------|--------------|-------|-----------|
| `gait_v1_initial.py` | First attempt — ±35° yaw, 300ms steps | ~0.3 cm/s | Toppled often |
| `gait_v2_stability.py` | Reduced yaw to ±25°, slowed to 600ms, more pitch lift | ~0.4 cm/s | Less wobble |
| `gait_v3_crabwalk.py` | Lowered stance, added crab walk, fixed backward toppling | ~0.6 cm/s | Stable forward, mild backward issues |
| `gait_v4_smooth.py` | Smoother movements, safe shutdown procedure | ~0.6 cm/s | Stable |
| `../crab_gait.py` (final) | Added actuator load feedback for ground contact detection, CSV logging | ~0.7 cm/s | Stable |

## Running an Iteration

Each script is standalone — just run it like any other:

```bash
source ~/robotenv/bin/activate
python3 gait_v3_crabwalk.py
```

## Why Keep Old Versions?

Documenting iterations is part of the portfolio (Point 8 — Learning Curve).
Each file shows a concrete tuning step, what was changed, and what improved.
