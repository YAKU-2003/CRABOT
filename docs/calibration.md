# 🔧 Calibration Guide

## Overview

Calibration sets the neutral (resting) position for all 8 motors so the bot stands level and has equal range of motion in both directions.

---

## Step 1 — Connect Hardware

1. Connect BusLinker to Pi via USB
2. Verify serial port: `ls /dev/ttyUSB*` → should show `/dev/ttyUSB0`
3. Connect 12V battery to BusLinker
4. Activate virtual environment: `source ~/robotenv/bin/activate`

---

## Step 2 — Run Motor Setup Tool

```bash
python3 scripts/motor_setup.py
```

Press **6** for the full guided workflow:
1. Torque turns off automatically
2. Physically pose the bot into a natural standing position
3. Press ENTER to read all positions
4. Save to `bot.env`

---

## Step 3 — Level the Bot

If the bot is leaning to one side:

```bash
python3 scripts/level.py
```

- Type `<motor_id> <angle>` to move individual motors
- Type `all` to read all current positions
- Focus on pitch motors (IDs 1, 4, 5, 8) to adjust height

**Pitch motor directions (mirrored mounting):**
| Motor | To raise leg | To lower leg |
|-------|-------------|-------------|
| FL_PITCH (ID 1) | Increase angle | Decrease angle |
| FR_PITCH (ID 4) | Decrease angle | Increase angle |
| BL_PITCH (ID 5) | Increase angle | Decrease angle |
| BR_PITCH (ID 8) | Decrease angle | Increase angle |

---

## Step 4 — Fix Negative Angles

If any YAW motor reads 0° or negative:

1. The horn is mounted past the servo's zero point
2. Use the setup tool to move the motor to 120°:
```bash
python3 -c "
from pylx16a.lx16a import LX16A
import time
LX16A.initialize('/dev/ttyUSB0', 0.1)
LX16A(6).move(120, time=1000)  # BL_YAW
LX16A(7).move(120, time=1000)  # BR_YAW
time.sleep(1.5)
print('Attach horns now!')
"
```
3. While motor holds at 120°, attach the servo horn and screw it in

---

## Step 5 — Update Neutral Positions

Once the bot stands level, update `NEUTRAL` in `crab_gait.py`:

```python
NEUTRAL = {
    "FL_PITCH": <your_value>,
    "FL_YAW":   120.0,
    "FR_YAW":   120.0,
    "FR_PITCH": <your_value>,
    "BL_PITCH": <your_value>,
    "BL_YAW":   120.0,
    "BR_YAW":   120.0,
    "BR_PITCH": <your_value>,
}
```

---

## Neutral Positions (Current)

| Motor | ID | Neutral (°) | Lift direction |
|-------|----|-------------|---------------|
| FL_PITCH | 1 | 80.0 | +10° |
| FL_YAW | 2 | 120.0 | ±35° |
| FR_YAW | 3 | 120.0 | ±35° |
| FR_PITCH | 4 | 188.0 | -10° |
| BL_PITCH | 5 | 170.0 | -10° |
| BL_YAW | 6 | 120.0 | ±35° |
| BR_YAW | 7 | 120.0 | ±35° |
| BR_PITCH | 8 | 86.0 | +10° |

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Motor reads 0° | Horn past zero point — remount horn |
| Bot leans left/right | Adjust pitch neutrals with `level.py` |
| Motor TIMEOUT | Check power and daisy chain connection |
| Horn hits body | Move one spline at a time away from body |
