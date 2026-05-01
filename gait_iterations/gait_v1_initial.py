"""
CrabBot Gait — Iteration 1 (Initial)
- 35 deg yaw swing
- 300ms step time
- 10 deg pitch lift
- No feedback, no shutdown procedure
- Bot was unstable and toppled often

Run: python3 gait_v1_initial.py
"""

import time
import sys
from pylx16a.lx16a import LX16A, ServoTimeoutError

SERIAL_PORT = "/dev/ttyUSB0"

NEUTRAL = {
    "FL_PITCH": 65.0,
    "FL_YAW":   120.0,
    "FR_YAW":   120.0,
    "FR_PITCH": 195.8,
    "BL_PITCH": 155.8,
    "BL_YAW":   120.0,
    "BR_YAW":   120.0,
    "BR_PITCH": 101.0,
}

ID = {
    "FL_PITCH": 1, "FL_YAW": 2, "FR_YAW": 3, "FR_PITCH": 4,
    "BL_PITCH": 5, "BL_YAW": 6, "BR_YAW": 7, "BR_PITCH": 8,
}

YAW_STEP   = 35
PITCH_LIFT = 10

PITCH_DIR = {"FL_PITCH": +1, "FR_PITCH": -1, "BL_PITCH": -1, "BR_PITCH": +1}
PAIR_A = ["FL", "BR"]
PAIR_B = ["FR", "BL"]

servos = {}

def clamp(val): return max(0, min(240, val))

def move(name, angle, ms=300):
    try:
        servos[name].move(clamp(angle), time=ms)
    except ServoTimeoutError:
        pass

def move_leg(leg, yaw_offset, lift, ms=300):
    yaw_name   = f"{leg}_YAW"
    pitch_name = f"{leg}_PITCH"
    yaw_angle   = NEUTRAL[yaw_name] + yaw_offset
    pitch_angle = NEUTRAL[pitch_name] + (PITCH_DIR[pitch_name] * PITCH_LIFT if lift else 0)
    move(yaw_name, yaw_angle, ms)
    move(pitch_name, pitch_angle, ms)

def all_neutral(ms=500):
    for name, angle in NEUTRAL.items():
        move(name, angle, ms)
    time.sleep(ms / 1000 + 0.1)

def trot_step(direction=1, ms=300):
    yaw = YAW_STEP * direction
    for leg in PAIR_A: move_leg(leg, +yaw, lift=True,  ms=ms)
    for leg in PAIR_B: move_leg(leg, -yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)
    for leg in PAIR_A: move_leg(leg, +yaw, lift=False, ms=ms)
    for leg in PAIR_B: move_leg(leg, -yaw, lift=True,  ms=ms)
    time.sleep(ms / 1000 + 0.05)
    for leg in PAIR_A: move_leg(leg, -yaw, lift=False, ms=ms)
    for leg in PAIR_B: move_leg(leg, +yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)

def main():
    LX16A.initialize(SERIAL_PORT, 0.1)
    for name, mid in ID.items():
        try: servos[name] = LX16A(mid)
        except ServoTimeoutError: pass

    all_neutral()
    print("Walking forward (5 steps)...")
    for _ in range(5):
        trot_step(direction=1)
    all_neutral()

if __name__ == "__main__":
    main()
