"""
CrabBot Gait — Iteration 3 (Lower Stance + Crab Walk)
Changes from v2:
- Lowered stance neutrals (FL 65->80, FR 195.8->180, BL 155.8->170, BR 101->86)
- Reduced yaw 25 -> 15 deg, faster 600 -> 350ms
- Added crab walk (sideways) gait
- Added backward step damping (60% of forward yaw)
- Bot stopped toppling backwards

Run: python3 gait_v3_crabwalk.py
"""

import time
import sys
from pylx16a.lx16a import LX16A, ServoTimeoutError

SERIAL_PORT = "/dev/ttyUSB0"

NEUTRAL = {
    "FL_PITCH": 80.0,
    "FL_YAW":   120.0,
    "FR_YAW":   120.0,
    "FR_PITCH": 180.0,
    "BL_PITCH": 170.0,
    "BL_YAW":   120.0,
    "BR_YAW":   120.0,
    "BR_PITCH": 86.0,
}

ID = {
    "FL_PITCH": 1, "FL_YAW": 2, "FR_YAW": 3, "FR_PITCH": 4,
    "BL_PITCH": 5, "BL_YAW": 6, "BR_YAW": 7, "BR_PITCH": 8,
}

YAW_STEP   = 15
PITCH_LIFT = 15

PITCH_DIR = {"FL_PITCH": +1, "FR_PITCH": -1, "BL_PITCH": -1, "BR_PITCH": +1}
PAIR_A = ["FL", "BR"]
PAIR_B = ["FR", "BL"]

servos = {}

def clamp(val): return max(0, min(240, val))

def move(name, angle, ms=350):
    try:
        servos[name].move(clamp(angle), time=ms)
    except ServoTimeoutError:
        pass

def move_leg(leg, yaw_offset, lift, ms=350):
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

def trot_step(direction=1, ms=350):
    yaw = (YAW_STEP if direction == 1 else YAW_STEP * 0.6) * direction
    for leg in PAIR_A: move_leg(leg, +yaw, lift=True,  ms=ms)
    for leg in PAIR_B: move_leg(leg, -yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)
    for leg in PAIR_A: move_leg(leg, +yaw, lift=False, ms=ms)
    for leg in PAIR_B: move_leg(leg, -yaw, lift=True,  ms=ms)
    time.sleep(ms / 1000 + 0.05)
    for leg in PAIR_A: move_leg(leg, -yaw * 0.5, lift=False, ms=ms)
    for leg in PAIR_B: move_leg(leg, +yaw * 0.5, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)

def crab_step(direction=1, ms=350):
    yaw = YAW_STEP * direction
    for leg in ["FL", "BL"]: move_leg(leg, +yaw, lift=True,  ms=ms)
    for leg in ["FR", "BR"]: move_leg(leg, +yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)
    for leg in ["FL", "BL"]: move_leg(leg, +yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)
    for leg in ["FR", "BR"]: move_leg(leg, -yaw, lift=True,  ms=ms)
    for leg in ["FL", "BL"]: move_leg(leg, -yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)
    for leg in ["FR", "BR"]: move_leg(leg, -yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)

def main():
    LX16A.initialize(SERIAL_PORT, 0.1)
    for name, mid in ID.items():
        try: servos[name] = LX16A(mid)
        except ServoTimeoutError: pass

    all_neutral()
    print("Commands: w=fwd  s=back  a=crab L  d=crab R  q=quit")
    while True:
        cmd = input(">> ").strip().lower()
        if cmd == "w":
            for _ in range(3): trot_step(direction=1)
            all_neutral()
        elif cmd == "s":
            for _ in range(3): trot_step(direction=-1)
            all_neutral()
        elif cmd == "a":
            for _ in range(3): crab_step(direction=-1)
            all_neutral()
        elif cmd == "d":
            for _ in range(3): crab_step(direction=1)
            all_neutral()
        elif cmd == "q":
            all_neutral()
            break

if __name__ == "__main__":
    main()
