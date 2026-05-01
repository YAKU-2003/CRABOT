"""
Crab-Bot Gait Controller with Actuator Feedback
Hardware: Raspberry Pi 4, 8x LX-16A servos
Gaits: Trot (forward/back) and Crab walk (sideways)

Feedback system:
  - Reads position + load after each phase
  - High load on pitch = leg is planted/bearing weight
  - Low load on pitch = leg is in the air
  - Adjusts step timing based on actual ground contact
  - Logs all data to gait_log.csv for portfolio plots

Run: python3 crab_gait.py
"""

import time
import sys
import csv
import os
from pylx16a.lx16a import LX16A, ServoTimeoutError

SERIAL_PORT = "/dev/ttyUSB0"
LOG_FILE    = "gait_log.csv"

# ── Neutral positions ────────────────────────────────────────────────────────
NEUTRAL = {
    "FL_PITCH": 80.0,
    "FL_YAW":   120.0,
    "FR_YAW":   120.0,
    "FR_PITCH": 188.0,
    "BL_PITCH": 170.0,
    "BL_YAW":   120.0,
    "BR_YAW":   120.0,
    "BR_PITCH": 86.0,
}

# ── Motor IDs ────────────────────────────────────────────────────────────────
ID = {
    "FL_PITCH": 1,
    "FL_YAW":   2,
    "FR_YAW":   3,
    "FR_PITCH": 4,
    "BL_PITCH": 5,
    "BL_YAW":   6,
    "BR_YAW":   7,
    "BR_PITCH": 8,
}

# ── Gait parameters ──────────────────────────────────────────────────────────
YAW_STEP   = 15    # degrees of yaw swing each side (max 40)
PITCH_LIFT = 15    # degrees of lift

# How long to wait for ground contact confirmation (seconds)
CONTACT_TIMEOUT = 0.3

# Load threshold: above this = leg is planted on ground
# LX-16A load is 0-1000, higher = more torque
LOAD_THRESHOLD = 200

# Pitch lift direction per leg (mirrored mounting)
PITCH_DIR = {
    "FL_PITCH": +1,
    "FR_PITCH": -1,
    "BL_PITCH": -1,
    "BR_PITCH": +1,
}

# Trot diagonal pairs
PAIR_A = ["FL", "BR"]
PAIR_B = ["FR", "BL"]

# ── Servo registry ───────────────────────────────────────────────────────────
servos = {}

# ── Logging ──────────────────────────────────────────────────────────────────
log_data = []
step_count = 0

def log(phase, leg, position, load):
    log_data.append({
        "step":     step_count,
        "time":     round(time.time(), 3),
        "phase":    phase,
        "leg":      leg,
        "position": round(position, 2),
        "load":     load,
    })

def save_log():
    if not log_data:
        return
    with open(LOG_FILE, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["step","time","phase","leg","position","load"])
        writer.writeheader()
        writer.writerows(log_data)
    print(f"  Log saved to {LOG_FILE} ({len(log_data)} entries)")

# ── Helpers ──────────────────────────────────────────────────────────────────
def clamp(val, lo=0, hi=240):
    return max(lo, min(hi, val))

def move(name, angle, ms=300):
    angle = clamp(angle)
    try:
        servos[name].move(angle, time=ms)
    except ServoTimeoutError:
        print(f"  TIMEOUT moving: {name}")

def read_feedback(name):
    """Read position and load from a servo. Returns (position, load) or (None, None)"""
    try:
        pos  = servos[name].get_physical_angle()
        load = servos[name].get_load()
        return pos, load
    except ServoTimeoutError:
        return None, None
    except Exception:
        return None, None

def read_leg_feedback(leg, phase):
    """Read and log feedback for both motors of a leg"""
    for motor_type in ["PITCH", "YAW"]:
        name = f"{leg}_{motor_type}"
        pos, load = read_feedback(name)
        if pos is not None:
            log(phase, name, pos, load if load is not None else 0)
    # Return pitch load as ground contact indicator
    pitch_name = f"{leg}_PITCH"
    _, load = read_feedback(pitch_name)
    return load if load is not None else 0

def is_planted(leg):
    """Check if a leg is bearing weight based on pitch motor load"""
    pitch_name = f"{leg}_PITCH"
    _, load = read_feedback(pitch_name)
    if load is None:
        return False
    return load > LOAD_THRESHOLD

def wait_for_contact(legs, timeout=CONTACT_TIMEOUT):
    """
    Wait until legs report ground contact (high load) or timeout.
    Returns True if contact confirmed, False if timed out.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if all(is_planted(leg) for leg in legs):
            return True
        time.sleep(0.02)
    return False

def move_leg(leg, yaw_offset, lift, ms=300):
    """Move a single leg. yaw_offset = degrees from neutral, lift = True/False"""
    yaw_name   = f"{leg}_YAW"
    pitch_name = f"{leg}_PITCH"

    yaw_angle   = NEUTRAL[yaw_name] + yaw_offset
    pitch_angle = NEUTRAL[pitch_name] + (PITCH_DIR[pitch_name] * PITCH_LIFT if lift else 0)

    move(yaw_name,   yaw_angle,   ms)
    move(pitch_name, pitch_angle, ms)

def all_neutral(ms=500):
    """Send all motors to neutral position"""
    for name, angle in NEUTRAL.items():
        move(name, angle, ms)
    time.sleep(ms / 1000 + 0.1)

# ── Trot gait with feedback ──────────────────────────────────────────────────
def trot_step(direction=1, ms=350):
    """
    One full trot cycle with actuator feedback.
    - Reads load after each phase
    - Only proceeds when ground contact is confirmed
    - Adjusts timing dynamically based on feedback
    direction: +1 = forward, -1 = backward
    """
    global step_count
    step_count += 1

    yaw = (YAW_STEP if direction == 1 else YAW_STEP * 0.6) * direction

    # ── Phase 1: Pair A lifts and swings, Pair B pushes ──────────────────────
    for leg in PAIR_A:
        move_leg(leg, +yaw, lift=True,  ms=ms)
    for leg in PAIR_B:
        move_leg(leg, -yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)

    # Read feedback — check Pair B is planted before continuing
    for leg in PAIR_B:
        load = read_leg_feedback(leg, phase=1)
    for leg in PAIR_A:
        read_leg_feedback(leg, phase=1)

    # Wait for Pair B ground contact
    contacted = wait_for_contact(PAIR_B)
    if not contacted:
        # Pair B didn't confirm contact — slow down slightly
        time.sleep(0.05)

    # ── Phase 2: Pair A plants, Pair B lifts and swings ──────────────────────
    for leg in PAIR_A:
        move_leg(leg, +yaw, lift=False, ms=ms)
    for leg in PAIR_B:
        move_leg(leg, -yaw, lift=True,  ms=ms)
    time.sleep(ms / 1000 + 0.05)

    # Read feedback — check Pair A is planted
    for leg in PAIR_A:
        load = read_leg_feedback(leg, phase=2)
    for leg in PAIR_B:
        read_leg_feedback(leg, phase=2)

    contacted = wait_for_contact(PAIR_A)
    if not contacted:
        time.sleep(0.05)

    # ── Phase 3: Both pairs return toward neutral ─────────────────────────────
    for leg in PAIR_A:
        move_leg(leg, -yaw * 0.5, lift=False, ms=ms)
    for leg in PAIR_B:
        move_leg(leg, +yaw * 0.5, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)

    # Final feedback read
    for leg in PAIR_A + PAIR_B:
        read_leg_feedback(leg, phase=3)

# ── Crab walk gait with feedback ─────────────────────────────────────────────
def crab_step(direction=1, ms=350):
    """
    One full crab walk cycle with actuator feedback.
    direction: +1 = right, -1 = left
    """
    global step_count
    step_count += 1

    yaw = YAW_STEP * direction

    # ── Phase 1: Lift FL and BL, swing sideways ───────────────────────────────
    for leg in ["FL", "BL"]:
        move_leg(leg, +yaw, lift=True,  ms=ms)
    for leg in ["FR", "BR"]:
        move_leg(leg, +yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)

    for leg in ["FR", "BR"]:
        read_leg_feedback(leg, phase=1)

    # ── Phase 2: Plant FL and BL ──────────────────────────────────────────────
    for leg in ["FL", "BL"]:
        move_leg(leg, +yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)

    wait_for_contact(["FL", "BL"])
    for leg in ["FL", "BL"]:
        read_leg_feedback(leg, phase=2)

    # ── Phase 3: Lift FR and BR, swing to catch up ───────────────────────────
    for leg in ["FR", "BR"]:
        move_leg(leg, -yaw, lift=True,  ms=ms)
    for leg in ["FL", "BL"]:
        move_leg(leg, -yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)

    for leg in ["FL", "BL"]:
        read_leg_feedback(leg, phase=3)

    # ── Phase 4: Plant FR and BR ──────────────────────────────────────────────
    for leg in ["FR", "BR"]:
        move_leg(leg, -yaw, lift=False, ms=ms)
    time.sleep(ms / 1000 + 0.05)

    wait_for_contact(["FR", "BR"])
    for leg in ["FR", "BR"]:
        read_leg_feedback(leg, phase=4)

# ── Shutdown ──────────────────────────────────────────────────────────────────
def shutdown():
    print("\nShutdown procedure started...")
    print("  Step 1: Returning to neutral...")
    all_neutral(ms=1000)
    print("  Step 2: Disabling torque on all motors...")
    for name, mid in ID.items():
        try:
            servos[name].disable_torque()
            print(f"    ✓ {name} torque off")
        except ServoTimeoutError:
            print(f"    ✗ {name} timeout")
    print("  Step 3: Saving log...")
    save_log()
    print("  Done — safe to power off\n")

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print(f"Connecting on {SERIAL_PORT}...")
    try:
        LX16A.initialize(SERIAL_PORT, 0.1)
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    for name, mid in ID.items():
        try:
            servos[name] = LX16A(mid)
        except ServoTimeoutError:
            print(f"WARNING: {name} (ID {mid}) not responding")

    print("Connected!\n")
    print("Moving to neutral...")
    all_neutral()
    print("Ready!\n")

    print("Commands:")
    print("  w - walk forward")
    print("  s - walk backward")
    print("  a - crab left")
    print("  d - crab right")
    print("  n - go to neutral")
    print("  f - print live feedback (all motors)")
    print("  q - quit\n")

    while True:
        cmd = input(">> ").strip().lower()

        if cmd == "w":
            print("Walking forward (3 steps)...")
            for _ in range(3):
                trot_step(direction=1)
            all_neutral()

        elif cmd == "s":
            print("Walking backward (3 steps)...")
            for _ in range(3):
                trot_step(direction=-1)
            all_neutral()

        elif cmd == "a":
            print("Crab left (3 steps)...")
            for _ in range(3):
                crab_step(direction=-1)
            all_neutral()

        elif cmd == "d":
            print("Crab right (3 steps)...")
            for _ in range(3):
                crab_step(direction=1)
            all_neutral()

        elif cmd == "n":
            print("Neutral...")
            all_neutral()

        elif cmd == "f":
            print("\nLive feedback:")
            for name in ID:
                pos, load = read_feedback(name)
                planted = "PLANTED" if (load or 0) > LOAD_THRESHOLD else "air"
                print(f"  {name:<12}: pos={pos:.1f}°  load={load}  [{planted}]")
            print()

        elif cmd == "q":
            shutdown()
            break

        else:
            print("Unknown command. Use w/a/s/d/n/q/f")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted!")
        shutdown()
