"""
Crab-Bot Motor Setup Tool
- Turn torque off so you can pose the bot by hand
- Read back all positions
- Save positions to bot.env for use in gait controller

Run: python3 motor_setup.py
"""

import time
import sys
import os
from pylx16a.lx16a import LX16A, ServoTimeoutError

SERIAL_PORT = "/dev/ttyUSB0"

MOTORS = {
    1: "FL_PITCH",
    2: "FL_YAW",
    3: "FR_YAW",
    4: "FR_PITCH",
    5: "BL_PITCH",
    6: "BL_YAW",
    7: "BR_YAW",
    8: "BR_PITCH",
}

def connect():
    print(f"Connecting on {SERIAL_PORT}...")
    try:
        LX16A.initialize(SERIAL_PORT, 0.1)
        print("Connected!\n")
    except Exception as e:
        print(f"ERROR: Could not connect: {e}")
        sys.exit(1)

def get_servo(mid):
    try:
        return LX16A(mid)
    except ServoTimeoutError:
        return None

def torque_all(enable: bool):
    state = "ON" if enable else "OFF"
    print(f"\nTurning torque {state} on all motors...")
    for mid, name in MOTORS.items():
        try:
            s = LX16A(mid)
            if enable:
                s.enable_torque()
            else:
                s.disable_torque()
            print(f"  ✓ {name} (ID {mid}) torque {state}")
        except ServoTimeoutError:
            print(f"  ✗ {name} (ID {mid}) TIMEOUT - not responding")
    print()

def read_all():
    print("\nCurrent positions:")
    positions = {}
    for mid, name in MOTORS.items():
        try:
            s = LX16A(mid)
            pos = s.get_physical_angle()
            positions[mid] = pos
            print(f"  {name} (ID {mid}): {pos:.1f}°")
        except ServoTimeoutError:
            positions[mid] = None
            print(f"  {name} (ID {mid}): TIMEOUT - not responding")
    print()
    return positions

def move_motor(mid, angle):
    try:
        s = LX16A(mid)
        s.enable_torque()
        s.move(angle, time=500)
        time.sleep(0.6)
        actual = s.get_physical_angle()
        print(f"  → {MOTORS[mid]} (ID {mid}): sent {angle:.1f}°, actual: {actual:.1f}°\n")
    except ServoTimeoutError:
        print(f"  TIMEOUT: Motor {mid} not responding\n")

def save_env(positions):
    print("\nSaving positions to bot.env...")
    with open("bot.env", "w") as f:
        f.write("# bot.env - neutral positions in degrees (0-240)\n")
        f.write(f"SERIAL_PORT={SERIAL_PORT}\n")
        for mid, name in MOTORS.items():
            pos = positions.get(mid)
            val = f"{pos:.1f}" if pos is not None else "120.0"
            f.write(f"{name}_NEUTRAL={val}\n")
    print("Saved to bot.env!\n")
    print("Contents:")
    with open("bot.env", "r") as f:
        print(f.read())

def print_menu():
    print("=" * 50)
    print("  CRAB-BOT MOTOR SETUP")
    print("=" * 50)
    print("  1  - Torque OFF (free move, pose by hand)")
    print("  2  - Torque ON  (lock all motors)")
    print("  3  - Read all positions")
    print("  4  - Move a single motor")
    print("  5  - Save current positions to bot.env")
    print("  6  - Full calibration workflow (recommended)")
    print("  q  - Quit")
    print("=" * 50)

def full_workflow():
    """Guided workflow: torque off -> pose -> read -> save"""
    print("\n--- FULL CALIBRATION WORKFLOW ---")
    print("Step 1: Turning torque OFF so you can pose the bot by hand...")
    torque_all(False)

    input("Step 2: Physically pose the bot into a natural standing position, then press ENTER...")

    print("Step 3: Reading positions...")
    positions = read_all()

    save = input("Step 4: Save these positions to bot.env? (y/n): ").strip().lower()
    if save == "y":
        save_env(positions)
        print("Step 5: Turning torque back ON...")
        torque_all(True)
        print("All done! You can now run the gait controller with these positions.\n")
    else:
        print("Not saved. Turning torque back ON...")
        torque_all(True)

def main():
    connect()

    # Print motor map
    print("Motor map:")
    for mid, name in MOTORS.items():
        print(f"  ID {mid} = {name}")
    print()

    while True:
        print_menu()
        cmd = input("  > ").strip().lower()

        if cmd == "1":
            torque_all(False)

        elif cmd == "2":
            torque_all(True)

        elif cmd == "3":
            read_all()

        elif cmd == "4":
            try:
                mid = int(input("  Motor ID (1-8): ").strip())
                if mid not in MOTORS:
                    print(f"  Invalid ID. Choose from {list(MOTORS.keys())}\n")
                    continue
                angle = float(input("  Angle (0-240): ").strip())
                if not 0 <= angle <= 240:
                    print("  Angle must be 0-240\n")
                    continue
                move_motor(mid, angle)
            except ValueError:
                print("  Please enter numbers only\n")

        elif cmd == "5":
            positions = read_all()
            save_env(positions)

        elif cmd == "6":
            full_workflow()

        elif cmd == "q":
            print("Turning torque ON before exit...")
            torque_all(True)
            print("Bye!")
            break

        else:
            print("  Unknown command\n")

if __name__ == "__main__":
    main()
