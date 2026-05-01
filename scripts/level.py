"""
Manual Position Setter - level the bot
Type motor ID and angle to move it
Run: python3 level.py
"""

import time
import sys
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

def main():
    print(f"Connecting on {SERIAL_PORT}...")
    LX16A.initialize(SERIAL_PORT, 0.1)
    print("Connected!\n")

    print("Motor IDs:")
    for mid, name in MOTORS.items():
        print(f"  {mid} = {name}")
    print("\nType: <id> <angle>  e.g.  4 195")
    print("Type: 'all' to read all positions")
    print("Type: 'quit' to exit\n")

    while True:
        cmd = input(">> ").strip().lower()

        if cmd == "quit":
            break

        elif cmd == "all":
            print("\nCurrent positions:")
            for mid, name in MOTORS.items():
                try:
                    pos = LX16A(mid).get_physical_angle()
                    print(f"  {name} (ID {mid}): {pos:.1f}°")
                except ServoTimeoutError:
                    print(f"  {name} (ID {mid}): TIMEOUT")
            print()

        else:
            parts = cmd.split()
            if len(parts) != 2:
                print("  Format: <id> <angle>  e.g.  4 195\n")
                continue
            try:
                mid   = int(parts[0])
                angle = float(parts[1])
            except ValueError:
                print("  Numbers only please\n")
                continue

            if mid not in MOTORS:
                print(f"  Invalid ID, choose from {list(MOTORS.keys())}\n")
                continue

            if not 0 <= angle <= 240:
                print("  Angle must be 0-240\n")
                continue

            try:
                LX16A(mid).move(angle, time=500)
                time.sleep(0.6)
                actual = LX16A(mid).get_physical_angle()
                print(f"  {MOTORS[mid]} → sent {angle}°, actual {actual:.1f}°\n")
            except ServoTimeoutError:
                print(f"  TIMEOUT on motor {mid}\n")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDone.")
