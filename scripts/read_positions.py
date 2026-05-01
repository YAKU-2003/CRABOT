"""
Motor Position Reader
Press ENTER to read all positions
Stop: Ctrl+C
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

def read_all():
    print("\nCurrent positions:")
    for mid, name in MOTORS.items():
        try:
            s = LX16A(mid)
            pos = s.get_physical_angle()
            print(f"  {name} (ID {mid}): {pos:.1f}°")
        except ServoTimeoutError:
            print(f"  {name} (ID {mid}): TIMEOUT")
    print()

def main():
    print(f"Connecting on {SERIAL_PORT}...")
    try:
        LX16A.initialize(SERIAL_PORT, 0.1)
        print("Connected!\n")
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    print("Press ENTER to read positions, Ctrl+C to quit\n")
    while True:
        input(">> Press ENTER to read...")
        read_all()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
