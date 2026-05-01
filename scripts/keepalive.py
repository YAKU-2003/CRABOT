"""
SSH Keepalive Script
Keeps the SSH session alive by printing a heartbeat every 30 seconds
Run: python3 keepalive.py
Stop: Ctrl+C
"""

import time
import datetime

print("SSH Keepalive running — press Ctrl+C to stop")
print("=" * 40)

count = 0
while True:
    count += 1
    now = datetime.datetime.now().strftime("%H:%M:%S")
    print(f"  [{now}] alive #{count}")
    time.sleep(30)
