#!/usr/bin/env python3
"""Stand-in for the Raspberry's BNO085 sender: runs until Ctrl+C, like the real one."""
import sys
import time

print('BNO085 initialized at I2C 0x4A', flush=True)
print(f'args: {" ".join(sys.argv[1:])}', flush=True)
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print('BNO085 sender stopped.', flush=True)
