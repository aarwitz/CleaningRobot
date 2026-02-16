#!/usr/bin/env python3
"""
Quick diagnostic: Read encoder 2 repeatedly to check for stability.
Motors stay OFF - this isolates encoder reading issues from motion issues.
"""

import smbus
import time
import struct

BUS_ID = 7
ADDR = 0x34

bus = smbus.SMBus(BUS_ID)

def read_encoders():
    """Read all encoders."""
    try:
        raw = bus.read_i2c_block_data(ADDR, 0x3C, 16)
        return struct.unpack('<iiii', bytes(raw))
    except (OSError, TimeoutError) as e:
        return None

print("╔══════════════════════════════════════════════════╗")
print("║           Encoder 2 Stability Check            ║")
print("║                                                  ║")
print("║  Reading encoder 2 at 20 Hz for 5 seconds      ║")
print("║  Motors OFF — checking for bad reads            ║")
print("╚══════════════════════════════════════════════════╝\n")

# Initialize motor driver
try:
    bus.write_byte_data(ADDR, 0x14, 1)
    bus.write_byte_data(ADDR, 0x15, 0)
    # Ensure motors are stopped
    bus.write_i2c_block_data(ADDR, 0x33, [0, 0, 0, 0])
    # Reset encoders
    bus.write_byte_data(ADDR, 0x35, 1)
    time.sleep(0.1)
except Exception as e:
    print(f"Failed to initialize: {e}")
    exit(1)

prev_enc = read_encoders()
if prev_enc is None:
    print("Failed to read encoders")
    exit(1)

print("Initial values:", prev_enc)
print("\nMonitoring encoder 2 (back-left wheel)...\n")
print("  Time    ENC2 Value    Delta    Status")
print("=" * 50)

spikes = []
reads = 0
errors = 0

for i in range(100):  # 5 seconds at 20 Hz
    time.sleep(0.05)
    enc = read_encoders()
    
    if enc is None:
        errors += 1
        print(f"  {i*0.05:5.2f}s   [I2C ERROR]")
        continue
    
    reads += 1
    delta = enc[2] - prev_enc[2]
    
    # Stationary encoder should have delta near zero
    if abs(delta) > 10:
        status = "⚠ JUMP"
        spikes.append((i*0.05, enc[2], delta))
    else:
        status = "ok"
    
    # Print every 10th reading + any anomalies
    if i % 10 == 0 or abs(delta) > 10:
        print(f"  {i*0.05:5.2f}s   {enc[2]:+10d}   {delta:+6d}    {status}")
    
    prev_enc = enc

print("\n" + "=" * 50)
print(f"\nSummary:")
print(f"  Total reads: {reads}")
print(f"  I2C errors: {errors}")
print(f"  Unexpected jumps: {len(spikes)}")

if spikes:
    print(f"\n⚠ ENCODER 2 IS UNSTABLE")
    print(f"  Detected {len(spikes)} jumps while motors were OFF")
    print(f"  This indicates a hardware/wiring issue, not a software problem")
    print(f"\n  Recommended actions:")
    print(f"  1. Check encoder 2 cable connections")
    print(f"  2. Inspect for loose wires or damaged connectors")
    print(f"  3. Swap encoder 2 with encoder 0 to verify if issue follows the hardware")
elif errors > 10:
    print(f"\n⚠ I2C BUS IS UNSTABLE")
    print(f"  Too many I2C communication errors")
    print(f"  Check I2C bus wiring and pull-up resistors")
else:
    print(f"\n✓ ENCODER 2 IS STABLE")
    print(f"  No issues detected in stationary test")
    print(f"  The problem may be motion-related or a different encoder")

print()
