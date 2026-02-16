#!/usr/bin/env python3
import smbus
import time
import struct
import math

BUS_ID = 7
ADDR = 0x34
LOOP_HZ = 4    # Dropping to 4Hz (250ms) for maximum stability
DT = 1.0 / LOOP_HZ

CMD_MAX = 100
MIN_CMD = 40 
MAX_STEP = 15

bus = smbus.SMBus(BUS_ID)

def safe_write(l, r):
    try:
        # We only write the block once
        bus.write_i2c_block_data(ADDR, 0x33, [int(r), int(r), int(l), int(l)])
        return True
    except OSError:
        return False

def safe_read():
    try:
        raw = bus.read_i2c_block_data(ADDR, 0x3C, 16)
        if not raw: return None
        vals = struct.unpack('<iiii', bytes(raw))
        # Keep the filters you've proven you need
        if all(v != -1 for v in vals) and abs(vals[0]) < 100000000:
            return vals
    except:
        pass
    return None

def run_test():
    # 1. INITIALIZATION PHASE
    print("Sending Wake-up commands...")
    try:
        bus.write_byte_data(ADDR, 0x14, 1) # Enable
        time.sleep(0.1)
        bus.write_byte_data(ADDR, 0x15, 0) # Mode
    except OSError:
        print("Initial Wake-up failed. Is the driver powered?")
        return

    print("WAITING 2 SECONDS FOR FIRMWARE TO SETTLE...")
    time.sleep(2.0) 

    # 2. BASELINE PHASE
    print("Polling baseline encoders...", end="", flush=True)
    baseline = None
    for _ in range(10):
        baseline = safe_read()
        if baseline: break
        time.sleep(0.2)
        print(".", end="", flush=True)

    if not baseline:
        print("\nFATAL: Driver is active but won't report encoders.")
        return
    print(" SUCCESS")

    # 3. MOTION PHASE
    start_time = time.monotonic()
    total_duration = 6.0
    last_valid_enc = baseline
    current_cmd = 0

    print(f"\n{'Time':<6} | {'Cmd':<5} | {'dL+dR':<8} | {'Status'}")
    print("-" * 40)

    while True:
        tick_start = time.monotonic()
        elapsed = tick_start - start_time
        if elapsed > total_duration: break

        # Calculate S-Curve Target
        phase = (elapsed / total_duration) * math.pi
        target = 0
        if math.sin(phase) > 0.05:
            target = MIN_CMD + (math.sin(phase) * (CMD_MAX - MIN_CMD))

        # Smooth the command
        diff = target - current_cmd
        current_cmd += max(-MAX_STEP, min(MAX_STEP, diff))

        # THE TRANSACTION: Write, then wait, then Read
        # We are putting a massive 100ms gap BETWEEN the write and read
        safe_write(current_cmd, current_cmd)
        
        time.sleep(0.1) # <--- The "Safety Buffer"
        
        enc = safe_read()

        if enc:
            deltas = [enc[i] - last_valid_enc[i] for i in range(4)]
            # Summing L motors and R motors
            move_sum = sum(deltas)
            print(f"{elapsed:5.2f}s | {int(current_cmd):5} | {move_sum:+8} | OK")
            last_valid_enc = enc
        else:
            print(f"{elapsed:5.2f}s | {int(current_cmd):5} | {'--':<8} | DROP")

        # Sleep to keep loop timing
        used = time.monotonic() - tick_start
        if used < DT:
            time.sleep(DT - used)

    safe_write(0, 0)

try:
    run_test()
finally:
    print("\nScript Finished.")
    try:
        safe_write(0, 0)
    except:
        pass