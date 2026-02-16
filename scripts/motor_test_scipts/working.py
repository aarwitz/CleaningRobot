#!/usr/bin/env python3
import smbus
import time
import struct
import math

# ============================================================
# CONFIG
# ============================================================
BUS_ID = 7
ADDR = 0x34
LOOP_HZ = 20   # High frequency polling prevents I2C frame drops
DT = 1.0 / LOOP_HZ

CMD_MAX = 70
MIN_CMD = 30
MAX_STEP = 1  # Scale down for 20x faster loop

bus = smbus.SMBus(BUS_ID)

# ============================================================
# I2C HELPERS - Match old.py retry pattern exactly
# ============================================================

def i2c_retry(func, *args, retries=10, delay=0.05):
    """Retry I2C operations, raise on final failure."""
    for i in range(retries):
        try:
            return func(*args)
        except (OSError, TimeoutError):
            if i == retries - 1:
                raise
            time.sleep(delay)

def read_encoders():
    """Read encoders with retry."""
    raw = i2c_retry(bus.read_i2c_block_data, ADDR, 0x3C, 16)
    return struct.unpack('<iiii', bytes(raw))

def set_motors(l, r):
    """Write motor command with retry."""
    i2c_retry(bus.write_i2c_block_data, ADDR, 0x33, [int(r), int(r), int(l), int(l)])

# ============================================================
# MAIN RUNNER
# ============================================================

def run_smooth_cycle(target_peak):
    time.sleep(2.0)  # Let firmware settle before starting
    print(f"\n>>> Profile Start: Peak={target_peak}")
    
    current_l = 0
    current_r = 0
    
    # 1. Establish baseline while stationary
    time.sleep(0.5)
    last_valid_enc = read_encoders()

    start_time = time.monotonic()
    total_duration = 25.0  # Shorter cycles with rest periods
    t_prev = start_time

    while True:
        tick_start = time.monotonic()
        elapsed = tick_start - start_time
        if elapsed > total_duration:
            break

        # Sleep at loop start (like old.py) to maintain rate
        time.sleep(DT)

        # A. Calculate Smooth Target
        phase = (elapsed / total_duration) * math.pi
        target = target_peak * math.sin(phase)
        if abs(target) < MIN_CMD: target = 0
            
        # B. Rate Limit
        diff = target - current_l
        step = max(-MAX_STEP, min(MAX_STEP, diff))
        current_l += step
        current_r += step

        # C. Write motor command (no delay before)
        set_motors(current_l, current_r)
        
        # Small delay between write and read (like old.py)
        time.sleep(0.01)
        
        # D. Read encoders
        enc = read_encoders()

        # E. Feedback
        t_now = time.monotonic()
        dt = t_now - t_prev
        
        deltas = [enc[i] - last_valid_enc[i] for i in range(4)]
        print(f"T={elapsed:4.2f}s | dt={dt:5.3f}s | Cmd={int(current_l):4} | dL={deltas[2]+deltas[3]:+5} dR={deltas[0]+deltas[1]:+5} | OK")
        last_valid_enc = enc
        t_prev = t_now

    # Stop motors at end
    try:
        set_motors(0, 0)
    except (OSError, TimeoutError):
        pass

# ============================================================
# EXECUTION
# ============================================================
try:
    print("Waking up motor driver...")
    # Standard Init with retry
    i2c_retry(bus.write_byte_data, ADDR, 0x14, 1)
    i2c_retry(bus.write_byte_data, ADDR, 0x15, 0)
    time.sleep(2.0)

    # Multiple shorter cycles with rest periods
    run_smooth_cycle(CMD_MAX)
    print("\n--- RESTING (5 sec) ---")
    time.sleep(5.0)
    
    run_smooth_cycle(CMD_MAX)
    print("\n--- RESTING (5 sec) ---")
    time.sleep(5.0)
    
    run_smooth_cycle(-CMD_MAX)
    print("\n--- RESTING (5 sec) ---")
    time.sleep(5.0)
    
    run_smooth_cycle(-CMD_MAX)

finally:
    print("\nShutting down.")
    try:
        set_motors(0, 0)
    except (OSError, TimeoutError):
        pass