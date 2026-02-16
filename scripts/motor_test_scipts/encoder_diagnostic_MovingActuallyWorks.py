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
LOOP_HZ = 5    # Slow and steady (200ms ticks) to prevent firmware saturation
DT = 1.0 / LOOP_HZ

CMD_MAX = 100
MIN_CMD = 40
MAX_STEP = 20  # Allowing slightly larger steps since HZ is lower

bus = smbus.SMBus(BUS_ID)

# ============================================================
# CRANKY FIRMWARE ABSTRACTION LAYER
# ============================================================

def set_motors(l, r):
    """Reliable write with internal recovery time."""
    try:
        bus.write_i2c_block_data(ADDR, 0x33, [int(r), int(r), int(l), int(l)])
        # The firmware needs a 'cooldown' after processing a PWM change
        time.sleep(0.05) 
        return True
    except OSError:
        return False

def get_encoders():
    """Single-shot read. If it fails, we don't hammer it; we just skip."""
    try:
        raw = bus.read_i2c_block_data(ADDR, 0x3C, 16)
        vals = struct.unpack('<iiii', bytes(raw))
        # Filter out the -1 sentinel and obviously corrupted atomic reads
        if all(v != -1 for v in vals) and abs(vals[0]) < 100000000:
            return vals
    except OSError:
        pass
    return None

# ============================================================
# MAIN RUNNER
# ============================================================

def run_smooth_cycle(target_peak):
    print(f"\n>>> Profile Start: Peak={target_peak}")
    
    current_l = 0
    current_r = 0
    
    # 1. Establish baseline while stationary
    baseline = None
    for _ in range(5):
        baseline = get_encoders()
        if baseline: break
        time.sleep(0.1)

    if not baseline:
        print("CRITICAL: Cannot talk to driver. Check wires.")
        return

    start_time = time.monotonic()
    total_duration = 6.0 
    last_valid_enc = baseline

    while True:
        tick_start = time.monotonic()
        elapsed = tick_start - start_time
        if elapsed > total_duration:
            break

        # A. Calculate Smooth Target
        phase = (elapsed / total_duration) * math.pi
        target = target_peak * math.sin(phase)
        if abs(target) < MIN_CMD: target = 0
            
        # B. Rate Limit
        diff = target - current_l
        step = max(-MAX_STEP, min(MAX_STEP, diff))
        current_l += step
        current_r += step

        # C. THE TRANSACTION WINDOW (Write -> Rest -> Read)
        # This mimics the 'Old' script's 10ms-20ms gap strategy
        write_success = set_motors(current_l, current_r)
        
        # Give it a tiny bit more breath before the read
        time.sleep(0.02) 
        
        enc = get_encoders()

        # D. Feedback
        if enc:
            deltas = [enc[i] - last_valid_enc[i] for i in range(4)]
            print(f"T={elapsed:4.2f}s | Cmd={int(current_l):4} | dL={deltas[2]+deltas[3]:+5} dR={deltas[0]+deltas[1]:+5} | OK")
            last_valid_enc = enc
        else:
            print(f"T={elapsed:4.2f}s | Cmd={int(current_l):4} | -- BUS BUSY / FRAME DROPPED --")

        # E. Maintain Loop Rate
        used = time.monotonic() - tick_start
        if used < DT:
            time.sleep(DT - used)

    set_motors(0, 0)

# ============================================================
# EXECUTION
# ============================================================
try:
    print("Waking up motor driver...")
    # Standard Init
    bus.write_byte_data(ADDR, 0x14, 1)
    bus.write_byte_data(ADDR, 0x15, 0)
    time.sleep(1.0) # Massive pause to let the driver settle

    run_smooth_cycle(CMD_MAX)
    print("\n--- Transitioning ---")
    time.sleep(2.0)
    run_smooth_cycle(-CMD_MAX)

finally:
    print("\nShutting down.")
    try:
        bus.write_i2c_block_data(ADDR, 0x33, [0,0,0,0])
    except:
        pass