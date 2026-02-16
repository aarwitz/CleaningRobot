#!/usr/bin/env python3
"""
Ultra-conservative motor control - separate read and write phases
Strategy: Avoid simultaneous read/write operations that might confuse firmware
"""
import smbus
import time
import struct
import math

BUS_ID = 7
ADDR = 0x34

bus = smbus.SMBus(BUS_ID)

def safe_write(left, right):
    """Write only, no reading"""
    try:
        bus.write_i2c_block_data(ADDR, 0x33, [int(right), int(right), int(left), int(left)])
        return True
    except:
        return False

def safe_read():
    """Read only, with validation"""
    try:
        raw = bus.read_i2c_block_data(ADDR, 0x3C, 16)
        vals = struct.unpack('<iiii', bytes(raw))
        
        # Reject corrupted data
        if any(abs(v) > 50000000 for v in vals) or any(v == -1 for v in vals):
            return None
        return vals
    except:
        return None

def run_separate_phases():
    """
    Run motor control with completely separate read/write phases
    - Send commands for 5 seconds
    - Stop and read encoders
    - Repeat
    """
    print("Initializing...")
    bus.write_byte_data(ADDR, 0x14, 1)
    bus.write_byte_data(ADDR, 0x15, 0)
    time.sleep(2.0)
    
    # Get initial reading
    baseline = None
    for _ in range(10):
        baseline = safe_read()
        if baseline:
            print(f"Baseline: {baseline}")
            break
        time.sleep(0.2)
    
    if not baseline:
        print("Cannot get baseline reading")
        return
    
    speeds = [0, 30, 50, 70, 50, 30, 0, -30, -50, -70, -50, -30, 0]
    
    for speed in speeds:
        print(f"\n=== Speed: {speed} ===")
        
        # WRITE PHASE - just send commands repeatedly
        print("Writing commands...")
        for i in range(5):
            if safe_write(speed, speed):
                print(f"  Write {i+1}/5: OK")
            else:
                print(f"  Write {i+1}/5: FAILED")
            time.sleep(0.3)
        
        # STOP for measurement
        print("Stopping for measurement...")
        safe_write(0, 0)
        time.sleep(0.5)  # Let motors coast
        
        # READ PHASE - get encoder values
        print("Reading encoders...")
        for attempt in range(5):
            enc = safe_read()
            if enc:
                deltas = [enc[i] - baseline[i] for i in range(4)]
                total_l = deltas[2] + deltas[3]
                total_r = deltas[0] + deltas[1]
                print(f"  Read {attempt+1}/5: Left={total_l:+6} Right={total_r:+6}")
                baseline = enc  # Update baseline
                break
            else:
                print(f"  Read {attempt+1}/5: FAILED")
                time.sleep(0.2)
        
        time.sleep(1.0)
    
    # Final stop
    safe_write(0, 0)
    print("\nDone!")

if __name__ == "__main__":
    try:
        run_separate_phases()
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        try:
            safe_write(0, 0)
        except:
            pass
