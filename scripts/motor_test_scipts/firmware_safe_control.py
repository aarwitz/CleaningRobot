#!/usr/bin/env python3
"""
Firmware-Safe Motor Control
Works around the firmware crash bug by:
1. Limiting maximum speed to avoid crash threshold
2. Adding rest periods to prevent firmware saturation
3. Never reading encoders during high-speed operation
"""
import smbus
import time
import struct

BUS_ID = 7
ADDR = 0x34

# CRITICAL: Stay below firmware crash threshold
SAFE_MAX_SPEED = 35  # Firmware seems to crash above ~40 sustained
REST_INTERVAL = 3.0   # Rest every N seconds to let firmware breathe

bus = smbus.SMBus(BUS_ID)

def safe_write(left, right):
    """Write with validation"""
    try:
        bus.write_i2c_block_data(ADDR, 0x33, [int(right), int(right), int(left), int(left)])
        time.sleep(0.05)
        return True
    except Exception as e:
        print(f"  Write failed: {e}")
        return False

def safe_read():
    """Read with corruption detection"""
    try:
        raw = bus.read_i2c_block_data(ADDR, 0x3C, 16)
        vals = struct.unpack('<iiii', bytes(raw))
        
        # Detect firmware corruption
        if any(abs(v) > 50000000 for v in vals) or any(v == -1 for v in vals):
            print(f"  WARNING: Corrupted data {vals}")
            return None
        return vals
    except Exception as e:
        print(f"  Read failed: {e}")
        return None

def init_driver():
    """Initialize driver"""
    print("Initializing motor driver...")
    try:
        bus.write_byte_data(ADDR, 0x14, 1)
        time.sleep(0.1)
        bus.write_byte_data(ADDR, 0x15, 0)
        time.sleep(1.0)
        safe_write(0, 0)
        time.sleep(0.5)
        print("Driver ready")
        return True
    except Exception as e:
        print(f"Init failed: {e}")
        return False

def run_with_rest_periods():
    """
    Run motors with mandatory rest periods to prevent firmware crash
    """
    if not init_driver():
        return
    
    # Get baseline
    baseline = safe_read()
    if not baseline:
        print("Cannot get baseline reading")
        return
    
    print(f"Baseline: {baseline}\n")
    
    # Test speeds - stay in safe range
    test_sequence = [
        (20, 3.0),   # Speed 20 for 3 seconds
        (0, 2.0),    # REST 2 seconds
        (30, 3.0),   # Speed 30 for 3 seconds  
        (0, 2.0),    # REST 2 seconds
        (35, 3.0),   # Speed 35 for 3 seconds (near limit)
        (0, 2.0),    # REST 2 seconds
        (-20, 3.0),  # Reverse 20 for 3 seconds
        (0, 2.0),    # REST 2 seconds
        (-30, 3.0),  # Reverse 30 for 3 seconds
        (0, 2.0),    # REST 2 seconds
        (-35, 3.0),  # Reverse 35 for 3 seconds
        (0, 2.0),    # REST 2 seconds
    ]
    
    last_baseline = baseline
    
    for speed, duration in test_sequence:
        if speed == 0:
            print(f"=== RESTING for {duration}s (firmware recovery) ===")
        else:
            print(f"=== Running at speed {speed:+3} for {duration}s ===")
        
        # Set speed
        if not safe_write(speed, speed):
            print("CRITICAL: Write failed, stopping")
            break
        
        # Run for duration
        start = time.time()
        while time.time() - start < duration:
            time.sleep(0.5)
        
        # Stop motors before reading
        safe_write(0, 0)
        time.sleep(0.3)
        
        # Now safe to read
        enc = safe_read()
        if enc:
            deltas = [enc[i] - last_baseline[i] for i in range(4)]
            total_l = deltas[2] + deltas[3]
            total_r = deltas[0] + deltas[1]
            print(f"  Encoders: L={total_l:+6} R={total_r:+6}")
            last_baseline = enc
        else:
            print("  ERROR: Failed to read encoders - firmware may have crashed")
            print("  Please power cycle the motor driver")
            break
        
        print()
    
    # Final stop
    safe_write(0, 0)
    print("Complete!")

def continuous_safe_operation():
    """
    Continuous operation with built-in safety limits
    Runs motors but never exceeds firmware crash threshold
    """
    if not init_driver():
        return
    
    baseline = safe_read()
    if not baseline:
        print("Cannot get baseline")
        return
    
    print(f"Baseline: {baseline}\n")
    print("Running continuous safe operation (Ctrl+C to stop)...")
    print("Speed will cycle: 0 -> 30 -> 0 -> -30 -> 0 with rest periods\n")
    
    try:
        cycle = 0
        last_baseline = baseline
        
        while True:
            cycle += 1
            print(f"=== Cycle {cycle} ===")
            
            # Forward
            for speed in [0, 15, 30]:
                safe_write(speed, speed)
                time.sleep(2.0)
            
            # Stop and read
            safe_write(0, 0)
            time.sleep(0.5)
            enc = safe_read()
            if enc:
                deltas = [enc[i] - last_baseline[i] for i in range(4)]
                print(f"  Forward: L={deltas[2]+deltas[3]:+6} R={deltas[0]+deltas[1]:+6}")
                last_baseline = enc
            else:
                print("  READ FAILED - firmware crashed, need power cycle")
                break
            
            # Rest period
            time.sleep(2.0)
            
            # Reverse
            for speed in [0, -15, -30]:
                safe_write(speed, speed)
                time.sleep(2.0)
            
            # Stop and read
            safe_write(0, 0)
            time.sleep(0.5)
            enc = safe_read()
            if enc:
                deltas = [enc[i] - last_baseline[i] for i in range(4)]
                print(f"  Reverse: L={deltas[2]+deltas[3]:+6} R={deltas[0]+deltas[1]:+6}")
                last_baseline = enc
            else:
                print("  READ FAILED - firmware crashed, need power cycle")
                break
            
            # Rest period between cycles
            time.sleep(3.0)
            
    except KeyboardInterrupt:
        print("\nStopping...")
    
    safe_write(0, 0)

if __name__ == "__main__":
    import sys
    
    print("Firmware-Safe Motor Control\n")
    print("Choose mode:")
    print("1. Test sequence with rest periods")
    print("2. Continuous safe operation")
    
    choice = input("\nEnter choice (1 or 2): ").strip()
    
    try:
        if choice == "1":
            run_with_rest_periods()
        elif choice == "2":
            continuous_safe_operation()
        else:
            print("Invalid choice")
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        try:
            safe_write(0, 0)
        except:
            pass
