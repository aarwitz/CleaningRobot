#!/usr/bin/env python3
"""
Enhanced motor control with firmware recovery strategies
"""
import smbus
import time
import struct
import math

# ============================================================
# CONFIG
# ============================================================
BUS_ID = 7
ADDR = 0x34
LOOP_HZ = 1
DT = 1.0 / LOOP_HZ

CMD_MAX = 100
MIN_CMD = 20
MAX_STEP = 5

# ============================================================
# FIRMWARE RECOVERY LAYER
# ============================================================

class RobustMotorDriver:
    def __init__(self, bus_id, addr):
        self.bus_id = bus_id
        self.addr = addr
        self.bus = None
        self.consecutive_failures = 0
        self.total_reads = 0
        self.failed_reads = 0
        
    def init_bus(self):
        """Initialize or reinitialize the I2C bus"""
        if self.bus:
            try:
                self.bus.close()
            except:
                pass
        
        self.bus = smbus.SMBus(self.bus_id)
        time.sleep(0.1)
        
    def reset_driver(self):
        """Attempt to reset the motor driver firmware"""
        print("\n!!! Attempting driver reset !!!")
        try:
            self.init_bus()
            self.bus.write_byte_data(self.addr, 0x14, 1)
            time.sleep(0.1)
            self.bus.write_byte_data(self.addr, 0x15, 0)
            time.sleep(0.5)
            # Stop motors
            self.bus.write_i2c_block_data(self.addr, 0x33, [0, 0, 0, 0])
            time.sleep(0.5)
            self.consecutive_failures = 0
            print("!!! Reset complete !!!")
            return True
        except Exception as e:
            print(f"!!! Reset failed: {e} !!!")
            return False
    
    def set_motors(self, left, right):
        """Write motor command with recovery"""
        for attempt in range(3):
            try:
                self.bus.write_i2c_block_data(self.addr, 0x33, 
                                             [int(right), int(right), int(left), int(left)])
                time.sleep(0.05)
                self.consecutive_failures = 0
                return True
            except OSError as e:
                print(f"  Write attempt {attempt+1} failed: {e}")
                time.sleep(0.1)
                
                if attempt == 2:
                    self.consecutive_failures += 1
                    if self.consecutive_failures >= 3:
                        self.reset_driver()
                    return False
        
        return False
    
    def get_encoders(self):
        """Read encoders with corruption detection"""
        self.total_reads += 1
        
        for attempt in range(2):
            try:
                raw = self.bus.read_i2c_block_data(self.addr, 0x3C, 16)
                vals = struct.unpack('<iiii', bytes(raw))
                
                # Corruption detection
                if any(v == -1 for v in vals):
                    continue
                    
                # Check for overflow (likely corruption)
                if any(abs(v) > 50000000 for v in vals):
                    print(f"  CORRUPTED data detected: {vals}")
                    self.failed_reads += 1
                    # This is serious - might need reset
                    self.consecutive_failures += 1
                    if self.consecutive_failures >= 3:
                        self.reset_driver()
                    continue
                
                self.consecutive_failures = 0
                return vals
                
            except OSError as e:
                if attempt == 0:
                    time.sleep(0.05)
                    continue
                else:
                    self.failed_reads += 1
                    self.consecutive_failures += 1
                    
        return None
    
    def get_stats(self):
        """Return communication statistics"""
        if self.total_reads > 0:
            success_rate = ((self.total_reads - self.failed_reads) / self.total_reads) * 100
        else:
            success_rate = 0
        return {
            'total_reads': self.total_reads,
            'failed_reads': self.failed_reads,
            'success_rate': success_rate
        }

# ============================================================
# MAIN RUNNER  
# ============================================================

def run_smooth_cycle(driver, target_peak):
    print(f"\n>>> Profile Start: Peak={target_peak}")
    
    current_l = 0
    current_r = 0
    
    # 1. Establish baseline
    baseline = None
    for attempt in range(10):
        baseline = driver.get_encoders()
        if baseline:
            print(f"Baseline established: {baseline}")
            break
        print(f"  Baseline attempt {attempt+1} failed, retrying...")
        time.sleep(0.2)
    
    if not baseline:
        print("CRITICAL: Cannot establish baseline after 10 attempts")
        return False
    
    start_time = time.monotonic()
    total_duration = 15.0
    last_valid_enc = baseline
    valid_readings = 0
    
    while True:
        tick_start = time.monotonic()
        elapsed = tick_start - start_time
        if elapsed > total_duration:
            break
        
        # Calculate target
        phase = (elapsed / total_duration) * math.pi
        target = target_peak * math.sin(phase)
        if abs(target) < MIN_CMD:
            target = 0
        
        # Rate limit
        diff = target - current_l
        step = max(-MAX_STEP, min(MAX_STEP, diff))
        current_l += step
        current_r += step
        
        # Write command
        write_ok = driver.set_motors(current_l, current_r)
        
        # Wait before read (give firmware time)
        time.sleep(0.1)
        
        # Read encoders
        enc = driver.get_encoders()
        
        # Feedback
        if enc:
            deltas = [enc[i] - last_valid_enc[i] for i in range(4)]
            print(f"T={elapsed:4.2f}s | Cmd={int(current_l):4} | "
                  f"dL={deltas[2]+deltas[3]:+5} dR={deltas[0]+deltas[1]:+5} | OK")
            last_valid_enc = enc
            valid_readings += 1
        else:
            status = "WRITE_FAIL" if not write_ok else "READ_FAIL"
            print(f"T={elapsed:4.2f}s | Cmd={int(current_l):4} | -- {status} --")
        
        # Maintain loop rate
        used = time.monotonic() - tick_start
        if used < DT:
            time.sleep(DT - used)
    
    # Stop motors
    driver.set_motors(0, 0)
    
    success_rate = (valid_readings / (total_duration / DT)) * 100
    print(f"Cycle complete: {valid_readings} valid readings, {success_rate:.1f}% success rate")
    
    return success_rate > 50  # Consider success if >50% readings valid

# ============================================================
# EXECUTION
# ============================================================

try:
    print("Initializing robust motor driver...")
    driver = RobustMotorDriver(BUS_ID, ADDR)
    driver.init_bus()
    
    # Initial reset/setup
    driver.reset_driver()
    
    # Run forward cycle
    success1 = run_smooth_cycle(driver, CMD_MAX)
    
    print("\n--- Transitioning (5 sec rest) ---")
    time.sleep(5.0)
    
    # Run backward cycle
    success2 = run_smooth_cycle(driver, -CMD_MAX)
    
    # Print statistics
    print("\n=== Communication Statistics ===")
    stats = driver.get_stats()
    print(f"Total reads: {stats['total_reads']}")
    print(f"Failed reads: {stats['failed_reads']}")
    print(f"Success rate: {stats['success_rate']:.1f}%")
    
    if not success1 or not success2:
        print("\n!!! WARNING: One or more cycles had poor communication !!!")
    
finally:
    print("\nShutting down.")
    try:
        driver.set_motors(0, 0)
    except:
        pass
