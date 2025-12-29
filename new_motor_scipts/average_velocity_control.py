import smbus
import time
import struct
"""
      2─────────────────3
      │                 │
 BACK │     Chassis     │ FRONT ==>
      │                 │
      0─────────────────1

(the numbers are the indices for each motor in velocity vector
we send over i2c)
"""
bus = smbus.SMBus(7)
ADDR = 0x34

BASE_SPEED_FWD = 75
BASE_SPEED_REV = -75
KP = 0.75               # Very gentle correction gain
DT = 0.1                # 10 Hz control (very slow) - 20hz fails
DURATION = 6.0
MIN_SPEED = 20
MAX_SPEED = 100
LOG_INTERVAL = 1.0      # Log every second

# ------------------------
# I2C retry helper
# ------------------------
def i2c_retry(func, *args, retries=10, delay=0.05):
    for i in range(retries):
        try:
            return func(*args)
        except OSError:
            if i == retries - 1:
                raise
            time.sleep(delay)

# ------------------------
# Init driver
# ------------------------
i2c_retry(bus.write_byte_data, ADDR, 0x14, 1)
i2c_retry(bus.write_byte_data, ADDR, 0x15, 0)

# ------------------------
# Helpers
# ------------------------
def read_all_encoders():
    raw = i2c_retry(bus.read_i2c_block_data, ADDR, 0x3C, 16)
    return struct.unpack('<iiii', bytes(raw))

def set_motor_speeds(m1, m2, m3, m4):
    def clamp(u):
        u = int(u)
        if abs(u) < MIN_SPEED and u != 0:
            return MIN_SPEED if u > 0 else -MIN_SPEED
        return max(-MAX_SPEED, min(MAX_SPEED, u))

    speeds = [clamp(m) for m in (m1, m2, m3, m4)]
    i2c_retry(bus.write_i2c_block_data, ADDR, 0x33, speeds)
    return speeds

def stop():
    try:
        i2c_retry(bus.write_i2c_block_data, ADDR, 0x33, [0, 0, 0, 0])
    except OSError:
        pass

# ------------------------
# Synchronized speed drive with gentle position-based feedback
# ------------------------
def drive_synchronized(base_speed, duration):
    print(f"\nDriving at synchronized speed={base_speed} for {duration}s")
    
    start_encoders = read_all_encoders()
    print(f"Start encoders: {start_encoders}")
    
    start_time = time.time()
    last_log = start_time
    
    # Initial speeds - all motors start at base speed
    motor_speeds = [base_speed, base_speed, base_speed, base_speed]
    
    while time.time() - start_time < duration:
        # Set current motor speeds
        set_motor_speeds(*motor_speeds)
        
        # Wait for control interval
        time.sleep(DT)
        
        # Read total movement since start
        curr_encoders = read_all_encoders()
        total_movement = [curr - start for curr, start in zip(curr_encoders, start_encoders)]
        
        # Calculate average total movement (reference)
        avg_movement = sum(total_movement) / 4.0
        
        # Very gently adjust each motor speed based on total position error
        for i in range(4):
            position_error = avg_movement - total_movement[i]
            # If motor moved less than average, nudge speed up slightly
            # If motor moved more than average, nudge speed down slightly
            motor_speeds[i] = base_speed + (KP * position_error)
            # Very tight clamp to prevent jumpiness
            motor_speeds[i] = max(base_speed - 3, min(base_speed + 3, motor_speeds[i]))
        
        # Log progress
        now = time.time()
        if now - last_log >= LOG_INTERVAL:
            print(f"Movement: {total_movement} | Speeds: {[int(s) for s in motor_speeds]}")
            last_log = now
    
    # Stop motors
    stop()
    
    end_encoders = read_all_encoders()
    print(f"End encoders: {end_encoders}")
    
    # Calculate total movement
    total_deltas = [end - start for start, end in zip(start_encoders, end_encoders)]
    print(f"Total movement: {total_deltas}")
    print(f"Max difference: {max(total_deltas) - min(total_deltas)} ticks")

# ------------------------
# Main
# ------------------------
try:
    print("\n=== Synchronized Speed Test ===")

    # Forward at speed 60 with sync
    print("\n--- FORWARD ---")
    drive_synchronized(BASE_SPEED_FWD, DURATION)
    
    # Pause with motors stopped
    print("\n--- PAUSE ---")
    print("Motors stopped for 3 seconds")
    stop()
    time.sleep(3.0)
    
    # Backward at speed -60 with sync
    print("\n--- BACKWARD ---")
    drive_synchronized(BASE_SPEED_REV, DURATION)
    
    print("\n=== Test Complete ===")

except KeyboardInterrupt:
    print("\nEmergency stop")

finally:
    stop()
    print("Motors stopped")
