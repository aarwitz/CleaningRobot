import smbus
import time
import struct

bus = smbus.SMBus(7)
ADDR = 0x34


def i2c_retry(func, *args, max_retries=10, delay=0.02, **kwargs):
    """Retry I2C operations that may fail with IOError"""
    for attempt in range(max_retries):
        try:
            return func(*args, **kwargs)
        except (IOError, OSError) as e:
            # print the error for debugging
            print(f"I2C error on attempt {attempt + 1}/{max_retries}: {e}")
            if attempt == max_retries - 1:
                raise  # Re-raise on final attempt
            # Exponential backoff: wait longer on each retry
            wait_time = delay * (1.5 ** attempt)
            time.sleep(wait_time)
    return None


# Init (run once)
i2c_retry(bus.write_byte_data, ADDR, 0x14, 1)  # Hall encoder motors
i2c_retry(bus.write_byte_data, ADDR, 0x15, 0)  # polarity


def read_encoder(bus, ADDR, reg):
    raw = i2c_retry(bus.read_i2c_block_data, ADDR, reg, 4)
    return struct.unpack('<i', bytes(raw))[0]


def read_all_encoders():
    """Read all 4 encoders in a single I2C block read (16 bytes starting at 0x3C)"""
    raw = i2c_retry(bus.read_i2c_block_data, ADDR, 0x3C, 16)
    # Unpack 4 consecutive 32-bit signed integers (little-endian)
    enc1, enc2, enc3, enc4 = struct.unpack('<iiii', bytes(raw))
    return enc1, enc2, enc3, enc4


def set_motor_speeds(m1, m2, m3, m4):
    """Set all four motor speeds at once"""
    # Clamp values to -100..100
    m1 = max(-100, min(100, int(m1)))
    m2 = max(-100, min(100, int(m2)))
    m3 = max(-100, min(100, int(m3)))
    m4 = max(-100, min(100, int(m4)))
    i2c_retry(bus.write_i2c_block_data, ADDR, 0x33, [m1, m2, m3, m4])


def stop():
    set_motor_speeds(0, 0, 0, 0)


class PIDController:
    def __init__(self, kp, ki, kd, max_output=60):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0
        self.last_error = 0
        
    def reset(self):
        self.integral = 0
        self.last_error = 0
        
    def update(self, error, dt):
        # Proportional term
        p = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i = self.ki * self.integral
        
        # Derivative term
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0
        d = self.kd * derivative
        
        # Calculate output
        output = p + i + d
        
        # Clamp output
        output = max(-self.max_output, min(self.max_output, output))
        
        self.last_error = error
        return output


def move_to_target(target_ticks, kp=0.2, ki=0.0, kd=0.0, timeout=10.0):
    """
    Move all motors to a target encoder position relative to current position
    Returns True if successful, False if timeout
    """
    # Read starting positions
    start1, start2, start3, start4 = read_all_encoders()
    
    # Create PID controllers for each motor and reset to clear any old state
    pid1 = PIDController(kp, ki, kd)
    pid1.reset()
    pid2 = PIDController(kp, ki, kd)
    pid2.reset()
    pid3 = PIDController(kp, ki, kd)
    pid3.reset()
    pid4 = PIDController(kp, ki, kd)
    pid4.reset()
    
    # Target positions
    target1 = start1 + target_ticks
    target2 = start2 + target_ticks
    target3 = start3 + target_ticks
    target4 = start4 + target_ticks
    
    start_time = time.time()
    last_time = start_time
    tolerance = 5  # Stop when within 10 ticks of target
    
    print(f"Moving to target: {target_ticks} ticks...")
    
    while time.time() - start_time < timeout:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Read current positions
        pos1, pos2, pos3, pos4 = read_all_encoders()
        
        # Calculate errors
        error1 = target1 - pos1
        error2 = target2 - pos2
        error3 = target3 - pos3
        error4 = target4 - pos4
        
        # Check if all motors are at target
        if (abs(error1) < tolerance and abs(error2) < tolerance and 
            abs(error3) < tolerance and abs(error4) < tolerance):
            print(f"Target reached in {current_time - start_time:.2f}s")
            stop()
            return True
        
        # Calculate motor speeds using PID
        speed1 = pid1.update(error1, dt)
        speed2 = pid2.update(error2, dt)
        speed3 = pid3.update(error3, dt)
        speed4 = pid4.update(error4, dt)
        
        # Set motor speeds
        set_motor_speeds(speed1, speed2, speed3, speed4)
        
        time.sleep(0.05)  # 20Hz update rate
    
    print(f"Timeout after {timeout}s")
    stop()
    return False


# Main execution with safety
try:
    print("=" * 50)
    print("PID Control Test - Move Forward and Back")
    print("=" * 50)

    # Record initial positions
    start1, start2, start3, start4 = read_all_encoders()

    print(f"\nInitial positions:")
    print(f"Motor 1: {start1}")
    print(f"Motor 2: {start2}")
    print(f"Motor 3: {start3}")
    print(f"Motor 4: {start4}")

    # Move forward 18000 ticks
    print("\n--- Moving Forward 18000 ticks ---")
    move_to_target(18000, kp=0.2, ki=0.0, kd=0.0, timeout=30.0)

    time.sleep(1.0)

    # Read positions after forward movement
    mid1, mid2, mid3, mid4 = read_all_encoders()

    print(f"\nPositions after forward movement:")
    print(f"Motor 1: {mid1} (moved {mid1 - start1})")
    print(f"Motor 2: {mid2} (moved {mid2 - start2})")
    print(f"Motor 3: {mid3} (moved {mid3 - start3})")
    print(f"Motor 4: {mid4} (moved {mid4 - start4})")

    # Move backward 18000 ticks (return to start)
    print("\n--- Moving Backward 18000 ticks ---")
    move_to_target(-18000, kp=0.2, ki=0.00, kd=0.0, timeout=30.0)

    time.sleep(1.0)

    # Read final positions
    end1, end2, end3, end4 = read_all_encoders()

    # Calculate net differences from start
    diff1 = end1 - start1
    diff2 = end2 - start2
    diff3 = end3 - start3
    diff4 = end4 - start4

    print(f"\nFinal positions:")
    print(f"Motor 1: Start={start1}, End={end1}, Diff={diff1}")
    print(f"Motor 2: Start={start2}, End={end2}, Diff={diff2}")
    print(f"Motor 3: Start={start3}, End={end3}, Diff={diff3}")
    print(f"Motor 4: Start={start4}, End={end4}, Diff={diff4}")

    print("\n" + "=" * 50)
    print(f"Maximum position error: {max(abs(diff1), abs(diff2), abs(diff3), abs(diff4))} ticks")
    print("=" * 50)

except KeyboardInterrupt:
    print("\n\nEmergency Stop Triggered (Ctrl+C)")
finally:
    # Ensure motors are always stopped, even if the script crashes
    stop()
    print("Motors stopped.")
