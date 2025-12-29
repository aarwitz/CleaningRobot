import smbus
import time
import struct
import random

# ============================================================
# Robot wheel layout
#
#        FRONT
#   ┌─────────────────┐
#   │   (3)      (1)  │
#   │                 │
#   │   (2)      (0)  │
#   └─────────────────┘
#        BACK
# ============================================================

BUS_ID = 7
ADDR = 0x34
HZ = 20         # Slower rate when motors active
DT_TARGET = 1.0 / HZ # seconds

CMD = 60          # gentle motion (safe)
DURATION = 2.0    # seconds per phase

MAX_REASONABLE_DELTA = 10000  # ticks per 0.2s (very generous)

bus = smbus.SMBus(BUS_ID)

# ------------------------
# I2C helper
# ------------------------
def i2c_retry(func, *args, retries=10, delay=0.05):
    for i in range(retries):
        try:
            return func(*args)
        except (OSError, TimeoutError):
            if i == retries - 1:
                raise
            time.sleep(delay)

# Init motor driver
i2c_retry(bus.write_byte_data, ADDR, 0x14, 1)
i2c_retry(bus.write_byte_data, ADDR, 0x15, 0)

def read_encoders():
    raw = i2c_retry(bus.read_i2c_block_data, ADDR, 0x3C, 16)
    return struct.unpack('<iiii', bytes(raw))

def set_motors(l, r):
    i2c_retry(bus.write_i2c_block_data, ADDR, 0x33,
              [r, r, l, l])

def stop():
    try:
        set_motors(0, 0)
    except OSError:
        pass

# ------------------------
# Diagnostic runner
# ------------------------
def run_phase(label, left_cmd, right_cmd):
    print(f"\n--- {label} ---")
    print(f"Commands: L={left_cmd} R={right_cmd}")

    # Start motors
    set_motors(left_cmd, right_cmd)
    
    # Give motors time to start and I2C bus to settle
    time.sleep(2.0)

    enc_prev = read_encoders()
    t_prev = time.time()

    t0 = time.time()
    while time.time() - t0 < DURATION:
        time.sleep(DT_TARGET)
        
        # Add small random variation to simulate real velocity controller updates
        random_variation_l = random.randint(-2, 2)
        random_variation_r = random.randint(-2, 2)
        set_motors(int(left_cmd + random_variation_l), int(right_cmd + random_variation_r))
        
        time.sleep(0.01)  # small delay to avoid I2C collisions

        enc = read_encoders()
        t_now = time.time()
        dt = t_now - t_prev

        d = [enc[i] - enc_prev[i] for i in range(4)]

        flags = []
        for i in range(4):
            if abs(d[i]) > MAX_REASONABLE_DELTA:
                flags.append(f"ENC{i} **SPIKE**")

        print(
            f"dt={dt:6.3f}s | "
            f"enc={enc} | "
            f"d={d} | "
            f"dL={d[2]+d[3]:+7d} dR={d[0]+d[1]:+7d} | "
            f"{' '.join(flags) if flags else 'OK'}"
        )

        if flags:
            print("!! Spike detected — stopping motors !!")
            stop()
            return

        enc_prev = enc
        t_prev = t_now

    stop()

# ------------------------
# Main test
# ------------------------
try:
    print("\n=== Encoder Motion Diagnostic ===")
    print("Robot will move SLOWLY. Keep space around it.\n")

    run_phase("FORWARD (slow)",  CMD,  CMD)
    
    print("\n--- PAUSING ---")
    stop()
    time.sleep(3.0)

    run_phase("BACKWARD (slow)", -CMD, -CMD)
    
    print("\n--- PAUSING ---")
    stop()
    time.sleep(1.0)

    print("\nTest complete.")

finally:
    stop()
