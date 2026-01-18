import smbus
import time
import struct

# ============================================================
# Encoder layout
#
#        FRONT
#   ┌─────────────────┐
#   │   (3)      (1)  │
#   │                 │
#   │   (2)      (0)  │
#   └─────────────────┘
#        BACK
#
# ============================================================

BUS_ID = 7
ADDR = 0x34
HZ = 5.0
PERIOD = 1.0 / HZ

TICKS_PER_METER = 18940.0

# Very generous physical sanity limit:
# At 1 m/s, 0.2 s → ~3800 ticks
MAX_REASONABLE_DELTA = 10000  

bus = smbus.SMBus(BUS_ID)

def i2c_retry(func, *args, retries=5, delay=0.01):
    for i in range(retries):
        try:
            return func(*args)
        except OSError:
            if i == retries - 1:
                raise
            time.sleep(delay)

# Init driver (same as your other scripts)
i2c_retry(bus.write_byte_data, ADDR, 0x14, 1)
i2c_retry(bus.write_byte_data, ADDR, 0x15, 0)

def read_encoders():
    raw = i2c_retry(bus.read_i2c_block_data, ADDR, 0x3C, 16)
    return struct.unpack('<iiii', bytes(raw))

print("\n=== Encoder Diagnostic Test (READ ONLY) ===")
print(f"Rate: {HZ} Hz")
print("Press Ctrl+C to stop\n")

enc_prev = read_encoders()
t_prev = time.time()

try:
    while True:
        time.sleep(PERIOD)

        enc = read_encoders()
        t_now = time.time()
        dt = t_now - t_prev

        d = [enc[i] - enc_prev[i] for i in range(4)]

        # Per-side sums (for convenience)
        dR = d[0] + d[1]
        dL = d[2] + d[3]

        # Sanity flags per encoder
        flags = []
        for i in range(4):
            if abs(d[i]) > MAX_REASONABLE_DELTA:
                flags.append(f"ENC{i} **SPIKE**")
        flag_str = " | ".join(flags) if flags else "OK"

        print(
            f"dt={dt:6.3f}s | "
            f"enc={enc} | "
            f"d={d} | "
            f"dL={dL:+7d} dR={dR:+7d} | "
            f"{flag_str}"
        )

        enc_prev = enc
        t_prev = t_now

except KeyboardInterrupt:
    print("\nStopped.")
