import smbus
import time
import struct
from enum import Enum

# ============================================================
# Robot geometry & constants
# ============================================================

BUS_ID = 7
ADDR = 0x34

TICKS_PER_METER = 18940.0
TRACK_WIDTH = 0.256

CMD_PER_MPS = 240.0

MIN_CMD = 20
MAX_CMD = 95
MAX_STEP = 5

KP = 6.0
KI = 1.5
I_CLAMP = 15.0

VEL_ALPHA = 0.3

HZ = 20.0
DT = 1.0 / HZ

LOG_INTERVAL = 0.2

# Velocity sanity limit (very generous)
MAX_TICKS_PER_SEC = 40000  

# ============================================================
# I2C setup
# ============================================================

bus = smbus.SMBus(BUS_ID)

def i2c_retry(func, *args, retries=5, delay=0.01):
    for i in range(retries):
        try:
            return func(*args)
        except OSError:
            if i == retries - 1:
                raise
            time.sleep(delay)

i2c_retry(bus.write_byte_data, ADDR, 0x14, 1)
i2c_retry(bus.write_byte_data, ADDR, 0x15, 0)

def read_encoders():
    raw = i2c_retry(bus.read_i2c_block_data, ADDR, 0x3C, 16)
    return struct.unpack('<iiii', bytes(raw))

def set_motors(left, right):
    i2c_retry(
        bus.write_i2c_block_data,
        ADDR,
        0x33,
        [right, right, left, left]
    )

# ============================================================
# Helpers
# ============================================================

def clamp_cmd(u):
    if abs(u) < MIN_CMD:
        return 0
    return max(-MAX_CMD, min(MAX_CMD, int(u)))

def rate_limit(new, old):
    return max(old - MAX_STEP, min(old + MAX_STEP, new))

# ============================================================
# PID
# ============================================================

class SidePID:
    def __init__(self):
        self.i = 0.0

    def reset(self):
        self.i = 0.0

    def update(self, error, dt):
        self.i += error * dt
        self.i = max(-I_CLAMP, min(I_CLAMP, self.i))
        return KP * error + KI * self.i

# ============================================================
# Controller state
# ============================================================

class State(Enum):
    IDLE = 0
    RUNNING = 1
    STOPPING = 2

# ============================================================
# Main controller
# ============================================================

def drive_v_omega(v, omega, duration):
    vL_ref = v - omega * TRACK_WIDTH / 2
    vR_ref = v + omega * TRACK_WIDTH / 2

    u_ff_L = CMD_PER_MPS * vL_ref
    u_ff_R = CMD_PER_MPS * vR_ref

    pid_L = SidePID()
    pid_R = SidePID()

    # --- Initialization ---
    state = State.IDLE

    # encoder baseline
    enc_last = read_encoders()
    t_last = time.monotonic()

    vL_f = 0.0
    vR_f = 0.0

    uL_prev = 0.0
    uR_prev = 0.0

    start_time = time.monotonic()
    last_log = start_time

    # --- Start motors ---
    set_motors(clamp_cmd(u_ff_L), clamp_cmd(u_ff_R))
    state = State.RUNNING

    # reset baseline AFTER motor command
    time.sleep(0.1)
    enc_last = read_encoders()
    t_last = time.monotonic()

    while time.monotonic() - start_time < duration:
        loop_start = time.monotonic()

        enc = read_encoders()
        t_now = time.monotonic()
        dt = t_now - t_last

        if dt <= 0:
            continue

        d = [enc[i] - enc_last[i] for i in range(4)]
        enc_last = enc
        t_last = t_now

        # --- velocity sanity check ---
        for i in range(4):
            ticks_per_sec = abs(d[i] / dt)
            if ticks_per_sec > MAX_TICKS_PER_SEC:
                print(f"\nENCODER RATE FAULT motor={i} rate={ticks_per_sec:.0f} ticks/s")
                state = State.STOPPING
                break

        if state == State.STOPPING:
            break

        # --- velocity estimation ---
        vR_raw = ((d[0] + d[1]) * 0.5) / TICKS_PER_METER / dt
        vL_raw = ((d[2] + d[3]) * 0.5) / TICKS_PER_METER / dt

        vR_f = VEL_ALPHA * vR_raw + (1 - VEL_ALPHA) * vR_f
        vL_f = VEL_ALPHA * vL_raw + (1 - VEL_ALPHA) * vL_f

        # --- control ---
        eL = vL_ref - vL_f
        eR = vR_ref - vR_f

        uL = u_ff_L + pid_L.update(eL, dt)
        uR = u_ff_R + pid_R.update(eR, dt)

        uL = rate_limit(uL, uL_prev)
        uR = rate_limit(uR, uR_prev)

        uL_prev = uL
        uR_prev = uR

        set_motors(clamp_cmd(uL), clamp_cmd(uR))

        # --- logging ---
        if t_now - last_log > LOG_INTERVAL:
            print(
                f"Cmd L:{clamp_cmd(uL):4d} R:{clamp_cmd(uR):4d} | "
                f"Vel L:{vL_f:+.3f} R:{vR_f:+.3f} | "
                f"Ref L:{vL_ref:+.3f} R:{vR_ref:+.3f}"
            )
            last_log = t_now

        # --- deterministic timing ---
        sleep_time = DT - (time.monotonic() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

    # --- controlled stop ---
    for _ in range(10):
        uL_prev *= 0.6
        uR_prev *= 0.6
        set_motors(clamp_cmd(uL_prev), clamp_cmd(uR_prev))
        time.sleep(0.05)

    set_motors(0, 0)

# ============================================================
# Test
# ============================================================

try:
    drive_v_omega(0.25, 0.0, 3.0)
    time.sleep(2.0)
    drive_v_omega(-0.25, 0.0, 3.0)
finally:
    set_motors(0, 0)
