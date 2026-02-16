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

HZ = 1.0
DT = 1.0 / HZ

LOG_INTERVAL = 0.2

# Velocity sanity limit (very generous)
MAX_TICKS_PER_SEC = 40000  

# ============================================================
# I2C setup
# ============================================================

bus = smbus.SMBus(BUS_ID)

# Last-good encoder values and I2C error counter so transient I2C failures
# don't crash the controller and we can monitor error frequency.
_last_good_encoders = (0, 0, 0, 0)
_i2c_error_count = 0
_consec_bad_counts = [0, 0, 0, 0]
BAD_THRESHOLD_CONSEC = 3

def i2c_retry(func, *args, retries=5, delay=0.5, backoff=1.5, **kwargs):
    """Retry I2C operations with exponential backoff.

    Raises the last exception if all retries fail.
    """
    for i in range(retries):
        try:
            return func(*args, **kwargs)
        except (OSError, IOError) as e:
            if i == retries - 1:
                raise
            sleep_time = delay * (backoff ** i)
            time.sleep(sleep_time)

i2c_retry(bus.write_byte_data, ADDR, 0x14, 1)
i2c_retry(bus.write_byte_data, ADDR, 0x15, 0)

def read_encoders():
    """Read all 4 encoders with retries and fallback to last-good values.

    On repeated I2C failures this will return the last-known-good encoder
    tuple and increment an error counter so callers can continue running
    while the issue is diagnosed.
    """
    global _last_good_encoders, _i2c_error_count
    try:
        raw = i2c_retry(bus.read_i2c_block_data, ADDR, 0x3C, 16, retries=8, delay=0.5)
        enc = struct.unpack('<iiii', bytes(raw))
        _last_good_encoders = enc
        return enc
    except Exception as e:
        _i2c_error_count += 1
        print(f"[WARN] read_encoders failed (count={_i2c_error_count}): {e}")
        # small delay to avoid tight error loop
        time.sleep(0.5)
        return _last_good_encoders


def set_motors(left, right):
    """Write motor command, but don't let transient I2C errors crash the program.

    We log failures and increment the I2C error counter so the operator can
    monitor health.
    """
    global _i2c_error_count
    try:
        i2c_retry(
            bus.write_i2c_block_data,
            ADDR,
            0x33,
            [right, right, left, left],
            retries=6,
            delay=0.5,
        )
    except Exception as e:
        _i2c_error_count += 1
        print(f"[WARN] set_motors I2C write failed (count={_i2c_error_count}): {e}")

# ============================================================
# Helpers
# ============================================================

def clamp_cmd(u):
    if abs(u) < MIN_CMD:
        return 0
    return max(-MAX_CMD, min(MAX_CMD, int(u)))

def rate_limit(new, old):
    return max(old - MAX_STEP, min(old + MAX_STEP, new))


def _delta_with_wrap(new, old):
    """Compute 32-bit wrap-safe difference between encoder counts.

    Handles signed or unsigned 32-bit counters by operating on the
    32-bit unsigned representation and returning a signed delta.
    """
    new_u = new & 0xFFFFFFFF
    old_u = old & 0xFFFFFFFF
    diff = (new_u - old_u) & 0xFFFFFFFF
    if diff & 0x80000000:
        diff -= 0x100000000
    return diff

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
    time.sleep(0.5)
    enc_last = read_encoders()
    t_last = time.monotonic()

    while time.monotonic() - start_time < duration:
        loop_start = time.monotonic()

        enc = read_encoders()
        t_now = time.monotonic()
        dt = t_now - t_last

        if dt <= 0:
            continue

        raw_d = [_delta_with_wrap(enc[i], enc_last[i]) for i in range(4)]
        new_enc_last = list(enc_last)
        d = [0, 0, 0, 0]
        # --- velocity sanity check with per-motor consecutive bad detection ---
        for i in range(4):
            ticks_per_sec = abs(raw_d[i] / dt)
            if ticks_per_sec > MAX_TICKS_PER_SEC:
                _consec_bad_counts[i] += 1
                if _consec_bad_counts[i] < BAD_THRESHOLD_CONSEC:
                    print(f"[WARN] transient large encoder delta motor={i} rate={ticks_per_sec:.0f} ticks/s (count={_consec_bad_counts[i]})")
                    print(f"[DEBUG] enc={enc} enc_last={enc_last} raw_d={raw_d[i]} dt={dt:.6f} i2c_err={_i2c_error_count}")
                    # ignore this sample for the motor (keep old enc_last)
                    d[i] = 0
                    continue
                else:
                    print(f"\nENCODER RATE FAULT motor={i} rate={ticks_per_sec:.0f} ticks/s (count={_consec_bad_counts[i]})")
                    print(f"[DEBUG] enc={enc} enc_last={enc_last} raw_d={raw_d[i]} dt={dt:.6f} i2c_err={_i2c_error_count}")
                    state = State.STOPPING
                    break
            else:
                # good reading, accept it
                _consec_bad_counts[i] = 0
                d[i] = raw_d[i]
                new_enc_last[i] = enc[i]

        enc_last = tuple(new_enc_last)
        t_last = t_now

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
        time.sleep(0.5)

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
