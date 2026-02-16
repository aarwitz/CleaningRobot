#!/usr/bin/env python3
"""
Stable smooth encoder diagnostic.

No nested I2C retries.
Exactly ONE write and ONE read per control tick.
Failures are skipped, not amplified.
"""

import smbus
import time
import struct
import math
import random

# ============================================================
# CONFIG
# ============================================================

BUS_ID = 7
ADDR = 0x34

LOOP_HZ = 5
DT = 1.0 / LOOP_HZ

CMD_MAX = 100
MIN_CMD = 40
MAX_STEP = 10

RAMP_TIME = 1.5
HOLD_TIME = 2.0
PAUSE_TIME = 2.0

MAX_REASONABLE_DELTA = 10000
ENCODER_ABS_SANITY_LIMIT = 2_000_000

PID_VARIATION = 2

bus = smbus.SMBus(BUS_ID)

_last_good_enc = (0, 0, 0, 0)
_i2c_errors = 0

# ============================================================
# LOW-LEVEL I2C (NO RETRY CASCADE)
# ============================================================

def write_motors_once(l, r):
    global _i2c_errors
    try:
        bus.write_i2c_block_data(ADDR, 0x33, [r, r, l, l])
        return True
    except (OSError, TimeoutError):
        _i2c_errors += 1
        return False


def read_encoders_once():
    global _i2c_errors, _last_good_enc
    try:
        raw = bus.read_i2c_block_data(ADDR, 0x3C, 16)
        enc = struct.unpack('<iiii', bytes(raw))
        _last_good_enc = enc
        return enc, True
    except (OSError, TimeoutError):
        _i2c_errors += 1
        return _last_good_enc, False


def stop():
    try:
        bus.write_i2c_block_data(ADDR, 0x33, [0, 0, 0, 0])
    except:
        pass


# ============================================================
# MOTION UTILITIES
# ============================================================

def clamp(value):
    if abs(value) < MIN_CMD:
        return 0
    return max(-CMD_MAX, min(CMD_MAX, int(value)))


def smart_rate_limit(target, prev):
    limited = max(prev - MAX_STEP, min(prev + MAX_STEP, target))
    if prev == 0 and target != 0:
        return MIN_CMD if target > 0 else -MIN_CMD
    if 0 < abs(limited) < MIN_CMD:
        return MIN_CMD if limited > 0 else -MIN_CMD
    return int(limited)


def s_curve(t, duration):
    p = max(0.0, min(1.0, t / duration))
    return 0.5 * (1.0 - math.cos(math.pi * p))


def profile(elapsed, direction):
    if elapsed < RAMP_TIME:
        frac = s_curve(elapsed, RAMP_TIME)
    elif elapsed < RAMP_TIME + HOLD_TIME:
        frac = 1.0
    elif elapsed < 2 * RAMP_TIME + HOLD_TIME:
        frac = 1.0 - s_curve(elapsed - RAMP_TIME - HOLD_TIME, RAMP_TIME)
    else:
        frac = 0.0
    return clamp(int(round(frac * CMD_MAX)) * direction)


# ============================================================
# PHASE RUNNER
# ============================================================

def run_phase(label, direction):

    print(f"\n===== {label} =====")

    phase_duration = 2 * RAMP_TIME + HOLD_TIME

    # Warmup at constant direction
    warm = 60 if direction > 0 else -60
    write_motors_once(warm, warm)
    time.sleep(0.6)

    enc_prev, _ = read_encoders_once()
    prev_left = warm
    prev_right = warm

    t_origin = time.monotonic()
    next_tick = t_origin + DT

    tick = 0
    overruns = 0

    while True:

        now = time.monotonic()
        elapsed = now - t_origin
        if elapsed >= phase_duration:
            break

        target = profile(elapsed, direction)

        if target != 0:
            lt = target + random.randint(-PID_VARIATION, PID_VARIATION)
            rt = target + random.randint(-PID_VARIATION, PID_VARIATION)
        else:
            lt, rt = 0, 0

        lt = clamp(lt)
        rt = clamp(rt)

        left_cmd = smart_rate_limit(lt, prev_left)
        right_cmd = smart_rate_limit(rt, prev_right)

        # ONE write per tick
        write_motors_once(left_cmd, right_cmd)

        # Small deterministic separation
        time.sleep(0.01)

        # ONE read per tick
        enc, ok = read_encoders_once()

        if ok:
            # Only evaluate deltas if read succeeded
            d = [enc[i] - enc_prev[i] for i in range(4)]

            if any(abs(v) > MAX_REASONABLE_DELTA for v in d):
                print(f"SPIKE detected Δ={d}")
                stop()
                return

            enc_prev = enc

        prev_left = left_cmd
        prev_right = right_cmd

        if tick % 5 == 0:
            print(
                f"t={elapsed:4.2f}s tgt={target:+4d} "
                f"L={left_cmd:+4d} R={right_cmd:+4d} "
                f"{'OK' if ok else 'READ_FAIL'}"
            )

        sleep_time = next_tick - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            overruns += 1

        next_tick += DT
        tick += 1

    write_motors_once(0, 0)
    print(f"Done. Overruns={overruns}, I2C errors={_i2c_errors}")


# ============================================================
# MAIN
# ============================================================

try:
    print("\n=== Stable Smooth Encoder Diagnostic ===\n")

    write_motors_once(0, 0)
    time.sleep(0.1)

    run_phase("FORWARD 0→100→0", +1)

    time.sleep(PAUSE_TIME)

    run_phase("BACKWARD 0→-100→0", -1)

    print("\nTest complete.\n")

finally:
    stop()
