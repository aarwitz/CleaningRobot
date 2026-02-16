#!/usr/bin/env python3
"""
Encoder diagnostic with smooth motor movement.

Ramps forward 0 → 100 → 0 (S-curve), pauses, then backward 0 → -100 → 0.
Uses fixed-rate loop with monotonic clock. Each tick: compute target → rate-limit → add PID
jitter → write motors → read encoders.

Minimum nonzero command: 40 (motor dead-zone floor).
"""

import smbus
import time
import struct
import math
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

# ── Timing (matches motor_controller_node: control_rate=20) ─
LOOP_HZ = 5             # Fixed command rate (Hz) - slowed down for I2C reliability
DT = 1.0 / LOOP_HZ      # 200 ms per tick

# ── Motion profile ──────────────────────────────────────────
CMD_MAX = 100            # Peak motor command
MIN_CMD = 40             # Minimum nonzero command (dead-zone floor)
MAX_STEP = 10            # Max command change per tick (was 5, but that's too slow from standstill)
RAMP_TIME = 1.5          # Seconds for S-curve ramp 0 → CMD_MAX
HOLD_TIME = 2.0          # Seconds to hold at peak
PAUSE_TIME = 2.0         # Seconds between forward and backward

# ── Diagnostics ─────────────────────────────────────────────
MAX_REASONABLE_DELTA = 10000   # Encoder ticks per loop — spike threshold
PRINT_INTERVAL = 5             # Print every Nth tick (~250 ms at 20 Hz)
PID_VARIATION = 2              # ± per cycle (simulates PID corrections)
ENCODER_ABS_SANITY_LIMIT = 2_000_000
WARMUP_TIME = 0.6

bus = smbus.SMBus(BUS_ID)

# Last-good encoder fallback (mirrors nav2_compatible_velocity_controller)
_last_good_encoders = (0, 0, 0, 0)
_i2c_errors = 0

# ─── I2C helpers ────────────────────────────────────────────

def i2c_retry(func, *args, retries=5, delay=0.02):
    """Retry I2C with backoff — matches motor_controller_node (5×20 ms).

    Worst-case 100 ms. If it still fails the caller can decide whether
    to crash or use a fallback.
    """
    for attempt in range(retries):
        try:
            result = func(*args)
            # Small settling delay after successful I2C transaction
            time.sleep(0.005)
            return result
        except (OSError, TimeoutError):
            if attempt == retries - 1:
                raise
            time.sleep(delay)


def read_encoders():
    """Read encoders with fallback to last-known-good on I2C failure."""
    global _last_good_encoders, _i2c_errors
    try:
        raw = i2c_retry(bus.read_i2c_block_data, ADDR, 0x3C, 16)
        enc = struct.unpack('<iiii', bytes(raw))
        _last_good_encoders = enc
        return enc
    except (OSError, TimeoutError) as e:
        _i2c_errors += 1
        print(f"  [WARN] encoder read failed (#{_i2c_errors}): {e}")
        return _last_good_encoders


def _encoder_frame_plausible(enc, prev=None):
    if any(v == -1 for v in enc):
        return False
    if any(abs(v) > ENCODER_ABS_SANITY_LIMIT for v in enc):
        return False
    if prev is not None and any(abs(enc[i] - prev[i]) > MAX_REASONABLE_DELTA for i in range(4)):
        return False
    return True


def read_encoders_robust(prev=None, retries=3):
    """Read encoders and reject obviously corrupted frames.

    Returns last good sample if all retries are implausible.
    """
    global _i2c_errors
    fallback = prev if prev is not None else _last_good_encoders
    for attempt in range(retries):
        enc = read_encoders()
        if _encoder_frame_plausible(enc, prev):
            return enc
        _i2c_errors += 1
        print(
            f"  [WARN] encoder frame invalid (#{_i2c_errors}, try {attempt + 1}/{retries}): {enc}"
        )
        time.sleep(0.01)
    return fallback


def set_motors(left, right):
    """Write motor command; log failures but don't crash."""
    global _i2c_errors
    try:
        i2c_retry(bus.write_i2c_block_data, ADDR, 0x33,
                  [right, right, left, left])
    except (OSError, TimeoutError) as e:
        _i2c_errors += 1
        print(f"  [WARN] motor write failed (#{_i2c_errors}): {e}")


def stop_immediate():
    """Hard stop — no ramp, for emergencies and cleanup."""
    try:
        i2c_retry(bus.write_i2c_block_data, ADDR, 0x33, [0, 0, 0, 0],
                  retries=10, delay=0.02)
    except OSError:
        pass


# ─── Motion math ────────────────────────────────────────────

def clamp(value):
    """Dead-zone–aware clamp (matches motor_controller_node._clamp_cmd).

    |value| < MIN_CMD → 0   (below dead zone — don't stall the motors)
    otherwise           → clip to ±CMD_MAX
    """
    if abs(value) < MIN_CMD:
        return 0
    return max(-CMD_MAX, min(CMD_MAX, int(value)))


def smart_rate_limit(target, prev):
    """Rate-limit that respects motor dead zone.
    
    If ramping toward a nonzero target, ensures output is either 0 or >= MIN_CMD.
    This prevents sending sub-threshold commands that cause motor stuttering.
    """
    # Standard rate limiting
    limited = max(prev - MAX_STEP, min(prev + MAX_STEP, target))
    
    # If we're transitioning from 0 to nonzero, jump directly to MIN_CMD
    # (motors can't move below this anyway, so ramping through 5, 10, 15 is pointless)
    if prev == 0 and target != 0:
        if target > 0:
            return max(MIN_CMD, min(limited, target))
        else:
            return min(-MIN_CMD, max(limited, target))
    
    # If limited result falls in dead zone but target is outside it, clamp to MIN_CMD
    if 0 < abs(limited) < MIN_CMD:
        if limited > 0:
            return MIN_CMD if target >= MIN_CMD else 0
        else:
            return -MIN_CMD if target <= -MIN_CMD else 0
    
    return int(limited)


def s_curve(t, duration):
    """Cosine ease-in-out:  t ∈ [0, duration] → [0.0, 1.0]."""
    p = max(0.0, min(1.0, t / duration))
    return 0.5 * (1.0 - math.cos(math.pi * p))


def profile_command(elapsed, direction):
    """Compute target motor command for elapsed time within one phase.

    Profile:  ramp up    →  hold     →  ramp down
              RAMP_TIME     HOLD_TIME    RAMP_TIME
    """
    if elapsed < RAMP_TIME:
        frac = s_curve(elapsed, RAMP_TIME)
    elif elapsed < RAMP_TIME + HOLD_TIME:
        frac = 1.0
    elif elapsed < 2 * RAMP_TIME + HOLD_TIME:
        frac = 1.0 - s_curve(elapsed - RAMP_TIME - HOLD_TIME, RAMP_TIME)
    else:
        frac = 0.0

    raw = int(round(frac * CMD_MAX)) * direction
    return clamp(raw)


# ─── Phase runner ───────────────────────────────────────────

def run_phase(label, direction):
    """Execute one motion phase at a fixed loop rate.

    Each tick:
      1. Compute target from S-curve profile
      2. Add small PID-like jitter
      3. Clamp to dead zone
      4. Smart rate-limit (prevents sub-threshold commands)
      5. Write motors  (I2C write)
      6. Read encoders (I2C read)

    Timing uses monotonic clock so I2C latency / retries don't cause drift.
    """
    phase_duration = 2 * RAMP_TIME + HOLD_TIME

    print(f"\n{'='*60}")
    print(f"  {label}")
    print(f"  Ramp {RAMP_TIME}s → Hold {HOLD_TIME}s → Ramp {RAMP_TIME}s")
    print(f"  {LOOP_HZ} Hz | Peak ±{CMD_MAX} | Dead zone <{MIN_CMD}"
          f" | Step ±{MAX_STEP}/tick")
    print(f"{'='*60}")

    warmup_cmd = 60 if direction > 0 else -60
    print(f"  Direction warmup at CMD={warmup_cmd} for {WARMUP_TIME:.1f}s (no stop transition)")
    set_motors(warmup_cmd, warmup_cmd)
    time.sleep(WARMUP_TIME)

    enc_prev = read_encoders_robust(prev=None, retries=8)
    print(f"  Baseline (moving): {enc_prev}")

    # Seed the fallback so it has a valid baseline
    global _last_good_encoders
    _last_good_encoders = enc_prev
    
    prev_left = warmup_cmd
    prev_right = warmup_cmd
    t_origin = time.monotonic()
    next_tick = t_origin + DT
    tick = 0
    overruns = 0
    work_max_ms = 0.0       # track worst-case tick work time

    while True:
        tick_start = time.monotonic()
        elapsed = tick_start - t_origin
        if elapsed >= phase_duration:
            break

        # 1. Target from S-curve profile
        target = profile_command(elapsed, direction)

        # 2. Add small PID-like jitter to the target
        if target != 0:
            left_target = target + random.randint(-PID_VARIATION, PID_VARIATION)
            right_target = target + random.randint(-PID_VARIATION, PID_VARIATION)
        else:
            left_target = 0
            right_target = 0

        # 3. Clamp to respect dead zone (before rate limiting)
        left_clamped = clamp(left_target)
        right_clamped = clamp(right_target)

        # 4. Smart rate-limit that prevents sub-threshold commands
        left_cmd = smart_rate_limit(left_clamped, prev_left)
        right_cmd = smart_rate_limit(right_clamped, prev_right)

        # 5. I2C write motors
        set_motors(left_cmd, right_cmd)
        
        # Critical: allow I2C bus to settle between write and read
        time.sleep(0.020)
        
        # 6. I2C read encoders
        enc = read_encoders_robust(prev=enc_prev, retries=3)

        # Track actual sent commands for next tick's rate-limit
        prev_left = left_cmd
        prev_right = right_cmd

        # ── Encoder diagnostics ──
        d = [enc[i] - enc_prev[i] for i in range(4)]
        spikes = [f"ENC{i}:SPIKE" for i in range(4)
                  if abs(d[i]) > MAX_REASONABLE_DELTA]
        
        # Track which encoders are spiking (for persistent issues)
        if spikes:
            spike_msg = f" | Absolute values: ENC=[{enc[0]:+8d} {enc[1]:+8d} {enc[2]:+8d} {enc[3]:+8d}]"
        else:
            spike_msg = ""

        work_ms = (time.monotonic() - tick_start) * 1000
        work_max_ms = max(work_max_ms, work_ms)

        # Print on regular intervals OR when spikes are detected
        should_print = (tick % PRINT_INTERVAL == 0) or spikes
        
        if should_print:
            print(
                f"  t={elapsed:5.2f}s  tgt={target:+4d}  "
                f"L={left_cmd:+4d} R={right_cmd:+4d}  "
                f"Δ=[{d[0]:+6d} {d[1]:+6d} {d[2]:+6d} {d[3]:+6d}]  "
                f"{'  '.join(spikes) if spikes else 'ok'}{spike_msg}"
            )

        if spikes:
            print("  !! SPIKE — emergency stop !!")
            print("  HINT: Check encoder wiring/connections for the failing encoder(s)")
            stop_immediate()
            return False

        enc_prev = enc
        tick += 1

        # ── Fixed-rate sleep (absorbs I2C jitter) ──
        sleep_time = next_tick - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            overruns += 1
        next_tick += DT

    set_motors(0, 0)
    total = time.monotonic() - t_origin
    print(f"  ✓ Done — {tick} ticks in {total:.2f}s"
          f"  (overruns: {overruns},"
          f" worst tick: {work_max_ms:.1f}ms / {DT*1000:.0f}ms budget,"
          f" i2c errors: {_i2c_errors})")
    return True


# ─── Main ───────────────────────────────────────────────────

try:
    print()
    print("╔══════════════════════════════════════════════════╗")
    print("║      Encoder Motion Diagnostic — PID Sim        ║")
    print("╠══════════════════════════════════════════════════╣")
    print(f"║  Loop: {LOOP_HZ:2d} Hz    Ramp: {RAMP_TIME}s    Hold: {HOLD_TIME}s       ║")
    print(f"║  Peak: {CMD_MAX:3d}       Min: {MIN_CMD:2d}       Step: ±{MAX_STEP}         ║")
    print("╚══════════════════════════════════════════════════╝")
    print("\n  Robot will move — keep area clear.\n")

    # Init motor driver
    i2c_retry(bus.write_byte_data, ADDR, 0x14, 1)
    i2c_retry(bus.write_byte_data, ADDR, 0x15, 0)
    time.sleep(0.1)  # Let motor driver stabilize

    ok = run_phase("FORWARD   0 → +100 → 0", direction=+1)

    if ok:
        print(f"\n  Pausing {PAUSE_TIME}s ...")
        time.sleep(PAUSE_TIME)
        run_phase("BACKWARD  0 → -100 → 0", direction=-1)

    print("\n  Test complete.\n")

finally:
    stop_immediate()
