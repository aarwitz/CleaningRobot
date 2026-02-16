#!/usr/bin/env python3

import smbus
import time
import struct
import random

# ============================================================
# CONFIG
# ============================================================

BUS_ID = 7
ADDR = 0x34

CMD = 60
LOOP_HZ = 20
DT = 1.0 / LOOP_HZ

TEST_TIME = 5.0
MAX_REASONABLE_DELTA = 10000

PHASE_SWEEP_MS = list(range(0, 101, 5))

bus = smbus.SMBus(BUS_ID)

# ============================================================
# RAW I2C (NO RETRIES)
# ============================================================

def read_encoders_raw():
    raw = bus.read_i2c_block_data(ADDR, 0x3C, 16)
    return struct.unpack('<iiii', bytes(raw))

def set_motors(l, r):
    bus.write_i2c_block_data(ADDR, 0x33, [r, r, l, l])

def stop():
    try:
        set_motors(0, 0)
    except Exception:
        pass

# ============================================================
# EXPERIMENT 1 — Atomicity Killer Test
# ============================================================

def atomicity_test():

    print("\n============================================================")
    print("EXPERIMENT 1 — Atomicity Killer Test (double-read hammer)")
    print("============================================================")

    set_motors(CMD, CMD)
    time.sleep(2.0)

    mismatch = 0
    total = 0
    read_err = 0

    t0 = time.monotonic()
    while time.monotonic() - t0 < TEST_TIME:

        try:
            t1 = time.monotonic()
            enc1 = read_encoders_raw()
            t2 = time.monotonic()
            enc2 = read_encoders_raw()
            t3 = time.monotonic()
        except OSError:
            read_err += 1
            continue

        if enc1 != enc2:
            mismatch += 1

        total += 1

    stop()

    print(f"Total reads: {total}")
    print(f"Atomic mismatches: {mismatch}")
    print(f"Read errors: {read_err}")

# ============================================================
# EXPERIMENT 2 — Read-Only Phase Sweep
# Removes motor writes inside loop
# ============================================================

def read_only_phase_sweep():

    print("\n============================================================")
    print("EXPERIMENT 2 — Read-Only Phase Sweep")
    print("============================================================")

    set_motors(CMD, CMD)
    time.sleep(2.0)

    for phase_ms in PHASE_SWEEP_MS:

        phase_delay = phase_ms / 1000.0
        print(f"\n>>> Phase offset: {phase_ms} ms")

        spikes = 0
        read_err = 0
        samples = 0

        try:
            enc_prev = read_encoders_raw()
        except OSError:
            print("Initial read failed.")
            continue

        start = time.monotonic()
        next_tick = start

        while time.monotonic() - start < TEST_TIME:

            now = time.monotonic()
            sleep_time = next_tick - now
            if sleep_time > 0:
                time.sleep(sleep_time)

            next_tick += DT

            if phase_delay > 0:
                time.sleep(phase_delay)

            try:
                enc = read_encoders_raw()
            except OSError:
                read_err += 1
                continue

            d = [enc[i] - enc_prev[i] for i in range(4)]
            if any(abs(v) > MAX_REASONABLE_DELTA for v in d):
                spikes += 1

            enc_prev = enc
            samples += 1

        print(f"Samples={samples} | Spikes={spikes} | ReadErr={read_err}")

    stop()

# ============================================================
# EXPERIMENT 3 — Write+Read Collision Test
# Tests whether motor writes are causing timeout
# ============================================================

def write_read_collision_test():

    print("\n============================================================")
    print("EXPERIMENT 3 — Write + Read Collision Stress Test")
    print("============================================================")

    set_motors(CMD, CMD)
    time.sleep(2.0)

    spikes = 0
    read_err = 0
    write_err = 0
    samples = 0

    start = time.monotonic()
    next_tick = start

    while time.monotonic() - start < TEST_TIME:

        now = time.monotonic()
        sleep_time = next_tick - now
        if sleep_time > 0:
            time.sleep(sleep_time)

        next_tick += DT

        try:
            set_motors(CMD + random.randint(-3, 3),
                       CMD + random.randint(-3, 3))
        except OSError:
            write_err += 1

        time.sleep(0.002)  # extremely tight spacing

        try:
            enc = read_encoders_raw()
        except OSError:
            read_err += 1
            continue

        samples += 1

    stop()

    print(f"Samples={samples} | WriteErr={write_err} | ReadErr={read_err}")

# ============================================================
# MAIN
# ============================================================

try:
    print("\n=== I2C Root Cause Probe ===")
    print("Robot will move slowly. Keep space clear.\n")

    atomicity_test()

    print("\nPausing...\n")
    stop()
    time.sleep(3.0)

    read_only_phase_sweep()

    print("\nPausing...\n")
    stop()
    time.sleep(3.0)

    write_read_collision_test()

    print("\nProbe complete.\n")

finally:
    stop()
