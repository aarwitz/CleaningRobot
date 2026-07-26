#!/usr/bin/env python3
"""Bearing from vision, radius by touch, then wide-open sweep — one run.

Pipeline for a ball that defeats the depth camera (rubber surface = 90%+
depth holes; near-cluster falls onto the background) and sits inside the
blind zone anyway:
  1. observe pose -> DINO if the RSL tunnel is up, else local HSV blob
     -> BEARING (the px->bearing axis is reliable; range is not)
  2. probe-walk OUTWARD along that bearing: slow closed-claw descents watching
     measured-z lag. Vertical probes never displace the ball (validated).
     First contact meaningfully above the empty/floor band = ball.
  3. immediately: rise, open WIDE (0.35), park on the +y side, sweep in while
     closing to 2.60 (operator technique; close deeper just squeezes the
     bands). Held = stall angle < 2.10 (empty-with-bands stalls at 2.25).

  python3 /scripts/ball_scan_pick.py [--r0 320] [--r1 430] [--keep]
"""
import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sock_cycle
sock_cycle.GRIP_OPEN = 0.35
import dino_client
from ball_pick import BallPick, BALL_PROMPT, OBSERVE
from sock_cycle import GRIP_CLOSED

WIDE = 0.35
CLOSE = 2.60
HELD_MAX = 2.10          # empty close with bands stalls at 2.250
FLOOR_BAND = -180.0


def band_held(c):
    a = (c.state or {}).get('arm') or {}
    ta = a.get('t')
    return (ta is not None and ta < HELD_MAX), (ta or 9.9), abs(a.get('torH') or 0)


def detect_bearing(c):
    """Ball bearing (rad) from the observe pose, DINO-first."""
    c.move(*OBSERVE, t=GRIP_CLOSED, speed=100.0)
    c.spin(0.6)
    cv2.imwrite('/tmp/_scan_frame.png', c.img)
    box = None
    try:
        dets = dino_client.detect('/tmp/_scan_frame.png', BALL_PROMPT,
                                  confidence=0.25)
        if dets:
            box = dets[0]['box']
            print(f'  [vision] DINO box {[round(v) for v in box]}')
    except Exception:
        pass
    if box is None:
        box = c.detect_orange_local()
        if box is not None:
            print(f'  [vision] local HSV box {[round(v) for v in box]}')
    if box is None:
        return None
    u = (box[0] + box[2]) / 2.0
    fx, cx = (c.K[0], c.K[2]) if c.K else (385.0, 318.7)
    # px -> bearing; camera y-offset -25mm folds into a small bearing bias
    return -math.atan2(u - cx, fx)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--r0', type=float, default=320.0)
    ap.add_argument('--r1', type=float, default=430.0)
    ap.add_argument('--step', type=float, default=18.0)
    ap.add_argument('--keep', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    c = BallPick()
    if not c.wait_ready():
        print('FAIL: teleop not ready')
        return 1
    c.wait_depth(timeout=3.0)   # intrinsics if available; not fatal

    th = detect_bearing(c)
    if th is None:
        print('FAIL: no ball detection at observe pose')
        return 2
    print(f'bearing {math.degrees(th):.1f} deg')

    hit = None
    r = a.r0
    while r <= a.r1:
        x, y = r * math.cos(th), r * math.sin(th)
        cz = c.touch_probe(x, y, z_start=-70.0, z_floor_wrist=FLOOR_BAND)
        tag = f'contact {cz:.0f}' if cz is not None else 'clean'
        print(f'  probe r={r:.0f} ({x:.0f},{y:.0f}) -> {tag}')
        c.move(x, y, -70.0, GRIP_CLOSED, speed=80.0)
        if cz is not None and cz > FLOOR_BAND + 14.0:
            # walk a touch further to find the dome peak (=center)
            best = (x, y, cz)
            for rr in (r + 14.0, r + 28.0):
                xx, yy = rr * math.cos(th), rr * math.sin(th)
                czz = c.touch_probe(xx, yy, z_start=-70.0,
                                    z_floor_wrist=FLOOR_BAND)
                print(f'  peak-walk r={rr:.0f} -> '
                      f'{"contact %.0f" % czz if czz is not None else "clean"}')
                c.move(xx, yy, -70.0, GRIP_CLOSED, speed=80.0)
                if czz is not None and czz > best[2]:
                    best = (xx, yy, czz)
            hit = best
            break
        r += a.step
    if hit is None:
        print('SCAN FAILED: no contact along bearing')
        c.destroy_node(); rclpy.shutdown()
        return 2

    bx, by, cz = hit
    gz = -178.0
    print(f'BALL at ({bx:.0f},{by:.0f}) (peak contact {cz:.0f}); '
          f'wide sweep from +y, gz {gz}')
    c.move(bx, by + 60.0, gz + 110.0, GRIP_CLOSED, speed=90.0)
    c.grip(WIDE, secs=1.2)
    c.move(bx, by + 60.0, gz + 25.0, speed=55.0, settle=False)
    c.sweep_in(bx, by, gz, 60.0, 10.0, CLOSE, close_start=0.12, z_blend=25.0)
    ok, ta, torh = band_held(c)
    print(f'close : held={ok} stall={ta:.3f} torH={torh}')
    if ok:
        c.move(bx, by - 10.0, gz + 40.0, speed=25.0)
        ok, ta, torh = band_held(c)
        print(f'lift40: held={ok} stall={ta:.3f} torH={torh}')
    if ok:
        c.move(bx, by - 10.0, gz + 120.0, speed=30.0)
        c.spin(0.6)
        ok, ta, torh = band_held(c)
        print(f'liftup: held={ok} stall={ta:.3f} torH={torh}')
    print('SCAN PICK ' + ('OK -- HOLDING' if ok else 'FAILED'))
    if ok and not a.keep:
        c.move(bx, by, gz + 8.0, speed=35.0)
        c.grip(WIDE, secs=1.0)
        c.move(bx, by, gz + 110.0, speed=80.0)
    c.destroy_node(); rclpy.shutdown()
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
