#!/usr/bin/env python3
"""The full-infrastructure ball pick. No fallbacks, no guesses:

  1. GroundingDINO on RSL (through the tunnel) — REQUIRED, fails loudly.
  2. RealSense aligned depth, near-cluster median inside the bbox — REQUIRED.
     (Near cluster because the rubber surface leaves >90% holes and a plain
     median falls through onto the background.)
  3. Pick point = slightly BEYOND the measured surface: depth + BALL_R along
     the camera ray = the object's true 3D centroid. cam->arm via the
     measured hand-eye (TX=+75 operator-validated, TY=-25).
  4. The end effector OPENS WIDE (0.35) DURING the transit to the approach
     point on the +y (robot-left) side — verified open before descending.
  5. Descend beside the ball, then sweep in from the left WHILE closing to
     2.60: the moving jaw rolls the ball into the fixed jaw without the claw
     ever descending onto it.
  6. Held = stall angle < 2.10 (empty close with rubber bands stalls at 2.25,
     so torque/gap are blind). Two-stage slow lift, re-verified at each stage.

  python3 /scripts/ball_rsl_pick.py [--keep] [--grasp-z -178]
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
from ball_pick import BallPick, BALL_PROMPT, BALL_R, OBSERVE, load_cal
from sock_cycle import GRIP_CLOSED

WIDE = 0.35
CLOSE = 2.60
HELD_STALL_MAX = 2.10


def band_held(c):
    a = (c.state or {}).get('arm') or {}
    ta = a.get('t')
    return (ta is not None and ta < HELD_STALL_MAX), (ta or 9.9), \
        abs(a.get('torH') or 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--keep', action='store_true')
    ap.add_argument('--grasp-z', type=float, default=-178.0)
    ap.add_argument('--approach-dy', type=float, default=60.0)
    ap.add_argument('--overshoot', type=float, default=10.0)
    ap.add_argument('--beyond', type=float, default=0.0,
                    help='extra mm along the bearing past the centroid; at '
                         'near-horizontal stretch (r>380) the pocket lands '
                         'short of the commanded wrist point')
    a = ap.parse_args()

    rclpy.init()
    c = BallPick()
    if not c.wait_ready():
        print('ABORT: teleop/camera not ready')
        return 1
    if not c.wait_depth():
        print('ABORT: no aligned depth / camera_info')
        return 1
    tx, ty = load_cal()
    print(f'hand-eye TX={tx:.0f} TY={ty:.0f}')

    # ── 1. observe + DINO (RSL) ─────────────────────────────────────────────
    c.move(*OBSERVE, t=GRIP_CLOSED, speed=100.0)
    c.spin(0.7)
    cv2.imwrite('/tmp/_rsl_frame.png', c.img)
    try:
        # ROI = lower frame: the pick target is on the floor near the robot;
        # without it DINO can win on background clutter (a plush dog toy at
        # 1.2m once outscored context: box picked at py 204)
        dets = dino_client.detect('/tmp/_rsl_frame.png', BALL_PROMPT,
                                  confidence=0.25, roi=(16, 260, 624, 480))
    except Exception as e:
        print(f'ABORT: RSL GroundingDINO unreachable: {e}')
        return 2
    if not dets:
        print('ABORT: DINO found no ball')
        return 2
    box = dets[0]['box']
    score = dets[0]['score']
    u, v = int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)
    print(f'DINO: box {[round(b) for b in box]} score {score:.2f} '
          f'center px ({u},{v})')

    # ── 2. real depth: near-cluster median inside the bbox ─────────────────
    x0, y0, x1, y1 = (int(t) for t in box)
    d = c.depth[max(0, y0):y1, max(0, x0):x1].astype(float)
    valid = d[(d > 100) & (d < 4000)]
    valid_frac = valid.size / max(d.size, 1)
    surf = None
    if valid.size >= 20 and valid_frac >= 0.05:
        near = valid[valid <= np.percentile(valid, 50)]
        surf = float(np.median(near))
        print(f'depth: {valid.size}/{d.size} valid px '
              f'({100*valid_frac:.0f}%), surface {surf:.0f}mm '
              f'(near-cluster of {near.size})')
    else:
        print(f'depth: {valid.size}/{d.size} valid px '
              f'({100*valid_frac:.1f}%) -- too sparse to trust')
    if surf is None:
        # Blind-zone regime: the ball itself returns nothing and the few
        # valid pixels are the background leaking through -- the reading is
        # a lie, and the object is CLOSE (<~300mm from camera). Use the DINO
        # bearing and find the radius by touch instead of trusting it.
        fx, _, cx, _ = c.K
        th_b = -math.atan2(u - cx, fx)
        print(f'depth is background leak (blind zone). Probe-walking along '
              f'DINO bearing {math.degrees(th_b):.1f} deg')
        hit = None
        for rr in (260.0, 288.0, 316.0, 344.0):
            # ty (camera y-offset) shifts the whole ray laterally
            px_, py_ = rr * math.cos(th_b), rr * math.sin(th_b) + ty
            cz = c.touch_probe(px_, py_, z_start=-70.0, z_floor_wrist=-180.0)
            tag = f'contact {cz:.0f}' if cz is not None else 'clean'
            print(f'  probe r={rr:.0f} ({px_:.0f},{py_:.0f}) -> {tag}')
            c.move(px_, py_, -70.0, GRIP_CLOSED, speed=80.0)
            if cz is not None and cz > -166.0:
                hit = (px_, py_)
                break
        if hit is None:
            print('ABORT: no touch contact along bearing; not guessing')
            return 3
        bx, by = hit
        print(f'ball by touch at ({bx:.0f},{by:.0f})')

    # ── 3. centroid: the near-cluster MEDIAN already sits mid-dome (the
    # sphere's curvature spreads valid pixels across its depth), so it IS
    # approximately the centroid depth. Adding +BALL_R double-counted and
    # made every pick land a few cm BEHIND the ball (operator-diagnosed).
    fx, fy, cx, cy = c.K
    if surf is not None:
        zc = surf
        xc = (u - cx) * zc / fx
        bx = zc + tx
        by = -xc + ty
    else:
        xc, zc = -by, bx    # already set by the touch probe branch
    if a.beyond:
        rr = math.hypot(bx, by)
        bx, by = bx * (rr + a.beyond) / rr, by * (rr + a.beyond) / rr
        print(f'  +{a.beyond:.0f}mm beyond -> ({bx:.0f},{by:.0f})')
    r = math.hypot(bx, by)
    print(f'centroid: cam ({xc:.0f},{zc:.0f}) -> arm ({bx:.0f},{by:.0f}) '
          f'r={r:.0f}')
    c.publish_flywheel(
        c.img, {'stage': 'localized', 'label': 'ball (GroundingDINO/RSL)',
                'score': score, 'box': [round(t) for t in box],
                'depth_mm': surf, 'valid_px': int(valid.size),
                'pick_arm': [round(bx), round(by), round(a.grasp_z)],
                'prompt': BALL_PROMPT},
        box=box, pick_px=(u, v))
    if not (200.0 <= r <= 445.0):
        print(f'ABORT: r={r:.0f} outside the reachable pick band; '
              'NOT driving the base. Re-stage the ball or tell me to move.')
        return 3

    # ── 4. open WIDE during the transit to the +y approach point ───────────
    gz = a.grasp_z
    c.move(bx, by + a.approach_dy, gz + 110.0, t=WIDE, speed=90.0,
           grip_ramp=True)
    c.spin(0.4)
    meas = c.pose()[3]
    # 1.08 is the claw's MECHANICAL full-open (commands below it just rest on
    # the hard stop) -- verify it got there, not the raw command value
    if meas > 1.25:
        print(f'ABORT: gripper failed to open (measured {meas:.2f}); '
              'not descending')
        return 4
    print(f'gripper open at {meas:.2f}; descending beside the ball')

    # ── 5. descend beside it, sweep in from the left while closing ─────────
    c.move(bx, by + a.approach_dy, gz + 25.0, speed=55.0, settle=False)
    c.sweep_in(bx, by, gz, a.approach_dy, a.overshoot, CLOSE,
               close_start=0.12, z_blend=25.0)
    ok, ta, th = band_held(c)
    print(f'close : held={ok} stall={ta:.3f} torH={th}')

    # ── 6. verified two-stage lift ──────────────────────────────────────────
    if ok:
        c.move(bx, by - a.overshoot, gz + 40.0, speed=25.0)
        ok, ta, th = band_held(c)
        print(f'lift40: held={ok} stall={ta:.3f} torH={th}')
    if ok:
        c.move(bx, by - a.overshoot, gz + 120.0, speed=30.0)
        c.spin(0.6)
        ok, ta, th = band_held(c)
        print(f'liftup: held={ok} stall={ta:.3f} torH={th}')
    print('RSL PICK ' + ('OK -- HOLDING THE BALL' if ok else 'FAILED'))
    c.publish_flywheel(
        c.img, {'stage': 'result', 'label': 'ball (GroundingDINO/RSL)',
                'held': bool(ok), 'stall': round(ta, 3), 'torH': th,
                'pick_arm': [round(bx), round(by), round(gz)]})
    if ok and not a.keep:
        c.move(bx, by, gz + 8.0, speed=35.0)
        c.grip(WIDE, secs=1.0)
        c.move(bx, by, gz + 110.0, speed=80.0)
    c.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 5


if __name__ == '__main__':
    sys.exit(main())
