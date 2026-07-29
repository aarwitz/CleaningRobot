#!/usr/bin/env python3
"""DINO + RealSense depth ball pick — no touch probing, recordable.

The pipeline the data flywheel runs on:
  1. GroundingDINO on RSL (tunnel), lower-frame ROI, tuned prompt.
  2. Aligned-depth near-cluster median inside the bbox = centroid depth
     (median already sits mid-dome; do NOT add a radius term).
  3. If the ball is inside the D455 blind zone (<5% valid depth px), DO NOT
     poke it with the claw (probing shoves it around — operator-banned).
     Instead: tuck the arm, back up 0.18m on encoders, localize from where
     depth works, then drive back the exact measured distance (validated
     ±5mm) and pick open-loop at the r~300 sweet spot.
  4. The winning grasp (operator-confirmed perfect pick 2026-07-26):
     open wide during transit, descend FULLY at ball_y+60 to gz -176,
     flat lateral sweep (z_blend=0, close_start=0.30, overshoot 14),
     close 2.60, secure-tighten 2.95 before any transit.
  5. Held-verify by VISION ONLY (floor clear of ball = held). Band stall
     angles are posture-dependent and once called a perfect pick a miss.
  6. --record: every attempt (hit or miss) is an episode on disk --
     frames + traj.jsonl + meta with success flag. Misses are data too.

  python3 /scripts/ball_rsl_pick.py --keep [--record] [--episodes N]
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
from ball_pick import BallPick, BALL_PROMPT, OBSERVE, load_cal
from behavior_manager_interfaces.srv import DriveRelative
from sock_cycle import GRIP_CLOSED, Recorder

WIDE = 0.35
CLOSE = 2.60
SECURE = 2.95
GZ = -176.0
PICK_R = 300.0          # the proven grasp radius (vertical-claw posture)
ROI = (16, 240, 624, 480)


class RslPick(BallPick):
    def __init__(self):
        super().__init__()
        self.drv = self.create_client(DriveRelative,
                                      '/motor_controller/drive_relative')

    def drive_yaw(self, dyaw, timeout=45.0):
        if not self.drv.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('drive_relative unavailable')
        fut = self.drv.call_async(DriveRelative.Request(
            dx=0.0, dy=0.0, dyaw=float(dyaw)))
        t0 = time.time()
        while not fut.done():
            self.spin(0.1)
            if time.time() - t0 > timeout:
                raise RuntimeError('yaw timed out')
        if not fut.result().success:
            raise RuntimeError(f'yaw failed: {fut.result().message}')

    def drive(self, dx=0.0, dy=0.0, timeout=45.0):
        if not self.drv.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('drive_relative unavailable')
        fut = self.drv.call_async(
            DriveRelative.Request(dx=float(dx), dy=float(dy), dyaw=0.0))
        t0 = time.time()
        while not fut.done():
            self.spin(0.1)
            if time.time() - t0 > timeout:
                raise RuntimeError('drive timed out')
        r = fut.result()
        if not r.success:
            raise RuntimeError(f'drive failed: {r.message}')
        return (r.actual_dx, r.actual_dy)

    def tuck(self):
        self.move(255, 0, 60, GRIP_CLOSED, speed=90.0)

    def localize(self, expect_u=None):
        """(bx, by, quality, u) from DINO + depth; quality in
        ok|blind|no detection|mismatch|unreachable."""
        self.move(*OBSERVE, t=GRIP_CLOSED, speed=100.0)
        self.spin(0.7)
        cv2.imwrite('/tmp/_rsl_frame.png', self.img)
        try:
            dets = dino_client.detect('/tmp/_rsl_frame.png', BALL_PROMPT,
                                      confidence=0.25, roi=ROI)
        except Exception as e:
            return None, None, f'DINO unreachable: {e}', None
        if not dets:
            return None, None, 'no detection', None
        box, score = dets[0]['box'], dets[0]['score']
        u = int((box[0] + box[2]) / 2)
        if expect_u is not None and abs(u - expect_u) > 70:
            # the ball cannot teleport: a pure-x backup barely moves its
            # column. A far-off detection is clutter (chrome reflections,
            # the plush) outscoring it -- attempt 1 chased one 95px away.
            print(f'  [loc] detection at u={u} but expected ~{expect_u} '
                  f'-- rejecting as mislocalization')
            return None, None, 'mismatch', u
        x0, y0, x1, y1 = (int(t) for t in box)
        d = self.depth[max(0, y0):y1, max(0, x0):x1].astype(float)
        valid = d[(d > 100) & (d < 4000)]
        frac = valid.size / max(d.size, 1)
        tx, ty = load_cal()
        # bbox clipped at the frame bottom = ball half out of view INSIDE the
        # blind zone; any "valid" depth in that bbox is background (attempt 3
        # trusted 305mm of floor and picked a phantom)
        if valid.size < 20 or frac < 0.05 or y1 >= 472:
            print(f'  [loc] box {[round(b) for b in box]} score {score:.2f} '
                  f'-- depth blind ({100*frac:.1f}% valid'
                  + (', bbox at frame bottom' if y1 >= 472 else '') + ')')
            return None, None, 'blind', u
        near = valid[valid <= np.percentile(valid, 50)]
        surf = float(np.median(near))
        fx, _, cx, _ = self.K
        xc = (u - cx) * surf / fx
        bx, by = surf + tx, -xc + ty
        print(f'  [loc] box {[round(b) for b in box]} score {score:.2f} '
              f'depth {surf:.0f}mm ({valid.size}px) -> arm ({bx:.0f},{by:.0f})')
        self.publish_flywheel(
            self.img, {'stage': 'localized', 'label': 'ball (DINO/RSL)',
                       'score': score, 'box': [round(t) for t in box],
                       'depth_mm': surf, 'valid_px': int(valid.size),
                       'pick_arm': [round(bx), round(by), round(GZ)],
                       'prompt': BALL_PROMPT},
            box=box, pick_px=(u, int((y0 + y1) / 2)))
        return bx, by, 'ok', u

    def grasp(self, bx, by, rec=None):
        """The winning recipe. Returns held (vision-verified)."""
        self.move(bx, by + 60.0, GZ + 110.0, t=WIDE, speed=90.0,
                  grip_ramp=True, rec=rec, phase='approach')
        self.spin(0.8)          # let the servo finish the physical open
        meas = self.pose()[3]
        if meas > 1.25:
            print(f'  gripper failed to open (measured {meas:.2f})')
            return False
        self.move(bx, by + 60.0, GZ, speed=45.0, settle=False,
                  rec=rec, phase='descend')
        self.sweep_in(bx, by, GZ, 60.0, 14.0, CLOSE,
                      close_start=0.30, z_blend=0.0, rec=rec, phase='grasp')
        self.grip(SECURE, secs=0.8, rec=rec, phase='secure')
        self.move(bx, by - 14.0, GZ + 40.0, speed=25.0, rec=rec, phase='lift')
        self.move(bx, by - 14.0, GZ + 130.0, speed=30.0, rec=rec, phase='lift')
        self.spin(0.8)
        # vision verify: is the floor clear of the ball?
        cv2.imwrite('/tmp/_verify.png', self.img)
        held = True
        try:
            v = dino_client.detect('/tmp/_verify.png', BALL_PROMPT,
                                   confidence=0.25, roi=ROI)
            floor_hits = [dv for dv in v
                          if (dv['box'][1] + dv['box'][3]) / 2 > 300]
            held = not floor_hits
            if floor_hits:
                print(f'  verify: ball still on floor '
                      f'{[round(t) for t in floor_hits[0]["box"]]} -- MISS')
            else:
                print('  verify: floor clear -- HELD')
        except Exception:
            print('  verify: DINO unreachable, assuming held')
        self.publish_flywheel(
            self.img, {'stage': 'result', 'label': 'ball (DINO/RSL)',
                       'held': bool(held),
                       'pick_arm': [round(bx), round(by), round(GZ)]})
        return held


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', type=int, default=1)
    ap.add_argument('--keep', action='store_true',
                    help='stay holding after a successful pick (single ep)')
    ap.add_argument('--record', action='store_true')
    ap.add_argument('--out', type=str, default='/demos_ball')
    ap.add_argument('--prompt', type=str, default='pick up the ball')
    a = ap.parse_args()

    rclpy.init()
    c = RslPick()
    if not c.wait_ready() or not c.wait_depth():
        print('ABORT: teleop/camera/depth not ready')
        return 1

    root = Path(a.out)
    if a.record:
        root.mkdir(parents=True, exist_ok=True)
    idx = 1 + max([int(p.name[3:]) for p in root.glob('ep_*')] or [-1]) \
        if a.record else 0

    ok_n = miss_n = 0
    for ep in range(a.episodes):
        print(f'\n── attempt {ep+1}/{a.episodes} ──')
        bx, by, why, u0 = c.localize()
        if why == 'blind':
            # Reposition instead of probing: tuck, back up on encoders,
            # localize from depth-valid range, drive back the exact amount.
            print('  [loc] blind zone -> tucked-arm encoder reposition')
            c.tuck()
            c.drive(dx=-0.18)
            bx, by, why, _ = c.localize(expect_u=u0)
            if why == 'mismatch':
                c.spin(1.0)     # fresh frame, one retry
                bx, by, why, _ = c.localize(expect_u=u0)
            if why != 'ok':
                print(f'  ABORT attempt: no localization after backup ({why})')
                continue
        elif why != 'ok':
            print(f'  ABORT attempt: {why}')
            continue

        # Align by YAW (accurate) not strafe (mecanum scrub axis), then close
        # the range so the ball sits at the proven r~300, dead ahead.
        rng = math.hypot(bx, by)
        bearing = math.atan2(by, bx)
        if abs(by) > 45.0 or rng > 345.0:
            c.tuck()
            if abs(bearing) > 0.06:
                print(f'  [align] yaw {math.degrees(bearing):.1f} deg')
                c.drive_yaw(bearing)
            fwd = max(-0.05, min(0.35, (rng - PICK_R) / 1000.0))
            if abs(fwd) > 0.015:
                print(f'  [align] range {rng:.0f} -> drive {fwd:.3f}m')
                fdx, _ = c.drive(dx=fwd)
                rng -= fdx * 1000.0
            bx, by = rng, 0.0
            print(f'  [align] picking at ({bx:.0f},{by:.0f})')

        rec = Recorder(root, idx, a.prompt) if a.record else None
        held = False
        try:
            held = c.grasp(bx, by, rec=rec)
        except RuntimeError as e:
            print(f'  attempt aborted: {e}')
        if rec:
            rec.finish(held, {'pick': [bx, by, GZ], 'held': held})
            idx += 1
        if held:
            ok_n += 1
            if a.keep and ep == a.episodes - 1:
                print('holding the ball (--keep)')
                break
            # put it back down at a slightly different spot for the next ep
            nx = bx + (20.0 if bx < 300 else -25.0)
            c.move(nx, by, GZ + 8.0, speed=35.0)
            c.grip(WIDE, secs=1.0)
            c.move(nx, by, GZ + 110.0, speed=80.0)
        else:
            miss_n += 1
            c.grip(WIDE, secs=0.8)
            c.move(*OBSERVE, t=GRIP_CLOSED, speed=90.0)

    print(f'\n{ok_n} held / {miss_n} missed'
          + (f' -> {root}' if a.record else ''))
    c.destroy_node()
    rclpy.shutdown()
    return 0 if ok_n else 2


if __name__ == '__main__':
    sys.exit(main())
