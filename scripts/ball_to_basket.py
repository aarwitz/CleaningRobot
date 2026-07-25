#!/usr/bin/env python3
"""Mobile pick-carry-place mission: pick the ball off the floor, turn 180,
find the wicker basket, drive to it, drop the ball in. Each SEGMENT is
recorded as its own episode with a prompt that names that segment's behavior
(the lesson from the sock fine-tune: a constant prompt that never matches the
behavior teaches the model to ignore language — and an episode set where
nothing ever ends in "release" teaches it to never let go).

Segment prompts (fixed, per operator spec):
  A  pick up the ball
  B  hold the ball and turn around until the wicker basket is in view
  C  drive toward the basket until the arm can reach it
  D  place the ball in the basket and let go of it

Base motion runs through /motor_controller/drive_relative (encoder
closed-loop, the validated SLAM-free primitive). The recorder logs
/wheel_odom and the drive-call vector per frame alongside the arm state, so a
later converter CAN put base displacement in the action space (phase 2);
today's arm-only converter just ignores those fields.

  python3 /scripts/ball_to_basket.py --practice     # no recording, one cycle
  python3 /scripts/ball_to_basket.py --missions 5   # record 5 full missions
  python3 /scripts/ball_to_basket.py --practice --skip-pick   # ball already held
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dino_client
from ball_pick import BallPick, load_cal, BALL_PROMPT, PLATE_DY, CAGE_Z, OBSERVE
from sock_cycle import Recorder, GRIP_OPEN, GRIP_CLOSED, clamp
from behavior_manager_interfaces.srv import DriveRelative
from nav_msgs.msg import Odometry

BASKET_PROMPT = 'wicker basket. woven basket. rattan basket. laundry basket'
CARRY = (250.0, 0.0, 60.0)     # travel pose: ball clear of the lens (r>=235)
                               # and high enough to keep the floor band visible
PROMPTS = {
    'A': 'pick up the ball',
    'B': 'hold the ball and turn around until the wicker basket is in view',
    'C': 'drive toward the basket until the arm can reach it',
    'D': 'place the ball in the basket and let go of it',
}


class Mission(BallPick):
    def __init__(self):
        super().__init__()
        self.odom = None
        self.last_drive = (0.0, 0.0, 0.0)
        self.create_subscription(Odometry, '/wheel_odom', self._od, 10)
        self.drv = self.create_client(DriveRelative,
                                      '/motor_controller/drive_relative')

    def _od(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.odom = (p.x, p.y, yaw)

    def drive(self, dx=0.0, dy=0.0, dyaw=0.0, rec=None, phase='drive',
              timeout=45.0):
        """Encoder closed-loop body-frame move; records frames while waiting."""
        if not self.drv.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('drive_relative service unavailable')
        self.last_drive = (dx, dy, dyaw)
        req = DriveRelative.Request(dx=float(dx), dy=float(dy), dyaw=float(dyaw))
        fut = self.drv.call_async(req)
        t0 = time.time()
        while not fut.done():
            self.spin(1.0 / 10.0)
            if rec:
                rec.row(self, phase, tuple(self.last_cmd) + (self.grip_cmd,))
            if time.time() - t0 > timeout:
                raise RuntimeError('drive_relative timed out')
        self.last_drive = (0.0, 0.0, 0.0)
        r = fut.result()
        if not r.success:
            raise RuntimeError(f'drive failed: {r.message}')
        return r

    def detect(self, prompt, conf=0.25):
        """(center_px, box, depth_mm|None) of top detection in the live frame."""
        self.spin(0.4)
        cv2.imwrite('/tmp/_m_frame.png', self.img)
        try:
            dets = dino_client.detect('/tmp/_m_frame.png', prompt, confidence=conf)
        except Exception as e:
            print(f'  [vision] {e}')
            return None
        if not dets:
            return None
        b = dets[0]['box']
        u, v = int((b[0]+b[2])/2), int((b[1]+b[3])/2)
        return (u, v), b, self.depth_at(u, v)


class MissionRecorder(Recorder):
    """Recorder + base state per frame (wheel odom + active drive vector)."""

    def __init__(self, root, idx, prompt, mission_id, segment):
        super().__init__(root, idx, prompt)
        self.meta.update({'mission': mission_id, 'segment': segment})
        self._m = None          # set right after construction

    def row(self, c, phase, target):
        # base fields piggyback through a wrapped state; simplest is to write
        # our own row (superset of Recorder's schema)
        if c.img is None:
            return
        name = f'{self.n:04d}.jpg'
        cv2.imwrite(str(self.dir / 'frames' / name), c.img,
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
        a = (c.state or {}).get('arm') or {}
        self.f.write(json.dumps({
            'i': self.n, 'ts': time.time(), 'phase': phase, 'frame': name,
            'pose': {k: a.get(k) for k in ('x', 'y', 'z', 't')},
            'joints': {k: a.get(k) for k in ('b', 's', 'e')},
            'torque': {k: a.get(k) for k in ('torB', 'torS', 'torE', 'torH')},
            'target': {'x': target[0], 'y': target[1], 'z': target[2],
                       't': target[3] if len(target) > 3 else None},
            'odom': list(c.odom) if c.odom else None,
            'drive': list(c.last_drive),
        }) + '\n')
        self.n += 1


def next_idx(root):
    return 1 + max([int(d.name[3:]) for d in Path(root).glob('ep_*')] or [-1])


def seg_recorder(root, mission_id, seg, enabled):
    if not enabled:
        return None
    return MissionRecorder(root, next_idx(root), PROMPTS[seg], mission_id, seg)


def segment_A_pick(c, rec, tx, ty, max_retries=2):
    """Localize ball, drive it into the pick band if needed, cage grasp."""
    for attempt in range(max_retries + 1):
        est, why = c.observe_ball(tx, ty)
        if est is None:
            # ball too close for depth? nudge back and retry once
            if 'depth' in why and attempt < max_retries:
                print(f'  [A] {why} -- backing up 0.22m for depth')
                c.drive(dx=-0.22, rec=rec, phase='approach')
                continue
            return False, f'observe: {why}'
        bx, by, bz = est
        r = math.hypot(bx, by)
        if r > 330.0 or r < 240.0:
            # bring the ball to r~300 dead ahead (recorded: approaching the
            # object IS part of "pick up the ball" once we're mobile)
            fwd = (r - 300.0) / 1000.0
            lat = by / 1000.0
            print(f'  [A] ball at r={r:.0f} y={by:.0f} -> drive '
                  f'dx={fwd:.3f} dy={lat:.3f}')
            c.drive(dx=fwd, dy=lat, rec=rec, phase='approach')
            continue
        ok = c.cage_grasp(bx, by)
        if ok:
            return True, 'ok'
        # recorded recovery: rise, re-observe, try again
        print(f'  [A] miss {attempt+1} -- recovering')
        c.grip(GRIP_OPEN, secs=0.6, rec=rec, phase='recover')
        c.move(bx, by - PLATE_DY, CAGE_Z + 110.0, speed=90.0,
               rec=rec, phase='recover')
    return False, 'grasp retries exhausted'


def segment_B_turn(c, rec, scan_step=0.35, max_scans=6):
    """Carry pose, two 90-deg encoder turns, then scan until basket in view."""
    c.move(*CARRY, speed=80.0, rec=rec, phase='carry')
    held, th, _, gap = c.held()
    if not held:
        return False, 'ball lost before turn'
    for half in (1.5708, 1.5708):
        c.drive(dyaw=half, rec=rec, phase='turn')
    for i in range(max_scans):
        d = c.detect(BASKET_PROMPT)
        if d is not None:
            (u, v), box, dm = d
            print(f'  [B] basket at px ({u},{v}) depth '
                  f'{dm if dm else "none"}')
            return True, 'ok'
        # alternate widening scan: +s, -2s, +3s...
        s = scan_step * (i + 1) * (1 if i % 2 == 0 else -1)
        print(f'  [B] no basket; scanning dyaw={s:.2f}')
        c.drive(dyaw=s, rec=rec, phase='scan')
    return False, 'basket never seen'


def segment_C_approach(c, rec, stop_depth_mm=560.0, max_hops=6):
    """Drive toward the basket until its rim is within arm reach.

    stop_depth_mm: distance from CAMERA to basket front at which the rim
    circle falls inside the arm annulus (tune on hardware). Never drive past
    the last hop blind: each hop is bounded and re-detected."""
    for hop in range(max_hops):
        d = c.detect(BASKET_PROMPT)
        if d is None:
            return False, 'lost the basket'
        (u, v), box, dm = d
        # center bearing first: px error -> yaw (54 px/10deg approx at 640w)
        yaw_err = -(u - 320) / 640.0 * 1.13   # HFOV ~65deg
        if abs(yaw_err) > 0.10:
            c.drive(dyaw=yaw_err, rec=rec, phase='aim')
            continue
        if dm is None:
            # too close for depth = we are close enough (blind zone ~300mm)
            print('  [C] basket inside depth blind zone -- in reach')
            return True, 'ok'
        if dm <= stop_depth_mm:
            print(f'  [C] basket at {dm:.0f}mm -- in reach')
            return True, 'ok'
        hop_m = clamp((dm - stop_depth_mm) / 1000.0, 0.08, 0.45)
        print(f'  [C] basket {dm:.0f}mm; hop {hop_m:.2f}m')
        c.drive(dx=hop_m, rec=rec, phase='approach')
    return False, 'approach hops exhausted'


def segment_D_place(c, rec, rim_r=330.0, rim_z=140.0):
    """Reach over the rim, open, retreat. rim_r/rim_z from --rim after the
    first live measurement (basket geometry is fixed for the session)."""
    held, *_ = c.held()
    if not held:
        return False, 'ball lost before place'
    c.move(rim_r, 0.0, rim_z + 80.0, speed=80.0, rec=rec, phase='transit')
    c.move(rim_r, 0.0, rim_z, speed=45.0, rec=rec, phase='over_basket')
    c.move(rim_r, 0.0, rim_z, t=GRIP_OPEN, speed=30.0, grip_ramp=True,
           rec=rec, phase='release')
    c.spin(0.6)
    c.move(*CARRY, speed=90.0, rec=rec, phase='retreat')
    _, th, ta, gap = c.held()
    if gap >= 0.03:
        return False, 'ball still in claw after release?'
    return True, 'ok'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--missions', type=int, default=1)
    ap.add_argument('--practice', action='store_true', help='no recording')
    ap.add_argument('--skip-pick', action='store_true',
                    help='ball already in the claw; start at segment B')
    ap.add_argument('--out', type=str, default='/demos_ball')
    ap.add_argument('--rim', type=str, default='330,140',
                    help='basket rim reach point r,z (arm mm)')
    ap.add_argument('--stop-depth', type=float, default=560.0)
    a = ap.parse_args()
    rim_r, rim_z = (float(v) for v in a.rim.split(','))

    rclpy.init()
    c = Mission()
    if not c.wait_ready() or not c.wait_depth():
        print('FAIL: teleop/camera/depth not ready')
        return 1
    tx, ty = load_cal()
    root = Path(a.out); root.mkdir(parents=True, exist_ok=True)
    rec_on = not a.practice
    mission_id = int(time.time())

    done = 0
    for m in range(a.missions):
        print(f'\n=== mission {m+1}/{a.missions} ===')
        segs = {}
        try:
            if not a.skip_pick:
                rec = seg_recorder(root, mission_id, 'A', rec_on)
                ok, why = segment_A_pick(c, rec, tx, ty)
                if rec: rec.finish(ok, {'why': why})
                segs['A'] = ok
                if not ok:
                    print(f'  A failed: {why}'); break
            rec = seg_recorder(root, mission_id, 'B', rec_on)
            ok, why = segment_B_turn(c, rec)
            if rec: rec.finish(ok, {'why': why})
            segs['B'] = ok
            if not ok:
                print(f'  B failed: {why}'); break
            rec = seg_recorder(root, mission_id, 'C', rec_on)
            ok, why = segment_C_approach(c, rec, stop_depth_mm=a.stop_depth)
            if rec: rec.finish(ok, {'why': why})
            segs['C'] = ok
            if not ok:
                print(f'  C failed: {why}'); break
            rec = seg_recorder(root, mission_id, 'D', rec_on)
            ok, why = segment_D_place(c, rec, rim_r=rim_r, rim_z=rim_z)
            if rec: rec.finish(ok, {'why': why})
            segs['D'] = ok
            print(f'  mission {"COMPLETE" if all(segs.values()) else "PARTIAL"}: {segs}')
            done += 1 if all(segs.values()) else 0
        except RuntimeError as e:
            print(f'  ABORT: {e}')
            break
        mission_id += 1
    print(f'\n{done}/{a.missions} missions complete -> {root}')
    c.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
