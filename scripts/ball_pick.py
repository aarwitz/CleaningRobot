#!/usr/bin/env python3
"""Depth-localized floor pick for a rigid rolling ball (tiny rubber basketball).

Why this differs from sock_cycle's sweep_in: a ball ROLLS. The lateral sweep
that scoops a sock just pushes a ball ahead of the closing jaw (measured:
close gap 0.024 = shut on air). And a straight-down descend AT the ball's xy
lands the FIXED jaw plate on top of it -- the plate hangs at the wrist xy, tips
~50 mm below the wrist z (measured 2026-07-25: tip rested on the 60 mm ball's
top at wrist z=-101.5, ball top -152, floor -212).

The grasp that works ("offset cage"):
  wrist xy = (ball_x, ball_y - PLATE_DY)   # plate hugs the ball's -y flank
  wrist z  = CAGE_Z                        # jaw tips below the ball equator
  then a slow close rolls the ball into the plate with the moving (+y) jaw,
  contact BELOW the equator so lift wedges it in instead of squirting it out.

Localization: GroundingDINO pixel + aligned depth median + pinhole
back-projection, then an axis-aligned cam->arm transform. TZ is
re-solved every observe from the FLOOR plane in the same frame (the floor is
z=FLOOR_Z by definition), so pitch/mount drift cannot bias z. TX/TY start from
the validated arm_bridge numbers and are refined by --probe-cal: touch-probe
the ball top (slow descend, watch z-lag) at the depth-estimated xy, then
spiral until contact says we are ON the ball; the residual updates TX/TY on
disk. After one calibration, picks are open-loop from a single observation.

  python3 /scripts/ball_pick.py --probe-cal      # once: solve TX/TY
  python3 /scripts/ball_pick.py --pick           # localize + cage grasp + lift
  python3 /scripts/ball_pick.py --pick --keep    # ...and stay holding it
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
from sock_cycle import Cycle, GRIP_OPEN, GRIP_CLOSED, clamp
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

BALL_PROMPT = 'orange ball. rubber basketball. dog toy ball'
BALL_R = 30.0                # mm, tiny basketball
FLOOR_Z = None               # solved from depth each observe; see floor_tz()
TIP_DROP = 50.0              # jaw tips hang this far below the wrist z
PLATE_DY = 33.0              # wrist -y offset so the fixed plate clears the ball
CAGE_Z = -145.0              # wrist z for the close: tips at -195, equator -182
OBSERVE = (255.0, 0.0, 20.0)  # canonical observation wrist pose (x, y, z)
CAL_FILE = '/demos/ball_handeye.json'
# axis-aligned cam->arm: arm_x = rs_z*1000 + TX ; arm_y = -rs_x*1000 + TY ;
# arm_z = -rs_y*1000 + TZ  (TZ re-solved from the floor plane every observe)
TX0, TY0 = 100.0, 0.0


class BallPick(Cycle):
    """Cycle + aligned depth + camera intrinsics."""

    def __init__(self):
        super().__init__()
        self.depth = None
        self.K = None
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw',
                                 self._d, 10)
        self.create_subscription(CameraInfo,
                                 '/camera/aligned_depth_to_color/camera_info',
                                 self._k, 10)

    def _d(self, m):
        self.depth = self.br.imgmsg_to_cv2(m, 'passthrough')  # uint16 mm

    def _k(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])  # fx, fy, cx, cy

    def wait_depth(self, timeout=8.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin(0.05)
            if self.depth is not None and self.K is not None:
                return True
        return False

    def depth_at(self, u, v, win=5):
        d = self.depth[max(0, v-win):v+win+1, max(0, u-win):u+win+1].astype(float)
        d = d[(d > 100) & (d < 4000)]
        return float(np.median(d)) if d.size else None

    def backproject(self, u, v, depth_mm):
        fx, fy, cx, cy = self.K
        z = depth_mm / 1000.0
        return ((u - cx) * z / fx, (v - cy) * z / fy, z)   # rs x right, y down, z fwd

    def floor_tz(self, ball_v):
        """Solve TZ from floor pixels in THIS frame: rows well below the ball's
        bbox bottom are floor; their arm z must equal the probed floor height."""
        floor_probe_z = -212.0
        # Rows just BELOW the ball's bbox: floor at roughly the ball's own
        # distance. Rows near the frame bottom are <300 mm from the camera --
        # inside the D455 blind zone, no depth there.
        samples = []
        h, w = self.depth.shape
        for v in range(min(ball_v + 6, h - 8), min(ball_v + 44, h - 4), 6):
            for u in range(60, w - 60, 40):
                dm = self.depth_at(u, v, win=2)
                if dm:
                    rs = self.backproject(u, v, dm)
                    samples.append(-rs[1] * 1000.0)
        if len(samples) < 6:
            return None
        return floor_probe_z - float(np.median(samples))

    def observe_ball(self, tx, ty):
        """From the canonical pose: DINO + depth -> ball center in arm mm."""
        self.move(*OBSERVE, t=GRIP_CLOSED, speed=110.0)
        self.spin(0.6)
        cv2.imwrite('/tmp/_ball_frame.png', self.img)
        dets = dino_client.detect('/tmp/_ball_frame.png', BALL_PROMPT,
                                  confidence=0.25)
        if not dets:
            return None, 'no detection'
        b = dets[0]['box']
        u, v = int((b[0]+b[2])/2), int((b[1]+b[3])/2)
        dm = self.depth_at(u, v)
        if dm is None:
            return None, f'no depth at px ({u},{v})'
        rs = self.backproject(u, v, dm)
        tz = self.floor_tz(int(b[3]))
        if tz is None:
            return None, 'no floor samples for TZ'
        # depth hits the ball SURFACE; push to center along the (unit-ish) ray
        cx = (rs[2]*1000.0 + BALL_R) + tx
        cy = -rs[0]*1000.0 + ty
        cz = -rs[1]*1000.0 + tz
        print(f'  ball px ({u},{v}) depth {dm:.0f}mm -> arm '
              f'({cx:.0f},{cy:.0f},{cz:.0f})  [TZ {tz:.0f}]')
        return (cx, cy, cz), 'ok'

    def touch_probe(self, x, y, z_start=-60.0, z_floor_wrist=-168.0):
        """Slow closed-claw descend at (x,y); returns wrist z at first contact
        (measured z stops tracking) or None if it reached near-floor clean."""
        self.move(x, y, z_start, GRIP_CLOSED, speed=60.0)
        self.spin(0.4)
        z = z_start
        while z > z_floor_wrist:
            z -= 4.0
            self.send(x, y, z, GRIP_CLOSED)
            self.spin(0.4)
            p = self.pose()
            if p[2] - z > 7.0:
                return p[2]
        return None

    def cage_grasp(self, bx, by, close_secs=2.0):
        """Offset cage: plate down the -y flank, slow close, slow lift."""
        wx, wy = bx, by - PLATE_DY
        self.move(wx, wy, CAGE_Z + 90.0, GRIP_OPEN, speed=100.0)
        self.spin(0.2)
        self.move(wx, wy, CAGE_Z, speed=35.0)
        self.spin(0.3)
        self.grip(GRIP_CLOSED, secs=close_secs)
        ok, th, ta, gap = self.held()
        print(f'  close: held={ok} torH={th} claw={ta:.3f} gap={gap:.3f}')
        self.move(wx, wy, CAGE_Z + 130.0, speed=35.0)
        self.spin(0.5)
        ok2, th2, ta2, gap2 = self.held()
        print(f'  lift : held={ok2} torH={th2} claw={ta2:.3f} gap={gap2:.3f}')
        return ok2


def load_cal():
    p = Path(CAL_FILE)
    if p.exists():
        d = json.loads(p.read_text())
        return d['tx'], d['ty']
    return TX0, TY0


def save_cal(tx, ty):
    Path(CAL_FILE).write_text(json.dumps({'tx': tx, 'ty': ty, 'ts': time.time()}))
    print(f'  hand-eye saved: TX={tx:.1f} TY={ty:.1f} -> {CAL_FILE}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--probe-cal', action='store_true',
                    help='touch-probe spiral to solve TX/TY from the ball')
    ap.add_argument('--pick', action='store_true')
    ap.add_argument('--keep', action='store_true', help='stay holding after pick')
    ap.add_argument('--observe-only', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    c = BallPick()
    if not c.wait_ready() or not c.wait_depth():
        print('FAIL: teleop/camera/depth not ready')
        return 1
    tx, ty = load_cal()
    print(f'hand-eye TX={tx:.1f} TY={ty:.1f}')

    est, why = c.observe_ball(tx, ty)
    if est is None:
        print(f'FAIL: {why}')
        return 2
    bx, by, bz = est

    if a.observe_only:
        return 0

    if a.probe_cal:
        # Touch the ball top at the estimate; spiral outward until contact says
        # we are ON it (contact well above the floor's wrist-contact height).
        offsets = [(0, 0)] + [(r*math.cos(t), r*math.sin(t))
                              for r in (18, 36)
                              for t in np.linspace(0, 2*math.pi, 8, endpoint=False)]
        hit = None
        for dx, dy in offsets:
            cz = c.touch_probe(bx + dx, by + dy)
            tag = f'{cz:.0f}' if cz is not None else 'floor/none'
            print(f'  probe ({bx+dx:.0f},{by+dy:.0f}) -> contact {tag}')
            c.move(bx + dx, by + dy, -40.0, GRIP_CLOSED, speed=80.0)
            if cz is not None and cz > -135.0:      # ball top, not floor
                hit = (bx + dx, by + dy)
                break
        if hit is None:
            print('FAIL: never touched the ball; depth estimate too far off')
            return 2
        # the residual is the correction depth-estimate was missing
        save_cal(tx + (hit[0] - bx), ty + (hit[1] - by))
        return 0

    if a.pick:
        ok = c.cage_grasp(bx, by)
        print(f'\n{"BALL PICK OK" if ok else "BALL PICK FAILED"}')
        if ok and not a.keep:
            c.move(bx, by - PLATE_DY, CAGE_Z + 8.0, speed=40.0)
            c.grip(GRIP_OPEN, secs=1.0)
            c.move(bx, by - PLATE_DY, CAGE_Z + 90.0, speed=80.0)
        c.destroy_node(); rclpy.shutdown()
        return 0 if ok else 2

    return 0


if __name__ == '__main__':
    sys.exit(main())
