#!/usr/bin/env python3
"""One cage-grasp attempt on a rigid ball: hover, straight-down descend so the
open jaws straddle it, slow ramp close, lift, report held(). A ball is the
anti-sock: incompressible and it ROLLS, so instead of the lateral sweep_in
(which would bat it away) the claw descends over it and the single moving jaw
rolls it into the fixed jaw during the close.

  python3 /scripts/ball_grasp_test.py --at 310,-55 --grasp-z -172 [--keep-held]
"""
import argparse
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sock_cycle import Cycle, GRIP_OPEN, GRIP_CLOSED


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--at', required=True, help='ball x,y (arm mm)')
    ap.add_argument('--grasp-z', type=float, default=-172.0,
                    help='wrist z for the close; jaws tip ~5mm below wrist')
    ap.add_argument('--hover', type=float, default=60.0)
    ap.add_argument('--close-to', type=float, default=GRIP_CLOSED)
    ap.add_argument('--close-secs', type=float, default=1.6)
    ap.add_argument('--keep-held', action='store_true',
                    help='stay lifted holding the ball instead of releasing')
    ap.add_argument('--release', action='store_true',
                    help='just open the claw at the current pose and exit')
    a = ap.parse_args()
    bx, by = (float(v) for v in a.at.split(','))

    rclpy.init()
    c = Cycle()
    if not c.wait_ready():
        print('FAIL: no teleop/camera')
        return 1

    if a.release:
        c.grip(GRIP_OPEN, secs=1.0)
        print('released')
        c.destroy_node(); rclpy.shutdown()
        return 0

    p = c.pose()
    print(f'arm at ({p[0]:.1f},{p[1]:.1f},{p[2]:.1f}) grip {p[3]:.2f}')

    # hover above the ball, claw open
    c.move(bx, by, a.grasp_z + a.hover, GRIP_OPEN, speed=90.0)
    c.spin(0.3)
    # straight-down cage: slow enough not to bounce the ball away on contact
    c.move(bx, by, a.grasp_z, speed=40.0)
    c.spin(0.3)
    # slow ramp close: the moving jaw rolls the ball into the fixed jaw
    c.grip(a.close_to, secs=a.close_secs)
    ok, th, ta, gap = c.held()
    print(f'close: held={ok} torH={th} claw={ta:.3f} gap={gap:.3f}')
    # lift SLOWLY and re-verify -- a rigid ball held between flat jaw plates
    # has two-point contact only; a brisk lift shakes it out
    c.move(bx, by, a.grasp_z + a.hover + 40.0, speed=35.0)
    c.spin(0.5)
    ok2, th2, ta2, gap2 = c.held()
    print(f'lift : held={ok2} torH={th2} claw={ta2:.3f} gap={gap2:.3f}')
    print(f'\n{"BALL GRASP OK" if ok2 else "BALL GRASP FAILED"}')
    if ok2 and not a.keep_held:
        # put it back down gently and release
        c.move(bx, by, a.grasp_z + 6.0, speed=45.0)
        c.grip(GRIP_OPEN, secs=1.0)
        c.move(bx, by, a.grasp_z + a.hover, speed=80.0)
    c.destroy_node(); rclpy.shutdown()
    return 0 if ok2 else 2


if __name__ == '__main__':
    sys.exit(main())
