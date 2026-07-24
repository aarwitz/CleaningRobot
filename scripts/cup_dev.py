#!/usr/bin/env python3
"""Development tool for the metal-cup-by-handle task.

Reuses sock_cycle's Cycle (topic-driven motion, torque feedback). Modes:

  --find-handle     creep -y at handle height until light contact -> post y
  --try-grasp Y     position claw around the post at y=Y, close, lift, verify
  --pose x,y,z,t    park (same as sock_cycle)
  --pitch-sweep     measure EoAT pitch vs (r, z) for the pour trajectory

The cup is RIGID: every approach move is slow with pose-lag contact detection
(commanded vs measured deviating = we are pressing on something) and aborts
rather than pushing through. Grasp verification on a rigid post uses the
commanded-vs-measured claw gap, which is unambiguous (the servo stalls ~50%
open on an ~8mm steel post vs 0.006 rad on empty air).
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sock_cycle import Cycle, GRIP_OPEN, GRIP_CLOSED  # noqa: E402

import rclpy  # noqa: E402


def lag(c):
    """|measured - commanded| in the xy plane (mm)."""
    p = c.pose()
    if p is None or c.last_cmd is None:
        return 0.0
    return ((p[0]-c.last_cmd[0])**2 + (p[1]-c.last_cmd[1])**2) ** 0.5


def creep_y(c, x, y_from, y_to, z, step=-3.0, delta_lag=3.5):
    """Slow lateral creep with contact detection. Returns contact y or None.

    Contact = lag rising `delta_lag` above the settled BASELINE (the arm
    routinely settles a few mm short of command; an absolute threshold
    false-triggers on that steady-state offset — bitten once at y=120 where
    the baseline alone was ~4mm)."""
    c.move(x, y_from, z, GRIP_OPEN, speed=50.0)
    c.spin(1.2)
    p = c.pose()
    y = p[1]                    # anchor the creep at the MEASURED start
    c.send(x, y, z, GRIP_OPEN)
    c.spin(0.5)
    base = lag(c)
    print(f'  baseline lag {base:.1f}mm at settled y={y:.1f}')
    while (step < 0 and y > y_to) or (step > 0 and y < y_to):
        y += step
        c.send(x, y, z, GRIP_OPEN)
        c.spin(0.45)
        p = c.pose()
        arm = (c.state or {}).get('arm') or {}
        lg = lag(c)
        print(f'  y_cmd={y:6.1f} y_meas={p[1]:6.1f} lag={lg:4.1f} '
              f'torB={arm.get("torB")} torS={arm.get("torS")}')
        if lg > base + delta_lag:
            print(f'  CONTACT: commanded {y:.1f}, measured {p[1]:.1f}')
            c.send(x, y - 2*step, z, GRIP_OPEN)   # back off
            c.spin(0.5)
            return p[1]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--find-handle', action='store_true')
    ap.add_argument('--cup-x', type=float, default=256.0)
    ap.add_argument('--handle-z', type=float, default=-80.0)
    ap.add_argument('--try-grasp', type=float, default=None,
                    help='claw-centre y for the grasp attempt')
    ap.add_argument('--lift', type=float, default=35.0)
    ap.add_argument('--pose', type=str, default=None)
    ap.add_argument('--pitch-sweep', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    c = Cycle()
    if not c.wait_ready():
        print('robot not ready'); return 1
    p = c.pose()
    print(f'arm at ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) grip={p[3]:.2f}')

    if a.pose:
        x, y, z, t = (float(v) for v in a.pose.split(','))
        c.move(x, y, z, t, speed=45.0); c.spin(0.5)
        p = c.pose(); print(f'now ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})')

    elif a.find_handle:
        # come in high and to the +y side, descend clear of the cup, creep in
        c.move(a.cup_x, 120.0, 30.0, GRIP_OPEN, speed=70.0)
        c.move(a.cup_x, 120.0, a.handle_z, GRIP_OPEN, speed=50.0)
        contact = creep_y(c, a.cup_x, 120.0, 35.0, a.handle_z)
        if contact is None:
            print('\nNO CONTACT down to y=35 — handle not at this x/z?')
        else:
            print(f'\nHANDLE OUTER FACE at y ~= {contact:.1f} '
                  f'(x={a.cup_x}, z={a.handle_z})')
        c.move(a.cup_x, 120.0, a.handle_z, speed=50.0)
        c.move(a.cup_x, 120.0, 30.0, speed=60.0)

    elif a.try_grasp is not None:
        y = a.try_grasp
        c.move(a.cup_x, y + 45.0, 30.0, GRIP_OPEN, speed=70.0)
        c.move(a.cup_x, y + 45.0, a.handle_z, GRIP_OPEN, speed=45.0)
        c.move(a.cup_x, y, a.handle_z, GRIP_OPEN, speed=30.0)   # straddle post
        c.spin(0.4)
        c.grip(GRIP_CLOSED, secs=1.4)
        ok, th, ta, gap = c.held()
        print(f'close: torH={th} claw={ta:.3f} gap={gap:.3f} held={ok}')
        if gap > 0.25:      # rigid post: claw stalls far from closed
            print('POST CAPTURED — lifting')
            c.move(a.cup_x, y, a.handle_z + a.lift, speed=35.0)
            c.spin(0.8)
            ok2, th2, ta2, gap2 = c.held()
            print(f'lift : torH={th2} claw={ta2:.3f} gap={gap2:.3f}')
            print('HOLDING CUP' if gap2 > 0.25 else 'LOST IT on lift')
            c.spin(1.5)
            c.move(a.cup_x, y, a.handle_z, speed=25.0)   # set gently down
            c.grip(GRIP_OPEN, secs=1.0)
            c.move(a.cup_x, y + 45.0, a.handle_z, speed=40.0)
            c.move(a.cup_x, y + 45.0, 30.0, speed=60.0)
        else:
            print('empty close — post not between jaws')
            c.grip(GRIP_OPEN, secs=0.8)
            c.move(a.cup_x, y + 45.0, a.handle_z, speed=40.0)
            c.move(a.cup_x, y + 45.0, 30.0, speed=60.0)

    elif a.pitch_sweep:
        # How much does EoAT pitch change over reachable (r, z)? Measured via
        # the shoulder+elbow angles the IK picks. Prints joint angles per pose;
        # the pour trajectory design reads pitch ~ f(r, z) off this table.
        for (x, z) in [(220, 60), (250, 30), (280, 0), (300, -30), (300, -70)]:
            c.move(x, 0, z, GRIP_OPEN, speed=70.0)
            c.spin(0.8)
            arm = (c.state or {}).get('arm') or {}
            print(f'  r={x:3d} z={z:4d} shoulder={arm.get("s"):.3f} '
                  f'elbow={arm.get("e"):.3f}')
        c.move(255, 0, 30, GRIP_OPEN, speed=60.0)

    c.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
