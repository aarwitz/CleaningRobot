#!/usr/bin/env python3
"""Repeated sock pick-and-place demo collection, driven entirely over ROS topics.

Why this exists alongside collect_pick_demos.py: that collector opens
/dev/ttyUSB0 itself and drives the arm with T:104, which the vendor documents as
BLOCKING and which WEDGES the ESP32 when streamed (see the
roarm-continuous-control-protocol notes). This one owns no hardware at all. It
publishes `goto:` setpoints on /teleop/action and reads pose+torque back from
/teleop/state, so teleop_node stays the single owner of the serial port and the
non-blocking T:1041 path is the only thing that ever reaches the arm. That also
means the operator's E-STOP in the browser halts a scripted run.

MISSED-PICK DETECTION is gripper TORQUE, not gripper angle. Closing on nothing
lets the claw reach its mechanical limit and the servo unloads (|torH| ~ 32);
closing on a sock leaves the servo straining against it indefinitely
(|torH| ~ 64-80). Angle barely moves between the two cases (3.132 vs 3.114) --
on a compressible sock it is close to useless, so torque is the discriminator.
Every episode is verified twice: after the close, and again after the lift (to
catch a sock that slipped out on the way up). A failed episode is DELETED, so
the dataset never contains a demonstration of failure labelled as success.

  # one careful cycle, nothing recorded -- always run this first
  python3 /scripts/sock_cycle.py --test-grasp --grasp-z -85

  # collect
  python3 /scripts/sock_cycle.py --episodes 30 --grasp-z -85
"""
import argparse
import json
import math
import random
import shutil
import sys
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

RATE = 20.0                  # setpoint stream rate; T:1041 does no interpolation
                             # of its own, so smoothness comes from step size.
                             # 20 Hz is the validated ceiling and the smoothest;
                             # note RATE changes ONLY smoothness, not wall-clock
                             # speed (steps = dist/(speed/RATE), time = dist/speed),
                             # so the tuned pick keeps its exact pace, just cleaner.
GRIP_OPEN, GRIP_CLOSED = 1.08, 3.14

# Gripper-torque grasp detector. See module docstring.
HELD_TORH_MIN = 50
HELD_T_MAX = 3.115           # closing on nothing settles at 3.132
GRIP_GAP_MIN = 0.03          # commanded-minus-measured that counts as "something there"
EMPTY_TORH = 32

# Envelope must match teleop_node's, or its clamp silently moves our target and
# the recorded action stops matching what the arm did.
R_MIN, R_MAX = 180.0, 300.0
Z_MIN, Z_MAX = -200.0, 320.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Cycle(Node):
    def __init__(self):
        super().__init__('sock_cycle')
        self.br = CvBridge()
        self.state = None
        self.img = None
        # The COMMANDED claw angle, tracked separately from the measured one.
        # On a compliant object the servo stalls short of its command (3.02 vs
        # 3.14 on this sock). Feeding the measured angle back as the next
        # command tells the servo it has arrived, it stops straining, and the
        # object slides out mid-lift. Grip commands must only ever come from
        # grip(); motion must hold this value untouched.
        self.grip_cmd = None
        self.create_subscription(String, '/teleop/state', self._st, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self._im, 10)
        self.act = self.create_publisher(String, '/teleop/action', 10)

    def _st(self, m):
        try:
            self.state = json.loads(m.data)
        except Exception:
            pass

    def _im(self, m):
        self.img = self.br.imgmsg_to_cv2(m, 'bgr8')

    # ── plumbing ────────────────────────────────────────────────────────────
    def spin(self, dt):
        t0 = time.time()
        while time.time() - t0 < dt:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_ready(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state and self.img is not None and self.act.get_subscription_count():
                return True
        return False

    def pose(self):
        a = (self.state or {}).get('arm') or {}
        if a.get('x') is None:
            return None
        return (a['x'], a['y'], a['z'], a['t'])

    def torh(self):
        return abs(((self.state or {}).get('arm') or {}).get('torH') or 0)

    def held(self):
        """(is_held, torH, claw_angle, gap).

        `gap` = commanded minus measured claw angle: how far short of its
        command the claw stalled, i.e. the thickness of whatever is between the
        jaws. That generalises the old fixed 3.115 threshold to ANY commanded
        close value, which matters because squeezing a sock all the way to 3.14
        extrudes it. Empty close leaves a gap of ~0.006; a held sock ~0.16.
        """
        a = (self.state or {}).get('arm') or {}
        th = abs(a.get('torH') or 0)
        ta = a.get('t')
        gap = (self.grip_cmd - ta) if (self.grip_cmd is not None
                                       and ta is not None) else 0.0
        ok = th >= HELD_TORH_MIN or gap >= GRIP_GAP_MIN
        return ok, th, ta, gap

    def estopped(self):
        return bool((self.state or {}).get('estop'))

    def send(self, x, y, z, t):
        r = math.hypot(x, y)
        if r > 1e-6:
            rc = clamp(r, R_MIN, R_MAX)
            if abs(rc - r) > 0.5:
                x, y = x * rc / r, y * rc / r
        z = clamp(z, Z_MIN, Z_MAX)
        m = String()
        m.data = f'goto:{x:.1f},{y:.1f},{z:.1f},{t:.3f}'
        self.act.publish(m)
        return (x, y, z, t)

    # ── motion ──────────────────────────────────────────────────────────────
    def move(self, x, y, z, t=None, speed=70.0, rec=None, phase='',
             settle=True, grip_ramp=False):
        """Interpolated cartesian move at `speed` mm/s, streaming T:1041.

        Streaming a setpoint train rather than one jump is what makes this look
        like a demonstration instead of a lurch -- and a policy trained on
        lurches learns lurches.

        settle=False skips the convergence wait at the end so this leg flows
        straight into the next one (no stop-and-restart between approach ->
        descend -> sweep). grip_ramp=True ramps the claw from its current
        command to `t` OVER the move instead of snapping -- used for the
        "aware" release, where the gripper opens gradually while the arm eases
        left and up.
        """
        cur = self.pose()
        if cur is None:
            raise RuntimeError('no arm feedback')
        x0, y0, z0, _ = cur
        if self.grip_cmd is None:
            self.grip_cmd = cur[3]
        g0 = self.grip_cmd
        g1 = t if t is not None else g0
        if not grip_ramp:
            self.grip_cmd = g1              # snap-and-hold (default behaviour)
        dist = math.dist((x0, y0, z0), (x, y, z))
        steps = max(1, int(round(dist / max(1e-6, speed / RATE))))
        for i in range(1, steps + 1):
            if self.estopped():
                raise RuntimeError('E-STOP during move')
            f = i / steps
            if grip_ramp:
                self.grip_cmd = g0 + (g1 - g0) * f
            tgt = self.send(x0 + (x - x0) * f, y0 + (y - y0) * f,
                            z0 + (z - z0) * f, self.grip_cmd)
            self.spin(1.0 / RATE)
            if rec:
                rec.row(self, phase, tgt)
        self.grip_cmd = g1
        if settle:
            self.settle(x, y, z, rec=rec, phase=phase)

    def settle(self, x, y, z, tol=6.0, timeout=2.5, rec=None, phase=''):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin(1.0 / RATE)
            if rec:
                rec.row(self, phase, (x, y, z, None))
            p = self.pose()
            if p and math.dist(p[:3], (x, y, z)) <= tol:
                return True
        return False

    def grip(self, target, secs=1.2, rec=None, phase=''):
        """Ramp the claw. Ramping (not snapping) keeps a light object from being
        batted away by the moving jaw before the other side closes on it."""
        p = self.pose()
        x, y, z, meas = p
        t0 = self.grip_cmd if self.grip_cmd is not None else meas
        steps = max(1, int(secs * RATE))
        for i in range(1, steps + 1):
            tgt = self.send(x, y, z, t0 + (target - t0) * i / steps)
            self.spin(1.0 / RATE)
            if rec:
                rec.row(self, phase, tgt)
        self.grip_cmd = target  # held from here on; motion must not overwrite it
        self.spin(0.4)          # let the servo load up before torque is read

    def sweep_in(self, sx, sy, gz, dy, overshoot, close_to,
                 close_start=0.12, close_end=0.85, speed=32.0,
                 z_blend=25.0, z_blend_frac=0.45, rec=None, phase='sweep'):
        """The one-sided-claw grasp. Assumes we're already parked at
        (sx, sy+dy, gz + z_blend) with the claw OPEN -- i.e. straight down the
        side, just ABOVE the object's depth.

        The moving jaw sits on the +y (robot-left) side. Descending straight
        onto a sock just mashes it flat and the fold extrudes out on lift
        (measured: close gap 0.28 -> lift gap 0.01). Instead we sweep the open
        claw laterally from +y toward the sock while it CLOSES, so that jaw
        catches the sock's outer edge and drags it INTO the fixed jaw rather
        than pressing down on it.

        Two things make it look natural (operator feedback 2026-07-22):
        * The last `z_blend` mm of descent are folded INTO the start of the
          sweep -- z finishes dropping to gz over the first `z_blend_frac` of
          the lateral move -- so the tool curves down-and-in as one arc instead
          of descend, stall, then sweep.
        * The claw starts closing early (close_start) and reaches fully closed
          BEFORE the lateral motion ends (close_end < 1.0), so the sock is
          captured while the jaw is still rotating through it rather than after
          the arm has stopped.
        """
        y0, y1 = sy + dy, sy - overshoot
        z0 = gz + z_blend
        self.grip_cmd = GRIP_OPEN
        steps = max(1, int(abs(y0 - y1) / max(1e-6, speed / RATE)))
        for i in range(1, steps + 1):
            if self.estopped():
                raise RuntimeError('E-STOP during sweep')
            f = i / steps
            y = y0 + (y1 - y0) * f
            zf = min(1.0, f / z_blend_frac) if z_blend_frac > 1e-6 else 1.0
            z = z0 + (gz - z0) * zf
            if f <= close_start:
                g = GRIP_OPEN
            elif f >= close_end:
                g = close_to
            else:
                g = GRIP_OPEN + (close_to - GRIP_OPEN) * \
                    (f - close_start) / (close_end - close_start)
            self.grip_cmd = g
            tgt = self.send(sx, y, z, g)
            self.spin(1.0 / RATE)
            if rec:
                rec.row(self, phase, tgt)
        self.spin(0.25)         # brief seat before torque read; not a full stop


class Recorder:
    """One episode on disk: frames/, traj.jsonl, meta.json."""

    def __init__(self, root, idx, prompt):
        self.dir = Path(root) / f'ep_{idx:04d}'
        if self.dir.exists():
            shutil.rmtree(self.dir)
        (self.dir / 'frames').mkdir(parents=True)
        self.f = open(self.dir / 'traj.jsonl', 'w')
        self.n = 0
        self.meta = {'prompt': prompt, 'episode': idx, 't_start': time.time()}

    def row(self, c, phase, target):
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
            'target': {'x': target[0], 'y': target[1],
                       'z': target[2], 't': target[3]},
        }) + '\n')
        self.n += 1

    def finish(self, ok, extra):
        self.f.close()
        self.meta.update(extra)
        self.meta.update({'t_end': time.time(), 'success': ok, 'n_frames': self.n})
        (self.dir / 'meta.json').write_text(json.dumps(self.meta, indent=1))

    def discard(self):
        self.f.close()
        shutil.rmtree(self.dir, ignore_errors=True)


def cycle(c, px, py, gz, hover, rec, place=None, settle_s=0.0,
          close_to=GRIP_CLOSED, approach_dy=55.0, overshoot=6.0,
          close_start=0.12, z_blend=25.0, release_dy=8.0):
    """One sweep-in pick-and-place. Returns (ok, reason, torques, place_point).

    Pick uses the one-sided-claw sweep (see Cycle.sweep_in): descend to depth
    on the +y side of the sock, then rotate in while closing so the moving jaw
    scoops the sock's outer edge into the fixed jaw. A straight-down pinch on
    this compressible sock fails ~every time (the fold extrudes out on lift);
    the sweep held torH ~148 through the lift in validation.

    Everything EXCEPT the sweep runs fast, and the transit legs flow into each
    other (settle=False) so there is no stop-restart before the pick. The pick
    itself is deliberately left at its tuned pace.
    """
    place = place or (px, py)
    tor = {}

    # Side approach: descend STRAIGHT down the +y side to just above the
    # object's depth (gz + z_blend), then hand off to sweep_in, which folds the
    # final z drop into the inward lateral move so it curves down-and-in as one
    # arc. settle=False on both legs -> they flow continuously into the sweep.
    c.move(px, py + approach_dy, gz + hover, GRIP_OPEN, speed=130.0,
           settle=False, rec=rec, phase='approach')
    c.move(px, py + approach_dy, gz + z_blend, speed=70.0,
           settle=False, rec=rec, phase='descend')
    if settle_s:
        c.spin(settle_s)
    c.sweep_in(px, py, gz, approach_dy, overshoot, close_to,
               close_start=close_start, z_blend=z_blend, rec=rec, phase='grasp')
    ok, th, ta, gap = c.held()
    tor['close'] = {'torH': th, 't': round(ta, 3), 'gap': round(gap, 3)}
    if not ok:
        return False, f'empty sweep (torH {th}, claw {ta:.3f}, gap {gap:.3f})', tor, place

    # Lift straight up (no radius change) to the top -- this is the clean,
    # no-lean motion. Pulling the radius IN would reach higher (~117 vs ~88 mm)
    # but the in/out lean and the kink at the direction change read as "leaning"
    # and "picking twice", so we stay at one radius. gz + hover is the top.
    c.move(px, py - overshoot, gz + hover, speed=90.0, settle=False,
           rec=rec, phase='lift')
    ok, th, ta, gap = c.held()
    tor['lift'] = {'torH': th, 't': round(ta, 3), 'gap': round(gap, 3)}
    if not ok:
        return False, f'dropped on lift (torH {th}, claw {ta:.3f}, gap {gap:.3f})', tor, place

    # Transit across at the top, then straight down to set down gently.
    c.move(place[0], place[1], gz + hover, speed=130.0, settle=False,
           rec=rec, phase='transit')
    c.move(place[0], place[1], gz + 6.0, speed=55.0, settle=False,
           rec=rec, phase='place')
    tor['place'] = c.torh()
    # Aware release: the single moving jaw is on the +y side, so opening it
    # drags the sock toward -y unless the base eases the same way. Rotate a few
    # mm toward +y (left) WHILE the claw ramps open and the arm rises, so the
    # sock is set down cleanly instead of being nudged as the jaw retracts.
    c.move(place[0], place[1] + release_dy, gz + 45.0, t=GRIP_OPEN, speed=40.0,
           grip_ramp=True, settle=False, rec=rec, phase='release')
    c.move(place[0], place[1] + release_dy, gz + hover, speed=110.0,
           settle=False, rec=rec, phase='retreat')
    return True, 'ok', tor, place


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', type=int, default=1)
    ap.add_argument('--grasp-z', type=float, required=True)
    ap.add_argument('--pick', type=str, default=None, help='x,y (default: current)')
    ap.add_argument('--hover', type=float, default=85.0,
                    help='mm above the grasp for the top of lift/transit/'
                         'retreat; ~85 is the clean straight-up reach ceiling '
                         'at this radius (higher needs a pull-in that leans)')
    ap.add_argument('--jitter', type=float, default=0.0,
                    help='mm of random place offset (per axis) each episode; '
                         '0 = put it straight back')
    ap.add_argument('--jitter-bound', type=float, default=32.0,
                    help='keep jittered place points within +/- this of the '
                         'ORIGINAL pick centre, so the random walk cannot drift '
                         'out of reach or out of frame over a long run')
    ap.add_argument('--out', type=str, default='/demos')
    ap.add_argument('--prompt', type=str, default='pick up the sock')
    ap.add_argument('--close-to', type=float, default=GRIP_CLOSED,
                    help='claw close target; below 3.14 squeezes a soft object '
                         'less and stops extruding it out of the jaws')
    ap.add_argument('--test-grasp', action='store_true')
    ap.add_argument('--test-sweep', action='store_true')
    ap.add_argument('--approach-dy', type=float, default=55.0,
                    help='+y (robot-left) side offset the sweep starts from; '
                         'larger = come in more fully from the left')
    ap.add_argument('--overshoot', type=float, default=6.0,
                    help='how far past sock centre (toward -y) the sweep ends')
    ap.add_argument('--close-start', type=float, default=0.12,
                    help='fraction of the sweep before the claw begins closing; '
                         'lower = claw closes WHILE still rotating in (fluid '
                         'catch-and-scoop) rather than contact-stop-then-close')
    ap.add_argument('--z-blend', type=float, default=25.0,
                    help='mm of final descent folded into the start of the '
                         'sweep, so the tool curves down-and-in as one arc '
                         'instead of descend-stall-sweep')
    ap.add_argument('--release-dy', type=float, default=8.0,
                    help='mm the base eases toward +y (left, the moving-jaw '
                         'side) while releasing, so the sock is not nudged as '
                         'the single jaw retracts')
    ap.add_argument('--pose', type=str, default=None,
                    help='x,y,z,t -- smooth move there and exit (diagnostics)')
    ap.add_argument('--probe', type=str, default=None,
                    help='x,y -- descend with the claw shut until it touches '
                         'down, to measure the support surface height')
    ap.add_argument('--probe-from', type=float, default=-60.0)
    ap.add_argument('--start', type=int, default=None)
    a = ap.parse_args()

    rclpy.init()
    c = Cycle()
    if not c.wait_ready():
        print('FAIL: no /teleop/state, no camera, or teleop_node not subscribed')
        return 1
    if c.estopped():
        print('FAIL: E-STOP is engaged -- clear it in the console first')
        return 1

    p = c.pose()
    print(f'arm at ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) grip={p[3]:.2f} '
          f'torH={c.torh()}')
    if a.pick:
        px, py = (float(v) for v in a.pick.split(','))
    else:
        px, py = p[0], p[1]
    print(f'pick point ({px:.1f}, {py:.1f}) grasp z {a.grasp_z} '
          f'hover +{a.hover}')

    if a.probe:
        # Contact = the commanded z runs away downward while the measured z
        # stops following, because the arm is now resting on the surface.
        x, y = (float(v) for v in a.probe.split(','))
        c.move(x, y, a.probe_from, GRIP_CLOSED, speed=50.0)
        c.spin(0.5)
        z, contact = a.probe_from, None
        while z > Z_MIN + 6:
            z -= 4.0
            c.send(x, y, z, GRIP_CLOSED)
            c.spin(0.45)
            p = c.pose()
            arm = (c.state or {}).get('arm') or {}
            err = p[2] - z
            print(f'  cmd {z:7.1f}  meas {p[2]:7.1f}  lag {err:5.1f}  '
                  f'torS {arm.get("torS")} torE {arm.get("torE")}')
            if err > 6.0:
                contact = p[2]
                break
        print(f'\nCONTACT at measured z = {contact:.1f}' if contact is not None
              else '\nno contact found before z limit')
        c.move(x, y, a.probe_from, speed=50.0)
        c.destroy_node(); rclpy.shutdown()
        return 0

    if a.pose:
        x, y, z, t = (float(v) for v in a.pose.split(','))
        c.move(x, y, z, t, speed=40.0)
        c.spin(0.6)
        p = c.pose()
        print(f'now ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) grip={p[3]:.3f} '
              f'torH={c.torh()}')
        c.destroy_node(); rclpy.shutdown()
        return 0

    if a.test_sweep:
        # straight down the +y side to just above depth, then blended sweep-in
        c.move(px, py + a.approach_dy, a.grasp_z + a.hover, GRIP_OPEN,
               speed=70.0)
        c.move(px, py + a.approach_dy, a.grasp_z + a.z_blend, speed=45.0)
        c.sweep_in(px, py, a.grasp_z, a.approach_dy, a.overshoot, a.close_to,
                   close_start=a.close_start, z_blend=a.z_blend)
        okh, th, ta, gap = c.held()
        print(f'  after sweep: torH {th} claw {ta:.3f} gap {gap:.3f} '
              f'held={okh}')
        c.move(px, py - a.overshoot, a.grasp_z + a.hover, speed=45.0)
        okh, th, ta, gap = c.held()
        print(f'  after lift : torH {th} claw {ta:.3f} gap {gap:.3f} '
              f'held={okh}')
        print(f'\n{"SWEEP PICK OK" if okh else "SWEEP FAILED"}')
        c.destroy_node(); rclpy.shutdown()
        return 0 if okh else 2

    if a.test_grasp:
        ok, why, tor, _ = cycle(c, px, py, a.grasp_z, a.hover, None,
                                 close_to=a.close_to, approach_dy=a.approach_dy,
                                 overshoot=a.overshoot, close_start=a.close_start,
                                 z_blend=a.z_blend, release_dy=a.release_dy)
        print(f'\n{"PICK OK" if ok else "PICK FAILED"}: {why}')
        print(f'torques: {tor}   (held needs |torH| >= {HELD_TORH_MIN}, '
              f'empty close reads ~{EMPTY_TORH})')
        c.destroy_node(); rclpy.shutdown()
        return 0 if ok else 2

    root = Path(a.out)
    root.mkdir(parents=True, exist_ok=True)
    idx = a.start if a.start is not None else (
        1 + max([int(d.name[3:]) for d in root.glob('ep_*')] or [-1]))

    ok_n = fail_n = 0
    consec = 0
    cx0, cy0 = px, py            # original centre; jitter is bounded around it
    for k in range(a.episodes):
        if a.jitter:
            # Random WALK (place becomes next pick, no perception), but bounded
            # to a box around the original centre so it cannot drift out of
            # reach or out of frame over a long run.
            place = (clamp(px + random.uniform(-a.jitter, a.jitter),
                           cx0 - a.jitter_bound, cx0 + a.jitter_bound),
                     clamp(py + random.uniform(-a.jitter, a.jitter),
                           cy0 - a.jitter_bound, cy0 + a.jitter_bound))
        else:
            place = (px, py)
        rec = Recorder(root, idx, a.prompt)
        try:
            ok, why, tor, place = cycle(c, px, py, a.grasp_z, a.hover, rec,
                                        place=place, close_to=a.close_to,
                                        approach_dy=a.approach_dy,
                                        overshoot=a.overshoot,
                                        close_start=a.close_start,
                                        z_blend=a.z_blend,
                                        release_dy=a.release_dy)
        except RuntimeError as e:
            rec.discard()
            print(f'ep {idx}: ABORT {e}')
            break
        if ok:
            rec.finish(True, {'pick': [px, py, a.grasp_z],
                              'place': [place[0], place[1], a.grasp_z],
                              'torque': tor})
            ok_n += 1; consec = 0
            print(f'ep {idx:4d}: OK   {rec.n:3d} frames  torH {tor}')
            # The sock now lives where we put it -- chain forward, exactly as
            # the grid collector does, so ground truth needs no perception.
            px, py = place
            idx += 1
        else:
            rec.discard()
            fail_n += 1; consec += 1
            print(f'ep {idx:4d}: FAIL {why}  [discarded]')
            try:
                c.grip(GRIP_OPEN, secs=0.6)
                c.move(px, py, a.grasp_z + a.hover, speed=45.0)
            except RuntimeError as e:
                print(f'  recovery aborted: {e}')
                break
            if consec >= 3:
                print('\n3 misses in a row -- the sock has almost certainly '
                      'moved out from under the pick point. Stopping rather '
                      'than flailing at empty air.')
                break

    print(f'\n{ok_n} recorded, {fail_n} discarded -> {root}')
    c.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
