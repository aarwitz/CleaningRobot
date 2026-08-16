#!/usr/bin/env python3
"""Scripted pick-and-place demonstration collector for pi0.5 fine-tuning.

The object starts at a known arm-frame pose. Each episode the arm picks it
there and places it at a randomly chosen point on a pre-calibrated workspace
grid; that place point becomes the next episode's pick point, so ground truth
chains forward without any perception in the loop.

Every waypoint is sampled at ~10 Hz into a trajectory of (camera frame, joint
state, torques, commanded target), which is what LeRobot conversion consumes.

Hardware facts measured on this arm (2026-07-21) that this script depends on:
  * The RoArm echoes commands, so feedback ({"T":105} -> {"T":1051,...}) must
    be read past the echo line.
  * Gripper commands clamp to [1.08 open, 3.14 closed]. Closing on nothing
    settles at t=3.132; anything held keeps it below ~3.11. That is the
    grasp detector — this arm has no other sensor.
  * There is no single floor height: this is a 4-DoF arm whose wrist tilts
    with posture, so floor contact ranges -205..-217 mm over the workspace.
    Each grid point is probed once (descend until the elbow torque spikes and
    commanded z stops tracking) and cached.
  * Reach dies around r=330 mm; r<=265 is safe.

THE PICK SURFACE MUST BE RAISED ~100 mm (2026-07-21). The D455 is bolted to the
arm's rotating base, ~120 mm off the floor, pitched only 2.5 deg down, with a
~42 deg vertical FOV — so it sees down to about -18.5 deg. A floor pick at
r=250 sits atan(120/250) = 25.6 deg below the lens, i.e. ~7 deg BELOW the
bottom of the frame. Episodes recorded that way show the gripper occluding the
centre and the object clipped off the bottom edge, which is useless for
training a policy. Standing the object on a flat, rigid ~100 mm platform puts
it at ~4.6 deg — mid-frame — and the probe below starts high enough to clear it.

Run INSIDE the container with arm_bridge STOPPED (this owns the serial port):
  docker exec docker-vision-1 kill $(pgrep -f arm_bridge_node)

  python3 scripts/collect_pick_demos.py --calibrate          # probe the grid
  python3 scripts/collect_pick_demos.py --episodes 20        # collect
  python3 scripts/collect_pick_demos.py --point-at 250,0     # park as a marker
"""
import argparse
import json
import math
import random
import threading
import time
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None

# ── gripper (command values clamp to these actuals) ──────────────────────────
OPEN, CLOSED = 0.0, 3.14
# Grasp detection. Closing on nothing settles at t=3.132 with torH=-32; a held
# sock reads torH=-80 closing and -64 after the lift, while t barely moves
# (3.114). Gripper TORQUE is therefore the discriminating signal, not angle.
EMPTY_CLOSE_T, EMPTY_TORH = 3.132, 32
HELD_TORH_MIN = 50           # |torH| above this => something is in the gripper
HELD_T_MAX = 3.115           # corroborating (weak) angle signal


def is_held(fb):
    if fb is None:
        return False
    return abs(fb.get('torH', 0)) >= HELD_TORH_MIN or fb.get('t', 9) <= HELD_T_MAX

# ── workspace (arm frame, mm) ───────────────────────────────────────────────
# Valid picks and drops happen in an ANNULUS, not just "within reach". The
# RealSense rides the arm's rotating base at the front of the robot, so any
# pose with a small radius swings whatever is in the gripper back into the
# camera. Observed 2026-07-21: a move to r=150 pressed the held toy flat
# against the right lens. R_WORK_MIN is the near wall of that annulus.
# 200 was still close enough that picks crowded the lens, so the near wall was
# pushed out to 235 (operator request 2026-07-21). Picks and drops now live in
# the outer band only; bearing +/-24 deg still gives ~+/-100 mm of lateral room
# at r=250, which is where the random place-transforms get their spread.
R_WORK_MIN = 235.0           # inside this, a carried object crowds the camera
R_SAFE_MAX = 265.0           # max radius for LOW targets (floor picks)
R_ELEVATED_MAX = 300.0       # max radius for raised targets (z >= -150)
HOME = (235.0, 0.0, 235.0)
LIFT = 120.0                 # transit height above the floor
APPROACH = 60.0              # hover height before descending


def max_radius_at(z):
    """Reach is z-dependent: the arm folds down to touch the floor, so its
    horizontal envelope SHRINKS at low z and opens up higher. R_SAFE_MAX=265
    was measured for floor picks (z about -210). Measured 2026-06: the arm
    reached (300, 0) down to z=-161, so elevated targets legitimately extend
    past 265 -- clamping them to 265 made reachable objects look unreachable.
    """
    if z is None or z <= -180.0:
        return R_SAFE_MAX
    if z >= -150.0:
        return R_ELEVATED_MAX
    span = (z - (-180.0)) / 30.0            # linear blend over -180..-150
    return R_SAFE_MAX + span * (R_ELEVATED_MAX - R_SAFE_MAX)


def clamp_to_workspace(x, y, z=None):
    """Push (x, y) into the valid annulus, preserving bearing.

    Clamps rather than raises so a stray waypoint degrades into a safe one
    instead of aborting a run mid-episode; the warning makes it visible.
    """
    r = math.hypot(x, y)
    if r < 1e-6:
        return R_WORK_MIN, 0.0
    r_new = min(max(r, R_WORK_MIN), max_radius_at(z))
    if abs(r_new - r) > 0.5:
        print(f'  [workspace] r={r:.0f} -> {r_new:.0f} '
              f'({"too close to camera" if r < R_WORK_MIN else "beyond reach"})')
        s = r_new / r
        return x * s, y * s
    return x, y


def grid_points(radii=(205, 235, 262), bearings_deg=(-24, -12, 0, 12, 24)):
    pts = []
    for r in radii:
        for b in bearings_deg:
            a = math.radians(b)
            x, y = r * math.cos(a), r * math.sin(a)
            if math.hypot(x, y) <= R_SAFE_MAX:
                pts.append((round(x, 1), round(y, 1)))
    return pts


class Arm:
    """RoArm M2-S serial driver. All I/O is serialized on one lock so the
    sampler can read feedback while a motion command is in flight."""

    def __init__(self, port, baud, dry_run=False):
        self.dry_run = dry_run
        self.lock = threading.Lock()
        self.ser = None
        if dry_run:
            return
        if serial is None:
            raise RuntimeError('pyserial missing and not --dry-run')
        self.ser = serial.Serial(port, baud, dsrdtr=None, timeout=0.5)
        self.ser.setRTS(False)
        self.ser.setDTR(False)
        time.sleep(1.0)
        self.ser.reset_input_buffer()

    def send(self, cmd):
        if self.dry_run:
            return
        with self.lock:
            self.ser.write(json.dumps(cmd).encode() + b'\n')

    def feedback(self, tries=10):
        """Return the T:1051 payload, skipping the command echo."""
        if self.dry_run:
            return {'x': 0., 'y': 0., 'z': 0., 'b': 0., 's': 0., 'e': 0.,
                    't': 1.08, 'torB': 0, 'torS': 0, 'torE': 0, 'torH': 0}
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(b'{"T":105}\n')
            for _ in range(tries):
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                try:
                    d = json.loads(line)
                except ValueError:
                    continue
                # NB: a T:104 echo also carries x/y/z/t, so match on the joint
                # keys that only real feedback has — otherwise the echo is
                # recorded as a sample with null joints and commanded cart.
                if d.get('T') == 1051 or ('b' in d and 'e' in d):
                    return d
        return None

    def move(self, x, y, z, grip, spd=0.25):
        # Every cartesian move goes through the annulus guard — including
        # ad-hoc poses from one-off scripts, which is how the camera got
        # fouled in the first place.
        x, y = clamp_to_workspace(x, y, z)
        self.send({'T': 104, 'x': x, 'y': y, 'z': z, 't': grip, 'spd': spd})

    def grip(self, x, y, z, grip, settle=3.0, spd=0.25):
        """Actuate the jaw at a held pose and WAIT for it to finish.

        The jaw is slow and a following cartesian move preempts it mid-travel,
        so the claw never fully closes/opens before the arm leaves. Keep the
        pose identical to the previous waypoint so only the jaw moves.
        """
        self.move(x, y, z, grip, spd)
        time.sleep(settle)

    def go(self, x, y, z, grip, dwell, spd=0.25):
        self.move(x, y, z, grip, spd)
        time.sleep(dwell)

    def home(self, dwell=2.5):
        self.go(*HOME, OPEN, dwell)

    def close(self):
        if self.ser:
            self.ser.close()


def probe_surface(arm, x, y, start_z=170.0, coarse=16.0, fine=4.0, err_limit=8.0,
                  tor_limit=170.0, min_z=-280.0):
    """Descend at (x, y) until the wrist stops tracking the commanded z or the
    elbow torque spikes. Returns the contact z, or None if never reached.

    Starts from ABOVE any elevated pick surface. The floor-only version began
    at -150 mm, which drives the gripper into the side of anything raised — and
    a raised surface is exactly what keeps the object inside the camera's view
    (a floor pick at r=250 sits ~26 deg below the lens, past the frame edge).

    Two passes so the higher start costs little time: coarse steps find the
    surface, then a fine pass from just above it recovers the accuracy the old
    6 mm descent had.
    """
    def descend(from_z, step):
        arm.go(x, y, from_z, OPEN, 2.2)
        z = from_z
        while z > min_z:
            z -= step
            arm.go(x, y, z, OPEN, 0.9)
            fb = arm.feedback()
            if fb is None:
                return None
            if (fb['z'] - z) > err_limit or fb['torE'] > tor_limit:
                return fb['z']
        return None

    hit = descend(start_z, coarse)
    if hit is None:
        return None
    hit = descend(hit + 2 * coarse, fine) or hit
    arm.go(x, y, hit + 60, OPEN, 1.2)
    return round(hit, 1)


def on_pick_surface(floors, start, span):
    """Grid points sitting on the same surface as the pick point.

    A point that overhangs a raised platform probes down to the floor ~100 mm
    lower. Placing the object there would drop it out of the camera's view and
    break the chain that makes the next episode's pick position known.
    """
    return {p: z for p, z in floors.items() if abs(z - floors[start]) <= span}


class Recorder:
    """Camera frames + arm state sampled through every motion."""

    def __init__(self, root, use_camera=True,
                 topic='/camera/color/image_raw/compressed'):
        self.root = Path(root)
        self.use_camera = use_camera
        self.ep = None
        self.rows = []
        self.frame_n = 0
        self.latest = None
        if not use_camera:
            return
        import rclpy
        from sensor_msgs.msg import CompressedImage
        rclpy.init()
        self._rclpy = rclpy
        self._node = rclpy.create_node('demo_recorder')
        self._node.create_subscription(
            CompressedImage, topic, lambda m: setattr(self, 'latest', bytes(m.data)), 5)

        # spin_once in a stoppable loop: rclpy.spin() cannot be interrupted, and
        # tearing the context down under it aborts the process at exit
        self._stop = threading.Event()

        def _spin():
            while not self._stop.is_set() and rclpy.ok():
                try:
                    rclpy.spin_once(self._node, timeout_sec=0.1)
                except Exception:
                    break
        self._thread = threading.Thread(target=_spin, daemon=True)
        self._thread.start()

    def wait_for_camera(self, timeout=15.0):
        if not self.use_camera:
            return True
        t0 = time.time()
        while self.latest is None and time.time() - t0 < timeout:
            time.sleep(0.2)
        return self.latest is not None

    def start(self, idx, meta):
        self.ep = self.root / f'ep_{idx:04d}'
        (self.ep / 'frames').mkdir(parents=True, exist_ok=True)
        self.rows = []
        self.frame_n = 0
        self.t0 = time.time()
        self.meta = dict(meta, episode=idx, t_start=self.t0)

    def sample(self, arm, cmd, phase):
        """One trajectory row: frame + joint state + torques + commanded target."""
        fb = arm.feedback()
        if fb is None:
            return
        frame = None
        if self.use_camera and self.latest is not None:
            frame = f'{self.frame_n:06d}.jpg'
            (self.ep / 'frames' / frame).write_bytes(self.latest)
            self.frame_n += 1
        self.rows.append({
            't': round(time.time() - self.t0, 3),
            'frame': frame,
            'phase': phase,
            # joint state (rad): base, shoulder, elbow, hand
            'joints': [fb.get(k) for k in ('b', 's', 'e', 't')],
            'cart': [fb.get(k) for k in ('x', 'y', 'z')],
            'torque': [fb.get(k) for k in ('torB', 'torS', 'torE', 'torH')],
            'cmd': list(cmd),          # commanded x, y, z, grip
        })

    def end(self, success, extra=None):
        self.meta.update(t_end=time.time(), success=success,
                         n_frames=self.frame_n, n_rows=len(self.rows),
                         **(extra or {}))
        (self.ep / 'meta.json').write_text(json.dumps(self.meta, indent=1))
        with (self.ep / 'traj.jsonl').open('w') as f:
            for r in self.rows:
                f.write(json.dumps(r) + '\n')

    def close(self):
        if not self.use_camera:
            return
        self._stop.set()
        self._thread.join(timeout=2.0)
        try:
            self._node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass


def sampled_move(arm, rec, x, y, z, grip, dwell, phase, hz=10.0):
    """Command a waypoint and sample the whole motion into the episode."""
    arm.move(x, y, z, grip)
    end = time.time() + dwell
    while time.time() < end:
        if rec is not None and rec.ep is not None:
            rec.sample(arm, (x, y, z, grip), phase)
        time.sleep(1.0 / hz)


# Offsets searched around the nominal pick point when the first close misses.
# Grasping flat terry cloth with these tapered prongs is very fold-dependent:
# a 7-position sweep found exactly one spot that survived a lift, so a miss at
# the nominal point says nothing about the neighbours.
SEARCH_OFFSETS = ((0, 0), (0, 22), (0, -22), (0, 44), (0, -44), (-18, 0), (18, 0))
VERIFY_LIFT = 60.0           # mm; closing torque alone is NOT proof of a grasp


def grasp(arm, rec, x, y, floor_z, depth=12.0):
    """Close on the object and verify by lifting.

    Depth stays ABOVE the probed contact height — the fingertips reach the
    wood before the probe's torque threshold trips, so floor_z sits a few mm
    below true contact and pressing into it scrapes the floor without
    improving the grip.

    Returns (held, actual_xy, grasp_z) with the arm already lifted by
    VERIFY_LIFT when held, since that lift doubles as the episode's own.
    """
    z = floor_z + depth
    for i, (dx, dy) in enumerate(SEARCH_OFFSETS):
        gx, gy = x + dx, y + dy
        phase = 'grasp' if i == 0 else 'research'
        sampled_move(arm, rec, gx, gy, floor_z + APPROACH, OPEN, 1.6, 'approach')
        sampled_move(arm, rec, gx, gy, z, OPEN, 1.8, 'descend')
        sampled_move(arm, rec, gx, gy, z, CLOSED, 2.2, phase)
        sampled_move(arm, rec, gx, gy, z + VERIFY_LIFT, CLOSED, 1.9, 'lift')
        fb = arm.feedback()
        if is_held(fb):
            print(f'    grasp at ({gx:.0f},{gy:.0f}) z={z:.0f}: '
                  f'torH={fb["torH"]} -> HELD')
            return True, (gx, gy), z
        if i == 0:
            print(f'    nominal point empty (torH={fb["torH"]}); searching...')
        sampled_move(arm, rec, gx, gy, z, CLOSED, 1.7, 'regrip')
        sampled_move(arm, rec, gx, gy, z, OPEN, 1.2, 'regrip')
    return False, None, None


def release(arm, rec, x, y, floor_z):
    sampled_move(arm, rec, x, y, floor_z + LIFT, CLOSED, 1.8, 'transit')
    sampled_move(arm, rec, x, y, floor_z + 16.0, CLOSED, 1.8, 'place')
    sampled_move(arm, rec, x, y, floor_z + 16.0, OPEN, 1.5, 'release')
    sampled_move(arm, rec, x, y, floor_z + LIFT, OPEN, 1.5, 'retreat')


def run_episode(arm, rec, idx, pick, place, floors, prompt):
    px, py = pick
    qx, qy = place
    rec.start(idx, {'pick': [px, py, floors[pick]], 'place': [qx, qy, floors[place]],
                    'prompt': prompt})
    held, actual, gz = grasp(arm, rec, px, py, floors[pick])
    if not held:
        # retreat BEFORE closing the episode, else those samples write frames
        # that no trajectory row references
        sampled_move(arm, rec, px, py, floors[pick] + LIFT, OPEN, 1.5, 'abort')
        rec.end(False, extra={'failure': 'grasp'})
        return False
    ax, ay = actual
    sampled_move(arm, rec, ax, ay, floors[pick] + LIFT, CLOSED, 1.8, 'lift')
    fb = arm.feedback()
    if not is_held(fb):
        print('    object dropped during lift')
        rec.end(False, extra={'failure': 'dropped'})
        return False
    release(arm, rec, qx, qy, floors[place])
    sampled_move(arm, rec, *HOME, OPEN, 2.0, 'home')
    rec.end(True, extra={'grasp_z': gz, 'actual_pick': [ax, ay]})
    return True


def load_floors(path):
    p = Path(path)
    if not p.exists():
        return {}
    return {tuple(map(float, k.split(','))): v
            for k, v in json.loads(p.read_text()).items()}


def save_floors(path, floors):
    Path(path).write_text(json.dumps(
        {f'{x},{y}': z for (x, y), z in floors.items()}, indent=1))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--episodes', type=int, default=20)
    ap.add_argument('--out', default='/demos')
    ap.add_argument('--floors', default='/demos/floor_map.json')
    ap.add_argument('--start', default='250,0', help='object start point "x,y" (mm)')
    ap.add_argument('--prompt', default='pick up the sock')
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--surface-tol', type=float, default=15.0,
                    help='mm the cached surface map may drift before refusing to run')
    ap.add_argument('--surface-span', type=float, default=25.0,
                    help='mm a grid point may sit off the pick surface and still be used')
    ap.add_argument('--calibrate', action='store_true', help='probe the floor grid')
    ap.add_argument('--point-at', default=None,
                    help='park the open gripper hovering over "x,y" and exit')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--no-camera', action='store_true')
    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)
    arm = Arm(args.port, args.baud, args.dry_run)
    floors = load_floors(args.floors)

    if args.point_at:
        x, y = (float(v) for v in args.point_at.split(','))
        fz = floors.get((x, y), -210.0)
        print(f'parking over ({x:.0f}, {y:.0f}); place the object under the gripper')
        arm.go(x, y, fz + 95, OPEN, 3.0)
        arm.close()
        return

    if args.calibrate or not floors:
        pts = grid_points()
        print(f'calibrating {len(pts)} grid points...')
        for (x, y) in pts:
            z = probe_surface(arm, x, y)
            floors[(x, y)] = z
            print(f'  ({x:6.1f},{y:6.1f}) floor z = {z}')
        arm.home()
        floors = {k: v for k, v in floors.items() if v is not None}
        save_floors(args.floors, floors)
        print(f'saved {len(floors)} points -> {args.floors}')
        if args.calibrate:
            arm.close()
            return

    start = tuple(float(v) for v in args.start.split(','))
    if start not in floors:
        start = min(floors, key=lambda p: math.dist(p, start))
        print(f'start point snapped to nearest calibrated grid point {start}')

    # A cached map from a different pick surface is dangerous, not just wrong:
    # every descent is commanded to surface+depth, so a map taken on the floor
    # would drive the gripper ~100 mm into a raised platform at full torque.
    # Re-probe one point and refuse to run if reality has moved.
    check = probe_surface(arm, *start)
    if check is None:
        print(f'no contact at {start}; is the pick surface in reach?')
        arm.close()
        return
    if abs(check - floors[start]) > args.surface_tol:
        print(f'surface at {start} is {check} but the map says {floors[start]} '
              f'(> {args.surface_tol:.0f} mm). Re-run with --calibrate.')
        arm.close()
        return
    floors[start] = check

    on_surface = on_pick_surface(floors, start, args.surface_span)
    if len(on_surface) < len(floors):
        print(f'ignoring {len(floors) - len(on_surface)} grid point(s) off the '
              f'pick surface (>{args.surface_span:.0f} mm from it)')
    if len(on_surface) < 2:
        print('need at least 2 points on the pick surface; use a wider platform '
              'or re-run with --calibrate')
        arm.close()
        return
    floors = on_surface

    rec = Recorder(args.out, use_camera=not args.no_camera)
    if not rec.wait_for_camera():
        print('WARNING: no camera frames; recording state only')

    pick, ok = start, 0
    for i in range(args.episodes):
        place = random.choice([p for p in floors if p != pick])
        print(f'ep {i}: pick {pick} -> place {place}')
        if run_episode(arm, rec, i, pick, place, floors, args.prompt):
            ok += 1
            pick = place            # ground truth chains forward
        else:
            print('  episode failed; object position is no longer known - stopping')
            break
    arm.home()
    arm.close()
    print(f'done: {ok}/{args.episodes} episodes -> {args.out}')
    rec.close()


if __name__ == '__main__':
    main()
