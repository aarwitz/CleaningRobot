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
EMPTY_CLOSE_T = 3.132        # measured: gripper closed on nothing
HELD_T_MAX = 3.110           # below this after closing => something is held

# ── workspace (arm frame, mm) ───────────────────────────────────────────────
R_SAFE_MAX = 265.0
HOME = (235.0, 0.0, 235.0)
LIFT = 120.0                 # transit height above the floor
APPROACH = 60.0              # hover height before descending


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
        self.send({'T': 104, 'x': x, 'y': y, 'z': z, 't': grip, 'spd': spd})

    def go(self, x, y, z, grip, dwell, spd=0.25):
        self.move(x, y, z, grip, spd)
        time.sleep(dwell)

    def home(self, dwell=2.5):
        self.go(*HOME, OPEN, dwell)

    def close(self):
        if self.ser:
            self.ser.close()


def probe_floor(arm, x, y, start_z=-150.0, step=6.0, err_limit=7.0,
                tor_limit=140.0, floor_min=-280.0):
    """Descend at (x, y) until the wrist stops tracking the commanded z or the
    elbow torque spikes. Returns the contact z, or None if never reached."""
    arm.go(x, y, start_z, OPEN, 2.6)
    z = start_z
    while z > floor_min:
        z -= step
        arm.go(x, y, z, OPEN, 1.15)
        fb = arm.feedback()
        if fb is None:
            return None
        if (fb['z'] - z) > err_limit or fb['torE'] > tor_limit:
            contact = fb['z']
            arm.go(x, y, min(start_z, contact + 60), OPEN, 1.2)
            return round(contact, 1)
    return None


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


def grasp(arm, rec, x, y, floor_z, offsets=(4.0, -1.0, -6.0)):
    """Close on the object, verifying with gripper feedback. Retries lower on
    failure — flat fabric needs the fingertips at floor level."""
    for i, off in enumerate(offsets):
        z = floor_z + off
        sampled_move(arm, rec, x, y, floor_z + APPROACH, OPEN, 1.6, 'approach')
        sampled_move(arm, rec, x, y, z, OPEN, 1.8, 'descend')
        sampled_move(arm, rec, x, y, z, CLOSED, 2.0, 'grasp')
        fb = arm.feedback()
        held = fb is not None and fb['t'] < HELD_T_MAX
        print(f'    grasp try {i + 1} at z={z:.1f}: t={fb["t"]:.3f} '
              f'torH={fb["torH"]} -> {"HELD" if held else "empty"}')
        if held:
            return True, z
        sampled_move(arm, rec, x, y, z, OPEN, 1.0, 'regrip')
    return False, None


def release(arm, rec, x, y, floor_z):
    sampled_move(arm, rec, x, y, floor_z + LIFT, CLOSED, 1.8, 'transit')
    sampled_move(arm, rec, x, y, floor_z + 12.0, CLOSED, 1.8, 'place')
    sampled_move(arm, rec, x, y, floor_z + 12.0, OPEN, 1.5, 'release')
    sampled_move(arm, rec, x, y, floor_z + LIFT, OPEN, 1.5, 'retreat')


def run_episode(arm, rec, idx, pick, place, floors, prompt):
    px, py = pick
    qx, qy = place
    rec.start(idx, {'pick': [px, py, floors[pick]], 'place': [qx, qy, floors[place]],
                    'prompt': prompt})
    held, gz = grasp(arm, rec, px, py, floors[pick])
    if not held:
        # retreat BEFORE closing the episode, else those samples write frames
        # that no trajectory row references
        sampled_move(arm, rec, px, py, floors[pick] + LIFT, OPEN, 1.5, 'abort')
        rec.end(False, extra={'failure': 'grasp'})
        return False
    sampled_move(arm, rec, px, py, floors[pick] + LIFT, CLOSED, 1.8, 'lift')
    fb = arm.feedback()
    if fb is not None and fb['t'] >= HELD_T_MAX:
        print('    object dropped during lift')
        rec.end(False, extra={'failure': 'dropped'})
        return False
    release(arm, rec, qx, qy, floors[place])
    sampled_move(arm, rec, *HOME, OPEN, 2.0, 'home')
    rec.end(True, extra={'grasp_z': gz})
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
            z = probe_floor(arm, x, y)
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
