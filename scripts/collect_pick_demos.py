#!/usr/bin/env python3
"""Scripted pick-and-place demonstration collector for pi0.5 fine-tuning.

The object starts at a known arm-frame pose. Each episode the arm does an
exact scripted pick there, then places at a randomly transformed pose inside
the reachable workspace sector; that place pose becomes the next episode's
pick pose, so ground truth is never lost. Camera frames, commanded targets,
and arm feedback are recorded per episode for later LeRobot conversion.

Run INSIDE the container with arm_bridge STOPPED (this script owns the serial
port). The camera stack must be up for frame recording.

  # find the floor height first (steps the open gripper down until you ^C):
  python3 scripts/collect_pick_demos.py --calibrate

  # collect 20 episodes, object starting at (250, 0) on the floor:
  python3 scripts/collect_pick_demos.py --episodes 20 --floor-z -95

  # logic test without hardware/ROS:
  python3 scripts/collect_pick_demos.py --dry-run --no-camera --episodes 3
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

# ── Workspace (arm frame, mm): reachable floor sector in front of the robot ──
R_MIN, R_MAX = 180.0, 320.0        # annulus radii
BEARING_MAX = math.radians(40.0)   # keep object inside camera FOV at home pan
MIN_MOVE_MM = 60.0                 # don't place on top of the pick point

GRIP_OPEN, GRIP_CLOSED = 0.0, 3.14
HOME = (235.0, 0.0, 235.0)
LIFT_MM = 130.0                    # hover height above the floor for transits
JUST_ABOVE_MM = 75.0


class Arm:
    """Minimal RoArm M2-S serial driver (T:104 cartesian, T:105 feedback)."""

    def __init__(self, port, baud, dry_run):
        self.dry_run = dry_run
        self.lock = threading.Lock()
        self.ser = None
        if not dry_run:
            if serial is None:
                raise RuntimeError('pyserial missing and not --dry-run')
            self.ser = serial.Serial(port, baudrate=baud, dsrdtr=None, timeout=0.5)
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            time.sleep(1.0)
            self.ser.reset_input_buffer()

    def send(self, cmd, dwell):
        line = json.dumps(cmd)
        if self.dry_run:
            print(f'[dry] {line} (dwell {dwell:.1f}s)')
            time.sleep(min(dwell, 0.05))
            return
        with self.lock:
            self.ser.write(line.encode() + b'\n')
        time.sleep(dwell)

    def move(self, x, y, z, grip, dwell, spd=0.25):
        self.send({'T': 104, 'x': x, 'y': y, 'z': z, 't': grip, 'spd': spd}, dwell)

    def feedback(self):
        """Query T:105 pose feedback; returns dict or None."""
        if self.dry_run:
            return None
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(b'{"T":105}\n')
            raw = self.ser.readline()
        try:
            return json.loads(raw.decode(errors='ignore'))
        except (ValueError, AttributeError):
            return None


class Recorder:
    """Camera frames + commanded/measured arm stream for one episode."""

    def __init__(self, root, use_camera):
        self.root = Path(root)
        self.use_camera = use_camera
        self.events = []
        self.ep_dir = None
        self.frame_n = 0
        self._bridge = None
        self._img_sub = None
        if use_camera:
            import rclpy
            from rclpy.node import Node  # noqa: F401
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
            rclpy.init()
            self._rclpy = rclpy
            self._node = rclpy.create_node('demo_recorder')
            self._bridge = CvBridge()
            self._img_msg_type = Image
            self._latest = None
            self._img_sub = self._node.create_subscription(
                Image, '/camera/camera/color/image_raw', self._on_img, 5)
            self._spin = threading.Thread(target=rclpy.spin, args=(self._node,),
                                          daemon=True)
            self._spin.start()

    def _on_img(self, msg):
        self._latest = msg

    def start_episode(self, idx, pick, place):
        self.ep_dir = self.root / f'ep_{idx:04d}'
        (self.ep_dir / 'frames').mkdir(parents=True, exist_ok=True)
        self.events = []
        self.frame_n = 0
        self.t0 = time.time()
        self.meta = {'episode': idx, 'pick_pose': pick, 'place_pose': place,
                     't_start': self.t0}

    def log(self, kind, **kw):
        self.events.append({'t': time.time() - self.t0, 'kind': kind, **kw})

    def snap(self):
        """Save the latest camera frame, tagged with episode-relative time."""
        if not self.use_camera or self._latest is None:
            return
        import cv2
        img = self._bridge.imgmsg_to_cv2(self._latest, desired_encoding='bgr8')
        cv2.imwrite(str(self.ep_dir / 'frames' / f'{self.frame_n:06d}.jpg'),
                    img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        self.log('frame', i=self.frame_n)
        self.frame_n += 1

    def end_episode(self, success):
        self.meta.update(t_end=time.time(), success=success, events=self.events)
        (self.ep_dir / 'meta.json').write_text(json.dumps(self.meta, indent=1))

    def close(self):
        if self.use_camera:
            self._rclpy.shutdown()


def sample_place(pick_xy):
    """Random reachable floor point ≥ MIN_MOVE_MM from the pick point."""
    for _ in range(200):
        r = random.uniform(R_MIN, R_MAX)
        b = random.uniform(-BEARING_MAX, BEARING_MAX)
        x, y = r * math.cos(b), r * math.sin(b)
        if math.hypot(x - pick_xy[0], y - pick_xy[1]) >= MIN_MOVE_MM:
            return x, y
    raise RuntimeError('could not sample a place point')


def run_episode(arm, rec, idx, pick_xy, place_xy, floor_z, snap_hz):
    """One scripted pick at pick_xy and place at place_xy. Frames are snapped
    during every dwell so the video covers the whole motion."""
    px, py = pick_xy
    qx, qy = place_xy
    rec.start_episode(idx, {'x': px, 'y': py, 'z': floor_z},
                      {'x': qx, 'y': qy, 'z': floor_z})

    def do(x, y, z, grip, dwell, label):
        rec.log('target', x=x, y=y, z=z, grip=grip, label=label)
        arm.move(x, y, z, grip, 0.0)
        end = time.time() + dwell
        while time.time() < end:
            rec.snap()
            fb = arm.feedback()
            if fb:
                rec.log('feedback', **{k: fb[k] for k in ('x', 'y', 'z')
                                       if k in fb})
            time.sleep(1.0 / snap_hz)

    try:
        do(px, py, floor_z + LIFT_MM, GRIP_OPEN, 2.5, 'above_pick')
        do(px, py, floor_z + JUST_ABOVE_MM, GRIP_OPEN, 1.5, 'just_above_pick')
        do(px, py, floor_z, GRIP_OPEN, 1.5, 'at_pick')
        do(px, py, floor_z, GRIP_CLOSED, 2.0, 'grasp')
        do(px, py, floor_z + LIFT_MM, GRIP_CLOSED, 2.0, 'lift')
        do(qx, qy, floor_z + LIFT_MM, GRIP_CLOSED, 2.5, 'above_place')
        do(qx, qy, floor_z + 15.0, GRIP_CLOSED, 2.0, 'at_place')
        do(qx, qy, floor_z + 15.0, GRIP_OPEN, 1.5, 'release')
        do(qx, qy, floor_z + LIFT_MM, GRIP_OPEN, 1.5, 'retreat')
        do(*HOME, GRIP_OPEN, 2.5, 'home')
        rec.end_episode(success=True)
        return True
    except Exception as e:
        print(f'episode {idx} failed: {e}')
        rec.end_episode(success=False)
        return False


def calibrate(arm):
    """Step the open gripper down over (250, 0) until ^C; prints each z so the
    user can read off the floor height for --floor-z."""
    print('Calibration: stepping down 10 mm every 2 s over (250, 0). '
          'Ctrl-C when the gripper tips touch the floor.')
    arm.move(250, 0, 150, GRIP_OPEN, 3.0)
    z = 150.0
    try:
        while True:
            z -= 10.0
            print(f'z = {z:.0f} mm')
            arm.move(250, 0, z, GRIP_OPEN, 2.0)
    except KeyboardInterrupt:
        print(f'\nfloor is at ~z = {z:.0f}. Use --floor-z {z:.0f}')
        arm.move(*HOME, GRIP_OPEN, 3.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--episodes', type=int, default=10)
    ap.add_argument('--out', default='/opt/vision_ws/src/../demos',
                    help='dataset root (bind-mounted so it lands in the repo)')
    ap.add_argument('--start-x', type=float, default=250.0)
    ap.add_argument('--start-y', type=float, default=0.0)
    ap.add_argument('--floor-z', type=float, default=None,
                    help='arm-frame z (mm) of a floor pick; find with --calibrate')
    ap.add_argument('--snap-hz', type=float, default=8.0)
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--calibrate', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--no-camera', action='store_true')
    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    arm = Arm(args.port, args.baud, args.dry_run)
    if args.calibrate:
        calibrate(arm)
        return

    if args.floor_z is None and not args.dry_run:
        ap.error('--floor-z is required (run --calibrate first)')
    floor_z = args.floor_z if args.floor_z is not None else -95.0

    rec = Recorder(args.out, use_camera=not args.no_camera)
    pick = (args.start_x, args.start_y)
    print(f'Object must be at arm-frame ({pick[0]:.0f}, {pick[1]:.0f}) mm. '
          f'Starting in 5 s...')
    time.sleep(5.0)

    ok = 0
    for i in range(args.episodes):
        place = sample_place(pick)
        print(f'ep {i}: pick ({pick[0]:.0f},{pick[1]:.0f}) -> '
              f'place ({place[0]:.0f},{place[1]:.0f})')
        if run_episode(arm, rec, i, pick, place, floor_z, args.snap_hz):
            ok += 1
            pick = place          # ground truth carries over
        else:
            print('stopping: a failed episode loses the object pose')
            break
    print(f'done: {ok}/{args.episodes} episodes at {rec.root}')
    rec.close()


if __name__ == '__main__':
    main()
