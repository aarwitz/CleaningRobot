#!/usr/bin/env python3
"""Record OPERATOR-DRIVEN episodes into the standard demos/picks format.

The missing flywheel input (flagged 2026-08-15, built 2026-08-19): the
scripted teacher demonstrates ONE sweep motion; two honest 0/5 pi0 evals
say that data does not teach grasping. Human teleop demos carry the
contact-rich close/adjust/recover dynamics the policy actually needs --
and the operator can drive them REMOTELY from the browser console.

Pure recorder: subscribes only; publishes nothing but flywheel overlays.
It NEVER commands motion (not subject to the robot-wrapper motion rule,
but invoked via `robot teleop-record` anyway for the audit trail).

Episode control comes from the console's REC buttons on /teleop/action:
  rec:start        begin an episode (next free ep_NNNN)
  rec:stop:held    end episode, success=True  (operator watched it happen)
  rec:stop:miss    end episode, success=False
  rec:abort        end episode and DELETE it (false start)
Labels are operator-asserted -- the human is watching the claw; that is
the gold standard the detector-verify never reached.

  python3 /scripts/teleop_record.py --object sock --out /demos/picks
"""
import argparse
import json
import shutil
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

RATE_HZ = 10.0


class TeleopRecorder(Node):
    def __init__(self, args):
        super().__init__('teleop_recorder')
        self.a = args
        self.head = self.wrist = None
        self.fb = {}
        self.ep = None          # active episode state dict or None
        self.n_done = 0
        self.create_subscription(CompressedImage,
                                 '/camera/color/image_raw/compressed',
                                 self._h, 2)
        self.create_subscription(CompressedImage,
                                 '/wrist_cam/image_raw/compressed',
                                 self._w, 2)
        self.create_subscription(String, '/teleop/state', self._s, 10)
        self.create_subscription(String, '/teleop/action', self._act, 10)
        self.fw = self.create_publisher(CompressedImage,
                                        '/flywheel/head/overlay', 2)
        # machine-readable state for the console's recorder panel: the
        # buttons were stateless and the operator could not tell whether
        # recording was live or what got labeled (operator, 2026-08-19)
        self.state_pub = self.create_publisher(String,
                                               '/teleop/record_state', 2)
        self.last_result = None      # {'ep': int, 'label': str}
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.create_timer(1.0, self._hud)
        self.create_timer(0.5, self._pub_state)
        print(f'[teleop-record] ready; waiting for rec:start '
              f'(object={args.object})', flush=True)

    def _h(self, m):
        self.head = m

    def _w(self, m):
        self.wrist = m

    def _s(self, m):
        try:
            self.fb = json.loads(m.data)
        except ValueError:
            pass

    # ── episode control from the console ─────────────────────────────────
    def _act(self, m):
        act = m.data.strip()
        if act == 'rec:start' and self.ep is None:
            root = Path(self.a.out)
            idx = 1 + max([int(p.name[3:]) for p in root.glob('ep_*')] or [-1])
            d = root / f'ep_{idx:04d}'
            (d / 'frames').mkdir(parents=True)
            (d / 'frames_wrist').mkdir()
            self.ep = {'dir': d, 'idx': idx, 'i': 0, 't0': time.time(),
                       'traj': open(d / 'traj.jsonl', 'w')}
            print(f'[teleop-record] ● REC ep_{idx:04d}', flush=True)
        elif act.startswith('rec:stop:') and self.ep is not None:
            held = act.endswith('held')
            self._finish(held)
        elif act == 'rec:abort' and self.ep is not None:
            self.ep['traj'].close()
            shutil.rmtree(self.ep['dir'])
            self.last_result = {'ep': self.ep['idx'], 'label': 'ABORTED',
                                'frames': 0}
            print(f'[teleop-record] ✗ aborted ep_{self.ep["idx"]:04d} '
                  '(deleted)', flush=True)
            self.ep = None

    def _pub_state(self):
        st = {'running': True, 'recording': self.ep is not None,
              'object': self.a.object, 'session_count': self.n_done}
        if self.ep is not None:
            st['ep'] = self.ep['idx']
            st['frames'] = self.ep['i']
            st['secs'] = round(time.time() - self.ep['t0'], 1)
        if self.last_result:
            st['last'] = self.last_result
        self.state_pub.publish(String(data=json.dumps(st)))

    def _finish(self, held):
        e = self.ep
        e['traj'].close()
        meta = {'prompt': f'pick up the {self.a.object}',
                'episode': e['idx'], 'object': self.a.object,
                'strategy': 'teleop', 'teleop': True,
                't_start': e['t0'], 't_end': time.time(),
                'held': held, 'success': held, 'n_frames': e['i'],
                'note': 'operator-driven episode; label operator-asserted '
                        'live (watching the claw)'}
        (e['dir'] / 'meta.json').write_text(json.dumps(meta, indent=1))
        self.n_done += 1
        self.last_result = {'ep': e['idx'],
                            'label': 'HELD' if held else 'MISS',
                            'frames': e['i']}
        print(f'[teleop-record] ■ ep_{e["idx"]:04d}: {e["i"]} frames, '
              f'{"HELD" if held else "miss"} '
              f'({self.n_done} this session)', flush=True)
        self.ep = None

    # ── recording tick ───────────────────────────────────────────────────
    def _tick(self):
        if self.ep is None or self.head is None or self.wrist is None:
            return
        arm = (self.fb.get('arm') or {})
        if arm.get('x') is None:
            return
        e = self.ep
        name = f'{e["i"]:04d}.jpg'
        (e['dir'] / 'frames' / name).write_bytes(bytes(self.head.data))
        (e['dir'] / 'frames_wrist' / name).write_bytes(bytes(self.wrist.data))
        e['traj'].write(json.dumps(
            {'i': e['i'], 'ts': time.time(), 'frame': name,
             'phase': 'teleop',
             'pose': {k: arm.get(k) for k in ('x', 'y', 'z', 't')},
             'joints': {k: arm.get(k) for k in ('b', 's', 'e')},
             'torque': {k: arm.get(k) for k in
                        ('torB', 'torS', 'torE', 'torH')}}) + '\n')
        e['i'] += 1

    # ── operator HUD via the flywheel head panel ─────────────────────────
    def _hud(self):
        if self.head is None:
            return
        img = cv2.imdecode(np.frombuffer(self.head.data, np.uint8),
                           cv2.IMREAD_COLOR)
        if self.ep is not None:
            txt = f'REC ep_{self.ep["idx"]:04d}  frames={self.ep["i"]}'
            color = (60, 60, 235)
            cv2.circle(img, (615, 22), 9, color, -1)
        else:
            txt = (f'teleop-record armed - {self.n_done} eps this session; '
                   'press REC')
            color = (60, 220, 255)
        cv2.rectangle(img, (0, 456), (640, 480), (20, 20, 20), -1)
        cv2.putText(img, txt + time.strftime('  @%H:%M:%S'), (8, 474),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        m = CompressedImage()
        m.format = 'jpeg'
        m.data = cv2.imencode('.jpg', img,
                              [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        self.fw.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--object', default='sock')
    ap.add_argument('--out', default='/demos/picks')
    ap.add_argument('--max-min', type=float, default=180.0)
    a = ap.parse_args()
    rclpy.init()
    n = TeleopRecorder(a)
    t_end = time.time() + a.max_min * 60.0
    try:
        while time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if n.ep is not None:
            n._finish(False)      # never lose frames on teardown
        print(f'[teleop-record] session over: {n.n_done} episodes',
              flush=True)


if __name__ == '__main__':
    main()
