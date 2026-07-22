#!/usr/bin/env python3
"""Headless driver for the teleop server — same wire protocol as the web UI.

Publishes /teleop/cmd exactly as ui/index.html does, so the whole chain
(intent -> ramp -> T:123 jog / cmd_vel) can be exercised and timed without a
browser in the loop. Also used to verify the deadman: stop publishing and the
robot must stop on its own.

  ros2 run  -- not registered; run directly inside the container:
    python3 /scripts/teleop_probe.py arm  z -1  1.5      # arm down 1.5 s
    python3 /scripts/teleop_probe.py base vx  1  1.0     # drive forward 1 s
    python3 /scripts/teleop_probe.py grip -1 2.0         # open gripper 2 s
    python3 /scripts/teleop_probe.py watch 5             # just print state
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Probe(Node):
    def __init__(self):
        super().__init__('teleop_probe')
        self.pub = self.create_publisher(String, '/teleop/cmd', 10)
        self.act = self.create_publisher(String, '/teleop/action', 10)
        self.state = None
        self.create_subscription(String, '/teleop/state', self._st, 10)
        self.seq = 0

    def _st(self, m):
        try:
            self.state = json.loads(m.data)
        except Exception:
            pass

    def send(self, arm=None, base=None, mode='normal'):
        self.seq += 1
        msg = String()
        msg.data = json.dumps({
            'seq': self.seq, 'mode': mode,
            'arm': arm or {'x': 0, 'y': 0, 'z': 0, 'grip': 0},
            'base': base or {'vx': 0, 'vy': 0, 'wz': 0},
        })
        self.pub.publish(msg)

    def action(self, a):
        m = String(); m.data = a
        self.act.publish(m)

    def spin(self, dur):
        t0 = time.time()
        while time.time() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self):
        s = self.state or {}
        a = s.get('arm') or {}
        return a.get('x'), a.get('y'), a.get('z'), a.get('t')


def main():
    rclpy.init()
    p = Probe()
    p.spin(1.0)                       # let /teleop/state arrive

    if len(sys.argv) < 2:
        print(__doc__)
        return

    what = sys.argv[1]

    if what == 'watch':
        dur = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
        t0 = time.time()
        while time.time() - t0 < dur:
            p.spin(0.5)
            s = p.state or {}
            print(f'  link={s.get("link")} age={s.get("fb_age")} '
                  f'pose={p.pose()} r={s.get("arm_r")} blocked={s.get("blocked")}')
        return

    if what == 'action':
        p.action(sys.argv[2]); p.spin(1.0); print('sent'); return

    mode = 'normal'
    if what == 'arm':
        ax, val, dur = sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
        arm = {'x': 0, 'y': 0, 'z': 0, 'grip': 0}; arm[ax] = val
        base = None
    elif what == 'grip':
        val, dur = float(sys.argv[2]), float(sys.argv[3])
        arm = {'x': 0, 'y': 0, 'z': 0, 'grip': val}; base = None
    elif what == 'base':
        ax, val, dur = sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
        base = {'vx': 0, 'vy': 0, 'wz': 0}; base[ax] = val
        arm = None
    else:
        print('unknown mode'); return

    print('before:', p.pose())
    t0 = time.time()
    while time.time() - t0 < dur:      # publish at 20 Hz like the UI
        p.send(arm=arm, base=base, mode=mode)
        p.spin(0.05)
    print('released — deadman should stop motion now')
    p.spin(1.5)
    print('after :', p.pose())
    s = p.state or {}
    print('state :', 'link', s.get('link'), 'blocked', s.get('blocked'),
          'watchdog', s.get('watchdog'))

    p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
