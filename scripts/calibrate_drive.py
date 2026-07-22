#!/usr/bin/env python3
"""Measure what a teleop drive burst ACTUALLY moves, per mode and duration.

Motivation (2026-07-22): open-loop bursts were wildly non-linear in practice --
0.35 s from rest moved nothing at all, while 0.55 s overshot by ~3x what the
commanded speed predicted. Two effects compound:

  * the I2C driver has a deadband (min_cmd 20) AND a slew limit (max_step 5
    per 20 Hz tick), so the first ~0.3 s of any burst produces little or no
    motion;
  * the commanded-to-actual velocity scale (cmd_per_mps) is not 1:1, so
    "0.10 m/s" is not 0.10 m/s.

Together those make short bursts useless and medium bursts overshoot. This
measures the real curve with wheel odometry so approach code can use numbers
instead of hope.

  python3 /scripts/calibrate_drive.py            # full sweep, forward
  python3 /scripts/calibrate_drive.py -1 0.5     # single reverse 0.5 s burst
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Cal(Node):
    def __init__(self):
        super().__init__('calibrate_drive')
        self.odom = None
        self.create_subscription(Odometry, '/wheel_odom', self._o, 10)
        self.pub = self.create_publisher(String, '/teleop/cmd', 10)
        self.seq = 0

    def _o(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)

    def spin(self, d):
        t0 = time.time()
        while time.time() - t0 < d:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_odom(self):
        t0 = time.time()
        while self.odom is None and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.odom

    def burst(self, vx, seconds, mode):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.seq += 1
            m = String()
            m.data = json.dumps({
                'seq': self.seq, 'mode': mode,
                'arm': {'x': 0, 'y': 0, 'z': 0, 'grip': 0},
                'base': {'vx': vx, 'vy': 0, 'wz': 0},
            })
            self.pub.publish(m)
            self.spin(0.05)
        self.spin(1.5)          # let the deadman stop it and the base settle

    def measure(self, vx, seconds, mode):
        a = self.wait_odom()
        if a is None:
            print('  no /wheel_odom'); return None
        self.burst(vx, seconds, mode)
        self.spin(0.5)
        b = self.odom
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        print(f'  {mode:9s} vx={vx:+.0f} {seconds:.2f}s -> {d*1000:6.1f} mm '
              f'({d/seconds:.3f} m/s avg over the burst)')
        return d


def main():
    rclpy.init()
    c = Cal()
    c.spin(1.0)

    if len(sys.argv) >= 3:
        vx = float(sys.argv[1]); secs = float(sys.argv[2])
        mode = sys.argv[3] if len(sys.argv) > 3 else 'precision'
        c.measure(vx, secs, mode)
    else:
        print('drive calibration sweep (alternating direction to stay put):')
        for mode in ('precision', 'normal'):
            for secs in (0.3, 0.5, 0.8, 1.2):
                c.measure(+1, secs, mode)
                c.measure(-1, secs, mode)
    c.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
