#!/usr/bin/env python3
"""Depth-closed-loop forward approach, driven through the teleop server.

Drives short bursts and RE-MEASURES between each one, so stopping distance is
measured rather than integrated from wheel timing (which drifts badly at these
speeds). Uses the wide flat obstacle face as the range metric instead of
tracking a small object blob -- a large surface gives a stable p20 depth even
as the view changes on approach.

Hard guards: bounded iterations, an absolute minimum standoff, and an abort if
depth ever goes invalid. Publishing stops on exit, so the teleop deadman halts
the base even if this script dies mid-burst.

  python3 /scripts/approach_target.py 0.30        # stop 30 cm from the face
"""
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

ROI = (260, 140, 380, 260)      # central corridor: x0,y0,x1,y1
ABS_MIN = 0.26                  # never command a burst below this
BURST_S = 0.8
NET_MPS = 0.21                  # measured net advance per second of burst
MAX_ITERS = 12


class Approach(Node):
    def __init__(self):
        super().__init__('approach_target')
        self.depth = None
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw',
                                 self._d, 10)
        self.pub = self.create_publisher(String, '/teleop/cmd', 10)
        self.seq = 0

    def _d(self, m):
        self.depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)

    def spin(self, dur):
        t0 = time.time()
        while time.time() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.02)

    def range_now(self, settle=0.6):
        """p20 of the central ROI = distance to the nearest broad surface."""
        self.depth = None
        t0 = time.time()
        while self.depth is None and time.time() - t0 < 4.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.depth is None:
            return None
        self.spin(settle)
        x0, y0, x1, y1 = ROI
        r = self.depth[y0:y1, x0:x1].astype(np.float32) / 1000.0
        v = r[(r > 0.10) & (r < 4.0)]
        if v.size < 200:
            return None
        return float(np.percentile(v, 20))

    def drive(self, seconds, vx=1.0):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.seq += 1
            m = String()
            m.data = json.dumps({
                'seq': self.seq, 'mode': 'precision',
                'arm': {'x': 0, 'y': 0, 'z': 0, 'grip': 0},
                'base': {'vx': vx, 'vy': 0, 'wz': 0},
            })
            self.pub.publish(m)
            self.spin(0.05)
        # Stop publishing: the teleop deadman brings the base to rest.
        self.spin(1.2)


def main():
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
    rclpy.init()
    a = Approach()
    a.spin(1.0)

    d = a.range_now()
    if d is None:
        print('no valid depth — aborting'); return
    print(f'start range {d:.3f} m, target {target:.3f} m')

    for i in range(MAX_ITERS):
        d = a.range_now()
        if d is None:
            print('lost depth — STOP'); break
        if d <= target:
            print(f'  reached {d:.3f} m'); break
        remain = d - target
        # Cap each burst so we never command past the target. MEASURED net
        # rate is ~0.21 m/s in 'precision' mode -- roughly double the nominal
        # 0.10 m/s, because the motor feedforward (cmd_per_mps) runs hot. Using
        # the nominal figure here overshoots every burst.
        burst = min(BURST_S, max(0.25, remain / NET_MPS))
        if d - (burst * NET_MPS) < ABS_MIN:
            burst = max(0.0, (d - ABS_MIN) / NET_MPS)
            if burst < 0.2:
                print(f'  at safety floor ({d:.3f} m) — stopping'); break
        print(f'  iter {i}: range {d:.3f}  remain {remain:.3f}  burst {burst:.2f}s')
        a.drive(burst)

    d = a.range_now()
    print(f'FINAL range {d:.3f} m' if d else 'FINAL range unknown')
    a.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
