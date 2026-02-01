import subprocess
import time
import argparse
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from collections import deque
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
"""
Lightweight SLAM timing validator

Checks per-imager frame rate, inter-frame jitter, and inter-imager timestamp offsets
against the project's specification.

Requirements tested:
 - Minimum imager framerate: 30 Hz
 - Maximum permissible jitter: +/- 2 milliseconds (we check max deviation from mean)
 - Maximum inter-imager offset: +/- 100 microseconds

Usage:
  python3 tests/test_slam_mode_startup.py [--bag /path/to/bag]

This script will optionally play a bag (if provided) and collect timestamps for
`/visual_slam/image_0` and `/visual_slam/image_1` for a few seconds, then print
results and return non-zero on failure.
"""

import argparse
import os
import sys
import time
import subprocess
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


TARGET_FPS = 30.0
MAX_JITTER_MS = 2.0
MAX_INTERIMAGER_OFFSET_US = 100.0
COLLECT_DURATION = 8.0


class SlamTimingTester(Node):
    def __init__(self):
        super().__init__('slam_timing_tester')
        self.ts0 = []
        self.ts1 = []
        self.cam_ts0 = []
        self.cam_ts1 = []
        self.intervals0 = []
        self.intervals1 = []
        self.cam_intervals0 = []
        self.cam_intervals1 = []
        self.cross_diffs = []
        self._last0 = None
        self._last1 = None
        # keep small window of recent timestamps for matching
        self.recent0 = deque(maxlen=1000)
        self.recent1 = deque(maxlen=1000)
        # Visual SLAM inputs
        self.sub0 = self.create_subscription(Image, '/visual_slam/image_0', self.cb0, 10)
        self.sub1 = self.create_subscription(Image, '/visual_slam/image_1', self.cb1, 10)
        # Raw camera topics (diagnostic)
        self.cam_sub0 = self.create_subscription(Image, '/camera/infra1/image_rect_raw', self.cam_cb0, 10)
        self.cam_sub1 = self.create_subscription(Image, '/camera/infra2/image_rect_raw', self.cam_cb1, 10)

    @staticmethod
    def _to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def cb0(self, msg: Image):
        t = self._to_seconds(msg.header.stamp)
        if self._last0 is not None:
            self.intervals0.append(t - self._last0)
        self._last0 = t
        self.ts0.append(t)
        self.recent0.append(t)
        # try to find closest in recent1
        if self.recent1:
            # linear scan over small deque is fine here
            nearest = min(self.recent1, key=lambda x: abs(x - t))
            self.cross_diffs.append(t - nearest)

    def cb1(self, msg: Image):
        t = self._to_seconds(msg.header.stamp)
        if self._last1 is not None:
            self.intervals1.append(t - self._last1)
        self._last1 = t
        self.ts1.append(t)
        self.recent1.append(t)
        if self.recent0:
            nearest = min(self.recent0, key=lambda x: abs(x - t))
            self.cross_diffs.append(nearest - t)

    # Camera diagnostics callbacks
    def cam_cb0(self, msg: Image):
        t = self._to_seconds(msg.header.stamp)
        if len(self.cam_ts0) >= 1:
            self.cam_intervals0.append(t - self.cam_ts0[-1])
        self.cam_ts0.append(t)

    def cam_cb1(self, msg: Image):
        t = self._to_seconds(msg.header.stamp)
        if len(self.cam_ts1) >= 1:
            self.cam_intervals1.append(t - self.cam_ts1[-1])
        self.cam_ts1.append(t)


def play_bag_if_requested(bag_path):
    if not bag_path:
        return None
    if not os.path.exists(bag_path):
        print(f'Bag not found: {bag_path}')
        return None
    print(f'Playing bag: {bag_path}')
    p = subprocess.Popen(['ros2', 'bag', 'play', bag_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # give bag a moment to start
    time.sleep(1.0)
    return p


def analyze_results(node: SlamTimingTester, duration):
    results = {}
    # image_0
    if len(node.intervals0) >= 1:
        mean_dt0 = statistics.mean(node.intervals0)
        std_dt0 = statistics.pstdev(node.intervals0)
        max_dev0 = max(abs(dt - mean_dt0) for dt in node.intervals0) * 1000.0
        fps0 = 1.0 / mean_dt0 if mean_dt0 > 0 else 0.0
    else:
        mean_dt0 = std_dt0 = max_dev0 = fps0 = 0.0

    # image_1
    if len(node.intervals1) >= 1:
        mean_dt1 = statistics.mean(node.intervals1)
        std_dt1 = statistics.pstdev(node.intervals1)
        max_dev1 = max(abs(dt - mean_dt1) for dt in node.intervals1) * 1000.0
        fps1 = 1.0 / mean_dt1 if mean_dt1 > 0 else 0.0
    else:
        mean_dt1 = std_dt1 = max_dev1 = fps1 = 0.0

    # camera raw rates
    if len(node.cam_intervals0) >= 1:
        mean_cam_dt0 = statistics.mean(node.cam_intervals0)
        fps_cam0 = 1.0 / mean_cam_dt0 if mean_cam_dt0 > 0 else 0.0
    else:
        fps_cam0 = 0.0

    if len(node.cam_intervals1) >= 1:
        mean_cam_dt1 = statistics.mean(node.cam_intervals1)
        fps_cam1 = 1.0 / mean_cam_dt1 if mean_cam_dt1 > 0 else 0.0
    else:
        fps_cam1 = 0.0

    # cross-imager offsets
    offsets = [abs(d) * 1e6 for d in node.cross_diffs]  # convert to microseconds
    mean_offset_us = statistics.mean(offsets) if offsets else 0.0
    max_offset_us = max(offsets) if offsets else 0.0

    results['fps0'] = fps0
    results['fps1'] = fps1
    results['jitter_std_ms_0'] = std_dt0 * 1000.0
    results['jitter_std_ms_1'] = std_dt1 * 1000.0
    results['max_dev_ms_0'] = max_dev0
    results['max_dev_ms_1'] = max_dev1
    results['mean_offset_us'] = mean_offset_us
    results['max_offset_us'] = max_offset_us
    results['samples0'] = len(node.ts0)
    results['samples1'] = len(node.ts1)

    print('\n=== SLAM TIMING SUMMARY ===')
    print(f'image_0 samples: {results["samples0"]}  mean fps: {fps0:.2f}  jitter(std ms): {results["jitter_std_ms_0"]:.3f}  max dev(ms): {results["max_dev_ms_0"]:.3f}')
    print(f'image_1 samples: {results["samples1"]}  mean fps: {fps1:.2f}  jitter(std ms): {results["jitter_std_ms_1"]:.3f}  max dev(ms): {results["max_dev_ms_1"]:.3f}')
    print(f'camera infra1 fps: {fps_cam0:.2f}  camera infra2 fps: {fps_cam1:.2f}')
    print(f'inter-imager mean offset: {mean_offset_us:.1f} us   max offset: {max_offset_us:.1f} us')

    # Evaluate against requirements
    ok = True
    if fps0 < TARGET_FPS or fps1 < TARGET_FPS:
        print(f'FAIL: FPS below target {TARGET_FPS}Hz (image_0 {fps0:.2f}, image_1 {fps1:.2f})')
        ok = False
    if results['max_dev_ms_0'] > MAX_JITTER_MS or results['max_dev_ms_1'] > MAX_JITTER_MS:
        print(f'FAIL: Jitter exceeds +/-{MAX_JITTER_MS} ms (max dev ms: img0 {results["max_dev_ms_0"]:.3f}, img1 {results["max_dev_ms_1"]:.3f})')
        ok = False
    if results['max_offset_us'] > MAX_INTERIMAGER_OFFSET_US:
        print(f'FAIL: Inter-imager offset exceeds {MAX_INTERIMAGER_OFFSET_US} us (max {results["max_offset_us"]:.1f} us)')
        ok = False

    if ok:
        print('\nRESULT: PASS — timing requirements met')
        return 0
    else:
        print('\nRESULT: FAIL — timing requirements NOT met')
        return 2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', help='Path to ros2 bag to play during the test')
    parser.add_argument('--duration', type=float, default=COLLECT_DURATION, help='Seconds to collect timestamps')
    args = parser.parse_args()

    bag_proc = play_bag_if_requested(args.bag)

    print('Waiting 2s for system to stabilize...')
    time.sleep(2.0)

    rclpy.init()
    node = SlamTimingTester()
    start = time.time()
    try:
        while time.time() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

    rc = analyze_results(node, args.duration)

    # cleanup bag
    if bag_proc:
        try:
            bag_proc.terminate()
            bag_proc.wait(timeout=3)
        except Exception:
            try:
                bag_proc.kill()
            except Exception:
                pass

    sys.exit(rc)


if __name__ == '__main__':
    main()