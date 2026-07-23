#!/usr/bin/env python3
"""Save one color frame (and optionally the aligned depth at a pixel) to disk.

Sanity-check tool: confirms what the policy camera actually sees before a run.
  python3 /scripts/grab_frame.py /tmp/f.png
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Grab(Node):
    def __init__(self):
        super().__init__('grab_frame')
        self.br = CvBridge()
        self.img = None
        self.create_subscription(Image, '/camera/color/image_raw', self._c, 10)

    def _c(self, m):
        self.img = self.br.imgmsg_to_cv2(m, 'bgr8')


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/frame.png'
    rclpy.init()
    n = Grab()
    for _ in range(200):
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.img is not None:
            break
    if n.img is None:
        print('no frame')
        return
    cv2.imwrite(out, n.img)
    print(f'saved {out} {n.img.shape}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
