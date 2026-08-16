#!/usr/bin/env python3
"""Wrist camera publisher: the arm-mounted global-shutter USB cam (32e4:0234)
-> /wrist_cam/image_raw/compressed (jpeg). Runs standalone in the container:

  nohup python3 /scripts/wrist_cam.py >/tmp/wrist_cam.log 2>&1 &

Publishes compressed-only: the consumers are the SUDS console (rosbridge)
and episode recorders, both of which want jpeg anyway.
"""
import sys
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

DEV = 0
HZ = 15.0
JPEG_Q = 80


class WristCam(Node):
    def __init__(self):
        super().__init__('wrist_cam')
        self.pub = self.create_publisher(CompressedImage,
                                         '/wrist_cam/image_raw/compressed', 2)
        self.cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'/dev/video{DEV} did not open')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.n = 0
        self.last_sig = None
        self.frozen_n = 0
        # Dedicated reader thread draining the driver at its NATIVE rate:
        # reading from a ROS timer at 15 Hz let V4L2 queue frames
        # (CAP_PROP_BUFFERSIZE is ignored by this backend) and the published
        # image lagged the real view by a growing number of seconds --
        # which silently poisoned the wrist-refine loop (2026-08-02).
        self._latest = None
        self._lk = threading.Lock()
        self._alive = True
        threading.Thread(target=self._reader, daemon=True).start()
        self.create_timer(1.0 / HZ, self.tick)
        self.get_logger().info('wrist cam up: /wrist_cam/image_raw/compressed')

    def _reader(self):
        while self._alive:
            try:
                ok, frame = self.cap.read()
            except Exception:      # cap swapped mid-read by reopen()
                ok = False
            if ok:
                with self._lk:
                    self._latest = frame
            else:
                time.sleep(0.05)

    def reopen(self):
        self.cap.release()
        time.sleep(1.0)
        self.cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            # let the launch respawn take it from here
            self.get_logger().error('reopen failed -- exiting for respawn')
            sys.exit(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def tick(self):
        with self._lk:
            frame = self._latest
        if frame is None:
            return
        # freeze watchdog: a live sensor never yields bit-identical frames
        # (seen 2026-08-02: driver served one stuck frame at full rate and
        # silently poisoned the wrist-refine loop)
        sig = frame[::16, ::16].tobytes()
        if sig == self.last_sig:
            self.frozen_n += 1
            if self.frozen_n >= int(3 * HZ):
                self.get_logger().warn('stream frozen ~3s -- reopening device')
                self.frozen_n = 0
                self.last_sig = None
                self.reopen()
                return
        else:
            self.frozen_n = 0
        self.last_sig = sig
        m = CompressedImage()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'wrist_cam'
        m.format = 'jpeg'
        m.data = cv2.imencode('.jpg', frame,
                              [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])[1].tobytes()
        self.pub.publish(m)
        self.n += 1


def main():
    rclpy.init()
    n = WristCam()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.cap.release()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
