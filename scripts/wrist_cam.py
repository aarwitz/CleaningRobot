#!/usr/bin/env python3
"""Wrist camera publisher: the arm-mounted global-shutter USB cam (32e4:0234)
-> /wrist_cam/image_raw/compressed (jpeg). Runs standalone in the container:

  nohup python3 /scripts/wrist_cam.py >/tmp/wrist_cam.log 2>&1 &

Publishes compressed-only: the consumers are the SUDS console (rosbridge)
and episode recorders, both of which want jpeg anyway.
"""
import sys
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
        self.create_timer(1.0 / HZ, self.tick)
        self.get_logger().info('wrist cam up: /wrist_cam/image_raw/compressed')

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('frame grab failed')
            return
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
