#!/usr/bin/env python3
"""Persistent flywheel overlay relay.

The pick scripts publish /flywheel/{head,wrist}/overlay + /flywheel/meta only
while they run; their 1 Hz republish dies with the process, so the operator
opening flywheel.html between runs saw empty panels (operator report
2026-08-02). This node caches the last message per topic and republishes at
1 Hz forever, so the last inference stays inspectable indefinitely.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

TOPICS = ['/flywheel/head/overlay', '/flywheel/wrist/overlay']


class FlywheelRelay(Node):
    def __init__(self):
        super().__init__('flywheel_relay')
        self.last = {}
        self.last_meta = None
        self.pubs = {}
        for t in TOPICS:
            self.pubs[t] = self.create_publisher(CompressedImage, t, 2)
            self.create_subscription(CompressedImage, t,
                                     lambda m, t=t: self._img(t, m), 2)
        self.meta_pub = self.create_publisher(String, '/flywheel/meta', 2)
        self.create_subscription(String, '/flywheel/meta', self._meta, 2)
        self.create_timer(1.0, self.tick)
        self.get_logger().info('flywheel relay up')

    def _img(self, topic, m):
        # ignore our own republished copies: only fresh script publishes
        # differ from the cache
        if self.last.get(topic) is not None and \
                bytes(m.data) == bytes(self.last[topic].data):
            return
        self.last[topic] = m

    def _meta(self, m):
        if self.last_meta is not None and m.data == self.last_meta.data:
            return
        self.last_meta = m

    def tick(self):
        for t, m in self.last.items():
            if m is not None:
                self.pubs[t].publish(m)
        if self.last_meta is not None:
            self.meta_pub.publish(self.last_meta)


def main():
    rclpy.init()
    n = FlywheelRelay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
