"""Near-range depth self-filter (INTERIM).

Zeros out depth pixels closer than `min_range_m` and republishes the image with
its header UNCHANGED (same stamp + frame_id), so nvblox's TF-based placement —
which relies on the depth-module clock — keeps working.

Why this exists: the robot's arm cantilevers forward into the camera's field of
view and nvblox (which integrates the raw depth image and does NOT self-filter
from a URDF) reconstructs it as a lethal obstacle ~0.3-0.7 m dead ahead, blocking
every forward Nav2 goal. The arm is the only thing that close, so a near-range
cutoff removes it while leaving real obstacles (which are farther) intact.

LIMITATION: this is a blunt, static stopgap. It also discards any *real* obstacle
closer than min_range_m, and it only makes sense while the arm holds a fixed pose.
The real fix is a URDF-aware self-filter that masks exactly the arm's pixels using
joint_states + TF, so the arm can move while driving. Replace this node with that.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthCutoffNode(Node):
    def __init__(self):
        super().__init__('depth_self_filter_node')
        self.declare_parameter('min_range_m', 0.75)
        self.declare_parameter('input_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('output_topic', '/camera/depth/image_self_filtered')

        self.min_range_m = self.get_parameter('min_range_m').value
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # Allow live tuning of the cutoff (ros2 param set ... min_range_m) so the
        # threshold can be dialed in without restarting the whole stack.
        self.add_on_set_parameters_callback(self._on_set_params)

        # Match the camera's sensor-data QoS (best-effort) so we actually receive
        # frames and downstream nvblox can subscribe the same way.
        self.pub = self.create_publisher(Image, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            Image, in_topic, self._on_depth, qos_profile_sensor_data)
        self._warned_encoding = False
        self.get_logger().info(
            f'depth self-filter (INTERIM near-range cutoff) up: '
            f'{in_topic} -> {out_topic}, zeroing depth < {self.min_range_m:.2f} m')

    def _on_set_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'min_range_m':
                self.min_range_m = p.value
                self.get_logger().info(f'min_range_m -> {self.min_range_m:.2f} m')
        return SetParametersResult(successful=True)

    def _on_depth(self, msg: Image):
        # RealSense raw depth is 16UC1 in millimetres. Only handle that; pass
        # anything else through untouched (and warn once) rather than corrupt it.
        if msg.encoding not in ('16UC1', 'mono16'):
            if not self._warned_encoding:
                self.get_logger().warn(
                    f'Unexpected depth encoding "{msg.encoding}"; passing through.')
                self._warned_encoding = True
            self.pub.publish(msg)
            return

        cutoff_mm = int(self.min_range_m * 1000.0)
        depth = np.frombuffer(msg.data, dtype=np.uint16).copy()
        # 0 is RealSense's "invalid/no-data" sentinel; setting near pixels to 0
        # makes nvblox treat them as missing rather than as a close surface.
        depth[(depth > 0) & (depth < cutoff_mm)] = 0

        out = Image()
        out.header = msg.header            # preserve stamp + frame_id exactly
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = depth.tobytes()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthCutoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
