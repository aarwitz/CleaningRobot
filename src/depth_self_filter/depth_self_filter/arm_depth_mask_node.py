"""Shape-based arm self-filter for depth images.

Removes the robot's own RoArm from the depth image BEFORE nvblox integrates it,
by projecting a collision-sphere model of the arm (placed via the live arm-link
TFs) into the camera and blanking the matching pixels. Unlike a blunt near-range
cutoff, this masks ONLY the arm's silhouette and only at the arm's depth (a
per-pixel z-test), so a real obstacle at a different range in the same pixel
direction is preserved — it does not blind the robot.

Pipeline position:  /camera/depth/image_rect_raw (raw) -> THIS NODE -> nvblox.
The output header (stamp + frame_id) is preserved so nvblox's depth-module-clock
TF placement still holds.

Requirements to work correctly (calibrated on hardware):
  * robot.urdf.xacro arm_mount_* transform must match the real arm base mount.
  * arm_bridge must publish /joint_states so the arm-link TFs track the real pose
    (until then the arm sits at the URDF zero pose and the mask will be wrong).
  * SPHERES below must cover the physical arm; verify by overlaying the projected
    discs on the depth image and padding radii until the arm is fully covered.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException

# Collision-sphere model of the arm, as (link_frame, x, y, z, radius_m), with the
# offset in the LINK's own frame. Spheres are strung along each link using the
# kinematic lengths from the RoArm-M2 URDF (upper arm link2 ~0.237 m along +x;
# forearm link3 ~0.216 m along -y). Radii are padded to cover servos/brackets.
# CALIBRATE: overlay projected discs on the live depth image and adjust.
DEFAULT_SPHERES = [
    # base / shoulder region
    ('arm_link1', 0.0, 0.0, 0.0, 0.055),
    # upper arm (link2 extends along +x toward the elbow at x=0.2368)
    ('arm_link2', 0.04, 0.0, 0.0, 0.050),
    ('arm_link2', 0.10, 0.0, 0.0, 0.045),
    ('arm_link2', 0.16, 0.0, 0.0, 0.045),
    ('arm_link2', 0.22, 0.0, 0.0, 0.045),
    # forearm (link3 extends along -y toward the gripper at y=-0.216)
    ('arm_link3', 0.0, -0.05, 0.0, 0.045),
    ('arm_link3', 0.0, -0.11, 0.0, 0.040),
    ('arm_link3', 0.0, -0.17, 0.0, 0.040),
    # gripper
    ('arm_gripper_link', 0.0, 0.0, 0.0, 0.045),
]


def _quat_to_R(x, y, z, w):
    """Rotation matrix from a quaternion (no external deps)."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


class ArmDepthMaskNode(Node):
    def __init__(self):
        super().__init__('arm_depth_mask_node')
        self.declare_parameter('input_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('output_topic', '/camera/depth/image_self_filtered')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        # Half-thickness (m) of the depth band around each sphere that gets masked.
        # A pixel is blanked only if its measured depth is within
        # [sphere_depth - radius - z_tol, sphere_depth + radius + z_tol]; this is
        # what keeps a closer/farther real object in the same direction visible.
        self.declare_parameter('z_tol_m', 0.06)
        self.declare_parameter('radius_pad_px', 1.0)   # extra disc margin (px)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.z_tol_m = self.get_parameter('z_tol_m').value
        self.radius_pad_px = self.get_parameter('radius_pad_px').value

        self.spheres = DEFAULT_SPHERES
        self.K = None            # (fx, fy, cx, cy)
        self._uu = None          # cached pixel-coord grids
        self._vv = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Image, out_topic, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, in_topic, self._on_depth, qos_profile_sensor_data)
        self._warned = set()
        self.get_logger().info(
            f'arm shape-mask up: {in_topic} -> {out_topic} '
            f'({len(self.spheres)} spheres, z_tol={self.z_tol_m} m)')

    def _on_info(self, msg: CameraInfo):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _sphere_centers_in_camera(self, cam_frame, stamp):
        """Return list of (Xc, Yc, Zc, radius) for every sphere, expressed in the
        depth camera optical frame, using the TF at `stamp` (fallback: latest)."""
        out = []
        # group spheres by link to do one TF lookup per link
        links = {}
        for (frame, x, y, z, r) in self.spheres:
            links.setdefault(frame, []).append((x, y, z, r))
        for frame, items in links.items():
            try:
                tf = self.tf_buffer.lookup_transform(
                    cam_frame, frame, stamp, timeout=Duration(seconds=0.02))
            except (LookupException, ExtrapolationException, ConnectivityException):
                try:
                    tf = self.tf_buffer.lookup_transform(
                        cam_frame, frame, rclpy.time.Time())  # latest available
                except Exception:
                    if frame not in self._warned:
                        self.get_logger().warn(f'no TF {cam_frame}<-{frame} yet')
                        self._warned.add(frame)
                    continue
            t = tf.transform.translation
            q = tf.transform.rotation
            R = _quat_to_R(q.x, q.y, q.z, q.w)
            p = np.array([t.x, t.y, t.z])
            for (x, y, z, r) in items:
                c = R @ np.array([x, y, z]) + p
                out.append((c[0], c[1], c[2], r))
        return out

    def _on_depth(self, msg: Image):
        if msg.encoding not in ('16UC1', 'mono16'):
            if 'enc' not in self._warned:
                self.get_logger().warn(f'depth encoding {msg.encoding}; passthrough')
                self._warned.add('enc')
            self.pub.publish(msg)
            return
        if self.K is None:
            self.pub.publish(msg)      # no intrinsics yet -> passthrough
            return

        fx, fy, cx, cy = self.K
        h, w = msg.height, msg.width
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w).copy()
        depth_m = depth.astype(np.float32) / 1000.0

        if self._uu is None or self._uu.shape != (h, w):
            self._vv, self._uu = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')

        centers = self._sphere_centers_in_camera(msg.header.frame_id, msg.header.stamp)
        masked = 0
        for (Xc, Yc, Zc, r) in centers:
            if Zc <= 0.05:
                continue
            u = fx * Xc / Zc + cx
            v = fy * Yc / Zc + cy
            r_px = fx * r / Zc + self.radius_pad_px
            # bounding box of the projected disc (clamp to image)
            u0 = max(int(u - r_px), 0); u1 = min(int(u + r_px) + 1, w)
            v0 = max(int(v - r_px), 0); v1 = min(int(v + r_px) + 1, h)
            if u0 >= u1 or v0 >= v1:
                continue
            sub_u = self._uu[v0:v1, u0:u1]
            sub_v = self._vv[v0:v1, u0:u1]
            in_disc = (sub_u - u) ** 2 + (sub_v - v) ** 2 <= r_px * r_px
            sub_d = depth_m[v0:v1, u0:u1]
            # z-test: only blank pixels whose depth matches the sphere's depth
            # band (or that are invalid/0). Closer/farther real objects survive.
            near = np.abs(sub_d - Zc) <= (r + self.z_tol_m)
            hit = in_disc & (near | (sub_d <= 0.0))
            block = depth[v0:v1, u0:u1]
            block[hit] = 0
            masked += int(hit.sum())

        out = Image()
        out.header = msg.header
        out.height = h; out.width = w
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = depth.tobytes()
        self.pub.publish(out)
        self.get_logger().info(f'masked {masked} arm px', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArmDepthMaskNode()
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
