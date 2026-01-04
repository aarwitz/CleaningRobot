#!/usr/bin/env python3
"""
Sock Perception Node - 3D target extraction from YOLO detections + depth

Subscribes to:
  - /yolo/detections (vision_msgs/Detection2DArray)
  - /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)
  - /camera/color/camera_info (sensor_msgs/CameraInfo)

Publishes:
  - /sock/target_point_map (geometry_msgs/PoseStamped) - filtered 3D point in map frame
  - /sock/target_point_camera (geometry_msgs/PointStamped) - debug, camera frame
  - /sock/target_depth_m (std_msgs/Float32) - debug, depth value

Services:
  - /sock_perception/enable (std_srvs/SetBool)
  - /sock_perception/reset_target (std_srvs/Trigger)

Algorithm:
1. Receive YOLO detection bbox
2. Compute bbox center pixel (u, v)
3. Extract 7x7 window from aligned depth, compute median (ignoring 0/NaN)
4. Back-project to camera coordinates using pinhole model:
   X = (u - cx) * Z / fx
   Y = (v - cy) * Z / fy
   Z = depth_m
5. Transform to map frame via TF2
6. Apply temporal filter (median of last K points)
7. Publish as PoseStamped
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from collections import deque
import cv2
from cv_bridge import CvBridge

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, Trigger
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs


class SockPerceptionNode(Node):
    def __init__(self):
        super().__init__('sock_perception_node')
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('temporal_filter_size', 5)
        self.declare_parameter('depth_window_size', 7)
        self.declare_parameter('max_depth_m', 5.0)
        self.declare_parameter('min_depth_m', 0.2)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('net_w', 640)  # YOLO network input width
        self.declare_parameter('net_h', 640)  # YOLO network input height
        
        # State
        self.enabled = True
        self.camera_info: CameraInfo = None
        self.latest_depth_image: np.ndarray = None
        self.bridge = CvBridge()
        
        # Temporal filter
        filter_size = self.get_parameter('temporal_filter_size').value
        self.point_history = deque(maxlen=filter_size)
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS for camera topics - try RELIABLE first to match RealSense publisher
        qos_profile_reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos_profile_best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            qos_profile_reliable  # Match RealSense publisher
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            qos_profile_best_effort  # This one is typically best effort
        )
        
        # Publishers
        self.target_map_pub = self.create_publisher(
            PoseStamped,
            '/sock/target_point_map',
            10
        )
        
        self.target_camera_pub = self.create_publisher(
            PointStamped,
            '/sock/target_point_camera',
            10
        )
        
        self.depth_pub = self.create_publisher(
            Float32,
            '/sock/target_depth_m',
            10
        )
        
        # Services
        self.enable_srv = self.create_service(
            SetBool,
            '/sock_perception/enable',
            self.enable_callback
        )
        
        self.reset_srv = self.create_service(
            Trigger,
            '/sock_perception/reset_target',
            self.reset_callback
        )
        
        # Rate limiting timer
        rate_hz = self.get_parameter('rate_hz').value
        self.process_timer = self.create_timer(1.0 / rate_hz, self.process_detections)
        self.pending_detection: Detection2DArray = None
        
        # Debug throttling
        self.last_debug_time = self.get_clock().now()
        
        self.get_logger().info(f'Sock perception initialized (rate: {rate_hz} Hz, enabled: {self.enabled})')
    
    def network_to_image_coords(self, u_net: float, v_net: float) -> tuple:
        """
        Convert YOLO network coordinates to camera image coordinates.
        Reverses letterboxing applied during YOLO preprocessing.
        
        Args:
            u_net, v_net: Coordinates in YOLO network space (e.g., 640x640)
        
        Returns:
            (u_img, v_img): Coordinates in camera image space (e.g., 640x480)
        """
        if self.camera_info is None:
            return u_net, v_net
        
        cam_w = self.camera_info.width
        cam_h = self.camera_info.height
        net_w = self.get_parameter('net_w').value
        net_h = self.get_parameter('net_h').value
        
        # Compute letterbox scale and padding
        scale = min(net_w / cam_w, net_h / cam_h)
        pad_x = (net_w - cam_w * scale) / 2.0
        pad_y = (net_h - cam_h * scale) / 2.0
        
        # Reverse letterboxing
        u_img = (u_net - pad_x) / scale
        v_img = (v_net - pad_y) / scale
        
        # Clamp to image bounds
        u_img = max(0, min(u_img, cam_w - 1))
        v_img = max(0, min(v_img, cam_h - 1))
        
        return u_img, v_img
    
    def enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Enable/disable perception processing"""
        self.enabled = request.data
        response.success = True
        response.message = f'Sock perception {"enabled" if self.enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response
    
    def reset_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Reset target and temporal filter"""
        self.point_history.clear()
        self.pending_detection = None
        response.success = True
        response.message = 'Target reset'
        self.get_logger().info(response.message)
        return response
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(f'Received camera info: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}')
    
    def depth_callback(self, msg: Image):
        """Store latest depth image"""
        try:
            # Depth is uint16 in millimeters typically, or float32 in meters
            if msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                self.latest_depth_image = depth_image.astype(np.float32) / 1000.0  # mm to m
            elif msg.encoding == '32FC1':
                self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
                return
            
            # Debug: log depth stats occasionally
            if self.latest_depth_image is not None:
                valid_pixels = self.latest_depth_image[self.latest_depth_image > 0]
                self.get_logger().debug(
                    f'Depth image: {msg.encoding}, shape={self.latest_depth_image.shape}, '
                    f'valid_pixels={len(valid_pixels)}, '
                    f'min={valid_pixels.min() if len(valid_pixels) > 0 else 0:.3f}, '
                    f'max={valid_pixels.max() if len(valid_pixels) > 0 else 0:.3f}',
                    throttle_duration_sec=5.0
                )
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def detection_callback(self, msg: Detection2DArray):
        """Queue detection for rate-limited processing"""
        if self.enabled:
            self.pending_detection = msg
    
    def process_detections(self):
        """Process detection at controlled rate"""
        if not self.enabled or self.pending_detection is None:
            return
        
        if self.camera_info is None:
            self.get_logger().warn('No camera info yet', throttle_duration_sec=5.0)
            return
        
        if self.latest_depth_image is None:
            self.get_logger().warn('No depth image yet', throttle_duration_sec=5.0)
            return
        
        # Process the queued detection
        msg = self.pending_detection
        self.pending_detection = None
        
        # Find best sock detection
        best_detection = None
        best_confidence = self.get_parameter('confidence_threshold').value
        
        for detection in msg.detections:
            if not detection.results:
                continue
            
            # Assuming results[0] is the class hypothesis
            confidence = detection.results[0].hypothesis.score
            class_id = detection.results[0].hypothesis.class_id
            
            # Check if this is a sock (you may need to check class_id or label)
            # For now, take any detection above threshold
            if confidence > best_confidence:
                best_detection = detection
                best_confidence = confidence
        
        if best_detection is None:
            return
        
        # Extract 3D point
        try:
            point_camera = self.extract_3d_point(best_detection, self.latest_depth_image)
            if point_camera is None:
                return
            
            # Add to temporal filter
            self.point_history.append(point_camera)
            
            # Compute filtered point (median of history)
            filtered_point = self.compute_filtered_point()
            if filtered_point is None:
                return
            
            # Publish in camera frame (debug)
            self.publish_camera_point(filtered_point, msg.header.stamp)
            
            # Transform to map frame
            point_map = self.transform_to_map(filtered_point, msg.header.stamp)
            if point_map is not None:
                self.publish_map_target(point_map, msg.header.stamp)
                
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
    
    def extract_3d_point(self, detection, depth_image: np.ndarray) -> np.ndarray:
        """
        Extract 3D point from detection bbox and depth image.
        
        Returns: [X, Y, Z] in camera frame, or None if invalid
        """
        bbox = detection.bbox
        
        # Get bbox center in YOLO network coordinates
        u_net = bbox.center.position.x
        v_net = bbox.center.position.y
        
        # Convert from network space to image space
        u_img, v_img = self.network_to_image_coords(u_net, v_net)
        
        # Debug logging (throttled to ~1Hz)
        now = self.get_clock().now()
        if (now - self.last_debug_time).nanoseconds > 1e9:  # 1 second
            self.last_debug_time = now
            self.get_logger().info(
                f'Coordinate conversion: net({u_net:.1f}, {v_net:.1f}) → '
                f'img({u_img:.1f}, {v_img:.1f})'
            )
        
        # Get camera intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        # Extract depth window around center (using IMAGE coordinates)
        window_size = self.get_parameter('depth_window_size').value
        half_win = window_size // 2
        
        u_int = int(u_img)
        v_int = int(v_img)
        
        # Check bounds
        h, w = depth_image.shape
        if not (half_win <= u_int < w - half_win and half_win <= v_int < h - half_win):
            self.get_logger().warn(f'Bbox center out of bounds: ({u_int}, {v_int})')
            return None
        
        # Extract window
        window = depth_image[
            v_int - half_win:v_int + half_win + 1,
            u_int - half_win:u_int + half_win + 1
        ]
        
        # Debug: log window stats
        self.get_logger().debug(
            f'Bbox at ({u_int}, {v_int}), window: '
            f'zeros={np.sum(window == 0)}, '
            f'nans={np.sum(~np.isfinite(window))}, '
            f'valid={np.sum((window > 0) & np.isfinite(window))}, '
            f'min={window[window > 0].min() if np.any(window > 0) else 0:.3f}, '
            f'max={window[window > 0].max() if np.any(window > 0) else 0:.3f}'
        )
        
        # Get valid depths (ignore 0 and NaN)
        valid_depths = window[(window > 0) & (np.isfinite(window))]
        
        if len(valid_depths) == 0:
            self.get_logger().warn(
                f'No valid depth in window at ({u_int}, {v_int}): '
                f'{np.sum(window == 0)} zeros, {np.sum(~np.isfinite(window))} invalid'
            )
            return None
        
        # Compute median depth
        depth_m = float(np.median(valid_depths))
        
        # Check depth range
        min_depth = self.get_parameter('min_depth_m').value
        max_depth = self.get_parameter('max_depth_m').value
        
        if not (min_depth < depth_m < max_depth):
            self.get_logger().warn(f'Depth out of range: {depth_m:.2f}m')
            return None
        
        # Back-project to 3D using pinhole model
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = depth_m
        X = (u_img - cx) * depth_m / fx
        Y = (v_img - cy) * depth_m / fy
        Z = depth_m
        
        # Publish debug depth
        depth_msg = Float32()
        depth_msg.data = depth_m
        self.depth_pub.publish(depth_msg)
        
        # Debug: log successful extraction (throttled)
        if (self.get_clock().now() - self.last_debug_time).nanoseconds < 1.5e9:
            self.get_logger().info(f'Depth at img({u_int}, {v_int}): {depth_m:.3f}m')
        
        return np.array([X, Y, Z])
    
    def compute_filtered_point(self) -> np.ndarray:
        """Compute median of point history for stability"""
        if len(self.point_history) == 0:
            return None
        
        # Stack points and compute median per axis
        points = np.array(list(self.point_history))
        filtered = np.median(points, axis=0)
        
        return filtered
    
    def publish_camera_point(self, point: np.ndarray, stamp):
        """Publish point in camera frame (debug)"""
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.get_parameter('camera_frame').value
        msg.point.x = float(point[0])
        msg.point.y = float(point[1])
        msg.point.z = float(point[2])
        
        self.target_camera_pub.publish(msg)
    
    def transform_to_map(self, point: np.ndarray, stamp) -> PoseStamped:
        """Transform point from camera frame to map frame"""
        camera_frame = self.get_parameter('camera_frame').value
        map_frame = self.get_parameter('map_frame').value
        
        # Create PointStamped in camera frame
        point_camera = PointStamped()
        point_camera.header.stamp = stamp
        point_camera.header.frame_id = camera_frame
        point_camera.point.x = float(point[0])
        point_camera.point.y = float(point[1])
        point_camera.point.z = float(point[2])
        
        try:
            # Transform to map frame
            transform = self.tf_buffer.lookup_transform(
                map_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
            
            # Convert to PoseStamped
            pose_map = PoseStamped()
            pose_map.header = point_map.header
            pose_map.pose.position = point_map.point
            pose_map.pose.orientation.w = 1.0  # Identity orientation
            
            return pose_map
            
        except TransformException as e:
            self.get_logger().warn(f'TF error: {e}', throttle_duration_sec=5.0)
            return None
    
    def publish_map_target(self, pose: PoseStamped, stamp):
        """Publish final filtered target in map frame"""
        pose.header.stamp = stamp
        self.target_map_pub.publish(pose)
        
        self.get_logger().info(
            f'Published sock target: ({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f}, {pose.pose.position.z:.2f})',
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = SockPerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
