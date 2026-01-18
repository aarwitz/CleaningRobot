#!/usr/bin/env python3
"""
Clothes Perception Node - Efficient 2D detection relay + on-demand 3D lookup

Subscribes to:
  - /yolo/detections (vision_msgs/Detection2DArray)
  - /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)
  - /camera/color/camera_info (sensor_msgs/CameraInfo)

Publishes:
  - /clothes/detected (vision_msgs/Detection2D) - best clothes detection (2D only)

Services:
  - /clothes_perception/enable (std_srvs/SetBool)
  - /clothes_perception/reset_target (std_srvs/Trigger)
  - /clothes_perception/get_clothes_3d (behavior_manager/GetClothes3D) - compute 3D on-demand

Algorithm:
  Lightweight mode (continuous):
    1. Receive YOLO detections
    2. Publish best detection (2D only) for behavior manager
    3. No depth sampling, no 3D math, no TF lookups
  
  On-demand mode (service call):
    1. Receive detection via service
    2. Extract 7x7 depth window, compute median
    3. Back-project to 3D using pinhole model
    4. Return PointStamped in camera frame
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge

from vision_msgs.msg import Detection2DArray, Detection2D
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_srvs.srv import SetBool, Trigger
from behavior_manager_interfaces.srv import GetClothes3D


class ClothesPerceptionNode(Node):
    def __init__(self):
        super().__init__('clothes_perception_node')
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('depth_window_size', 7)
        self.declare_parameter('max_depth_m', 5.0)
        self.declare_parameter('min_depth_m', 0.2)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('net_w', 640)  # YOLO network input width
        self.declare_parameter('net_h', 640)  # YOLO network input height
        
        # State
        self.enabled = True
        self.camera_info: CameraInfo = None
        self.latest_depth_image: np.ndarray = None
        self.bridge = CvBridge()
        self.latest_detection: Detection2D = None
        
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
        
        # Publishers - lightweight 2D only
        self.detection_pub = self.create_publisher(
            Detection2D,
            '/clothes/detected',
            10
        )
        
        # Services
        self.get_3d_srv = self.create_service(
            GetClothes3D,
            '/clothes_perception/get_clothes_3d',
            self.get_clothes_3d_callback
        )
        
        self.enable_srv = self.create_service(
            SetBool,
            '/clothes_perception/enable',
            self.enable_callback
        )
        
        self.reset_srv = self.create_service(
            Trigger,
            '/clothes_perception/reset_target',
            self.reset_callback
        )
        
        # Rate limiting timer
        rate_hz = self.get_parameter('rate_hz').value
        self.process_timer = self.create_timer(1.0 / rate_hz, self.process_detections)
        self.pending_detection: Detection2DArray = None
        
        # Debug throttling
        self.last_debug_time = self.get_clock().now()
        
        self.get_logger().info(f'Clothes perception initialized (rate: {rate_hz} Hz, enabled: {self.enabled})')
    
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
    
    def get_clothes_3d_callback(self, request: GetClothes3D.Request, response: GetClothes3D.Response):
        """On-demand 3D lookup - only called when entering PICK state"""
        if self.camera_info is None:
            response.success = False
            response.message = "Camera info not available"
            return response
        
        if self.latest_depth_image is None:
            response.success = False
            response.message = "Depth image not available"
            return response
        
        try:
            # Reuse existing extract_3d_point logic
            point_3d = self.extract_3d_point(request.detection, self.latest_depth_image)
            
            if point_3d is None:
                response.success = False
                response.message = "No valid depth at detection location"
                return response
            
            # Create PointStamped in camera frame
            response.point = PointStamped()
            response.point.header.stamp = self.get_clock().now().to_msg()
            response.point.header.frame_id = self.get_parameter('camera_frame').value
            response.point.point.x = float(point_3d[0])
            response.point.point.y = float(point_3d[1])
            response.point.point.z = float(point_3d[2])
            
            response.success = True
            response.message = f"3D position computed: ({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f})"
            
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Error computing 3D: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Enable/disable perception processing"""
        self.enabled = request.data
        response.success = True
        response.message = f'Clothes perception {"enabled" if self.enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response
    
    def reset_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Reset detection state"""
        self.latest_detection = None
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
        """LIGHTWEIGHT - just find and publish best 2D detection"""
        if not self.enabled or self.pending_detection is None:
            return
        
        if self.camera_info is None:
            self.get_logger().warn('No camera info yet', throttle_duration_sec=5.0)
            return
        
        # Process the queued detection
        msg = self.pending_detection
        self.pending_detection = None
        
        # Find best clothes detection
        best_detection = None
        best_confidence = self.get_parameter('confidence_threshold').value
        
        for detection in msg.detections:
            if not detection.results:
                continue
            
            confidence = detection.results[0].hypothesis.score
            
            if confidence > best_confidence:
                best_detection = detection
                best_confidence = confidence
        
        if best_detection:
            self.latest_detection = best_detection
            self.detection_pub.publish(best_detection)  # 2D only - no depth processing!
            self.get_logger().info(
                f'Published 2D detection with confidence {best_confidence:.2f}',
                throttle_duration_sec=2.0
            )
    
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
        
        # Debug: log successful extraction (throttled)
        if (self.get_clock().now() - self.last_debug_time).nanoseconds < 1.5e9:
            self.get_logger().info(f'Depth at img({u_int}, {v_int}): {depth_m:.3f}m')
        
        return np.array([X, Y, Z])


def main(args=None):
    rclpy.init(args=args)
    node = ClothesPerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
