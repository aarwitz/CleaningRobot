#!/usr/bin/env python3
"""
Minimal YOLOv8 -> 3D object detector / visualizer for ROS2 (Humble)
- Subscribes to:
    /detections_output           (vision_msgs/Detection2DArray)
    /aligned_depth_to_color/image_raw   (sensor_msgs/Image)        <-- depth aligned to color
    /image_rect                  (sensor_msgs/Image)          <-- color (bgr8)
    /aligned_depth_to_color/camera_info (sensor_msgs/CameraInfo)
    /camera_info_rect            (sensor_msgs/CameraInfo)     <-- YOLO image size
- Publishes:
    /yolov8_debug                (sensor_msgs/Image)   overlay image (bgr8 JPEG-friendly)
    /detected_objects_pointcloud (sensor_msgs/PointCloud2)
    /detected_objects_3d        (geometry_msgs/PointStamped)
Async processing with timer-based queue and vectorized operations for performance.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
import numpy as np
import cv2
import struct
import time
import json
from collections import deque
import threading

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray
import message_filters
from message_filters import ApproximateTimeSynchronizer

# Small helpers
def pointcloud2_from_numpy_fast(points_xyz, colors_rgb, header):
    """Vectorized PointCloud2 creation - 100x faster than Python loop.
    points_xyz: (N,3) float32, colors_rgb: (N,3) uint8"""
    if points_xyz is None or len(points_xyz) == 0:
        return None

    pts = points_xyz.astype(np.float32)
    cols = colors_rgb.astype(np.uint8)
    N = pts.shape[0]

    # Define structured dtype for XYZRGB point (16 bytes: 3 floats + 3 uint8 + 1 pad)
    dt = np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('r', np.uint8), ('g', np.uint8), ('b', np.uint8),
        ('_pad', np.uint8)
    ])
    
    # Create structured array and fill
    cloud_arr = np.zeros(N, dtype=dt)
    cloud_arr['x'] = pts[:, 0]
    cloud_arr['y'] = pts[:, 1]
    cloud_arr['z'] = pts[:, 2]
    cloud_arr['r'] = cols[:, 0]
    cloud_arr['g'] = cols[:, 1]
    cloud_arr['b'] = cols[:, 2]
    
    # Convert to bytes in one shot
    data_bytes = cloud_arr.tobytes()

    # Build message
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
        PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
        PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
    ]

    pc2 = PointCloud2()
    pc2.header = header
    pc2.height = 1
    pc2.width = N
    pc2.fields = fields
    pc2.is_bigendian = False
    pc2.point_step = 16
    pc2.row_step = 16 * N
    pc2.is_dense = True
    pc2.data = data_bytes
    return pc2


"""
- A node is like a process that topics can be run on
- A topic is a stream that subscribers and publishers can listen or publish to
- message_filters is a ros utility library that provides wrappers and combiners 
"""

class SimpleYoloV83D(Node):
    def __init__(self):
        super().__init__('yolov8_3d_detector_simple') # Creates an rclpy Node object registered with the ROS 2 graph under the name "yolov8_3d_detector_simple"
        # an rclpy Node object is the main access point to communications with the ROS 2 system with python.
        self.br = CvBridge()

        # Topics - change if your launch uses different names
        self.detections_topic = '/detections_output'
        self.depth_topic = '/camera/aligned_depth_to_color/image_raw'
        self.color_topic = '/camera/color/image_raw'
        self.depth_info_topic = '/camera/aligned_depth_to_color/camera_info'
        self.yolo_info_topic = '/dnn_image_encoder/resize/camera_info'

        # Publishers
        self.debug_img_pub = self.create_publisher(Image, '/yolov8_debug', 5) # buffer size of 5 messages so subscribers running late are less likely to miss any published messages
        self.obj_pc_pub = self.create_publisher(PointCloud2, '/detected_objects_pointcloud', 5)
        self.center_pub = self.create_publisher(PointStamped, '/detected_objects_3d', 5)

        # Async processing queue (keep only latest frame) so executor thread never blocks
        self.frame_queue = deque(maxlen=1)
        self.queue_lock = threading.Lock()

        # Subscribers with approximate sync
        self.detection_sub = message_filters.Subscriber(self, Detection2DArray, self.detections_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.color_sub = message_filters.Subscriber(self, Image, self.color_topic)
        self.depth_info_sub = message_filters.Subscriber(self, CameraInfo, self.depth_info_topic)
        self.yolo_info_sub = message_filters.Subscriber(self, CameraInfo, self.yolo_info_topic)

        # ApproximateTimeSynchronizer is part of message_filters 
        # it is a synchronizer/filter that buffers messages from the message_filters.Subscriber objects you pass it
        # and looks for sets whose header.stamp values are within the configured slop.
        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub, self.color_sub, self.depth_info_sub, self.yolo_info_sub],
            queue_size=30, # increased from 10 - TODO why does increasing queue size and slop help "smooth" detection?
            slop=0.2       # increased from 0.08 (200 ms tolerance for matching timestamps)
        )
        self.sync.registerCallback(self.callback_sync)
        # registerCallback(your_fn) simply tells that synchronizer which function to call when it finds a matched set.
        # The message_filters.Subscribers receive messages from the ROS middleware, forward them into the synchronizer’s buffers,
        # and the synchronizer invokes your registered callback with the matched messages.

        #1: Timer for processing (30 Hz) TODO: is this even used?
        self.process_timer = self.create_timer(1.0 / 30.0, self.process_frame)

        # Simple class list for label drawing (adjust to your model)
        self.class_names = ['sock']

        # Basic thresholds
        self.min_depth_mm = 100      # 10 cm
        self.max_depth_mm = 5000     # 5 m

        self.frame_count = 0
        self.get_logger().info("Simple YOLOv8 3D detector started")

    def callback_sync(self, detections_msg, depth_msg, color_msg, depth_info_msg, yolo_info_msg):
        """Enqueue synced frame immediately (non-blocking). 
            TODO: how does this help move heavy work off the executor thread
        """
        with self.queue_lock:
            self.frame_queue.append({
                'detections': detections_msg,
                'depth': depth_msg,
                'color': color_msg,
                'depth_info': depth_info_msg,
                'yolo_info': yolo_info_msg
            })

    def process_frame(self):
        """Process latest frame from queue (runs on timer). TODO: how does this help get work off main executor thread?"""
        with self.queue_lock:
            if not self.frame_queue:
                return
            frame = self.frame_queue.pop()
        
        self.frame_count += 1
        verbose = (self.frame_count % 30 == 0)  # Log every second
        
        detections_msg = frame['detections']
        depth_msg = frame['depth']
        color_msg = frame['color']
        depth_info_msg = frame['depth_info']
        yolo_info_msg = frame['yolo_info']

        if verbose:
            self.get_logger().info(f"Frame {self.frame_count}: {len(detections_msg.detections)} detections")
        try:
            color = self.br.imgmsg_to_cv2(color_msg, desired_encoding='bgr8') # convert to bgr for opencv
            depth = self.br.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # DEBUG: Check depth image stats
        if verbose:
            depth_nonzero = depth[depth > 0]
            if depth_nonzero.size > 0:
                self.get_logger().info(f"Depth image: shape={depth.shape}, dtype={depth.dtype}, "
                                      f"nonzero pixels={depth_nonzero.size}, "
                                      f"depth range: {np.min(depth_nonzero)} to {np.max(depth_nonzero)}")
            else:
                self.get_logger().warn(f"Depth image has NO valid data! All zeros.")

        # Intrinsics from aligned depth camera_info
        # fx, fy = focal lengths expressed in pixels. 
        # cx, cy = principal point (optical center) in pixel coordinates.
        fx = depth_info_msg.k[0]
        fy = depth_info_msg.k[4]
        cx = depth_info_msg.k[2]
        cy = depth_info_msg.k[5]

        det_w = int(yolo_info_msg.width) # 640
        det_h = int(yolo_info_msg.height) # 640
        color_h, color_w = color.shape[:2] # 480 x 640 (h x w)

        # Make copy for overlay
        overlay = color.copy()

        # Loop over detections
        for i, det in enumerate(detections_msg.detections):
            if not det.results:
                continue
            res = det.results[0] # The results array contains multiple class predictions for the same bounding box. Take the first one.
            score = float(res.hypothesis.score)
            cls_id = int(res.hypothesis.class_id)
            label = self.class_names[cls_id] if cls_id < len(self.class_names) else f"class{cls_id}"

            # get bbox pixel coordinates of detection in YOLO image space
            bbox_cx = det.bbox.center.position.x
            bbox_cy = det.bbox.center.position.y
            bbox_w = det.bbox.size_x
            bbox_h = det.bbox.size_y

            # Letterbox transformation: camera 1280x720 -> YOLO 640x640
            # Scale factor: min(640/1280, 640/720) = 0.5
            # Scaled dims: 640x360, letterbox padding: (640-360)/2 = 140px top/bottom
            scale = min(det_w / color_w, det_h / color_h)
            scaled_w = color_w * scale
            scaled_h = color_h * scale
            pad_x = (det_w - scaled_w) / 2
            pad_y = (det_h - scaled_h) / 2
            
            # Convert YOLO coords (640x640) to camera coords (1280x720)
            x1 = int(round((bbox_cx - bbox_w / 2 - pad_x) / scale))
            y1 = int(round((bbox_cy - bbox_h / 2 - pad_y) / scale))
            x2 = int(round((bbox_cx + bbox_w / 2 - pad_x) / scale))
            y2 = int(round((bbox_cy + bbox_h / 2 - pad_y) / scale))

            # clamp to image size
            x1 = max(0, min(color_w-1, x1))
            x2 = max(0, min(color_w-1, x2))
            y1 = max(0, min(color_h-1, y1))
            y2 = max(0, min(color_h-1, y2))

            # draw bbox & label
            cv2.rectangle(overlay, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(overlay, f"{label} {score:.2f}", (x1, max(12,y1-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
            
            # draw center crosshair
            cx_draw = int(round((bbox_cx - pad_x) / scale))
            cy_draw = int(round((bbox_cy - pad_y) / scale))
            cv2.drawMarker(overlay, (cx_draw, cy_draw), (0,0,255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=12, thickness=2)

            # DEBUG: Check depth at bbox location
            if verbose:
                bbox_depth_crop = depth[y1:y2, x1:x2]
                depth_nz = bbox_depth_crop[bbox_depth_crop > 0]
                self.get_logger().info(f"  Bbox coords: x=[{x1},{x2}], y=[{y1},{y2}], "
                                      f"bbox_depth nonzero={depth_nz.size}, "
                                      f"range={np.min(depth_nz) if depth_nz.size > 0 else 0} to {np.max(depth_nz) if depth_nz.size > 0 else 0}")

            # create object point cloud from bbox
            points, colors = self.create_object_pointcloud(depth, color, x1, y1, x2, y2, fx, fy, cx, cy)
            
            # Skip this detection if no valid pointcloud
            if points is None or len(points) == 0:
                if verbose:
                    self.get_logger().warn(f"  No valid pointcloud for detection {i}")
                continue
                
            # publish object point cloud message
            pc2 = pointcloud2_from_numpy_fast(points, colors, detections_msg.header)
            if pc2 is not None:
                self.obj_pc_pub.publish(pc2)

            # Compute 3D center from median of pointcloud (much more robust than 5x5 patch!)
            centroid = np.median(points, axis=0)
            
            # Publish 3D center
            center_msg = PointStamped()
            center_msg.header = detections_msg.header
            center_msg.point.x = float(centroid[0])
            center_msg.point.y = float(centroid[1])
            center_msg.point.z = float(centroid[2])
            self.center_pub.publish(center_msg)
            
            # log the center point x, y, z in meters
            if verbose:
                self.get_logger().info(
                    f"  [{i}] {label}: x={centroid[0]:.3f}, y={centroid[1]:.3f}, z={centroid[2]:.3f} m ({len(points)} pts)"
                )
            # processed += 1

        # publish overlay image
        try:
            img_msg = self.br.cv2_to_imgmsg(overlay, encoding='bgr8')
            img_msg.header = detections_msg.header
            self.debug_img_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish overlay image: {e}")

    def create_object_pointcloud(self, depth_img, color_img, x1, y1, x2, y2, fx, fy, cx, cy):
        """Crop bbox, filter depth, return Nx3 points (meters) and Nx3 uint8 RGB colors."""
        # Safe crop
        h, w = depth_img.shape
        x1c = max(0, min(w-1, x1)); x2c = max(0, min(w-1, x2))
        y1c = max(0, min(h-1, y1)); y2c = max(0, min(h-1, y2))
        if x2c <= x1c or y2c <= y1c:
            return None, None

        depth_crop = depth_img[y1c:y2c+1, x1c:x2c+1]
        color_crop = color_img[y1c:y2c+1, x1c:x2c+1]

        if depth_crop.size == 0:
            return None, None

        # handle depth dtype (uint16 in mm OR float32 in meters)
        is_uint16 = np.issubdtype(depth_crop.dtype, np.integer)
        if is_uint16:
            depth_mm = depth_crop.astype(np.float32)
        else:
            # assume float32 in meters => convert to mm for thresholds then to meters later
            depth_mm = (depth_crop.astype(np.float32) * 1000.0)

        # Valid depth mask
        mask = (depth_mm > self.min_depth_mm) & (depth_mm < self.max_depth_mm)

        if not np.any(mask):
            return None, None

        # Use central region to compute robust median depth to avoid rim pixels
        H, W = depth_mm.shape
        cx0 = int(0.3 * W); cx1 = int(0.7 * W)
        cy0 = int(0.3 * H); cy1 = int(0.7 * H)
        center_patch = depth_mm[cy0:cy1, cx0:cx1]
        valid_center = center_patch[(center_patch > self.min_depth_mm) & (center_patch < self.max_depth_mm)]
        if valid_center.size == 0:
            # fallback to all valid
            z_med_mm = float(np.median(depth_mm[mask]))
        else:
            z_med_mm = float(np.median(valid_center))

        # Keep points within +/- 50 mm of median (tunable)
        band_mm = 50.0
        inlier_mask = mask & (np.abs(depth_mm - z_med_mm) <= band_mm)

        if not np.any(inlier_mask):
            # loosen band if nothing remains
            inlier_mask = mask

        v_idx, u_idx = np.nonzero(inlier_mask)
        if v_idx.size == 0:
            return None, None

        depth_vals_m = depth_mm[v_idx, u_idx] / 1000.0  # meters

        # compute pixel coords in full image (not crop)
        pixel_x = x1c + u_idx
        pixel_y = y1c + v_idx

        X = (pixel_x - cx) * depth_vals_m / fx
        Y = (pixel_y - cy) * depth_vals_m / fy
        Z = depth_vals_m

        points = np.stack([X, Y, Z], axis=1)

        # grab colors (BGR -> RGB)
        rgb = color_crop[v_idx, u_idx]  # BGR
        colors = np.stack([rgb[:,2], rgb[:,1], rgb[:,0]], axis=1).astype(np.uint8)

        return points, colors

def main(args=None):
    rclpy.init(args=args) # initializes the Python ROS 2 client library (rclpy) so we can create Node objects
    node = SimpleYoloV83D() # init our node
    try:
        rclpy.spin(node) # Starts the ROS executor loop for that node and blocks the current thread, 
        # processing incoming ROS work (subscription callbacks, service requests, timers, action events, etc.) 
        # until the process is shut down.
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()