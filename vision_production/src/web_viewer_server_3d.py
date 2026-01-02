#!/usr/bin/env python3
# filepath: /home/taylor/workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/web_viewer_server_3d.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

import cv2
import asyncio
import websockets
import threading
import time
import json
import struct
import traceback
import numpy as np
import hashlib
import json
import numpy as np
import struct
import traceback

class Enhanced3DViewer(Node):
    def __init__(self):
        super().__init__('enhanced_3d_viewer_debug')
        self.bridge = CvBridge()
        
        # Store latest data with timestamps
        self._latest_2d_frame = None
        self._latest_pointcloud = None
        self._latest_full_scene = None
        self._latest_metadata = None
        self._latest_center_point = None
        self._latest_slam_trajectory = []
        self._latest_slam_pose = None
        self._latest_slam_landmarks = None
        self._last_2d_time = 0
        self._last_3d_time = 0
        self._last_full_scene_time = 0
        self._last_meta_time = 0
        self._last_center_time = 0
        self._last_slam_trajectory_time = 0
        self._last_slam_pose_time = 0
        self._last_slam_landmarks_time = 0
        self._lock = threading.Lock()
        
        # Cache hashes to detect changes
        self._last_2d_hash = None
        self._last_3d_hash = None
        self._last_full_scene_hash = None

        # Counters for debugging
        self._2d_count = 0
        self._3d_count = 0
        self._full_scene_count = 0
        self._meta_count = 0
        self._center_count = 0
        self._slam_odom_count = 0
        self._slam_landmarks_count = 0

        # Subscribers with debugging
        self.create_subscription(Image, '/yolov8_debug', self.image_callback, 10)
        self.create_subscription(PointCloud2, '/detected_objects_pointcloud', self.pointcloud_callback, 10)
        self.create_subscription(PointCloud2, '/full_scene_pointcloud', self.full_scene_callback, 10)
        self.create_subscription(String, '/detection_metadata', self.metadata_callback, 10)
        self.create_subscription(PointStamped, '/detected_objects_3d', self.center_point_callback, 10)
        
        # SLAM subscribers
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.slam_odom_callback, 10)
        self.create_subscription(PointCloud2, '/visual_slam/vis/landmarks_cloud', self.slam_landmarks_callback, 10)

        # Create a timer to report status
        self.create_timer(5.0, self.report_status)

        self.get_logger().info('🚀 Enhanced 3D Viewer Debug Node Started!')

    def report_status(self):
        """Report status every 5 seconds"""
        with self._lock:
            now = time.time()
            self.get_logger().info(
                f"📊 Status - 2D: {self._2d_count} msgs "
                f"(last: {now - self._last_2d_time:.1f}s ago), "
                f"3D: {self._3d_count} msgs "
                f"(last: {now - self._last_3d_time:.1f}s ago), "
                f"FullScene: {self._full_scene_count} msgs "
                f"(last: {now - self._last_full_scene_time:.1f}s ago), "
                f"Meta: {self._meta_count} msgs "
                f"(last: {now - self._last_meta_time:.1f}s ago), "
                f"Center: {self._center_count} msgs "
                f"(last: {now - self._last_center_time:.1f}s ago), "
                f"SLAM Odom: {self._slam_odom_count} msgs "
                f"(last: {now - self._last_slam_trajectory_time:.1f}s ago), "
                f"SLAM Landmarks: {self._slam_landmarks_count} msgs "
                f"(last: {now - self._last_slam_landmarks_time:.1f}s ago)"
            )

    def image_callback(self, msg):
        try:
            self._2d_count += 1
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # JPEG quality 80 - good balance of quality and size
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                jpeg_bytes = jpeg.tobytes()
                with self._lock:
                    self._latest_2d_frame = jpeg_bytes
                    self._last_2d_time = time.time()
                    self._last_2d_hash = hashlib.md5(jpeg_bytes).digest()
                if self._2d_count % 10 == 0:  # Log every 10th frame
                    self.get_logger().info(f"🖼️ Processed 2D frame #{self._2d_count}, size: {len(jpeg_bytes)} bytes")
        except Exception as e:
            self.get_logger().error(f"Failed to process 2D image: {e}")
            traceback.print_exc()

    def pointcloud_callback(self, msg):
        try:
            self._3d_count += 1
            # Convert PointCloud2 to binary for web transmission (reduced for performance)
            points_bytes = self.pointcloud2_to_binary(msg, max_points=3000)
            with self._lock:
                self._latest_pointcloud = points_bytes
                self._last_3d_time = time.time()
                self._last_3d_hash = hashlib.md5(points_bytes).digest()
            if self._3d_count % 5 == 0:  # Log every 5th point cloud
                try:
                    n = struct.unpack_from('<I', points_bytes, 0)[0]
                    self.get_logger().info(f"📡 Processed point cloud #{self._3d_count} with {n} points, {len(points_bytes)} bytes")
                except Exception:
                    self.get_logger().info(f"📡 Processed point cloud #{self._3d_count}")
        except Exception as e:
            self.get_logger().error(f"Failed to process point cloud: {e}")
            traceback.print_exc()

    def full_scene_callback(self, msg):
        try:
            self._full_scene_count += 1
            # Convert PointCloud2 to binary for web transmission (reduced for performance)
            points_bytes = self.pointcloud2_to_binary(msg, max_points=5000)
            with self._lock:
                self._latest_full_scene = points_bytes
                self._last_full_scene_time = time.time()
                self._last_full_scene_hash = hashlib.md5(points_bytes).digest()
            if self._full_scene_count % 5 == 0:  # Log every 5th full scene
                try:
                    n = struct.unpack_from('<I', points_bytes, 0)[0]
                    self.get_logger().info(f"🌍 Processed full scene #{self._full_scene_count} with {n} points, {len(points_bytes)} bytes")
                except Exception:
                    self.get_logger().info(f"🌍 Processed full scene #{self._full_scene_count}")
        except Exception as e:
            self.get_logger().error(f"Failed to process full scene: {e}")
            traceback.print_exc()

    def metadata_callback(self, msg):
        try:
            self._meta_count += 1
            # msg.data is already a string when using std_msgs/String
            json_str = msg.data
            with self._lock:
                self._latest_metadata = json_str.encode('utf-8')
                self._last_meta_time = time.time()
            if self._meta_count % 5 == 0:  # Log every 5th metadata
                self.get_logger().info(f"📊 Received metadata #{self._meta_count}: {json_str[:100]}...")
        except Exception as e:
            self.get_logger().error(f"Failed to process metadata: {e}")
            traceback.print_exc()

    def center_point_callback(self, msg):
        try:
            self._center_count += 1
            # Convert PointStamped to JSON for web transmission
            center_data = {
                'x': float(-msg.point.x),  # Apply same coordinate transforms as point cloud
                'y': float(-msg.point.y),
                'z': float(msg.point.z),
                'timestamp': time.time()
            }
            with self._lock:
                self._latest_center_point = json.dumps(center_data).encode('utf-8')
                self._last_center_time = time.time()
            if self._center_count % 5 == 0:  # Log every 5th center point
                self.get_logger().info(f"🎯 Received center point #{self._center_count}: ({center_data['x']:.3f}, {center_data['y']:.3f}, {center_data['z']:.3f})")
        except Exception as e:
            self.get_logger().error(f"Failed to process center point: {e}")
            traceback.print_exc()

    def slam_odom_callback(self, msg):
        """Build trajectory from odometry messages"""
        try:
            self._slam_odom_count += 1
            # Extract position and orientation
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation
            
            pose_data = {
                'position': {
                    'x': float(pos.x),
                    'y': float(pos.y),
                    'z': float(pos.z)
                },
                'orientation': {
                    'x': float(orient.x),
                    'y': float(orient.y),
                    'z': float(orient.z),
                    'w': float(orient.w)
                },
                'timestamp': time.time()
            }
            
            with self._lock:
                # Add to trajectory (keep last 1000 points)
                self._latest_slam_trajectory.append(pose_data['position'])
                if len(self._latest_slam_trajectory) > 1000:
                    self._latest_slam_trajectory.pop(0)
                
                # Update current pose
                self._latest_slam_pose = json.dumps(pose_data).encode('utf-8')
                self._last_slam_trajectory_time = time.time()
                self._last_slam_pose_time = time.time()
                
            if self._slam_odom_count % 30 == 0:  # Log every 30th message
                self.get_logger().info(f"🗺️  SLAM odometry #{self._slam_odom_count}, trajectory points: {len(self._latest_slam_trajectory)}")
        except Exception as e:
            self.get_logger().error(f"Failed to process SLAM odometry: {e}")
            traceback.print_exc()

    def slam_landmarks_callback(self, msg):
        """Process SLAM landmark point cloud"""
        try:
            self._slam_landmarks_count += 1
            # Convert to binary (limit points for performance)
            landmarks_bytes = self.pointcloud2_to_binary(msg, max_points=10000)
            with self._lock:
                self._latest_slam_landmarks = landmarks_bytes
                self._last_slam_landmarks_time = time.time()
            if self._slam_landmarks_count % 10 == 0:
                self.get_logger().info(f"🏔️  SLAM landmarks #{self._slam_landmarks_count}")
        except Exception as e:
            self.get_logger().error(f"Failed to process SLAM landmarks: {e}")
            traceback.print_exc()



    def pointcloud2_to_binary(self, pc_msg, max_points=10000):
        """Efficiently pack point cloud to binary: header (uint32 N) + float32 xyz (3*N) + uint8 rgb (3*N)"""
        try:
            # Get field offsets
            x_offset = y_offset = z_offset = r_off = g_off = b_off = None
            for field in pc_msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
                elif field.name == 'r':
                    r_off = field.offset
                elif field.name == 'g':
                    g_off = field.offset
                elif field.name == 'b':
                    b_off = field.offset
            
            if any(offset is None for offset in [x_offset, y_offset, z_offset, r_off, g_off, b_off]):
                self.get_logger().warn(f"Missing required point cloud fields. Available: {[f.name for f in pc_msg.fields]}")
                return struct.pack('<I', 0)
            
            # Convert to bytes
            if hasattr(pc_msg.data, 'tobytes'):
                data_bytes = pc_msg.data.tobytes()
            else:
                data_bytes = bytes(pc_msg.data)
            
            point_step = pc_msg.point_step
            total_points = int(pc_msg.width) * int(pc_msg.height or 1)
            
            # Sample points uniformly if too many
            if total_points <= max_points:
                indices = np.arange(total_points)
            else:
                stride = max(1, total_points // max_points)
                indices = np.arange(0, total_points, stride)[:max_points]
            
            num_points = len(indices)
            
            # Vectorized extraction using numpy (much faster than struct loop)
            data_array = np.frombuffer(data_bytes, dtype=np.uint8)
            
            # Extract XYZ as float32
            xyz = np.empty((num_points, 3), dtype=np.float32)
            for i, idx in enumerate(indices):
                offset = idx * point_step
                xyz[i, 0] = -np.frombuffer(data_bytes[offset + x_offset:offset + x_offset + 4], dtype='<f4')[0]  # -x
                xyz[i, 1] = -np.frombuffer(data_bytes[offset + y_offset:offset + y_offset + 4], dtype='<f4')[0]  # -y
                xyz[i, 2] = np.frombuffer(data_bytes[offset + z_offset:offset + z_offset + 4], dtype='<f4')[0]   # z
            
            # Extract RGB as uint8
            rgb = np.empty((num_points, 3), dtype=np.uint8)
            for i, idx in enumerate(indices):
                offset = idx * point_step
                rgb[i, 0] = data_array[offset + r_off]
                rgb[i, 1] = data_array[offset + g_off]
                rgb[i, 2] = data_array[offset + b_off]
            
            # Pack: uint32 count + float32 xyz array + uint8 rgb array
            header = struct.pack('<I', num_points)
            return header + xyz.tobytes() + rgb.tobytes()
            
        except Exception as e:
            self.get_logger().error(f"Error in pointcloud2_to_binary: {e}")
            traceback.print_exc()
            return struct.pack('<I', 0)
    
    def pointcloud2_to_json(self, pc_msg):
        """Convert PointCloud2 to JSON format for web visualization"""
        try:
            # Parse point cloud data
            points = []
            colors = []
            
            # Get field offsets for separate r, g, b fields
            x_offset = y_offset = z_offset = r_off = g_off = b_off = None
            for field in pc_msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
                elif field.name == 'r':
                    r_off = field.offset
                elif field.name == 'g':
                    g_off = field.offset
                elif field.name == 'b':
                    b_off = field.offset
            
            if any(offset is None for offset in [x_offset, y_offset, z_offset, r_off, g_off, b_off]):
                self.get_logger().warn(f"Missing required point cloud fields. Available: {[f.name for f in pc_msg.fields]}")
                return {'points': [], 'colors': [], 'num_points': 0, 'timestamp': time.time()}
            
            # Convert array.array to bytes if needed
            if hasattr(pc_msg.data, 'tobytes'):
                data_bytes = pc_msg.data.tobytes()
            else:
                data_bytes = bytes(pc_msg.data)
            
            # Extract points
            point_step = pc_msg.point_step
            
            # Compute total points
            total_points = int(pc_msg.width) * int(pc_msg.height or 1)
            max_points = min(total_points, 20000)  # limit to 20k for web

            # Choose indices to sample uniformly (avoid bias)
            if total_points <= max_points:
                indices = list(range(total_points))
            else:
                # sample without replacement
                # fast approach: numpy.choice (may allocate) or strided sampling
                # here do uniform striding for speed
                stride = max(1, total_points // max_points)
                indices = list(range(0, total_points, stride))[:max_points]
            
            for i in indices:
                offset = i * point_step
                
                try:
                    # Extract XYZ using struct.unpack_from for better performance
                    x = struct.unpack_from('<f', data_bytes, offset + x_offset)[0]
                    y = struct.unpack_from('<f', data_bytes, offset + y_offset)[0]
                    z = struct.unpack_from('<f', data_bytes, offset + z_offset)[0]
                    
                    # Keep the coordinate system flips you added
                    points.extend([float(-x), float(-y), float(z)])
                    
                    # Extract separate R, G, B bytes (UINT8 fields)
                    r = data_bytes[offset + r_off]
                    g = data_bytes[offset + g_off]
                    b = data_bytes[offset + b_off]
                    colors.extend([r/255.0, g/255.0, b/255.0])  # Normalize to 0-1
                    
                except (struct.error, IndexError) as e:
                    self.get_logger().warn(f"Failed to unpack point {i}: {e}")
                    continue
            
            # Debug: Log color information
            if len(colors) > 0:
                colors_array = np.array(colors).reshape(-1, 3)
                self.get_logger().info(
                    f"🎨 WebSocket RGB Debug - Transmitting colors: "
                    f"R({colors_array[:,0].min():.3f}-{colors_array[:,0].max():.3f}), "
                    f"G({colors_array[:,1].min():.3f}-{colors_array[:,1].max():.3f}), "
                    f"B({colors_array[:,2].min():.3f}-{colors_array[:,2].max():.3f}) "
                    f"[{len(colors)//3} points]"
                )
            
            return {
                'points': points,
                'colors': colors,
                'num_points': len(points) // 3,
                'timestamp': time.time()
            }
        except Exception as e:
            self.get_logger().error(f"Error in pointcloud2_to_json: {e}")
            traceback.print_exc()
            return {'points': [], 'colors': [], 'num_points': 0, 'timestamp': time.time()}

    
    def get_latest_data(self, data_type):
        with self._lock:
            if data_type == '2d':
                return self._latest_2d_frame
            elif data_type == '3d':
                return self._latest_pointcloud
            elif data_type == 'full_scene':
                return self._latest_full_scene
            elif data_type == 'metadata':
                return self._latest_metadata
            elif data_type == 'center':
                return self._latest_center_point
            elif data_type == 'slam_trajectory':
                # Return trajectory as JSON
                return json.dumps(self._latest_slam_trajectory).encode('utf-8')
            elif data_type == 'slam_pose':
                return self._latest_slam_pose
            elif data_type == 'slam_landmarks':
                return self._latest_slam_landmarks
        return None
    
    def get_latest_hash(self, data_type):
        """Get hash of latest data to detect changes"""
        with self._lock:
            if data_type == '2d':
                return self._last_2d_hash
            elif data_type == '3d':
                return self._last_3d_hash
            elif data_type == 'full_scene':
                return self._last_full_scene_hash
        return None

    def has_data(self, data_type):
        """Check if we have recent data"""
        now = time.time()
        with self._lock:
            if data_type == '2d':
                return self._latest_2d_frame is not None and (now - self._last_2d_time) < 10
            elif data_type == '3d':
                return self._latest_pointcloud is not None and (now - self._last_3d_time) < 10
            elif data_type == 'full_scene':
                return self._latest_full_scene is not None and (now - self._last_full_scene_time) < 10
            elif data_type == 'metadata':
                return self._latest_metadata is not None and (now - self._last_meta_time) < 10
            elif data_type == 'center':
                return self._latest_center_point is not None and (now - self._last_center_time) < 10
            elif data_type == 'slam_trajectory':
                return len(self._latest_slam_trajectory) > 0 and (now - self._last_slam_trajectory_time) < 10
            elif data_type == 'slam_pose':
                return self._latest_slam_pose is not None and (now - self._last_slam_pose_time) < 10
            elif data_type == 'slam_landmarks':
                return self._latest_slam_landmarks is not None and (now - self._last_slam_landmarks_time) < 10
        return False

node_ref = None

async def stream_handler(websocket, path=None):
    """Handle WebSocket connections with better error handling"""
    
    # Extract path from websocket if not provided as parameter
    if path is None:
        try:
            path = websocket.path
        except AttributeError:
            # For newer websockets library versions
            path = websocket.request.path if hasattr(websocket, 'request') else '/'
    
    print(f"🔌 Client connected to {path}")
    
    try:
        # Determine what type of data to stream based on path
        if path == '/2d':
            data_type = '2d'
        elif path == '/3d':
            data_type = '3d'
        elif path == '/full_scene':
            data_type = 'full_scene'
        elif path == '/metadata':
            data_type = 'metadata'
        elif path == '/center':
            data_type = 'center'
        elif path == '/slam_trajectory':
            data_type = 'slam_trajectory'
        elif path == '/slam_pose':
            data_type = 'slam_pose'
        elif path == '/slam_landmarks':
            data_type = 'slam_landmarks'
        else:
            await websocket.send("Invalid path. Use /2d, /3d, /full_scene, /metadata, /center, /slam_trajectory, /slam_pose, or /slam_landmarks")
            return

        # Send initial status
        if node_ref and not node_ref.has_data(data_type):
            await websocket.send(f"Waiting for {data_type} data...".encode('utf-8'))
            print(f"⏳ Waiting for {data_type} data...")

        frame_count = 0
        last_hash = None
        while True:
            try:
                data = node_ref.get_latest_data(data_type) if node_ref else None
                current_hash = node_ref.get_latest_hash(data_type) if node_ref else None
                
                # Only send if data changed (for binary types) or always send for text types
                if data:
                    should_send = True
                    if data_type in ['2d', '3d', 'full_scene'] and current_hash:
                        should_send = (current_hash != last_hash)
                        if should_send:
                            last_hash = current_hash
                    
                    if should_send:
                        await websocket.send(data)
                        frame_count += 1
                        if frame_count % 20 == 0:  # Log every 20 frames
                            print(f"📤 Sent {frame_count} {data_type} frames to client")
                
                # Adjust sleep based on data type (2D faster, 3D slower)
                if data_type == '2d':
                    await asyncio.sleep(0.05)  # 20 FPS for 2D (faster)
                elif data_type in ['metadata', 'center', 'slam_pose']:
                    await asyncio.sleep(0.05)  # 20 FPS for metadata/pose
                elif data_type in ['slam_trajectory', 'slam_landmarks']:
                    await asyncio.sleep(0.2)  # 5 FPS for trajectory/landmarks (less frequent updates)
                else:
                    await asyncio.sleep(0.1)  # 10 FPS for 3D (slower to reduce load)
                
            except websockets.exceptions.ConnectionClosed:
                print(f"🔌 Client disconnected from {path}")
                break
            except Exception as e:
                print(f"❌ Send error on {path}: {e}")
                traceback.print_exc()
                break
            
    except Exception as e:
        print(f"❌ Stream handler error on {path}: {e}")
        traceback.print_exc()
    finally:
        print(f"✅ Stream handler done for {path}")

async def main_ws_server():
    print("Starting Enhanced WebSocket server on port 8765...")
    print("Available endpoints:")
    print("  ws://localhost:8765/2d - 2D detection images")
    print("  ws://localhost:8765/3d - 3D point cloud data")
    print("  ws://localhost:8765/full_scene - Full scene point cloud data")
    print("  ws://localhost:8765/metadata - Detection metadata")
    print("  ws://localhost:8765/center - 3D center point data")
    print("  ws://localhost:8765/slam_trajectory - SLAM robot trajectory")
    print("  ws://localhost:8765/slam_pose - SLAM current pose")
    print("  ws://localhost:8765/slam_landmarks - SLAM map landmarks")
    
    try:
        server = await websockets.serve(stream_handler, "0.0.0.0", 8765)
        print("✅ WebSocket server running!")
        await server.wait_closed()
    except Exception as e:
        print(f"❌ Server error: {e}")
        traceback.print_exc()

def spin_ros():
    try:
        print("🔄 Starting ROS spin...")
        rclpy.spin(node_ref)
    except Exception as e:
        print(f"❌ ROS spin error: {e}")
        traceback.print_exc()

def main():
    global node_ref
    
    try:
        print("🚀 Starting Enhanced 3D Viewer...")
        rclpy.init()
        node_ref = Enhanced3DViewer()

        ros_thread = threading.Thread(target=spin_ros, daemon=True)
        ros_thread.start()
        
        print("⏳ Waiting 3 seconds for ROS to initialize...")
        time.sleep(3)

        # Run the websocket server
        asyncio.run(main_ws_server())
        
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    except Exception as e:
        print(f"❌ Main error: {e}")
        traceback.print_exc()
    finally:
        if node_ref:
            node_ref.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()