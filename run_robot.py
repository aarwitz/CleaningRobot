#!/usr/bin/env python3
"""run_robot.py

Intelligent automatic robot control script that listens to ROS detection topics
and continuously picks up detected objects using smart selection logic.

The script collects detection poses (up to 100) with confidence scores, then:
1. Filters for detections from the last 5 seconds with confidence > 0.3
2. Selects the top 20 highest confidence detections  
3. From those, picks the one closest to the robot's current position
4. This minimizes robot movement and ensures high-quality picks

Usage:
python run_robot.py /dev/ttyUSB0                    # Run with robot
python run_robot.py /dev/ttyUSB0 --dry-run         # Simulate only
python run_robot.py /dev/ttyUSB0 --once            # Pick once and exit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time
import argparse
import json
import threading
import queue
import math
from collections import deque

try:
    import serial
except Exception:
    serial = None


def realsense_to_robot_coords(rs_x, rs_y, rs_z):
    """
    Convert RealSense coordinates to robot coordinates using calibration math
    
    Args:
        rs_x, rs_y, rs_z: RealSense coordinates in meters
    
    Returns:
        dict with 'above_pick' and 'just_above_pick' robot coordinates in mm
    """

    # Above pick point (100mm higher)
    above_x = (1000 * rs_z) + 100
    above_y = (-1 * 1000 * rs_x)
    above_z = (-1 * 1000 * rs_y) + 130
    
    # Just above pick point (touching ball level)
    just_above_x = (1000 * rs_z) + 115
    just_above_y = (-1 * 1000 * rs_x) + 25
    just_above_z = (-1 * 1000 * rs_y) - 40
    
    return {
        'above_pick': {'x': above_x, 'y': above_y, 'z': above_z},
        'just_above_pick': {'x': just_above_x, 'y': just_above_y, 'z': just_above_z},
        'pick_point': {'x': just_above_x, 'y': just_above_y, 'z': just_above_z - 115},
    }

def euclidean_distance(pos1, pos2):
    """Calculate 3D euclidean distance between two positions"""
    return math.sqrt(
        (pos1['x'] - pos2['x'])**2 + 
        (pos1['y'] - pos2['y'])**2 + 
        (pos1['z'] - pos2['z'])**2
    )

def select_best_detection(detections, current_robot_pos=None, top_n=20):
    """
    Select best detection from history using confidence and distance.
    
    Args:
        detections: List of detection dictionaries with x,y,z,confidence,timestamp
        current_robot_pos: Current robot position dict with x,y,z (in robot coords)
        top_n: Number of top confidence detections to consider
    
    Returns:
        Best detection dict or None if no good candidates
    """
    if not detections:
        return None
    
    # Filter recent detections (last 5 seconds)
    current_time = time.time()
    recent_detections = [
        d for d in detections 
        if current_time - d['timestamp'] < 5.0 and d['confidence'] > 0.3
    ]
    
    if not recent_detections:
        return None
    
    # Sort by confidence and take top N
    top_confident = sorted(recent_detections, key=lambda d: d['confidence'], reverse=True)[:top_n]
    
    if not current_robot_pos:
        # If no robot position known, just return highest confidence
        return top_confident[0]
    
    # Convert detections to robot coordinates and calculate distances
    candidates = []
    for detection in top_confident:
        robot_coords = realsense_to_robot_coords(detection['x'], detection['y'], detection['z'])
        target_pos = robot_coords['above_pick']  # Use above_pick as reference point
        
        distance = euclidean_distance(target_pos, current_robot_pos)
        
        candidates.append({
            'detection': detection,
            'robot_coords': robot_coords,
            'distance': distance,
            'score': detection['confidence'] * (1.0 / (1.0 + distance/1000.0))  # Combine confidence and distance
        })
    
    # Sort by combined score (higher is better)
    candidates.sort(key=lambda c: c['score'], reverse=True)
    
    return candidates[0]['detection'] if candidates else None

def make_robot_sequence(rs_x, rs_y, rs_z):
    """Generate robot pick sequence from RealSense coordinates"""
    
    coords = realsense_to_robot_coords(rs_x, rs_y, rs_z)
    above = coords['above_pick']
    just_above = coords['just_above_pick']
    pick_point = coords['pick_point']

    return [
        # 1. Move to above pick point
        ({"T": 104, "x": above['x'], "y": above['y'], "z": above['z'], "t": 0, "spd": 0.25}, 2.0),
        
        # 2. Move to just above pick point (about to touch)
        ({"T": 104, "x": just_above['x'], "y": just_above['y'], "z": just_above['z'], "t": 0, "spd": 0.25}, 2.0),
        
        # 3. Move to pick point actual (close gripper)
        ({"T": 104, "x": pick_point['x'], "y": pick_point['y'], "z": pick_point['z'], "t": 3.14, "spd": 0.25}, 4.0),
        
        # 4. Move back to above pick point with gripper closed
        ({"T": 104, "x": above['x'], "y": above['y'], "z": above['z'], "t": 3.14, "spd": 0.25}, 3.0),

        # 5. Stay above pick point but release sock out of gripper
        ({"T": 104, "x": above['x'], "y": above['y'], "z": above['z'], "t": 0, "spd": 0.25}, 4.0),
    ], coords  # Also return coords for position tracking


class RobotController(Node):
    def __init__(self, port, dry_run=False, pick_once=False):
        super().__init__('robot_controller')
        
        self.port = port
        self.dry_run = dry_run
        self.pick_once = pick_once
        self.ser = None
        
        # Store detection history with confidence scores
        self.detection_history = deque()  # Keep last 100 detections
        self.max_history_size = 100
        self.current_robot_pos = None  # Track robot position for distance calculations
        
        # Subscribers
        self.point_sub = self.create_subscription(
            PointStamped, 
            'detected_objects_3d', 
            self.point_callback, 
            10
        )
        
        self.metadata_sub = self.create_subscription(
            String,
            'detection_metadata',
            self.metadata_callback,
            10
        )
        
        # Status tracking
        self.last_detection_time = 0
        self.robot_busy = False
        self.pending_metadata = {}  # Store metadata until we get the corresponding point
        
        # Collection parameters
        self.collection_window = 3.0  # Collect detections for 3 seconds
        self.min_detections_for_pick = 10  # Minimum detections before considering a pick
        
        # Initialize serial connection
        if not self.dry_run:
            self.init_serial()
        
        self.get_logger().info(f'🚀 Robot Controller Started!')
        self.get_logger().info(f'📡 Port: {port}, Dry-run: {dry_run}, Pick-once: {pick_once}')
        self.get_logger().info('🎯 Waiting for object detections...')

    def init_serial(self):
        """Initialize serial connection to robot"""
        try:
            if serial is None:
                raise RuntimeError("pyserial required: pip install pyserial")
            
            self.ser = serial.Serial(self.port, baudrate=115200, dsrdtr=None)
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            time.sleep(0.1)
            self.get_logger().info(f"🔌 Connected to robot on {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to robot: {e}")
            raise

    def point_callback(self, msg):
        """Callback for 3D point detections"""
        try:
            x = msg.point.x
            y = msg.point.y 
            z = msg.point.z
            timestamp = time.time()
            
            # Look for matching metadata (use timestamp proximity)
            confidence = 0.5  # Default confidence
            class_name = "unknown"
            
            # Find closest metadata by timestamp (within 0.1 seconds)
            best_match = None
            best_time_diff = float('inf')
            
            for ts, metadata in list(self.pending_metadata.items()):
                time_diff = abs(timestamp - ts)
                if time_diff < 0.1 and time_diff < best_time_diff:
                    best_match = (ts, metadata)
                    best_time_diff = time_diff
            
            if best_match:
                ts, metadata = best_match
                confidence = metadata.get('confidence', 0.5)
                class_name = metadata.get('class_name', 'unknown')
                # Remove used metadata
                del self.pending_metadata[ts]
            
            # Store detection with all info
            detection = {
                'x': x, 'y': y, 'z': z,
                'confidence': confidence,
                'class_name': class_name,
                'timestamp': timestamp
            }
            
            self.detection_history.append(detection)
            
            # Maintain max history size manually
            if len(self.detection_history) > self.max_history_size:
                self.detection_history.popleft()
            
            self.last_detection_time = timestamp
            
            self.get_logger().info(f'🎯 Detection: {class_name} ({x:.3f}, {y:.3f}, {z:.3f}) conf={confidence:.3f}')
            
            # Start robot sequence if not busy and we have enough detections
            if not self.robot_busy and len(self.detection_history) >= self.min_detections_for_pick:
                self.start_robot_sequence()
                
        except Exception as e:
            self.get_logger().error(f'❌ Error in point callback: {e}')

    def metadata_callback(self, msg):
        """Callback for detection metadata (for additional info)"""
        try:
            data = json.loads(msg.data)
            timestamp = time.time()
            
            if data:
                # Store metadata temporarily until matching point arrives
                for detection in data:
                    self.pending_metadata[timestamp] = {
                        'confidence': detection.get('confidence', 0.5),
                        'class_name': detection.get('class_name', 'unknown'),
                        'graspable': detection.get('graspable', False)
                    }
                    
                # Clean up old metadata (older than 1 second)
                cutoff_time = timestamp - 1.0
                old_keys = [ts for ts in self.pending_metadata.keys() if ts < cutoff_time]
                for old_key in old_keys:
                    del self.pending_metadata[old_key]
                    
        except Exception as e:
            self.get_logger().debug(f'Metadata parse error: {e}')

    def start_robot_sequence(self):
        """Start robot pick sequence in separate thread"""
        if self.robot_busy:
            return
            
        try:
            # Select best detection from history
            best_detection = select_best_detection(
                list(self.detection_history), 
                self.current_robot_pos, 
                top_n=20
            )
            
            if not best_detection:
                self.get_logger().debug('No suitable detections for robot sequence')
                return
            
            self.get_logger().info(
                f'🎯 Selected best target: {best_detection["class_name"]} '
                f'conf={best_detection["confidence"]:.3f} '
                f'pos=({best_detection["x"]:.3f}, {best_detection["y"]:.3f}, {best_detection["z"]:.3f})'
            )
            
            # Start robot sequence in background thread
            robot_thread = threading.Thread(
                target=self.execute_robot_sequence,
                args=(best_detection["x"], best_detection["y"], best_detection["z"]),
                daemon=True
            )
            robot_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'❌ Error starting robot sequence: {e}')

    def execute_robot_sequence(self, rs_x, rs_y, rs_z):
        """Execute complete robot pick sequence"""
        self.robot_busy = True
        
        try:
            self.get_logger().info(f'🤖 Starting robot sequence for ({rs_x:.3f}, {rs_y:.3f}, {rs_z:.3f})')
            
            # Generate robot sequence and get coordinates
            sequence, coords = make_robot_sequence(rs_x, rs_y, rs_z)
            
            # Update current robot position tracking
            self.current_robot_pos = coords['above_pick'].copy()
            
            # Execute sequence
            self.send_sequence(sequence)
            
            self.get_logger().info('✅ Robot sequence completed!')
            
            # Clear old detections after successful pick (keep last 10)
            if len(self.detection_history) > 10:
                # Keep only the most recent detections
                recent_detections = list(self.detection_history)[-10:]
                self.detection_history.clear()
                self.detection_history.extend(recent_detections)
            
            # If pick_once mode, shutdown
            if self.pick_once:
                self.get_logger().info('🛑 Pick-once mode: shutting down')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'❌ Robot sequence failed: {e}')
        finally:
            self.robot_busy = False

    def send_sequence(self, sequence):
        """Send command sequence to robot"""
        if self.dry_run:
            self.get_logger().info("🧪 Dry-run mode: printing commands only")
        
        try:
            for i, (msg, delay) in enumerate(sequence, start=1):
                line = json.dumps(msg, separators=(',', ':'))
                self.get_logger().info(f"Step {i}/{len(sequence)}: {line} (delay {delay}s)")
                
                if not self.dry_run and self.ser:
                    self.ser.write(line.encode('utf-8') + b"\n")
                
                time.sleep(delay)
                
        except Exception as e:
            self.get_logger().error(f'❌ Send sequence error: {e}')

    def cleanup(self):
        """Clean up resources"""
        if self.ser:
            self.ser.close()
            self.get_logger().info('🔌 Serial connection closed')


def main():
    parser = argparse.ArgumentParser(description='Automatic robot controller for object picking')
    parser.add_argument('port', type=str, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--dry-run', action='store_true', help='Print commands instead of sending')
    parser.add_argument('--once', action='store_true', help='Pick once and exit')
    
    args = parser.parse_args()
    
    # Initialize ROS
    rclpy.init()
    
    controller = None
    try:
        # Create robot controller
        controller = RobotController(args.port, dry_run=args.dry_run, pick_once=args.once)
        
        # Start spinning
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print('\n🛑 Shutting down robot controller...')
    except Exception as e:
        print(f'❌ Error: {e}')
    finally:
        if controller:
            controller.cleanup()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()