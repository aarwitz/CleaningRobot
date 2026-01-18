#!/usr/bin/env python3
"""
Arm Bridge Node

Provides service interfaces for Waveshare RoArm v2 manipulation.
Uses serial communication and hand-eye calibration for pick/place operations.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PointStamped
import json
import time
import threading

try:
    import serial
except ImportError:
    serial = None


def realsense_to_robot_coords(rs_x, rs_y, rs_z):
    """
    Convert RealSense coordinates to robot arm coordinates using hand-eye calibration.
    
    Args:
        rs_x, rs_y, rs_z: RealSense coordinates in meters
    
    Returns:
        dict with 'above_pick', 'just_above_pick', and 'pick_point' in mm
    """
    # Above pick point (100mm higher)
    above_x = (1000 * rs_z) + 100
    above_y = (-1 * 1000 * rs_x)
    above_z = (-1 * 1000 * rs_y) + 130
    
    # Just above pick point (touching object level)
    just_above_x = (1000 * rs_z) + 115
    just_above_y = (-1 * 1000 * rs_x) + 25
    just_above_z = (-1 * 1000 * rs_y) - 40
    
    return {
        'above_pick': {'x': above_x, 'y': above_y, 'z': above_z},
        'just_above_pick': {'x': just_above_x, 'y': just_above_y, 'z': just_above_z},
        'pick_point': {'x': just_above_x, 'y': just_above_y, 'z': just_above_z - 115},
    }


class ArmBridgeNode(Node):
    """ROS2 node providing arm manipulation services"""
    
    def __init__(self):
        super().__init__('arm_bridge_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('enable_arm', True)  # Can disable for testing without arm
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.enable_arm = self.get_parameter('enable_arm').value
        
        # Serial connection
        self.ser = None
        self.ser_lock = threading.Lock()
        
        if self.enable_arm:
            self._init_serial()
        else:
            self.get_logger().info('Arm disabled (enable_arm=False)')
        
        # Create services
        self.pick_srv = self.create_service(
            Trigger,
            '/arm/pick_at',
            self.pick_callback
        )
        
        self.place_srv = self.create_service(
            Trigger,
            '/arm/place_at',
            self.place_callback
        )
        
        # Subscribe to target point for pick operations
        self.target_sub = self.create_subscription(
            PointStamped,
            '/clothes/target_point_camera',
            self.target_callback,
            10
        )
        
        self.last_target = None
        
        self.get_logger().info(f'Arm bridge node started (port: {self.serial_port})')

    def _init_serial(self):
        """Initialize serial connection to arm"""
        if serial is None:
            self.get_logger().error('pyserial not available, arm will not function')
            return
            
        try:
            self.ser = serial.Serial(
                self.serial_port,
                baudrate=self.baud_rate,
                dsrdtr=None
            )
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            time.sleep(0.1)
            self.get_logger().info(f'Connected to arm on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to arm: {e}')
            self.ser = None

    def _send_arm_command(self, cmd_dict, delay=0.0):
        """
        Send JSON command to arm and wait for delay.
        
        Args:
            cmd_dict: Dictionary with arm command (T, x, y, z, t, spd)
            delay: Time to wait after sending command
        """
        if not self.ser:
            self.get_logger().debug(f'Arm disabled, would send: {cmd_dict}')
            time.sleep(delay)
            return True
        
        try:
            with self.ser_lock:
                line = json.dumps(cmd_dict, separators=(',', ':'))
                self.ser.write(line.encode('utf-8') + b"\n")
                self.get_logger().debug(f'Sent: {line}')
            time.sleep(delay)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to send arm command: {e}')
            return False

    def _execute_pick_sequence(self, rs_x, rs_y, rs_z):
        """
        Execute complete pick sequence for target at RealSense coordinates.
        
        Args:
            rs_x, rs_y, rs_z: Target position in camera frame (meters)
        
        Returns:
            bool: True if successful
        """
        coords = realsense_to_robot_coords(rs_x, rs_y, rs_z)
        above = coords['above_pick']
        just_above = coords['just_above_pick']
        pick_point = coords['pick_point']
        
        self.get_logger().info(f'Pick sequence for camera coords ({rs_x:.3f}, {rs_y:.3f}, {rs_z:.3f})')
        self.get_logger().info(f'Robot coords: above=({above["x"]:.1f}, {above["y"]:.1f}, {above["z"]:.1f})')
        
        # Sequence: move above → move to touch → close gripper → lift
        sequence = [
            ({'T': 104, 'x': above['x'], 'y': above['y'], 'z': above['z'], 't': 0, 'spd': 0.25}, 2.0),
            ({'T': 104, 'x': just_above['x'], 'y': just_above['y'], 'z': just_above['z'], 't': 0, 'spd': 0.25}, 2.0),
            ({'T': 104, 'x': pick_point['x'], 'y': pick_point['y'], 'z': pick_point['z'], 't': 3.14, 'spd': 0.25}, 4.0),
            ({'T': 104, 'x': above['x'], 'y': above['y'], 'z': above['z'], 't': 3.14, 'spd': 0.25}, 3.0),
        ]
        
        for i, (cmd, delay) in enumerate(sequence, 1):
            if not self._send_arm_command(cmd, delay):
                self.get_logger().error(f'Pick sequence failed at step {i}')
                return False
        
        return True

    def _execute_place_sequence(self):
        """
        Execute place sequence (open gripper to drop object).
        
        Returns:
            bool: True if successful
        """
        self.get_logger().info('Executing place sequence')
        
        # Open gripper (t=0) at current position
        # The arm should already be at a safe position from previous command
        return self._send_arm_command({'T': 104, 't': 0, 'spd': 0.25}, 4.0)

    def target_callback(self, msg):
        """Store latest target point"""
        self.last_target = msg
        self.get_logger().debug(f'Target updated: ({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})')

    def pick_callback(self, request, response):
        """Handle pick service request"""
        if self.last_target is None:
            response.success = False
            response.message = 'No target point available'
            self.get_logger().warn('Pick requested but no target available')
            return response
        
        # Execute pick sequence
        success = self._execute_pick_sequence(
            self.last_target.point.x,
            self.last_target.point.y,
            self.last_target.point.z
        )
        
        response.success = success
        if success:
            response.message = f'Picked at camera ({self.last_target.point.x:.3f}, {self.last_target.point.y:.3f}, {self.last_target.point.z:.3f})'
            self.get_logger().info(f'Pick successful: {response.message}')
        else:
            response.message = 'Pick sequence failed'
            self.get_logger().error(response.message)
        
        return response

    def place_callback(self, request, response):
        """Handle place service request"""
        success = self._execute_place_sequence()
        
        response.success = success
        if success:
            response.message = 'Object placed (gripper opened)'
            self.get_logger().info(response.message)
        else:
            response.message = 'Place sequence failed'
            self.get_logger().error(response.message)
        
        return response

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.ser:
            try:
                # Open gripper on shutdown (safe state)
                self._send_arm_command({'T': 104, 't': 0, 'spd': 0.25}, 1.0)
                self.ser.close()
                self.get_logger().info('Serial connection closed')
            except Exception as e:
                self.get_logger().error(f'Error closing serial: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
