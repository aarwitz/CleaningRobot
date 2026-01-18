#!/usr/bin/env python3
"""
Motor Controller Node

Subscribes to /cmd_vel and implements velocity PID control 
for I2C motor driver (address 0x34 on bus 7).

Based on nav2_compatible_velocity_controller.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import smbus
import struct
import time
import threading
from enum import Enum


class State(Enum):
    IDLE = 0
    RUNNING = 1
    STOPPING = 2


class SidePID:
    """PID controller for one wheel side"""
    def __init__(self, kp, ki, i_clamp):
        self.kp = kp
        self.ki = ki
        self.i_clamp = i_clamp
        self.i = 0.0

    def reset(self):
        self.i = 0.0

    def update(self, error, dt):
        self.i += error * dt
        self.i = max(-self.i_clamp, min(self.i_clamp, self.i))
        return self.kp * error + self.ki * self.i


class MotorControllerNode(Node):
    """ROS2 node for velocity control of differential drive robot"""
    
    def __init__(self):
        super().__init__('motor_controller_node')
        
        # Declare parameters
        self.declare_parameter('bus_id', 7)
        self.declare_parameter('i2c_addr', 0x34)
        self.declare_parameter('ticks_per_meter', 18940.0)
        self.declare_parameter('track_width', 0.256)
        self.declare_parameter('cmd_per_mps', 240.0)
        self.declare_parameter('min_cmd', 20)
        self.declare_parameter('max_cmd', 95)
        self.declare_parameter('max_step', 5)
        self.declare_parameter('kp', 6.0)
        self.declare_parameter('ki', 1.5)
        self.declare_parameter('i_clamp', 15.0)
        self.declare_parameter('vel_alpha', 0.3)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_ticks_per_sec', 40000.0)
        self.declare_parameter('cmd_timeout', 0.5)  # Stop if no cmd_vel for 0.5s
        
        # Get parameters
        self.bus_id = self.get_parameter('bus_id').value
        self.i2c_addr = self.get_parameter('i2c_addr').value
        self.ticks_per_meter = self.get_parameter('ticks_per_meter').value
        self.track_width = self.get_parameter('track_width').value
        self.cmd_per_mps = self.get_parameter('cmd_per_mps').value
        self.min_cmd = self.get_parameter('min_cmd').value
        self.max_cmd = self.get_parameter('max_cmd').value
        self.max_step = self.get_parameter('max_step').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.i_clamp = self.get_parameter('i_clamp').value
        self.vel_alpha = self.get_parameter('vel_alpha').value
        self.control_rate = self.get_parameter('control_rate').value
        self.max_ticks_per_sec = self.get_parameter('max_ticks_per_sec').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        
        self.dt = 1.0 / self.control_rate
        
        # Initialize I2C
        try:
            self.bus = smbus.SMBus(self.bus_id)
            self._i2c_retry(self.bus.write_byte_data, self.i2c_addr, 0x14, 1)
            self._i2c_retry(self.bus.write_byte_data, self.i2c_addr, 0x15, 0)
            self.get_logger().info(f'I2C initialized on bus {self.bus_id}, addr 0x{self.i2c_addr:02x}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize I2C: {e}')
            self.bus = None
        
        # State
        self.state = State.IDLE
        self.vL_ref = 0.0
        self.vR_ref = 0.0
        self.last_cmd_time = self.get_clock().now()
        
        # PID controllers
        self.pid_L = SidePID(self.kp, self.ki, self.i_clamp)
        self.pid_R = SidePID(self.kp, self.ki, self.i_clamp)
        
        # Encoder state
        self.enc_last = [0, 0, 0, 0]
        self.t_last = time.monotonic()
        
        # Velocity state
        self.vL_f = 0.0
        self.vR_f = 0.0
        self.uL_prev = 0.0
        self.uR_prev = 0.0
        
        # Create subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        
        # Create service for emergency stop
        self.estop_srv = self.create_service(Trigger, 'motor_controller/emergency_stop', self.estop_callback)
        
        # Control loop timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # Initialize encoders
        if self.bus:
            try:
                self.enc_last = self._read_encoders()
                self.t_last = time.monotonic()
            except Exception as e:
                self.get_logger().warn(f'Failed to read initial encoders: {e}')
        
        self.get_logger().info('Motor controller node started')

    def _i2c_retry(self, func, *args, retries=5, delay=0.01):
        """Retry I2C operations on failure"""
        for i in range(retries):
            try:
                return func(*args)
            except OSError:
                if i == retries - 1:
                    raise
                time.sleep(delay)

    def _read_encoders(self):
        """Read all 4 encoder values"""
        if not self.bus:
            return [0, 0, 0, 0]
        raw = self._i2c_retry(self.bus.read_i2c_block_data, self.i2c_addr, 0x3C, 16)
        return list(struct.unpack('<iiii', bytes(raw)))

    def _set_motors(self, left, right):
        """Set motor commands (left and right)"""
        if not self.bus:
            return
        self._i2c_retry(
            self.bus.write_i2c_block_data,
            self.i2c_addr,
            0x33,
            [right, right, left, left]
        )

    def _clamp_cmd(self, u):
        """Clamp command with deadzone"""
        if abs(u) < self.min_cmd:
            return 0
        return max(-self.max_cmd, min(self.max_cmd, int(u)))

    def _rate_limit(self, new, old):
        """Rate limit command changes"""
        return max(old - self.max_step, min(old + self.max_step, new))

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        # Convert twist to wheel velocities
        v = msg.linear.x
        omega = msg.angular.z
        
        self.vL_ref = v - omega * self.track_width / 2.0
        self.vR_ref = v + omega * self.track_width / 2.0
        
        self.last_cmd_time = self.get_clock().now()
        
        if self.state == State.IDLE:
            self.state = State.RUNNING
            self.pid_L.reset()
            self.pid_R.reset()

    def estop_callback(self, request, response):
        """Emergency stop service"""
        self.get_logger().warn('Emergency stop requested!')
        self.state = State.STOPPING
        self.vL_ref = 0.0
        self.vR_ref = 0.0
        response.success = True
        response.message = 'Emergency stop activated'
        return response

    def control_loop(self):
        """Main control loop executed at control_rate Hz"""
        
        # Check for command timeout
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.cmd_timeout and self.state == State.RUNNING:
            self.get_logger().debug('Command timeout, stopping motors')
            self.vL_ref = 0.0
            self.vR_ref = 0.0
            self.state = State.IDLE
        
        # Read encoders
        try:
            enc = self._read_encoders()
            t_now = time.monotonic()
            dt = t_now - self.t_last
            
            if dt <= 0:
                return
            
            # Calculate encoder deltas
            d = [enc[i] - self.enc_last[i] for i in range(4)]
            self.enc_last = enc
            self.t_last = t_now
            
            # Velocity sanity check
            for i in range(4):
                ticks_per_sec = abs(d[i] / dt)
                if ticks_per_sec > self.max_ticks_per_sec:
                    self.get_logger().error(f'ENCODER RATE FAULT motor={i} rate={ticks_per_sec:.0f} ticks/s')
                    self.state = State.STOPPING
                    break
            
            # Velocity estimation (right motors = 0,1, left motors = 2,3)
            vR_raw = ((d[0] + d[1]) * 0.5) / self.ticks_per_meter / dt
            vL_raw = ((d[2] + d[3]) * 0.5) / self.ticks_per_meter / dt
            
            # Low-pass filter
            self.vR_f = self.vel_alpha * vR_raw + (1 - self.vel_alpha) * self.vR_f
            self.vL_f = self.vel_alpha * vL_raw + (1 - self.vel_alpha) * self.vL_f
            
            # Publish odometry
            self._publish_odometry(self.vL_f, self.vR_f)
            
            # Control
            if self.state == State.RUNNING:
                # Calculate errors
                eL = self.vL_ref - self.vL_f
                eR = self.vR_ref - self.vR_f
                
                # Feedforward + PID
                u_ff_L = self.cmd_per_mps * self.vL_ref
                u_ff_R = self.cmd_per_mps * self.vR_ref
                
                uL = u_ff_L + self.pid_L.update(eL, dt)
                uR = u_ff_R + self.pid_R.update(eR, dt)
                
                # Rate limiting
                uL = self._rate_limit(uL, self.uL_prev)
                uR = self._rate_limit(uR, self.uR_prev)
                
                self.uL_prev = uL
                self.uR_prev = uR
                
                self._set_motors(self._clamp_cmd(uL), self._clamp_cmd(uR))
                
            elif self.state == State.STOPPING:
                # Controlled stop
                self.uL_prev *= 0.6
                self.uR_prev *= 0.6
                self._set_motors(self._clamp_cmd(self.uL_prev), self._clamp_cmd(self.uR_prev))
                
                if abs(self.uL_prev) < 1 and abs(self.uR_prev) < 1:
                    self._set_motors(0, 0)
                    self.state = State.IDLE
                    self.pid_L.reset()
                    self.pid_R.reset()
                    
            else:  # IDLE
                self._set_motors(0, 0)
                
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

    def _publish_odometry(self, vL, vR):
        """Publish wheel odometry"""
        # Calculate robot velocity from wheel velocities
        v = (vL + vR) / 2.0
        omega = (vR - vL) / self.track_width
        
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom_msg)

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.bus:
            try:
                self._set_motors(0, 0)
                self.get_logger().info('Motors stopped')
            except Exception as e:
                self.get_logger().error(f'Error stopping motors: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
