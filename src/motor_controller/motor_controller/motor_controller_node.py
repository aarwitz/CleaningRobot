#!/usr/bin/env python3
"""
Motor Controller Node — 4-wheel MECANUM omnidirectional drive.

Subscribes to /cmd_vel (geometry_msgs/Twist) and drives a 4-channel I2C motor
driver (address 0x34 on bus 7, HiWonder driver) with per-wheel velocity PID.
Publishes /wheel_odom (twist only — SLAM owns the TF tree).

Channel / wheel / encoder map (reverse-engineered on hardware, 2026-06-25):

    channel  encoder  corner            positive cmd drives
    -------  -------  ----------------  -------------------
       0       0      FR (front-right)  forward
       1       1      RL (rear-left)    forward
       2       2      FL (front-left)   backward   (mirror-mounted)
       3       3      RR (rear-right)   backward   (mirror-mounted)

The I2C block write at register 0x33 takes [ch0, ch1, ch2, ch3]; encoders are
read as 4x int32 little-endian at register 0x3C in the SAME channel order.

Mecanum kinematics (ROS REP-103 body frame: x fwd, y left, z up CCW). With
w[i] the "forward-driving" wheel speed of corner i (positive => pushes robot
forward), L = lx + ly (half-wheelbase + half-track):

    w_ref[i] = vx*KX[i] + vy*KY[i] + omega*L*KW[i]
    KX = [ 1,  1,  1,  1]   # FR RL FL RR
    KY = [ 1,  1, -1, -1]
    KW = [ 1, -1, -1,  1]

The raw channel command applies SIGN[i] to convert forward-driving speed into
the channel's measured polarity (FL/RR are inverted):

    SIGN = [+1, +1, -1, -1]
    cmd[i] = SIGN[i] * (cmd_per_mps * w_ref[i] + pid_i)

Forward kinematics for odometry (w_meas[i] = SIGN[i] * enc_rate[i] in m/s):

    vx    = (w0 + w1 + w2 + w3) / 4
    vy    = (w0 + w1 - w2 - w3) / 4
    omega = (w0 - w1 - w2 + w3) / (4 * L)

All three primitives (forward / strafe-left / rotate-CCW) were verified
open-loop on hardware before this controller was written.

I2C CONTENTION (important): the HiWonder driver shares one I2C device for both
motor writes (reg 0x33) and encoder reads (reg 0x3C). Interleaving a read after
a write every control cycle (20 Hz) corrupts the bus — reads return garbage
(impossible 50+ m/s wheel speeds) and eventually time out (errno 110), killing
motion. So this controller runs OPEN-LOOP by default (feedforward only, NO
per-cycle encoder reads). Nav2 closes the loop via VSLAM odometry
(/visual_slam/tracking/odometry), not wheel odometry, so nothing downstream
needs the encoders. Set use_encoder_feedback:=true to re-enable per-wheel PID
(it tolerates corrupt reads by discarding implausible samples), but expect the
contention above unless the firmware/wiring is fixed.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import smbus
import struct
import time
from enum import Enum


# Channel-indexed kinematic constants. Index = motor-driver channel = encoder
# index. Order is [FR, RL, FL, RR].
KX = (1.0, 1.0, 1.0, 1.0)
KY = (1.0, 1.0, -1.0, -1.0)
KW = (1.0, -1.0, -1.0, 1.0)
SIGN = (1.0, 1.0, -1.0, -1.0)
CORNER = ('FR', 'RL', 'FL', 'RR')


class State(Enum):
    IDLE = 0
    RUNNING = 1
    STOPPING = 2


class WheelPID:
    """PI velocity controller for one wheel."""
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
    """ROS2 node for velocity control of a 4-wheel mecanum base."""

    def __init__(self):
        super().__init__('motor_controller_node')

        # --- Parameters ---
        self.declare_parameter('bus_id', 7)
        self.declare_parameter('i2c_addr', 0x34)
        self.declare_parameter('ticks_per_meter', 18940.0)
        # L = lx + ly (half-wheelbase + half-track), in meters. Scales omega.
        # Calibrate against SLAM yaw if rotation rate is off; ~0.25 for this
        # compact HiWonder chassis (old track_width was 0.256 => ly~0.128).
        self.declare_parameter('wheel_geom_L', 0.25)
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
        self.declare_parameter('cmd_timeout', 0.5)
        # Open-loop by default — see I2C CONTENTION note in the module docstring.
        self.declare_parameter('use_encoder_feedback', False)

        self.bus_id = self.get_parameter('bus_id').value
        self.i2c_addr = self.get_parameter('i2c_addr').value
        self.ticks_per_meter = self.get_parameter('ticks_per_meter').value
        self.L = self.get_parameter('wheel_geom_L').value
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
        self.use_encoder_feedback = self.get_parameter('use_encoder_feedback').value

        self.dt = 1.0 / self.control_rate

        # --- I2C init ---
        try:
            self.bus = smbus.SMBus(self.bus_id)
            self._i2c_retry(self.bus.write_byte_data, self.i2c_addr, 0x14, 1)
            self._i2c_retry(self.bus.write_byte_data, self.i2c_addr, 0x15, 0)
            self.get_logger().info(
                f'I2C initialized on bus {self.bus_id}, addr 0x{self.i2c_addr:02x}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize I2C: {e}')
            self.bus = None

        # --- State ---
        self.state = State.IDLE
        self.vx_ref = 0.0   # body forward (m/s)
        self.vy_ref = 0.0   # body left (m/s)
        self.wz_ref = 0.0   # body yaw (rad/s, CCW+)
        self.last_cmd_time = self.get_clock().now()

        # Per-wheel PID, measured speed filter, last command (channel order).
        self.pid = [WheelPID(self.kp, self.ki, self.i_clamp) for _ in range(4)]
        self.w_f = [0.0, 0.0, 0.0, 0.0]      # filtered measured wheel speeds
        self.u_prev = [0.0, 0.0, 0.0, 0.0]   # last raw commands (rate limiting)

        self.enc_last = [0, 0, 0, 0]
        self.t_last = time.monotonic()

        # --- ROS interfaces ---
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.estop_srv = self.create_service(
            Trigger, 'motor_controller/emergency_stop', self.estop_callback)
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        # Keep zero-writing for a short window after going idle, then go quiet to
        # minimize I2C traffic (motors latch the last command).
        self._idle_writes = 0

        if self.bus and self.use_encoder_feedback:
            try:
                self.enc_last = self._read_encoders()
                self.t_last = time.monotonic()
            except Exception as e:
                self.get_logger().warn(f'Failed to read initial encoders: {e}')

        mode = 'closed-loop (encoder PID)' if self.use_encoder_feedback else 'OPEN-LOOP (feedforward)'
        self.get_logger().info(f'Mecanum motor controller node started [{mode}]')

    # --- I2C helpers ---
    def _i2c_retry(self, func, *args, retries=5, delay=0.01):
        for i in range(retries):
            try:
                return func(*args)
            except OSError:
                if i == retries - 1:
                    raise
                time.sleep(delay)

    def _read_encoders(self):
        if not self.bus:
            return [0, 0, 0, 0]
        raw = self._i2c_retry(self.bus.read_i2c_block_data, self.i2c_addr, 0x3C, 16)
        return list(struct.unpack('<iiii', bytes(raw)))

    def _write_motors(self, cmd4):
        """Write raw signed commands to channels [0,1,2,3]."""
        if not self.bus:
            return
        block = [int(c) & 0xFF for c in cmd4]
        self._i2c_retry(self.bus.write_i2c_block_data, self.i2c_addr, 0x33, block)

    def _clamp_cmd(self, u):
        if abs(u) < self.min_cmd:
            return 0
        return max(-self.max_cmd, min(self.max_cmd, int(u)))

    def _rate_limit(self, new, old):
        return max(old - self.max_step, min(old + self.max_step, new))

    # --- Callbacks ---
    def cmd_vel_callback(self, msg):
        self.vx_ref = msg.linear.x
        self.vy_ref = msg.linear.y
        self.wz_ref = msg.angular.z
        self.last_cmd_time = self.get_clock().now()
        if self.state == State.IDLE:
            self.state = State.RUNNING
            for p in self.pid:
                p.reset()

    def estop_callback(self, request, response):
        self.get_logger().warn('Emergency stop requested!')
        self.state = State.STOPPING
        self.vx_ref = self.vy_ref = self.wz_ref = 0.0
        response.success = True
        response.message = 'Emergency stop activated'
        return response

    def _wheel_refs(self):
        """Forward-driving wheel speed targets (m/s) per channel."""
        return [
            self.vx_ref * KX[i] + self.vy_ref * KY[i] + self.wz_ref * self.L * KW[i]
            for i in range(4)
        ]

    # --- Control loop ---
    def control_loop(self):
        # Command timeout -> ramp to stop.
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.cmd_timeout and self.state == State.RUNNING:
            self.vx_ref = self.vy_ref = self.wz_ref = 0.0
            self.state = State.STOPPING

        try:
            dt = self._update_feedback()  # reads encoders only in closed-loop mode

            if self.state == State.RUNNING:
                w_ref = self._wheel_refs()
                cmd4 = [0, 0, 0, 0]
                for i in range(4):
                    u_ff = self.cmd_per_mps * w_ref[i]
                    if self.use_encoder_feedback and dt > 0:
                        u = u_ff + self.pid[i].update(w_ref[i] - self.w_f[i], dt)
                    else:
                        u = u_ff  # open-loop feedforward
                    u = self._rate_limit(u, self.u_prev[i])
                    self.u_prev[i] = u
                    cmd4[i] = self._clamp_cmd(SIGN[i] * u)
                self._write_motors(cmd4)
                self._idle_writes = 5
                self.get_logger().info(
                    f'cmd_vel(vx={self.vx_ref:.2f} vy={self.vy_ref:.2f} '
                    f'wz={self.wz_ref:.2f}) -> motors{cmd4}',
                    throttle_duration_sec=0.5)

            elif self.state == State.STOPPING:
                done = True
                cmd4 = [0, 0, 0, 0]
                for i in range(4):
                    self.u_prev[i] *= 0.6
                    cmd4[i] = self._clamp_cmd(SIGN[i] * self.u_prev[i])
                    if abs(self.u_prev[i]) >= 1:
                        done = False
                self._write_motors(cmd4)
                self._idle_writes = 5
                if done:
                    self.u_prev = [0.0, 0.0, 0.0, 0.0]
                    self._write_motors([0, 0, 0, 0])
                    self.state = State.IDLE
                    for p in self.pid:
                        p.reset()

            else:  # IDLE — write zeros a few times, then stay quiet on the bus.
                if self._idle_writes > 0:
                    self._write_motors([0, 0, 0, 0])
                    self._idle_writes -= 1

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

    def _update_feedback(self):
        """In closed-loop mode, read encoders and update filtered wheel speeds and
        odometry. Returns dt (>0) when a valid sample was processed, else 0.0.
        In open-loop mode does nothing (no I2C reads) and returns 0.0."""
        if not self.use_encoder_feedback:
            return 0.0
        enc = self._read_encoders()
        t_now = time.monotonic()
        dt = t_now - self.t_last
        if dt <= 0:
            return 0.0
        d = [enc[i] - self.enc_last[i] for i in range(4)]

        # Reject corrupt reads (I2C contention yields impossible deltas) instead
        # of faulting — discard the sample and wait for a clean one.
        if any(abs(d[i] / dt) > self.max_ticks_per_sec for i in range(4)):
            self.get_logger().warn(
                f'Discarding corrupt encoder read: deltas={d} dt={dt:.3f}',
                throttle_duration_sec=2.0)
            self.enc_last = enc
            self.t_last = t_now
            return 0.0

        self.enc_last = enc
        self.t_last = t_now
        w_meas = [SIGN[i] * (d[i] / self.ticks_per_meter) / dt for i in range(4)]
        for i in range(4):
            self.w_f[i] = (self.vel_alpha * w_meas[i]
                           + (1 - self.vel_alpha) * self.w_f[i])
        self._publish_odometry(self.w_f)
        return dt

    def _publish_odometry(self, w):
        """Publish omnidirectional wheel odometry (twist only)."""
        vx = (w[0] + w[1] + w[2] + w[3]) / 4.0
        vy = (w[0] + w[1] - w[2] - w[3]) / 4.0
        omega = (w[0] - w[1] - w[2] + w[3]) / (4.0 * self.L)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega
        self.odom_pub.publish(odom_msg)

    def destroy_node(self):
        if self.bus:
            try:
                self._write_motors([0, 0, 0, 0])
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
