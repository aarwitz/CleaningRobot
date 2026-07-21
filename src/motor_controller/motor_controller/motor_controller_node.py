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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from behavior_manager_interfaces.srv import DriveRelative
import smbus
import struct
import time
import math
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

        # --- DriveRelative (closed-loop relative move) parameters ---
        # The relative-move service drives a bounded body-frame displacement using
        # encoder feedback in a move->pause->read loop. Reads are safe because the
        # service and the control loop share one mutually-exclusive callback group
        # (they never run at the same time), so an encoder read never races a motor
        # write. Slip factors map wheel-odometry to true ground motion (calibrated
        # on hardware 2026-06-28): forward has no roller slip; strafe/yaw do.
        self.declare_parameter('k_strafe', 0.89)        # ground/odom for strafe
        self.declare_parameter('k_yaw', 0.97)           # ground/odom for yaw
        self.declare_parameter('move_lin_speed', 0.14)  # m/s burst (>deadband)
        # 0.6 rad/s bricks the HiWonder driver (board falls off the I2C bus on
        # the NEXT transaction); 0.4 ran 23 consecutive turns clean 2026-07-02.
        self.declare_parameter('move_yaw_speed', 0.4)   # rad/s burst
        self.declare_parameter('move_tol_lin', 0.015)   # m
        self.declare_parameter('move_tol_yaw', 0.052)   # rad (~3 deg)
        self.declare_parameter('move_undershoot', 0.85) # approach from below
        self.declare_parameter('move_burst_min', 0.20)  # s (clear deadband ramp)
        self.declare_parameter('move_burst_max', 1.00)  # s
        # Vector (fwd+strafe) moves: faster and much longer single bursts so a
        # multi-meter path segment is one continuous glide, not hop-stutter
        self.declare_parameter('move_vec_speed', 0.20)      # m/s command
        self.declare_parameter('move_burst_max_vec', 8.0)   # s
        self.declare_parameter('move_max_iters', 10)
        self.declare_parameter('move_max_lin', 2.0)     # per-call bound, m
        self.declare_parameter('move_max_yaw', 1.57)    # per-call bound, rad
        self.declare_parameter('move_settle_sec', 0.9)  # idle wait before read

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

        self.k_strafe = self.get_parameter('k_strafe').value
        self.k_yaw = self.get_parameter('k_yaw').value
        self.move_lin_speed = self.get_parameter('move_lin_speed').value
        self.move_yaw_speed = self.get_parameter('move_yaw_speed').value
        self.move_tol_lin = self.get_parameter('move_tol_lin').value
        self.move_tol_yaw = self.get_parameter('move_tol_yaw').value
        self.move_undershoot = self.get_parameter('move_undershoot').value
        self.move_burst_min = self.get_parameter('move_burst_min').value
        self.move_burst_max = self.get_parameter('move_burst_max').value
        self.move_vec_speed = self.get_parameter('move_vec_speed').value
        self.move_burst_max_vec = self.get_parameter('move_burst_max_vec').value
        self.move_max_iters = self.get_parameter('move_max_iters').value
        self.move_max_lin = self.get_parameter('move_max_lin').value
        self.move_max_yaw = self.get_parameter('move_max_yaw').value
        self.move_settle_sec = self.get_parameter('move_settle_sec').value

        self.dt = 1.0 / self.control_rate

        # --- I2C init ---
        self.bus = None
        self._try_init_i2c()

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
        self._last_write_t = 0.0             # monotonic time of last motor write

        self.enc_last = [0, 0, 0, 0]
        self.t_last = time.monotonic()

        # While a DriveRelative move is executing the service callback owns the
        # I2C bus exclusively; _abort lets the (reentrant) e-stop interrupt it
        # between bursts without ever touching I2C itself.
        self._move_active = False
        self._abort = False

        # Callback groups: the control loop, /cmd_vel, and the relative-move
        # service all do I2C, so they share ONE mutually-exclusive group — the
        # executor never runs two of them at once, so no read ever races a write.
        # E-stop is reentrant and I2C-free (it only sets flags), so it can fire
        # even while a multi-second move is blocking the I2C group.
        self.io_group = MutuallyExclusiveCallbackGroup()
        self.estop_group = ReentrantCallbackGroup()

        # --- ROS interfaces ---
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10,
            callback_group=self.io_group)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.battery_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        # Idle-only battery poll: reads only when motors have been stopped
        # >2 s (the 0x34 driver bricks if reads interleave with motion).
        self.battery_timer = self.create_timer(
            30.0, self._battery_timer_cb, callback_group=self.io_group)
        self.estop_srv = self.create_service(
            Trigger, 'motor_controller/emergency_stop', self.estop_callback,
            callback_group=self.estop_group)
        self.drive_rel_srv = self.create_service(
            DriveRelative, 'motor_controller/drive_relative',
            self.drive_relative_callback, callback_group=self.io_group)
        self.control_timer = self.create_timer(
            self.dt, self.control_loop, callback_group=self.io_group)

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
        self._last_write_t = time.monotonic()
        self._i2c_retry(self.bus.write_i2c_block_data, self.i2c_addr, 0x33, block)

    def _clamp_cmd(self, u):
        if abs(u) < 0.4 * self.min_cmd:
            return 0
        if abs(u) < self.min_cmd:
            # Deadband compensation: Nav2 rotation commands (~0.2 rad/s) map to
            # ~14 counts, under the ~20-count motor deadband — without a floor
            # the base silently ignores them and goals time out.
            u = math.copysign(self.min_cmd, u)
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
        # I2C-free: just set flags. _abort interrupts an in-progress relative
        # move between bursts; the control loop / move loop do the actual stop
        # write so this never races a motor write on the bus.
        self._abort = True
        self.state = State.STOPPING
        self.vx_ref = self.vy_ref = self.wz_ref = 0.0
        response.success = True
        response.message = 'Emergency stop activated'
        return response

    def _wheel_refs_from(self, vx, vy, wz):
        """Forward-driving wheel speed targets (m/s) per channel for a body
        velocity (vx fwd, vy left, wz CCW)."""
        return [
            vx * KX[i] + vy * KY[i] + wz * self.L * KW[i]
            for i in range(4)
        ]

    def _wheel_refs(self):
        """Forward-driving wheel speed targets (m/s) per channel."""
        return self._wheel_refs_from(self.vx_ref, self.vy_ref, self.wz_ref)

    # --- DriveRelative (closed-loop relative move) ---
    def _compute_cmd4(self, vx, vy, wz):
        """Open-loop feedforward channel commands for a body velocity, with the
        same rate-limit + deadband clamp as the main control loop."""
        w_ref = self._wheel_refs_from(vx, vy, wz)
        cmd4 = [0, 0, 0, 0]
        for i in range(4):
            u = self._rate_limit(self.cmd_per_mps * w_ref[i], self.u_prev[i])
            self.u_prev[i] = u
            cmd4[i] = self._clamp_cmd(SIGN[i] * u)
        return cmd4

    def _read_wheels(self):
        """Per-wheel forward-positive distance (m) from encoders. Caller MUST be
        on the io_group (no concurrent motor write)."""
        enc = self._read_encoders()
        return [SIGN[i] * enc[i] / self.ticks_per_meter for i in range(4)]

    def _body_disp(self, w0, w):
        """Body-frame (dx fwd, dy left, dyaw CCW) from start wheel dists w0 -> w,
        with slip factors applied so the result is true ground motion."""
        d = [w[i] - w0[i] for i in range(4)]
        dx = (d[0] + d[1] + d[2] + d[3]) / 4.0
        dy = (d[0] + d[1] - d[2] - d[3]) / 4.0 * self.k_strafe
        dyaw = (d[0] - d[1] - d[2] + d[3]) / (4.0 * self.L) * self.k_yaw
        return [dx, dy, dyaw]

    def _drive_burst(self, vx, vy, wz, seconds):
        """Drive a body velocity for `seconds`, ramped via _compute_cmd4."""
        n = max(1, int(seconds * self.control_rate))
        for _ in range(n):
            if self._abort:
                break
            self._write_motors(self._compute_cmd4(vx, vy, wz))
            time.sleep(self.dt)

    def _stop_and_settle(self):
        """Ramp to a stop, then go quiet on the bus so the next encoder read is
        clean (no write/read interleave)."""
        for _ in range(12):
            for i in range(4):
                self.u_prev[i] *= 0.6
            self._write_motors([self._clamp_cmd(SIGN[i] * self.u_prev[i]) for i in range(4)])
            time.sleep(self.dt)
        self.u_prev = [0.0, 0.0, 0.0, 0.0]
        self._write_motors([0, 0, 0, 0])
        time.sleep(self.move_settle_sec)
        self._log_battery()

    def _try_init_i2c(self):
        """(Re)open the bus and configure the 0x34 driver. The board can be
        hung off the bus at boot (errno 110 — e.g. a previous shutdown killed
        a transaction mid-flight); it comes back only after a power cycle, so
        the battery timer keeps retrying this instead of the node dying."""
        try:
            bus = smbus.SMBus(self.bus_id)
            self._i2c_retry(bus.write_byte_data, self.i2c_addr, 0x14, 1)
            self._i2c_retry(bus.write_byte_data, self.i2c_addr, 0x15, 0)
            self.bus = bus
            self.get_logger().info(
                f'I2C initialized on bus {self.bus_id}, addr 0x{self.i2c_addr:02x}')
            return True
        except Exception as e:
            self.get_logger().error(
                f'Failed to initialize I2C: {e} (motor board absent/hung — '
                'power cycle it; will keep retrying)')
            self.bus = None
            return False

    def _log_battery(self):
        """Battery ADC (reg 0x00, mV LE) — supply-sag telemetry for the brick
        hunt. Only ever called from the io_group (same serialization rule as
        encoder reads); read failures are logged, never raised."""
        if self.bus is None:
            return
        try:
            raw = self._i2c_retry(self.bus.read_i2c_block_data, self.i2c_addr, 0x00, 2)
            mv = struct.unpack('<H', bytes(raw))[0]
            self.get_logger().info(f'battery: {mv} mV')
            self.battery_pub.publish(Float32(data=mv / 1000.0))
        except OSError as e:
            self.get_logger().warn(f'battery read failed: {e}')

    def _battery_timer_cb(self):
        """Slow idle-only battery poll for telemetry/UI. Skips whenever the
        motors have written within 2 s — reads must never interleave with
        motion (same brick rule as encoder reads). Doubles as the I2C
        reconnect loop while the bus is down."""
        if self.bus is None:
            self._try_init_i2c()
            return
        if any(self.u_prev) or (time.monotonic() - self._last_write_t) < 2.0:
            return
        self._log_battery()

    def _run_axis(self, w_start, idx, target, vmag, tol, is_yaw):
        """Closed-loop drive of one body axis (0=fwd,1=strafe,2=yaw) to target."""
        for _ in range(self.move_max_iters):
            if self._abort:
                return
            moved = self._body_disp(w_start, self._read_wheels())[idx]
            remaining = target - moved
            if abs(remaining) <= tol:
                return
            direction = 1.0 if remaining > 0 else -1.0
            # effective speed: forward/strafe under-deliver ~74%; yaw ~full.
            speed_eff = vmag * (1.0 if is_yaw else 0.74)
            burst = max(self.move_burst_min,
                        min(self.move_burst_max,
                            self.move_undershoot * abs(remaining) / max(speed_eff, 1e-3)))
            vx = direction * vmag if idx == 0 else 0.0
            vy = direction * vmag if idx == 1 else 0.0
            wz = direction * vmag if idx == 2 else 0.0
            self._drive_burst(vx, vy, wz, burst)
            self._stop_and_settle()

    def _run_vector(self, w_start, tx, ty, vmag, tol):
        """Closed-loop simultaneous forward+strafe: one long continuous burst
        along the remaining (dx, dy) vector, re-aimed after each settle. This
        replaces the axis-sequential fwd-then-strafe (visibly jerky L-shaped
        moves). Encoders are still read ONLY between bursts."""
        for _ in range(self.move_max_iters):
            if self._abort:
                return
            d = self._body_disp(w_start, self._read_wheels())
            rx, ry = tx - d[0], ty - d[1]
            rem = math.hypot(rx, ry)
            if rem <= tol:
                return
            ux, uy = rx / rem, ry / rem
            speed_eff = vmag * 0.74
            burst = max(self.move_burst_min,
                        min(self.move_burst_max_vec,
                            self.move_undershoot * rem / max(speed_eff, 1e-3)))
            self._drive_burst(ux * vmag, uy * vmag, 0.0, burst)
            self._stop_and_settle()

    def drive_relative_callback(self, request, response):
        if not self.bus:
            response.success = False
            response.message = 'no I2C bus'
            return response
        if (abs(request.dx) > self.move_max_lin or abs(request.dy) > self.move_max_lin
                or abs(request.dyaw) > self.move_max_yaw):
            response.success = False
            response.message = (
                f'target exceeds per-call bounds (|lin|<={self.move_max_lin} m, '
                f'|yaw|<={self.move_max_yaw} rad)')
            return response

        self._abort = False
        self._move_active = True
        self.state = State.IDLE
        self.vx_ref = self.vy_ref = self.wz_ref = 0.0
        self.u_prev = [0.0, 0.0, 0.0, 0.0]
        self.get_logger().info(
            f'DriveRelative: dx={request.dx:.3f} dy={request.dy:.3f} dyaw={request.dyaw:.3f}')
        final = [0.0, 0.0, 0.0]
        try:
            # The 0x34 board NACKs reads that follow a write stream too closely
            # (errno 121 when a move starts right after Nav2 stops commanding).
            # Give the bus a quiet window before the first encoder read.
            quiet = time.monotonic() - self._last_write_t
            if quiet < 0.6:
                time.sleep(0.6 - quiet)
            w_start = self._read_wheels()
            # Yaw first (aim), then forward+strafe together as one smooth
            # vector move. Cross-axis coupling is negligible (validated).
            if abs(request.dyaw) > self.move_tol_yaw:
                self._run_axis(w_start, 2, request.dyaw, self.move_yaw_speed,
                               self.move_tol_yaw, True)
            if not self._abort and math.hypot(request.dx, request.dy) > self.move_tol_lin:
                self._run_vector(w_start, request.dx, request.dy,
                                 self.move_vec_speed, self.move_tol_lin)
            final = self._body_disp(w_start, self._read_wheels())
        except Exception as e:
            self.get_logger().error(f'DriveRelative error: {e}')
            response.success = False
            response.message = f'error: {e}'
        finally:
            self.u_prev = [0.0, 0.0, 0.0, 0.0]
            try:
                self._write_motors([0, 0, 0, 0])
            except Exception:
                pass
            self._move_active = False
            self.state = State.IDLE

        response.actual_dx, response.actual_dy, response.actual_dyaw = final
        if not response.message or response.message == '':
            response.success = not self._abort
            response.message = 'aborted by e-stop' if self._abort else 'ok'
        self.get_logger().info(
            f'DriveRelative done: actual dx={final[0]:.3f} dy={final[1]:.3f} '
            f'dyaw={math.degrees(final[2]):.1f}deg success={response.success}')
        return response

    # --- Control loop ---
    def control_loop(self):
        # A DriveRelative move owns the I2C bus while it runs; stay off the bus.
        # (The shared mutually-exclusive callback group already prevents overlap;
        # this is belt-and-suspenders in case the executor config ever changes.)
        if self._move_active:
            return
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
    # MultiThreaded so the reentrant e-stop can fire while a relative move blocks
    # the I2C (io) group. The io group itself stays serialized (no bus race).
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
