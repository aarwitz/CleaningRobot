"""Operator teleop server: keyboard -> smooth base + arm motion.

Topology
--------
    ui/index.html --(rosbridge)--> /teleop/cmd    (std_msgs/String, JSON, ~20 Hz)
                                   /teleop/action (std_msgs/String, edge events)
    this node     --> /cmd_vel      (geometry_msgs/Twist)  -> motor_controller
                  --> /dev/ttyUSB0  (T:123 velocity jog)   -> RoArm M2-S
                  --> /teleop/state (std_msgs/String, JSON, 10 Hz) -> UI HUD

JSON over String rather than custom .msg types is deliberate: rosbridge speaks
it natively and it keeps the whole operator interface in one file, with no
interface package to rebuild when a field is added.

Design notes that matter
------------------------
* SMOOTHNESS comes from ramping here, not from the firmware. A T:123 jog
  changes speed the instant a new `spd` lands, so a raw key press would be a
  velocity step. Every axis is run through an acceleration limiter, which is
  what makes the arm and base feel natural rather than twitchy.
* The arm keeps jogging FOREVER until told to stop. If the operator's browser
  tab closes mid-jog, nothing else will stop it. Hence the deadman: no
  /teleop/cmd within `watchdog_s` and every axis is stopped.
* Workspace limits are enforced against live T:105 feedback with a speed
  taper, because the firmware has no soft limits of its own.
"""

import json
import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from robot_teleop.arm_link import (
    ArmLink, AXIS_X, AXIS_Y, AXIS_Z, GRIP_OPEN, GRIP_CLOSED, MM_S_PER_SPD,
)


def ramp(cur, target, accel, dt):
    """Move `cur` toward `target` no faster than `accel` per second."""
    step = accel * dt
    if target > cur:
        return min(cur + step, target)
    if target < cur:
        return max(cur - step, target)
    return cur


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')

        p = self.declare_parameter
        p('serial_port', '/dev/ttyUSB0')
        p('baud_rate', 115200)
        p('control_rate', 20.0)      # jog/base update rate
        p('feedback_rate', 8.0)      # T:105 poll rate; >10 Hz starves the servo loop
        p('state_rate', 10.0)        # HUD publish rate
        p('watchdog_s', 0.4)         # deadman: silence longer than this = stop

        # Arm cartesian speeds (mm/s) per mode.
        p('arm_speed_precision', 10.0)
        p('arm_speed_normal', 35.0)
        p('arm_speed_boost', 70.0)
        p('arm_accel', 150.0)        # mm/s^2
        p('grip_speed', 12.0)        # T:123 spd units for the gripper axis

        # Base speeds. The I2C driver has a ~0.10 m/s deadband (min_cmd 20), so
        # even "precision" must stay above it or the wheels just buzz.
        p('base_lin_precision', 0.10)
        p('base_lin_normal', 0.14)
        p('base_lin_boost', 0.22)
        p('base_yaw_precision', 0.25)
        p('base_yaw_normal', 0.40)   # 0.4 not 0.6 — higher yaw overshoots badly
        p('base_yaw_boost', 0.60)
        p('base_lin_accel', 0.50)    # m/s^2
        p('base_yaw_accel', 1.50)    # rad/s^2

        # Arm workspace envelope (arm frame, mm). r_min protects the RealSense,
        # which is mounted on the arm's own rotating base: a carried object at
        # small radius swings straight into the lens.
        p('r_min', 180.0)
        p('r_max', 480.0)    # probed 2026-07-25: cartesian goto tracks to
                             # r=484 @ z=30 and r=446 @ z=-90 (droop 2-10mm).
                             # The REAL ceiling is reach_max below.
        # Spherical reach limit, measured from the SHOULDER (the z origin of
        # the arm frame). The firmware IK's elbow-straight singularity is at
        # l2+l3 = 518.9mm and its T:1041 handler has NO NaN guard (verified in
        # vendor source) — past the singularity acos() NaNs and garbage servo
        # targets go on the wire. 505 keeps a margin.
        p('reach_max', 505.0)

        p('z_min', -200.0)
        p('z_max', 320.0)
        p('limit_taper_mm', 30.0)    # slow down within this distance of a limit

        p('home_x', 250.0)
        p('home_y', 0.0)
        p('home_z', 150.0)
        p('home_t', 3.0)

        g = lambda k: self.get_parameter(k).value
        self.rate = g('control_rate')
        self.watchdog_s = g('watchdog_s')

        self.arm = ArmLink(g('serial_port'), g('baud_rate'), self.get_logger())
        self.arm_ok = self.arm.open()

        # ── operator intent (normalized -1..1), updated by /teleop/cmd ─────
        self.want = {'ax': 0.0, 'ay': 0.0, 'az': 0.0, 'grip': 0.0,
                     'bx': 0.0, 'by': 0.0, 'bw': 0.0}
        self.mode = 'normal'
        self.estop = False
        self.last_cmd_time = 0.0
        self.seq = 0
        # anti-hunt: fb x-history for the idle limit-cycle detector
        self._hunt_hist = []
        self._hunt_last_escape = 0.0

        # ── ramped actual velocities ──────────────────────────────────────
        self.v = {'ax': 0.0, 'ay': 0.0, 'az': 0.0,       # mm/s
                  'bx': 0.0, 'by': 0.0, 'bw': 0.0}       # m/s, m/s, rad/s
        self.grip_rate = 0.0
        self.blocked = []

        self._lock = threading.Lock()
        self._last_reset = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/teleop/state', 10)
        self.create_subscription(String, '/teleop/cmd', self._cmd_cb, 10)
        self.create_subscription(String, '/teleop/action', self._action_cb, 10)

        self.create_timer(1.0 / self.rate, self._control_tick)
        self.create_timer(1.0 / g('feedback_rate'), self._feedback_tick)
        self.create_timer(1.0 / g('state_rate'), self._state_tick)

        if self.arm_ok:
            fb = self.arm.poll_feedback(timeout=0.6)
            if fb is None:
                self.get_logger().warn('arm silent at startup — resetting firmware')
                self.arm.reset_firmware()
                self.arm.poll_feedback(timeout=0.8)
            self.arm.stop_all()

        self.get_logger().info(
            f'teleop ready | arm={"up" if self.arm_ok else "DOWN"} '
            f'| deadman {self.watchdog_s}s | cmd {self.rate} Hz')

    # ── inbound ─────────────────────────────────────────────────────────
    def _cmd_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            a = d.get('arm', {})
            b = d.get('base', {})
            self.want['ax'] = clamp(float(a.get('x', 0.0)), -1, 1)
            self.want['ay'] = clamp(float(a.get('y', 0.0)), -1, 1)
            self.want['az'] = clamp(float(a.get('z', 0.0)), -1, 1)
            self.want['grip'] = clamp(float(a.get('grip', 0.0)), -1, 1)
            self.want['bx'] = clamp(float(b.get('vx', 0.0)), -1, 1)
            self.want['by'] = clamp(float(b.get('vy', 0.0)), -1, 1)
            self.want['bw'] = clamp(float(b.get('wz', 0.0)), -1, 1)
            self.mode = d.get('mode', 'normal')
            if d.get('estop'):
                self.estop = True
            self.seq = int(d.get('seq', 0))
            self.last_cmd_time = time.time()

    def _action_cb(self, msg):
        act = msg.data.strip()
        # goto arrives as a streamed setpoint train during scripted runs; logging
        # every one of them at 10 Hz buries everything else in the log.
        if not act.startswith('goto:'):
            self.get_logger().info(f'action: {act}')
        if act == 'estop':
            self.estop = True
            self._all_stop()
        elif act == 'clear_estop':
            self.estop = False
        elif act == 'home':
            if not self.estop:
                self.arm.stop_all()
                self.arm.goto(self.get_parameter('home_x').value,
                              self.get_parameter('home_y').value,
                              self.get_parameter('home_z').value,
                              self.get_parameter('home_t').value)
        elif act.startswith('led'):
            # "led" -> 255, "led:0".."led:255"
            try:
                val = int(act.split(':', 1)[1]) if ':' in act else 255
            except Exception:
                val = 255
            self.arm.led(val)
        elif act == 'torque_off':
            self.arm.stop_all()
            self.arm.torque(False)
        elif act == 'torque_on':
            self.arm.torque(True)
        elif act == 'reset_arm':
            self.arm.stop_all()
            if self.arm.reset_firmware():
                self.arm.poll_feedback(timeout=0.8)
        elif act.startswith('goto:'):
            # "goto:x,y,z,t" — absolute placement for scripted setup moves.
            # Uses the non-blocking T:1041 and still respects the envelope, so
            # a bad number cannot drive the tool into the camera.
            if self.estop:
                return          # E-STOP must halt scripted motion, not just jogs
            try:
                x, y, z, t = (float(v) for v in act.split(':', 1)[1].split(','))
            except Exception:
                self.get_logger().error(f'bad goto: {act}')
                return
            r = math.hypot(x, y)
            r_max = self.get_parameter('r_max').value
            r_min = self.get_parameter('r_min').value
            if r > 1e-6 and not (r_min <= r <= r_max):
                r_c = clamp(r, r_min, r_max)
                x, y = x * r_c / r, y * r_c / r
                self.get_logger().warn(f'goto clamped r {r:.0f} -> {r_c:.0f}')
            z = clamp(z, self.get_parameter('z_min').value,
                      self.get_parameter('z_max').value)
            # Spherical clamp against the IK singularity (see reach_max):
            # keep z, pull the radius in so hypot(r, z) stays inside.
            reach = self.get_parameter('reach_max').value
            r = math.hypot(x, y)
            if r > 1e-6 and math.hypot(r, z) > reach:
                r_c = math.sqrt(max(0.0, reach**2 - z*z))
                self.get_logger().warn(
                    f'goto clamped to reach sphere: r {r:.0f} -> {r_c:.0f} '
                    f'at z {z:.0f}')
                x, y = x * r_c / r, y * r_c / r
            self.arm.stop_all()
            self.arm.goto(x, y, z, t)
        elif act == 'grip_open':
            self._preset_grip(GRIP_OPEN)
        elif act == 'grip_close':
            self._preset_grip(GRIP_CLOSED)
        elif act.startswith('raw:'):
            # Diagnostic passthrough: "raw:{...}" sends the JSON verbatim to
            # the firmware. E-STOP-gated like goto. Added 2026-07-25 while
            # chasing the firmware's cartesian-mode reach ceiling (~r399 with
            # the elbow pinned at pi/2) — joint-space probing needs T:101/102,
            # which the cartesian teleop surface cannot express.
            if self.estop:
                return
            try:
                obj = json.loads(act[4:])
            except Exception:
                self.get_logger().error(f'bad raw: {act}')
                return
            self.get_logger().info(f'raw passthrough: T={obj.get("T")}')
            self.arm._write(obj)

    def _preset_grip(self, t):
        fb = self.arm.last_fb
        if fb:
            self.arm.goto(fb['x'], fb['y'], fb['z'], t)

    # ── limits ──────────────────────────────────────────────────────────
    def _limit_scale(self, vx, vy, vz):
        """Return per-axis multipliers that keep the tool inside the envelope.

        Taper rather than hard-stop: motion slows as a limit approaches, so the
        operator feels a soft wall instead of a jolt. Only the component that
        WORSENS a violation is restricted — retreating is always allowed.
        """
        fb = self.arm.last_fb
        self.blocked = []
        if not fb:
            return 1.0, 1.0, 1.0
        x, y, z = fb['x'], fb['y'], fb['z']
        r = math.hypot(x, y)
        r_min = self.get_parameter('r_min').value
        r_max = self.get_parameter('r_max').value
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value
        band = max(1.0, self.get_parameter('limit_taper_mm').value)

        sx = sy = sz = 1.0
        # Radial: dr/dt from x/y motion is (x/r)vx + (y/r)vy.
        if r > 1e-6:
            ur_x, ur_y = x / r, y / r
            dr = ur_x * vx + ur_y * vy
            if dr > 0:                       # moving outward
                room = r_max - r
                s = clamp(room / band, 0.0, 1.0)
                if s < 1.0:
                    sx = sy = s
                    if s <= 0.01:
                        self.blocked.append('r_max')
            elif dr < 0:                     # moving inward toward the camera
                room = r - r_min
                s = clamp(room / band, 0.0, 1.0)
                if s < 1.0:
                    sx = sy = s
                    if s <= 0.01:
                        self.blocked.append('r_min')
        if vz > 0:
            s = clamp((z_max - z) / band, 0.0, 1.0)
            sz = s
            if s <= 0.01:
                self.blocked.append('z_max')
        elif vz < 0:
            s = clamp((z - z_min) / band, 0.0, 1.0)
            sz = s
            if s <= 0.01:
                self.blocked.append('z_min')
        # Spherical reach taper (see reach_max param): distance from the
        # shoulder must stay below the IK singularity. Only motion that grows
        # that distance is tapered; retreating is always allowed.
        reach = self.get_parameter('reach_max').value
        lc = math.hypot(r, z)
        if lc > 1e-6:
            dr = (x * vx + y * vy) / r if r > 1e-6 else 0.0
            dlc = (r * dr + z * vz) / lc
            if dlc > 0:
                s = clamp((reach - lc) / band, 0.0, 1.0)
                if s < 1.0:
                    sx, sy, sz = sx * s, sy * s, sz * s
                    if s <= 0.01:
                        self.blocked.append('reach_max')
        return sx, sy, sz

    # ── control ─────────────────────────────────────────────────────────
    def _speeds(self):
        m = self.mode
        g = lambda k: self.get_parameter(k).value
        if m == 'precision':
            return g('arm_speed_precision'), g('base_lin_precision'), g('base_yaw_precision')
        if m == 'boost':
            return g('arm_speed_boost'), g('base_lin_boost'), g('base_yaw_boost')
        return g('arm_speed_normal'), g('base_lin_normal'), g('base_yaw_normal')

    def _all_stop(self):
        for k in self.v:
            self.v[k] = 0.0
        self.grip_rate = 0.0
        if self.arm_ok:
            self.arm.stop_all()
        self.cmd_vel_pub.publish(Twist())

    def _control_tick(self):
        dt = 1.0 / self.rate
        with self._lock:
            want = dict(self.want)
            estop = self.estop
        alive = (time.time() - self.last_cmd_time) < self.watchdog_s

        if estop or not alive:
            # Deadman / e-stop: ramp base down fast but command the arm to a
            # hard stop immediately — a coasting arm is the dangerous one.
            changed = any(abs(self.v[k]) > 1e-6 for k in self.v) or abs(self.grip_rate) > 1e-6
            for k in self.v:
                self.v[k] = 0.0
            self.grip_rate = 0.0
            if changed and self.arm_ok:
                self.arm.stop_all()
            self.cmd_vel_pub.publish(Twist())
            return

        arm_spd, lin_spd, yaw_spd = self._speeds()
        a_acc = self.get_parameter('arm_accel').value
        l_acc = self.get_parameter('base_lin_accel').value
        w_acc = self.get_parameter('base_yaw_accel').value

        # Ramp toward the operator's requested velocity.
        self.v['ax'] = ramp(self.v['ax'], want['ax'] * arm_spd, a_acc, dt)
        self.v['ay'] = ramp(self.v['ay'], want['ay'] * arm_spd, a_acc, dt)
        self.v['az'] = ramp(self.v['az'], want['az'] * arm_spd, a_acc, dt)
        self.v['bx'] = ramp(self.v['bx'], want['bx'] * lin_spd, l_acc, dt)
        self.v['by'] = ramp(self.v['by'], want['by'] * lin_spd, l_acc, dt)
        self.v['bw'] = ramp(self.v['bw'], want['bw'] * yaw_spd, w_acc, dt)

        if self.arm_ok:
            # ── anti-hunt (2026-08-19): at gravity-neutral elbow poses the
            # firmware's position hold limit-cycles (torE swings through
            # zero; fb x oscillates ~10mm; operator saw "reaching in/out").
            # With ZERO operator intent, watch fb x peak-to-peak over ~2s;
            # sustained oscillation -> one goto 18mm up to move the elbow
            # off its torque zero-crossing. Never fires while driving.
            idle = all(abs(want[k]) < 1e-6
                       for k in ('ax', 'ay', 'az', 'grip'))
            fb = self.arm.last_fb or {}
            if idle and fb.get('x') is not None:
                now = time.time()
                self._hunt_hist.append((now, fb['x']))
                self._hunt_hist = [(t_, x_) for t_, x_ in self._hunt_hist
                                   if now - t_ < 2.0]
                xs = [x_ for _, x_ in self._hunt_hist]
                if (len(xs) > 20 and max(xs) - min(xs) > 6.0
                        and now - self._hunt_last_escape > 10.0
                        and fb.get('z') is not None):
                    self._hunt_last_escape = now
                    self._hunt_hist.clear()
                    self.get_logger().warn(
                        'anti-hunt: idle limit-cycle detected '
                        f'(x pp={max(xs)-min(xs):.1f}mm) -> escape +18mm z')
                    self.arm.goto(fb['x'], fb.get('y', 0.0),
                                  fb['z'] + 18.0, fb.get('t', 2.0))
            elif not idle:
                self._hunt_hist.clear()
            sx, sy, sz = self._limit_scale(self.v['ax'], self.v['ay'], self.v['az'])
            self.arm.jog(AXIS_X, self.v['ax'] * sx)
            self.arm.jog(AXIS_Y, self.v['ay'] * sy)
            self.arm.jog(AXIS_Z, self.v['az'] * sz)
            gr = want['grip'] * self.get_parameter('grip_speed').value
            self.grip_rate = gr
            self.arm.jog_grip(gr)

        tw = Twist()
        tw.linear.x = self.v['bx']
        tw.linear.y = self.v['by']
        tw.angular.z = self.v['bw']
        self.cmd_vel_pub.publish(tw)

    # ── feedback + HUD ──────────────────────────────────────────────────
    def _feedback_tick(self):
        if not self.arm_ok:
            return
        self.arm.poll_feedback(timeout=0.15)
        age = self.arm.fb_age()
        if age > 3.0:
            self.arm.link_state = 'stale'
        # Auto-heal a wedged firmware, but only when the operator is not
        # actively commanding motion and not more than once every 15 s.
        if age > 6.0 and (time.time() - self._last_reset) > 15.0:
            idle = all(abs(v) < 1e-6 for v in self.v.values())
            if idle:
                self._last_reset = time.time()
                self.get_logger().warn(f'arm feedback stale {age:.1f}s — auto-reset')
                self.arm.reset_firmware()
                self.arm.poll_feedback(timeout=0.8)

    def _state_tick(self):
        fb = self.arm.last_fb or {}
        st = {
            'ts': time.time(),
            'seq': self.seq,
            'link': self.arm.link_state if self.arm_ok else 'down',
            'fb_age': round(self.arm.fb_age(), 2) if self.arm_ok else None,
            'estop': self.estop,
            'watchdog': (time.time() - self.last_cmd_time) < self.watchdog_s,
            'mode': self.mode,
            'arm': {k: fb.get(k) for k in
                    ('x', 'y', 'z', 't', 'b', 's', 'e',
                     'torB', 'torS', 'torE', 'torH')},
            'arm_r': round(math.hypot(fb['x'], fb['y']), 1) if 'x' in fb else None,
            'vel': {'ax': round(self.v['ax'], 1), 'ay': round(self.v['ay'], 1),
                    'az': round(self.v['az'], 1), 'grip': round(self.grip_rate, 1),
                    'bx': round(self.v['bx'], 3), 'by': round(self.v['by'], 3),
                    'bw': round(self.v['bw'], 3)},
            'blocked': self.blocked,
            'limits': {'r_min': self.get_parameter('r_min').value,
                       'r_max': self.get_parameter('r_max').value,
                       'z_min': self.get_parameter('z_min').value,
                       'z_max': self.get_parameter('z_max').value},
        }
        m = String()
        m.data = json.dumps(st)
        self.state_pub.publish(m)

    def destroy_node(self):
        try:
            self._all_stop()
            self.arm.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Normal on Ctrl-C or a launch-level kill; the finally block still
        # stops the arm, which is the part that actually matters.
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
