#!/usr/bin/env python3
"""Nav2-based autonomous sock-collection mission director.

The production mission loop on the full Isaac stack (cuVSLAM VO -> nvblox ->
Nav2), replacing the SLAM-free encoder_wander driver. Validated chain
(2026-07-03): detect -> Nav2 standoff approach (RotationShim/DWB) -> encoder
creep to cam z<=0.35 -> arm pick envelope.

States:
  SCAN     paced rescan-in-place (45 deg steps + dwell) so nvblox integrates a
           ring map; continuous rotation integrates almost nothing.
  WANDER   Nav2 goal into the most-open costmap direction; interrupted the
           moment a stable (non-blacklisted) sock detection appears.
  APPROACH Nav2 goal at a standoff short of the sock 3D fix, facing it.
  CREEP    encoder crab-hops (drive_relative) until the fix is inside the
           arm's proven pick window (cam z <= 0.35).
  PICK     hand off to arm_bridge's autonomous loop (base parked), or a
           simulated pick when the arm is disabled; the sock's map position
           is blacklisted afterwards so the mission moves on.

All coarse motion is Nav2 (obstacle-aware); all terminal motion is the
closed-loop encoder primitive (<5 mm error). The arm loop runs ONLY while the
base is parked (mid-approach picks grab air - observed 2026-07-02).

Run standalone (supervised):
  ros2 run behavior_manager mission_director
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped support)

from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2D

from behavior_manager_interfaces.srv import DriveRelative, GetSock3D, PanCamera


class MissionDirector(Node):
    def __init__(self):
        super().__init__('mission_director')
        # -- mission scope
        self.declare_parameter('max_picks', 3)
        self.declare_parameter('max_wander_legs', 20)
        self.declare_parameter('simulate_pick', True)   # ENABLE_ARM=false default
        # -- wander
        self.declare_parameter('wander_leg_m', 1.2)
        self.declare_parameter('min_leg_m', 0.5)
        self.declare_parameter('lethal_cost', 65)       # occupancy 0..100
        self.declare_parameter('scan_step_rad', math.pi / 4)
        self.declare_parameter('scan_dwell_s', 4.0)     # nvblox needs the pause
        self.declare_parameter('legs_between_scans', 3)
        # -- approach / creep (validated numbers)
        self.declare_parameter('standoff_m', 0.55)
        self.declare_parameter('approach_stop_z_m', 0.35)
        self.declare_parameter('creep_margin_m', 0.30)
        self.declare_parameter('creep_hop_max_m', 0.25)
        self.declare_parameter('blacklist_radius_m', 0.5)
        # -- detection stability gate
        self.declare_parameter('det_stable_n', 3)       # sightings within window
        self.declare_parameter('det_window_s', 2.5)

        self._lock = threading.Lock()
        self.detection = None
        self.det_stamp = 0.0
        self.det_times = []          # recent sighting times (stability gate)
        self.costmap = None
        self.depth = None
        self.depth_stamp = 0.0
        self.arm_busy = False
        self.pick_events = 0
        self.blacklist = []          # [(x, y)] map-frame picked/abandoned socks
        self.last_fix = None         # (cam_pt, map_pt, t) most recent good fix
        self.state = 'INIT'

        cb = ReentrantCallbackGroup()
        self.cb = cb
        self.create_subscription(Detection2D, '/clothes/detected',
                                 self._det_cb, 10, callback_group=cb)
        # Nav2 publishes the costmap transient-local; default QoS never
        # receives it.
        costmap_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._costmap_cb, costmap_qos,
                                 callback_group=cb)
        self.create_subscription(Image, '/camera/depth/image_rect_raw',
                                 self._depth_cb, 1, callback_group=cb)
        self.create_subscription(String, '/arm/status',
                                 self._arm_cb, 10, callback_group=cb)

        self.drive = self.create_client(DriveRelative,
                                        '/motor_controller/drive_relative',
                                        callback_group=cb)
        self.get_3d = self.create_client(GetSock3D,
                                         '/clothes_perception/get_3d_pose',
                                         callback_group=cb)
        self.arm_active = self.create_client(SetBool, '/arm_bridge/set_active',
                                             callback_group=cb)
        self.pan_cli = self.create_client(PanCamera, '/arm_bridge/pan_camera',
                                          callback_group=cb)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose',
                                callback_group=cb)

        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)

        # operator console feed
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.target_pub = self.create_publisher(PointStamped, '/mission/target', 10)
        self.create_timer(1.0, self._pub_state, callback_group=cb)

    # ------------------------------------------------------------- callbacks

    def _det_cb(self, msg):
        now = time.monotonic()
        with self._lock:
            self.detection = msg
            self.det_stamp = now
            self.det_times.append(now)
            self.det_times = [t for t in self.det_times if now - t < 6.0]

    def _costmap_cb(self, msg):
        self.costmap = msg

    def _depth_cb(self, msg):
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        self.depth = d.astype(np.float32) / 1000.0
        self.depth_stamp = time.monotonic()

    def _arm_cb(self, msg):
        s = msg.data
        if s.startswith(('Stable detection', 'Picking at', 'Pick #',
                         'Place step', 'Pick step')):
            self.arm_busy = True
        if s.startswith('Place complete'):
            self.arm_busy = False
            with self._lock:
                self.pick_events += 1
        if 'skipping' in s or 'FAILED' in s or 'failed' in s or 'error' in s:
            self.arm_busy = False

    def _pub_state(self):
        self.state_pub.publish(String(data=self.state))

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'[mission] {self.state} -> {s}')
        self.state = s
        self.state_pub.publish(String(data=s))

    # ------------------------------------------------------------- geometry

    def robot_pose(self, timeout=2.0):
        """(x, y, yaw) of base_link in map, or None."""
        try:
            tr = self.tfbuf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout))
        except Exception:
            return None
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (tr.transform.translation.x, tr.transform.translation.y, yaw)

    def cam_point_to_map(self, pt_stamped, timeout=2.0):
        pt_stamped.header.stamp = rclpy.time.Time().to_msg()
        try:
            return self.tfbuf.transform(
                pt_stamped, 'map',
                timeout=rclpy.duration.Duration(seconds=timeout))
        except Exception as e:
            self.get_logger().warn(f'[mission] TF to map failed: {e}')
            return None

    # ------------------------------------------------------------- perception

    def stable_detection(self):
        """Detection seen det_stable_n times in det_window_s, fresh <1.5 s."""
        p = self.get_parameter
        now = time.monotonic()
        with self._lock:
            if self.detection is None or now - self.det_stamp > 1.5:
                return None
            recent = [t for t in self.det_times
                      if now - t < p('det_window_s').value]
            if len(recent) < p('det_stable_n').value:
                return None
            return self.detection

    def sock_fix(self, det=None):
        """(cam_point, map_point) for the freshest detection, or (None, None)."""
        det = det or self.stable_detection()
        if det is None:
            return None, None
        for _ in range(4):
            res = self.get_3d.call(GetSock3D.Request(detection=det))
            if res is not None and res.success:
                pmap = self.cam_point_to_map(res.point)
                if pmap is None:
                    return res.point.point, None
                return res.point.point, pmap.point
            time.sleep(0.4)
            det = self.stable_detection() or det
        return None, None

    def is_blacklisted(self, map_pt):
        r = self.get_parameter('blacklist_radius_m').value
        return any(math.hypot(map_pt.x - x, map_pt.y - y) < r
                   for x, y in self.blacklist)

    def actionable_sock(self):
        """Stable detection with a map fix outside the blacklist, else None."""
        det = self.stable_detection()
        if det is None:
            return None
        cam, mp = self.sock_fix(det)
        if mp is None:
            return None
        if self.is_blacklisted(mp):
            return None
        ps = PointStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point = mp
        self.target_pub.publish(ps)
        # cache: a sock spotted mid-pan leaves the camera view once the pan
        # re-homes, but its map fix stays valid for a blind approach
        self.last_fix = (cam, mp, time.monotonic())
        return (cam, mp)

    def cached_sock(self, max_age=45.0):
        if self.last_fix is None:
            return None
        cam, mp, t = self.last_fix
        if time.monotonic() - t > max_age or self.is_blacklisted(mp):
            return None
        return (cam, mp)

    # ------------------------------------------------------------- motion

    def move(self, dx=0.0, dy=0.0, dyaw=0.0, timeout=30.0, retries=1):
        """drive_relative with one retry: the first move after a motor power
        cycle intermittently fails its encoder read (errno 121) even though
        the motion is fine; each caller re-detects afterwards, so a retried
        hop self-corrects."""
        if not self.drive.wait_for_service(timeout_sec=3.0):
            return None
        res = None
        for attempt in range(retries + 1):
            fut = self.drive.call_async(
                DriveRelative.Request(dx=float(dx), dy=float(dy),
                                      dyaw=float(dyaw)))
            t0 = time.monotonic()
            while not fut.done():
                if time.monotonic() - t0 > timeout:
                    return None
                time.sleep(0.05)
            res = fut.result()
            if res is not None and res.success:
                return res
            self.get_logger().warn(
                f'[mission] move failed (attempt {attempt + 1}): '
                f'{getattr(res, "message", "timeout")}')
            time.sleep(1.5)
        return res

    def nav_to(self, x, y, yaw, watch_for_socks=False, timeout=120.0):
        """NavigateToPose; returns 'succeeded'|'failed'|'sock'|'timeout'.
        With watch_for_socks, cancels and returns 'sock' on a stable,
        non-blacklisted detection."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self.nav.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('[mission] Nav2 action server missing')
            return 'failed'
        send = self.nav.send_goal_async(goal)
        t0 = time.monotonic()
        while not send.done():
            if time.monotonic() - t0 > 10.0:
                return 'failed'
            time.sleep(0.05)
        gh = send.result()
        if not gh.accepted:
            return 'failed'
        rf = gh.get_result_async()
        t0 = time.monotonic()
        while not rf.done():
            if time.monotonic() - t0 > timeout:
                gh.cancel_goal_async()
                time.sleep(2.0)
                return 'timeout'
            if watch_for_socks and self.stable_detection() is not None:
                # full fix+blacklist check is slow; only run it when a stable
                # detection exists at all
                if self.actionable_sock() is not None:
                    self.get_logger().info('[mission] sock spotted - cancelling leg')
                    gh.cancel_goal_async()
                    time.sleep(1.5)
                    return 'sock'
            time.sleep(0.25)
        return 'succeeded' if rf.result().status == 4 else 'failed'

    # ------------------------------------------------------------- costmap

    def _cost_at(self, grid, wx, wy):
        info = grid.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if not (0 <= mx < info.width and 0 <= my < info.height):
            return None  # off-grid = unknown
        v = grid.data[my * info.width + mx]
        return v

    def free_run(self, pose, bearing):
        """Metres of traversable ray from the robot along bearing.
        Unknown (-1/off-grid) beyond 0.7 m is allowed (exploration); any cell
        >= lethal_cost stops the ray."""
        grid = self.costmap
        if grid is None:
            return 0.0
        lethal = self.get_parameter('lethal_cost').value
        x0, y0, _ = pose
        r = 0.35
        while r < 2.5:
            wx = x0 + r * math.cos(bearing)
            wy = y0 + r * math.sin(bearing)
            v = self._cost_at(grid, wx, wy)
            if v is not None and v >= lethal:
                return r
            if (v is None or v < 0) and r < 0.7:
                # unknown right next to the robot: not yet mapped, don't trust
                return r
            r += 0.05
        return r

    def pick_wander_goal(self):
        """(x, y, yaw) for the next exploration leg, or None if boxed in."""
        p = self.get_parameter
        pose = self.robot_pose()
        if pose is None:
            return None
        x0, y0, th = pose
        # prefer continuing straight, then progressively wider turns
        best = None
        for off in (0.0, -math.pi / 6, math.pi / 6, -math.pi / 3, math.pi / 3,
                    -math.pi / 2, math.pi / 2, math.pi * 5 / 6, -math.pi * 5 / 6,
                    math.pi):
            b = th + off
            run = self.free_run(pose, b)
            leg = min(p('wander_leg_m').value, run - 0.45)
            if leg >= p('min_leg_m').value:
                best = (x0 + leg * math.cos(b), y0 + leg * math.sin(b), b)
                break
        return best

    # ------------------------------------------------------------- behaviors

    def pan_camera(self, angle_rad, timeout=30.0):
        """Absolute camera pan via the arm base joint. None if unavailable."""
        if not self.pan_cli.wait_for_service(timeout_sec=2.0):
            return None
        fut = self.pan_cli.call_async(
            PanCamera.Request(angle_rad=float(angle_rad)))
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                return None
            time.sleep(0.05)
        return fut.result()

    def scan_in_place(self):
        """Look around WITHOUT the wheels: pan the camera on the arm's base
        joint (+-135 deg sweep, 45 deg steps, dwell per heading so nvblox
        integrates). While parked, VO-only cuVSLAM keeps map->camera correct
        during the pan (the un-modelled pan is absorbed by the base pose,
        which is wrong mid-pan and recovers at home) — so the map AND any
        sock fixes taken mid-pan are valid; just never drive until the
        camera is back at 0. Falls back to a wheel sweep when the arm is
        unavailable. Returns True when an actionable sock was spotted."""
        p = self.get_parameter
        self.set_state('SCAN')
        step = p('scan_step_rad').value
        dwell = p('scan_dwell_s').value

        if self.pan_cli.wait_for_service(timeout_sec=2.0):
            found = False
            # Sweep left then right, stepping BACK DOWN through intermediate
            # stops — a single continuous 110 deg+ rotation costs cuVSLAM
            # ~0.36 m / 17 deg phantom drift, 45 deg legs recover to ~1 cm.
            # 1.92 rad (110 deg) max: the camera mount collides with chassis
            # hardware beyond ~120 deg; arm_bridge hard-clamps at 114.6 deg.
            # Long dwell only at NEW headings (nvblox integration); short
            # pause on the way back.
            plan = [(step, dwell), (2 * step, dwell), (1.92, dwell),
                    (2 * step, 0.8), (step, 0.8), (0.0, 0.8),
                    (-step, dwell), (-2 * step, dwell), (-1.92, dwell),
                    (-2 * step, 0.8), (-step, 0.8)]
            cur = 0.0
            try:
                for a, pause in plan:
                    r = self.pan_camera(a)
                    if r is None or not r.success:
                        self.get_logger().warn('[mission] camera pan failed')
                        break
                    cur = a
                    time.sleep(pause)
                    if self.actionable_sock() is not None:
                        found = True
                        break
            finally:
                # re-home in <=45 deg legs from wherever the sweep ended
                while abs(cur) > 1e-3:
                    cur = math.copysign(max(abs(cur) - step, 0.0), cur)
                    self.pan_camera(cur)
                    time.sleep(0.8)
                time.sleep(2.0)  # let the VO base pose settle at home
            return found

        # fallback: encoder yaw sweep (+135 then across to -135, no full 360)
        plan = [step] * 3 + [-step] * 6
        for dyaw in plan:
            if self.actionable_sock() is not None:
                return True
            res = self.move(dyaw=dyaw)
            if res is None or not res.success:
                self.get_logger().warn('[mission] scan turn failed')
                return False
            time.sleep(dwell)
        return False

    def approach(self, mp):
        """Nav2 to a standoff short of map point mp, facing it."""
        self.set_state('APPROACH')
        standoff = self.get_parameter('standoff_m').value
        pose = self.robot_pose()
        if pose is None:
            return 'failed'
        rx, ry, _ = pose
        dx, dy = mp.x - rx, mp.y - ry
        dist = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        self.get_logger().info(
            f'[mission] approach: sock at map ({mp.x:.2f},{mp.y:.2f}), '
            f'{dist:.2f} m away')
        if dist <= standoff + 0.10:
            return 'succeeded'
        gx = mp.x - standoff * math.cos(yaw)
        gy = mp.y - standoff * math.sin(yaw)
        return self.nav_to(gx, gy, yaw)

    def creep(self):
        """Encoder hops until the freshest fix is inside the pick window."""
        p = self.get_parameter
        self.set_state('CREEP')
        for i in range(8):
            cam, mp = self.sock_fix()
            if cam is None:
                self.get_logger().warn('[mission] creep: fix lost')
                return 'lost'
            rng = math.hypot(cam.x, cam.z)
            self.get_logger().info(
                f'[mission] creep{i}: cam ({cam.x:.2f},{cam.y:.2f},{cam.z:.2f})')
            if cam.z <= p('approach_stop_z_m').value:
                return 'parked'
            step = min(p('creep_hop_max_m').value,
                       max(0.08, rng - p('creep_margin_m').value))
            res = self.move(dx=step * cam.z / rng, dy=step * (-cam.x) / rng)
            if res is None or not res.success:
                return 'drive_failed'
            time.sleep(1.5)
        return 'parked'

    def do_pick(self, mp):
        """Arm pick (parked) or simulated pick; blacklist the spot either way."""
        self.set_state('PICK')
        picked = False
        if self.get_parameter('simulate_pick').value:
            self.get_logger().info(
                f'[mission] PICK (simulated) at map ({mp.x:.2f},{mp.y:.2f}) - '
                'arm disabled, marking collected')
            time.sleep(3.0)
            picked = True
        else:
            if self.arm_active.wait_for_service(timeout_sec=3.0):
                self.arm_active.call(SetBool.Request(data=True))
                t0 = time.monotonic()
                start = self.pick_events
                while time.monotonic() - t0 < 45.0:
                    if self.pick_events > start:
                        picked = True
                        break
                    time.sleep(0.5)
                self.arm_active.call(SetBool.Request(data=False))
            else:
                self.get_logger().warn('[mission] arm service missing - simulating')
                picked = True
        self.blacklist.append((mp.x, mp.y))
        # clear the stability window so the same frames don't re-trigger
        with self._lock:
            self.det_times = []
        return picked

    # ------------------------------------------------------------- main loop

    def run(self):
        p = self.get_parameter
        log = self.get_logger()
        for cli, name in ((self.drive, 'drive_relative'),
                          (self.get_3d, 'get_3d_pose')):
            if not cli.wait_for_service(timeout_sec=15.0):
                log.error(f'[mission] required service {name} missing')
                return 1
        # arm_bridge boots with its autonomous pick loop ACTIVE; suppress it —
        # the arm may only pick while the base is parked (do_pick re-enables)
        if self.arm_active.wait_for_service(timeout_sec=3.0):
            self.arm_active.call(SetBool.Request(data=False))
            log.info('[mission] arm autonomous loop suppressed until PICK')
        # wait for localization
        t0 = time.monotonic()
        while self.robot_pose() is None:
            if time.monotonic() - t0 > 30.0:
                log.error('[mission] no map->base_link TF - is VSLAM up?')
                return 1
            time.sleep(0.5)
        log.info('[mission] localized; starting')

        picks = 0
        legs = 0
        legs_since_scan = 999  # force an initial scan
        while rclpy.ok() and picks < p('max_picks').value \
                and legs < p('max_wander_legs').value:

            # 1) anything actionable right now?
            sock = self.actionable_sock()

            # 2) periodic map-building scan (skipped when a sock is in hand)
            if sock is None and legs_since_scan >= p('legs_between_scans').value:
                if self.scan_in_place():
                    # mid-pan sightings leave view when the camera re-homes;
                    # fall back to the cached map fix
                    sock = self.actionable_sock() or self.cached_sock()
                legs_since_scan = 0

            # 3) wander a leg, eyes open
            if sock is None:
                self.set_state('WANDER')
                goal = self.pick_wander_goal()
                if goal is None:
                    log.warn('[mission] boxed in / no costmap - rescanning')
                    self.scan_in_place()
                    legs_since_scan = 0
                    continue
                legs += 1
                legs_since_scan += 1
                r = self.nav_to(*goal, watch_for_socks=True)
                log.info(f'[mission] wander leg {legs}: {r}')
                if r == 'sock':
                    sock = self.actionable_sock() or self.cached_sock()
                elif r in ('failed', 'timeout'):
                    # let the map catch up, then try a different direction
                    self.scan_in_place()
                    legs_since_scan = 0
                    continue
                else:
                    continue

            if sock is None:
                continue
            cam, mp = sock

            # 4) approach -> creep -> pick
            r = self.approach(mp)
            if r != 'succeeded':
                log.warn(f'[mission] approach {r} - back to wander')
                continue
            r = self.creep()
            if r == 'parked':
                if self.do_pick(mp):
                    picks += 1
                    log.info(f'[mission] picks: {picks}/{p("max_picks").value}')
                # step back so the next leg doesn't start nose-in-floor
                self.move(dx=-0.3)
            else:
                log.warn(f'[mission] creep {r} - back to wander')

        self.set_state('DONE')
        log.info(f'[mission] done: {picks} picks, {legs} legs')
        return 0


def main():
    rclpy.init()
    node = MissionDirector()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 130
    node.set_state('STOPPED')
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    main()
