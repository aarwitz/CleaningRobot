#!/usr/bin/env python3
"""Encoder-based autonomous sock-collection mission loop.

SLAM-free apartment mission: WANDER (depth-gated forward hops + turns toward
clearance) -> on stable sock detection APPROACH (crab hops toward the 3D fix)
-> park inside the arm's pick window and WAIT for arm_bridge's autonomous
pick loop to grab it -> resume wandering. All base motion goes through the
validated closed-loop /motor_controller/drive_relative service (encoder
odometry). No Nav2, no cuVSLAM.

Companion processes (all standalone): motor_controller_node (I2C owner),
clothes_perception (relay + get_3d_pose), arm_bridge (active autonomous pick
loop, depth gate ~[0.15,0.5] m). behavior_manager must NOT be running — it
toggles /arm_bridge/set_active on its mode transitions and would fight this
loop for the arm.

Run standalone (supervised):
  python3 encoder_wander.py --ros-args -p max_picks:=3

Depth-band geometry (measured 2026-07-02, camera low + slightly tilted down):
rows 20-55%% of the image see the forward scene at >=1.6 m when clear and
never see the near floor, so any reading < ~1.2 m there is a real obstacle.
The homed RoArm sits below the camera FOV and needs no masking. Floor socks
sit below the band and never block the lane gate.

Known depth bias: the adaptive floor-depth estimate reads SHORT at grazing
angles (it catches floor in front of the sock), so a single approach pass
tends to stop early. The nudge loop after convergence covers this: small
forward hops until the arm's own depth gate accepts the target.
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2D

from behavior_manager_interfaces.srv import DriveRelative, GetSock3D


class SockMission(Node):
    def __init__(self):
        super().__init__('encoder_wander')
        self.declare_parameter('max_picks', 3)          # end mission after this many picks
        self.declare_parameter('wander_budget', 12)     # moves+turns per wander leg
        self.declare_parameter('hop_max_m', 0.5)
        self.declare_parameter('hop_min_m', 0.15)
        # 0.7 demanded a 0.95m lane and the robot just spun in tight areas;
        # 0.45 still leaves ~1.5 robot-lengths beyond every hop
        self.declare_parameter('clear_margin_m', 0.45)
        self.declare_parameter('turn_rad', 0.5)         # per-turn yaw (~29 deg), keep <=0.6
        self.declare_parameter('max_consecutive_turns', 7)
        self.declare_parameter('settle_s', 1.5)
        # Park threshold: pick at cam z=0.306 grabbed, z=0.37 fell SHORT (arm
        # x=485mm is edge-of-reach). Park at <=0.35 so the grasp lands inside
        # the proven envelope; D455 depth still valid down to ~0.30.
        self.declare_parameter('approach_stop_z_m', 0.35)
        self.declare_parameter('approach_margin_m', 0.30)   # aim short of the sock by this
        self.declare_parameter('pick_wait_s', 45.0)     # arm pick+place takes ~17 s
        self.declare_parameter('max_nudges', 3)         # small hops if arm gate rejects range
        self.declare_parameter('navigate_only', False)  # pure navigation test: ignore socks

        self.depth = None
        self.depth_stamp = 0.0
        self.detection = None
        self.detection_stamp = 0.0
        self.arm_status = ''
        self.arm_busy = False
        self.arm_busy_stamp = 0.0
        self.pick_events = 0
        self._lock = threading.Lock()

        # Dead-reckoned pose in the frame where the robot started at (0,0,0),
        # x forward, y left, th CCW — integrated from drive_relative actuals.
        self.pose = [0.0, 0.0, 0.0]
        # Coverage memory: visit counts per 0.5 m grid cell of the pose frame.
        self.visited = {}
        self._mark_visited()

        cb = ReentrantCallbackGroup()
        self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self._depth_cb, 1, callback_group=cb)
        self.create_subscription(
            Detection2D, '/clothes/detected', self._det_cb, 10, callback_group=cb)
        self.create_subscription(
            String, '/arm/status', self._arm_cb, 10, callback_group=cb)
        self.drive = self.create_client(DriveRelative, '/motor_controller/drive_relative',
                                        callback_group=cb)
        self.get_3d = self.create_client(GetSock3D, '/clothes_perception/get_3d_pose',
                                         callback_group=cb)
        self.arm_active = self.create_client(SetBool, '/arm_bridge/set_active',
                                             callback_group=cb)

    def set_arm_active(self, active):
        """The arm's autonomous loop must run ONLY while the base is parked —
        a mid-approach pick grabs air as the base moves out from under it
        (observed 2026-07-02)."""
        if not self.arm_active.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('[mission] set_active service unavailable')
            return False
        req = SetBool.Request()
        req.data = bool(active)
        fut = self.arm_active.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > 5.0:
                return False
            time.sleep(0.05)
        return True

    # --- callbacks ------------------------------------------------------------

    def _depth_cb(self, msg):
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        self.depth = d.astype(np.float32) / 1000.0
        self.depth_stamp = time.monotonic()

    def _det_cb(self, msg):
        self.detection = msg
        self.detection_stamp = time.monotonic()

    def _arm_cb(self, msg):
        s = msg.data
        self.arm_status = s
        if s.startswith(('Stable detection', 'Picking at', 'Pick #', 'Place step', 'Pick step')):
            self.arm_busy = True
            self.arm_busy_stamp = time.monotonic()
        if s.startswith('Place complete'):
            self.arm_busy = False
            with self._lock:
                self.pick_events += 1
        if 'skipping' in s or 'FAILED' in s or 'failed' in s or 'error' in s:
            self.arm_busy = False

    # --- perception helpers ---------------------------------------------------

    def wait_fresh_depth(self, timeout=5.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.depth is not None and time.monotonic() - self.depth_stamp < 0.5:
                return self.depth
            time.sleep(0.05)
        return None

    def fresh_detection(self, max_age=1.5, timeout=4.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.detection is not None and time.monotonic() - self.detection_stamp < max_age:
                return self.detection
            time.sleep(0.1)
        return None

    def clearances(self, depth):
        """Nearest-obstacle distance (p05 of valid depth) in left/center/right
        lanes of the reliable forward band."""
        h, w = depth.shape
        band = depth[int(0.20 * h):int(0.55 * h), :]
        out = []
        for c0, c1 in ((0.05, 0.35), (0.33, 0.67), (0.65, 0.95)):
            blk = band[:, int(c0 * w):int(c1 * w)]
            v = blk[blk > 0.05]
            out.append(float(np.percentile(v, 5)) if v.size > 50 else 0.0)
        return out  # [left, center, right]

    def sock_3d(self, detection):
        if not self.get_3d.wait_for_service(timeout_sec=3.0):
            return None
        req = GetSock3D.Request()
        req.detection = detection
        fut = self.get_3d.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > 5.0:
                return None
            time.sleep(0.05)
        res = fut.result()
        return res.point.point if res is not None and res.success else None

    # --- motion ---------------------------------------------------------------

    def wait_arm_idle(self, timeout=60.0):
        """Never move the base while the arm is mid-pick."""
        t0 = time.monotonic()
        while self.arm_busy and time.monotonic() - t0 < timeout:
            time.sleep(0.3)
        return not self.arm_busy

    def move(self, dx=0.0, dy=0.0, dyaw=0.0):
        if not self.wait_arm_idle():
            self.get_logger().error('[mission] arm stuck busy — refusing to move')
            return None
        req = DriveRelative.Request()
        req.dx, req.dy, req.dyaw = float(dx), float(dy), float(dyaw)
        if not self.drive.wait_for_service(timeout_sec=3.0):
            return None
        fut = self.drive.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > 30.0:
                return None
            time.sleep(0.1)
        res = fut.result()
        if res is not None and res.success:
            self._integrate(res.actual_dx, res.actual_dy, res.actual_dyaw)
        return res

    # --- dead reckoning + coverage ---------------------------------------------

    def _integrate(self, dx, dy, dyaw):
        """Body-frame deltas -> pose frame. Rotation applied at the pre-move
        heading (drive_relative runs yaw first, then translation, so for
        combined moves this is approximate; wander/approach never combine)."""
        x, y, th = self.pose
        th2 = th + dyaw
        c, s = math.cos(th2), math.sin(th2)
        self.pose = [x + c * dx - s * dy, y + s * dx + c * dy,
                     math.atan2(math.sin(th2), math.cos(th2))]
        self._mark_visited()
        self.get_logger().info(
            f'[pose] x={self.pose[0]:.2f} y={self.pose[1]:.2f} th={math.degrees(self.pose[2]):.0f}deg')

    def _cell(self, x, y):
        return (int(math.floor(x / 0.5)), int(math.floor(y / 0.5)))

    def _mark_visited(self):
        c = self._cell(self.pose[0], self.pose[1])
        self.visited[c] = self.visited.get(c, 0) + 1

    def _visits_toward(self, dth):
        """Visit count of the cell ~1 m ahead if we first turned by dth."""
        x, y, th = self.pose
        return self.visited.get(
            self._cell(x + math.cos(th + dth), y + math.sin(th + dth)), 0)

    # --- mission phases ---------------------------------------------------------

    def wander_leg(self):
        """Depth-gated exploration until a sock shows up. Returns 'detection',
        'budget', 'boxed_in', or a failure string."""
        p = lambda name: self.get_parameter(name).value
        log = self.get_logger()
        turns = 0
        stuck = False
        turn_dir = 0  # commit to one direction while blocked (no L/R dither)
        for i in range(p('wander_budget')):
            if (not p('navigate_only') and self.detection is not None
                    and time.monotonic() - self.detection_stamp < 1.0):
                score = (self.detection.results[0].hypothesis.score
                         if self.detection.results else 0.0)
                log.info(f'[wander] sock detected (conf {score:.2f}) → approach')
                return 'detection'
            depth = self.wait_fresh_depth()
            if depth is None:
                log.error('[wander] no fresh depth — aborting')
                return 'no_depth'
            left, center, right = self.clearances(depth)
            log.info(f'[wander] {i+1}/{p("wander_budget")} clear L={left:.2f} C={center:.2f} R={right:.2f}')
            if not stuck and center - p('clear_margin_m') >= p('hop_min_m'):
                hop = min(p('hop_max_m'), center - p('clear_margin_m'))
                res = self.move(dx=hop)
                if res is None or not res.success:
                    log.error(f'[wander] drive failed: {getattr(res, "message", "timeout")}')
                    return 'drive_failed'
                # Slip/stall: encoders say we moved far less than commanded —
                # something is physically resisting that depth couldn't see
                if abs(res.actual_dx) < 0.4 * hop:
                    log.warn(f'[wander] STUCK: commanded {hop:.2f} moved {res.actual_dx:.3f} — turning away')
                    stuck = True
                else:
                    log.info(f'[wander] hop {hop:.2f} → actual {res.actual_dx:.3f}')
                turns = 0
                turn_dir = 0
            else:
                turns += 1
                if turns > p('max_consecutive_turns'):
                    log.warn('[wander] boxed in — stopping')
                    return 'boxed_in'
                # Direction: once turning, keep going the same way until a hop
                # lands (L/R dithering trapped it in corners). First turn of a
                # block: clearance if one side is clearly better, else coverage.
                if turn_dir == 0:
                    if abs(left - right) > 0.4:
                        to_left = left > right
                    else:
                        to_left = self._visits_toward(1.0) <= self._visits_toward(-1.0)
                    turn_dir = 1 if to_left else -1
                dyaw = turn_dir * p('turn_rad')
                res = self.move(dyaw=dyaw)
                if res is None or not res.success:
                    log.error(f'[wander] turn failed: {getattr(res, "message", "timeout")}')
                    return 'drive_failed'
                log.info(f'[wander] turn {"L" if dyaw > 0 else "R"} → actual {res.actual_dyaw:.3f} rad')
                stuck = False
            time.sleep(p('settle_s'))
        return 'budget'

    def approach(self):
        """Crab-hop toward the freshest sock 3D fix until inside the arm's
        pick window. Returns 'parked', 'lost', or a failure string."""
        p = lambda name: self.get_parameter(name).value
        log = self.get_logger()
        for i in range(8):
            det = self.fresh_detection()
            if det is None:
                log.warn('[approach] detection lost — back to wander')
                return 'lost'
            pt = self.sock_3d(det)
            if pt is None:
                log.warn('[approach] no 3D fix — back to wander')
                return 'lost'
            rng = math.hypot(pt.x, pt.z)
            log.info(f'[approach] sock at cam ({pt.x:.2f},{pt.y:.2f},{pt.z:.2f}) rng={rng:.2f}')
            if pt.z <= p('approach_stop_z_m'):
                log.info('[approach] inside pick window — parked')
                return 'parked'
            step = min(p('hop_max_m'), max(0.08, rng - p('approach_margin_m')))
            dx = step * pt.z / rng
            dy = step * (-pt.x) / rng
            # never outrun the lane: cap by current clearance
            depth = self.wait_fresh_depth()
            if depth is not None:
                center = self.clearances(depth)[1]
                if center - 0.30 < dx:
                    dx = max(0.0, center - 0.30)
                    if dx < 0.05:
                        # Only worth parking if the sock is already in reach;
                        # otherwise this sock sits in clutter — try another
                        if pt.z <= 0.55:
                            log.warn('[approach] lane blocked, sock in reach — parking')
                            return 'parked'
                        log.warn(f'[approach] lane blocked {pt.z - 0.42:.2f} m short of sock — skipping it')
                        return 'lost'
            res = self.move(dx=dx, dy=dy)
            if res is None or not res.success:
                log.error(f'[approach] hop failed: {getattr(res, "message", "timeout")}')
                return 'drive_failed'
            log.info(f'[approach] hop dx={dx:.2f} dy={dy:.2f} → actual ({res.actual_dx:.3f},{res.actual_dy:.3f})')
            time.sleep(p('settle_s'))
        return 'parked'  # 8 hops in — close enough, let the nudge loop finish it

    def wait_for_pick(self):
        """Sit still and let arm_bridge's loop take the shot; nudge forward a
        few cm if its depth gate keeps rejecting. Returns True on a pick."""
        p = lambda name: self.get_parameter(name).value
        log = self.get_logger()
        start_picks = self.pick_events
        for nudge in range(p('max_nudges') + 1):
            t0 = time.monotonic()
            while time.monotonic() - t0 < p('pick_wait_s'):
                if self.pick_events > start_picks:
                    log.info(f'[pick] arm reports pick complete (#{self.pick_events})')
                    return True
                if 'outside' in self.arm_status and time.monotonic() - t0 > 8.0:
                    break  # arm keeps rejecting the range — nudge closer
                time.sleep(0.5)
            if self.pick_events > start_picks:
                return True
            if nudge < p('max_nudges'):
                log.info('[pick] arm gate rejecting — nudging 0.08 m forward')
                self.set_arm_active(False)
                res = self.move(dx=0.08)
                self.set_arm_active(True)
                if res is None or not res.success:
                    return False
                time.sleep(2.0)
        log.warn('[pick] arm never picked — giving up on this sock')
        return False

    def run(self):
        p = lambda name: self.get_parameter(name).value
        log = self.get_logger()
        if p('navigate_only'):
            self.set_arm_active(False)
            result = self.wander_leg()
            log.info(f'[mission] navigation leg ended: {result}; dead-reckon pose '
                     f'x={self.pose[0]:.2f} y={self.pose[1]:.2f} th={math.degrees(self.pose[2]):.0f}deg, '
                     f'{len(self.visited)} cells visited')
            return result

        picks = 0
        legs_without_sock = 0
        self.set_arm_active(False)  # arm fires only while parked
        while picks < p('max_picks'):
            result = self.wander_leg()
            if result == 'detection':
                legs_without_sock = 0
                a = self.approach()
                if a == 'parked':
                    self.set_arm_active(True)
                    try:
                        if self.wait_for_pick():
                            picks += 1
                            log.info(f'[mission] picks so far: {picks}/{p("max_picks")}')
                    finally:
                        self.set_arm_active(False)
                    # move on either way: back away slightly so a failed/finished
                    # spot leaves the pick zone and the lane gate sees past it
                    self.move(dx=-0.25)
                    time.sleep(1.0)
                elif a != 'lost':
                    log.error(f'[mission] approach ended: {a} — stopping')
                    return a
            elif result == 'budget':
                legs_without_sock += 1
                if legs_without_sock >= 2:
                    log.info('[mission] no socks found in 2 wander legs — stopping')
                    return 'no_socks'
            else:
                log.error(f'[mission] wander ended: {result} — stopping')
                return result
        log.info(f'[mission] complete: {picks} picks')
        return 'complete'


def main():
    rclpy.init()
    node = SockMission()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        result = node.run()
        node.get_logger().info(f'[mission] finished: {result}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
