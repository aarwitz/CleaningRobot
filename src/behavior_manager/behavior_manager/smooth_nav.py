#!/usr/bin/env python3
"""Smooth planned navigation: occupancy map -> A* -> turn-and-glide follower.

Receding-horizon, SLAM-free: builds a 2D occupancy grid from D455 depth in the
encoder dead-reckon frame, plans A* around obstacles, shortcuts the path to a
few long straight segments, and drives each as yaw-to-heading (0.4 rad/s) then
one continuous forward glide — all four mecanum wheels at equal speed, the
smoothest motion this base makes. A watchdog aborts a glide via
/motor_controller/emergency_stop if the lane closes. Map updates only while
stationary (pose is only trusted at stops). Diagonal/strafe vector moves are
avoided: mecanum diagonals run one wheel pair near the deadband and look like
dragging (user-observed 2026-07-02).

Run: python3 smooth_nav.py --ros-args -p goal_x:=2.5 -p goal_y:=0.0
Goal is in the START frame: x = where the robot initially faces, y = left.
"""

import heapq
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from behavior_manager_interfaces.srv import DriveRelative

CELL = 0.10           # m per grid cell
GRID = 240            # cells per side (24 m x 24 m, robot starts centered)
MAX_MAP_RANGE = 1.5   # m — beyond this the floor bleeds into the depth band
OCC_THRESH = 2        # log-odds counter above which a cell is an obstacle


class SmoothNav(Node):
    def __init__(self):
        super().__init__('smooth_nav')
        self.declare_parameter('goal_x', 2.5)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tol_m', 0.20)
        self.declare_parameter('robot_radius_m', 0.24)
        self.declare_parameter('segment_max_m', 1.5)
        # Last-ditch only (~4 cm stopping distance at glide speed). 0.45 killed
        # legitimate short glides near furniture and in doorways — collision
        # avoidance is the inflated planner's job, not the watchdog's.
        self.declare_parameter('watchdog_stop_m', 0.30)
        self.declare_parameter('max_steps', 30)
        self.declare_parameter('scan_first', True)

        self.pose = [0.0, 0.0, 0.0]     # x, y, th in start frame
        self.depth = None
        self.depth_stamp = 0.0
        self.fx = None
        self.cx = None
        # occupancy counters: >0 leans obstacle, <0 leans free
        self.grid = np.zeros((GRID, GRID), dtype=np.int8)
        self.moving = False

        cb = ReentrantCallbackGroup()
        self.create_subscription(Image, '/camera/depth/image_rect_raw',
                                 self._depth_cb, 1, callback_group=cb)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                 self._info_cb, 1, callback_group=cb)
        self.drive = self.create_client(DriveRelative, '/motor_controller/drive_relative',
                                        callback_group=cb)
        self.estop = self.create_client(Trigger, '/motor_controller/emergency_stop',
                                        callback_group=cb)

    def _info_cb(self, msg):
        if self.fx is None:
            self.fx, self.cx = msg.k[0], msg.k[2]
            self.get_logger().info(f'depth intrinsics fx={self.fx:.1f} cx={self.cx:.1f}')

    def _depth_cb(self, msg):
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        self.depth = d.astype(np.float32) / 1000.0
        self.depth_stamp = time.monotonic()
        # mid-glide collision watchdog: forward glides face the camera down
        # the driven lane, so the center band guards the motion
        if self.moving:
            h, w = self.depth.shape
            band = self.depth[int(0.20 * h):int(0.55 * h), int(0.33 * w):int(0.67 * w)]
            v = band[band > 0.05]
            if v.size > 50 and float(np.percentile(v, 5)) < self.get_parameter('watchdog_stop_m').value:
                self.get_logger().warn('[watchdog] lane closing — emergency stop')
                self.moving = False
                self.estop.call_async(Trigger.Request())

    # --- grid helpers ---------------------------------------------------------

    def _cell(self, x, y):
        return (int(round(x / CELL)) + GRID // 2, int(round(y / CELL)) + GRID // 2)

    def _in_grid(self, i, j):
        return 0 <= i < GRID and 0 <= j < GRID

    def _mark(self, i, j, delta, lo=-8, hi=8):
        if self._in_grid(i, j):
            self.grid[i, j] = max(lo, min(hi, int(self.grid[i, j]) + delta))

    def update_map(self):
        """Project the reliable depth band into the grid: per image column,
        nearest obstacle distance; free-mark the ray, obstacle-mark the hit.
        Only call while stationary."""
        if self.depth is None or self.fx is None:
            return
        d = self.depth
        h, w = d.shape
        x0, y0, th = self.pose
        band = d[int(0.20 * h):int(0.55 * h), :]
        for u in range(4, w - 4, 8):
            col = band[:, u - 2:u + 3]
            v = col[col > 0.05]
            if v.size < 10:
                continue
            z = float(np.percentile(v, 5))
            hit = z <= MAX_MAP_RANGE
            reach = min(z, MAX_MAP_RANGE)
            # camera-frame lateral offset (right +) -> robot frame (y left)
            bearing = math.atan2((u - self.cx) / self.fx, 1.0)
            wth = th - bearing
            # free cells along the ray, stopping short of the hit
            steps = int((reach - (0.5 * CELL if hit else 0.0)) / (CELL * 0.8))
            for s in range(1, max(steps, 1)):
                r = s * CELL * 0.8
                self._mark(*self._cell(x0 + r * math.cos(wth), y0 + r * math.sin(wth)), -1)
            if hit:
                self._mark(*self._cell(x0 + z * math.cos(wth), y0 + z * math.sin(wth)), +3)
        # the robot's own footprint is definitionally free
        ci, cj = self._cell(x0, y0)
        for di in (-2, -1, 0, 1, 2):
            for dj in (-2, -1, 0, 1, 2):
                self._mark(ci + di, cj + dj, -2)

    def _inflated(self):
        occ = self.grid >= OCC_THRESH
        r = int(math.ceil(self.get_parameter('robot_radius_m').value / CELL))
        inf = occ.copy()
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r and (di or dj):
                    inf |= np.roll(np.roll(occ, di, 0), dj, 1)
        return inf

    # --- planning -------------------------------------------------------------

    def _planning_grid(self):
        """Inflated obstacles, with the robot's own footprint force-cleared —
        the robot physically occupies that space, so it is free by definition
        (without this it can neither plan nor shortcut its way OUT of a tight
        spot; both bugs bit in early runs)."""
        blocked = self._inflated()
        start = self._cell(*self.pose[:2])
        r = int(math.ceil(self.get_parameter('robot_radius_m').value / CELL))
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if self._in_grid(start[0] + di, start[1] + dj):
                    blocked[start[0] + di, start[1] + dj] = False
        return blocked

    def plan(self, goal_xy, blocked):
        """A* start->goal on the planning grid; unknown counts as free."""
        start = self._cell(*self.pose[:2])
        goal = self._cell(*goal_xy)
        if not (self._in_grid(*start) and self._in_grid(*goal)):
            return None
        if blocked[goal]:
            return None
        h = lambda c: math.hypot(c[0] - goal[0], c[1] - goal[1])
        openq = [(h(start), 0.0, start, None)]
        came, cost = {}, {start: 0.0}
        while openq:
            _, g, cur, parent = heapq.heappop(openq)
            if cur in came:
                continue
            came[cur] = parent
            if cur == goal:
                path = []
                while cur:
                    path.append(cur)
                    cur = came[cur]
                return path[::-1]
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (cur[0] + di, cur[1] + dj)
                if not self._in_grid(*nxt) or blocked[nxt] or nxt in came:
                    continue
                ng = g + math.hypot(di, dj)
                if ng < cost.get(nxt, 1e9):
                    cost[nxt] = ng
                    heapq.heappush(openq, (ng + h(nxt), ng, nxt, cur))
        return None

    def _line_clear(self, a, b, blocked):
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
        for s in range(n + 1):
            t = s / max(n, 1)
            i = int(round(a[0] + t * (b[0] - a[0])))
            j = int(round(a[1] + t * (b[1] - a[1])))
            if not self._in_grid(i, j) or blocked[i, j]:
                return False
        return True

    def shortcut(self, path, blocked):
        """Farthest waypoint reachable in a straight line -> one long segment."""
        for k in range(len(path) - 1, 0, -1):
            if self._line_clear(path[0], path[k], blocked):
                return path[k]
        return path[1] if len(path) > 1 else path[0]

    # --- motion ---------------------------------------------------------------

    def _call_drive(self, dx=0.0, dy=0.0, dyaw=0.0, timeout=40.0):
        if not self.drive.wait_for_service(timeout_sec=3.0):
            return None
        req = DriveRelative.Request()
        req.dx, req.dy, req.dyaw = float(dx), float(dy), float(dyaw)
        fut = self.drive.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                return None
            time.sleep(0.05)
        res = fut.result()
        if res is not None:
            x, y, th = self.pose
            th2 = th + res.actual_dyaw
            c, s = math.cos(th2), math.sin(th2)
            self.pose = [x + c * res.actual_dx - s * res.actual_dy,
                         y + s * res.actual_dx + c * res.actual_dy,
                         math.atan2(math.sin(th2), math.cos(th2))]
        return res

    def turn_to(self, heading):
        dyaw = math.atan2(math.sin(heading - self.pose[2]),
                          math.cos(heading - self.pose[2]))
        while abs(dyaw) > 0.12:
            step = max(-1.5, min(1.5, dyaw))
            res = self._call_drive(dyaw=step)
            if res is None or not res.success:
                return False
            dyaw = math.atan2(math.sin(heading - self.pose[2]),
                              math.cos(heading - self.pose[2]))
        return True

    def glide(self, dist):
        """One continuous forward move with the depth watchdog armed."""
        self.moving = True
        try:
            res = self._call_drive(dx=dist)
        finally:
            self.moving = False
        return res

    # --- mission ----------------------------------------------------------------

    def run(self):
        p = lambda n: self.get_parameter(n).value
        log = self.get_logger()
        goal = (p('goal_x'), p('goal_y'))
        t0 = time.monotonic()
        while self.fx is None or self.depth is None:
            if time.monotonic() - t0 > 10:
                return 'no_camera'
            time.sleep(0.2)

        if p('scan_first'):
            log.info('[nav] initial 360 scan')
            for k in range(6):
                self.update_map()
                res = self._call_drive(dyaw=math.pi / 3)
                if res is None or not res.success:
                    return 'drive_failed'
                time.sleep(0.6)
            self.update_map()

        for step in range(p('max_steps')):
            self.update_map()
            dx = goal[0] - self.pose[0]
            dy = goal[1] - self.pose[1]
            dist = math.hypot(dx, dy)
            log.info(f'[nav] step {step + 1}: pose ({self.pose[0]:.2f},{self.pose[1]:.2f},'
                     f'{math.degrees(self.pose[2]):.0f}deg) goal dist {dist:.2f}')
            if dist <= p('goal_tol_m'):
                log.info('[nav] GOAL REACHED')
                return 'goal'
            blocked = self._planning_grid()
            path = self.plan(goal, blocked)
            if path is None:
                # decay the map (stale/false obstacles fade) and retry rather
                # than giving up — a couple of bad marks shouldn't end a run
                log.warn('[nav] no path — decaying map and retrying')
                self.grid = (self.grid // 2).astype(np.int8)
                blocked = self._planning_grid()
                path = self.plan(goal, blocked)
                if path is None:
                    log.warn('[nav] still no path to goal')
                    return 'no_path'
            wp = self.shortcut(path, blocked)
            wx = (wp[0] - GRID // 2) * CELL
            wy = (wp[1] - GRID // 2) * CELL
            seg = min(math.hypot(wx - self.pose[0], wy - self.pose[1]), p('segment_max_m'))
            if seg < 0.05:
                log.warn('[nav] degenerate segment — replanning after rescan')
                continue
            heading = math.atan2(wy - self.pose[1], wx - self.pose[0])
            log.info(f'[nav] segment: turn to {math.degrees(heading):.0f}deg, glide {seg:.2f} m')
            if not self.turn_to(heading):
                return 'drive_failed'
            time.sleep(0.5)  # fresh frame after the turn
            self.update_map()
            res = self.glide(seg)
            if res is None:
                return 'drive_failed'
            if not res.success:
                if 'exceeds' in (res.message or ''):
                    # parameter rejection, not a physical block — do NOT poison
                    # the map (this exact mistake sent it the wrong way once)
                    log.error(f'[nav] glide rejected: {res.message}')
                    return 'drive_failed'
                # watchdog e-stop: the robot now faces the obstacle, so the
                # next update_map() records it — no manual painting (manual
                # +6 marks once sealed the robot into its own map)
                log.warn(f'[nav] glide stopped early ({res.message})')
            time.sleep(0.5)
        return 'step_budget'


def main():
    rclpy.init()
    node = SmoothNav()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        result = node.run()
        node.get_logger().info(f'[nav] finished: {result}')
        occ = int((node.grid >= OCC_THRESH).sum())
        free = int((node.grid <= -OCC_THRESH).sum())
        node.get_logger().info(f'[nav] map: {occ} obstacle cells, {free} free cells')
        np.save('/tmp/nav_grid.npy', node.grid)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
