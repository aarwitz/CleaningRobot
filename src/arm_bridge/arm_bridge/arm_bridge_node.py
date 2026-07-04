#!/usr/bin/env python3
"""
Arm Bridge Node – Autonomous pick-and-place demo

Subscribes to /clothes/detected (Detection2D) from clothes_perception.
When detections are stable for N consecutive frames, calls the
/clothes_perception/get_3d_pose service to obtain a 3D camera-frame point,
converts it to robot arm coordinates via hand-eye calibration, and executes
a serial pick-place sequence on the Waveshare RoArm v2.

The pick-place loop runs continuously:
  IDLE → DETECTING → PICKING → PLACING → COOLDOWN → IDLE
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from vision_msgs.msg import Detection2D
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from behavior_manager_interfaces.srv import GetSock3D, PanCamera
import json
import math
import time
import threading

try:
    import serial
except ImportError:
    serial = None


# ── Hand-eye calibration ─────────────────────────────────────────────────
def realsense_to_robot_coords(rs_x, rs_y, rs_z):
    """Convert RealSense camera-frame coords (m) → robot arm coords (mm)."""
    above_x = (1000 * rs_z) + 100
    above_y = (-1 * 1000 * rs_x)
    above_z = (-1 * 1000 * rs_y) + 130

    just_above_x = (1000 * rs_z) + 115
    just_above_y = (-1 * 1000 * rs_x) + 25
    just_above_z = (-1 * 1000 * rs_y) - 40

    return {
        'above_pick': {'x': above_x, 'y': above_y, 'z': above_z},
        'just_above_pick': {'x': just_above_x, 'y': just_above_y, 'z': just_above_z},
        'pick_point': {'x': just_above_x, 'y': just_above_y, 'z': just_above_z - 115},
    }


class ArmBridgeNode(Node):
    def __init__(self):
        super().__init__('arm_bridge_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('enable_arm', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('stable_frames', 4)
        self.declare_parameter('stable_max_drift_px', 40.0)
        self.declare_parameter('cooldown_s', 5.0)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 0.60)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.enable_arm = self.get_parameter('enable_arm').value
        self.dry_run = self.get_parameter('dry_run').value
        self.stable_frames = self.get_parameter('stable_frames').value
        self.stable_max_drift = self.get_parameter('stable_max_drift_px').value
        self.cooldown_s = self.get_parameter('cooldown_s').value
        self.min_depth = self.get_parameter('min_depth_m').value
        self.max_depth = self.get_parameter('max_depth_m').value

        # ── State ─────────────────────────────────────────────────────────
        self.ser = None
        self.ser_lock = threading.Lock()
        self.busy = False
        self.last_pick_time = 0.0
        self.consecutive_detections = 0
        self.prev_cx = 0.0
        self.prev_cy = 0.0
        self.pick_count = 0
        # When False, the autonomous detection→pick loop is suppressed and the
        # arm only picks on an explicit execute_pick request. The behavior
        # manager toggles this with /arm_bridge/set_active so the arm runs only
        # in PERCEPTION mode; left True so the node still works standalone.
        self.active = True
        self.latest_detection = None

        # ── Serial ────────────────────────────────────────────────────────
        if self.enable_arm and not self.dry_run:
            self._init_serial()
        elif self.dry_run:
            self.get_logger().info('DRY-RUN mode – arm commands will be logged only')
        else:
            self.get_logger().info('Arm disabled (enable_arm=False)')

        # ── ROS interfaces ────────────────────────────────────────────────
        cb_group = ReentrantCallbackGroup()

        self.detection_sub = self.create_subscription(
            Detection2D,
            '/clothes/detected',
            self._detection_cb,
            10,
            callback_group=cb_group,
        )

        self.get_3d_client = self.create_client(
            GetSock3D,
            '/clothes_perception/get_3d_pose',
            callback_group=cb_group,
        )

        # Behavior-manager coordination: gate the autonomous loop, and a
        # blocking pick trigger that returns real success/failure.
        self.set_active_srv = self.create_service(
            SetBool, '/arm_bridge/set_active', self._set_active_cb,
            callback_group=cb_group,
        )
        self.execute_pick_srv = self.create_service(
            Trigger, '/arm_bridge/execute_pick', self._execute_pick_cb,
            callback_group=cb_group,
        )
        # Camera pan: the RealSense is mounted on the BASE joint, so rotating
        # joint 1 pans the camera without touching the wheels. Range +-180 deg.
        self.pan_srv = self.create_service(
            PanCamera, '/arm_bridge/pan_camera', self._pan_camera_cb,
            callback_group=cb_group,
        )
        self.pan_angle = 0.0

        # Status publisher for web viewer / debugging
        self.status_pub = self.create_publisher(String, '/arm/status', 10)

        self.get_logger().info(
            f'Arm bridge ready | port={self.serial_port} '
            f'dry_run={self.dry_run} stable_frames={self.stable_frames} '
            f'cooldown={self.cooldown_s}s depth=[{self.min_depth},{self.max_depth}]m'
        )

    # ── Serial helpers ────────────────────────────────────────────────────
    def _init_serial(self):
        if serial is None:
            self.get_logger().error('pyserial not installed – arm will not move')
            return
        try:
            self.ser = serial.Serial(self.serial_port, baudrate=self.baud_rate, dsrdtr=None)
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            time.sleep(0.1)
            self.get_logger().info(f'Connected to arm on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Serial connect failed: {e}')
            self.ser = None

    def _send(self, cmd: dict, delay: float = 0.0) -> bool:
        line = json.dumps(cmd, separators=(',', ':'))
        if self.dry_run or self.ser is None:
            self.get_logger().info(f'[dry-run] {line}  (wait {delay}s)')
            time.sleep(delay)
            return True
        try:
            with self.ser_lock:
                self.ser.write(line.encode('utf-8') + b'\n')
            self.get_logger().debug(f'Sent: {line}')
            time.sleep(delay)
            return True
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')
            return False

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(status)

    # ── Detection callback ────────────────────────────────────────────────
    def _detection_cb(self, msg: Detection2D):
        # Always keep the most recent detection so an explicit execute_pick
        # request has a fresh target even when the autonomous loop is off.
        self.latest_detection = msg

        if self.busy:
            return

        # Autonomous loop suppressed unless explicitly activated
        if not self.active:
            return

        # Cooldown guard
        if time.monotonic() - self.last_pick_time < self.cooldown_s:
            return

        cx = msg.bbox.center.position.x
        cy = msg.bbox.center.position.y
        score = msg.results[0].hypothesis.score if msg.results else 0.0

        # Stability check: has the bbox center stayed close?
        drift = math.hypot(cx - self.prev_cx, cy - self.prev_cy)
        if drift < self.stable_max_drift:
            self.consecutive_detections += 1
        else:
            self.consecutive_detections = 1

        self.prev_cx = cx
        self.prev_cy = cy

        if self.consecutive_detections >= self.stable_frames:
            # Lock out further detections BEFORE spawning the thread
            self.busy = True
            self.consecutive_detections = 0
            # Snapshot the detection so the thread uses exactly this one
            pick_detection = msg
            self._publish_status(
                f'Stable detection (score={score:.2f}) – requesting 3D pose'
            )
            # Kick off pick in a thread so we don't block the executor
            threading.Thread(
                target=self._pick_pipeline,
                args=(pick_detection,),
                daemon=True,
            ).start()

    # ── Pick pipeline (runs in its own thread) ────────────────────────────
    def _pick_pipeline(self, detection: Detection2D) -> bool:
        # self.busy is already True (set by the caller before invocation)
        try:
            # 1. Request 3D pose from clothes_perception
            point = self._request_3d_pose(detection)
            if point is None:
                self._publish_status('3D pose request failed – returning to IDLE')
                return False

            rs_x = point.point.x
            rs_y = point.point.y
            rs_z = point.point.z

            # Depth sanity check
            if not (self.min_depth <= rs_z <= self.max_depth):
                self._publish_status(
                    f'Depth {rs_z:.3f}m outside [{self.min_depth},{self.max_depth}] – skipping'
                )
                return False

            self._publish_status(
                f'Picking at camera ({rs_x:.3f}, {rs_y:.3f}, {rs_z:.3f})m'
            )

            # 2. Execute pick
            if not self._execute_pick(rs_x, rs_y, rs_z):
                self._publish_status('Pick sequence FAILED')
                return False

            self.pick_count += 1
            self._publish_status(f'Pick #{self.pick_count} complete – placing')

            # 3. Execute place (open gripper + return home)
            self._execute_place()
            self._publish_status(
                f'Place complete – cooldown {self.cooldown_s}s'
            )
            return True

        except Exception as e:
            self._publish_status(f'Pick pipeline error: {e}')
            return False
        finally:
            self.last_pick_time = time.monotonic()
            # Reset stability state so the first detection after cooldown
            # cannot match stale pre-pick coordinates
            self.consecutive_detections = 0
            self.prev_cx = 0.0
            self.prev_cy = 0.0
            # place/home returns the base joint (and the camera on it) to 0
            self.pan_angle = 0.0
            self.busy = False

    # ── Behavior-manager coordination services ────────────────────────────
    def _set_active_cb(self, request, response):
        """Enable/disable the autonomous detection→pick loop."""
        self.active = request.data
        state = 'active' if self.active else 'idle'
        self._publish_status(f'Arm autonomous loop {state}')
        response.success = True
        response.message = f'arm {state}'
        return response

    def _execute_pick_cb(self, request, response):
        """Blocking pick of the latest detection; returns real success.

        Lets the behavior manager delegate PICK to the arm (the sole owner of
        the serial port) instead of simulating it. Runs synchronously in this
        callback's thread (MultiThreadedExecutor + ReentrantCallbackGroup).
        """
        if self.busy:
            response.success = False
            response.message = 'arm busy'
            return response
        if self.latest_detection is None:
            response.success = False
            response.message = 'no detection available'
            return response

        self.busy = True
        ok = self._pick_pipeline(self.latest_detection)
        response.success = ok
        response.message = 'pick complete' if ok else 'pick failed'
        return response

    def _request_3d_pose(self, detection: Detection2D):
        if not self.get_3d_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('get_3d_pose service unavailable')
            return None

        req = GetSock3D.Request()
        req.detection = detection
        future = self.get_3d_client.call_async(req)

        # Block this thread until service responds (max 5s)
        start = time.monotonic()
        while not future.done():
            time.sleep(0.05)
            if time.monotonic() - start > 5.0:
                self.get_logger().error('get_3d_pose service timed out')
                return None

        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().warn(
                f'3D pose failed: {resp.message if resp else "no response"}'
            )
            return None

        return resp.point

    # ── Camera pan (base joint) ───────────────────────────────────────────
    def _pan_camera_cb(self, request, response):
        """Rotate BASE_JOINT (joint 1) to an absolute angle — pans the
        RealSense mounted on it. Open-loop: wait out the motion, then reply.
        Refused while a pick sequence owns the arm."""
        if self.busy:
            response.success = False
            response.message = 'arm busy (pick in progress)'
            return response
        # +-114.6 deg HARD LIMIT: beyond ~120 deg the camera mount collides
        # with other hardware on the chassis (operator constraint 2026-07-04).
        angle = max(-2.0, min(2.0, float(request.angle_rad)))
        delta = abs(angle - self.pan_angle)
        # spd is servo steps/s (4096 = one rev). 100 ~= 8.8 deg/s: at 200 a
        # +-110 deg sweep cost cuVSLAM ~0.36 m / 17 deg of phantom pose
        # drift (measured 2026-07-04); slower pans keep feature lock.
        spd = 100
        # firmware generations disagree on the key name (rad vs radian);
        # send both — unknown keys are ignored.
        ok = self._send({'T': 101, 'joint': 1, 'rad': angle, 'radian': angle,
                         'spd': spd, 'acc': 5},
                        delay=delta * (4096 / (2 * math.pi)) / spd + 0.8)
        if ok:
            self.pan_angle = angle
        response.success = ok
        response.message = f'pan at {math.degrees(self.pan_angle):.0f} deg'
        return response

    # ── Arm sequences ─────────────────────────────────────────────────────
    def _execute_pick(self, rs_x, rs_y, rs_z) -> bool:
        coords = realsense_to_robot_coords(rs_x, rs_y, rs_z)
        above = coords['above_pick']
        just_above = coords['just_above_pick']
        pick = coords['pick_point']

        self.get_logger().info(
            f'Robot coords: above=({above["x"]:.0f},{above["y"]:.0f},{above["z"]:.0f}) '
            f'pick=({pick["x"]:.0f},{pick["y"]:.0f},{pick["z"]:.0f})'
        )

        steps = [
            # Open gripper + move above target
            ({'T': 104, 'x': above['x'], 'y': above['y'], 'z': above['z'],
              't': 0, 'spd': 0.25}, 2.0),
            # Lower to just above
            ({'T': 104, 'x': just_above['x'], 'y': just_above['y'], 'z': just_above['z'],
              't': 0, 'spd': 0.25}, 2.0),
            # Lower to pick point + close gripper
            ({'T': 104, 'x': pick['x'], 'y': pick['y'], 'z': pick['z'],
              't': 3.14, 'spd': 0.25}, 4.0),
            # Lift back up with object
            ({'T': 104, 'x': above['x'], 'y': above['y'], 'z': above['z'],
              't': 3.14, 'spd': 0.25}, 3.0),
        ]

        for i, (cmd, delay) in enumerate(steps, 1):
            self.get_logger().info(f'Pick step {i}/{len(steps)}')
            if not self._send(cmd, delay):
                return False
        return True

    def _execute_place(self):
        # Open gripper to release, then return toward home
        steps = [
            # Open gripper at current (above) position
            ({'T': 104, 'x': 200, 'y': 0, 'z': 200, 't': 0, 'spd': 0.25}, 3.0),
            # Return to neutral home pose
            ({'T': 104, 'x': 235, 'y': 0, 'z': 235, 't': 0, 'spd': 0.25}, 2.0),
        ]
        for i, (cmd, delay) in enumerate(steps, 1):
            self.get_logger().info(f'Place step {i}/{len(steps)}')
            self._send(cmd, delay)

    # ── Cleanup ───────────────────────────────────────────────────────────
    def destroy_node(self):
        if self.ser:
            try:
                self._send({'T': 104, 'x': 235, 'y': 0, 'z': 235, 't': 0, 'spd': 0.25}, 1.0)
                self.ser.close()
                self.get_logger().info('Serial closed')
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmBridgeNode()
    executor = rclpy.executors.MultiThreadedExecutor()
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
