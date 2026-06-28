#!/usr/bin/env python3
"""
Behavior Manager Node - State machine for clothes-collecting robot

States: WANDER → APPROACH_CLOTHES → PICK → GO_TO_BASKET → PLACE → (RECOVER) → WANDER

Navigation (SLAM + Nav2) always runs.
Perception is gated (enabled/disabled) based on state.
Behavior manager is the ONLY component sending Nav2 goals.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from enum import Enum, auto
import time
import math
import random

from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from vision_msgs.msg import Detection2D
from behavior_manager_interfaces.srv import GetSock3D, DriveRelative
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)


class RobotState(Enum):
    """Robot behavior states"""
    DETECT = auto()
    WANDER = auto()
    APPROACH_CLOTHES = auto()
    PICK = auto()
    GO_TO_BASKET = auto()
    PLACE = auto()
    RECOVER = auto()


class OperatingMode(Enum):
    """GPU operating modes (mutually exclusive heavy pipelines).

    The Jetson Orin Nano cannot run YOLO (TensorRT) and SLAM + nvblox at full
    rate simultaneously, so the behavior manager keeps exactly one heavy GPU
    consumer active at a time:

      NAV        - navigation active (Nav2 planning + /cmd_vel), perception/arm
                   idle. The robot drives blind toward a fixed map-frame goal.
      PERCEPTION - perception + arm active, navigation paused. The robot is
                   stationary while it detects and picks.

    Note: SLAM is left running in both modes so localization/TF survive the
    switch; only the perception (YOLO) vs reconstruction (nvblox) GPU load is
    expected to be gated externally once Phase-1 hardware measurement confirms
    the real headroom (see _apply_gpu_pipeline).
    """
    NAV = auto()
    PERCEPTION = auto()


# Which operating mode each behavior state requires.
STATE_OPERATING_MODE = {
    RobotState.DETECT: OperatingMode.PERCEPTION,
    RobotState.WANDER: OperatingMode.NAV,
    RobotState.APPROACH_CLOTHES: OperatingMode.NAV,
    RobotState.PICK: OperatingMode.PERCEPTION,
    RobotState.GO_TO_BASKET: OperatingMode.NAV,
    RobotState.PLACE: OperatingMode.PERCEPTION,
    RobotState.RECOVER: OperatingMode.NAV,
}


class BehaviorManagerNode(Node):
    # ==================== State: DETECT ====================
    def enter_detect(self):
        """Enable minimal perception for detection and depth at aligned depth rate"""
        self.get_logger().info('Entering DETECT')
        # Enable perception at aligned depth rate (15 Hz)
        self.set_perception_enabled(True)
        # Optionally, set YOLO and camera to 15 Hz via service or parameter if supported
        # Reset detection state
        self.latest_detection = None
        self.clothes_target = None
        self.clothes_first_seen_time = None
        self.clothes_frame_count = 0
        # No navigation, no arm, just detection and depth

    def update_detect(self):
        """In DETECT mode, process YOLO and aligned depth as fast as possible"""
        # This function can be called at the same rate as aligned depth publishes
        # Only publish detection results and 3D centroid if available
        # No navigation or arm actions in this mode
        pass
    """
    Central behavior orchestrator
    
    Controls:
    - Nav2 goals (ONLY source of navigation commands)
    - Perception gating (enable/disable clothes detection)
    - Arm manipulation (via service calls)
    - State transitions based on environment feedback
    """
    
    def __init__(self):
        super().__init__('behavior_manager')
        
        # Parameters
        self.declare_parameter('wander_radius_m', 3.0)
        self.declare_parameter('wander_timeout_s', 30.0)
        self.declare_parameter('clothes_confidence_threshold', 0.7)
        self.declare_parameter('clothes_stable_frames_required', 5)
        self.declare_parameter('clothes_stable_time_s', 2.0)
        self.declare_parameter('grasp_offset_m', 0.30)
        self.declare_parameter('approach_stop_distance_m', 0.35)
        self.declare_parameter('goal_update_threshold_m', 0.10)
        self.declare_parameter('goal_update_max_rate_s', 2.0)
        self.declare_parameter('basket_x', 0.0)
        self.declare_parameter('basket_y', 0.0)
        self.declare_parameter('basket_z', 0.3)
        self.declare_parameter('approach_timeout_s', 60.0)
        self.declare_parameter('pick_timeout_s', 30.0)
        self.declare_parameter('wander_perception_rate_hz', 3.0)
        self.declare_parameter('approach_perception_rate_hz', 8.0)

        # Approach strategy. 'encoder' (default) drives SLAM-free via the
        # motor_controller DriveRelative service in visual-servo hops using the
        # camera-frame bearing/range to the target — cuVSLAM odometry is not
        # reliable enough in this environment for map-frame Nav2 approach. 'nav'
        # is the legacy Nav2 map-frame approach (kept as a fallback).
        self.declare_parameter('approach_mode', 'encoder')
        # Per-hop translation cap for the visual-servo approach: crab at most
        # this far toward the target, then re-detect and repeat.
        self.declare_parameter('approach_step_m', 0.30)
        # Yaw is OFF by default: this mecanum base crabs at the target instead of
        # pivoting (pivots brown out / brick the I2C driver). If a future gripper
        # needs the base square to the target, set approach_use_yaw True; the
        # per-hop yaw is then capped to approach_max_yaw_step_rad and kept gentle.
        self.declare_parameter('approach_use_yaw', False)
        self.declare_parameter('approach_max_yaw_step_rad', 0.4)
        # Master gate. When False the state machine does NOT run its per-state
        # logic (no wandering, no driving) — only state publishing and the
        # /behavior/test_drive hook stay live. Lets the node be brought up safely
        # for bring-up/wiring tests without it driving off autonomously.
        self.declare_parameter('autonomous_enabled', True)
        self.autonomous_enabled = self.get_parameter('autonomous_enabled').value

        # State
        self.current_state = RobotState.WANDER
        self.state_enter_time = time.time()
        
        self.approach_mode = self.get_parameter('approach_mode').value

        # Clothes tracking
        self.latest_detection = None  # 2D only during WANDER/APPROACH
        self.clothes_target = None  # 3D target as a map-frame PoseStamped
        self.clothes_target_cam = None  # latest 3D target as a camera-frame point
        self.clothes_first_seen_time = None
        self.clothes_frame_count = 0
        self.last_clothes_update_time = 0.0
        self.awaiting_3d = False  # a 3D-capture service call is in flight

        # Encoder visual-servo approach state
        self.approach_move_active = False   # a DriveRelative hop is in flight
        self.approach_last_hop = None       # actuals from the last hop
        self.approach_redetect_pending = False  # need a fresh 3D fix before next hop

        # TF: used to convert the camera-frame 3D point to the map frame so the
        # robot can navigate to a fixed target while perception is off (NAV mode)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_frame = 'map'

        # Navigation
        self.current_odom = None
        self.nav_goal_handle = None
        self.nav_goal_result = None
        self.last_goal_update_time = 0.0
        self.last_goal_position = None
        
        # Manipulation status
        self.pick_success = False
        self.place_success = False
        # Tri-state pick result from arm_bridge: None=pending, True/False=done
        self.pick_result = None
        
        # Service clients
        self.perception_enable_client = self.create_client(
            SetBool, '/clothes_perception/enable')
        self.perception_reset_client = self.create_client(
            Trigger, '/clothes_perception/reset_target')
        self.get_clothes_3d_client = self.create_client(
            GetSock3D, '/clothes_perception/get_3d_pose')
        # SLAM-free relative motion (encoder closed-loop) executed by
        # motor_controller_node — the approach driver in 'encoder' mode.
        self.drive_relative_client = self.create_client(
            DriveRelative, '/motor_controller/drive_relative')

        # Would be arm service clients (stubs for now)
        # self.arm_pick_client = self.create_client(...)
        # self.arm_place_client = self.create_client(...)
        
        # Action clients
        self.nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # ---- Operating-mode supervision (GPU budget) ----
        self.declare_parameter('enable_mode_switching', True)
        self.operating_mode = None  # set on first request_mode()
        self.mode_pub = self.create_publisher(String, '/robot/mode', 10)
        # Pause/resume the Nav2 lifecycle so it stops planning and emitting
        # /cmd_vel while the robot is stationary in PERCEPTION mode.
        self.nav_lifecycle_client = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        # Gate arm_bridge's autonomous pick loop so it only runs in PERCEPTION
        # mode (arm_bridge owns the serial port; see Phase 6 arm delegation).
        self.arm_active_client = self.create_client(
            SetBool, '/arm_bridge/set_active')
        # Delegate the actual pick to arm_bridge and get real success/failure.
        self.arm_execute_pick_client = self.create_client(
            Trigger, '/arm_bridge/execute_pick')
        
        # Subscribers
        self.clothes_detection_sub = self.create_subscription(
            Detection2D,
            '/clothes/detected',
            self.clothes_detection_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )

        # Bring-up/test hook: publish a Twist on /behavior/test_drive to issue a
        # single DriveRelative hop (linear.x=dx m, linear.y=dy m, angular.z=dyaw
        # rad) through the same path the approach uses — lets us validate the
        # behavior_manager -> motor_controller wiring without a clothing target.
        self.test_drive_sub = self.create_subscription(
            Twist, '/behavior/test_drive', self.test_drive_callback, 10)

        # Publishers
        self.state_pub = self.create_publisher(String, '/robot/state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State machine timer (10 Hz)
        self.state_timer = self.create_timer(0.1, self.state_machine_update)
        
        # Initialize to DETECT
        self.request_mode(STATE_OPERATING_MODE[RobotState.DETECT])
        self.enter_detect()

        self.get_logger().info('Behavior manager initialized in DETECT state')
    
    # ==================== State Machine Core ====================
    
    def state_machine_update(self):
        """Main state machine loop"""
        # Master gate: when disabled, run no state logic (no autonomous driving);
        # still publish state below and keep the /behavior/test_drive hook live.
        if not self.autonomous_enabled:
            state_msg = String()
            state_msg.data = self.current_state.name
            self.state_pub.publish(state_msg)
            return
        # Check transitions based on current state
        if self.current_state == RobotState.DETECT:
            self.update_detect()
        elif self.current_state == RobotState.WANDER:
            self.update_wander()
        elif self.current_state == RobotState.APPROACH_CLOTHES:
            self.update_approach_clothes()
        elif self.current_state == RobotState.PICK:
            self.update_pick()
        elif self.current_state == RobotState.GO_TO_BASKET:
            self.update_go_to_basket()
        elif self.current_state == RobotState.PLACE:
            self.update_place()
        elif self.current_state == RobotState.RECOVER:
            self.update_recover()
        # Publish current state
        state_msg = String()
        state_msg.data = self.current_state.name
        self.state_pub.publish(state_msg)
    
    def transition_to(self, new_state: RobotState):
        """Execute state transition"""
        old_state = self.current_state
        self.current_state = new_state
        self.state_enter_time = time.time()

        self.get_logger().info(f'State transition: {old_state.name} → {new_state.name}')

        # Apply the operating mode this state requires before its enter logic
        # runs (so navigation is resumed/paused before goals are sent).
        self.request_mode(STATE_OPERATING_MODE[new_state])

        # Call enter function
        if new_state == RobotState.WANDER:
            self.enter_wander()
        elif new_state == RobotState.APPROACH_CLOTHES:
            self.enter_approach_clothes()
        elif new_state == RobotState.PICK:
            self.enter_pick()
        elif new_state == RobotState.GO_TO_BASKET:
            self.enter_go_to_basket()
        elif new_state == RobotState.PLACE:
            self.enter_place()
        elif new_state == RobotState.RECOVER:
            self.enter_recover()
    
    # ==================== State: WANDER ====================
    
    def enter_wander(self):
        """Enable low-rate perception, start random exploration"""
        self.get_logger().info('Entering WANDER')
        
        # Enable perception at low rate
        rate_hz = self.get_parameter('wander_perception_rate_hz').value
        self.set_perception_enabled(True)
        self.get_logger().info(f'Perception enabled at {rate_hz} Hz')
        
        # Reset clothes tracking
        self.call_perception_reset()
        self.latest_detection = None
        self.clothes_target = None
        self.clothes_first_seen_time = None
        self.clothes_frame_count = 0
        self.awaiting_3d = False

        # Send random wander goal
        self.send_random_wander_goal()

    def update_wander(self):
        """Check for clothes detection or wander timeout"""
        # On a stable detection, capture the 3D target (camera→map) BEFORE
        # switching to NAV-mode approach — perception is live here, but goes
        # off the moment we transition. The transition happens in the service
        # callback once clothes_target is populated.
        if self.is_clothes_stable() and not self.awaiting_3d:
            self.capture_clothes_target()
            return

        # Don't send new wander goals while a capture is in flight
        if self.awaiting_3d:
            return

        # Check wander timeout - send new goal
        elapsed = time.time() - self.state_enter_time
        timeout = self.get_parameter('wander_timeout_s').value
        if elapsed > timeout:
            self.get_logger().info('Wander timeout - sending new goal')
            self.send_random_wander_goal()
            self.state_enter_time = time.time()
    
    def send_random_wander_goal(self):
        """Send random exploration goal around current position"""
        if self.current_odom is None:
            self.get_logger().warn('No odometry yet, cannot wander')
            return
        
        # Get current position
        curr_x = self.current_odom.pose.pose.position.x
        curr_y = self.current_odom.pose.pose.position.y
        
        # Generate random goal within radius
        radius = self.get_parameter('wander_radius_m').value
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(radius * 0.5, radius)
        
        goal_x = curr_x + distance * math.cos(angle)
        goal_y = curr_y + distance * math.sin(angle)
        
        # Create goal pose
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.send_nav_goal(goal)
        self.get_logger().info(f'Wander goal: ({goal_x:.2f}, {goal_y:.2f})')
    
    # ==================== State: APPROACH_CLOTHES ====================
    
    def enter_approach_clothes(self):
        """Cancel wander, enable moderate perception, approach clothes"""
        self.get_logger().info('Entering APPROACH_CLOTHES')

        # Cancel any existing goal
        self.cancel_nav_goal()

        # Enable perception at higher rate
        rate_hz = self.get_parameter('approach_perception_rate_hz').value
        self.set_perception_enabled(True)
        self.get_logger().info(f'Perception rate increased to {rate_hz} Hz')

        if self.approach_mode == 'encoder':
            # SLAM-free visual-servo approach: perception stays live and the
            # update loop issues DriveRelative hops from the camera-frame target.
            # PERCEPTION posture pauses Nav2 so it can't fight DriveRelative on
            # /cmd_vel; transition_to() applied NAV before this, so flip it.
            self.request_mode(OperatingMode.PERCEPTION)
            self.approach_move_active = False
            self.approach_redetect_pending = False
            self.approach_last_hop = None
            # We already have a fresh camera target from the WANDER capture.
            return

        # Legacy 'nav' approach: send the first map-frame goal.
        if self.clothes_target:
            self.send_approach_goal(self.clothes_target)
    
    def update_approach_clothes(self):
        """Drive toward the (fixed map-frame) clothes target, check arrival.

        In NAV mode perception is disabled, so the target is treated as static
        (clothes do not move) and we navigate to the captured map pose. The
        "clothes lost" check and live goal updates only apply while perception
        is actually running (PERCEPTION mode).
        """
        if self.approach_mode == 'encoder':
            self._update_approach_encoder()
            return

        perception_live = self.operating_mode == OperatingMode.PERCEPTION

        # Check if clothes lost (only meaningful while perception is running)
        if perception_live:
            time_since_update = time.time() - self.last_clothes_update_time
            if time_since_update > 5.0:
                self.get_logger().warn('Clothes lost during approach')
                self.transition_to(RobotState.RECOVER)
                return

        # Check if close enough to pick
        if self.is_near_clothes():
            self.transition_to(RobotState.PICK)
            return

        # Update goal if target moved significantly (only with live perception)
        if perception_live and self.clothes_target:
            self.update_approach_goal(self.clothes_target)
        
        # Check timeout
        elapsed = time.time() - self.state_enter_time
        timeout = self.get_parameter('approach_timeout_s').value
        if elapsed > timeout:
            self.get_logger().warn('Approach timeout')
            self.transition_to(RobotState.RECOVER)
    
    def send_approach_goal(self, target: PoseStamped):
        """Send Nav2 goal offset from clothes by grasp distance"""
        if self.current_odom is None:
            return
        
        # Compute approach pose (offset from clothes toward robot)
        grasp_offset = self.get_parameter('grasp_offset_m').value
        
        # Vector from robot to clothes
        dx = target.pose.position.x - self.current_odom.pose.pose.position.x
        dy = target.pose.position.y - self.current_odom.pose.pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < 0.01:
            return
        
        # Unit vector
        ux = dx / dist
        uy = dy / dist
        
        # Goal is grasp_offset before the clothes
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = target.pose.position.x - grasp_offset * ux
        goal.pose.position.y = target.pose.position.y - grasp_offset * uy
        goal.pose.position.z = 0.0
        
        # Orient toward clothes
        goal.pose.orientation.w = math.cos(math.atan2(dy, dx) / 2)
        goal.pose.orientation.z = math.sin(math.atan2(dy, dx) / 2)
        
        self.send_nav_goal(goal)
        self.last_goal_update_time = time.time()
        self.last_goal_position = (goal.pose.position.x, goal.pose.position.y)
    
    def update_approach_goal(self, target: PoseStamped):
        """Update goal if target moved significantly"""
        # Rate limiting
        time_since_last = time.time() - self.last_goal_update_time
        max_rate = self.get_parameter('goal_update_max_rate_s').value
        if time_since_last < max_rate:
            return
        
        # Distance threshold
        if self.last_goal_position is None:
            self.send_approach_goal(target)
            return
        
        dx = target.pose.position.x - self.last_goal_position[0]
        dy = target.pose.position.y - self.last_goal_position[1]
        dist_moved = math.sqrt(dx**2 + dy**2)
        
        threshold = self.get_parameter('goal_update_threshold_m').value
        if dist_moved > threshold:
            self.get_logger().info(f'Updating goal (moved {dist_moved:.2f}m)')
            self.send_approach_goal(target)
    
    def is_near_clothes(self) -> bool:
        """Check if robot is within grasp distance of clothes"""
        if self.clothes_target is None or self.current_odom is None:
            return False
        
        dx = self.clothes_target.pose.position.x - self.current_odom.pose.pose.position.x
        dy = self.clothes_target.pose.position.y - self.current_odom.pose.pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        
        stop_dist = self.get_parameter('approach_stop_distance_m').value
        return dist < stop_dist

    # ---------- Encoder (SLAM-free) visual-servo approach ----------

    def _update_approach_encoder(self):
        """Drive toward the clothes in DriveRelative hops using the camera-frame
        bearing/range — no SLAM. One hop at a time, re-detecting between hops so
        the approach is closed-loop on perception, not on cuVSLAM odometry."""
        # Overall timeout.
        elapsed = time.time() - self.state_enter_time
        if elapsed > self.get_parameter('approach_timeout_s').value:
            self.get_logger().warn('Approach timeout (encoder)')
            self.transition_to(RobotState.RECOVER)
            return

        # A hop or a 3D refresh is in flight — wait for it.
        if self.approach_move_active or self.awaiting_3d:
            return

        # After a hop we re-detect before computing the next one (the robot — and
        # possibly the target — moved). Also refresh if we have no target yet.
        if self.approach_redetect_pending or self.clothes_target_cam is None:
            self._request_clothes_3d_refresh()
            return

        # Lost the target for too long → recover.
        if time.time() - self.last_clothes_update_time > 5.0:
            self.get_logger().warn('Clothes lost during encoder approach')
            self.transition_to(RobotState.RECOVER)
            return

        dx, dy, dyaw, rng = self._compute_hop_from_cam(self.clothes_target_cam)

        # Close enough → hand off to PICK.
        if rng <= self.get_parameter('approach_stop_distance_m').value:
            self.get_logger().info(f'Within stop distance ({rng:.2f} m) → PICK')
            self.transition_to(RobotState.PICK)
            return

        # Nothing meaningful to command but not yet within stop distance: take a
        # minimum forward nudge toward the target to make progress.
        if abs(dx) < 0.02 and abs(dy) < 0.02 and abs(dyaw) < 0.03:
            dx = min(self.get_parameter('approach_step_m').value,
                     max(0.05, rng - self.get_parameter('grasp_offset_m').value))

        self._issue_drive_relative(dx, dy, dyaw, tag='approach')

    def _compute_hop_from_cam(self, point_cam: PointStamped):
        """Map a camera-optical-frame target point to a bounded body-frame hop.

        Optical frame is x=right, y=down, z=forward. Body: forward=+z_opt,
        left=-x_opt. This base is mecanum/holonomic, so it CRABS straight at the
        target (forward+strafe) instead of pivoting — an in-place yaw spins all
        four wheels against each other, spikes current, and browns out / bricks
        the I2C driver (see memory motor-driver-i2c-contention-kills-board). Yaw
        is therefore off by default; enable approach_use_yaw only if the gripper
        needs the base square to the target, and then it's small and gentle.
        Returns (dx, dy, dyaw, horizontal_range)."""
        x = point_cam.point.x
        z = point_cam.point.z
        rng = math.hypot(x, z)

        step = self.get_parameter('approach_step_m').value
        grasp = self.get_parameter('grasp_offset_m').value

        # Translate toward the target, stopping grasp_offset short, capped to step.
        advance = max(0.0, min(step, rng - grasp))
        if rng > 1e-3:
            dx = advance * (z / rng)        # body forward component
            dy = advance * (-x / rng)       # body left component
        else:
            dx = dy = 0.0

        dyaw = 0.0
        if self.get_parameter('approach_use_yaw').value:
            max_yaw = self.get_parameter('approach_max_yaw_step_rad').value
            bearing = math.atan2(-x, z)
            dyaw = max(-max_yaw, min(max_yaw, bearing))
        return dx, dy, dyaw, rng

    def _request_clothes_3d_refresh(self):
        """Ask clothes_perception for a fresh 3D fix on the latest detection and
        store it as the camera-frame target (no state transition)."""
        if self.latest_detection is None:
            return
        if not self.get_clothes_3d_client.service_is_ready():
            return
        self.awaiting_3d = True
        request = GetSock3D.Request()
        request.detection = self.latest_detection
        future = self.get_clothes_3d_client.call_async(request)
        future.add_done_callback(self._on_approach_3d)

    def _on_approach_3d(self, future):
        self.awaiting_3d = False
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'approach 3D refresh error: {e}')
            return
        if not response.success:
            self.get_logger().warn(
                f'approach 3D refresh failed: {response.message}',
                throttle_duration_sec=2.0)
            return
        self.clothes_target_cam = response.point
        self.last_clothes_update_time = time.time()
        self.approach_redetect_pending = False

    def _issue_drive_relative(self, dx, dy, dyaw, tag='move'):
        """Fire a single DriveRelative hop (async). Sets approach_move_active so
        the state machine waits for completion."""
        if not self.drive_relative_client.service_is_ready():
            if not self.drive_relative_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn(
                    'drive_relative service unavailable; hop dropped',
                    throttle_duration_sec=5.0)
                return
        request = DriveRelative.Request()
        request.dx = float(dx)
        request.dy = float(dy)
        request.dyaw = float(dyaw)
        self.approach_move_active = True
        self.get_logger().info(
            f'[{tag}] hop: dx={dx:.3f} dy={dy:.3f} dyaw={math.degrees(dyaw):.1f}deg')
        future = self.drive_relative_client.call_async(request)
        future.add_done_callback(lambda f: self._on_drive_relative_done(f, tag))

    def _on_drive_relative_done(self, future, tag):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'[{tag}] drive_relative error: {e}')
            self.approach_move_active = False
            return
        self.approach_last_hop = response
        self.get_logger().info(
            f'[{tag}] hop done: actual dx={response.actual_dx:.3f} '
            f'dy={response.actual_dy:.3f} dyaw={math.degrees(response.actual_dyaw):.1f}deg '
            f'success={response.success}')
        self.approach_move_active = False
        if tag == 'approach':
            # Re-detect before the next hop so the approach stays closed-loop.
            self.approach_redetect_pending = True

    def test_drive_callback(self, msg: Twist):
        """Bring-up hook: /behavior/test_drive Twist -> one DriveRelative hop
        (linear.x=dx m, linear.y=dy m, angular.z=dyaw rad)."""
        if self.approach_move_active:
            self.get_logger().warn('test_drive ignored: a hop is already active')
            return
        self._issue_drive_relative(msg.linear.x, msg.linear.y, msg.angular.z,
                                   tag='test')

    # ==================== State: PICK ====================
    
    def enter_pick(self):
        """Stop base and delegate the pick to arm_bridge.

        PERCEPTION mode (set on transition) keeps YOLO + the arm active.
        arm_bridge is the sole owner of the serial port: it does its own 3D
        lookup on the latest detection and runs the grasp, then reports real
        success via the execute_pick service.
        """
        self.get_logger().info('Entering PICK')

        # Cancel Nav2 goal and stop the base before the arm moves
        self.cancel_nav_goal()
        self.stop_base()

        # Delegate to arm_bridge; result arrives in _arm_pick_done
        self.pick_success = False
        self.pick_result = None
        self.call_arm_execute_pick()

    def call_arm_execute_pick(self):
        """Ask arm_bridge to pick the latest detection (async)."""
        if not self.arm_execute_pick_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('arm execute_pick service unavailable')
            self.pick_result = False
            return
        future = self.arm_execute_pick_client.call_async(Trigger.Request())
        future.add_done_callback(self._arm_pick_done)

    def _arm_pick_done(self, future):
        try:
            resp = future.result()
            self.pick_result = bool(resp.success)
            self.get_logger().info(
                f'Arm pick result: {resp.success} ({resp.message})')
        except Exception as e:
            self.get_logger().error(f'execute_pick error: {e}')
            self.pick_result = False

    def update_pick(self):
        """Wait for the delegated pick to complete."""
        if self.pick_result is True:
            self.get_logger().info('Pick succeeded')
            self.transition_to(RobotState.GO_TO_BASKET)
            return
        if self.pick_result is False:
            self.get_logger().warn('Pick failed')
            self.transition_to(RobotState.RECOVER)
            return

        # Still pending — guard with a timeout
        elapsed = time.time() - self.state_enter_time
        timeout = self.get_parameter('pick_timeout_s').value
        if elapsed > timeout:
            self.get_logger().warn('Pick timeout')
            self.transition_to(RobotState.RECOVER)
    
    # ==================== State: GO_TO_BASKET ====================
    
    def enter_go_to_basket(self):
        """Navigate to known basket pose"""
        self.get_logger().info('Entering GO_TO_BASKET')
        
        # Perception stays disabled
        
        # Send goal to basket
        basket = PoseStamped()
        basket.header.frame_id = 'map'
        basket.header.stamp = self.get_clock().now().to_msg()
        basket.pose.position.x = self.get_parameter('basket_x').value
        basket.pose.position.y = self.get_parameter('basket_y').value
        basket.pose.position.z = 0.0
        basket.pose.orientation.w = 1.0
        
        self.send_nav_goal(basket)
        self.get_logger().info(f'Navigating to basket: ({basket.pose.position.x}, {basket.pose.position.y})')
    
    def update_go_to_basket(self):
        """Wait for navigation to complete"""
        # Check if arrived (Nav2 result)
        if self.nav_goal_result is not None:
            # Nav2 goal completed
            self.transition_to(RobotState.PLACE)
            return
        
        # Check timeout
        elapsed = time.time() - self.state_enter_time
        if elapsed > 90.0:
            self.get_logger().warn('Go to basket timeout')
            self.transition_to(RobotState.RECOVER)
    
    # ==================== State: PLACE ====================
    
    def enter_place(self):
        """Place clothes in basket"""
        self.get_logger().info('Entering PLACE')
        
        # Stop base
        self.stop_base()
        
        # Call arm place service (stub)
        self.place_success = False
        self.get_logger().info('Calling arm place service (stub)')
    
    def update_place(self):
        """Wait for place completion"""
        elapsed = time.time() - self.state_enter_time
        
        # Simulate place taking 3 seconds
        if elapsed > 3.0:
            self.place_success = True  # Stub
        
        if self.place_success:
            self.get_logger().info('Place succeeded - returning to WANDER')
            self.transition_to(RobotState.WANDER)
            return
        
        # Check timeout
        if elapsed > 30.0:
            self.get_logger().warn('Place timeout')
            self.transition_to(RobotState.RECOVER)
    
    # ==================== State: RECOVER ====================
    
    def enter_recover(self):
        """Handle failures - reset and return to wander"""
        self.get_logger().info('Entering RECOVER')
        
        # Stop everything
        self.cancel_nav_goal()
        self.stop_base()
        
        # Reset perception
        self.call_perception_reset()
        self.set_perception_enabled(True)
        
        # Reset state
        self.clothes_target = None
        self.pick_success = False
        self.place_success = False
        self.awaiting_3d = False

    def update_recover(self):
        """Wait briefly then return to wander"""
        elapsed = time.time() - self.state_enter_time
        
        if elapsed > 2.0:
            self.get_logger().info('Recovery complete - returning to WANDER')
            self.transition_to(RobotState.WANDER)
    
    # ==================== Helper Functions ====================
    
    def is_clothes_stable(self) -> bool:
        """Check if clothes has been tracked consistently"""
        if self.latest_detection is None:
            return False
        
        required_frames = self.get_parameter('clothes_stable_frames_required').value
        required_time = self.get_parameter('clothes_stable_time_s').value
        
        if self.clothes_frame_count < required_frames:
            return False
        
        if self.clothes_first_seen_time is None:
            return False
        
        elapsed = time.time() - self.clothes_first_seen_time
        return elapsed >= required_time
    
    def stop_base(self):
        """Publish zero velocity"""
        twist = Twist()
        for _ in range(5):  # Send multiple times to ensure
            self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Base stopped')
    
    def set_perception_enabled(self, enabled: bool):
        """Enable/disable clothes perception"""
        if not self.perception_enable_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Perception enable service not available', throttle_duration_sec=5.0)
            return
        
        request = SetBool.Request()
        request.data = enabled
        
        future = self.perception_enable_client.call_async(request)
        # Don't block on response
    
    def call_perception_reset(self):
        """Reset clothes target in perception"""
        if not self.perception_reset_client.wait_for_service(timeout_sec=0.5):
            return
        
        request = Trigger.Request()
        future = self.perception_reset_client.call_async(request)
    
    # ==================== Operating-mode supervision ====================

    def request_mode(self, mode: OperatingMode):
        """Switch the active GPU operating mode (idempotent).

        NAV       -> resume Nav2, disable perception + arm.
        PERCEPTION -> pause Nav2, enable perception + arm.
        """
        if mode == self.operating_mode:
            return  # already in this mode; nothing to toggle

        if not self.get_parameter('enable_mode_switching').value:
            # Switching disabled: just record/publish intent without toggling
            # pipelines (useful when bringing the stack up all-on for testing).
            self.operating_mode = mode
            self._publish_mode(mode)
            return

        self.get_logger().info(
            f'Operating mode: {self.operating_mode} → {mode.name}')

        if mode == OperatingMode.NAV:
            self._set_nav_active(True)
            self.set_perception_enabled(False)
            self._set_arm_active(False)
        else:  # PERCEPTION
            self._set_nav_active(False)
            self.set_perception_enabled(True)
            self._set_arm_active(True)

        self._apply_gpu_pipeline(mode)

        self.operating_mode = mode
        self._publish_mode(mode)

    def _publish_mode(self, mode: OperatingMode):
        msg = String()
        msg.data = mode.name
        self.mode_pub.publish(msg)

    def _set_nav_active(self, active: bool):
        """Resume (active) or pause (idle) the Nav2 lifecycle manager."""
        if not self.nav_lifecycle_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                'Nav2 lifecycle manager not available; cannot toggle nav',
                throttle_duration_sec=5.0)
            return
        request = ManageLifecycleNodes.Request()
        request.command = (ManageLifecycleNodes.Request.RESUME if active
                           else ManageLifecycleNodes.Request.PAUSE)
        self.nav_lifecycle_client.call_async(request)

    def _set_arm_active(self, active: bool):
        """Enable/disable arm_bridge's autonomous pick loop."""
        if not self.arm_active_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                'arm_bridge set_active service not available',
                throttle_duration_sec=5.0)
            return
        request = SetBool.Request()
        request.data = active
        self.arm_active_client.call_async(request)

    def _apply_gpu_pipeline(self, mode: OperatingMode):
        """Hook for gating the heavy GPU pipelines (YOLO vs nvblox).

        The exact mechanism (composable-node load/unload on the vision/nvblox
        containers, or per-pipeline enable services) is deferred until Phase-1
        hardware measurement confirms which pipelines actually contend for the
        GPU. Until then this only logs intent; SLAM + nvblox + YOLO all stay
        loaded and the mode switch relies on Nav2/perception/arm gating above.
        """
        self.get_logger().debug(
            f'GPU pipeline hook: target mode {mode.name} (no-op until wired)')

    def send_nav_goal(self, pose: PoseStamped):
        """Send a NavigateToPose goal to Nav2."""
        if not self.nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                'navigate_to_pose action server not available; goal dropped')
            return

        # Reset result so update_* states wait for this goal's completion
        self.nav_goal_result = None

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            f'Sending Nav2 goal: ({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f}) in {pose.header.frame_id}')

        send_goal_future = self.nav_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def cancel_nav_goal(self):
        """Cancel active Nav2 goal"""
        if self.nav_goal_handle is not None:
            future = self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
            self.get_logger().info('Cancelled Nav2 goal')
    
    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            return
        
        self.nav_goal_handle = goal_handle
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_goal_result_callback)
    
    def nav_goal_result_callback(self, future):
        """Handle Nav2 goal completion"""
        result = future.result().result
        self.nav_goal_result = result
        self.get_logger().info('Nav2 goal completed')
    
    # ==================== Callbacks ====================
    
    def clothes_detection_callback(self, msg: Detection2D):
        """Receive lightweight 2D detection from perception"""
        self.latest_detection = msg
        self.last_clothes_update_time = time.time()
        
        # Track stability
        if self.clothes_first_seen_time is None:
            self.clothes_first_seen_time = time.time()
        
        self.clothes_frame_count += 1
    
    def capture_clothes_target(self):
        """Request the 3D pose of the stable detection (async).

        Called from WANDER while perception is live. The response is converted
        to a fixed map-frame target in _on_clothes_3d, after which we switch to
        APPROACH (NAV mode). Sets awaiting_3d to avoid re-entrancy.
        """
        if self.latest_detection is None:
            return
        if not self.get_clothes_3d_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('get_3d_pose service unavailable; staying in WANDER')
            return
        self.awaiting_3d = True
        self.get_logger().info('Stable clothes — capturing 3D map target')
        request = GetSock3D.Request()
        request.detection = self.latest_detection
        future = self.get_clothes_3d_client.call_async(request)
        future.add_done_callback(self._on_clothes_3d)

    def _on_clothes_3d(self, future):
        """Convert the camera-frame 3D point to a map-frame target, then approach."""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'3D service error: {e}')
            self.awaiting_3d = False
            return

        if not response.success:
            # Couldn't get a 3D fix — abandon this detection and keep wandering
            self.get_logger().warn(f'3D capture failed: {response.message}')
            self.awaiting_3d = False
            self.call_perception_reset()
            self.clothes_first_seen_time = None
            self.clothes_frame_count = 0
            return

        # Always keep the raw camera-frame point — the encoder approach servos
        # on its bearing/range directly (no SLAM needed).
        self.clothes_target_cam = response.point
        self.last_clothes_update_time = time.time()

        if self.approach_mode == 'encoder':
            self.get_logger().info(
                f'Captured camera target: ({response.point.point.x:.2f}, '
                f'{response.point.point.y:.2f}, {response.point.point.z:.2f}) — '
                f'encoder approach')
            self.awaiting_3d = False
            self.transition_to(RobotState.APPROACH_CLOTHES)
            return

        # Legacy 'nav' approach: transform the camera-frame point into the map
        # frame so the target is fixed in the world and we can drive to it with
        # perception off.
        try:
            point_map = self.tf_buffer.transform(
                response.point, self.map_frame, timeout=Duration(seconds=0.5))
        except TransformException as e:
            self.get_logger().warn(
                f'TF camera→map failed ({e}); cannot localize target, wandering')
            self.awaiting_3d = False
            return

        self.clothes_target = PoseStamped()
        self.clothes_target.header.frame_id = self.map_frame
        self.clothes_target.header.stamp = point_map.header.stamp
        self.clothes_target.pose.position = point_map.point
        self.clothes_target.pose.orientation.w = 1.0
        self.get_logger().info(
            f'Captured map target: ({point_map.point.x:.2f}, '
            f'{point_map.point.y:.2f}, {point_map.point.z:.2f})')

        self.awaiting_3d = False
        self.transition_to(RobotState.APPROACH_CLOTHES)
    
    def odom_callback(self, msg: Odometry):
        """Receive odometry from SLAM"""
        self.current_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down behavior manager')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
