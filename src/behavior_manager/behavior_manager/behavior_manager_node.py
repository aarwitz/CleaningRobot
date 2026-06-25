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
from enum import Enum, auto
import time
import math
import random

from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from vision_msgs.msg import Detection2D
from behavior_manager_interfaces.srv import GetSock3D


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
        
        # State
        self.current_state = RobotState.WANDER
        self.state_enter_time = time.time()
        
        # Clothes tracking
        self.latest_detection = None  # 2D only during WANDER/APPROACH
        self.clothes_target = None  # 3D position from service call
        self.clothes_first_seen_time = None
        self.clothes_frame_count = 0
        self.last_clothes_update_time = 0.0
        
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
        
        # Send random wander goal
        self.send_random_wander_goal()
    
    def update_wander(self):
        """Check for clothes detection or wander timeout"""
        # Check if clothes detected and stable
        if self.is_clothes_stable():
            self.transition_to(RobotState.APPROACH_CLOTHES)
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
        
        # Send initial approach goal
        if self.clothes_target:
            self.send_approach_goal(self.clothes_target)
    
    def update_approach_clothes(self):
        """Drive toward the (fixed map-frame) clothes target, check arrival.

        In NAV mode perception is disabled, so the target is treated as static
        (clothes do not move) and we navigate to the captured map pose. The
        "clothes lost" check and live goal updates only apply while perception
        is actually running (PERCEPTION mode).
        """
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
    
    def handle_clothes_3d_response(self, future):
        """Handle 3D position service response"""
        try:
            response = future.result()
            if response.success:
                # Convert PointStamped to PoseStamped for compatibility
                self.clothes_target = PoseStamped()
                self.clothes_target.header = response.point.header
                self.clothes_target.pose.position = response.point.point
                self.clothes_target.pose.orientation.w = 1.0  # Identity orientation
                
                self.get_logger().info(
                    f'Got 3D position: ({response.point.point.x:.3f}, '
                    f'{response.point.point.y:.3f}, {response.point.point.z:.3f})'
                )
                
                # In real system, would pass clothes_target to arm controller here
            else:
                self.get_logger().warn(f'3D service failed: {response.message}')
                self.transition_to(RobotState.RECOVER)
        except Exception as e:
            self.get_logger().error(f'3D service error: {e}')
            self.transition_to(RobotState.RECOVER)
    
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
