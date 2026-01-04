#!/usr/bin/env python3
"""
Behavior Manager Node - State machine for sock-collecting robot

States: WANDER → APPROACH_SOCK → PICK → GO_TO_BASKET → PLACE → (RECOVER) → WANDER

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


class RobotState(Enum):
    """Robot behavior states"""
    WANDER = auto()
    APPROACH_SOCK = auto()
    PICK = auto()
    GO_TO_BASKET = auto()
    PLACE = auto()
    RECOVER = auto()


class BehaviorManagerNode(Node):
    """
    Central behavior orchestrator
    
    Controls:
    - Nav2 goals (ONLY source of navigation commands)
    - Perception gating (enable/disable sock detection)
    - Arm manipulation (via service calls)
    - State transitions based on environment feedback
    """
    
    def __init__(self):
        super().__init__('behavior_manager')
        
        # Parameters
        self.declare_parameter('wander_radius_m', 3.0)
        self.declare_parameter('wander_timeout_s', 30.0)
        self.declare_parameter('sock_confidence_threshold', 0.7)
        self.declare_parameter('sock_stable_frames_required', 5)
        self.declare_parameter('sock_stable_time_s', 2.0)
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
        
        # Sock tracking
        self.sock_target: PoseStamped = None
        self.sock_first_seen_time = None
        self.sock_frame_count = 0
        self.last_sock_update_time = 0.0
        
        # Navigation
        self.current_odom: Odometry = None
        self.nav_goal_handle = None
        self.nav_goal_result = None
        self.last_goal_update_time = 0.0
        self.last_goal_position = None
        
        # Manipulation status
        self.pick_success = False
        self.place_success = False
        
        # Service clients
        self.perception_enable_client = self.create_client(
            SetBool, '/sock_perception/enable')
        self.perception_reset_client = self.create_client(
            Trigger, '/sock_perception/reset_target')
        
        # Would be arm service clients (stubs for now)
        # self.arm_pick_client = self.create_client(...)
        # self.arm_place_client = self.create_client(...)
        
        # Action clients
        self.nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers
        self.sock_target_sub = self.create_subscription(
            PoseStamped,
            '/sock/target_point_map',
            self.sock_target_callback,
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
        
        # Initialize to WANDER
        self.enter_wander()
        
        self.get_logger().info('Behavior manager initialized in WANDER state')
    
    # ==================== State Machine Core ====================
    
    def state_machine_update(self):
        """Main state machine loop"""
        # Check transitions based on current state
        if self.current_state == RobotState.WANDER:
            self.update_wander()
        elif self.current_state == RobotState.APPROACH_SOCK:
            self.update_approach_sock()
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
        
        # Call enter function
        if new_state == RobotState.WANDER:
            self.enter_wander()
        elif new_state == RobotState.APPROACH_SOCK:
            self.enter_approach_sock()
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
        
        # Reset sock tracking
        self.call_perception_reset()
        self.sock_target = None
        self.sock_first_seen_time = None
        self.sock_frame_count = 0
        
        # Send random wander goal
        self.send_random_wander_goal()
    
    def update_wander(self):
        """Check for sock detection or wander timeout"""
        # Check if sock detected and stable
        if self.is_sock_stable():
            self.transition_to(RobotState.APPROACH_SOCK)
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
    
    # ==================== State: APPROACH_SOCK ====================
    
    def enter_approach_sock(self):
        """Cancel wander, enable moderate perception, approach sock"""
        self.get_logger().info('Entering APPROACH_SOCK')
        
        # Cancel any existing goal
        self.cancel_nav_goal()
        
        # Enable perception at higher rate
        rate_hz = self.get_parameter('approach_perception_rate_hz').value
        self.set_perception_enabled(True)
        self.get_logger().info(f'Perception rate increased to {rate_hz} Hz')
        
        # Send initial approach goal
        if self.sock_target:
            self.send_approach_goal(self.sock_target)
    
    def update_approach_sock(self):
        """Update Nav2 goal as sock target moves, check arrival"""
        # Check if sock lost
        time_since_update = time.time() - self.last_sock_update_time
        if time_since_update > 5.0:
            self.get_logger().warn('Sock lost during approach')
            self.transition_to(RobotState.RECOVER)
            return
        
        # Check if close enough to pick
        if self.is_near_sock():
            self.transition_to(RobotState.PICK)
            return
        
        # Update goal if target moved significantly
        if self.sock_target:
            self.update_approach_goal(self.sock_target)
        
        # Check timeout
        elapsed = time.time() - self.state_enter_time
        timeout = self.get_parameter('approach_timeout_s').value
        if elapsed > timeout:
            self.get_logger().warn('Approach timeout')
            self.transition_to(RobotState.RECOVER)
    
    def send_approach_goal(self, target: PoseStamped):
        """Send Nav2 goal offset from sock by grasp distance"""
        if self.current_odom is None:
            return
        
        # Compute approach pose (offset from sock toward robot)
        grasp_offset = self.get_parameter('grasp_offset_m').value
        
        # Vector from robot to sock
        dx = target.pose.position.x - self.current_odom.pose.pose.position.x
        dy = target.pose.position.y - self.current_odom.pose.pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < 0.01:
            return
        
        # Unit vector
        ux = dx / dist
        uy = dy / dist
        
        # Goal is grasp_offset before the sock
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = target.pose.position.x - grasp_offset * ux
        goal.pose.position.y = target.pose.position.y - grasp_offset * uy
        goal.pose.position.z = 0.0
        
        # Orient toward sock
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
    
    def is_near_sock(self) -> bool:
        """Check if robot is within grasp distance of sock"""
        if self.sock_target is None or self.current_odom is None:
            return False
        
        dx = self.sock_target.pose.position.x - self.current_odom.pose.pose.position.x
        dy = self.sock_target.pose.position.y - self.current_odom.pose.pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        
        stop_dist = self.get_parameter('approach_stop_distance_m').value
        return dist < stop_dist
    
    # ==================== State: PICK ====================
    
    def enter_pick(self):
        """Stop base, disable perception, pick sock"""
        self.get_logger().info('Entering PICK')
        
        # Cancel Nav2 goal
        self.cancel_nav_goal()
        
        # Stop base
        self.stop_base()
        
        # Disable perception
        self.set_perception_enabled(False)
        
        # Call arm pick service (stub)
        self.pick_success = False
        self.get_logger().info('Calling arm pick service (stub)')
        
        # Simulate pick - in real system, call arm service
        # For now, assume success after delay
    
    def update_pick(self):
        """Wait for pick completion"""
        elapsed = time.time() - self.state_enter_time
        
        # Simulate pick taking 3 seconds
        if elapsed > 3.0:
            self.pick_success = True  # Stub - would come from arm service
        
        if self.pick_success:
            self.get_logger().info('Pick succeeded')
            self.transition_to(RobotState.GO_TO_BASKET)
            return
        
        # Check timeout
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
        """Place sock in basket"""
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
        self.sock_target = None
        self.pick_success = False
        self.place_success = False
    
    def update_recover(self):
        """Wait briefly then return to wander"""
        elapsed = time.time() - self.state_enter_time
        
        if elapsed > 2.0:
            self.get_logger().info('Recovery complete - returning to WANDER')
            self.transition_to(RobotState.WANDER)
    
    # ==================== Helper Functions ====================
    
    def is_sock_stable(self) -> bool:
        """Check if sock has been tracked consistently"""
        if self.sock_target is None:
            return False
        
        required_frames = self.get_parameter('sock_stable_frames_required').value
        required_time = self.get_parameter('sock_stable_time_s').value
        
        if self.sock_frame_count < required_frames:
            return False
        
        if self.sock_first_seen_time is None:
            return False
        
        elapsed = time.time() - self.sock_first_seen_time
        return elapsed >= required_time
    
    def stop_base(self):
        """Publish zero velocity"""
        twist = Twist()
        for _ in range(5):  # Send multiple times to ensure
            self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Base stopped')
    
    def set_perception_enabled(self, enabled: bool):
        """Enable/disable sock perception"""
        if not self.perception_enable_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Perception enable service not available', throttle_duration_sec=5.0)
            return
        
        request = SetBool.Request()
        request.data = enabled
        
        future = self.perception_enable_client.call_async(request)
        # Don't block on response
    
    def call_perception_reset(self):
        """Reset sock target in perception"""
        if not self.perception_reset_client.wait_for_service(timeout_sec=0.5):
            return
        
        request = Trigger.Request()
        future = self.perception_reset_client.call_async(request)
    
    def send_nav_goal(self, pose: PoseStamped):
        """Send Nav2 action goal"""
        if not self.nav_action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Nav2 action server not available', throttle_duration_sec=5.0)
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.nav_goal_result = None
        future = self.nav_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav_goal_response_callback)
    
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
    
    def sock_target_callback(self, msg: PoseStamped):
        """Receive sock target from perception"""
        self.sock_target = msg
        self.last_sock_update_time = time.time()
        
        # Track stability
        if self.sock_first_seen_time is None:
            self.sock_first_seen_time = time.time()
        
        self.sock_frame_count += 1
    
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
