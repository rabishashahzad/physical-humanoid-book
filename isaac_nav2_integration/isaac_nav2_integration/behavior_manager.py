import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
import time
import math
import threading
from enum import Enum


class NavigationState(Enum):
    IDLE = "idle"
    PLANNING = "planning"
    EXECUTING = "executing"
    RECOVERY = "recovery"
    COMPLETED = "completed"
    FAILED = "failed"


class IsaacBehaviorManager(Node):
    def __init__(self):
        super().__init__('isaac_behavior_manager')

        # Navigation state
        self.current_state = NavigationState.IDLE
        self.previous_state = NavigationState.IDLE
        self.goal_pose = None
        self.start_time = None
        self.timeout_duration = 300.0  # 5 minutes timeout

        # Behavior parameters
        self.recovery_behavior = 'clear_costmap'  # Options: clear_costmap, back_up, wait
        self.max_recovery_attempts = 3
        self.recovery_attempts = 0

        # Create subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        self.navigation_state_sub = self.create_subscription(
            String,
            'navigation_state',
            self.navigation_state_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.velocity_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )

        # Create publishers
        self.state_pub = self.create_publisher(
            String,
            'behavior_state',
            10
        )

        self.recovery_pub = self.create_publisher(
            String,
            'recovery_command',
            10
        )

        self.cancel_pub = self.create_publisher(
            Bool,
            'navigation_cancel',
            10
        )

        self.planner_status_sub = self.create_subscription(
            String,
            'planner_status',
            self.planner_status_callback,
            10
        )

        # Timer for state monitoring
        self.state_timer = self.create_timer(0.5, self.state_monitor)

        # Timer for timeout checking
        self.timeout_timer = self.create_timer(1.0, self.timeout_check)

        self.get_logger().info('Isaac Behavior Manager initialized')

    def goal_callback(self, msg):
        """Process new navigation goal"""
        self.goal_pose = msg.pose
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.current_state = NavigationState.PLANNING
        self.recovery_attempts = 0
        self.get_logger().info(f'New goal received, transitioning to {self.current_state.value}')

    def navigation_state_callback(self, msg):
        """Update based on navigation state"""
        nav_state = msg.data.lower()

        if nav_state == 'executing' and self.current_state == NavigationState.PLANNING:
            self.current_state = NavigationState.EXECUTING
        elif nav_state == 'completed' and self.current_state == NavigationState.EXECUTING:
            self.current_state = NavigationState.COMPLETED
        elif nav_state == 'failed' or nav_state == 'error':
            if self.current_state == NavigationState.EXECUTING:
                self.handle_navigation_failure()

    def planner_status_callback(self, msg):
        """Handle planner status updates"""
        status = msg.data.lower()

        if status == 'path_found' and self.current_state == NavigationState.PLANNING:
            self.current_state = NavigationState.EXECUTING
        elif status == 'path_failed':
            self.handle_planning_failure()

    def odom_callback(self, msg):
        """Process odometry for behavior management"""
        # Monitor robot movement for stuck detection
        linear_vel = math.sqrt(
            msg.twist.twist.linear.x**2 +
            msg.twist.twist.linear.y**2 +
            msg.twist.twist.linear.z**2
        )

        angular_vel = math.sqrt(
            msg.twist.twist.angular.x**2 +
            msg.twist.twist.angular.y**2 +
            msg.twist.twist.angular.z**2
        )

        # In a real implementation, detect if the robot is stuck
        # For now, we'll just log the velocities
        if self.current_state == NavigationState.EXECUTING:
            self.get_logger().debug(f'Robot velocity - Linear: {linear_vel:.3f}, Angular: {angular_vel:.3f}')

    def velocity_callback(self, msg):
        """Process velocity commands for behavior management"""
        # Monitor commanded velocities for behavior analysis
        cmd_linear = abs(msg.linear.x)
        cmd_angular = abs(msg.angular.z)

        # In a real implementation, check if commanded velocities are not being executed
        # This could indicate the robot is stuck or blocked

    def state_monitor(self):
        """Monitor and manage navigation state"""
        # Publish current state
        state_msg = String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)

        # Log state changes
        if self.current_state != self.previous_state:
            self.get_logger().info(f'State changed from {self.previous_state.value} to {self.current_state.value}')
            self.previous_state = self.current_state

        # Handle specific state behaviors
        if self.current_state == NavigationState.COMPLETED:
            self.get_logger().info('Navigation completed successfully')
            self.current_state = NavigationState.IDLE
        elif self.current_state == NavigationState.FAILED:
            self.handle_final_failure()

    def timeout_check(self):
        """Check for navigation timeouts"""
        if (self.current_state in [NavigationState.PLANNING, NavigationState.EXECUTING] and
            self.start_time is not None):
            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed = current_time - self.start_time

            if elapsed > self.timeout_duration:
                self.get_logger().warn('Navigation timed out')
                self.current_state = NavigationState.FAILED
                self.cancel_navigation()

    def handle_navigation_failure(self):
        """Handle navigation failure and attempt recovery"""
        self.recovery_attempts += 1

        if self.recovery_attempts <= self.max_recovery_attempts:
            self.get_logger().warn(f'Navigation failed, attempting recovery (attempt {self.recovery_attempts})')
            self.current_state = NavigationState.RECOVERY
            self.execute_recovery_behavior()
        else:
            self.get_logger().error('Maximum recovery attempts reached, navigation failed')
            self.current_state = NavigationState.FAILED

    def handle_planning_failure(self):
        """Handle path planning failure"""
        self.get_logger().warn('Path planning failed')
        self.current_state = NavigationState.FAILED

    def handle_final_failure(self):
        """Handle final navigation failure"""
        self.get_logger().error('Navigation failed permanently')
        self.cancel_navigation()

    def execute_recovery_behavior(self):
        """Execute recovery behavior based on configuration"""
        recovery_msg = String()

        if self.recovery_behavior == 'clear_costmap':
            recovery_msg.data = 'clear_costmap'
        elif self.recovery_behavior == 'back_up':
            recovery_msg.data = 'back_up'
        elif self.recovery_behavior == 'wait':
            recovery_msg.data = 'wait_5s'
        else:
            recovery_msg.data = 'clear_costmap'  # Default

        self.recovery_pub.publish(recovery_msg)
        self.get_logger().info(f'Executing recovery behavior: {recovery_msg.data}')

        # After recovery, try navigation again
        # In a real implementation, wait for recovery to complete
        # For simulation, we'll just transition back to executing
        time.sleep(2)  # Simulate recovery time
        self.current_state = NavigationState.EXECUTING

    def cancel_navigation(self):
        """Cancel current navigation"""
        cancel_msg = Bool()
        cancel_msg.data = True
        self.cancel_pub.publish(cancel_msg)
        self.get_logger().info('Navigation cancelled')

    def reset(self):
        """Reset the behavior manager to initial state"""
        self.current_state = NavigationState.IDLE
        self.previous_state = NavigationState.IDLE
        self.goal_pose = None
        self.start_time = None
        self.recovery_attempts = 0


def main(args=None):
    rclpy.init(args=args)

    behavior_manager = IsaacBehaviorManager()

    try:
        rclpy.spin(behavior_manager)
    except KeyboardInterrupt:
        pass
    finally:
        # Cancel any ongoing navigation on shutdown
        cancel_msg = Bool()
        cancel_msg.data = True
        behavior_manager.cancel_pub.publish(cancel_msg)

        behavior_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import math  # Need this for the velocity calculations
    main()