import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from builtin_interfaces.msg import Time
from tf2_ros import TransformListener, Buffer
import numpy as np
import math


class IsaacNav2Controller(Node):
    def __init__(self):
        super().__init__('isaac_nav2_controller')

        # Navigation state
        self.current_pose = None
        self.goal_pose = None
        self.path = None
        self.is_navigating = False
        self.navigation_state = 'idle'  # idle, planning, executing, paused, completed

        # Controller parameters
        self.linear_vel_max = 0.5  # m/s
        self.angular_vel_max = 1.0  # rad/s
        self.arrival_threshold = 0.3  # meters
        self.rotation_threshold = 0.1  # radians

        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            'plan',
            self.path_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.current_goal_pub = self.create_publisher(
            PoseStamped,
            'current_goal',
            10
        )

        self.navigation_state_pub = self.create_publisher(
            String,
            'navigation_state',
            10
        )

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Safety timer
        self.safety_timer = self.create_timer(0.5, self.safety_check)

        self.get_logger().info('Isaac Nav2 Controller initialized')

    def odom_callback(self, msg):
        """Update current robot pose from odometry"""
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        """Process new navigation goal"""
        self.goal_pose = msg.pose
        self.is_navigating = True
        self.navigation_state = 'planning'
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def path_callback(self, msg):
        """Process received path plan"""
        self.path = msg.poses
        if self.path:
            self.navigation_state = 'executing'
            self.get_logger().info(f'Path received with {len(self.path)} waypoints')

    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        # Check for obstacles in front of the robot
        front_scan = msg.ranges[len(msg.ranges)//2 - 30:len(msg.ranges)//2 + 30]
        min_distance = min([r for r in front_scan if not math.isnan(r) and r > 0], default=float('inf'))

        if min_distance < 0.5:  # Obstacle within 0.5m
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m, slowing down')
            # In a real implementation, this would trigger obstacle avoidance

    def control_loop(self):
        """Main navigation control loop"""
        if not self.is_navigating or not self.current_pose or not self.goal_pose:
            return

        cmd_vel = Twist()

        if self.navigation_state == 'executing':
            # Calculate control commands to follow the path
            if self.path and len(self.path) > 0:
                cmd_vel = self.calculate_path_following_control()
            else:
                # If no path, go directly to goal
                cmd_vel = self.calculate_goal_control()

        elif self.navigation_state == 'planning':
            # Still waiting for path
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)

        # Check if goal is reached
        if self.is_goal_reached():
            self.navigation_state = 'completed'
            self.is_navigating = False
            self.get_logger().info('Goal reached successfully!')
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)  # Stop the robot

    def calculate_goal_control(self):
        """Calculate control commands to reach the goal directly"""
        cmd_vel = Twist()

        if not self.current_pose or not self.goal_pose:
            return cmd_vel

        # Calculate error
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate desired orientation
        desired_yaw = math.atan2(dy, dx)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        # Calculate angular error
        angle_error = self.normalize_angle(desired_yaw - current_yaw)

        # Simple proportional controller
        if distance > self.arrival_threshold:
            # Move toward goal
            cmd_vel.linear.x = min(self.linear_vel_max * 0.8, max(0.1, distance * 0.5))
        else:
            # Close to goal, focus on orientation
            cmd_vel.linear.x = 0.0

        if abs(angle_error) > self.rotation_threshold:
            # Rotate toward goal
            cmd_vel.angular.z = max(-self.angular_vel_max, min(self.angular_vel_max, angle_error * 1.0))

        return cmd_vel

    def calculate_path_following_control(self):
        """Calculate control commands to follow the path"""
        cmd_vel = Twist()

        if not self.current_pose or not self.path or len(self.path) == 0:
            return cmd_vel

        # Find closest point on path
        closest_idx = self.find_closest_waypoint()
        if closest_idx is None:
            return cmd_vel

        # Look ahead to a point on the path
        look_ahead_idx = min(closest_idx + 5, len(self.path) - 1)
        look_ahead_point = self.path[look_ahead_idx].pose.position

        # Calculate control based on look ahead point
        dx = look_ahead_point.x - self.current_pose.position.x
        dy = look_ahead_point.y - self.current_pose.position.y

        # Calculate desired heading
        desired_yaw = math.atan2(dy, dx)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        angle_error = self.normalize_angle(desired_yaw - current_yaw)

        # Calculate distance to look ahead point
        distance = math.sqrt(dx*dx + dy*dy)

        # Set velocities
        cmd_vel.linear.x = min(self.linear_vel_max * 0.7, max(0.1, distance * 0.3))
        cmd_vel.angular.z = max(-self.angular_vel_max, min(self.angular_vel_max, angle_error * 1.5))

        return cmd_vel

    def find_closest_waypoint(self):
        """Find the closest waypoint on the path"""
        if not self.current_pose or not self.path:
            return None

        min_distance = float('inf')
        closest_idx = 0

        for i, pose in enumerate(self.path):
            dx = pose.pose.position.x - self.current_pose.position.x
            dy = pose.pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < min_distance:
                min_distance = distance
                closest_idx = i

        return closest_idx if min_distance < 5.0 else None  # Only return if reasonably close

    def is_goal_reached(self):
        """Check if the robot has reached the goal"""
        if not self.current_pose or not self.goal_pose:
            return False

        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Check both position and orientation
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        goal_yaw = self.quaternion_to_yaw(self.goal_pose.orientation)
        angle_diff = abs(self.normalize_angle(current_yaw - goal_yaw))

        return distance < self.arrival_threshold and angle_diff < self.rotation_threshold

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def safety_check(self):
        """Perform safety checks and publish navigation state"""
        state_msg = String()
        state_msg.data = self.navigation_state
        self.navigation_state_pub.publish(state_msg)

        # Check if we're stuck
        if (self.navigation_state == 'executing' and
            self.current_pose and self.goal_pose):
            # In a real implementation, check if the robot has made progress
            pass


def main(args=None):
    rclpy.init(args=args)

    nav2_controller = IsaacNav2Controller()

    try:
        rclpy.spin(nav2_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on shutdown
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        nav2_controller.cmd_vel_pub.publish(cmd_vel)

        nav2_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()