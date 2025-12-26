import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
from builtin_interfaces.msg import Time
import numpy as np
import math
from collections import deque


class IsaacPathPlanner(Node):
    def __init__(self):
        super().__init__('isaac_path_planner')

        # Planner parameters
        self.planner_type = 'a_star'  # Can be 'a_star', 'dijkstra', 'rrt'
        self.map_resolution = 0.05  # meters per pixel
        self.robot_radius = 0.3  # meters
        self.inflation_radius = 0.5  # meters

        # Map data
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        # Create subscribers
        self.start_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.start_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        # Create publishers
        self.path_pub = self.create_publisher(
            Path,
            'plan',
            10
        )

        self.global_plan_pub = self.create_publisher(
            Path,
            'global_plan',
            10
        )

        self.planner_status_pub = self.create_publisher(
            String,
            'planner_status',
            10
        )

        self.start_pose = None
        self.goal_pose = None

        self.get_logger().info('Isaac Path Planner initialized')

    def map_callback(self, msg):
        """Process occupancy grid map"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_resolution = msg.info.resolution

        self.get_logger().info(f'Map received: {self.map_width}x{self.map_height}, resolution: {self.map_resolution}')

    def start_callback(self, msg):
        """Process initial pose"""
        self.start_pose = msg.pose.pose
        self.get_logger().info('Start pose received')

    def goal_callback(self, msg):
        """Process goal pose and plan path"""
        self.goal_pose = msg.pose
        self.get_logger().info(f'Goal pose received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        if self.map_data is not None and self.start_pose is not None:
            self.plan_path()

    def plan_path(self):
        """Plan path from start to goal"""
        if not self.start_pose or not self.goal_pose:
            self.get_logger().warn('Start or goal pose not set')
            return

        # Convert world coordinates to map coordinates
        start_map = self.world_to_map(self.start_pose.position.x, self.start_pose.position.y)
        goal_map = self.world_to_map(self.goal_pose.position.x, self.goal_pose.position.y)

        if not self.is_valid_cell(start_map[0], start_map[1]) or not self.is_valid_cell(goal_map[0], goal_map[1]):
            self.get_logger().warn('Start or goal is in invalid cell (obstacle or outside map)')
            return

        # Plan path using selected algorithm
        if self.planner_type == 'a_star':
            path = self.a_star_plan(start_map, goal_map)
        elif self.planner_type == 'dijkstra':
            path = self.dijkstra_plan(start_map, goal_map)
        else:
            path = self.a_star_plan(start_map, goal_map)  # Default to A*

        if path:
            # Convert path back to world coordinates
            world_path = [self.map_to_world(x, y) for x, y in path]
            self.publish_path(world_path)
            self.get_logger().info(f'Path planned with {len(path)} waypoints')
        else:
            self.get_logger().warn('Failed to find a path to the goal')
            status_msg = String()
            status_msg.data = 'path_failed'
            self.planner_status_pub.publish(status_msg)

    def a_star_plan(self, start, goal):
        """A* path planning algorithm"""
        # Define movement directions (8-connected)
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]

        # Calculate movement costs (diagonal = sqrt(2), straight = 1)
        costs = [
            math.sqrt(2), 1, math.sqrt(2),
            1,           1,
            math.sqrt(2), 1, math.sqrt(2)
        ]

        # Initialize open and closed sets
        open_set = [(0, start)]  # (f_score, position)
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            # Get node with lowest f_score
            current = min(open_set, key=lambda x: x[0])[1]
            open_set = [item for item in open_set if item[1] != current]

            # Check if we reached the goal
            if current == goal:
                return self.reconstruct_path(came_from, current)

            # Explore neighbors
            for i, direction in enumerate(directions):
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                tentative_g_score = g_score[current] + costs[i]

                # Check if neighbor is valid and not in obstacle
                if (self.is_valid_cell(neighbor[0], neighbor[1]) and
                    self.is_traversable(neighbor[0], neighbor[1])):

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        open_set.append((f_score[neighbor], neighbor))

        return None  # No path found

    def heuristic(self, pos1, pos2):
        """Calculate heuristic distance (Euclidean)"""
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        return math.sqrt(dx*dx + dy*dy)

    def reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def is_valid_cell(self, x, y):
        """Check if cell coordinates are within map bounds"""
        return 0 <= x < self.map_width and 0 <= y < self.map_height

    def is_traversable(self, x, y):
        """Check if a cell is traversable (not an obstacle)"""
        if not self.is_valid_cell(x, y):
            return False

        # Check if cell is occupied (value > 50) or unknown (value = -1)
        cell_value = self.map_data[y, x]
        return cell_value < 50  # Consider cells with value < 50 as traversable

    def world_to_map(self, x, y):
        """Convert world coordinates to map indices"""
        map_x = int((x - self.map_origin_x) / self.map_resolution)
        map_y = int((y - self.map_origin_y) / self.map_resolution)
        return (map_x, map_y)

    def map_to_world(self, x, y):
        """Convert map indices to world coordinates"""
        world_x = self.map_origin_x + x * self.map_resolution
        world_y = self.map_origin_y + y * self.map_resolution
        return (world_x, world_y)

    def publish_path(self, path_points):
        """Publish the planned path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in path_points:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.global_plan_pub.publish(path_msg)

        # Publish success status
        status_msg = String()
        status_msg.data = 'path_found'
        self.planner_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    path_planner = IsaacPathPlanner()

    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        pass
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()