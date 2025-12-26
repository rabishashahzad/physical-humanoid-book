import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from builtin_interfaces.msg import Time


class IsaacSimBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')

        # Create publishers for Isaac Sim sensor data
        self.camera_publisher = self.create_publisher(
            Image,
            'isaac_sim/camera/image_raw',
            10
        )

        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            'isaac_sim/camera/camera_info',
            10
        )

        self.imu_publisher = self.create_publisher(
            Imu,
            'isaac_sim/imu/data',
            10
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            'isaac_sim/odom',
            10
        )

        self.laser_publisher = self.create_publisher(
            LaserScan,
            'isaac_sim/laser_scan',
            10
        )

        # Create subscribers for robot commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer to simulate Isaac Sim data publishing
        self.timer = self.create_timer(0.1, self.publish_simulated_data)

        # Initialize simulation state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.get_logger().info('Isaac Sim Bridge node initialized')

    def cmd_vel_callback(self, msg):
        """Process velocity commands from ROS 2 and apply to simulated robot"""
        # Update robot position based on velocity command (simplified)
        dt = 0.1  # 10Hz update rate
        self.robot_x += msg.linear.x * np.cos(self.robot_theta) * dt
        self.robot_y += msg.linear.x * np.sin(self.robot_theta) * dt
        self.robot_theta += msg.angular.z * dt

        # Log the command
        self.get_logger().info(f'Command received: linear={msg.linear.x}, angular={msg.angular.z}')

    def publish_simulated_data(self):
        """Publish simulated sensor data from Isaac Sim"""
        # Publish simulated camera image
        self.publish_camera_data()

        # Publish simulated IMU data
        self.publish_imu_data()

        # Publish simulated odometry
        self.publish_odom_data()

        # Publish simulated laser scan
        self.publish_laser_data()

    def publish_camera_data(self):
        """Publish simulated camera image and camera info"""
        # Create a simulated image (simplified - in real implementation this would come from Isaac Sim)
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_link'
        img_msg.height = 480
        img_msg.width = 640
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = 0
        img_msg.step = 640 * 3  # width * channels

        # Create dummy image data (in real implementation this would come from Isaac Sim)
        dummy_data = np.zeros((480, 640, 3), dtype=np.uint8)
        # Add some simulated features to the image
        cv2.rectangle(dummy_data, (100, 100), (200, 200), (255, 0, 0), 2)  # Blue rectangle
        cv2.circle(dummy_data, (300, 300), 50, (0, 255, 0), 2)  # Green circle

        img_msg.data = dummy_data.tobytes()
        self.camera_publisher.publish(img_msg)

        # Publish camera info
        cam_info_msg = CameraInfo()
        cam_info_msg.header.stamp = img_msg.header.stamp
        cam_info_msg.header.frame_id = 'camera_link'
        cam_info_msg.height = 480
        cam_info_msg.width = 640
        cam_info_msg.distortion_model = 'plumb_bob'
        cam_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        cam_info_msg.k = [300.0, 0.0, 320.0,  # fx, 0, cx
                          0.0, 300.0, 240.0,  # 0, fy, cy
                          0.0, 0.0, 1.0]      # 0, 0, 1
        cam_info_msg.r = [1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
        cam_info_msg.p = [300.0, 0.0, 320.0, 0.0,  # fx, 0, cx, T
                          0.0, 300.0, 240.0, 0.0,  # 0, fy, cy, T
                          0.0, 0.0, 1.0, 0.0]      # 0, 0, 1, T
        self.camera_info_publisher.publish(cam_info_msg)

    def publish_imu_data(self):
        """Publish simulated IMU data"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Simulate some IMU data (in real implementation this would come from Isaac Sim)
        imu_msg.linear_acceleration.x = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.y = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.z = 9.8 + np.random.normal(0, 0.1)

        imu_msg.angular_velocity.x = np.random.normal(0, 0.01)
        imu_msg.angular_velocity.y = np.random.normal(0, 0.01)
        imu_msg.angular_velocity.z = np.random.normal(0, 0.01)

        # Set orientation (simplified)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = np.sin(self.robot_theta / 2.0)
        imu_msg.orientation.w = np.cos(self.robot_theta / 2.0)

        self.imu_publisher.publish(imu_msg)

    def publish_odom_data(self):
        """Publish simulated odometry data"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        odom_msg.pose.pose.position.z = 0.0

        # Set orientation
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = np.sin(self.robot_theta / 2.0)
        odom_msg.pose.pose.orientation.w = np.cos(self.robot_theta / 2.0)

        # Set velocity (simplified)
        odom_msg.twist.twist.linear.x = 0.5  # Placeholder
        odom_msg.twist.twist.angular.z = 0.2  # Placeholder

        self.odom_publisher.publish(odom_msg)

    def publish_laser_data(self):
        """Publish simulated laser scan data"""
        laser_msg = LaserScan()
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.header.frame_id = 'laser_link'
        laser_msg.angle_min = -np.pi / 2
        laser_msg.angle_max = np.pi / 2
        laser_msg.angle_increment = np.pi / 180  # 1 degree
        laser_msg.time_increment = 0.0
        laser_msg.scan_time = 0.1
        laser_msg.range_min = 0.1
        laser_msg.range_max = 10.0

        # Create simulated ranges (in a real implementation this would come from Isaac Sim)
        num_readings = int((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment) + 1
        ranges = []

        for i in range(num_readings):
            angle = laser_msg.angle_min + i * laser_msg.angle_increment
            # Simulate some obstacles around the robot
            distance = 3.0 + 0.5 * np.sin(4 * angle)  # Add some variation
            ranges.append(distance)

        laser_msg.ranges = ranges
        laser_msg.intensities = [100.0] * len(ranges)  # Dummy intensities

        self.laser_publisher.publish(laser_msg)


def main(args=None):
    rclpy.init(args=args)

    # Import cv2 here to avoid dependency issues if not available
    global cv2
    try:
        import cv2
    except ImportError:
        from PIL import Image as PILImage
        import numpy as np
        # Create a simple function to mimic cv2 operations if needed
        class MockCV2:
            @staticmethod
            def rectangle(img, pt1, pt2, color, thickness):
                # This is a placeholder - in a real implementation, we'd use actual image processing
                pass
            @staticmethod
            def circle(img, center, radius, color, thickness):
                # This is a placeholder - in a real implementation, we'd use actual image processing
                pass
        cv2 = MockCV2

    isaac_sim_bridge = IsaacSimBridgeNode()

    try:
        rclpy.spin(isaac_sim_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        isaac_sim_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()