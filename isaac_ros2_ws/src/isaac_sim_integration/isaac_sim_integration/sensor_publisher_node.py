import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import numpy as np
from builtin_interfaces.msg import Time
import struct


class SensorPublisherNode(Node):
    def __init__(self):
        super().__init__('sensor_publisher_node')

        # Create publishers for different sensor types
        self.rgb_publisher = self.create_publisher(
            Image,
            'isaac_ros/rgb/image_raw',
            10
        )

        self.depth_publisher = self.create_publisher(
            Image,
            'isaac_ros/depth/image_raw',
            10
        )

        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            'isaac_ros/pointcloud',
            10
        )

        # Timer to publish sensor data
        self.timer = self.create_timer(0.033, self.publish_sensor_data)  # ~30 FPS

        self.get_logger().info('Isaac ROS Sensor Publisher node initialized')

    def publish_sensor_data(self):
        """Publish simulated sensor data from Isaac ROS"""
        # Publish RGB image
        self.publish_rgb_image()

        # Publish depth image
        self.publish_depth_image()

        # Publish point cloud
        self.publish_pointcloud()

    def publish_rgb_image(self):
        """Publish RGB image data"""
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'rgb_camera_link'
        img_msg.height = 720
        img_msg.width = 1280
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = 0
        img_msg.step = 1280 * 3  # width * channels

        # Create dummy RGB image data
        rgb_data = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)
        # Add some structured patterns to simulate a scene
        for i in range(0, 720, 50):
            rgb_data[i:i+5, :, :] = [255, 0, 0]  # Horizontal red lines
        for j in range(0, 1280, 50):
            rgb_data[:, j:j+5, :] = [0, 255, 0]  # Vertical green lines

        img_msg.data = rgb_data.tobytes()
        self.rgb_publisher.publish(img_msg)

    def publish_depth_image(self):
        """Publish depth image data"""
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'depth_camera_link'
        img_msg.height = 720
        img_msg.width = 1280
        img_msg.encoding = '32FC1'  # 32-bit float
        img_msg.is_bigendian = 0
        img_msg.step = 1280 * 4  # width * 4 bytes per float

        # Create dummy depth data (in meters)
        depth_data = np.random.uniform(0.5, 10.0, (720, 1280)).astype(np.float32)
        # Add some depth patterns
        for i in range(300, 400):
            for j in range(500, 800):
                depth_data[i, j] = 2.0  # Create a "wall" at 2m

        img_msg.data = depth_data.tobytes()
        self.depth_publisher.publish(img_msg)

    def publish_pointcloud(self):
        """Publish point cloud data"""
        pc_msg = PointCloud2()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = 'pointcloud_frame'
        pc_msg.height = 1
        pc_msg.width = 1000  # Number of points
        pc_msg.is_dense = False
        pc_msg.is_bigendian = False

        # Define fields
        pc_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]
        pc_msg.point_step = 16  # 3 floats (xyz) + 1 uint32 (rgb) = 16 bytes
        pc_msg.row_step = pc_msg.point_step * pc_msg.width

        # Create point cloud data
        points = []
        for i in range(1000):
            # Generate points in a spherical pattern
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi)
            r = np.random.uniform(1.0, 5.0)

            x = r * np.sin(phi) * np.cos(theta)
            y = r * np.sin(phi) * np.sin(theta)
            z = r * np.cos(phi)

            # Create RGB value
            r_color = int(np.random.uniform(0, 255))
            g_color = int(np.random.uniform(0, 255))
            b_color = int(np.random.uniform(0, 255))
            rgb = struct.unpack('I', struct.pack('BBBB', b_color, g_color, r_color, 0))[0]

            points.extend([x, y, z, float(rgb)])

        # Convert to bytes
        pc_data = b''
        for val in points:
            if isinstance(val, float):
                pc_data += struct.pack('f', val)
            else:
                pc_data += struct.pack('f', float(val))

        pc_msg.data = pc_data
        self.pointcloud_publisher.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)

    sensor_publisher = SensorPublisherNode()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()