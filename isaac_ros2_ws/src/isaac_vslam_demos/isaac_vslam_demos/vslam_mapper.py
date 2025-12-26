import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformListener, Buffer
from cv_bridge import CvBridge
import cv2
import numpy as np
from builtin_interfaces.msg import Time
import struct
import tf_transformations


class VSLAMMapperNode(Node):
    def __init__(self):
        super().__init__('vslam_mapper')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize map data
        self.map_resolution = 0.05  # meters per pixel
        self.map_width = 1000  # pixels
        self.map_height = 1000  # pixels
        self.map_origin_x = -25.0  # meters
        self.map_origin_y = -25.0  # meters
        self.occupancy_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.occupancy_map.fill(-1)  # Unknown (-1), Free (0), Occupied (100)

        # Initialize features and map points
        self.keyframes = []
        self.map_points = {}  # 3D points in global coordinate system
        self.next_point_id = 0

        # Create subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'vslam/pose',
            self.pose_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            'left/image_rect',
            self.image_callback,
            10
        )

        # Create publishers
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            'vslam/map',
            10
        )

        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            'vslam/pointcloud',
            10
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'vslam/landmarks',
            10
        )

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically publish the map
        self.map_timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info('VSLAM Mapper node initialized')

    def pose_callback(self, msg):
        """Process pose information for mapping"""
        # Store keyframe when significant movement occurs
        current_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # Check if we should create a new keyframe (significant movement)
        if len(self.keyframes) == 0:
            self.keyframes.append({
                'timestamp': msg.header.stamp,
                'position': current_pos,
                'orientation': np.array([
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                ])
            })
        else:
            last_pos = self.keyframes[-1]['position']
            distance = np.linalg.norm(current_pos - last_pos)

            if distance > 0.5:  # Create keyframe if moved more than 0.5m
                self.keyframes.append({
                    'timestamp': msg.header.stamp,
                    'position': current_pos,
                    'orientation': np.array([
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w
                    ])
                })
                self.get_logger().info(f'Created keyframe at position: {current_pos}')

    def image_callback(self, msg):
        """Process image for feature extraction and mapping"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) if len(cv_image.shape) == 3 else cv_image

            # Extract features using ORB (for simplicity, could use more advanced methods)
            orb = cv2.ORB_create(nfeatures=500)
            keypoints, descriptors = orb.detectAndCompute(gray, None)

            if keypoints is not None and len(keypoints) > 0:
                # For each keypoint, try to triangulate a 3D point
                # This is a simplified approach - in real VSLAM, you'd use stereo or multiple views
                for kp in keypoints[:50]:  # Limit to first 50 features
                    # In a real implementation, you would triangulate 3D points
                    # For simulation, we'll create points relative to camera position
                    # This is a simplified approach for demonstration
                    pass

        except Exception as e:
            self.get_logger().error(f'Error processing image for mapping: {str(e)}')

    def add_map_point(self, point_3d, descriptor=None):
        """Add a 3D point to the map"""
        point_id = self.next_point_id
        self.next_point_id += 1

        self.map_points[point_id] = {
            'position': point_3d,
            'descriptor': descriptor,
            'observations': 1
        }

        return point_id

    def update_occupancy_map(self, pose):
        """Update the occupancy grid based on current pose"""
        # This is a simplified occupancy grid update
        # In a real implementation, you would integrate sensor data (e.g., depth, laser)

        # For demonstration, we'll add some simulated obstacles around the robot
        robot_x = pose.position.x
        robot_y = pose.position.y

        # Convert world coordinates to map coordinates
        map_x = int((robot_x - self.map_origin_x) / self.map_resolution)
        map_y = int((robot_y - self.map_origin_y) / self.map_resolution)

        # Add some obstacles in the map around the robot
        if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
            # Add a simulated obstacle in front of the robot
            obstacle_x = map_x + int(2.0 / self.map_resolution)  # 2m in front
            obstacle_y = map_y

            if 0 <= obstacle_x < self.map_width and 0 <= obstacle_y < self.map_height:
                self.occupancy_map[obstacle_y, obstacle_x] = 100  # Occupied

    def publish_map(self):
        """Publish the occupancy grid map"""
        try:
            # Create OccupancyGrid message
            map_msg = OccupancyGrid()
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = 'map'

            # Set map metadata
            map_msg.info.resolution = self.map_resolution
            map_msg.info.width = self.map_width
            map_msg.info.height = self.map_height
            map_msg.info.origin.position.x = self.map_origin_x
            map_msg.info.origin.position.y = self.map_origin_y
            map_msg.info.origin.position.z = 0.0
            map_msg.info.origin.orientation.x = 0.0
            map_msg.info.origin.orientation.y = 0.0
            map_msg.info.origin.orientation.z = 0.0
            map_msg.info.origin.orientation.w = 1.0

            # Flatten the occupancy map for the message
            map_msg.data = self.occupancy_map.flatten().tolist()

            self.map_publisher.publish(map_msg)

            # Publish visualization markers for landmarks
            self.publish_landmark_markers()

        except Exception as e:
            self.get_logger().error(f'Error publishing map: {str(e)}')

    def publish_landmark_markers(self):
        """Publish visualization markers for map landmarks"""
        marker_array = MarkerArray()

        # Create markers for each map point
        for point_id, point_data in self.map_points.items():
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'landmarks'
            marker.id = point_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set position
            pos = point_data['position']
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0

            # Set size and color
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def publish_pointcloud(self):
        """Publish a point cloud representation of the map"""
        # Create a simplified point cloud from the occupancy grid
        points = []
        for y in range(0, self.map_height, 10):  # Downsample for performance
            for x in range(0, self.map_width, 10):
                if self.occupancy_map[y, x] == 100:  # Occupied cells
                    world_x = self.map_origin_x + x * self.map_resolution
                    world_y = self.map_origin_y + y * self.map_resolution
                    # Assume height of 0.5m for obstacles
                    world_z = 0.5

                    # Create RGB value (red for obstacles)
                    rgb = struct.unpack('I', struct.pack('BBBB', 255, 0, 0, 0))[0]
                    points.extend([world_x, world_y, world_z, float(rgb)])

        if points:
            pc_msg = PointCloud2()
            pc_msg.header.stamp = self.get_clock().now().to_msg()
            pc_msg.header.frame_id = 'map'
            pc_msg.height = 1
            pc_msg.width = len(points) // 4  # 4 values per point (xyz + rgb)
            pc_msg.is_dense = False
            pc_msg.is_bigendian = False

            pc_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
            ]
            pc_msg.point_step = 16
            pc_msg.row_step = pc_msg.point_step * pc_msg.width

            # Convert points to bytes
            pc_data = b''
            for val in points:
                pc_data += struct.pack('f', float(val))

            pc_msg.data = pc_data
            self.pointcloud_publisher.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)

    vslam_mapper = VSLAMMapperNode()

    try:
        rclpy.spin(vslam_mapper)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()