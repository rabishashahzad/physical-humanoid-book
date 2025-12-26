import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np
from builtin_interfaces.msg import Time
import tf_transformations


class VSLAMNode(Node):
    def __init__(self):
        super().__init__('vslam_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize VSLAM state
        self.prev_image = None
        self.prev_gray = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion (x, y, z, w)

        # Create subscribers for stereo or monocular camera data
        self.left_image_sub = self.create_subscription(
            Image,
            'left/image_rect',
            self.left_image_callback,
            10
        )

        self.right_image_sub = self.create_subscription(
            Image,
            'right/image_rect',
            self.right_image_callback,
            10
        )

        self.left_camera_info_sub = self.create_subscription(
            CameraInfo,
            'left/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publishers for VSLAM output
        self.odom_publisher = self.create_publisher(
            Odometry,
            'vslam/odometry',
            10
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            'vslam/pose',
            10
        )

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Feature detection parameters
        self.feature_params = dict(
            maxCorners=100,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7
        )

        # Lucas-Kanade tracking parameters
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )

        self.get_logger().info('VSLAM node initialized')

    def camera_info_callback(self, msg):
        """Process camera calibration information"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration parameters loaded')

    def left_image_callback(self, msg):
        """Process left camera image for VSLAM"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) if len(cv_image.shape) == 3 else cv_image

            # Process VSLAM if we have a previous frame
            if self.prev_gray is not None:
                self.process_vslam(self.prev_gray, gray, msg.header.stamp)

            # Store current frame for next iteration
            self.prev_gray = gray.copy()
        except Exception as e:
            self.get_logger().error(f'Error processing left image: {str(e)}')

    def right_image_callback(self, msg):
        """Process right camera image (for stereo depth estimation)"""
        # In a real implementation, this would be used for stereo processing
        pass

    def process_vslam(self, prev_gray, curr_gray, timestamp):
        """Process VSLAM using feature tracking"""
        try:
            # Detect features in the previous frame
            prev_features = cv2.goodFeaturesToTrack(
                prev_gray,
                mask=None,
                **self.feature_params
            )

            if prev_features is not None:
                # Track features using Lucas-Kanade optical flow
                curr_features, status, error = cv2.calcOpticalFlowPyrLK(
                    prev_gray, curr_gray,
                    prev_features, None,
                    **self.lk_params
                )

                # Filter good points
                good_new = curr_features[status == 1]
                good_old = prev_features[status == 1]

                if len(good_new) >= 10:  # Need minimum features to estimate motion
                    # Estimate motion from feature correspondences
                    motion_estimate = self.estimate_motion(good_old, good_new)

                    if motion_estimate is not None:
                        # Update position and orientation
                        self.update_pose(motion_estimate)

                        # Publish odometry and pose
                        self.publish_odometry(timestamp)
                        self.publish_pose(timestamp)

                        # Broadcast TF
                        self.broadcast_transform(timestamp)

        except Exception as e:
            self.get_logger().error(f'Error in VSLAM processing: {str(e)}')

    def estimate_motion(self, points_old, points_new):
        """Estimate camera motion from tracked features"""
        try:
            # Simple motion estimation using essential matrix
            if len(points_old) >= 5:
                # Calculate essential matrix
                E, mask = cv2.findEssentialMat(
                    points_new, points_old,
                    self.camera_matrix,
                    method=cv2.RANSAC,
                    prob=0.999,
                    threshold=1.0
                )

                if E is not None:
                    # Recover pose from essential matrix
                    _, R, t, mask_pose = cv2.recoverPose(
                        E, points_new, points_old, self.camera_matrix
                    )

                    # Convert rotation matrix to quaternion
                    quat = tf_transformations.quaternion_from_matrix(
                        np.block([[R, np.zeros((3, 1))], [np.zeros((1, 3)), 1]])
                    )

                    return {
                        'rotation': R,
                        'translation': t.flatten(),
                        'quaternion': quat
                    }

        except Exception as e:
            self.get_logger().error(f'Error estimating motion: {str(e)}')

        return None

    def update_pose(self, motion_estimate):
        """Update the estimated pose based on motion estimate"""
        if motion_estimate:
            # Apply translation
            self.position += motion_estimate['translation'] * 0.1  # Scale factor for stability

            # Apply rotation
            new_quat = motion_estimate['quaternion']
            # For simplicity, we'll just update the orientation directly
            # In a real implementation, proper quaternion multiplication would be needed
            self.orientation = new_quat

    def publish_odometry(self, timestamp):
        """Publish odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'camera_link'

        # Set position
        odom_msg.pose.pose.position.x = float(self.position[0])
        odom_msg.pose.pose.position.y = float(self.position[1])
        odom_msg.pose.pose.position.z = float(self.position[2])

        # Set orientation
        odom_msg.pose.pose.orientation.x = float(self.orientation[0])
        odom_msg.pose.pose.orientation.y = float(self.orientation[1])
        odom_msg.pose.pose.orientation.z = float(self.orientation[2])
        odom_msg.pose.pose.orientation.w = float(self.orientation[3])

        # Set velocities (estimated)
        odom_msg.twist.twist.linear.x = 0.1  # Placeholder
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_publisher.publish(odom_msg)

    def publish_pose(self, timestamp):
        """Publish pose message"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = float(self.position[0])
        pose_msg.pose.position.y = float(self.position[1])
        pose_msg.pose.position.z = float(self.position[2])

        pose_msg.pose.orientation.x = float(self.orientation[0])
        pose_msg.pose.orientation.y = float(self.orientation[1])
        pose_msg.pose.orientation.z = float(self.orientation[2])
        pose_msg.pose.orientation.w = float(self.orientation[3])

        self.pose_publisher.publish(pose_msg)

    def broadcast_transform(self, timestamp):
        """Broadcast transform from map to camera"""
        t = TransformStamped()

        t.header.stamp = timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_link'

        t.transform.translation.x = float(self.position[0])
        t.transform.translation.y = float(self.position[1])
        t.transform.translation.z = float(self.position[2])

        t.transform.rotation.x = float(self.orientation[0])
        t.transform.rotation.y = float(self.orientation[1])
        t.transform.rotation.z = float(self.orientation[2])
        t.transform.rotation.w = float(self.orientation[3])

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    vslam_node = VSLAMNode()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()