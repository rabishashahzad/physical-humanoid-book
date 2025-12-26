# Quickstart Guide: Module 3 - AI-Robot Brain (NVIDIA Isaac)

**Feature**: Module 3 - AI-Robot Brain (NVIDIA Isaac)
**Date**: 2025-12-17
**Input**: Feature specification from `/specs/module-3-ai-robot-brain/spec.md`

## Prerequisites

Before starting this module, ensure you have:

1. **ROS 2 Background**: Understanding of ROS 2 concepts (nodes, topics, services)
2. **NVIDIA GPU**: CUDA-compatible GPU with appropriate drivers
3. **Isaac Sim**: Installed and properly configured
4. **Isaac ROS**: Installed and compatible with your ROS 2 distribution
5. **Basic knowledge** of computer vision and navigation concepts

### Installation Check

Verify your Isaac Sim installation:
```bash
# Check Isaac Sim version and availability
isaac-sim --version  # or equivalent command based on your installation

# Verify CUDA installation
nvidia-smi
nvcc --version
```

Verify your ROS 2 and Isaac ROS installation:
```bash
# Check ROS 2 installation
ros2 --version

# Check Isaac ROS packages
ros2 pkg list | grep isaac_ros
```

## Getting Started with Isaac Sim Basics

### 1. Launch Isaac Sim

```bash
# Launch Isaac Sim with basic environment
isaac-sim --exec "omni.isaac.examples.robots.1001_turtlebot3"

# Or launch with specific configuration
isaac-sim --config "config/robot_simulation.json"
```

### 2. Basic Robot Simulation

Create a simple robot configuration file `basic_robot_config.json`:
```json
{
  "scene": {
    "name": "Basic Scene",
    "usd_path": "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
  },
  "robots": [
    {
      "name": "turtlebot3",
      "usd_path": "/Isaac/Robots/TurtleBot3Burger/turtlebot3_burger.usd",
      "position": [0, 0, 0.1],
      "orientation": [0, 0, 0, 1]
    }
  ],
  "sensors": [
    {
      "name": "camera",
      "type": "rgb_camera",
      "parent_link": "camera_link",
      "parameters": {
        "resolution": [640, 480],
        "fov": 60
      },
      "ros_topic": "/camera/image_raw"
    }
  ]
}
```

### 3. Configure Physics and Environment

In Isaac Sim, you can configure physics properties through the UI or via USD files:
- Set gravity to Earth's gravity (9.81 m/s²)
- Configure material properties for realistic interactions
- Set up collision properties for accurate physics simulation

## Isaac ROS (VSLAM) Integration

### 1. Set up VSLAM Pipeline

Create a launch file for VSLAM: `vslam_pipeline.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # Isaac ROS Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_rectified_edge': True,
            'enable_debug_mode': False,
            'rectified_images_only': True,
            'enable_slam': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'confidence_threshold': 0.5,
            'max_num_landmarks': 1000,
        }],
        remappings=[
            ('/visual_slam/image', '/camera/image_rect_color'),
            ('/visual_slam/camera_info', '/camera/camera_info'),
            ('/visual_slam/imu', '/imu/data'),
        ],
        output='screen'
    )

    # Isaac ROS Image Rectification node
    image_rectify_node = Node(
        package='isaac_ros_image_rectifier',
        executable='image_rectifier_node',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('image_rect', '/camera/image_rect_color'),
        ],
        output='screen'
    )

    return LaunchDescription([
        image_rectify_node,
        visual_slam_node,
    ])
```

### 2. Launch VSLAM Pipeline

```bash
# Source ROS 2 and Isaac ROS
source /opt/ros/humble/setup.bash
source /opt/isaac_ros/setup.sh  # Adjust path as needed

# Launch the VSLAM pipeline
ros2 launch your_package/vslam_pipeline.launch.py
```

### 3. Visualize VSLAM Results

```bash
# In another terminal, visualize the results
rviz2 -d /opt/isaac_ros/share/isaac_ros_visual_slam/rviz/visual_slam.rviz
```

## Nav2 Navigation Integration

### 1. Configure Nav2 for Isaac Sim

Create a Nav2 configuration file `nav2_config.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: $(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_w_replanning_and_recovery.xml
    default_nav_to_pose_bt_xml: $(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_w_replanning_and_recovery.xml
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Controller parameters
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      rotation_shim:
        plugin: "nav2_rotation_shim_controller::RotationShim"
        max_angular_velocity_overshoot: 1.5
        goal_angle_offset: 0.0
        simulate_ahead_time: 1.0
        consider_docking_heading: false
        docking_heading_threshold: 0.1
        min_error_to_correct: 0.05
        min_error_to_rotate: 0.1

      nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController:
        plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        desired_linear_vel: 0.5
        max_linear_accel: 2.5
        max_linear_decel: 2.5
        desired_angular_vel: 1.0
        max_angular_accel: 3.2
        min_turn_radius: 0.0
        lookahead_dist: 0.6
        min_lookahead_dist: 0.3
        max_lookahead_dist: 0.9
        use_velocity_scaled_lookahead_dist: false
        lookahead_time: 1.5
        use_interpolation: true
        use_regulated_linear_velocity: false
        use_regulated_angular_velocity: false
        regulated_linear_scaling_min_radius: 0.9
        regulated_linear_scaling_min_speed: 0.25
        use_cost_regulated_linear_velocity: true
        cost_scaling_dist: 1.0
        cost_scaling_gain: 1.0
        inflation_cost_scaling_factor: 3.0
        replanning_enabled: true
        replanning_time_threshold: 0.5
```

### 2. Launch Navigation System

Create a launch file `navigation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('your_package'),
                'config',
                'nav2_config.yaml'
            ]),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Automatically startup the nav2 stack'),

        navigation_launch,
    ])
```

### 3. Run Navigation

```bash
# Source ROS 2 and navigation packages
source /opt/ros/humble/setup.bash
source /opt/navigation/setup.sh  # Adjust path as needed

# Launch navigation
ros2 launch your_package navigation.launch.py
```

## Integration Example: Complete AI Robot Brain

Here's an example of how to integrate Isaac Sim, Isaac ROS VSLAM, and Nav2:

```python
# complete_robot_brain.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class AIBrainNode(Node):
    def __init__(self):
        super().__init__('ai_brain_node')

        # Create action client for navigation
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Subscribe to VSLAM pose estimates
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/pose',
            self.pose_callback,
            10
        )

        # Subscribe to other sensors as needed
        # (IMU, camera, etc.)

        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.get_logger().info(f'Updated pose: {msg.pose}')

    def navigate_to_goal(self, x, y, theta):
        """Send navigation goal to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta  # Simplified orientation

        self.get_logger().info(f'Sending navigation goal: ({x}, {y}, {theta})')

        future = self.nav_client.send_goal_async(goal_msg)
        return future

def main(args=None):
    rclpy.init(args=args)
    ai_brain = AIBrainNode()

    # Example: Navigate to a specific point
    ai_brain.navigate_to_goal(5.0, 3.0, 0.0)

    try:
        rclpy.spin(ai_brain)
    except KeyboardInterrupt:
        pass
    finally:
        ai_brain.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

1. **Chapter 1**: Complete the Isaac Sim basics tutorial with physics and sensor configuration
2. **Chapter 2**: Implement VSLAM pipeline with Isaac ROS and process visual data
3. **Chapter 3**: Configure Nav2 navigation and integrate with Isaac Sim environment

## Troubleshooting

### Common Issues

1. **GPU Memory**: Isaac Sim requires significant GPU memory; reduce scene complexity if needed
2. **CUDA Compatibility**: Ensure Isaac ROS packages are compatible with your CUDA version
3. **Timing Issues**: Use `use_sim_time: True` when working with Isaac Sim
4. **Sensor Calibration**: Calibrate sensors properly for accurate VSLAM results

### Getting Help

- Check the Isaac ROS documentation: https://nvidia-isaac-ros.github.io/
- Use `ros2 launch` commands to test individual components
- Monitor topics with `ros2 topic echo` to verify data flow
- Use `rqt_graph` to visualize the ROS graph