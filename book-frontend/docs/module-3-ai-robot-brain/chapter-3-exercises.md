---
title: Chapter 3 Exercises - Humanoid Navigation Practical Applications
sidebar_position: 7
---

# Chapter 3 Exercises: Humanoid Navigation Practical Applications

## Exercise 3.1: Setting Up Humanoid-Specific Navigation Stack

### Objective
Configure and launch a navigation stack specifically adapted for humanoid robot characteristics.

### Steps
1. Set up humanoid-specific costmap configuration
2. Configure balance-aware path planning
3. Launch the humanoid navigation stack
4. Test basic navigation capabilities

### Requirements
- Isaac Sim with humanoid robot model
- Isaac ROS perception stack running
- Nav2 navigation stack installed

### Instructions

1. **Create humanoid navigation configuration:**
   ```yaml
   # humanoid_nav_config.yaml
   amcl:
     ros__parameters:
       use_sim_time: true
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

   local_costmap:
     ros__parameters:
       use_sim_time: true
       global_frame: "odom"
       robot_base_frame: "base_link"
       update_frequency: 5.0
       publish_frequency: 2.0
       static_map: false
       rolling_window: true
       width: 6.0
       height: 6.0
       resolution: 0.05
       # Humanoid-specific footprint (larger to account for balance)
       footprint: "[[-0.4, -0.3], [-0.4, 0.3], [0.4, 0.3], [0.4, -0.3]]"
       footprint_padding: 0.05
       plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
       inflation_layer:
         cost_scaling_factor: 3.0
         inflation_radius: 0.55
       obstacle_layer:
         enabled: true
         observation_sources: scan
         scan:
           topic: /scan
           max_obstacle_height: 2.0  # Humanoid height consideration
           clearing: true
           marking: true
           data_type: "LaserScan"
           obstacle_range: 2.5
           raytrace_range: 3.0
       voxel_layer:
         enabled: true
         max_obstacle_height: 2.0
         origin_z: 0.0
         z_resolution: 0.2
         z_voxels: 10
         unknown_threshold: 15
         mark_threshold: 0
         observation_sources: point_cloud_sensor
         point_cloud_sensor:
           topic: /head_lidar/points
           max_obstacle_height: 2.0
           clearing: true
           marking: true
           data_type: "PointCloud2"
           obstacle_range: 2.5
   ```

2. **Create humanoid navigation launch file:**
   ```python
   # humanoid_navigation.launch.py
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

       # Humanoid-specific controller
       humanoid_controller = Node(
           package='humanoid_nav_controller',
           executable='humanoid_controller',
           parameters=[{
               'use_sim_time': use_sim_time,
               'step_size_limit': 0.5,
               'turn_rate_limit': 0.52,
               'balance_margin': 0.1,
           }],
           remappings=[
               ('cmd_vel', 'humanoid_cmd_vel'),
               ('odom', 'odom'),
           ],
           output='screen'
       )

       return LaunchDescription([
           DeclareLaunchArgument(
               'use_sim_time',
               default_value='True',
               description='Use simulation (Gazebo) clock if true'),

           DeclareLaunchArgument(
               'params_file',
               default_value=PathJoinSubstitution([
                   FindPackageShare('humanoid_nav_bringup'),
                   'config',
                   'humanoid_nav_config.yaml'
               ]),
               description='Full path to the ROS2 parameters file to use for all launched nodes'),

           DeclareLaunchArgument(
               'autostart',
               default_value='True',
               description='Automatically startup the nav2 stack'),

           navigation_launch,
           humanoid_controller,
       ])
   ```

3. **Launch the navigation stack:**
   ```bash
   # Source ROS 2 and navigation packages
   source /opt/ros/humble/setup.bash
   source /opt/navigation/setup.sh  # Adjust path as needed

   # Launch humanoid navigation
   ros2 launch humanoid_nav_bringup humanoid_navigation.launch.py
   ```

### Expected Outcome
- Navigation stack launches without errors
- Humanoid-specific costmap accounts for robot dimensions
- Path planning considers humanoid constraints
- Controller generates humanoid-appropriate commands

## Exercise 3.2: Humanoid Footstep Planning Integration

### Objective
Integrate footstep planning with the navigation system for stable humanoid locomotion.

### Steps
1. Configure footstep planner parameters
2. Integrate with path planner
3. Test stability during navigation
4. Evaluate balance maintenance

### Instructions

1. **Footstep planner configuration:**
   ```python
   # footstep_planner_config.py
   footstep_planner_params = {
       'step_size_limit': 0.5,      # Maximum step size in meters
       'step_width_limit': 0.4,     # Maximum step width
       'step_rotation_limit': 0.52, # Maximum rotation per step (30 degrees)
       'support_polygon_margin': 0.1,  # Safety margin for support polygon
       'min_step_duration': 0.5,    # Minimum time per step
       'max_step_duration': 2.0,    # Maximum time per step
       'swing_height': 0.1,         # Height of swing foot
       'stance_time': 0.1,          # Time in double support phase
   }
   ```

2. **Integration with navigation:**
   ```python
   class HumanoidPathFollower:
       def __init__(self):
           self.footstep_planner = FootstepPlanner()
           self.balance_controller = BalanceController()
           self.path_tracker = PathTracker()

       def follow_path(self, path):
           # Convert global path to footstep sequence
           footsteps = self.footstep_planner.plan_footsteps(path)

           # Execute footsteps with balance control
           for step in footsteps:
               self.balance_controller.maintain_balance()
               self.execute_footstep(step)
               self.balance_controller.verify_stability()
   ```

3. **Testing stability:**
   ```bash
   # Monitor balance metrics
   ros2 topic echo /balance_metrics

   # Check ZMP (Zero Moment Point)
   ros2 topic echo /zmp_reference
   ros2 topic echo /zmp_actual
   ```

### Expected Outcome
- Footstep planner generates stable step sequences
- Balance is maintained during navigation
- Path following is smooth and stable
- Recovery behaviors activate when needed

## Exercise 3.3: Stair Navigation for Humanoids

### Objective
Implement and test stair navigation capabilities for humanoid robots.

### Steps
1. Configure stair detection system
2. Plan safe climbing trajectories
3. Execute stair climbing behavior
4. Validate safety and stability

### Instructions

1. **Stair detection configuration:**
   ```python
   # stair_detection_config.py
   stair_detector_params = {
       'min_stair_height': 0.1,     # Minimum detectable step height
       'max_stair_height': 0.25,    # Maximum step height for humanoid
       'min_stair_depth': 0.2,      # Minimum step depth
       'max_stair_depth': 0.5,      # Maximum step depth
       'stair_angle_tolerance': 0.1, # Tolerance for level stairs
       'detection_range': 1.5,      # Range for stair detection
       'confidence_threshold': 0.7, # Minimum confidence for detection
   }
   ```

2. **Stair climbing behavior:**
   ```python
   class StairClimbingBehavior:
       def __init__(self):
           self.stair_detector = StairDetector()
           self.footstep_planner = FootstepPlanner()
           self.balance_controller = BalanceController()

       def climb_stairs(self, stair_info):
           # Approach stairs safely
           self.approach_stairs(stair_info['start'])

           # Execute climbing sequence
           for step_idx in range(stair_info['num_steps']):
               # Plan footstep for this step
               footstep = self.plan_stair_footstep(step_idx, stair_info)

               # Execute with balance control
               self.balance_controller.set_stair_mode(True)
               self.execute_footstep(footstep)
               self.balance_controller.verify_stability()

           # Depart from stairs
           self.depart_stairs(stair_info['end'])
   ```

3. **Testing stair navigation:**
   ```bash
   # Test with different stair configurations
   # Monitor joint torques during climbing
   ros2 topic echo /joint_states

   # Check balance during stair climbing
   ros2 topic echo /center_of_mass
   ```

### Expected Outcome
- Stairs are properly detected and classified
- Safe climbing trajectories are generated
- Balance is maintained during climbing
- Robot successfully navigates various stair configurations

## Exercise 3.4: Human-Aware Navigation

### Objective
Implement navigation behaviors that consider human presence and social conventions.

### Steps
1. Set up human detection and tracking
2. Configure social navigation parameters
3. Implement human-aware path planning
4. Test social navigation behaviors

### Instructions

1. **Human detection configuration:**
   ```yaml
   # social_navigation_config.yaml
   human_detector:
     ros__parameters:
       detection_range: 3.0
       tracking_timeout: 2.0
       min_detection_confidence: 0.8
       max_tracked_humans: 10
       personal_space_radius: 0.8  # Distance to maintain from humans
   ```

2. **Social navigation parameters:**
   ```python
   # social_nav_params.py
   social_navigation_params = {
       'personal_space_radius': 0.8,    # Maintain distance from humans
       'social_zone_inflation': 0.5,    # Inflate costmap around humans
       'right_side_passing': True,      # Pass on right side when possible
       'eye_contact_distance': 1.5,     # Distance for "eye contact"
       'group_awareness': True,         # Consider groups of humans
       'dynamic_prediction_horizon': 2.0,  # Predict human motion
   }
   ```

3. **Human-aware path planning:**
   ```python
   class SocialPathPlanner:
       def plan_path(self, start, goal, humans):
           # Calculate paths that respect personal space
           base_path = self.base_planner.plan_path(start, goal)

           # Modify path to avoid human personal space
           social_path = self.avoid_human_space(base_path, humans)

           # Optimize for social compliance
           final_path = self.optimize_for_social_norms(social_path, humans)

           return final_path
   ```

4. **Testing social navigation:**
   ```bash
   # Launch with human models in Isaac Sim
   # Monitor navigation around humans
   ros2 topic echo /social_navigation_metrics
   ```

### Expected Outcome
- Humans are detected and tracked reliably
- Personal space is maintained during navigation
- Social conventions are followed
- Navigation remains efficient while being socially aware

## Exercise 3.5: Multi-Modal Navigation (Stairs, Doors, etc.)

### Objective
Implement navigation that can handle multiple types of obstacles and transitions.

### Steps
1. Create multi-modal path planner
2. Implement obstacle-specific behaviors
3. Test transitions between different navigation modes
4. Validate safety and efficiency

### Instructions

1. **Multi-modal navigation configuration:**
   ```python
   # multimodal_nav_config.py
   multimodal_params = {
       'navigation_modes': [
           'walking',      # Normal walking navigation
           'stair_climbing', # Stair navigation mode
           'door_passing',  # Door navigation mode
           'elevator_use',  # Elevator navigation mode
       ],
       'transition_detection_range': 2.0,
       'mode_switch_timeout': 5.0,
       'safety_margin_multiplier': 2.0,  # Extra safety for transitions
   }
   ```

2. **Mode-specific behaviors:**
   ```python
   class MultiModalNavigator:
       def __init__(self):
           self.walking_planner = WalkingPlanner()
           self.stair_planner = StairPlanner()
           self.door_planner = DoorPlanner()
           self.mode_selector = ModeSelector()

       def navigate(self, goal):
           # Detect required navigation mode
           required_mode = self.mode_selector.select_mode(goal)

           # Switch to appropriate mode
           self.switch_mode(required_mode)

           # Execute navigation in selected mode
           if required_mode == 'stair_climbing':
               return self.execute_stair_navigation(goal)
           elif required_mode == 'door_passing':
               return self.execute_door_navigation(goal)
           else:
               return self.execute_walking_navigation(goal)
   ```

3. **Mode transition testing:**
   ```bash
   # Test transitions between different navigation modes
   # Monitor mode switching behavior
   ros2 topic echo /navigation_mode
   ros2 topic echo /transition_status
   ```

### Expected Outcome
- Robot correctly identifies required navigation mode
- Smooth transitions between different navigation behaviors
- Safety maintained during mode transitions
- Efficient navigation across complex environments

## Self-Assessment Questions

1. How does humanoid-specific footprint configuration affect navigation performance?
2. What are the key differences between bipedal and wheeled robot navigation?
3. How can social navigation be balanced with navigation efficiency?
4. What safety measures are essential for humanoid navigation in human environments?

## Solutions

Solutions to these exercises can be found in the Module 3 Solutions Guide.