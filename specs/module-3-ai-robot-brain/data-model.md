# Data Model: Module 3 - AI-Robot Brain (NVIDIA Isaac)

**Feature**: Module 3 - AI-Robot Brain (NVIDIA Isaac)
**Date**: 2025-12-17
**Input**: Feature specification from `/specs/module-3-ai-robot-brain/spec.md`

## Overview

This document defines the data structures and models used in Module 3: AI-Robot Brain (NVIDIA Isaac). It covers the conceptual data models for Isaac Sim simulation, Isaac ROS perception, VSLAM algorithms, and Nav2 navigation.

## Isaac Sim Data Models

### IsaacSimEnvironment Model
```
IsaacSimEnvironment:
  - name: string (environment name)
  - description: string (environment description)
  - usd_path: string (USD file path)
  - physics_properties: PhysicsProperties
  - lighting_settings: LightingSettings
  - objects: list<SimObject>
  - robots: list<RobotConfig>
  - sensors: list<SensorConfig>
  - extensions: list<string> (loaded extensions)
  - gpu_settings: GPUSettings
```

### PhysicsProperties Model
```
PhysicsProperties:
  - gravity: Vector3 (gravity vector)
  - solver_type: string (PBD, Featherstone, etc.)
  - substeps: int (physics substeps per frame)
  - fixed_timestep: float (physics timestep)
  - enable_gpu: boolean (GPU physics acceleration)
```

### SimObject Model
```
SimObject:
  - name: string (object name)
  - usd_path: string (USD asset path)
  - pose: Pose (position and orientation)
  - physics: PhysicsConfig
  - visual: VisualConfig
  - collision: CollisionConfig
  - static: boolean (immovable object)
```

### RobotConfig Model
```
RobotConfig:
  - name: string (robot name)
  - usd_path: string (robot USD path)
  - pose: Pose (initial pose)
  - joints: list<JointConfig>
  - links: list<LinkConfig>
  - controllers: list<ControllerConfig>
  - sensors: list<SensorConfig>
  - drive_type: DriveType (ackermann, differential, etc.)
```

### SensorConfig Model
```
SensorConfig:
  - name: string (sensor name)
  - type: SensorType (camera, lidar, imu, etc.)
  - pose: Pose (sensor pose relative to parent)
  - parent_link: string (link to attach sensor to)
  - parameters: dict<string, any> (sensor-specific parameters)
  - ros_topic: string (ROS topic name for output)
  - update_rate: float (sensor update rate in Hz)
```

### GPUSettings Model
```
GPUSettings:
  - device_id: int (GPU device ID)
  - memory_budget: int (GPU memory in MB)
  - render_resolution: Resolution (render resolution)
  - compute_capability: string (CUDA compute capability)
  - multi_gpu_enabled: boolean
```

## Isaac ROS Data Models

### IsaacROSNode Model
```
IsaacROSNode:
  - name: string (node name)
  - package: string (ROS package name)
  - executable: string (executable name)
  - parameters: dict<string, any> (node parameters)
  - input_topics: list<TopicConfig>
  - output_topics: list<TopicConfig>
  - gpu_enabled: boolean (GPU acceleration enabled)
  - tensorrt_config: TensorRTConfig (if applicable)
```

### TopicConfig Model
```
TopicConfig:
  - name: string (topic name)
  - type: string (message type)
  - qos_profile: QoSProfile
  - queue_size: int
  - reliability: ReliabilityPolicy
  - durability: DurabilityPolicy
```

### TensorRTConfig Model
```
TensorRTConfig:
  - engine_path: string (path to TensorRT engine)
  - input_tensor_names: list<string>
  - output_tensor_names: list<string>
  - batch_size: int
  - precision: PrecisionType (fp32, fp16, int8)
  - dynamic_shapes: dict<string, ShapeRange>
```

### VSLAMPipeline Model
```
VSLAMPipeline:
  - name: string (pipeline name)
  - input_streams: list<StreamConfig> (camera and IMU streams)
  - feature_detector: FeatureDetectorConfig
  - tracker: TrackerConfig
  - mapper: MapperConfig
  - optimizer: OptimizerConfig
  - output_topics: list<string> (pose, map, etc.)
  - parameters: VSLAMParameters
```

### VSLAMParameters Model
```
VSLAMParameters:
  - max_features: int (maximum features to track)
  - min_matches: int (minimum matches for tracking)
  - relocalization_threshold: float
  - map_merge_threshold: float
  - keyframe_threshold: float
  - bundle_adjustment_enabled: boolean
  - loop_closure_enabled: boolean
```

## Nav2 Navigation Data Models

### NavigationConfig Model
```
NavigationConfig:
  - robot_radius: float (robot radius for collision checking)
  - global_frame: string (global frame name)
  - robot_base_frame: string (robot base frame)
  - transform_tolerance: float (transform tolerance)
  - recovery_enabled: boolean
  - clear_costmap: boolean
  - global_planner: GlobalPlannerConfig
  - local_planner: LocalPlannerConfig
  - controller: ControllerConfig
  - recovery_behaviors: list<RecoveryBehaviorConfig>
```

### GlobalPlannerConfig Model
```
GlobalPlannerConfig:
  - type: string (planner type)
  - costmap_topic: string
  - footprint_topic: string
  - parameters: dict<string, any>
  - allow_unknown: boolean
  - planner_frequency: float
```

### LocalPlannerConfig Model
```
LocalPlannerConfig:
  - type: string (planner type)
  - costmap_topic: string
  - footprint_topic: string
  - parameters: dict<string, any>
  - controller_frequency: float
  - velocity_scaling_enabled: boolean
```

### ControllerConfig Model
```
ControllerConfig:
  - type: string (controller type)
  - max_linear_speed: float
  - max_angular_speed: float
  - min_linear_speed: float
  - min_angular_speed: float
  - xy_goal_tolerance: float
  - yaw_goal_tolerance: float
```

### RecoveryBehaviorConfig Model
```
RecoveryBehaviorConfig:
  - name: string (behavior name)
  - type: string (behavior type)
  - parameters: dict<string, any>
  - enabled: boolean
```

## Perception Pipeline Models

### PerceptionPipeline Model
```
PerceptionPipeline:
  - name: string (pipeline name)
  - nodes: list<PerceptionNode>
  - connections: list<Connection>
  - input_topics: list<string>
  - output_topics: list<string>
  - gpu_config: GPUConfig
  - performance_metrics: PerformanceMetrics
```

### PerceptionNode Model
```
PerceptionNode:
  - id: string (unique node ID)
  - type: NodeType (image_proc, feature_detector, etc.)
  - parameters: dict<string, any>
  - input_ports: list<PortConfig>
  - output_ports: list<PortConfig>
  - gpu_accelerated: boolean
```

### PortConfig Model
```
PortConfig:
  - name: string (port name)
  - type: string (data type)
  - connection_type: ConnectionType (publisher, subscriber)
```

## Sensor Data Models

### ImageData Model
```
ImageData:
  - timestamp: float (timestamp)
  - width: int (image width)
  - height: int (image height)
  - encoding: string (image encoding)
  - step: int (bytes per row)
  - data: bytes (image data)
  - camera_info: CameraInfo
  - frame_id: string
```

### CameraInfo Model
```
CameraInfo:
  - width: int
  - height: int
  - distortion_model: string
  - distortion_coefficients: list<float>
  - intrinsic_matrix: list<float> (9 elements)
  - projection_matrix: list<float> (12 elements)
  - rectification_matrix: list<float> (9 elements)
```

### PointCloudData Model
```
PointCloudData:
  - timestamp: float
  - width: int
  - height: int
  - fields: list<PointField>
  - is_bigendian: boolean
  - point_step: int
  - row_step: int
  - data: bytes
  - is_dense: boolean
  - frame_id: string
```

### PointField Model
```
PointField:
  - name: string
  - offset: int
  - datatype: PointFieldType (INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64)
  - count: int
```

## Navigation State Models

### RobotState Model
```
RobotState:
  - pose: Pose (current pose)
  - twist: Twist (current velocity)
  - timestamp: float
  - frame_id: string
  - covariance: list<float> (pose covariance)
```

### NavigationGoal Model
```
NavigationGoal:
  - pose: Pose (target pose)
  - frame_id: string (target frame)
  - behavior_tree: string (BT XML for navigation)
  - goal_checker: GoalCheckerConfig
  - planner_id: string (global planner ID)
  - controller_id: string (controller ID)
  - tolerance: float (goal tolerance)
```

### Path Model
```
Path:
  - poses: list<Pose> (waypoints)
  - header: Header
  - frame_id: string
  - timestamp: float
```

## Isaac Sim-ROS Bridge Models

### BridgeConfig Model
```
BridgeConfig:
  - type: BridgeType (pubsub, service, action)
  - ros_topic: string (ROS topic name)
  - omni_topic: string (Omniverse topic name)
  - message_type: string (message type)
  - qos_profile: QoSProfile
  - direction: DirectionType (ros_to_omni, omni_to_ros, bidirectional)
```

### SimulationState Model
```
SimulationState:
  - timestamp: float
  - sim_time: float
  - real_time_factor: float
  - paused: boolean
  - entities: list<EntityState>
  - sensors: list<SensorState>
```

### EntityState Model
```
EntityState:
  - entity_id: string
  - pose: Pose
  - velocity: Twist
  - acceleration: Acceleration
  - timestamp: float
  - frame_id: string
```

## Example Isaac Sim Configuration

```
ExampleIsaacSimConfig:
  - environment: IsaacSimEnvironment
    - name: "warehouse_simulation"
    - usd_path: "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    - physics_properties:
        gravity: [0, 0, -9.81]
        solver_type: "PBD"
        substeps: 8
        fixed_timestep: 0.008333  # 120 Hz
    - robots:
        - RobotConfig:
            name: "turtlebot3"
            usd_path: "/Isaac/Robots/TurtleBot3Burger/turtlebot3_burger.usd"
            pose: {position: [0, 0, 0.1], orientation: [0, 0, 0, 1]}
            sensors:
              - SensorConfig:
                  name: "camera"
                  type: "rgb_camera"
                  parent_link: "camera_link"
                  parameters: {resolution: [640, 480], fov: 60}
                  ros_topic: "/camera/image_raw"
              - SensorConfig:
                  name: "imu"
                  type: "imu"
                  parent_link: "base_link"
                  ros_topic: "/imu/data"
    - gpu_settings:
        device_id: 0
        memory_budget: 4096
        render_resolution: {width: 1280, height: 720}
```

## Example VSLAM Configuration

```
ExampleVSLAMConfig:
  - pipeline: VSLAMPipeline
    - name: "orb_slam_pipeline"
    - input_streams:
        - StreamConfig:
            type: "camera"
            topic: "/camera/image_raw"
            camera_info_topic: "/camera/camera_info"
        - StreamConfig:
            type: "imu"
            topic: "/imu/data"
    - feature_detector:
        type: "ORB"
        parameters: {n_features: 2000, scale_factor: 1.2, n_levels: 8}
    - tracker:
        type: "feature_tracker"
        parameters: {max_tracks: 1000, track_threshold: 10}
    - mapper:
        type: "keyframe_mapper"
        parameters: {keyframe_threshold: 0.1, bundle_adjustment: true}
    - output_topics:
        - "/visual_slam/pose"
        - "/visual_slam/map"
        - "/visual_slam/path"
    - parameters: VSLAMParameters
        max_features: 2000
        min_matches: 20
        relocalization_threshold: 0.5
        map_merge_threshold: 0.8
        bundle_adjustment_enabled: true
        loop_closure_enabled: true
```

## Example Nav2 Configuration

```
ExampleNav2Config:
  - config: NavigationConfig
    - robot_radius: 0.2
    - global_frame: "map"
    - robot_base_frame: "base_link"
    - transform_tolerance: 0.2
    - recovery_enabled: true
    - global_planner: GlobalPlannerConfig
        type: "nav2_navfn_planner/NavfnPlanner"
        parameters: {use_astar: false, allow_unknown: true}
    - local_planner: LocalPlannerConfig
        type: "nav2_dwb_controller/DWBLocalPlanner"
        parameters:
          {max_vel_x: 0.5, min_vel_x: -0.25, max_vel_y: 0.0, min_vel_y: 0.0,
           max_vel_theta: 1.0, min_vel_theta: -1.0, acc_lim_x: 2.5, acc_lim_y: 0.0,
           acc_lim_theta: 3.2}
    - controller: ControllerConfig
        type: "FollowPath"
        max_linear_speed: 0.5
        max_angular_speed: 1.0
        xy_goal_tolerance: 0.25
        yaw_goal_tolerance: 0.25
    - recovery_behaviors:
        - RecoveryBehaviorConfig:
            name: "spin"
            type: "nav2_spin_spin"
            parameters: {spin_dist: 1.57}
        - RecoveryBehaviorConfig:
            name: "backup"
            type: "nav2_backup_backup"
            parameters: {backup_dist: 0.15, backup_speed: 0.05}
```