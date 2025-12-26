# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: Module 2 - The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-17
**Input**: Feature specification from `/specs/module-2-digital-twin/spec.md`

## Overview

This document defines the data structures and models used in Module 2: The Digital Twin (Gazebo & Unity). It covers the conceptual data models for Gazebo simulation, Unity 3D environments, digital twin synchronization, and sensor simulation.

## Gazebo Data Models

### World Model
```
World:
  - name: string
  - version: string
  - gravity: Vector3 (x, y, z gravitational acceleration)
  - physics_engine: PhysicsEngineType (ode, bullet, dart)
  - physics_properties: PhysicsProperties
  - models: list<Model>
  - lights: list<Light>
  - plugins: list<Plugin>
  - frames: list<Frame>
```

### PhysicsProperties Model
```
PhysicsProperties:
  - max_step_size: float (maximum time step)
  - real_time_factor: float (real-time update rate)
  - real_time_update_rate: float (updates per second)
  - solver_type: string (quick, world, etc.)
  - iters: int (solver iterations)
  - sor: float (successive over-relaxation parameter)
```

### Model Model
```
Model:
  - name: string
  - pose: Pose (position and orientation)
  - static: boolean (immovable object)
  - canonical_link: string (primary reference link)
  - links: list<Link>
  - joints: list<Joint>
  - plugins: list<Plugin>
```

### Link Model
```
Link:
  - name: string
  - pose: Pose
  - inertial: Inertial
  - collision: Collision
  - visual: Visual
  - gravity: boolean (affected by gravity)
  - self_collide: boolean (self-collision enabled)
  - kinematic: boolean (kinematic joint)
  - must_be_base_link: boolean (root link)
```

### Joint Model
```
Joint:
  - name: string
  - type: JointType (revolute, continuous, prismatic, fixed, etc.)
  - parent: string (parent link name)
  - child: string (child link name)
  - pose: Pose
  - axis: JointAxis
  - physics: JointPhysics
```

### JointAxis Model
```
JointAxis:
  - xyz: Vector3 (axis of rotation/translation)
  - limit: JointLimit
  - dynamics: JointDynamics
  - use_parent_model_frame: boolean
```

### JointLimit Model
```
JointLimit:
  - lower: float (lower limit in radians/meters)
  - upper: float (upper limit in radians/meters)
  - effort: float (maximum effort)
  - velocity: float (maximum velocity)
```

### JointDynamics Model
```
JointDynamics:
  - damping: float (damping coefficient)
  - friction: float (friction coefficient)
  - spring_reference: float (spring reference position)
  - spring_stiffness: float (spring stiffness)
```

### Collision Model
```
Collision:
  - name: string
  - geometry: Geometry
  - surface: SurfaceProperties
  - max_contacts: int (maximum contact points)
```

### Visual Model
```
Visual:
  - name: string
  - geometry: Geometry
  - material: Material
  - cast_shadows: boolean
  - transparency: float (0.0-1.0)
  - laser_retro: float (laser retro value)
```

### Sensor Model
```
Sensor:
  - name: string
  - type: SensorType (camera, depth, rgbd, lidar, imu, gps, etc.)
  - pose: Pose
  - topic: string (ROS topic name)
  - update_rate: float (updates per second)
  - always_on: boolean
  - visualize: boolean
  - plugin: Plugin (optional)
  - sensor_specific: SensorSpecificData
```

### CameraSensor Model
```
CameraSensor:
  - horizontal_fov: float (horizontal field of view in radians)
  - image: ImageParameters
  - clip: ClipParameters
```

### ImageParameters Model
```
ImageParameters:
  - width: int (image width in pixels)
  - height: int (image height in pixels)
  - format: string (image format)
```

### ClipParameters Model
```
ClipParameters:
  - near: float (near clipping distance)
  - far: float (far clipping distance)
```

## Unity Data Models

### GameObject Model
```
GameObject:
  - name: string
  - tag: string
  - layer: int
  - active: boolean
  - components: list<Component>
  - transform: Transform
  - children: list<GameObject>
```

### Transform Model
```
Transform:
  - position: Vector3
  - rotation: Quaternion
  - scale: Vector3
  - parent: GameObject (optional)
```

### Component Model
```
Component:
  - type: ComponentType (MeshRenderer, Rigidbody, Camera, etc.)
  - properties: dict<string, any>
  - enabled: boolean
```

### Scene Model
```
Scene:
  - name: string
  - game_objects: list<GameObject>
  - lighting_settings: LightingSettings
  - physics_settings: PhysicsSettings
  - audio_settings: AudioSettings
```

### LightingSettings Model
```
LightingSettings:
  - ambient_light: Color
  - fog: boolean
  - fog_color: Color
  - fog_mode: FogMode (linear, exponential, exponential_squared)
  - fog_density: float
```

### PhysicsSettings Model
```
PhysicsSettings:
  - gravity: Vector3
  - default_material: Material
  - bounce_threshold: float
  - sleep_threshold: float
```

## Digital Twin Synchronization Models

### DigitalTwin Model
```
DigitalTwin:
  - physical_system_id: string
  - virtual_representation: GameObject
  - synchronization_mode: SyncMode (real_time, periodic, event_driven)
  - data_mapping: list<DataMapping>
  - update_frequency: float (Hz)
  - accuracy_threshold: float
```

### DataMapping Model
```
DataMapping:
  - source: DataSource (physical_sensor, simulation_output)
  - target: DataTarget (virtual_representation, visualization)
  - transformation: DataTransformation
  - data_type: DataType (position, orientation, sensor_data, etc.)
```

### DataTransformation Model
```
DataTransformation:
  - scale: Vector3
  - offset: Vector3
  - rotation: Quaternion
  - filter_type: FilterType (none, low_pass, high_pass, etc.)
  - filter_params: dict<string, any>
```

## Sensor Simulation Data Models

### LiDARData Model
```
LiDARData:
  - ranges: list<float> (distance measurements)
  - intensities: list<float> (intensity values)
  - resolution: float (angular resolution)
  - min_range: float
  - max_range: float
  - field_of_view: float (angular field of view)
  - frame_id: string
```

### DepthCameraData Model
```
DepthCameraData:
  - color_image: ImageData
  - depth_image: ImageData
  - camera_info: CameraInfo
  - point_cloud: PointCloud
```

### CameraInfo Model
```
CameraInfo:
  - width: int
  - height: int
  - distortion_model: string
  - distortion_coefficients: list<float>
  - intrinsic_matrix: list<float> (3x3)
  - projection_matrix: list<float> (3x4)
```

### IMUData Model
```
IMUData:
  - orientation: Quaternion
  - orientation_covariance: list<float> (9 elements)
  - angular_velocity: Vector3
  - angular_velocity_covariance: list<float> (9 elements)
  - linear_acceleration: Vector3
  - linear_acceleration_covariance: list<float> (9 elements)
  - frame_id: string
```

## Human-Robot Interaction Models

### InteractionEvent Model
```
InteractionEvent:
  - type: InteractionType (click, drag, voice_command, gesture)
  - position: Vector3 (world coordinates)
  - target: GameObject
  - timestamp: float
  - parameters: dict<string, any>
```

### HumanInput Model
```
HumanInput:
  - device_type: InputDeviceType (mouse, keyboard, touch, vr_controller)
  - input_type: InputType (position, rotation, button, axis)
  - value: any (input value)
  - timestamp: float
```

### RobotResponse Model
```
RobotResponse:
  - command: RobotCommand
  - execution_status: ExecutionStatus (pending, executing, completed, failed)
  - feedback: RobotFeedback
  - timestamp: float
```

### RobotCommand Model
```
RobotCommand:
  - command_type: CommandType (move_to, grasp, speak, etc.)
  - parameters: dict<string, any>
  - priority: int
  - timeout: float
```

## ROS 2 Integration Models

### ROSBridge Model
```
ROSBridge:
  - connection_type: ConnectionType (websocket, tcp, udp)
  - host: string
  - port: int
  - topics: list<TopicMapping>
  - services: list<ServiceMapping>
  - actions: list<ActionMapping>
```

### TopicMapping Model
```
TopicMapping:
  - ros_topic: string
  - unity_topic: string
  - message_type: string
  - queue_size: int
  - direction: TopicDirection (publish, subscribe, bidirectional)
```

## Simulation Synchronization Models

### SimulationState Model
```
SimulationState:
  - timestamp: float
  - simulation_time: float
  - real_time_factor: float
  - paused: boolean
  - entities: list<EntityState>
```

### EntityState Model
```
EntityState:
  - entity_id: string
  - position: Vector3
  - orientation: Quaternion
  - linear_velocity: Vector3
  - angular_velocity: Vector3
  - timestamp: float
```

## Example Digital Twin Configuration

```
ExampleDigitalTwin:
  - physical_system_id: "humanoid_robot_001"
  - virtual_representation:
      name: "HumanoidRobotDigitalTwin"
      transform: {position: [0, 0, 0], rotation: [0, 0, 0, 1], scale: [1, 1, 1]}
      components:
        - type: "MeshRenderer"
          properties: {mesh: "humanoid_model.fbx", material: "robot_material"}
        - type: "Rigidbody"
          properties: {mass: 50.0, useGravity: true}
  - synchronization_mode: "real_time"
  - update_frequency: 30.0  # Hz
  - data_mapping:
    - source: "physical_imu_data"
      target: "virtual_imu_visualization"
      transformation: {scale: [1, 1, 1], offset: [0, 0, 0], rotation: [0, 0, 0, 1]}
      data_type: "orientation"
    - source: "physical_lidar_data"
      target: "virtual_lidar_visualization"
      transformation: {scale: [1, 1, 1], offset: [0, 0, 0], rotation: [0, 0, 0, 1]}
      data_type: "range_data"
```