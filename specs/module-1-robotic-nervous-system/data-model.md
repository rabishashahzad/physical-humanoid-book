# Data Model: Module 1 - Robotic Nervous System (ROS 2)

**Feature**: Module 1 - Robotic Nervous System (ROS 2)
**Date**: 2025-12-17
**Input**: Feature specification from `/specs/module-1-robotic-nervous-system/spec.md`

## Overview

This document defines the data structures and models used in Module 1: Robotic Nervous System (ROS 2). It covers the conceptual data models for ROS 2 concepts, Python integration patterns, and URDF structures for humanoid modeling.

## ROS 2 Core Data Models

### Node Data Model
```
Node:
  - node_name: string
  - namespace: string (optional)
  - parameters: dict<string, Parameter>
  - publishers: list<Publisher>
  - subscribers: list<Subscriber>
  - services: list<Service>
  - clients: list<Client>
```

### Message Data Model
```
Message:
  - header: Header
  - data: any (type-specific content)
  - timestamp: Time
  - frame_id: string (for coordinate frames)
```

### Topic Data Model
```
Topic:
  - name: string
  - type: string (e.g., "std_msgs/String")
  - publishers: list<Node>
  - subscribers: list<Node>
  - qos_profile: QoSProfile
```

### Service Data Model
```
Service:
  - name: string
  - type: string (e.g., "std_srvs/SetBool")
  - node: Node
  - requests: list<Request>
  - responses: list<Response>
```

## Python Integration Data Models (rclpy)

### Publisher Model
```
Publisher:
  - node: Node
  - topic: Topic
  - message_type: MessageType
  - qos_profile: QoSProfile
  - queue_size: int
```

### Subscriber Model
```
Subscriber:
  - node: Node
  - topic: Topic
  - message_type: MessageType
  - callback: function
  - qos_profile: QoSProfile
```

### Client Model
```
Client:
  - node: Node
  - service: Service
  - service_type: ServiceType
  - callback_group: CallbackGroup
```

### Service Server Model
```
ServiceServer:
  - node: Node
  - service: Service
  - service_type: ServiceType
  - callback: function
  - callback_group: CallbackGroup
```

## URDF Data Models

### Link Model
```
Link:
  - name: string
  - visual: Visual (optional)
  - collision: Collision (optional)
  - inertial: Inertial (optional)
  - origin: Pose (optional)
```

### Joint Model
```
Joint:
  - name: string
  - type: JointType (revolute, continuous, prismatic, fixed, etc.)
  - parent: string (link name)
  - child: string (link name)
  - origin: Pose (optional)
  - axis: Vector3 (optional)
  - limit: JointLimit (optional, for revolute/prismatic)
```

### JointLimit Model
```
JointLimit:
  - lower: float (radians for revolute, meters for prismatic)
  - upper: float (radians for revolute, meters for prismatic)
  - effort: float (N for prismatic, N-m for revolute)
  - velocity: float (m/s for prismatic, rad/s for revolute)
```

### Visual Model
```
Visual:
  - geometry: Geometry
  - material: Material (optional)
  - origin: Pose (optional)
```

### Collision Model
```
Collision:
  - geometry: Geometry
  - origin: Pose (optional)
```

### Geometry Model
```
Geometry:
  - type: GeometryType (box, cylinder, sphere, mesh)
  - dimensions: Vector3 (for box, cylinder, sphere)
  - filename: string (for mesh)
  - scale: Vector3 (for mesh)
```

### Material Model
```
Material:
  - name: string
  - color: Color (optional)
  - texture: string (optional, filename)
```

### Color Model
```
Color:
  - r: float (0.0-1.0)
  - g: float (0.0-1.0)
  - b: float (0.0-1.0)
  - a: float (0.0-1.0)
```

### Pose Model
```
Pose:
  - position: Vector3
  - orientation: Quaternion
```

### Vector3 Model
```
Vector3:
  - x: float
  - y: float
  - z: float
```

### Quaternion Model
```
Quaternion:
  - x: float
  - y: float
  - z: float
  - w: float
```

## Humanoid Robot Data Model

### HumanoidModel
```
HumanoidModel:
  - name: string
  - version: string
  - links: list<Link>
  - joints: list<Joint>
  - sensors: list<Sensor>
  - materials: list<Material>
  - gazebo_extensions: list<GazeboExtension> (optional)
```

### Sensor Model
```
Sensor:
  - name: string
  - type: SensorType (camera, imu, lidar, force_torque, etc.)
  - parent_link: string
  - origin: Pose
  - parameters: dict<string, any>
```

## Example Data Structure for Simple Humanoid

```
SimpleHumanoid:
  - name: "simple_humanoid"
  - links:
    - {name: "base_link", visual: {geometry: {type: "cylinder", dimensions: [0.1, 0.2]}, material: {name: "blue", color: {r: 0.0, g: 0.0, b: 1.0, a: 1.0}}}}
    - {name: "torso", visual: {geometry: {type: "box", dimensions: [0.3, 0.1, 0.5]}, material: {name: "red", color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}}}}
    - {name: "head", visual: {geometry: {type: "sphere", dimensions: [0.15, 0.15, 0.15]}, material: {name: "white", color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}}}}
  - joints:
    - {name: "base_to_torso", type: "fixed", parent: "base_link", child: "torso", origin: {position: [0, 0, 0.1]}}
    - {name: "torso_to_head", type: "revolute", parent: "torso", child: "head", origin: {position: [0, 0, 0.3]}, axis: [0, 1, 0], limit: {lower: -1.57, upper: 1.57, effort: 100, velocity: 1}}
```

## ROS 2 Message Types for Humanoid Control

### JointState Message
```
JointState:
  - name: list<string> (joint names)
  - position: list<float> (joint positions in radians/meters)
  - velocity: list<float> (joint velocities)
  - effort: list<float> (joint efforts)
```

### Twist Message (for movement)
```
Twist:
  - linear: Vector3 (linear velocity)
  - angular: Vector3 (angular velocity)
```

### Humanoid Command Message (Custom)
```
HumanoidCommand:
  - joint_commands: list<JointCommand>
  - base_velocity: Twist (optional)
  - walk_command: WalkCommand (optional)
```

### JointCommand
```
JointCommand:
  - joint_name: string
  - position: float
  - velocity: float
  - effort: float
```

### WalkCommand
```
WalkCommand:
  - step_size: float
  - step_height: float
  - direction: Vector2 (x, y)
  - speed: float
```