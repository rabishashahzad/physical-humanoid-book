---
sidebar_position: 4
title: "Chapter 3 - URDF Modeling: Links, joints, sensors, simple humanoid"
---

# Chapter 3: URDF Modeling - Links, joints, sensors, simple humanoid

This chapter covers URDF (Unified Robot Description Format) modeling for humanoid robots, including links, joints, sensors, and creating a simple humanoid model.

## What is URDF?

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It can contain:

- Kinematic and dynamic description of each link
- Visual and collision properties
- Physical properties (inertial parameters)
- Joint definitions between links
- Sensor definitions
- Transmission information for actuators

URDF is primarily used in ROS for:
- Robot visualization (RViz, Gazebo)
- Kinematics computation
- Robot state publishing
- Collision detection

## Basic URDF Structure

A basic URDF file looks like this:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.1 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>
</robot>
```

## Link Elements

Each `<link>` element represents a rigid body and can contain:

### Visual Element
Defines how the link looks in visualization:
- `<geometry>`: Shape definition (box, cylinder, sphere, mesh)
- `<material>`: Color and texture properties
- `<origin>`: Position and orientation relative to joint

### Collision Element
Defines collision properties for physics simulation:
- `<geometry>`: Shape definition (similar to visual)
- `<origin>`: Position and orientation relative to joint

### Inertial Element
Defines mass and inertial properties:
- `<mass>`: Mass value
- `<inertia>`: Inertia tensor values (ixx, ixy, ixz, iyy, iyz, izz)

### Material Element
Defines color and appearance:
- `<color>`: RGBA values (red, green, blue, alpha)
- `<texture>`: Texture file reference

## Joint Types

### Fixed Joint
No movement between links:
```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>
```

### Revolute Joint
Rotational joint with limits:
```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

### Continuous Joint
Rotational joint without limits:
```xml
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="0 0 1"/>
</joint>
```

### Prismatic Joint
Linear sliding joint:
```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="100" velocity="1"/>
</joint>
```

## Creating a Simple Humanoid Model

Here's a complete example of a simple humanoid model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.03"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.025"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.03"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.025"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint Definitions -->
  <!-- Torso to base -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.05"/>
  </joint>

  <!-- Head to torso -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm Joints -->
  <joint name="torso_to_left_upper_arm" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 1.57"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <joint name="left_upper_arm_to_lower_arm" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Arm Joints -->
  <joint name="torso_to_right_upper_arm" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.1 0 0.1" rpy="0 0 -1.57"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <joint name="right_upper_arm_to_lower_arm" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Left Leg Joints -->
  <joint name="torso_to_left_upper_leg" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.05 0 -0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.3" effort="100" velocity="1"/>
  </joint>

  <joint name="left_upper_leg_to_lower_leg" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <joint name="left_lower_leg_to_foot" type="fixed">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.15"/>
  </joint>

  <!-- Right Leg Joints -->
  <joint name="torso_to_right_upper_leg" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.05 0 -0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.3" effort="100" velocity="1"/>
  </joint>

  <joint name="right_upper_leg_to_lower_leg" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <joint name="right_lower_leg_to_foot" type="fixed">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.15"/>
  </joint>
</robot>
```

## Working with URDF in ROS 2

### Loading URDF in ROS 2

To use your URDF model in ROS 2, you typically:

1. Store the URDF file in your package
2. Use the `robot_state_publisher` node to publish the robot description
3. Use RViz to visualize the robot

### Example launch file for visualization:

```python
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('your_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_humanoid.urdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
```

## Xacro for Complex Models

For more complex models, you can use Xacro (XML Macros), which allows you to:

- Define properties and reuse them
- Create macros for repeated elements
- Include other Xacro files
- Use mathematical expressions

Example Xacro snippet:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_radius" value="0.05" />
  <xacro:property name="base_length" value="0.1" />

  <!-- Macro for creating arm links -->
  <xacro:macro name="arm_link" params="name parent xyz">
    <joint name="${parent}_to_${name}" type="revolute">
      <parent link="${parent}"/>
      <child link="${name}"/>
      <origin xyz="${xyz}"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>

    <link name="${name}">
      <visual>
        <geometry>
          <cylinder length="0.2" radius="0.03"/>
        </geometry>
        <material name="blue">
          <color rgba="0 0 1 1"/>
        </material>
      </visual>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:arm_link name="left_upper_arm" parent="torso" xyz="0.1 0 0.1"/>

</robot>
```

## Summary

In this chapter, you've learned about:
- URDF structure and elements (links, joints, materials)
- Different joint types and their properties
- How to create a complete humanoid model
- Working with URDF in ROS 2
- Using Xacro for more complex models

URDF modeling is essential for creating robot descriptions that can be used with ROS 2 tools for visualization, simulation, and control.

## Next Steps

Congratulations! You've completed Module 1: Robotic Nervous System (ROS 2). You now understand:
- ROS 2 basics: nodes, topics, services
- Python integration with rclpy
- URDF modeling for humanoid robots

These concepts form the foundation for developing robotic applications with ROS 2.