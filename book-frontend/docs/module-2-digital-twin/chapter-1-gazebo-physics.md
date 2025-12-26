---
sidebar_position: 6
title: "Chapter 1 - Gazebo Physics: Fundamentals of Simulation"
---

# Chapter 1: Gazebo Physics - Fundamentals of Simulation

This chapter introduces the physics simulation capabilities of Gazebo, including physics engines, gravity, collisions, and material properties.

## Physics Engines in Gazebo

Gazebo supports multiple physics engines, each with different characteristics:

- **ODE (Open Dynamics Engine)**: Default engine, good balance of performance and accuracy
- **Bullet**: Good for complex collision detection
- **DART**: Advanced dynamics and kinematics

### Configuring Physics Properties

Physics properties are typically set in the world file:

```xml
<physics name="default_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

### Key Physics Parameters

- **max_step_size**: Maximum time step for the simulation (smaller = more accurate but slower)
- **real_time_factor**: Target simulation speed relative to real time (1.0 = real-time)
- **real_time_update_rate**: Updates per second to maintain real-time performance

## Gravity and Material Properties

### Gravity Configuration

Gravity is defined as a 3D vector in meters/second²:

- Default: `0 0 -9.8` (Earth gravity)
- Can be modified per simulation needs
- Affects all objects with mass

```xml
<gravity>0 0 -9.8</gravity>
```

### Material Properties

Materials define surface characteristics that affect interactions:

- **Friction**: Static and dynamic friction coefficients
- **Restitution**: Bounciness (0 = no bounce, 1 = perfectly elastic)
- **Damping**: Energy dissipation properties

```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>
      <mu2>1.0</mu2>
    </ode>
  </friction>
  <bounce>
    <restitution_coefficient>0.1</restitution_coefficient>
    <threshold>100000</threshold>
  </bounce>
  <contact>
    <ode>
      <soft_cfm>0</soft_cfm>
      <soft_erp>0.2</soft_erp>
      <kp>1e+13</kp>
      <kd>1</kd>
      <max_vel>0.01</max_vel>
      <min_depth>0</min_depth>
    </ode>
  </contact>
</surface>
```

## Collision Detection

Gazebo provides multiple collision detection algorithms:

- **Bullet**: Best for complex shapes
- **ODE**: Good for basic shapes
- **SimBody**: For biological simulations

### Collision Geometry Types

- **Box**: Rectangular prism
- **Sphere**: Perfect sphere
- **Cylinder**: Cylindrical shape
- **Mesh**: Complex custom shapes
- **Plane**: Infinite flat surface

```xml
<collision name="collision">
  <geometry>
    <box>
      <size>1 1 1</size>
    </box>
  </geometry>
</collision>
```

## Joint Physics

Joints connect different parts of a robot and can have physical properties:

```xml
<joint name="joint_name" type="revolute">
  <parent>parent_link</parent>
  <child>child_link</child>
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>100</effort>
      <velocity>1</velocity>
    </limit>
    <dynamics>
      <damping>1.0</damping>
      <friction>0.1</friction>
    </dynamics>
  </axis>
</joint>
```

## Practical Exercise: Physics Configuration

Create a simple world file that demonstrates different physics properties:

1. Create a world with different gravity settings
2. Add objects with different material properties
3. Observe how they interact differently

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physics_demo">
    <!-- Custom physics settings -->
    <physics name="custom_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <gravity>0 0 -5.0</gravity> <!-- Reduced gravity -->
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Object with high restitution (bouncy) -->
    <model name="bouncy_sphere">
      <pose>0 0 2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.2</radius>
            </sphere>
          </geometry>
          <surface>
            <bounce>
              <restitution_coefficient>0.9</restitution_coefficient>
            </bounce>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.2</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.008</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.008</iyy>
            <iyz>0</iyz>
            <izz>0.008</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Next Steps

In the next chapter, we'll explore environment modeling and how to create simulation worlds with various components.