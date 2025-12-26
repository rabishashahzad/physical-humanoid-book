# Quickstart Guide: Module 2 in Docusaurus - Gazebo & Unity Simulation

**Feature**: Module 2 Docusaurus Setup
**Date**: 2025-12-17
**Input**: Feature specification from `/specs/module-2-docusaurus-setup/spec.md`

## Prerequisites

Before setting up Module 2 content in Docusaurus, ensure you have:

1. **Access to the Docusaurus project** in the `book-frontend` directory
2. **Basic knowledge** of Markdown and Docusaurus frontmatter
3. **Module 1 content** already set up (as Module 2 builds on ROS 2 basics)

### Verification

Verify your Docusaurus project is set up correctly:
```bash
cd book-frontend
npm run start
```

You should see the existing content including Module 1.

## Setting up Module 2 Directory Structure

### 1. Create Module 2 Directory

```bash
# Navigate to the docs directory
cd book-frontend/docs

# Create the module 2 directory
mkdir -p module-2-digital-twin
```

### 2. Create Module 2 Content Files

Create the following files in `book-frontend/docs/module-2-digital-twin/`:

#### Module 2 Overview Page
```bash
touch book-frontend/docs/module-2-digital-twin/index.md
```

#### Chapter Files
```bash
touch book-frontend/docs/module-2-digital-twin/chapter-1-gazebo-physics.md
touch book-frontend/docs/module-2-digital-twin/chapter-2-environment-modeling.md
touch book-frontend/docs/module-2-digital-twin/chapter-3-sensor-simulation.md
```

## Creating Module 2 Content

### 1. Module 2 Overview Page

Create `book-frontend/docs/module-2-digital-twin/index.md`:

```markdown
---
sidebar_position: 5
title: Module 2 - The Digital Twin (Gazebo & Unity)
---

# Module 2: The Digital Twin (Gazebo & Unity)

In this module, you'll learn about physics-based simulation and environment modeling using Gazebo and Unity for digital twins of humanoid robots.

## About This Module

This module builds on your ROS 2 basics knowledge to teach you:
- Gazebo physics simulation fundamentals
- Environment modeling and configuration
- Sensor simulation in Gazebo and Unity

## Prerequisites

- Completion of Module 1: Robotic Nervous System (ROS 2 basics)
- Understanding of robotics concepts

## Chapters

1. [Chapter 1 - Gazebo Physics: Fundamentals of Simulation](./chapter-1-gazebo-physics.md)
2. [Chapter 2 - Environment Modeling: Creating Simulation Worlds](./chapter-2-environment-modeling.md)
3. [Chapter 3 - Sensor Simulation: LiDAR, Cameras, IMUs](./chapter-3-sensor-simulation.md)

## Learning Objectives

By the end of this module, you will be able to:
- Configure physics properties in Gazebo simulations
- Create and customize simulation environments
- Implement various sensor types in simulation
- Connect Gazebo with Unity for digital twin visualization
```

### 2. Chapter 1: Gazebo Physics

Create `book-frontend/docs/module-2-digital-twin/chapter-1-gazebo-physics.md`:

```markdown
---
sidebar_position: 6
title: Chapter 1 - Gazebo Physics: Fundamentals of Simulation
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

## Gravity and Material Properties

### Gravity Configuration

Gravity is defined as a 3D vector in meters/second²:

- Default: `0 0 -9.8` (Earth gravity)
- Can be modified per simulation needs
- Affects all objects with mass

### Material Properties

Materials define surface characteristics:

- **Friction**: Static and dynamic friction coefficients
- **Restitution**: Bounciness (0 = no bounce, 1 = perfectly elastic)
- **Damping**: Energy dissipation properties

## Collision Detection

Gazebo provides multiple collision detection algorithms:

- **Bullet**: Best for complex shapes
- **ODE**: Good for basic shapes
- **SimBody**: For biological simulations

## Next Steps

In the next chapter, we'll explore environment modeling and how to create simulation worlds with various components.
```

### 3. Chapter 2: Environment Modeling

Create `book-frontend/docs/module-2-digital-twin/chapter-2-environment-modeling.md`:

```markdown
---
sidebar_position: 7
title: Chapter 2 - Environment Modeling: Creating Simulation Worlds
---

# Chapter 2: Environment Modeling - Creating Simulation Worlds

This chapter covers how to create and configure simulation environments with lighting, terrain, and objects.

## World Files Structure

Gazebo worlds are defined using SDF (Simulation Description Format):

```xml
<sdf version="1.7">
  <world name="my_world">
    <!-- World content goes here -->
  </world>
</sdf>
```

## Adding Components to Worlds

### Lighting

Add different types of lights to your world:

```xml
<light name="sun" type="directional">
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.3 0.3 -0.9</direction>
</light>
```

### Static Models

Include ground planes, buildings, or other static objects:

```xml
<include>
  <uri>model://ground_plane</uri>
</include>

<include>
  <uri>model://my_custom_model</uri>
  <pose>1 1 0 0 0 0</pose>
</include>
```

## Terrain and Elevation Maps

Gazebo supports heightmap-based terrains:

```xml
<model name="uneven_terrain">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>file://path/to/heightmap.png</uri>
          <size>100 100 20</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <heightmap>
          <uri>file://path/to/texture.png</uri>
          <size>100 100 20</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
```

## Custom Models

You can create custom models using SDF files:

```xml
<model name="simple_box">
  <pose>0 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </visual>
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.166667</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.166667</iyy>
        <iyz>0</iyz>
        <izz>0.166667</izz>
      </inertia>
    </inertial>
  </link>
</model>
```

## Next Steps

In the next chapter, we'll explore sensor simulation and how to implement various sensor types in Gazebo.
```

### 4. Chapter 3: Sensor Simulation

Create `book-frontend/docs/module-2-digital-twin/chapter-3-sensor-simulation.md`:

```markdown
---
sidebar_position: 8
title: Chapter 3 - Sensor Simulation: LiDAR, Cameras, IMUs
---

# Chapter 3: Sensor Simulation - LiDAR, Cameras, IMUs

This chapter covers implementing various sensor types in Gazebo simulation, including LiDAR, cameras, and IMUs.

## LiDAR Simulation

LiDAR sensors are implemented using ray-based simulation:

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
</sensor>
```

### LiDAR Parameters

- **Samples**: Number of rays in the horizontal plane
- **Resolution**: Number of rays per radian
- **Min/Max angle**: Angular range of the sensor
- **Range**: Detection range (min, max, resolution)

## Camera Simulation

### RGB Camera

```xml
<sensor name="camera" type="camera">
  <camera name="rgb_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Depth Camera

```xml
<sensor name="depth_camera" type="depth">
  <camera name="depth_cam">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

## IMU Simulation

IMU sensors provide orientation, angular velocity, and linear acceleration:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Sensor Integration with ROS 2

Sensors in Gazebo can publish directly to ROS 2 topics:

```xml
<sensor name="camera" type="camera">
  <!-- camera configuration -->
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_frame</frame_name>
    <topic_name>image_raw</topic_name>
  </plugin>
</sensor>
```

## Unity Integration

Unity can receive sensor data from Gazebo through ROS 2 bridge or custom protocols:

1. Use ROS TCP Connector to receive sensor data
2. Process data in Unity scripts
3. Visualize sensor information in Unity environment

## Next Steps

Congratulations! You've completed Module 2 on Gazebo & Unity simulation. You now understand physics simulation, environment modeling, and sensor simulation for creating digital twins of humanoid robots.
```

## Updating Navigation

### 1. Update the Sidebar Configuration

Edit `book-frontend/sidebars.js` to add Module 2 to the navigation:

```javascript
tutorialSidebar: [
  'intro',
  {
    type: 'category',
    label: 'Module 1: Robotic Nervous System (ROS 2)',
    items: [
      'module-1-robotic-nervous-system/index',
      'module-1-robotic-nervous-system/chapter-1-ros2-basics',
      'module-1-robotic-nervous-system/chapter-2-python-ros2',
      'module-1-robotic-nervous-system/chapter-3-urdf-modeling',
    ],
  },
  {
    type: 'category',
    label: 'Module 2: The Digital Twin (Gazebo & Unity)',
    items: [
      'module-2-digital-twin/index',
      'module-2-digital-twin/chapter-1-gazebo-physics',
      'module-2-digital-twin/chapter-2-environment-modeling',
      'module-2-digital-twin/chapter-3-sensor-simulation',
    ],
  },
  // Add more modules here as they are created
],
```

### 2. Test the Setup

Start the Docusaurus development server to verify the setup:

```bash
cd book-frontend
npm run start
```

Navigate to your new Module 2 content to ensure it displays correctly with proper navigation.

## Troubleshooting

### Common Issues

1. **Navigation not showing**: Verify sidebar configuration and restart the development server
2. **Incorrect positioning**: Check sidebar_position values in frontmatter
3. **Broken links**: Verify relative paths in navigation and content links
4. **Frontmatter errors**: Ensure proper YAML formatting in the frontmatter sections

### Getting Help

- Check the Docusaurus documentation: https://docusaurus.io/docs
- Verify your configuration in `docusaurus.config.js`
- Use `npm run build` to check for build errors