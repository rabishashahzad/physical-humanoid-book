# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: Module 2 - The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-17
**Input**: Feature specification from `/specs/module-2-digital-twin/spec.md`

## Prerequisites

Before starting this module, ensure you have:

1. **Completed Module 1**: Understanding of ROS 2 basics, nodes, topics, services
2. **Gazebo**: Gazebo 11+ or Garden installed
3. **Unity**: Unity 2021.3 LTS or newer installed
4. **ROS 2**: Humble Hawksbill or compatible distribution installed
5. **Basic knowledge** of 3D modeling and simulation concepts

### Installation Check

Verify your Gazebo installation:
```bash
# Check Gazebo version
gz --version  # For Garden
# OR
gazebo --version  # For older versions

# Launch Gazebo to test
gz sim  # For Garden
# OR
gazebo  # For older versions
```

Verify your Unity installation:
```bash
# Unity should be available through Unity Hub or direct executable
# Check version through Unity Hub or Unity editor
```

## Getting Started with Gazebo Fundamentals

### 1. Create a Basic Gazebo World

Create a simple world file `basic_world.sdf`:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define physics properties -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Add a simple box -->
    <model name="box">
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
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
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
  </world>
</sdf>
```

### 2. Launch the World

```bash
# Launch with Garden
gz sim basic_world.sdf

# OR for older versions
gazebo basic_world.sdf
```

### 3. Experiment with Physics Properties

Try modifying the physics properties in your world file:
- Change `max_step_size` to see how it affects simulation accuracy
- Adjust `real_time_factor` to speed up or slow down simulation
- Modify gravity values to see different effects

## Sensor Simulation in Gazebo

### 1. Create a Robot with LiDAR

Create a robot model file `robot_with_lidar.sdf`:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="robot_with_lidar">
    <!-- Robot body -->
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient>
          <diffuse>0 0 1 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.2</iyy>
          <iyz>0</iyz>
          <izz>0.15</izz>
        </inertia>
      </inertial>
    </link>

    <!-- LiDAR sensor -->
    <link name="lidar_link">
      <pose>0 0 0.2 0 0 0</pose>
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
    </link>

    <!-- Joint to connect LiDAR to chassis -->
    <joint name="chassis_to_lidar" type="fixed">
      <parent>chassis</parent>
      <child>lidar_link</child>
    </joint>
  </model>
</sdf>
```

### 2. Create a World with the Robot

Create `robot_world.sdf`:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="robot_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Include the robot -->
    <include>
      <uri>model://robot_with_lidar</uri>
    </include>
  </world>
</sdf>
```

### 3. Launch and View Sensor Data

```bash
# Launch the world
gz sim robot_world.sdf

# OR for older versions
gazebo robot_world.sdf
```

## Setting up Unity Digital Twin

### 1. Create a New Unity Project

1. Open Unity Hub
2. Create a new 3D project named "DigitalTwin"
3. Select the Universal Render Pipeline (URP) template for better performance

### 2. Basic Robot Model Setup

1. Create a new GameObject (Right-click in Hierarchy → 3D Object → Capsule) for the body
2. Add child GameObjects for head, arms, and legs
3. Position and scale the parts to form a humanoid shape
4. Add materials to differentiate parts

### 3. Connect to ROS 2 (Optional Advanced)

To connect Unity to ROS 2, you can use the Unity Robotics Package:

1. In Unity, go to Window → Package Manager
2. Install "ROS TCP Connector" from the package manager
3. Add the ROSConnection component to your main camera or a dedicated GameObject

## Basic Unity Robot Controller Script

Create a C# script `RobotController.cs`:

```csharp
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Header("Robot Configuration")]
    public Transform body;
    public Transform head;
    public Transform leftArm;
    public Transform rightArm;
    public Transform leftLeg;
    public Transform rightLeg;

    [Header("Simulation Data")]
    public float headRotationX;
    public float headRotationY;
    public Vector3 position;

    void Update()
    {
        // Update robot pose based on simulation data
        if (body != null)
        {
            body.position = position;
            body.Rotate(headRotationX, headRotationY, 0);
        }
    }

    // Method to update from external data source
    public void UpdateFromSimulation(Vector3 pos, float headX, float headY)
    {
        position = pos;
        headRotationX = headX;
        headRotationY = headY;
    }
}
```

## Gazebo-Unity Synchronization

### 1. Basic Data Flow

The synchronization between Gazebo and Unity typically involves:

1. Gazebo publishes robot state data via ROS 2 topics
2. A bridge node forwards this data to Unity
3. Unity updates the digital twin representation in real-time

### 2. Example ROS 2 Bridge Setup

Create a simple bridge script in Python:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import socket
import json

class GazeboUnityBridge(Node):
    def __init__(self):
        super().__init__('gazebo_unity_bridge')

        # Connect to Unity
        self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.unity_socket.connect(('localhost', 5005))

        # Subscribe to robot pose
        self.subscription = self.create_subscription(
            Odometry,
            'robot/odom',
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        # Extract position and orientation
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation

        # Create message for Unity
        unity_msg = {
            'position': {'x': pos.x, 'y': pos.y, 'z': pos.z},
            'rotation': {'x': quat.x, 'y': quat.y, 'z': quat.z, 'w': quat.w}
        }

        # Send to Unity
        self.unity_socket.send(json.dumps(unity_msg).encode())

def main(args=None):
    rclpy.init(args=args)
    bridge = GazeboUnityBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

1. **Chapter 1**: Complete the Gazebo fundamentals tutorial with physics, gravity, and collisions
2. **Chapter 2**: Implement sensor simulation with LiDAR, depth cameras, and IMUs
3. **Chapter 3**: Create Unity digital twin with rendering and human-robot interaction

## Troubleshooting

### Common Issues

1. **Gazebo Performance**: Reduce world complexity or adjust physics parameters
2. **Unity Connection**: Ensure ports are open and firewall allows connections
3. **Model Loading**: Verify file paths and formats are correct
4. **Synchronization**: Check that both systems use the same coordinate systems

### Getting Help

- Check the Gazebo documentation: http://gazebosim.org/
- Check the Unity documentation: https://docs.unity3d.com/
- Use `gz topic -l` to list available topics in Gazebo
- Use `ros2 topic list` to see available ROS 2 topics