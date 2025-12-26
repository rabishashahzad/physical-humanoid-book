---
sidebar_position: 8
title: "Chapter 3 - Sensor Simulation: LiDAR, Cameras, IMUs"
---

# Chapter 3: Sensor Simulation - LiDAR, Cameras, IMUs

This chapter covers implementing various sensor types in Gazebo simulation, including LiDAR, cameras, and IMUs, and connecting them to ROS 2.

## LiDAR Simulation

LiDAR sensors are implemented using ray-based simulation in Gazebo. They emit rays in a pattern and measure the distance to objects that reflect back.

### Basic LiDAR Configuration

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

### LiDAR Parameters Explained

- **samples**: Number of rays in the horizontal plane
- **resolution**: Number of rays per radian (or per unit angle)
- **min_angle/max_angle**: Angular range of the sensor (in radians)
- **range**: Detection range (min, max, resolution in meters)
- **update_rate**: How often the sensor updates (Hz)
- **visualize**: Whether to show the sensor rays in visualization

### 3D LiDAR (HDL-64E Example)

For 3D LiDAR sensors like the popular Velodyne models:

```xml
<sensor name="velodyne_VLP_16" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>512</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>  <!-- -15 degrees -->
        <max_angle>0.261799</max_angle>   <!-- 15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.2</min>
      <max>100</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
</sensor>
```

## Camera Simulation

### RGB Camera

RGB cameras simulate standard visual sensors that capture color images:

```xml
<sensor name="camera" type="camera">
  <camera name="rgb_camera">
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees in radians -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Depth Camera

Depth cameras provide both RGB and depth information:

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
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Stereo Camera

Stereo cameras simulate two cameras for depth perception:

```xml
<sensor name="stereo_camera" type="multicamera">
  <camera name="left">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>300</far>
    </clip>
  </camera>
  <camera name="right">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>300</far>
    </clip>
    <pose>0.2 0 0 0 0 0</pose>  <!-- 20cm baseline -->
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

## IMU Simulation

IMU (Inertial Measurement Unit) sensors provide orientation, angular velocity, and linear acceleration:

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
          <bias_mean>0.00001</bias_mean>
          <bias_stddev>0.000001</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
          <bias_mean>0.00001</bias_mean>
          <bias_stddev>0.000001</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
          <bias_mean>0.00001</bias_mean>
          <bias_stddev>0.000001</bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.01</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.01</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.01</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Other Sensor Types

### GPS Sensor

```xml
<sensor name="gps_sensor" type="gps">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <gps>
    <position_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>
      </vertical>
    </position_sensing>
  </gps>
</sensor>
```

### Force/Torque Sensor

```xml
<sensor name="force_torque" type="force_torque">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
</sensor>
```

## ROS 2 Integration

Sensors in Gazebo can publish directly to ROS 2 topics using plugins:

### Camera with ROS 2 Plugin

```xml
<sensor name="camera" type="camera">
  <camera name="rgb_camera">
    <!-- camera configuration -->
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_frame</frame_name>
    <topic_name>image_raw</topic_name>
    <hack_baseline>0.07</hack_baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
  </plugin>
</sensor>
```

### IMU with ROS 2 Plugin

```xml
<sensor name="imu_sensor" type="imu">
  <!-- IMU configuration -->
  <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
    <frame_name>imu_frame</frame_name>
    <topic_name>imu/data</topic_name>
    <serviceName>imu/service</serviceName>
  </plugin>
</sensor>
```

### LiDAR with ROS 2 Plugin

```xml
<sensor name="lidar" type="ray">
  <!-- LiDAR configuration -->
  <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
    <frame_name>laser_frame</frame_name>
    <topic_name>scan</topic_name>
    <min_range>0.1</min_range>
    <max_range>30.0</max_range>
    <gaussian_noise>0.01</gaussian_noise>
  </plugin>
</sensor>
```

## Sensor Fusion Concepts

In robotics, multiple sensors are often combined to provide more robust perception:

- **LiDAR + Camera**: Combines precise distance measurements with visual information
- **IMU + GPS**: Combines high-frequency IMU data with absolute position from GPS
- **Multiple cameras**: Stereo vision for depth perception

## Practical Exercise: Complete Sensor Setup

Create a robot model with multiple sensors:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="sensor_robot">
    <link name="chassis">
      <pose>0 0 0.3 0 0 0</pose>
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

    <!-- LiDAR on top of chassis -->
    <link name="lidar_link">
      <pose>0 0 0.5 0 0 0</pose>
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

    <!-- Camera -->
    <link name="camera_link">
      <pose>0.2 0 0.3 0 0 0</pose>
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
    </link>

    <!-- IMU -->
    <link name="imu_link">
      <pose>0 0 0.2 0 0 0</pose>
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
      </sensor>
    </link>

    <!-- Connect sensors to chassis -->
    <joint name="chassis_to_lidar" type="fixed">
      <parent>chassis</parent>
      <child>lidar_link</child>
    </joint>

    <joint name="chassis_to_camera" type="fixed">
      <parent>chassis</parent>
      <child>camera_link</child>
    </joint>

    <joint name="chassis_to_imu" type="fixed">
      <parent>chassis</parent>
      <child>imu_link</child>
    </joint>
  </model>
</sdf>
```

## Unity Integration Considerations

When creating digital twins with Unity:

1. **Data Mapping**: Map Gazebo sensor data to Unity visualization
2. **Performance**: Consider the computational cost of processing sensor data in real-time
3. **Synchronization**: Ensure Unity visualization matches Gazebo simulation state
4. **Visualization**: Create appropriate visual representations for different sensor types

## Next Steps

Congratulations! You've completed Module 2 on Gazebo & Unity simulation. You now understand physics simulation, environment modeling, and sensor simulation for creating digital twins of humanoid robots. In the next module, we'll explore advanced topics in robotics and simulation.