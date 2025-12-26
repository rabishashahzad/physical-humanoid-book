---
sidebar_position: 7
title: "Chapter 2 - Environment Modeling: Creating Simulation Worlds"
---

# Chapter 2: Environment Modeling - Creating Simulation Worlds

This chapter covers how to create and configure simulation environments with lighting, terrain, and objects.

## World Files Structure

Gazebo worlds are defined using SDF (Simulation Description Format). The basic structure includes:

```xml
<sdf version="1.7">
  <world name="my_world">
    <!-- Physics configuration -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Custom models and objects -->
    <!-- More content here -->
  </world>
</sdf>
```

## Adding Components to Worlds

### Lighting

Gazebo supports different types of lights to illuminate your simulation:

#### Directional Light (Sun)
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

#### Point Light
```xml
<light name="point_light" type="point">
  <pose>2 2 3 0 0 0</pose>
  <diffuse>0.5 0.5 1 1</diffuse>
  <specular>0.5 0.5 1 1</specular>
  <attenuation>
    <range>10</range>
    <constant>0.2</constant>
    <linear>0.03</linear>
    <quadratic>0.05</quadratic>
  </attenuation>
</light>
```

### Static Models

Include pre-built models from Gazebo's model database:

```xml
<include>
  <uri>model://ground_plane</uri>
</include>

<include>
  <uri>model://sun</uri>
</include>

<include>
  <uri>model://cylinder</uri>
  <pose>1 1 0.5 0 0 0</pose>
</include>
```

Or include custom models:

```xml
<include>
  <uri>model://my_custom_model</uri>
  <pose>1 1 0 0 0 0</pose>
</include>
```

## Terrain and Elevation Maps

Gazebo supports heightmap-based terrains for creating realistic outdoor environments:

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
<model name="simple_house">
  <pose>5 5 0 0 0 0</pose>

  <!-- House base -->
  <link name="base">
    <collision name="collision">
      <geometry>
        <box>
          <size>4 4 3</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>4 4 3</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.6 0.4 1</ambient>
        <diffuse>0.8 0.6 0.4 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>100.0</mass>
      <inertia>
        <ixx>200</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>200</iyy>
        <iyz>0</iyz>
        <izz>200</izz>
      </inertia>
    </inertial>
  </link>

  <!-- House roof -->
  <link name="roof">
    <collision name="collision">
      <geometry>
        <mesh>
          <uri>file://meshes/roof.dae</uri>
        </mesh>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>file://meshes/roof.dae</uri>
        </mesh>
      </geometry>
    </visual>
    <inertial>
      <mass>50.0</mass>
      <inertia>
        <ixx>100</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>100</iyy>
        <iyz>0</iyz>
        <izz>100</izz>
      </inertia>
    </inertial>
  </link>

  <!-- Connect roof to base -->
  <joint name="base_to_roof" type="fixed">
    <parent>base</parent>
    <child>roof</child>
    <pose>0 0 1.5 0 0 0</pose>
  </joint>
</model>
```

## Atmosphere and Weather Effects

Gazebo supports atmospheric effects to make environments more realistic:

```xml
<atmosphere type="adiabatic">
  <temperature>288.15</temperature>
  <pressure>101325</pressure>
  <temperature_gradient>-0.0065</temperature_gradient>
</atmosphere>
```

## Practical Exercise: Creating an Indoor Environment

Create a simple indoor environment with walls, furniture, and lighting:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="indoor_environment">
    <!-- Physics -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Simple room with walls -->
    <!-- Back wall -->
    <model name="back_wall">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>0 5 1.5 0 0 0</pose>
    </model>

    <!-- Side walls -->
    <model name="left_wall">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 10 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 10 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>-5 0 1.5 0 0 0</pose>
    </model>

    <!-- Floor -->
    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 10 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 10 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>0 0 -0.1 0 0 0</pose>
    </model>

    <!-- Ceiling -->
    <model name="ceiling">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 10 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 10 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>0 0 3 0 0 0</pose>
    </model>

    <!-- Indoor lighting -->
    <light name="ceiling_light" type="point">
      <pose>0 0 2.8 0 0 0</pose>
      <diffuse>0.8 0.8 0.7 1</diffuse>
      <specular>0.8 0.8 0.7 1</specular>
      <attenuation>
        <range>15</range>
        <constant>0.05</constant>
        <linear>0.02</linear>
        <quadratic>0.01</quadratic>
      </attenuation>
    </light>

    <!-- Simple table -->
    <model name="table">
      <link name="top">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 0.8 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 0.8 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.5</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>3.4</iyy>
            <iyz>0</iyz>
            <izz>4.5</izz>
          </inertia>
        </inertial>
      </link>
      <link name="leg1">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.7</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.7</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.01</iyy>
            <iyz>0</iyz>
            <izz>0.0001</izz>
          </inertia>
        </inertial>
      </link>
      <!-- Additional legs would be defined similarly -->
      <joint name="top_to_leg1" type="fixed">
        <parent>top</parent>
        <child>leg1</child>
        <pose>-0.7 0.35 -0.35 0 0 0</pose>
      </joint>
    </model>
  </world>
</sdf>
```

## Best Practices for Environment Modeling

1. **Performance**: Keep geometry simple where possible to maintain simulation speed
2. **Realism**: Use appropriate textures and materials for visual quality
3. **Scale**: Ensure objects are properly scaled relative to each other
4. **Lighting**: Balance lighting to ensure good visibility without overexposure
5. **Collision**: Use simple collision geometry for better performance

## Next Steps

In the next chapter, we'll explore sensor simulation and how to implement various sensor types in Gazebo.