# Data Model: Module 2 Docusaurus Setup - Gazebo & Unity Simulation

**Feature**: Module 2 Docusaurus Setup
**Date**: 2025-12-17
**Input**: Feature specification from `/specs/module-2-docusaurus-setup/spec.md`

## Overview

This document defines the data structures and models used in the Docusaurus implementation for Module 2, including content organization, navigation structures, and content metadata for Gazebo & Unity simulation.

## Docusaurus Content Model

### Document
```
Document:
  - id: string (unique identifier)
  - title: string (page title)
  - description: string (page description)
  - slug: string (URL path)
  - sidebar_position: number (position in sidebar)
  - tags: array<string> (content tags)
  - frontMatter: FrontMatter (metadata)
  - content: string (Markdown content)
  - source: string (file path)
  - module: string (module identifier)
  - chapter: string (chapter identifier)
```

### FrontMatter
```
FrontMatter:
  - title: string (page title)
  - description: string (page description)
  - sidebar_label: string (label in sidebar)
  - sidebar_position: number (position in sidebar)
  - hide_table_of_contents: boolean (hide TOC)
  - keywords: array<string> (SEO keywords)
  - image: string (social card image)
  - module: string (module identifier)
  - chapter: string (chapter identifier)
```

## Module Structure Model

### Module
```
Module:
  - id: string (module identifier)
  - title: string (module title)
  - description: string (module description)
  - chapters: array<Chapter>
  - position: number (order in book)
  - sidebar_position: number (position in sidebar)
  - prerequisites: array<string> (required knowledge)
  - learning_objectives: array<string> (what students will learn)
```

### Chapter
```
Chapter:
  - id: string (chapter identifier)
  - title: string (chapter title)
  - description: string (chapter description)
  - content: string (Markdown content)
  - prerequisites: array<string> (required knowledge)
  - objectives: array<string> (learning objectives)
  - exercises: array<Exercise> (practice problems)
  - position: number (order in module)
  - sidebar_position: number (position in sidebar)
  - next_chapter: string (ID of next chapter)
  - previous_chapter: string (ID of previous chapter)
```

### Exercise
```
Exercise:
  - id: string (exercise identifier)
  - title: string (exercise title)
  - description: string (exercise description)
  - difficulty: string (easy/medium/hard)
  - type: string (coding, theory, practical)
  - solution: string (solution content)
  - hints: array<string> (helpful hints)
```

## Navigation Structure Model

### Sidebar
```
Sidebar:
  - label: string (sidebar name)
  - items: array<SidebarItem>
  - collapsible: boolean (can collapse)
  - collapsed: boolean (default collapsed state)
```

### SidebarItem
```
SidebarItem:
  - type: SidebarItemType (doc, link, category, html)
  - label: string (display text)
  - id: string (document ID, for type=doc)
  - href: string (URL, for type=link)
  - items: array<SidebarItem> (sub-items, for type=category)
  - className: string (CSS class)
  - customProps: object (additional properties)
```

### SidebarItemType
```
SidebarItemType:
  - doc: Reference to a document
  - link: External or internal link
  - category: Group of items
  - html: HTML content
```

## Simulation Content Models

### PhysicsConcept
```
PhysicsConcept:
  - name: string (concept name)
  - description: string (detailed explanation)
  - examples: array<CodeExample>
  - parameters: array<PhysicsParameter>
  - applications: array<string> (use cases)
```

### PhysicsParameter
```
PhysicsParameter:
  - name: string (parameter name)
  - type: string (data type)
  - default_value: any (default value)
  - min_value: any (minimum allowed value)
  - max_value: any (maximum allowed value)
  - description: string (parameter purpose)
```

### EnvironmentComponent
```
EnvironmentComponent:
  - name: string (component name)
  - type: EnvironmentComponentType (light, model, terrain, etc.)
  - properties: object (component-specific properties)
  - configuration: string (SDF/XML configuration)
  - visualization: string (how it appears in simulation)
```

### EnvironmentComponentType
```
EnvironmentComponentType:
  - light: Lighting source
  - model: 3D model object
  - terrain: Ground surface
  - atmosphere: Atmospheric effects
  - plugin: Custom simulation plugin
```

### SensorType
```
SensorType:
  - lidar: LiDAR/ray-based sensor
  - camera: RGB camera
  - depth_camera: Depth camera
  - stereo_camera: Stereo camera
  - imu: Inertial measurement unit
  - gps: Global positioning system
  - force_torque: Force/torque sensor
  - contact: Contact sensor
```

### SensorSpecification
```
SensorSpecification:
  - type: SensorType
  - name: string (sensor name)
  - topic: string (ROS topic name)
  - update_rate: float (updates per second)
  - range: RangeSpecification (detection range)
  - noise: NoiseModel (sensor noise characteristics)
  - visualization: boolean (show in visualization)
```

### RangeSpecification
```
RangeSpecification:
  - min: float (minimum range)
  - max: float (maximum range)
  - resolution: float (resolution of measurements)
```

### NoiseModel
```
NoiseModel:
  - type: NoiseType (gaussian, custom, etc.)
  - mean: float (mean of noise distribution)
  - std_dev: float (standard deviation)
  - bias: float (systematic bias)
```

## Code Example Model

### CodeExample
```
CodeExample:
  - title: string (example title)
  - language: string (programming language)
  - code: string (actual code content)
  - description: string (explanation of the code)
  - purpose: string (what the code demonstrates)
  - output: string (expected output or behavior)
```

## Module 2 Specific Structure

```
Module2Structure:
  - module: Module
    - id: "module-2-digital-twin"
    - title: "Module 2: The Digital Twin (Gazebo & Unity)"
    - description: "Physics-based simulation and environment modeling using Gazebo and Unity for digital twins of humanoid robots"
    - position: 2
    - sidebar_position: 5  # After module 1 which has 4 items (intro + 3 chapters)
    - prerequisites: ["Module 1: ROS 2 basics"]
    - learning_objectives: [
        "Understand Gazebo physics simulation fundamentals",
        "Create realistic simulation environments",
        "Implement sensor simulation in Gazebo",
        "Connect Gazebo with Unity for digital twin visualization"
      ]
    - chapters: [
        Chapter:
          - id: "chapter-1-gazebo-physics"
          - title: "Chapter 1 - Gazebo Physics: Fundamentals of Simulation"
          - description: "Learn about physics engines, gravity, collisions, and material properties in Gazebo"
          - position: 1
          - sidebar_position: 6
          - objectives: [
              "Understand physics engine options in Gazebo",
              "Configure gravity and material properties",
              "Set up collision detection and response"
            ]
        Chapter:
          - id: "chapter-2-environment-modeling"
          - title: "Chapter 2 - Environment Modeling: Creating Simulation Worlds"
          - description: "Learn to create and configure simulation environments with lighting, terrain, and objects"
          - position: 2
          - sidebar_position: 7
          - objectives: [
              "Create SDF world files",
              "Configure lighting and atmospheric effects",
              "Add terrain and static objects to simulations"
            ]
        Chapter:
          - id: "chapter-3-sensor-simulation"
          - title: "Chapter 3 - Sensor Simulation: LiDAR, Cameras, IMUs"
          - description: "Implement various sensor types in Gazebo and understand their simulation characteristics"
          - position: 3
          - sidebar_position: 8
          - objectives: [
              "Configure LiDAR sensors in Gazebo",
              "Set up camera and depth camera simulation",
              "Implement IMU and other sensor types"
            ]
      ]
```

## Docusaurus Configuration Model

### SidebarConfiguration
```
SidebarConfiguration:
  - tutorialSidebar: array<SidebarItem>
    - intro: "intro"
    - module_1_category: {
        type: "category",
        label: "Module 1: Robotic Nervous System (ROS 2)",
        items: [
          "module-1-robotic-nervous-system/index",
          "module-1-robotic-nervous-system/chapter-1-ros2-basics",
          "module-1-robotic-nervous-system/chapter-2-python-ros2",
          "module-1-robotic-nervous-system/chapter-3-urdf-modeling"
        ]
      }
    - module_2_category: {
        type: "category",
        label: "Module 2: The Digital Twin (Gazebo & Unity)",
        items: [
          "module-2-digital-twin/index",
          "module-2-digital-twin/chapter-1-gazebo-physics",
          "module-2-digital-twin/chapter-2-environment-modeling",
          "module-2-digital-twin/chapter-3-sensor-simulation"
        ]
      }
```