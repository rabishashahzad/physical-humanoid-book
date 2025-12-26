# Quickstart Guide: Docusaurus Book Site with Module 1

**Feature**: Docusaurus Book Site
**Date**: 2025-12-17
**Input**: Feature specification from `/specs/docusaurus-book-site/spec.md`

## Prerequisites

Before starting this project, ensure you have:

1. **Node.js** version 18.0 or higher installed
2. **npm** or **yarn** package manager
3. **Git** for version control
4. **Basic knowledge** of Markdown and JavaScript

### Installation Check

Verify your Node.js installation:
```bash
node --version
npm --version
```

## Setting up Docusaurus

### 1. Create a New Docusaurus Project

```bash
# Create a new directory for the project
mkdir docusaurus-book
cd docusaurus-book

# Initialize the Docusaurus project
npx create-docusaurus@latest website-classic

# Select the following options when prompted:
# - Website name: docusaurus-book
# - Template: classic
# - Language: TypeScript (or JavaScript)
```

### 2. Project Structure Overview

After initialization, your project structure should look like:

```
docusaurus-book/
├── blog/
│   ├── 2021-08-26-welcome/
│   └── 2019-05-28-first-blog-post.md
├── docs/
│   ├── tutorial-basics/
│   │   ├── deploy-your-site.md
│   │   └── ...
│   └── intro.md
├── src/
│   ├── components/
│   ├── css/
│   └── pages/
├── static/
│   └── img/
├── docusaurus.config.js
├── package.json
├── sidebars.js
└── README.md
```

### 3. Install Additional Dependencies

```bash
# Navigate to the project directory
cd docusaurus-book

# Install any additional dependencies if needed
npm install
```

### 4. Start the Development Server

```bash
# Start the development server
npm run start

# This will start a local development server at http://localhost:3000
```

## Creating Module 1 Content

### 1. Create Module Directory Structure

```bash
# Create directory for Module 1
mkdir -p docs/module-1-robotic-nervous-system

# Create the three chapter files
touch docs/module-1-robotic-nervous-system/index.md
touch docs/module-1-robotic-nervous-system/chapter-1-ros2-basics.md
touch docs/module-1-robotic-nervous-system/chapter-2-python-ros2.md
touch docs/module-1-robotic-nervous-system/chapter-3-urdf-modeling.md
```

### 2. Create Module 1 Overview Page

Create `docs/module-1-robotic-nervous-system/index.md`:

```markdown
---
sidebar_position: 1
title: Module 1: Robotic Nervous System (ROS 2)
---

# Module 1: Robotic Nervous System (ROS 2)

Welcome to Module 1 of our robotics education series. This module focuses on ROS 2 middleware for humanoid control, Python agents integration, and URDF modeling.

## About This Module

In this module, you will learn:
- The fundamentals of ROS 2 architecture
- How to integrate Python agents with ROS 2
- How to model humanoid robots using URDF

## Prerequisites

- Basic Python programming knowledge
- Understanding of robotics concepts (helpful but not required)

## Chapters

1. [ROS 2 Basics: Nodes, Topics, Services](./chapter-1-ros2-basics.md)
2. [Python & ROS 2: rclpy, publishers/subscribers](./chapter-2-python-ros2.md)
3. [URDF Modeling: Links, joints, sensors, simple humanoid](./chapter-3-urdf-modeling.md)
```

### 3. Create Chapter 1 Content

Create `docs/module-1-robotic-nervous-system/chapter-1-ros2-basics.md`:

```markdown
---
sidebar_position: 2
title: Chapter 1 - ROS 2 Basics
---

# Chapter 1: ROS 2 Basics - Nodes, Topics, Services

This chapter introduces the fundamental concepts of ROS 2: nodes, topics, and services.

## What is ROS 2?

ROS 2 (Robot Operating System 2) is flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

## Core Concepts

### Nodes

A node is a process that performs computation. ROS 2 is designed with the philosophy that a single process should only contain one node. Nodes can be written in different programming languages and can be distributed across multiple machines.

### Topics

Topics are named buses over which nodes exchange messages. A node can publish messages to a topic and/or subscribe to messages from a topic. This creates a many-to-many relationship where multiple nodes can publish and/or subscribe to the same topic.

### Services

Services provide a request/response communication pattern. A service has a client that sends a request and a server that receives the request, performs some action, and returns a response.

## Example: Simple Publisher and Subscriber

Here's a simple example of a publisher and subscriber in Python:

```python
# Publisher example
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Next Steps

In the next chapter, we'll explore how to integrate Python agents with ROS 2 using rclpy.
```

### 4. Create Chapter 2 Content

Create `docs/module-1-robotic-nervous-system/chapter-2-python-ros2.md`:

```markdown
---
sidebar_position: 3
title: Chapter 2 - Python & ROS 2
---

# Chapter 2: Python & ROS 2 - rclpy, publishers/subscribers

This chapter focuses on integrating Python agents with ROS 2 using the rclpy library.

## rclpy Overview

rclpy is the Python client library for ROS 2. It provides the interface between Python programs and the ROS 2 middleware.

## Installation

rclpy comes with the ROS 2 installation. Make sure you have sourced your ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash  # For Humble Hawksbill
```

## Creating Publishers and Subscribers

### Publisher

A publisher sends messages to a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherExample(Node):
    def __init__(self):
        super().__init__('publisher_example')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Python: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1
```

### Subscriber

A subscriber receives messages from a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberExample(Node):
    def __init__(self):
        super().__init__('subscriber_example')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

## Running the Example

1. Source your ROS 2 environment
2. Run the publisher: `python3 publisher_example.py`
3. In another terminal, run the subscriber: `python3 subscriber_example.py`

## Next Steps

In the next chapter, we'll explore URDF modeling for humanoid robots.
```

### 5. Create Chapter 3 Content

Create `docs/module-1-robotic-nervous-system/chapter-3-urdf-modeling.md`:

```markdown
---
sidebar_position: 4
title: Chapter 3 - URDF Modeling
---

# Chapter 3: URDF Modeling - Links, joints, sensors, simple humanoid

This chapter covers URDF (Unified Robot Description Format) modeling for humanoid robots.

## What is URDF?

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It can contain:

- Kinematic and dynamic description of each link
- Visual and collision properties
- Physical properties (inertial parameters)
- Joint definitions between links
- Sensor definitions

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

- `visual`: Defines how the link looks in visualization
- `collision`: Defines collision properties for physics simulation
- `inertial`: Defines mass and inertial properties
- `material`: Defines color and texture properties

## Joint Types

- `fixed`: No movement between links
- `revolute`: Rotational joint with limits
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint
- `floating`: 6 DOF joint
- `planar`: Movement on a plane

## Creating a Simple Humanoid

A simple humanoid model would include:
- Links for body parts (head, torso, arms, legs)
- Joints to connect these parts
- Appropriate joint types for realistic movement

## Next Steps

Congratulations! You've completed Module 1. You now understand ROS 2 basics, Python integration, and URDF modeling for humanoid robots.
```

## Configuring the Sidebar

### 1. Update sidebars.js

Edit `sidebars.js` to include Module 1:

```javascript
// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
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
    // Add more modules here
  ],
};

module.exports = sidebars;
```

## Running and Building

### Development Mode

```bash
# Start development server
npm run start

# This opens your site in your browser at http://localhost:3000
# The site reloads automatically when you make changes
```

### Production Build

```bash
# Build the site for production
npm run build

# This creates a build/ directory with optimized static files
```

### Local Preview of Production Build

```bash
# Preview the production build locally
npm run serve
```

## Deploying to GitHub Pages

1. Update `docusaurus.config.js` with your GitHub repository details
2. Run the deployment command:

```bash
# Deploy to GitHub Pages
GIT_USER=<your-github-username> npm run deploy
```

## Troubleshooting

### Common Issues

1. **Node.js version too old**: Make sure you have Node.js 18.0 or higher
2. **Missing dependencies**: Run `npm install` to install all dependencies
3. **Port already in use**: The development server runs on port 3000 by default; if it's in use, Docusaurus will try another port

### Getting Help

- Check the [Docusaurus documentation](https://docusaurus.io/docs)
- Use `npm run start -- --port <port_number>` to specify a different port
- Verify your configuration in `docusaurus.config.js`