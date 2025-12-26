# Quickstart Guide: Module 1 - Robotic Nervous System (ROS 2)

**Feature**: Module 1 - Robotic Nervous System (ROS 2)
**Date**: 2025-12-17
**Input**: Feature specification from `/specs/module-1-robotic-nervous-system/spec.md`

## Prerequisites

Before starting this module, ensure you have:

1. **ROS 2 Humble Hawksbill** installed (or newer LTS version)
2. **Python 3.8+** installed
3. **Basic Python programming knowledge**
4. **Terminal/command line familiarity**

### Installation Check

Verify your ROS 2 installation:
```bash
# Check ROS 2 version
ros2 --version

# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # For Ubuntu with Humble
# OR for development workspace:
source install/setup.bash
```

### Python Dependencies

Install required Python packages:
```bash
pip3 install rclpy
```

## Getting Started with ROS 2 Basics

### 1. Create a Simple Publisher Node

Create a new ROS 2 Python package:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python simple_publisher
cd simple_publisher/simple_publisher
```

Create a publisher script `publisher_member_function.py`:
```python
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


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Create a Simple Subscriber Node

Create a subscriber script `subscriber_member_function.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Run the Publisher and Subscriber

Terminal 1 - Run the publisher:
```bash
cd ~/ros2_ws
source install/setup.bash
python3 install/simple_publisher/lib/python3.8/site-packages/simple_publisher/publisher_member_function.py
```

Terminal 2 - Run the subscriber:
```bash
cd ~/ros2_ws
source install/setup.bash
python3 install/simple_publisher/lib/python3.8/site-packages/simple_publisher/subscriber_member_function.py
```

## Creating Your First Service

### 1. Create a Service Server

Create `service_member_function.py`:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Create a Service Client

Create `client_member_function.py`:
```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Basic URDF Model

### 1. Create a Simple URDF File

Create `simple_humanoid.urdf`:
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
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

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.1 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.1 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joint connecting base to torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joint connecting torso to head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

### 2. Visualize the URDF Model

To visualize the URDF model:
```bash
# Install visualization tools if not already installed
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher

# Launch RViz to visualize the model
ros2 run rviz2 rviz2

# In another terminal, publish the robot state:
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat simple_humanoid.urdf)
```

## Next Steps

1. **Chapter 1**: Complete the ROS 2 Basics tutorial with nodes, topics, and services
2. **Chapter 2**: Implement Python agents with rclpy for publishers and subscribers
3. **Chapter 3**: Create more complex humanoid models with URDF

## Troubleshooting

### Common Issues

1. **ROS 2 Environment Not Sourced**
   - Error: "command 'ros2' not found"
   - Solution: Source your ROS 2 installation: `source /opt/ros/humble/setup.bash`

2. **Python Import Errors**
   - Error: "No module named 'rclpy'"
   - Solution: Ensure you have sourced ROS 2 environment before running Python scripts

3. **Node Communication Issues**
   - Symptom: Nodes not communicating
   - Solution: Check that nodes are on the same ROS domain ID and network

### Getting Help

- Check the ROS 2 documentation: https://docs.ros.org/en/humble/
- Use `ros2 topic list` and `ros2 service list` to see available topics/services
- Use `ros2 run` command to run nodes from packages