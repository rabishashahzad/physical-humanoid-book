---
sidebar_position: 3
title: "Chapter 2 - Python & ROS 2 Integration"
---

# Chapter 2: Python & ROS 2 - rclpy, publishers/subscribers

This chapter focuses on integrating Python agents with ROS 2 using the rclpy library, including creating publishers and subscribers.

## rclpy Overview

rclpy is the Python client library for ROS 2. It provides the interface between Python programs and the ROS 2 middleware, allowing you to create nodes, publishers, subscribers, services, and clients.

Key features of rclpy:
- Pythonic API that follows Python conventions
- Integration with ROS 2 middleware
- Support for all ROS 2 communication patterns
- Asynchronous execution capabilities

## Installation and Setup

rclpy comes with the ROS 2 installation. Make sure you have sourced your ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash  # For Humble Hawksbill
# OR for other distributions:
# source /opt/ros/iron/setup.bash
# source /opt/ros/rolling/setup.bash
```

## Creating Publishers and Subscribers

### Publisher

A publisher sends messages to a topic. Here's a more detailed example:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherExample(Node):
    def __init__(self):
        super().__init__('publisher_example')
        # Create a publisher with a topic name and message type
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)

        # Create a timer to publish messages at regular intervals
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Python: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    publisher_example = PublisherExample()

    try:
        rclpy.spin(publisher_example)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_example.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
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
        # Create a subscription with topic name, message type, and callback
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)  # QoS history depth
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    subscriber_example = SubscriberExample()

    try:
        rclpy.spin(subscriber_example)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_example.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Publisher Patterns

### Publisher with Custom Message Types

You can also publish custom message types:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # For robot movement commands

class MovementPublisher(Node):
    def __init__(self):
        super().__init__('movement_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def send_move_command(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent movement command: linear={linear_x}, angular={angular_z}')
```

### Publisher with Parameters

Publishers can also use parameters to control their behavior:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

class ParameterizedPublisher(Node):
    def __init__(self):
        super().__init__('parameterized_publisher')
        self.publisher_ = self.create_publisher(String, 'parameterized_topic', 10)

        # Declare a parameter with a default value
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('message_prefix', 'Parameterized: ')

        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.message_prefix = self.get_parameter('message_prefix').value

        # Create timer based on parameter
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.message_prefix}{self.counter}'
        self.publisher_.publish(msg)
        self.counter += 1
```

## Quality of Service (QoS) Settings

When creating publishers and subscribers, you can specify Quality of Service settings:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# Create a custom QoS profile
qos_profile = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE
)

# Use the QoS profile when creating publisher/subscriber
self.publisher_ = self.create_publisher(String, 'topic', qos_profile)
```

## Running the Examples

1. Source your ROS 2 environment
2. Run the publisher: `python3 publisher_example.py`
3. In another terminal, run the subscriber: `python3 subscriber_example.py`
4. You should see messages being published and received

## Best Practices

- Always use try/except blocks with rclpy.spin() to handle Ctrl+C gracefully
- Use proper logging with self.get_logger().info()
- Declare parameters at node initialization
- Consider QoS settings based on your application's requirements
- Use appropriate message types for your data

## Summary

In this chapter, you've learned how to:
- Use rclpy to create Python nodes
- Implement publishers and subscribers
- Work with parameters in ROS 2 nodes
- Configure Quality of Service settings

These skills are essential for creating Python-based ROS 2 applications and will be crucial as you work with more complex robotic systems.

## Next Steps

In the next chapter, we'll explore URDF modeling for humanoid robots, which will allow you to create robot descriptions that can be used with ROS 2.