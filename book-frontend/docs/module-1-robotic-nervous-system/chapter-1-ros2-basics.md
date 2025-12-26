---
sidebar_position: 2
title: "Chapter 1 - ROS 2 Basics: Nodes, Topics, Services"
---

# Chapter 1: ROS 2 Basics - Nodes, Topics, Services

This chapter introduces the fundamental concepts of ROS 2: nodes, topics, and services.

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

ROS 2 addresses many of the limitations of the original ROS, including:
- Improved real-time support
- Better security and authentication
- Enhanced multi-robot support
- More reliable communication

## Core Concepts

### Nodes

A node is a process that performs computation. ROS 2 is designed with the philosophy that a single process should only contain one node. Nodes can be written in different programming languages and can be distributed across multiple machines.

Key characteristics of nodes:
- Each node runs independently
- Nodes communicate with each other through topics and services
- Nodes can be written in different languages (C++, Python, etc.)
- Nodes are managed by the ROS 2 execution model

### Topics

Topics are named buses over which nodes exchange messages. A node can publish messages to a topic and/or subscribe to messages from a topic. This creates a many-to-many relationship where multiple nodes can publish and/or subscribe to the same topic.

Characteristics of topics:
- Unidirectional communication
- Publish/subscribe pattern
- Anonymous communication (publishers don't know who subscribes)
- Multiple publishers and subscribers can exist for the same topic

### Services

Services provide a request/response communication pattern. A service has a client that sends a request and a server that receives the request, performs some action, and returns a response.

Characteristics of services:
- Synchronous communication
- Request/response pattern
- Direct client-server relationship
- Request and response types are defined in service definition files

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

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# Subscriber example
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

## Running the Example

To run these examples:

1. Make sure your ROS 2 environment is sourced
2. Save the publisher code as `publisher_member_function.py`
3. Save the subscriber code as `subscriber_member_function.py`
4. Run the publisher: `python3 publisher_member_function.py`
5. In another terminal, run the subscriber: `python3 subscriber_member_function.py`

## Summary

In this chapter, you've learned about the fundamental concepts of ROS 2:
- Nodes as computational units
- Topics for publish/subscribe communication
- Services for request/response communication

These concepts form the foundation of all ROS 2 applications and will be essential as you continue through this module.

## Next Steps

In the next chapter, we'll explore how to integrate Python agents with ROS 2 using rclpy, including creating publishers and subscribers.