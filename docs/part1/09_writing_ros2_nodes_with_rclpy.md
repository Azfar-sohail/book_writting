---
sidebar_position: 9
title: "Chapter 9: Writing ROS 2 Nodes with Python (rclpy)"
---

# Chapter 9: Writing ROS 2 Nodes with Python (rclpy)

## Introduction

Python provides an excellent interface for ROS 2 development through the rclpy library. This chapter covers creating ROS 2 nodes using Python, including best practices for robotics software development.

## Learning Objectives

- Understand the rclpy library and its usage
- Learn to create nodes with various communication patterns
- Explore node lifecycle and management
- Implement proper error handling and logging
- Recognize best practices for Python-based robotics software

## Conceptual Foundations

The rclpy library provides Python bindings for ROS 2, allowing developers to create ROS 2 nodes using Python. This provides a high-level, easy-to-use interface for robotics development while maintaining access to ROS 2's powerful communication and tooling features.

Python nodes are ideal for rapid prototyping, scripting, and applications where development speed is more important than maximum performance. The language's rich ecosystem of libraries also makes it excellent for data processing and AI applications.

## Technical Explanation

The rclpy library structure includes:

- **Initialization**: `rclpy.init()` initializes the ROS client library
- **Node Creation**: `rclpy.create_node()` creates a node instance
- **Spin Functions**: `rclpy.spin()` processes callbacks and services
- **Cleanup**: `rclpy.shutdown()` cleans up resources

Node components include:

- **Publishers**: Create publishers for topic communication
- **Subscribers**: Create subscribers for receiving messages
- **Services**: Create service servers and clients
- **Actions**: Create action servers and clients
- **Timers**: Execute callbacks at regular intervals
- **Parameters**: Manage configuration parameters

## Practical Examples

Consider a complete publisher-subscriber example:

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
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

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
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    publisher = MinimalPublisher()
    subscriber = MinimalSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example shows a complete publisher-subscriber pair in Python.

## System Integration Perspective

Python node integration involves:

- **Performance Considerations**: Understanding Python's performance characteristics
- **Threading Models**: Managing concurrency and callbacks
- **Resource Management**: Proper cleanup and resource handling
- **Error Handling**: Managing exceptions and node failures
- **Logging Integration**: Using ROS 2's logging system

The integration must balance development speed with system requirements.

## Diagram Placeholders


*Diagram showing the rclpy library architecture*


*Illustration of ROS 2 node components*

## Summary

- rclpy provides Python bindings for ROS 2
- Python is excellent for rapid prototyping and scripting
- Nodes include publishers, subscribers, services, and more
- Proper error handling and resource management are essential
- Integration must consider performance and resource usage

## Exercises

1. Create a ROS 2 node that publishes robot joint states.
2. Implement a subscriber node that processes sensor data and publishes control commands.
3. Design a node with parameters for configuring robot behavior.
4. Research and implement a multi-threaded ROS 2 node for handling multiple tasks.
5. Create a complex node that combines multiple communication patterns.