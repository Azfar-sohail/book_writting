---
sidebar_position: 7
title: "Chapter 7: Nodes, Topics, and Messages"
---

# Chapter 7: Nodes, Topics, and Messages

## Introduction

The publish-subscribe communication pattern is fundamental to ROS 2 architecture. This chapter explores nodes, topics, and messages - the core building blocks for distributed robotics software.

## Learning Objectives

- Understand the concept of nodes in ROS 2
- Learn about topics and the publish-subscribe pattern
- Explore message types and their structure
- Implement simple publisher and subscriber nodes
- Recognize best practices for communication design

## Conceptual Foundations

Nodes are the fundamental execution units in ROS 2, each running in its own process. They communicate through topics using the publish-subscribe pattern, which enables loose coupling between different parts of the system.

Topics allow for one-to-many communication where publishers send messages to topics and subscribers receive messages from topics. This pattern enables flexible system architectures where components can be added or removed without affecting others.

Messages define the data structures used for communication, with standardized types for common data and the ability to define custom types for specific applications.

## Technical Explanation

Nodes in ROS 2 have several key characteristics:

- **Process Isolation**: Each node runs in its own process
- **Resource Management**: Nodes manage their own resources
- **Communication Interface**: Nodes expose interfaces through topics, services, and actions
- **Lifecycle**: Nodes have well-defined states (unconfigured, inactive, active, finalized)

Topics enable asynchronous communication with several important features:

- **One-to-Many**: A publisher can have multiple subscribers
- **Asynchronous**: Publishers and subscribers operate independently
- **Message Types**: Each topic has a specific message type
- **Quality of Service**: Configurable delivery guarantees

## Practical Examples

Consider a sensor data publisher:

```python
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

def main():
    rclpy.init()
    node = rclpy.create_node('laser_publisher')
    publisher = node.create_publisher(LaserScan, '/scan', 10)

    def publish_scan():
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 0.01
        msg.ranges = [1.0] * 314  # 314 range measurements
        publisher.publish(msg)

    timer = node.create_timer(0.1, publish_scan)  # 10 Hz
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

This example shows how to create a publisher for laser scan data.

## System Integration Perspective

Node and topic integration involves:

- **Topic Naming**: Using consistent naming conventions
- **Message Types**: Choosing appropriate message types for data
- **Quality of Service**: Configuring appropriate delivery guarantees
- **Data Rates**: Managing communication frequency
- **Resource Management**: Efficient use of communication resources

The integration must ensure reliable and efficient communication between nodes.

## Diagram Placeholders


*Diagram showing nodes communicating via topics*


*Illustration of message types and structure*

## Summary

- Nodes are the fundamental execution units in ROS 2
- Topics enable publish-subscribe communication
- Messages define data structures for communication
- Quality of Service provides configurable delivery guarantees
- Integration must ensure efficient communication

## Exercises

1. Create a publisher node that publishes robot pose information.
2. Implement a subscriber node that processes pose data and calculates distance traveled.
3. Design custom message types for a humanoid robot's joint states.
4. Explain the difference between reliable and best-effort QoS settings for topics.
5. Analyze the communication patterns in a multi-robot system.