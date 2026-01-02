---
sidebar_position: 8
title: "Chapter 8: Services, Actions, and Parameters"
---

# Chapter 8: Services, Actions, and Parameters

## Introduction

Beyond topics, ROS 2 provides additional communication patterns for synchronous operations, long-running tasks, and configuration management. This chapter covers services, actions, and parameters for comprehensive robotics communication.

## Learning Objectives

- Understand the service communication pattern in ROS 2
- Learn about actions for long-running tasks with feedback
- Explore parameter management in ROS 2
- Implement service servers and clients
- Recognize when to use each communication pattern

## Conceptual Foundations

Services provide synchronous request-response communication, ideal for operations that have a clear beginning and end. They are blocking calls where the client waits for the server to process the request and return a response.

Actions are designed for long-running tasks that provide feedback during execution and have clear goal, feedback, and result structures. They support preemption and are ideal for navigation, manipulation, and other extended operations.

Parameters provide a configuration system for runtime settings that can be changed during operation. They support different data types and can be set at startup or dynamically during runtime.

## Technical Explanation

Services in ROS 2 have the following characteristics:

- **Synchronous**: Client waits for server response
- **Request-Response**: One request, one response
- **Service Types**: Defined using .srv files with request/response structure
- **Blocking Calls**: Client blocks until response received

Actions provide more sophisticated communication:

- **Goal-Feeback-Result**: Three-part communication pattern
- **Long-Running**: Designed for operations taking significant time
- **Preemption**: Ability to cancel ongoing actions
- **Action Types**: Defined using .action files with goal, feedback, result

Parameters manage configuration:

- **Configuration**: Runtime configuration management
- **Dynamic Reconfiguration**: Changing parameters during runtime
- **Parameter Types**: Support for various data types
- **Parameter Callbacks**: Hooks for parameter changes

## Practical Examples

Consider a navigation service:

```python
import rclpy
from rclpy.action import ActionServer
from geometry_msgs.msg import Point
from nav_msgs.action import NavigateToPose

class NavigationActionServer:
    def __init__(self):
        self._action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        target_pose = goal_handle.request.pose
        feedback_msg = NavigateToPose.Feedback()

        # Simulate navigation progress
        for i in range(0, 100):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()

            # Update feedback
            feedback_msg.distance_remaining = 100 - i
            goal_handle.publish_feedback(feedback_msg)

            # Simulate movement
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle.succeed()
        result = NavigateToPose.Result()
        result.result = True
        return result
```

This example shows how to implement an action server for navigation.

## System Integration Perspective

Integration of services, actions, and parameters involves:

- **Pattern Selection**: Choosing the appropriate communication pattern
- **Error Handling**: Managing failures and exceptions
- **Configuration Management**: Using parameters for system configuration
- **Service Discovery**: Finding and connecting to available services
- **Action Management**: Managing long-running operations

The integration must ensure appropriate use of each communication pattern.

## Diagram Placeholders


*Diagram comparing topics, services, and actions*


*Illustration of parameter system in ROS 2*

## Summary

- Services provide synchronous request-response communication
- Actions handle long-running tasks with feedback
- Parameters manage runtime configuration
- Each pattern has appropriate use cases
- Integration must match patterns to requirements

## Exercises

1. Implement a service for robot navigation to a specified goal.
2. Create an action for robot arm trajectory execution with feedback.
3. Design a parameter system for configuring robot behavior.
4. Compare the use of topics vs services vs actions for different robotics scenarios.
5. Implement a complex action with multiple feedback types.