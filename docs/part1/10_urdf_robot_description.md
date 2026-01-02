---
sidebar_position: 10
title: "Chapter 10: URDF & Robot Description"
---

# Chapter 10: URDF & Robot Description

## Introduction

URDF (Unified Robot Description Format) is the standard for describing robot models in ROS. This chapter covers creating and working with URDF files to define robot geometry, kinematics, and dynamics for simulation and visualization.

## Learning Objectives

- Understand the structure and components of URDF files
- Learn to create robot models with links and joints
- Explore visual and collision properties
- Implement kinematic chains and robot descriptions
- Recognize best practices for robot modeling

## Conceptual Foundations

URDF files define robot models using XML, describing the physical structure of robots including links (rigid bodies), joints (connections between links), and their properties. This description is used by ROS tools for visualization, simulation, and kinematic calculations.

The robot model defines the kinematic structure, which is essential for motion planning, control, and simulation. The accuracy of the model affects the performance of algorithms that depend on it.

## Technical Explanation

URDF structure includes:

- **Links**: Rigid bodies that make up the robot
- **Joints**: Connections between links with defined motion
- **Visual**: Geometry for visualization
- **Collision**: Geometry for collision detection
- **Inertial**: Mass properties for dynamics simulation

Joint types include:

- **Fixed**: No movement between links
- **Revolute**: Single axis rotation with limits
- **Continuous**: Single axis rotation without limits
- **Prismatic**: Single axis translation with limits
- **Floating**: Six degrees of freedom
- **Planar**: Motion in a plane

## Practical Examples

Consider a simple robot arm:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- First joint -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- First link -->
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

This example shows how to define a simple robot with links and joints.

## System Integration Perspective

URDF integration involves:

- **Model Accuracy**: Ensuring the model matches the real robot
- **Kinematic Chain**: Defining proper parent-child relationships
- **Mass Properties**: Providing accurate inertial parameters
- **Collision Geometry**: Defining appropriate collision models
- **Visualization**: Providing good visual representation

The integration must ensure accurate robot models for simulation and control.

## Diagram Placeholders


*Diagram showing the structure of URDF files*


*Example of a robot model visualized from URDF*

## Summary

- URDF defines robot models using XML
- Links and joints form the kinematic structure
- Visual and collision properties define appearance and interaction
- Accurate models are essential for simulation and control
- Integration must ensure model accuracy

## Exercises

1. Create a URDF file for a simple 6-DOF robotic arm.
2. Design a mobile robot URDF with differential drive kinematics.
3. Add visual and collision properties to your robot model.
4. Research and implement a URDF using Xacro macros for complex robots.
5. Validate your URDF model using ROS tools.