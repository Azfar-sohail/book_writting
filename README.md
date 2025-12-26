# AI-Native Technical Textbook Platform

This repository contains an AI-Native Technical Textbook Platform built with Docusaurus v3, covering Physical AI, Robotics, and Humanoid Systems. The platform includes comprehensive documentation on ROS 2, simulation, vision-language-action systems, and autonomous humanoid development.

## Structure

The textbook is organized in 2 comprehensive parts with 20 detailed chapters:

### Part I – Foundations & Robotic Systems
1. What Is Physical AI?
2. Basics of Robotics Systems
3. AI Meets the Physical World
4. Software Stack for Humanoid Robots
5. Simulation Before Reality
6. Introduction to ROS 2
7. Nodes, Topics, and Messages
8. Services, Actions, and Parameters
9. Writing ROS 2 Nodes with Python (rclpy)
10. URDF & Robot Description

### Part II – Digital Twins, Autonomy & Vision-Language-Action
1. Gazebo: Physics & Simulation
2. Sensors in Simulation
3. Unity for Human-Robot Interaction
4. NVIDIA Isaac Sim
5. Isaac ROS & Navigation (Nav2)
6. What Is Vision-Language-Action (VLA)?
7. Voice to Text with Whisper
8. Language to Action Planning
9. Autonomous Decision Making
10. Capstone: The Autonomous Humanoid

## Development

This textbook is built using Docusaurus 3. To run locally:

```bash
npm install
npm start
```

The site will be available at http://localhost:3000/

## Deployment

This website is deployed to GitHub Pages.

### Local Development

```
npm install
npm start
```

### Deployment to GitHub Pages

The site is automatically deployed via GitHub Actions when changes are pushed to the main branch.

To deploy manually:
```
GIT_USER=<Your GitHub username> USE_SSH=true npm run deploy
```

The website will be available at: https://your-username.github.io/ai-native-textbook-platform/

## Target Audience

- Beginners to intermediate learners
- Engineering students
- AI and Robotics practitioners
- Developers working with ROS 2 and autonomous systems

## Features

- Comprehensive coverage of ROS 2 and robotics development
- Integration with modern simulation platforms (Gazebo, Isaac Sim)
- Vision-Language-Action (VLA) systems implementation
- NVIDIA Isaac ROS integration
- GitHub Pages deployment ready
- Beginner-friendly with practical examples
- Industry-aligned content and practices# bok-assongment
