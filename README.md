# Quori ROS2 Workspace

This repository contains ROS2 packages developed for the Quori robot platform. The workspace includes tools for person detection, teleoperation, and point cloud processing.

---

## Repository Structure

### **Packages**
- **person_detection**: 
  - Nodes for detecting people using laser scans and point clouds.
  - Detects the closest person and publishes their location, and turns to face them.

- **ros2_teleop**: 
  - Teleoperation utilities for controlling the Quori robot.
  - Provides nodes for sending velocity commands from a controller to the robots base controller.
  - Code updated for ROS 2 allowing for easy editing and greater capability.

---

## Installation

### Prerequisites
- ROS2 Humble or later
- PCL (Point Cloud Library)
- Eigen3

### Build Instructions
1. Clone the repository:
   ```bash
   git clone https://github.com/jaydengrunge/quori.git

2. Install dependencies:
   ```bash
   sudo apt update

3. Build the packages with:
   ```bash
   colcon build

4. Source the worspace before launching:
   ```bash
   source install/setup.bash

---

## Usage

### Launch Files:
1. Launch the teleop node:
   ```bash
   ros2 launch ros2_teleop ros2_teleop_launch.py

2. Launch the person detection node:
   ```bash
   ros2 launch person_detection person_detection_launch.py
