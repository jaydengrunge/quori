# Quori ROS2 Workspace

This repository contains ROS2 packages developed for the Quori robot platform. The workspace includes tools for person detection, teleoperation, and point cloud processing.

---

## Repository Structure

### **Packages**
- **person_detection**: 
  - Nodes for detecting people using laser scans and point clouds, including clustering and filtering.
  - Includes utilities for detecting the closest person and publishing their location, and turning to face them.
  - Configurable via launch files and RViz visualization.

- **ros2_teleop**: 
  - Teleoperation utilities for controlling the Quori robot.
  - Provides nodes for sending velocity commands and interacting with the robot's base controller.
  - Code updated for ROS 2 allowing for easy editing and greater capability.

---

## Installation

### Prerequisites
- ROS2 Humble or later
- PCL (Point Cloud Library)
- Eigen3

### Build Instructions
1. Clone the repository:
   git clone https://github.com/jaydengrunge/quori.git

2. Install dependencies:
   sudo apt update

3. Build the packages with:
   colcon build

4. Source the worspace before launching:
   source install/setup.bash

---

## Usage

### Launch Files:


