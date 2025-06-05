# This package must be launched from a ubuntu 24 docker container on Quori with a 2-way ROS bridge.

# To launch object detection and waist rotation use:
ros2 launch person_detection person_detection_launch.py

# The laser_filter node can be adjusted to filter for different ranges or angles.