# This is a rewritten version of Quori's teleop package. 
# It can be launched from the rolling_ws using a docker container and ROS bridge.
ros2 launch ros2_teleop ros2_teleop_launch.py

# Having this node in ROS 2 allows for teleoperation from a remote desktop on ubuntu 24.
# Previous teleoperation bugs such as freezing on startup have been repaired.