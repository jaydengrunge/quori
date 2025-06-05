from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the laser_filter node
        Node(
            package='person_detection',
            executable='laser_filter',
            name='laser_filter',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Launch the laser detector node
        Node(
            package='person_detection',
            executable='detector',
            name='laser_detector',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # # Launch the cloud_filter node
        # Node(
        #     package='person_detection',
        #     executable='cloud_filter',
        #     name='cloud_filter',
        #     output='screen',
        #     parameters=[]
        # ),
        # # Launch the pointcloud detector node
        # Node(
        #     package='person_detection',
        #     executable='cloud_detector',
        #     name='cloud_detector',
        #     output='screen',
        #     parameters=[]
        # ),
        # Launch the face person node
        Node(
            package='person_detection',
            executable='face_closest',
            name='face_closest',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # # Launch RViz2
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', 'install/person_detection/share/person_detection/config/person_detection.rviz']
        # ),
    ])