from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        # Add trajectory_publisher node
        Node(
            package='trajectory_publisher',
            executable='trajectory_publisher',
            name='trajectory_publisher',
            output='screen',
            parameters=[{
                'trajectory_type': 1,
                'shape_omega': 1.2,
                'initpos_z': 2.0,
                'max_acc': 10.0,
                'Kpos_x': 8.0,
                'Kpos_y': 8.0,
                'Kpos_z': 30.0,
                'reference_type': 16
            }]
        )
    ])
