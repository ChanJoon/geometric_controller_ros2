from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    return LaunchDescription([
        DeclareLaunchArgument('mav_name', default_value='iris'),
        DeclareLaunchArgument('trajectory_type', default_value='1'),
        DeclareLaunchArgument('shape_omega', default_value='1.2'),
        DeclareLaunchArgument('init_pos_z', default_value='0.1'),
        DeclareLaunchArgument('reference_type', default_value='2'), # 2 for circle, 16 for hover
        DeclareLaunchArgument('enable_sim', default_value='true'),
        DeclareLaunchArgument('ctrl_mode', default_value='2'),
        DeclareLaunchArgument('visualization', default_value='true'),

        # Trajectory Publisher Node
        Node(
            package='trajectory_publisher',
            executable='trajectory_publisher',
            name='trajectory_publisher',
            output='screen',
            parameters=[{
                # 'use_sim_time': True,
                'trajectory_type': LaunchConfiguration('trajectory_type'),
                'shape_omega': LaunchConfiguration('shape_omega'),
                'init_pos_z': LaunchConfiguration('init_pos_z'),
                'reference_type': LaunchConfiguration('reference_type'),
            }]
        ),
    ])
