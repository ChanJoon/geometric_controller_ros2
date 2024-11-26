from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mav_name', default_value='iris'),
        DeclareLaunchArgument('command_input', default_value='2'),
        DeclareLaunchArgument('gazebo_simulation', default_value='true'),
        DeclareLaunchArgument('visualization', default_value='true'),
        DeclareLaunchArgument('log_output', default_value='screen'),

        # Geometric Controller Node
        Node(
            package='geometric_controller',
            executable='geometric_controller_node',
            name='geometric_controller',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'mav_name': LaunchConfiguration('mav_name'),
                'ctrl_mode': LaunchConfiguration('command_input'),
                'enable_sim': LaunchConfiguration('gazebo_simulation'),
                'enable_gazebo_state': True,
                'max_acc': 20.0,
                'Kp_x': 12.0,
                'Kp_y': 12.0,
                'Kp_z': 10.0,
                'Kv_x': 3.0,
                'Kv_y': 3.0,
                'Kv_z': 3.3,
            }]
        ),

        # Trajectory Publisher Node
        Node(
            package='trajectory_publisher',
            executable='trajectory_publisher',
            name='trajectory_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'trajectory_type': 2,
                'shape_omega': 1.2,
                'initpos_z': 2.0,
                'reference_type': 2,
            }]
        ),

        # RViz Visualization
        # GroupAction([
        #     Node(
        #         package='rviz2',
        #         executable='rviz2',
        #         name='rviz',
        #         arguments=[
        #             '-d', os.path.join(
        #                 get_package_share_directory('geometric_controller'), 'launch', 'config_file.rviz')],
        #         condition=IfCondition(LaunchConfiguration('visualization'))
        #     )
        # ]),
    ])
