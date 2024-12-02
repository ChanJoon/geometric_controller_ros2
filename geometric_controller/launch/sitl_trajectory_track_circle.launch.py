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
        DeclareLaunchArgument('initpos_z', default_value='2.0'),
        DeclareLaunchArgument('reference_type', default_value='2'), # 2 for circle, 16 for hover
        DeclareLaunchArgument('enable_sim', default_value='true'),
        DeclareLaunchArgument('ctrl_mode', default_value='2'),
        DeclareLaunchArgument('visualization', default_value='true'),

        # Geometric Controller Node
        Node(
            package='geometric_controller',
            executable='geometric_controller_node',
            name='geometric_controller',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'mav_name': LaunchConfiguration('mav_name'),
                'ctrl_mode': LaunchConfiguration('ctrl_mode'),
                'enable_sim': LaunchConfiguration('enable_sim'),
                'enable_gazebo_state': True,
                'max_acc': 10.0,
                'attctrl_constant': 0.3,
                'normalizedthrust_constant': 0.06,
                'normalizedthrust_offset': 0.1,
                'Kp_x': 10.0,
                'Kp_y': 10.0,
                'Kp_z': 10.0,
                'Kv_x': 5.0,
                'Kv_y': 5.0,
                'Kv_z': 10.0,
            }],
            # prefix='gdb -ex run --args',
        ),

        # Trajectory Publisher Node
        Node(
            package='trajectory_publisher',
            executable='trajectory_publisher',
            name='trajectory_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'trajectory_type': LaunchConfiguration('trajectory_type'),
                'shape_omega': LaunchConfiguration('shape_omega'),
                'initpos_z': LaunchConfiguration('initpos_z'),
                'reference_type': LaunchConfiguration('reference_type'),
            }]
        ),

        # Optional Visualization with RViz
        # GroupAction([
        #     Node(
        #         package='rviz2',
        #         executable='rviz2',
        #         name='rviz',
        #         output='screen',
        #         arguments=['-d', os.path.join(
        #             get_package_share_directory('geometric_controller'), 'launch', 'config_file.rviz')],
        #         condition=IfCondition(LaunchConfiguration('visualization')),
        #     )
        # ])
    ])
