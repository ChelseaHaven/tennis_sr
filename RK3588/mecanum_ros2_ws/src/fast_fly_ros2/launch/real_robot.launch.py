"""Real chassis launch: F407 CAN telemetry -> odometry -> planner -> CAN commands."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fast_fly_ros2'))
    real_time = {'use_sim_time': False}
    v_max = LaunchConfiguration('v_max')
    a_max = LaunchConfiguration('a_max')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('v_max', default_value='0.3',
                              description='Conservative real-robot speed limit in m/s'),
        DeclareLaunchArgument('a_max', default_value='0.5',
                              description='Conservative real-robot acceleration limit in m/s^2'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[real_time, {'robot_description': (share / 'urdf/mecanum.urdf').read_text()}],
             output='screen'),
        Node(package='fast_fly_ros2', executable='f407_odom', parameters=[real_time],
             output='screen'),
        Node(package='fast_fly_ros2', executable='odom_tf', parameters=[real_time]),
        Node(package='fast_fly_ros2', executable='planner', parameters=[real_time, {
            'v_max': v_max, 'a_max': a_max}], output='screen'),
        Node(package='fast_fly_ros2', executable='tracker', parameters=[real_time],
             output='screen'),
        Node(package='fast_fly_ros2', executable='rviz_x11',
             arguments=['-d', str(share / 'rviz/mecanum.rviz')], parameters=[real_time],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
