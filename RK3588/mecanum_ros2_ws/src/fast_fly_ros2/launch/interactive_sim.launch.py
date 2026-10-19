"""Harmonic physics + ROS 2 planning + RViz2, with no ROS 1 components."""
import os
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    share = Path(get_package_share_directory('fast_fly_ros2'))
    plugin_lib = str(Path(get_package_prefix('mecanum_gz')) / 'lib')
    v_max = float(LaunchConfiguration('v_max').perform(context))
    a_max = float(LaunchConfiguration('a_max').perform(context))
    if not all(math.isfinite(v) and v > 0 for v in [v_max, a_max]):
        raise ValueError('v_max and a_max must be positive finite values')
    description = (share / 'urdf/mecanum.urdf').read_text()
    description = description.replace('<v_max>4.5</v_max>', f'<v_max>{v_max}</v_max>')
    description = description.replace('<a_max>3.0</a_max>', f'<a_max>{a_max}</a_max>')
    # Gazebo's URDF converter needs filesystem mesh URIs. RViz uses package URIs.
    gz_description = description.replace('package://fast_fly_ros2/', f'file://{share}/')
    sim_time = {'use_sim_time': True}
    return [
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                               plugin_lib + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')),
        # Run gz directly instead of through ros_gz_sim's shell=True wrapper.
        # The direct child process lets ros2 launch reliably stop Gazebo on Ctrl+C.
        ExecuteProcess(
            cmd=['gz', 'sim', '-r'] +
                ([] if LaunchConfiguration('gui').perform(context) == 'true' else ['-s']) +
                [str(share / 'worlds/mecanum.sdf'), '--force-version', '8'],
            name='gazebo', output='screen', on_exit=Shutdown(),
            sigterm_timeout='5', sigkill_timeout='2'),
        Node(package='ros_gz_sim', executable='create', name='spawn_mecanum',
             arguments=['-world', 'mecanum', '-name', 'nexus_4wd_mecanum',
                        '-string', gz_description, '-z', '0.02'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
             output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim_time, {'robot_description': description}], output='screen'),
        Node(package='fast_fly_ros2', executable='odom_tf', parameters=[sim_time]),
        Node(package='fast_fly_ros2', executable='planner', parameters=[sim_time, {
            'v_max': v_max, 'a_max': a_max}],
             output='screen'),
        Node(package='fast_fly_ros2', executable='tracker', parameters=[sim_time], output='screen'),
        Node(package='fast_fly_ros2', executable='rviz_x11', arguments=['-d', str(share / 'rviz/mecanum.rviz')],
             parameters=[sim_time], condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('v_max', default_value='4.5'),
        DeclareLaunchArgument('a_max', default_value='3.0'),
        OpaqueFunction(function=launch_nodes),
    ])
