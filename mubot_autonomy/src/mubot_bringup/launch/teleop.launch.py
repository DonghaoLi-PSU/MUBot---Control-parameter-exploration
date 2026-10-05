"""Simulation + gait generator + manual driving through cmd_vel. Nothing autonomous."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lc = LaunchConfiguration
    policy = PathJoinSubstitution([FindPackageShare('mubot_learning'), 'policies', lc('policy')])
    speed = PathJoinSubstitution([FindPackageShare('mubot_localization'), 'config', 'speed_model.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('policy', default_value='gamma_star_example.yaml'),
        IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare('mubot_bringup'), 'launch', 'sim.launch.py'])),
        Node(package='mubot_control', executable='gait_generator',
             parameters=[{'policy_file': policy, 'use_sim_time': True}],
             remappings=[('/mubot/primitive', '/mubot/primitive_override')]),
        Node(package='mubot_control', executable='twist_to_primitive',
             parameters=[{'speed_model': speed, 'use_sim_time': True}]),
        Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
             prefix='xterm -e'),
        Node(package='mubot_bringup', executable='supervisor_node.py',
             parameters=[{'use_sim_time': True}]),
    ])
