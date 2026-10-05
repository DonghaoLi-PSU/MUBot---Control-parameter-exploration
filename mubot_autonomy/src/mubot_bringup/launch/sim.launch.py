"""Gazebo world + μBot + bridges + robot_state_publisher."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lc = LaunchConfiguration
    gz_share = FindPackageShare('mubot_gazebo')
    xacro_file = PathJoinSubstitution([FindPackageShare('mubot_description'), 'urdf', 'mubot.urdf.xacro'])
    robot = ParameterValue(Command(['xacro ', xacro_file, ' noa:=', lc('noa'), ' ar:=', lc('ar'),
                                    ' hydro:=', lc('hydro'), ' added_mass:=', lc('added_mass'),
                                    ' sensors:=', lc('sensors')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='mubot_tank.sdf'),
        DeclareLaunchArgument('noa', default_value='4'),
        DeclareLaunchArgument('ar', default_value='AR2'),
        DeclareLaunchArgument('hydro', default_value='HM3'),
        DeclareLaunchArgument('added_mass', default_value='true'),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
            launch_arguments={'gz_args': [PathJoinSubstitution([gz_share, 'worlds', lc('world')]), ' -r']}.items()),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'mubot', '-topic', 'robot_description', '-z', '0.0']),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': PathJoinSubstitution([gz_share, 'config', 'bridge.yaml']),
                          'use_sim_time': True}]),
        Node(package='mubot_gazebo', executable='mubot_gz_cmd_bridge',
             parameters=[{'use_sim_time': True}]),
    ])
