"""Show the robot model in RViz without simulation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    noa = LaunchConfiguration('noa')
    xacro_file = PathJoinSubstitution([FindPackageShare('mubot_description'), 'urdf', 'mubot.urdf.xacro'])
    description = ParameterValue(Command(['xacro ', xacro_file, ' noa:=', noa]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('noa', default_value='4'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2'),
    ])
