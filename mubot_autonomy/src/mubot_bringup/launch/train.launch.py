"""EM-PGPE training with N headless Gazebo workers. No ROS graph needed."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('case', default_value='AN4_Kmed_HM3_AR2'),
        DeclareLaunchArgument('workers', default_value='8'),
        DeclareLaunchArgument('out', default_value='data/train'),
        ExecuteProcess(cmd=['ros2', 'run', 'mubot_learning', 'trainer', '--case', lc('case'),
                            '--workers', lc('workers'), '--out', lc('out')], output='screen'),
    ])
