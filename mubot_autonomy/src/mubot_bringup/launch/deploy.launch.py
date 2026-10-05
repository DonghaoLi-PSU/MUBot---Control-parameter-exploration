"""Full autonomy stack on top of the simulation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def cfg(pkg, name):
    return PathJoinSubstitution([FindPackageShare(pkg), 'config', name])


def generate_launch_description():
    lc = LaunchConfiguration
    sim = {'use_sim_time': True}
    policy = PathJoinSubstitution([FindPackageShare('mubot_learning'), 'policies', lc('policy')])
    speed = cfg('mubot_localization', 'speed_model.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('policy', default_value='gamma_star_example.yaml'),
        IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare('mubot_bringup'), 'launch', 'sim.launch.py'])),
        Node(package='mubot_perception', executable='perception',
             parameters=[cfg('mubot_perception', 'perception.yaml'), sim]),
        Node(package='mubot_control', executable='behavior',
             parameters=[cfg('mubot_control', 'behavior.yaml'), sim]),
        Node(package='mubot_control', executable='gait_generator',
             parameters=[cfg('mubot_control', 'gait_generator.yaml'), {'policy_file': policy}, sim]),
        Node(package='mubot_localization', executable='gait_odometry',
             parameters=[cfg('mubot_localization', 'slam.yaml'), {'speed_model': speed}, sim]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[cfg('mubot_localization', 'ekf.yaml')],
             remappings=[('odometry/filtered', '/mubot/odometry/filtered')]),
        Node(package='mubot_localization', executable='slam',
             parameters=[cfg('mubot_localization', 'slam.yaml'), sim]),
        Node(package='mubot_mission', executable='mission',
             parameters=[cfg('mubot_mission', 'mission.yaml'), sim]),
        Node(package='mubot_bringup', executable='supervisor_node.py', parameters=[sim]),
        Node(package='diagnostic_aggregator', executable='aggregator_node',
             parameters=[cfg('mubot_bringup', 'diagnostics.yaml'), sim]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('mubot_bringup'), 'rviz', 'mubot.rviz'])],
             parameters=[sim]),
    ])
