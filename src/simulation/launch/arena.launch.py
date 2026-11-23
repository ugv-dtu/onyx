from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('simulation'), 'launch', 'simulation.launch.py'])),
        launch_arguments={
            'world_name': 'arena',
            'headless': LaunchConfiguration('headless'),
            'seed': LaunchConfiguration('seed'),
            'min_obstacles': LaunchConfiguration('min_obstacles'),
            'max_obstacles': LaunchConfiguration('max_obstacles'),
            'publish_world_tf': LaunchConfiguration('publish_world_tf'),
        }.items(),
    )
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('description'), 'launch', 'description.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('seed', default_value='-1',
            description='-1 for random seed; otherwise repeat the same layout.'),
        DeclareLaunchArgument('min_obstacles', default_value='3',
            description='Minimum active cones on reset, from 0 to 10.'),
        DeclareLaunchArgument('max_obstacles', default_value='10',
            description='Maximum active cones on reset, from 0 to 10.'),
        DeclareLaunchArgument('publish_world_tf', default_value='true',
            description='Publish ground-truth world->odom; disable if localization publishes it.'),
        description_launch,
        sim_launch,
    ])
