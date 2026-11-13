from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_launch = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'launch',
        'bringup_base.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='mppi',
            description='mppi, rl, or mppi_rl.',
        ),
        DeclareLaunchArgument(
            'train',
            default_value='false',
            description='Must remain false for real-robot operation.',
        ),
        DeclareLaunchArgument(
            'policy',
            default_value='',
            description='Saved policy file for RL operation.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'mode': LaunchConfiguration('mode'),
                'train': LaunchConfiguration('train'),
                'policy': LaunchConfiguration('policy'),
            }.items(),
        ),
    ])
