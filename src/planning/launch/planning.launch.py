import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planning_share = get_package_share_directory('planning')
    slam_share = get_package_share_directory('slam_toolbox')
    nav2_share = get_package_share_directory('nav2_bringup')

    slam_params = os.path.join(planning_share, 'config', 'slam_params.yaml')
    nav2_params = os.path.join(planning_share, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(planning_share, 'rviz', 'mppi_navigation.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz config file'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_rviz_config,
        slam_launch,
        nav2_launch,
        rviz_node,
    ])
