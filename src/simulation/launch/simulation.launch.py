from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
    TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
    PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    world_name = LaunchConfiguration('world')
    world_sdf = PythonExpression(["'", world_name, ".sdf'"])

    world = PathJoinSubstitution([
        FindPackageShare('simulation'), 'worlds', world_sdf])

    sim_flags = PythonExpression([
        "'-r -s ' if '", LaunchConfiguration('headless'),
        "' == 'true' else '-r '"])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': [sim_flags, world_path], 'gz_version': '8'}.items(),
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', world_name,
                '-topic', 'robot_description',
                '-name', 'onyx',
            ],
            output='screen',
        )],
    )

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/front_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/front_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/rear_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/rear_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/aiming_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/aiming_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/turret/yaw/cmd@std_msgs/msg/Float64]gz.msgs.Double',
        '/turret/pitch/cmd@std_msgs/msg/Float64]gz.msgs.Double',
    ]

    extra_actions = []

    if is_arena:
        bridge_args.extend([
            '/arena/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            f'/world/{gz_world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ])
        arena_manager = Node(
            package='simulation',
            executable='arena_env',
            name='arena_env',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
                'min_obstacles': ParameterValue(LaunchConfiguration('min_obstacles'), value_type=int),
                'max_obstacles': ParameterValue(LaunchConfiguration('max_obstacles'), value_type=int),
                'publish_world_tf': ParameterValue(LaunchConfiguration('publish_world_tf'), value_type=bool),
            }],
        )
        extra_actions.append(arena_manager)

    bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen',
    )

    gps_visualizer = Node(
        package='simulation',
        executable='onyx_gps_visualizer',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    show_sim_rviz = PythonExpression([
        "'", LaunchConfiguration('headless'), "' != 'true' and '",
        LaunchConfiguration('use_rviz'), "' == 'true'"
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('simulation'), 'rviz', 'onyx_lidar.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(show_sim_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='tekno_world',
            description='World name without .sdf (tekno_world or mppi_test_world).'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = no window (use for training).'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch simulation RViz2 window.'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gz_sim,
        spawn_robot,
        bridges,
        gps_visualizer,
        rviz,
        *extra_actions,
    ]


def generate_launch_description():
    resource_path = [
        FindPackageShare('simulation'), ':',
        PathJoinSubstitution([FindPackagePrefix('description'), 'share']), ':',
        EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='arena',
            description='World to load: arena (navigation_arena) or tekno (tekno_world).'),
        DeclareLaunchArgument('headless', default_value='false',
            description='Run Gazebo in headless mode.'),
        DeclareLaunchArgument('seed', default_value='-1',
            description='Random seed for arena layout (-1 for random).'),
        DeclareLaunchArgument('min_obstacles', default_value='3',
            description='Minimum active cones on reset (0 to 10).'),
        DeclareLaunchArgument('max_obstacles', default_value='10',
            description='Maximum active cones on reset (0 to 10).'),
        DeclareLaunchArgument('publish_world_tf', default_value='true',
            description='Publish ground-truth world->odom TF.'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        OpaqueFunction(function=launch_setup),
    ])
