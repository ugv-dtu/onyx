from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def configure(context):
    mode = LaunchConfiguration('mode').perform(context)
    train = LaunchConfiguration('train').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    policy = LaunchConfiguration('policy').perform(context).strip()

    if mode not in ('mppi', 'rl', 'mppi_rl'):
        raise RuntimeError('mode must be mppi, rl, or mppi_rl.')

    if train not in ('true', 'false'):
        raise RuntimeError('train must be true or false.')

    if use_sim_time not in ('true', 'false'):
        raise RuntimeError('use_sim_time must be true or false.')

    training = train == 'true'
    simulation = use_sim_time == 'true'

    if training and not simulation:
        raise RuntimeError('Training is supported only in simulation.')

    if training and mode == 'mppi':
        raise RuntimeError('MPPI has no training mode. Use train:=false.')

    if mode != 'mppi' and not training:
        if not policy:
            raise RuntimeError(
                'A saved policy is required. Set policy:=/path/to/model.zip'
            )
        if not Path(policy).is_file():
            raise RuntimeError(f'Policy file does not exist: {policy}')

    environment = 'simulation' if simulation else 'real'
    operation = 'training' if training else 'evaluation or operation'

    actions = [
        LogInfo(msg=f'Environment: {environment}'),
        LogInfo(msg=f'Mode: {mode}'),
        LogInfo(msg=f'Operation: {operation}'),
    ]

    if mode == 'mppi':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('planning'),
                        'launch',
                        'planning.launch.py',
                    ])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'use_rviz': use_rviz,
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('description'),
                'launch',
                'description.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch navigation RViz2 window.',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='mppi',
            description='Controller selection: mppi, rl, or mppi_rl.',
        ),
        DeclareLaunchArgument(
            'train',
            default_value='false',
            description='Enable learning in simulation for an RL mode.',
        ),
        DeclareLaunchArgument(
            'policy',
            default_value='',
            description='Saved policy file required for RL evaluation or operation.',
        ),
        description_launch,
        OpaqueFunction(function=configure),
    ])
