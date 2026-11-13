from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def configure(context):
    mode = LaunchConfiguration('mode').perform(context)
    train = LaunchConfiguration('train').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
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

    return [
        LogInfo(msg=f'Environment: {environment}'),
        LogInfo(msg=f'Mode: {mode}'),
        LogInfo(msg=f'Operation: {operation}'),
        LogInfo(msg='Launch configuration validated. Component startup is pending.'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock.',
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
        OpaqueFunction(function=configure),
    ])
