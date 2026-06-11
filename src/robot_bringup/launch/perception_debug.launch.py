#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_debug_nodes(context, *args, **kwargs):
    bringup_share = get_package_share_directory('robot_bringup')
    default_params = os.path.join(bringup_share, 'config', 'perception_debug.yaml')
    params_file_path = LaunchConfiguration('params_file').perform(context)
    enabled_cameras = LaunchConfiguration('enabled_cameras').perform(context).strip()
    obs_parameters = [params_file_path]
    if enabled_cameras:
        obs_parameters.append({'ros__parameters': {'enabled_cameras': enabled_cameras}})

    return [
        Node(
            package='observation_aggregator',
            executable='observation_aggregator_node',
            name='observation_aggregator',
            output='screen',
            parameters=obs_parameters,
        ),
        Node(
            package='experiment_logger',
            executable='experiment_logger_node',
            name='experiment_logger',
            output='screen',
            parameters=[params_file_path],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('robot_bringup')
    default_params = os.path.join(bringup_share, 'config', 'perception_debug.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='YAML file for perception debug pipeline.',
            ),
            DeclareLaunchArgument(
                'enabled_cameras',
                default_value='',
                description='If non-empty, overrides observation_aggregator enabled_cameras (e.g. cam1,cam3).',
            ),
            OpaqueFunction(function=_build_debug_nodes),
        ]
    )
