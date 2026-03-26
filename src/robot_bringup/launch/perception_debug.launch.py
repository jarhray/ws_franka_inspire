#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('robot_bringup')
    default_params = os.path.join(bringup_share, 'config', 'perception_debug.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='YAML file for perception debug pipeline.',
            ),
            Node(
                package='observation_aggregator',
                executable='observation_aggregator_node',
                name='observation_aggregator',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='experiment_logger',
                executable='experiment_logger_node',
                name='experiment_logger',
                output='screen',
                parameters=[params_file],
            ),
        ]
    )
