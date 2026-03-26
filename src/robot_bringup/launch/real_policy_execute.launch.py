#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('robot_bringup')
    default_params = os.path.join(bringup_share, 'config', 'real_policy_execute.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='YAML file for real policy execution pipeline.',
            ),
            Node(
                package='observation_aggregator',
                executable='observation_aggregator_node',
                name='observation_aggregator',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='policy_manager',
                executable='policy_manager_node',
                name='policy_manager',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='action_router',
                executable='action_router_node',
                name='action_router',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='fr3_franky_executor',
                executable='fr3_franky_executor_node',
                name='fr3_franky_executor',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='inspire_executor',
                executable='inspire_executor_node',
                name='inspire_executor',
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
