#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_policy_type_from_yaml(params_file_path: str) -> str:
    try:
        with open(params_file_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return (
            data.get("policy_manager", {})
            .get("ros__parameters", {})
            .get("policy_type", "dummy")
        )
    except Exception:
        return "dummy"


def _build_nodes(context, bringup_share: str, policy_manager_share: str):
    params_file_path = LaunchConfiguration("params_file").perform(context)
    policy_params_file_arg = LaunchConfiguration("policy_params_file").perform(context)

    # policy_params_file:=auto 时，从 pipeline YAML 的 policy_type 自动选择模型 YAML。
    if policy_params_file_arg == "auto":
        policy_type = _resolve_policy_type_from_yaml(params_file_path)
        policy_params_file_path = os.path.join(
            policy_manager_share, "config", f"{policy_type}_policy.yaml"
        )
    else:
        policy_params_file_path = policy_params_file_arg

    return [
        Node(
            package='observation_aggregator',
            executable='observation_aggregator_node',
            name='observation_aggregator',
            output='screen',
            parameters=[params_file_path],
        ),
        Node(
            package='policy_manager',
            executable='policy_manager_node',
            name='policy_manager',
            output='screen',
            parameters=[params_file_path, policy_params_file_path],
        ),
        Node(
            package='action_router',
            executable='action_router_node',
            name='action_router',
            output='screen',
            parameters=[params_file_path],
        ),
        Node(
            package='inspire_executor',
            executable='inspire_executor_node',
            name='inspire_executor',
            output='screen',
            parameters=[params_file_path],
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
    policy_manager_share = get_package_share_directory('policy_manager')
    default_params = os.path.join(bringup_share, 'config', 'real_policy_execute.yaml')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='YAML file for real policy execution pipeline.',
            ),
            DeclareLaunchArgument(
                'policy_params_file',
                default_value='auto',
                description='Model YAML for policy_manager. Use auto to choose by policy_type in params_file.',
            ),
            OpaqueFunction(
                function=lambda context: _build_nodes(
                    context, bringup_share=bringup_share, policy_manager_share=policy_manager_share
                )
            ),
        ]
    )
