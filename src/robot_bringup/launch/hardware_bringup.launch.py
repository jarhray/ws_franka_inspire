#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: MIT
"""一键启动 RealSense + FR3(executor + state publish) + Inspire(Modbus) 三条硬件链路。

分别 include 上游包内已有 launch（不修改 realsense / inspire 源码）：
  - realsense2_camera/launch/rs_launch.py
  - fr3_franky_executor/fr3_franky_executor_node.py
  - inspire_hand_modbus_ros2/launch/control.launch.py

用法示例：
  ros2 launch robot_bringup hardware_bringup.launch.py \\
    robot_ip:=192.168.1.2 inspire_mode:=2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('robot_bringup')

    default_executor_params = os.path.join(
        bringup_share, 'config', 'fr3_franky_executor_bringup.yaml'
    )
    executor_params_file = LaunchConfiguration('executor_params_file')

    # ----- RealSense (rs_launch.py) -----
    camera_name = LaunchConfiguration('camera_name')
    camera_namespace = LaunchConfiguration('camera_namespace')
    serial_no = LaunchConfiguration('serial_no')

    # ----- FR3 executor (franky) -----
    robot_ip = LaunchConfiguration('robot_ip')
    joint_state_rate = LaunchConfiguration('joint_state_rate')
    fr3_franky_relative_dynamics_factor = LaunchConfiguration('fr3_franky_relative_dynamics_factor')
    fr3_state_bridge_controller_mode = LaunchConfiguration('fr3_state_bridge_controller_mode')
    fr3_state_bridge_joint_topic = LaunchConfiguration('fr3_state_bridge_joint_topic')
    fr3_state_bridge_pose_topic = LaunchConfiguration('fr3_state_bridge_pose_topic')
    fr3_state_bridge_twist_topic = LaunchConfiguration('fr3_state_bridge_twist_topic')

    # ----- Inspire (control.launch.py) -----
    inspire_mode = LaunchConfiguration('inspire_mode')

    realsense_launch = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )
    inspire_launch = PathJoinSubstitution(
        [FindPackageShare('inspire_hand_modbus_ros2'), 'launch', 'control.launch.py']
    )

    return LaunchDescription(
        [
            LogInfo(msg=['[hardware_bringup] Including realsense2_camera, fr3_franky_executor, inspire_hand_modbus_ros2']),
            # --- Declare arguments ---
            DeclareLaunchArgument(
                'executor_params_file',
                default_value=default_executor_params,
                description='YAML file for fr3_franky_executor parameters.',
            ),
            DeclareLaunchArgument(
                'camera_name',
                default_value='camera',
                description='RealSense node name (rs_launch camera_name).',
            ),
            DeclareLaunchArgument(
                'camera_namespace',
                default_value='camera',
                description='RealSense namespace (rs_launch camera_namespace). '
                'camera_name=camera & camera_namespace=camera -> /camera/camera/... topics.',
            ),
            DeclareLaunchArgument(
                'serial_no',
                default_value='_333422302680',
                description="RealSense serial_no filter; default empty ''",
            ),
            DeclareLaunchArgument(
                'robot_ip',
                default_value='192.168.1.2',
                description='FR3 FCI robot IP (used by fr3_franky_executor).',
            ),
            DeclareLaunchArgument(
                'joint_state_rate',
                default_value='30',
                description='FR3 state publish rate (Hz).',
            ),
            DeclareLaunchArgument(
                'fr3_franky_relative_dynamics_factor',
                default_value='0.1',
                description='fr3_franky_executor relative_dynamics_factor (0~1, smaller is safer).',
            ),
            DeclareLaunchArgument(
                'fr3_state_bridge_controller_mode',
                default_value='joint_impedance',
                description='fr3_franky_executor controller_mode: joint_impedance or cartesian_impedance.',
            ),
            DeclareLaunchArgument(
                'fr3_state_bridge_joint_topic',
                default_value='/franka_robot_state_broadcaster/measured_joint_states',
                description='JointState topic published by fr3_franky_executor.',
            ),
            DeclareLaunchArgument(
                'fr3_state_bridge_pose_topic',
                default_value='/franka_robot_state_broadcaster/current_pose',
                description='PoseStamped topic published by fr3_franky_executor.',
            ),
            DeclareLaunchArgument(
                'fr3_state_bridge_twist_topic',
                default_value='/franka_robot_state_broadcaster/desired_end_effector_twist',
                description='TwistStamped topic published by fr3_franky_executor.',
            ),
            DeclareLaunchArgument(
                'inspire_mode',
                default_value='2',
                description='Inspire control.launch mode: 1=control only, 2=control + angle/topic publisher.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([realsense_launch]),
                launch_arguments={
                    'camera_name': camera_name,
                    'camera_namespace': camera_namespace,
                    'serial_no': serial_no,
                }.items(),
            ),
            Node(
                package='fr3_franky_executor',
                executable='fr3_franky_executor_node',
                name='fr3_franky_executor',
                output='screen',
                parameters=[executor_params_file, {'mock_mode': False, 'fci_hostname': robot_ip}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([inspire_launch]),
                launch_arguments={
                    'mode': inspire_mode,
                }.items(),
            ),
        ]
    )
