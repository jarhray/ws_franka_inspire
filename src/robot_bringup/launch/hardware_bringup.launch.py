#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: MIT
"""一键启动 RealSense + FR3(ros2_control) + Inspire(Modbus) 三条硬件链路。

分别 include 上游包内已有 launch（不修改 franka / realsense / inspire 源码）：
  - realsense2_camera/launch/rs_launch.py
  - franka_bringup/launch/franka.launch.py
  - inspire_hand_modbus_ros2/launch/control.launch.py

用法示例：
  ros2 launch robot_bringup hardware_bringup.launch.py \\
    robot_type:=fr3 use_fake_hardware:=true inspire_mode:=2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ----- RealSense (rs_launch.py) -----
    camera_name = LaunchConfiguration('camera_name')
    camera_namespace = LaunchConfiguration('camera_namespace')
    serial_no = LaunchConfiguration('serial_no')

    # ----- Franka (franka.launch.py) -----
    robot_type = LaunchConfiguration('robot_type')
    arm_prefix = LaunchConfiguration('arm_prefix')
    franka_namespace = LaunchConfiguration('franka_namespace')
    robot_ip = LaunchConfiguration('robot_ip')
    load_gripper = LaunchConfiguration('load_gripper')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    joint_state_rate = LaunchConfiguration('joint_state_rate')
    controllers_yaml = LaunchConfiguration('controllers_yaml')

    # ----- Inspire (control.launch.py) -----
    inspire_mode = LaunchConfiguration('inspire_mode')

    realsense_launch = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )
    franka_launch = PathJoinSubstitution(
        [FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py']
    )
    inspire_launch = PathJoinSubstitution(
        [FindPackageShare('inspire_hand_modbus_ros2'), 'launch', 'control.launch.py']
    )

    return LaunchDescription(
        [
            LogInfo(msg=['[hardware_bringup] Including realsense2_camera, franka_bringup, inspire_hand_modbus_ros2']),
            # --- Declare arguments (forwarded to child launches) ---
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
                default_value="''",
                description="RealSense serial_no filter; default empty ''",
            ),
            DeclareLaunchArgument(
                'robot_type',
                default_value='fr3',
                description='Franka robot_type xacro id (e.g. fr3).',
            ),
            DeclareLaunchArgument(
                'arm_prefix',
                default_value='',
                description='Franka arm_prefix.',
            ),
            DeclareLaunchArgument(
                'franka_namespace',
                default_value='',
                description='Franka ros2_control / joint_state namespace (franka.launch namespace).',
            ),
            DeclareLaunchArgument(
                'robot_ip',
                default_value='172.16.0.3',
                description='Franka robot IP.',
            ),
            DeclareLaunchArgument(
                'load_gripper',
                default_value='false',
                description='Franka load_gripper.',
            ),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='false',
                description='Franka use_fake_hardware.',
            ),
            DeclareLaunchArgument(
                'fake_sensor_commands',
                default_value='false',
                description='Franka fake_sensor_commands.',
            ),
            DeclareLaunchArgument(
                'joint_state_rate',
                default_value='30',
                description='Franka joint_state_publisher rate (Hz).',
            ),
            DeclareLaunchArgument(
                'controllers_yaml',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('franka_bringup'), 'config', 'controllers.yaml']
                ),
                description='Franka controllers yaml path.',
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([franka_launch]),
                launch_arguments={
                    'robot_type': robot_type,
                    'arm_prefix': arm_prefix,
                    'namespace': franka_namespace,
                    'robot_ip': robot_ip,
                    'load_gripper': load_gripper,
                    'use_fake_hardware': use_fake_hardware,
                    'fake_sensor_commands': fake_sensor_commands,
                    'joint_state_rate': joint_state_rate,
                    'controllers_yaml': controllers_yaml,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([inspire_launch]),
                launch_arguments={
                    'mode': inspire_mode,
                }.items(),
            ),
        ]
    )
