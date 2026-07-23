#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: MIT
"""一键启动 RealSense（可选多路）+ FR3(executor + state publish) + Inspire(Modbus) 三条硬件链路。

分别 include 上游包内已有 launch（不修改 realsense / inspire 源码）：
  - realsense2_camera/launch/rs_launch.py（每个相机独立 include 一次）
  - fr3_franky_executor/fr3_franky_executor_node.py
  - inspire_hand_modbus_ros2/launch/control.launch.py

RealSense 每路分辨率、同步等参数见包内 config/realsense_cameras.yaml，可用 launch 参数
``realsense_cameras_file`` 指向自定义副本。

用法示例：
  ros2 launch robot_bringup hardware_bringup.launch.py \\
    robot_ip:=192.168.1.2 inspire_mode:=2

  # 多相机子集（逻辑名须在 realsense_cameras.yaml 的 cameras[].camera_name 中）
  ros2 launch robot_bringup hardware_bringup.launch.py \\
    enabled_cameras:="cam1,cam3" robot_ip:=192.168.1.2
"""

import os
from typing import Any, Dict, Mapping, MutableMapping

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# rs_launch.py 中 DeclareLaunchArgument 的 name 集合；仅这些键可从 YAML 透传到 include（避免无效键报错）。
_RS_LAUNCH_ARG_NAMES = frozenset(
    {
        "camera_name",
        "camera_namespace",
        "serial_no",
        "usb_port_id",
        "device_type",
        "config_file",
        "json_file_path",
        "initial_reset",
        "accelerate_gpu_with_glsl",
        "rosbag_filename",
        "log_level",
        "output",
        "enable_color",
        "rgb_camera.color_profile",
        "rgb_camera.color_format",
        "rgb_camera.enable_auto_exposure",
        "enable_depth",
        "enable_infra",
        "enable_infra1",
        "enable_infra2",
        "depth_module.depth_profile",
        "depth_module.depth_format",
        "depth_module.infra_profile",
        "depth_module.infra_format",
        "depth_module.infra1_format",
        "depth_module.infra2_format",
        "depth_module.exposure",
        "depth_module.gain",
        "depth_module.hdr_enabled",
        "depth_module.enable_auto_exposure",
        "depth_module.exposure.1",
        "depth_module.gain.1",
        "depth_module.exposure.2",
        "depth_module.gain.2",
        "enable_sync",
        "enable_rgbd",
        "enable_gyro",
        "enable_accel",
        "gyro_fps",
        "accel_fps",
        "unite_imu_method",
        "clip_distance",
        "angular_velocity_cov",
        "linear_accel_cov",
        "diagnostics_period",
        "publish_tf",
        "tf_publish_rate",
        "pointcloud.enable",
        "pointcloud.stream_filter",
        "pointcloud.stream_index_filter",
        "pointcloud.ordered_pc",
        "pointcloud.allow_no_texture_points",
        "align_depth.enable",
        "colorizer.enable",
        "decimation_filter.enable",
        "spatial_filter.enable",
        "temporal_filter.enable",
        "disparity_filter.enable",
        "hole_filling_filter.enable",
        "hdr_merge.enable",
        "wait_for_device_timeout",
        "reconnect_timeout",
    }
)

# YAML 简写 -> rs_launch 参数名
_RS_YAML_ALIASES = {
    "color_profile": "rgb_camera.color_profile",
    "depth_profile": "depth_module.depth_profile",
}

# 不从「通用键值」循环写入 launch 的元数据键（camera_name / serial_no 单独处理）
_RS_YAML_SKIP_KEYS = frozenset({"camera_name", "serial_no", "camera_namespace"})


def _yaml_scalar_to_launch_str(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return str(value)
    return str(value)


def _load_cameras_yaml(path: str) -> Dict[str, Dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    rows = data.get("cameras") or []
    table: Dict[str, Dict[str, Any]] = {}
    for row in rows:
        if not isinstance(row, dict):
            continue
        name = row.get("camera_name")
        if not name:
            continue
        table[str(name)] = dict(row)
    return table


def _row_to_rs_launch_args(
    row: Mapping[str, Any],
    *,
    log_level: str,
    warn_state: MutableMapping[str, bool],
) -> Dict[str, str]:
    """单路相机：YAML 条目 -> rs_launch launch_arguments（log_level 由 launch 统一覆盖）。"""
    cid = str(row["camera_name"])
    serial = row.get("serial_no")
    if serial is None or str(serial).strip() == "":
        raise RuntimeError(f"Camera {cid!r}: missing serial_no in realsense_cameras yaml")

    ns = str(row.get("camera_namespace", "camera")).strip()

    out: Dict[str, str] = {
        "camera_name": cid,
        "camera_namespace": ns,
        "serial_no": str(serial),
    }

    for key, value in row.items():
        if key in _RS_YAML_SKIP_KEYS:
            continue
        if value is None:
            continue
        launch_key = _RS_YAML_ALIASES.get(str(key), str(key))
        if launch_key in ("camera_name", "camera_namespace", "serial_no"):
            continue
        if launch_key not in _RS_LAUNCH_ARG_NAMES:
            if not warn_state.get("unknown"):
                print(
                    "[hardware_bringup][WARN] Ignoring unknown RealSense yaml key(s) not in rs_launch "
                    f"(first hit: {key!r}). Use rs_launch argument names or aliases color_profile / depth_profile."
                )
                warn_state["unknown"] = True
            continue
        out[launch_key] = _yaml_scalar_to_launch_str(value)

    out.setdefault("enable_color", "true")
    out.setdefault("enable_depth", "true")
    out["log_level"] = log_level
    return out


def _launch_realsense_cameras(context, *args, **kwargs):
    enabled_raw = LaunchConfiguration("enabled_cameras").perform(context)
    cam_ids = [x.strip() for x in enabled_raw.split(",") if x.strip()]
    if not cam_ids:
        raise RuntimeError("enabled_cameras is empty; specify e.g. cam1 or cam1,cam3")

    log_level = LaunchConfiguration("realsense_log_level").perform(context)

    cam_yaml_path = LaunchConfiguration("realsense_cameras_file").perform(context)
    if not os.path.isfile(cam_yaml_path):
        raise RuntimeError(f"realsense_cameras_file not found or not a file: {cam_yaml_path!r}")

    warn_state: Dict[str, bool] = {"unknown": False}
    camera_table = _load_cameras_yaml(cam_yaml_path)

    rs_share = get_package_share_directory("realsense2_camera")
    rs_launch = os.path.join(rs_share, "launch", "rs_launch.py")

    actions = []
    for cid in cam_ids:
        if cid not in camera_table:
            known = ", ".join(sorted(camera_table.keys()))
            raise RuntimeError(
                f"Unknown camera id {cid!r} (not in {cam_yaml_path}). "
                f"Known camera_name values: {known or '(none)'}"
            )
        row = camera_table[cid]
        if str(row.get("camera_name", cid)) != cid:
            raise RuntimeError(
                f"Camera table key {cid!r} must match row camera_name={row.get('camera_name')!r}"
            )
        launch_args = _row_to_rs_launch_args(row, log_level=log_level, warn_state=warn_state)
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([rs_launch]),
                launch_arguments=launch_args.items(),
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("robot_bringup")

    default_executor_params = os.path.join(
        bringup_share, "config", "fr3_franky_executor_bringup.yaml"
    )
    default_realsense_cameras = os.path.join(bringup_share, "config", "realsense_cameras.yaml")
    executor_params_file = LaunchConfiguration("executor_params_file")

    # ----- FR3 executor (franky) -----
    robot_ip = LaunchConfiguration("robot_ip")
    inspire_mode = LaunchConfiguration("inspire_mode")
    inspire_publish_rate = LaunchConfiguration("inspire_publish_rate")
    inspire_spacename = LaunchConfiguration("inspire_spacename")
    inspire_name = LaunchConfiguration("inspire_name")

    inspire_launch = os.path.join(
        get_package_share_directory("inspire_hand_modbus_ros2"),
        "launch",
        "control.launch.py",
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=[
                    "[hardware_bringup] RealSense (multi) + fr3_franky_executor + inspire_hand_modbus_ros2"
                ]
            ),
            DeclareLaunchArgument(
                "executor_params_file",
                default_value=default_executor_params,
                description="YAML file for fr3_franky_executor parameters.",
            ),
            DeclareLaunchArgument(
                "realsense_cameras_file",
                default_value=default_realsense_cameras,
                description="YAML listing all RealSense cameras (serial, profiles, enable_sync, ...).",
            ),
            DeclareLaunchArgument(
                "enabled_cameras",
                default_value="cam1",
                description="Comma-separated logical camera_name values defined in realsense_cameras_file.",
            ),
            DeclareLaunchArgument(
                "realsense_log_level",
                default_value="info",
                description="realsense2_camera_node --log-level for every started RealSense (e.g. info, debug).",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.1.2",
                description="FR3 FCI robot IP (used by fr3_franky_executor).",
            ),
            DeclareLaunchArgument(
                "inspire_mode",
                default_value="2",
                description="Inspire control.launch mode: 1=control only, 2=control + angle/topic publisher.",
            ),
            DeclareLaunchArgument(
                "inspire_publish_rate",
                default_value="10.0",
                description="Inspire state topic publish rate in Hz.",
            ),
            DeclareLaunchArgument(
                "inspire_spacename",
                default_value="inspire",
                description="Inspire topic space prefix; use an empty value to disable it.",
            ),
            DeclareLaunchArgument(
                "inspire_name",
                default_value="hand1",
                description="Inspire hand name prefix; use an empty value to disable it.",
            ),
            OpaqueFunction(function=_launch_realsense_cameras),
            Node(
                package="fr3_franky_executor",
                executable="fr3_franky_executor_node",
                name="fr3_franky_executor",
                output="screen",
                parameters=[executor_params_file, {"mock_mode": False, "fci_hostname": robot_ip}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([inspire_launch]),
                launch_arguments={
                    "mode": inspire_mode,
                    "publish_rate": inspire_publish_rate,
                    "spacename": inspire_spacename,
                    "name": inspire_name,
                }.items(),
            ),
        ]
    )
