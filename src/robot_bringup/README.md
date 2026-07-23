# robot_bringup

`robot_bringup` 负责统一启动 FR3 + Inspire + RealSense + Policy 链路中本项目新增节点，并集中管理参数文件。

## 提供内容

- `launch/perception_debug.launch.py`
  - 仅启动 `observation_aggregator`
  - 用于观测链路联调（不下发控制动作）
- `launch/hardware_bringup.launch.py`
  - 启动硬件底座链路：
    - `realsense2_camera/rs_launch.py`（多路参数见 `config/realsense_cameras.yaml`，由 `enabled_cameras` 选择子集，如 `cam1,cam3`）
    - `fr3_franky_executor`（`mock_mode=false`，并发布 FR3 三路状态）
    - `inspire_hand_modbus_ros2/control.launch.py`
- `launch/real_policy_execute.launch.py`
  - 启动完整链路：
    - `observation_aggregator`
    - `policy_manager`
    - `action_router`
    - `inspire_executor`
  - 机械臂执行依赖已启动的 `hardware_bringup.launch.py`（其中包含 `fr3_franky_executor`）

## 参数文件

- `config/perception_debug.yaml`
  - 仅包含 `observation_aggregator` 相关参数
- `config/real_policy_execute.yaml`
  - 包含完整链路节点参数（policy/router/executors/observation）
- `config/realsense_cameras.yaml`
  - 每路 RealSense 的 `serial_no`、`color_profile` / `depth_profile`、`enable_sync` 等（与 `realsense2_camera` rs_launch 一致）

## 构建

```bash
cd ~/ws_franka_inspire
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_bringup
source install/setup.bash
```

## 使用方法

### 1) 仅观测链路调试

```bash
ros2 launch robot_bringup perception_debug.launch.py
```

### 2) 完整策略执行链路

```bash
# 需先启动 hardware_bringup.launch.py
ros2 launch robot_bringup real_policy_execute.launch.py
```

### 3) 硬件底座一键启动（推荐）

```bash
ros2 launch robot_bringup hardware_bringup.launch.py \
  robot_ip:=192.168.1.2 inspire_mode:=2
```

Inspire 默认使用 `inspire_publish_rate:=10.0`、`inspire_spacename:=inspire`、
`inspire_name:=hand1`，对应状态话题 `/inspire/hand1/angle_data`、
`/inspire/hand1/touch_data` 和控制话题 `/inspire/hand1/set_angle_data`。
如覆盖空间名或手名，需同步修改策略参数文件中的 `hand_angle_topic`、
`hand_touch_topic` 和 `set_angle_topic`。

多相机子集（逻辑名须在 `config/realsense_cameras.yaml` 的 `cameras[].camera_name` 中定义）：

```bash
ros2 launch robot_bringup hardware_bringup.launch.py \
  enabled_cameras:="cam1,cam3" robot_ip:=192.168.1.2
```

自定义相机表：`realsense_cameras_file:=/你的路径/realsense_cameras.yaml`。

对应地，在 `config/real_policy_execute.yaml`（或 `perception_debug.yaml`）里为 `observation_aggregator` 设置相同的 `enabled_cameras`（如 `cam1,cam3`），节点会按 `/camera/{id}/color|depth|camera_info` 自动订阅多路图像；也可用 `ros2 launch ... enabled_cameras:=cam1,cam3` 覆盖 YAML。若修改 `realsense_cameras.yaml` 中的 `camera_namespace`，需同步修改 `realsense_camera_namespace`。策略模型 YAML 中用 `*_camera_id` / `*_depth_camera_id` 选择输入路。

### 4) 指定自定义参数文件

```bash
ros2 launch robot_bringup real_policy_execute.launch.py \
  params_file:=/absolute/path/to/real_policy_execute.yaml
```

## 联调建议

- 先确认底层设备/上游节点已启动：
  - FR3 状态话题可用
  - RealSense 图像话题可用
  - Inspire 驱动节点可用
  - `/inspire/hand1/angle_data` 与 `/inspire/hand1/touch_data` 有数据
- FR3 采用单持有者模式：仅 `fr3_franky_executor(mock_mode=false)` 持有 FCI；禁止并行运行其他直连 `franky.Robot` 脚本
- 再启动 `robot_bringup` 的 launch，检查：
  - `/robot/observation`
  - `/robot/whole_body_action`
  - `/robot/arm_action`
  - `/robot/hand_action`
  - `/robot/arm_execution/status`
  - `/robot/arm_execution/error`
