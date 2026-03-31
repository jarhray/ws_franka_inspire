# robot_bringup

`robot_bringup` 负责统一启动 FR3 + Inspire + RealSense + Policy 链路中本项目新增节点，并集中管理参数文件。

## 提供内容

- `launch/perception_debug.launch.py`
  - 仅启动 `observation_aggregator`
  - 用于观测链路联调（不下发控制动作）
- `launch/hardware_bringup.launch.py`
  - 启动硬件底座链路：
    - `realsense2_camera/rs_launch.py`
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
- FR3 采用单持有者模式：仅 `fr3_franky_executor(mock_mode=false)` 持有 FCI；禁止并行运行其他直连 `franky.Robot` 脚本
- 再启动 `robot_bringup` 的 launch，检查：
  - `/robot/observation`
  - `/robot/whole_body_action`
  - `/robot/arm_action`
  - `/robot/hand_action`
  - `/robot/arm_execution/status`
  - `/robot/arm_execution/error`
