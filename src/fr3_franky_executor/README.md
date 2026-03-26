# fr3_franky_executor

`fr3_franky_executor` 是机械臂执行层框架节点：接收标准化 `ArmAction`，按控制模式分发执行，并提供 stop/home/reset fault 服务与状态/错误发布。

当前版本已实现**完整接口框架**，默认使用 `mock_mode=true` 便于先打通系统链路；后续可替换为真实 Franky/Franka 后端调用。

## 功能概览

- 支持 `ArmAction.control_mode`：
  - `0`: `joint_position`
  - `1`: `joint_velocity`
  - `2`: `cartesian_pose`
  - `3`: `cartesian_velocity`
- 提供服务：
  - `stop_arm`（`robot_interfaces/srv/StopArm`）
  - `home_arm`（`robot_interfaces/srv/HomeArm`）
  - `reset_arm_fault`（`robot_interfaces/srv/ResetArmFault`）
- 发布执行状态与错误：
  - 状态 topic（默认 `/robot/arm_execution/status`）
  - 错误 topic（默认 `/robot/arm_execution/error`）
- 命令超时监控：
  - 超过 `command_timeout_sec` 未收到新指令时发布错误

## 订阅 / 发布 / 服务

- 订阅
  - `arm_action_topic`（默认 `/robot/arm_action`），类型：`robot_interfaces/msg/ArmAction`
- 发布
  - `status_topic`（默认 `/robot/arm_execution/status`），类型：`std_msgs/msg/String`
  - `error_topic`（默认 `/robot/arm_execution/error`），类型：`std_msgs/msg/String`
- 服务
  - `/stop_arm`
  - `/home_arm`
  - `/reset_arm_fault`

## 主要参数

- `arm_action_topic`：输入机械臂动作
- `status_topic`：状态输出
- `error_topic`：错误输出
- `mock_mode`：是否启用 mock 后端（默认 `true`）
- `status_rate_hz`：状态心跳发布频率
- `command_timeout_sec`：指令超时阈值
- `default_reference_frame`：当输入 `ArmAction.reference_frame` 为空时的默认参考系

## 构建

```bash
cd ~/ws_franka_inspire
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_franky_executor
source install/setup.bash
```

## 使用方法

### 1) 启动节点（默认 mock）

```bash
ros2 run fr3_franky_executor fr3_franky_executor_node
```

### 2) 参数化启动示例

```bash
ros2 run fr3_franky_executor fr3_franky_executor_node --ros-args \
  -p arm_action_topic:=/robot/arm_action \
  -p status_topic:=/robot/arm_execution/status \
  -p error_topic:=/robot/arm_execution/error \
  -p mock_mode:=true \
  -p command_timeout_sec:=2.0
```

### 3) 服务调用示例

```bash
ros2 service call /stop_arm robot_interfaces/srv/StopArm "{immediate: true}"
ros2 service call /home_arm robot_interfaces/srv/HomeArm "{wait: false, timeout_sec: 5.0}"
ros2 service call /reset_arm_fault robot_interfaces/srv/ResetArmFault "{hard_reset: false, timeout_sec: 3.0}"
```

## 对接真实后端建议

- 在 `fr3_franky_executor_node.py` 中保留 `FrankyBackend` 接口不变
- 将 `execute_arm_action/stop/home/reset_fault` 的 mock 实现替换为真实 SDK/库调用
- 保持 topic/service 语义不变，可最小化上层改动
