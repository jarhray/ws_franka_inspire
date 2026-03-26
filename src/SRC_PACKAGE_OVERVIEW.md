# ws_franka_inspire/src 包结构与链路说明

本文档用于快速理解 `~/ws_franka_inspire/src` 下各 ROS2 包的职责边界、目录结构和数据链路。

---

## 1. 顶层结构分组

`src` 下当前主要分为两类：

- 项目业务链路包（建议主要在这里扩展）
  - `robot_interfaces`
  - `observation_aggregator`
  - `policy_manager`
  - `action_router`
  - `fr3_franky_executor`
  - `inspire_executor`
  - `experiment_logger`
  - `robot_bringup`
- 上游/第三方栈（建议尽量不改核心结构）
  - `franka_ros2`
  - `inspire_ros2`
  - `realsense-ros-4.55.1`

---

## 2. 业务链路包职责与关键目录

### 2.1 `robot_interfaces`（接口协议层）

职责：定义全链路统一消息与服务接口，供其余包依赖。

关键目录：

- `robot_interfaces/msg/`
  - `RobotObservation.msg`
  - `ArmAction.msg`
  - `HandAction.msg`
  - `WholeBodyAction.msg`
- `robot_interfaces/srv/`
  - `StopArm.srv`
  - `HomeArm.srv`
  - `ResetArmFault.srv`
- `robot_interfaces/CMakeLists.txt`
  - 通过 `rosidl_generate_interfaces(...)` 生成接口代码

---

### 2.2 `observation_aggregator`（观测聚合层）

职责：将 FR3/Inspire/RealSense 多源数据统一汇聚为 `RobotObservation`。

关键目录：

- `observation_aggregator/scripts/observation_aggregator_node.py`（当前主实现）
- `observation_aggregator/README.md`（参数、topic、运行说明）
- `observation_aggregator/src/observation_aggregator_node.cpp`（历史实现保留）

默认核心输出：

- `/robot/observation` (`robot_interfaces/msg/RobotObservation`)

---

### 2.3 `policy_manager`（策略入口层）

职责：根据 `policy_type` 选择策略来源并输出整机动作。

关键目录：

- `policy_manager/policy_manager/policy_manager_node.py`
- `policy_manager/setup.py` / `setup.cfg` / `resource/`（`ament_python`）
- `policy_manager/README.md`

当前策略模式：

- `dummy`：固定动作输出（已实现）
- `bc` / `vla`：接口占位，当前回退到 dummy

默认输出：

- `/robot/whole_body_action` (`robot_interfaces/msg/WholeBodyAction`)

---

### 2.4 `action_router`（动作路由与安全层）

职责：将 `WholeBodyAction` 解释并拆分为 `ArmAction` / `HandAction`，处理相对动作与限幅。

关键目录：

- `action_router/action_router/action_router_node.py`
- `action_router/README.md`

支持机械臂模式：

- `joint_position`
- `joint_velocity`
- `cartesian_pose`
- `cartesian_velocity`

默认输出：

- `/robot/arm_action` (`robot_interfaces/msg/ArmAction`)
- `/robot/hand_action` (`robot_interfaces/msg/HandAction`)

---

### 2.5 `fr3_franky_executor`（机械臂执行层）

职责：作为机械臂执行入口，消费标准化 `ArmAction` 并提供执行状态与控制服务。

关键目录：

- `fr3_franky_executor/fr3_franky_executor/fr3_franky_executor_node.py`
- `fr3_franky_executor/README.md`

默认接口：

- 订阅：`/robot/arm_action`
- 发布：
  - `/robot/arm_execution/status`
  - `/robot/arm_execution/error`
- 服务：
  - `/stop_arm`
  - `/home_arm`
  - `/reset_arm_fault`

备注：当前支持 `mock_mode`，便于先打通链路。

---

### 2.6 `inspire_executor`（灵巧手执行层）

职责：将标准化 `HandAction` 映射为 Inspire 驱动可消费消息。

关键目录：

- `inspire_executor/inspire_executor/inspire_executor_node.py`
- `inspire_executor/README.md`

默认行为：

- 订阅 `/robot/hand_action`
- 发布 `set_angle_data`（`service_interfaces/msg/SetAngle1`）
- 支持 finger 顺序映射、上下限裁剪、保持最后命令重发

---

### 2.7 `experiment_logger`（实验记录层）

职责：记录观测/动作/执行状态/错误，输出 JSONL 以便离线分析与回放。

关键目录：

- `experiment_logger/experiment_logger/experiment_logger_node.py`
- `experiment_logger/README.md`

默认记录文件（按 session 目录分开）：

- `observation.jsonl`
- `arm_action.jsonl`
- `hand_action.jsonl`
- `arm_status.jsonl`
- `arm_error.jsonl`

默认目录：

- `~/.ros/experiment_logs/<timestamp>_<run_name>/`

---

### 2.8 `robot_bringup`（统一启动编排层）

职责：统一管理 launch 与参数文件，组织最小链路或全链路启动。

关键目录：

- `robot_bringup/launch/`
  - `perception_debug.launch.py`
  - `real_policy_execute.launch.py`
- `robot_bringup/config/`
  - `perception_debug.yaml`
  - `real_policy_execute.yaml`
- `robot_bringup/README.md`

当前 launch 编排：

- `perception_debug.launch.py`
  - `observation_aggregator`
  - `experiment_logger`
- `real_policy_execute.launch.py`
  - `observation_aggregator`
  - `policy_manager`
  - `action_router`
  - `fr3_franky_executor`
  - `inspire_executor`
  - `experiment_logger`

---

## 3. Topic 数据流总览

```text
RealSense / FR3 状态 / Inspire 状态
    -> observation_aggregator
    -> /robot/observation (RobotObservation)
    -> policy_manager
    -> /robot/whole_body_action (WholeBodyAction)
    -> action_router
       -> /robot/arm_action (ArmAction) -> fr3_franky_executor
       -> /robot/hand_action (HandAction) -> inspire_executor

fr3_franky_executor
    -> /robot/arm_execution/status (String)
    -> /robot/arm_execution/error  (String)

experiment_logger 订阅：
    /robot/observation
    /robot/arm_action
    /robot/hand_action
    /robot/arm_execution/status
    /robot/arm_execution/error
```

---

## 4. 依赖关系（业务包视角）

核心依赖顺序可理解为：

1. `robot_interfaces`（先编译）
2. `observation_aggregator` / `policy_manager` / `action_router` / `fr3_franky_executor` / `inspire_executor` / `experiment_logger`
3. `robot_bringup`（依赖上述运行包）

说明：

- `robot_bringup` 只做启动编排，不承载业务逻辑。
- 推荐保持“接口定义”与“业务实现”分离，避免把业务逻辑回写进上游包（`franka_ros2` / `realsense-ros-4.55.1`）。

---

## 5. 上游/第三方包定位

- `franka_ros2`：Franka 官方 ROS2 生态（bringup、硬件接口、控制器、状态广播等）
- `inspire_ros2`：Inspire 手驱动与 `service_interfaces`
- `realsense-ros-4.55.1`：RealSense ROS2 驱动及相关消息

这些包主要作为底层设备能力提供者；项目业务建议通过“新增独立包 + topic/service 对接”方式扩展。

