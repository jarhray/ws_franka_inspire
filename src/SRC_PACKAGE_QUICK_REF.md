# ws_franka_inspire/src 速查表

面向联调时快速查阅：每个业务包的输入、输出、关键参数与常用命令。

---

## 1) `robot_interfaces`（接口定义）

- 输入：无（接口包）
- 输出：`msg/srv` 类型定义供其他包依赖
- 关键文件：
  - `robot_interfaces/msg/RobotObservation.msg`
  - `robot_interfaces/msg/WholeBodyAction.msg`
  - `robot_interfaces/msg/ArmAction.msg`
  - `robot_interfaces/msg/HandAction.msg`
  - `robot_interfaces/srv/StopArm.srv` / `HomeArm.srv` / `ResetArmFault.srv`
- 常用命令：
  - `colcon build --packages-select robot_interfaces`

---

## 2) `observation_aggregator`（观测汇聚）

- 输入（默认）：
  - FR3 关节/位姿/twist 相关 topic（按候选路径订阅）
  - `/angle_data`（Inspire 角度）
  - `/camera/color/image_raw`
  - `/camera/depth/image_rect_raw`
  - `/camera/color/camera_info`
- 输出（默认）：
  - `/robot/observation` (`robot_interfaces/msg/RobotObservation`)
- 关键参数：
  - `observation_topic`
  - `publish_rate_hz`
  - `franka_namespace`
  - `hand_angle_topic`
  - `rs_rgb_image_topic` / `rs_depth_image_topic` / `rs_camera_info_topic`
- 常用命令：
  - `ros2 run observation_aggregator observation_aggregator_node.py`

---

## 3) `policy_manager`（策略输出）

- 输入（默认）：
  - `/robot/observation` (`RobotObservation`)
- 输出（默认）：
  - `/robot/whole_body_action` (`WholeBodyAction`)
- 关键参数：
  - `policy_type` (`dummy|bc|vla`)
  - `publish_rate_hz`
  - `require_observation_before_publish`
  - `arm_*` / `hand_*` dummy 输出参数
- 常用命令：
  - `ros2 run policy_manager policy_manager_node`

---

## 4) `action_router`（路由+安全）

- 输入（默认）：
  - `/robot/whole_body_action` (`WholeBodyAction`)
  - `/robot/observation` (`RobotObservation`)
- 输出（默认）：
  - `/robot/arm_action` (`ArmAction`)
  - `/robot/hand_action` (`HandAction`)
- 关键参数：
  - `arm_joint_count` / `hand_joint_count`
  - `arm_joint_position_min/max`
  - `arm_joint_velocity_limit`
  - `cartesian_linear_velocity_max`
  - `cartesian_angular_velocity_max`
  - `hand_joint_position_min/max`
- 常用命令：
  - `ros2 run action_router action_router_node`

---

## 5) `fr3_franky_executor`（机械臂执行）

- 输入（默认）：
  - `/robot/arm_action` (`ArmAction`)
- 输出（默认）：
  - `/robot/arm_execution/status` (`std_msgs/String`)
  - `/robot/arm_execution/error` (`std_msgs/String`)
- 服务：
  - `/stop_arm`
  - `/home_arm`
  - `/reset_arm_fault`
- 关键参数：
  - `mock_mode`
  - `status_rate_hz`
  - `command_timeout_sec`
  - `default_reference_frame`
- 常用命令：
  - `ros2 run fr3_franky_executor fr3_franky_executor_node`

---

## 6) `inspire_executor`（手部执行）

- 输入（默认）：
  - `/robot/hand_action` (`HandAction`)
- 输出（默认）：
  - `set_angle_data` (`service_interfaces/msg/SetAngle1`)
- 关键参数：
  - `set_angle_topic`
  - `command_rate_hz`
  - `hold_last_command`
  - `hand_finger_id_order`
  - `joint_position_min/max`
- 常用命令：
  - `ros2 run inspire_executor inspire_executor_node`

---

## 7) `experiment_logger`（实验日志）

- 输入（默认）：
  - `/robot/observation`
  - `/robot/arm_action`
  - `/robot/hand_action`
  - `/robot/arm_execution/status`
  - `/robot/arm_execution/error`
- 输出：
  - `~/.ros/experiment_logs/<session>/observation.jsonl`
  - `~/.ros/experiment_logs/<session>/arm_action.jsonl`
  - `~/.ros/experiment_logs/<session>/hand_action.jsonl`
  - `~/.ros/experiment_logs/<session>/arm_status.jsonl`
  - `~/.ros/experiment_logs/<session>/arm_error.jsonl`
- 关键参数：
  - `enabled`
  - `log_dir`
  - `run_name`
  - `flush_every_n`
- 常用命令：
  - `ros2 run experiment_logger experiment_logger_node`

---

## 8) `robot_bringup`（统一启动）

- 输入：`config/*.yaml` 参数文件
- 输出：按 launch 拉起节点
- launch：
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
- 常用命令：
  - `ros2 launch robot_bringup perception_debug.launch.py`
  - `ros2 launch robot_bringup real_policy_execute.launch.py`

---

## 9) 最小链路排查顺序（建议）

- 先看观测：`/robot/observation`
- 再看策略输出：`/robot/whole_body_action`
- 再看路由输出：`/robot/arm_action` + `/robot/hand_action`
- 再看执行反馈：`/robot/arm_execution/status` + `/robot/arm_execution/error`
- 最后确认日志：`~/.ros/experiment_logs/<session>/*.jsonl`

