# action_router

`action_router` 用于将策略层输出的 `robot_interfaces/WholeBodyAction` 进行统一解释和安全处理，然后拆分并发布为标准化 `ArmAction` 与 `HandAction`。

## 功能描述

- 模式解释：支持机械臂 `joint_position / joint_velocity / cartesian_pose / cartesian_velocity`
- 相对/绝对转换：当 `is_relative=true` 时，基于 `/robot/observation` 的当前状态转换为绝对目标
- 安全限幅：
  - 机械臂关节位置上下限裁剪
  - 机械臂关节速度限幅
  - 笛卡尔线速度/角速度幅值限幅
  - 手部关节位置上下限裁剪
- 输出标准动作：发布到 `/robot/arm_action` 与 `/robot/hand_action`

## 订阅与发布

- 订阅
  - `whole_body_action_topic`（默认 `/robot/whole_body_action`），类型：`robot_interfaces/msg/WholeBodyAction`
  - `observation_topic`（默认 `/robot/observation`），类型：`robot_interfaces/msg/RobotObservation`
- 发布
  - `arm_action_topic`（默认 `/robot/arm_action`），类型：`robot_interfaces/msg/ArmAction`
  - `hand_action_topic`（默认 `/robot/hand_action`），类型：`robot_interfaces/msg/HandAction`

## 主要参数

- `whole_body_action_topic`：输入整机动作 topic
- `observation_topic`：观测 topic（相对动作转换依赖）
- `arm_action_topic`：机械臂输出动作 topic
- `hand_action_topic`：灵巧手输出动作 topic
- `arm_joint_count`：机械臂关节数（默认 7）
- `hand_joint_count`：手关节数（默认 6）
- `arm_joint_position_min / arm_joint_position_max`：机械臂关节位置上下限
- `arm_joint_velocity_limit`：机械臂关节速度绝对值上限
- `cartesian_linear_velocity_max / cartesian_angular_velocity_max`：笛卡尔速度幅值上限
- `hand_joint_position_min / hand_joint_position_max`：手关节位置上下限
- `duration_sec_min / duration_sec_max`：执行时长裁剪范围

## 使用方法

### 1) 编译

```bash
cd ~/ws_franka_inspire
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select action_router
source install/setup.bash
```

### 2) 启动节点

```bash
ros2 run action_router action_router_node
```

### 3) 带参数启动示例

```bash
ros2 run action_router action_router_node --ros-args \
  -p whole_body_action_topic:=/robot/whole_body_action \
  -p observation_topic:=/robot/observation \
  -p arm_action_topic:=/robot/arm_action \
  -p hand_action_topic:=/robot/hand_action \
  -p arm_joint_velocity_limit:=1.5 \
  -p cartesian_linear_velocity_max:=0.5 \
  -p cartesian_angular_velocity_max:=0.8
```

## 联调建议

- 启动 `observation_aggregator`，确保 `/robot/observation` 持续发布
- 启动 `action_router` 后，向 `/robot/whole_body_action` 发送测试动作
- 观察 `/robot/arm_action` 与 `/robot/hand_action` 的数值是否符合预期（尤其是相对动作与限幅）

