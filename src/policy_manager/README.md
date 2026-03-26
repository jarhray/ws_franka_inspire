# policy_manager

`policy_manager` 负责根据 `policy_type` 选择策略来源，并统一发布 `WholeBodyAction` 给后续 `action_router`。

当前已支持参数切换：`bc | vla | dummy`。其中 `dummy` 为默认实现，用固定动作持续输出，便于全链路联调。

## 功能概览

- 策略类型切换：
  - `policy_type=dummy`：发布参数指定的固定动作
  - `policy_type=bc`：当前保留接口，暂回退到 dummy 输出
  - `policy_type=vla`：当前保留接口，暂回退到 dummy 输出
- 可选“先等观测再发动作”：
  - `require_observation_before_publish=true` 时，在收到 `RobotObservation` 前不发布动作
- 统一输出：
  - `WholeBodyAction`（默认话题 `/robot/whole_body_action`）

## 订阅与发布

- 订阅
  - `observation_topic`（默认 `/robot/observation`），类型：`robot_interfaces/msg/RobotObservation`
- 发布
  - `whole_body_action_topic`（默认 `/robot/whole_body_action`），类型：`robot_interfaces/msg/WholeBodyAction`

## 主要参数

- 策略与节拍
  - `policy_type`：`dummy|bc|vla`
  - `publish_rate_hz`：动作发布频率
  - `require_observation_before_publish`：是否等待观测
- 输出话题
  - `observation_topic`
  - `whole_body_action_topic`
- dummy 输出动作（可用于链路调试）
  - `arm_control_mode`、`hand_control_mode`
  - `is_relative`
  - `arm_reference_frame`
  - `arm_joint_position`、`arm_joint_velocity`
  - `hand_joint_position`、`hand_joint_velocity`
  - `arm_duration_sec`、`hand_duration_sec`

## 构建

```bash
cd ~/ws_franka_inspire
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select policy_manager
source install/setup.bash
```

## 使用方法

### 1) 启动默认 dummy 策略

```bash
ros2 run policy_manager policy_manager_node
```

### 2) 指定策略类型与动作参数

```bash
ros2 run policy_manager policy_manager_node --ros-args \
  -p policy_type:=dummy \
  -p publish_rate_hz:=10.0 \
  -p observation_topic:=/robot/observation \
  -p whole_body_action_topic:=/robot/whole_body_action \
  -p arm_control_mode:=0 \
  -p hand_control_mode:=0 \
  -p is_relative:=false \
  -p arm_joint_position:="[0.0,-0.6,0.0,-2.0,0.0,1.5,0.7]" \
  -p hand_joint_position:="[0.0,0.0,0.0,0.0,0.0,0.0]"
```

## 联调建议

- 配合 `observation_aggregator` 与 `action_router` 一起启动
- 先 `ros2 topic echo /robot/whole_body_action` 确认策略输出正常
- 再检查 `action_router` 输出是否符合预期（相对/绝对与限幅）
