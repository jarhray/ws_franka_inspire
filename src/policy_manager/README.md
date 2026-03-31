# policy_manager

`policy_manager` 负责根据 `policy_type` 选择策略来源，并统一发布 `WholeBodyAction` 给后续 `action_router`。

当前已支持参数切换：`dummy | act`（`bc | vla` 预留）。
其中各模型参数建议放在独立 YAML：

- `config/dummy_policy.yaml`
- `config/act_policy.yaml`

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

## 参数组织方式

- 通用参数（放在 pipeline YAML，如 `robot_bringup/config/real_policy_execute.yaml`）
  - `observation_topic`
  - `whole_body_action_topic`
  - `require_observation_before_publish`
- 模型参数（放在模型 YAML）
  - dummy: `publish_rate_hz`、`arm_*`、`hand_*`、`is_relative`
  - act: `act_*`、`arm_reference_frame`、`arm_duration_sec`、`hand_duration_sec`

## 主要参数

- 策略与节拍
  - `policy_type`：`dummy|act|bc|vla`
  - `publish_rate_hz`（dummy）：动作发布频率
  - `act_control_hz`（act）：ACT 推理频率
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

### 1) 直接运行（默认参数）

```bash
ros2 run policy_manager policy_manager_node
```

### 2) 在 YAML 里改 policy_type（推荐）

在 `robot_bringup/config/real_policy_execute.yaml` 中修改：

```yaml
policy_manager:
  ros__parameters:
    policy_type: act   # 或 dummy
```

`real_policy_execute.launch.py` 默认 `policy_params_file:=auto`，会按该 `policy_type` 自动选择：

- `dummy` -> `policy_manager/config/dummy_policy.yaml`
- `act` -> `policy_manager/config/act_policy.yaml`

### 3) 手动指定模型 YAML（可选）

默认会加载 `policy_manager/config/dummy_policy.yaml`。切到 ACT 示例：

```bash
ros2 launch robot_bringup real_policy_execute.launch.py \
  policy_params_file:=/home/jhr/ws_franka_inspire/src/policy_manager/config/act_policy.yaml
```

### 4) CLI 方式覆盖参数（可选）

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
