# policy_manager

`policy_manager` 根据 `policy_type` 创建策略，将 `RobotObservation` 转换为统一的
`WholeBodyAction`，并发布给后续的 `action_router`。

## 支持的策略

目前支持以下策略：

- `dummy`：发布配置中指定的固定动作，用于链路调试。
- `act_dex_0`：鸡蛋抓取 ACT，输入三路 RGB、13 维本体状态和 `2×5` 触觉。
- `act_exp3_0`：加载 LeRobot ACT checkpoint，使用三路 RGB/深度和 8 维状态进行实时推理。

对应配置文件：

- `config/dummy_policy.yaml`
- `config/act_dex_0_policy.yaml`
- `config/act_exp3_0_policy.yaml`

其他历史实验策略及其配置文件已移除；传入未注册的 `policy_type` 时，节点会列出当前可用类型并启动失败。

## act_dex_0 数据约定

- checkpoint：`dex_tac/outputs/act_egg_grasping_tac_200k/checkpoints/200000/pretrained_model`。
- `observation.state`：13 维，内容为 `ee_pose(7) + hand_hardware_angle(6)`；手部状态直接读取 `RobotObservation.hand_joint_hardware_position` 中的原始硬件角度 `k`，不经过弧度换算。
- `observation.tactile`：`2×5`，第一行为 5 指法向力，第二行为 5 指切向力，按 `finger_id=0..4` 排列。
- `action`：13 维，内容为绝对 `ee_pose(7) + hand_target_hardware(6)`；手部硬件目标会转换为执行链使用的 `[-1,1]`。
- `cam1` / `cam3`：在线图像直接缩放到 `240×320`。
- `cam4`：保持宽高比缩放到宽 320（原始 `240×424` 时高度约 181），然后用黑边上下居中补到 `240×320`。

运行该策略：

```bash
ros2 run policy_manager policy_manager_node --ros-args \
  --params-file src/policy_manager/config/act_dex_0_policy.yaml
```

完整 pipeline 中，将 `robot_bringup/config/real_policy_execute.yaml` 的
`policy_type` 改为 `act_dex_0`，`policy_params_file:=auto` 会自动加载对应配置。

## 输入与输出

- 订阅 `observation_topic`，默认 `/robot/observation`，消息类型为
  `robot_interfaces/msg/RobotObservation`。
- 发布 `whole_body_action_topic`，默认 `/robot/whole_body_action`，消息类型为
  `robot_interfaces/msg/WholeBodyAction`。
- `require_observation_before_publish=true` 时，收到第一条观测前不会发布动作。

## act_exp3_0 数据约定

- `observation.state`：8 维，内容为 `ee_pose(7) + hand_grasp_binary(1)`。
- `action`：8 维，内容为 `ee_pose(7) + hand_grasp_binary(1)`。
- 图像：默认使用 `cam1`、`cam3`、`cam4` 三路 RGB 和深度。
- 输出：机械臂使用绝对笛卡尔位姿，手部根据阈值输出 reset 或 grasp 模板。

模型参数集中在 `config/act_exp3_0_policy.yaml`：

- `act_exp3_0_checkpoint_dir`：LeRobot `pretrained_model` 目录。
- `act_exp3_0_device`：`auto`、`cpu` 或 `cuda`。
- `act_exp3_0_camera_ids` / `act_exp3_0_image_keys`：RGB 相机与模型输入键，数量必须一致。
- `act_exp3_0_depth_camera_ids` / `act_exp3_0_depth_keys`：深度相机与模型输入键，启用深度时数量必须与 RGB 一致。
- `act_exp3_0_control_hz`：策略推理频率。
- `act_exp3_0_state_dim` / `act_exp3_0_action_dim`：默认均为 8。

使用前请确认 checkpoint 路径、相机 ID、图像键和训练数据保持一致。

## 构建

使用工作区内包含 `torch` 和 `lerobot` 依赖的虚拟环境：

```bash
cd ~/ws_franka_inspire
source .venv/bin/activate
source /opt/ros/humble/setup.bash
.venv/bin/python -m colcon build --packages-select policy_manager
source install/setup.bash
```

必须通过 `.venv` 的 Python 启动 `colcon`，否则生成的 ROS 入口脚本可能使用系统
Python，运行模型时会找不到 `torch` / `lerobot`。

## 运行

直接运行节点时默认使用 `dummy`：

```bash
ros2 run policy_manager policy_manager_node --ros-args \
  --params-file src/policy_manager/config/dummy_policy.yaml
```

运行 `act_exp3_0`：

```bash
ros2 run policy_manager policy_manager_node --ros-args \
  --params-file src/policy_manager/config/act_exp3_0_policy.yaml
```

运行完整真实机器人 pipeline：

```bash
ros2 launch robot_bringup real_policy_execute.launch.py
```

`real_policy_execute.yaml` 默认选择 `act_exp3_0`。当
`policy_params_file:=auto` 时，launch 文件会根据 `policy_type` 自动加载
`policy_manager/config/{policy_type}_policy.yaml`。切换到 `dummy` 时，只需将 pipeline
配置中的 `policy_type` 改为 `dummy`。

也可以手动指定策略配置：

```bash
ros2 launch robot_bringup real_policy_execute.launch.py \
  policy_params_file:=/home/jhr/ws_franka_inspire/src/policy_manager/config/act_exp3_0_policy.yaml
```

## 联调建议

- 确认 `observation_aggregator` 发布的相机顺序和配置中的 camera ID 一致。
- 先检查 `/robot/observation` 是否完整，再观察 `/robot/whole_body_action`。
- 最后检查 `action_router` 输出的机械臂和手部动作是否符合预期。
