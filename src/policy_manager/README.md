# policy_manager

`policy_manager` 负责根据 `policy_type` 选择策略来源，并统一发布 `WholeBodyAction` 给后续 `action_router`。

当前已支持参数切换：`dummy | act | act_exp1 | act_exp1_1 | act_exp1_2 | act_exp1_3 | act_exp3_0 | act_exp3_0_det | act_exp3_5_det | act_exp3_1 | act_exp3_2 | pi05_exp1_3_lora`（`bc | vla` 回退为 dummy 参数行为）。
其中各模型参数建议放在独立 YAML：

- `config/dummy_policy.yaml`
- `config/act_policy.yaml`（与 `act_eg.py` 对齐的默认 ACT：13 维 `observation.state` + `observation.camera_3.rgb`）
- `config/act_exp1_policy.yaml`（与 `exp1_lerobot/meta/info.json` 对齐：30 维 state + RGB/深度，动作为 13 维关节目标）
- `config/act_exp1_1_policy.yaml`（与 `data_recorded/exp1_1/meta/info.json` 对齐：13 维 state/action，仅 ee_pose+手，RGB/深度）
- `config/act_exp1_2_policy.yaml`（与 `data_recorded/exp1_2/meta/info.json` 对齐：8 维 state/action，ee_pose+二值抓取，RGB）
- `config/act_exp3_0_policy.yaml`（与 `datasets/exp3_0/meta/info.json` 对齐：8 维 state/action=ee_pose(7)+二值抓取，三路 RGB+深度：cam1/cam3/cam4）
- `config/act_exp3_0_det_policy.yaml`（在 `act_exp3_0` 基础上增加 RT-DETRv4 + 深度 + 手眼标定目标位置估计；支持将检测得到的 base x,y 注入 `observation.environment_state`）
- `config/act_exp3_5_det_policy.yaml`（与 `exp3_5` 训练 `config.json` 对齐：8 维 joint state/action + `observation.environment_state`；检测与 `act_exp3_0_det` 相同，机械臂下发 `joint_position`）
- `config/act_exp3_1_policy.yaml`（与 `datasets/exp3_1/meta/info.json` 对齐：8 维 state/action=7关节+二值抓取，三路 RGB+深度：cam1/cam3/cam4）
- `config/act_exp3_2_policy.yaml`（与 `exp3_2` 训练配置对齐：8 维 state + 三路 RGB/深度，动作按**相对 ee_pose(7)+二值抓取**下发）

## ACT（policy_type=act）语义与 `act_eg.py` 对齐

- **观测 `observation.state`（13）**：由 `RobotObservation` 拼接  
  - `[0:7]` = `ee_pose`（`position xyz` + `orientation xyzw`）  
  - `[7:13]` = `hand_joint_position`（6 维，须与训练一致）
- **图像**：默认键 `observation.camera_3.rgb`；从 `RobotObservation` 的 RGB 选取（`act_camera_id` 对应多相机槽位，空则用 legacy `rgb_image`），缩放到 `act_img_h/w`。
- **动作（13）**：  
  - `[0:7]` → `ArmAction.cartesian_pose`（`control_mode=2`，绝对位姿）  
  - `[7:13]` → `HandAction.joint_position`（**保持训练时的归一化 [-1,1]**，不在此节点反归一化）
- **加载失败**：`ACTPolicy.from_pretrained` 失败会直接抛错，**节点无法启动**（无 dummy 回退）。
- **运行时错误**：默认 `act_strict_on_error:=true` —— 观测类问题抛出 `ValueError` 时仅跳过本周期；其它推理异常会 **记录 fatal 并退出进程**。可设为 `false` 改为仅告警跳过。

## 功能概览

- 策略类型切换：
  - `policy_type=dummy`：发布参数指定的固定动作
  - `policy_type=act`：lerobot `ACTPolicy` 实时推理
  - `policy_type=bc` / `vla`：未接模型，使用与 dummy 相同的参数化固定动作
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
  - act: `act_*`、`act_strict_on_error`、`act_camera_id`（多相机时填 `cam1` 等；空则使用 `RobotObservation` 的 legacy `rgb_image`）、`arm_reference_frame`、`arm_duration_sec`、`hand_duration_sec`
  - act_exp1: 同上另有一套 `act_exp1_*`，并包含 RGB（`observation.images.rs_color`）与深度（`observation.rs_depth`）；多相机时用 `act_exp1_camera_id`、`act_exp1_depth_camera_id` 选择 `RobotObservation` 中并行数组槽位
  - act_exp1_1 / act_exp1_3 / pi05_exp1_3_lora：`{prefix}_camera_id` 与 `{prefix}_depth_camera_id`（深度可与 RGB 不同路）
  - act_exp3_0：`act_exp3_0_camera_ids` / `act_exp3_0_depth_camera_ids` 与 `act_exp3_0_image_keys` / `act_exp3_0_depth_keys` 一一对应（默认 cam1/cam3/cam4）
  - act_exp3_0_det：同 `act_exp3_0`，并增加 `act_exp3_0_det_detector_*` 参数（RT-DETRv4 仓库/权重/相机/标定/阈值）
  - act_exp3_5_det：同 `act_exp3_0_det` 检测参数（前缀 `act_exp3_5_det_*`），动作为 7 关节 + 手二值 → `ArmAction.joint_position`
  - act_exp3_1：`act_exp3_1_camera_ids` / `act_exp3_1_depth_camera_ids` 与 `act_exp3_1_image_keys` / `act_exp3_1_depth_keys` 一一对应（默认 cam1/cam3/cam4）
  - act_exp3_2：`act_exp3_2_camera_ids` / `act_exp3_2_depth_camera_ids` 与 `act_exp3_2_image_keys` / `act_exp3_2_depth_keys` 一一对应（默认 cam1/cam3/cam4）
  - act_exp1_2：`act_exp1_2_camera_id`（仅 RGB）

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

请使用工作区内的虚拟环境（与 `torch` / `lerobot` 一致）：

```bash
cd ~/ws_franka_inspire
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select policy_manager
source install/setup.bash
```

说明：`policy_manager_node` 由该 venv 的 Python 解释器运行（通过 `colcon` 安装后的入口脚本），开发时请在同一 venv 下 `colcon build`，避免系统 Python 缺依赖。

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
