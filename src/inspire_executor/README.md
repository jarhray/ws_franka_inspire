# inspire_executor

`inspire_executor` 将标准化的 `robot_interfaces/HandAction` 转换为 `inspire_hand_modbus_ros2` 可直接消费的 `service_interfaces/SetAngle1` 指令，并支持参数化映射、关节裁剪与发送频率控制。

## 功能描述

- 接收标准手动作：`HandAction`
- 关节映射：按 `hand_finger_id_order` 生成 `SetAngle1.finger_ids`
- 上下限裁剪：按每个关节的 `joint_position_min/max` 裁剪目标值
- 频率控制：按 `command_rate_hz` 定时发送（可保持最后一帧指令）
- 可选输入缩放：支持将 `[0,1]` 归一化输入缩放到 `[0,1000]`

## 订阅与发布

- 订阅
  - `hand_action_topic`（默认 `/robot/hand_action`），类型：`robot_interfaces/msg/HandAction`
- 发布
  - `set_angle_topic`（默认 `set_angle_data`），类型：`service_interfaces/msg/SetAngle1`

> 说明：`inspire_hand_modbus_ros2` 中 `inspire_hand_modbus_topic.py` 默认订阅 `set_angle_data`。

## 主要参数

- `hand_action_topic`：输入手动作 topic
- `set_angle_topic`：输出角度控制 topic（通常保持 `set_angle_data`）
- `hand_finger_id_order`：关节到手指 ID 的映射顺序（默认 `[0,1,2,3,4,5]`）
- `joint_position_min / joint_position_max`：每个关节的上下限
- `command_rate_hz`：发布频率（Hz）
- `hold_last_command`：
  - `true`：收到一次动作后持续按频率重发最后一帧
  - `false`：仅在收到新动作时发布一次
- `input_normalized_0_1`：
  - `true`：输入范围按 `[0,1]` 解释并乘以 1000
  - `false`：输入按原值（通常 0~1000）使用

## 使用方法

### 1) 编译

```bash
cd ~/ws_franka_inspire
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select inspire_executor
source install/setup.bash
```

### 2) 启动依赖节点（Inspire 驱动）

确保 `inspire_hand_modbus_ros2` 已启动且在运行 `inspire_hand_modbus_topic.py`（订阅 `set_angle_data`）。

### 3) 启动执行器

```bash
ros2 run inspire_executor inspire_executor_node
```

### 4) 参数化启动示例

```bash
ros2 run inspire_executor inspire_executor_node --ros-args \
  -p hand_action_topic:=/robot/hand_action \
  -p set_angle_topic:=set_angle_data \
  -p command_rate_hz:=20.0 \
  -p hold_last_command:=true \
  -p hand_finger_id_order:="[0,1,2,3,4,5]" \
  -p joint_position_min:="[0.0,0.0,0.0,0.0,0.0,0.0]" \
  -p joint_position_max:="[1000.0,1000.0,1000.0,1000.0,1000.0,1000.0]"
```

## 联调建议

- 先确认 `set_angle_data` 上有消息：
  - `ros2 topic echo /set_angle_data`
- 再确认手部反馈（如 `/angle_data`）是否按预期变化
- 初次联调建议低频、低幅度动作，逐步放开参数

