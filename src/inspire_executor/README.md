# inspire_executor

将上游 `HandAction.joint_position` 中归一化关节值 **x∈[-1,1]** 线性映射为 Inspire 硬件指令 **k∈[0,1000]**，并发布 `service_interfaces/SetAngle1`（供 `inspire_hand_modbus_ros2` 的 `/inspire/hand1/set_angle_data` 使用）。

本包**只做 x→k 的线性映射**，不在此做额外裁剪或多种输入模式，避免与其它节点参数约定冲突。

## 映射规则

采用固定线性映射（逐关节相同）：

- 输入：`x ∈ [-1, 1]`
- 输出：`k = round((x + 1) * 500)`，并限制到 `[0, 1000]`

单关节时间复杂度 **O(1)**，单条 `HandAction`（6 关节）为常数级开销。

## 订阅与发布

- 订阅：`hand_action_topic`（默认 `/robot/hand_action`），`robot_interfaces/msg/HandAction`
- 发布：`set_angle_topic`（默认 `/inspire/hand1/set_angle_data`），`service_interfaces/msg/SetAngle1`

## 参数（仅保留与其它包配合所需）

- `hand_action_topic`
- `set_angle_topic`
- `command_rate_hz` + `hold_last_command`（定时重发上一帧，行为与先前一致）
- `hand_finger_id_order`：`joint_position[i]` 对应的手指 ID 顺序（默认 `[0,1,2,3,4,5]`）

## 编译与运行

```bash
cd ~/ws_franka_inspire
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select inspire_executor
source install/setup.bash
ros2 run inspire_executor inspire_executor_node
```

## 联调

- `ros2 topic echo /inspire/hand1/set_angle_data`
- 检查模型输出是否稳定在 `[-1,1]`，超界值会被饱和到边界
