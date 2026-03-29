# inspire_executor

将上游 `HandAction.joint_position` 中的关节弧度 **r** 转为 Inspire 硬件指令 **k∈[0,1000]**，并发布 `service_interfaces/SetAngle1`（供 `inspire_hand_modbus_ros2` 的 `set_angle_data` 使用）。

本包**只做 r→k**，不在此做额外裁剪或多种输入模式，避免与其它节点参数约定冲突。

## 标定（与 observation 成对）

正向关系 **r = f(k)**（k 为硬件 0–1000）：

| finger_id | 说明 | r = f(k) |
|-----------|------|----------|
| 0–3 | 四指弯曲 | \(r = -5\times10^{-10} k^3 + 9\times10^{-7} k^2 - 0.0018\,k + 1.4191\) |
| 4 | 拇指弯曲 | \(r = 8\times10^{-11} k^3 - 4\times10^{-8} k^2 - 0.0006\,k + 0.5869\) |
| 5 | 拇指侧摆 | \(r = -0.0012\,k + 1.1641\) |

- **finger_id 0–4**：模块导入时预计算 `f(0)…f(1000)`，对目标 r 在表上做二分，取最近整数 k（约 **log₂(1001)≈10** 次浮点比较/关节，**热路径不计算三次幂**）。
- **finger_id 5**：线性闭式 **k = round((1.1641 − r) / 0.0012)**，再限制到 [0,1000]，**O(1)**。

单条 `HandAction` 约 **6×10** 量级简单运算，远低于典型控制周期预算；若仍要极限优化，可将二分改为从上一帧 k 出发的局部搜索（需状态，当前未实现）。

## 订阅与发布

- 订阅：`hand_action_topic`（默认 `/robot/hand_action`），`robot_interfaces/msg/HandAction`
- 发布：`set_angle_topic`（默认 `set_angle_data`），`service_interfaces/msg/SetAngle1`

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

- `ros2 topic echo /set_angle_data`
- 与 `observation_aggregator` 中实现的 **k→r** 使用同一组 **f(k)**，便于闭环核对
