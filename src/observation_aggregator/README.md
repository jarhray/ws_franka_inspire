# observation_aggregator

将 **FR3（Franka）**、**Inspire 灵巧手** 与 **RealSense** 的原始话题汇聚为一条统一的 `robot_interfaces/msg/RobotObservation` 消息，供策略层或记录模块消费。当前实现为 **“最新值汇总 + 固定频率发布”**，时间戳取各子源中较新的时间（为后续精确时间同步预留扩展空间）。

## 功能概览

| 来源 | 订阅内容 | 写入 `RobotObservation` 字段 |
|------|----------|------------------------------|
| FR3 | `sensor_msgs/JointState`（测量关节状态） | `arm_joint_position`、`arm_joint_velocity` |
| FR3 | `geometry_msgs/PoseStamped`（末端位姿） | `ee_pose`、`ee_pose_frame` |
| FR3 | `geometry_msgs/TwistStamped`（末端 twist，来自 Franka 状态广播中的期望项） | `ee_twist` |
| Inspire | `service_interfaces/msg/GetAngleAct1`（如 `inspire_hand_modbus_topic` 发布的 `angle_data`） | `hand_joint_position`；`hand_joint_velocity` 由相邻消息差分估计 |
| RealSense | `sensor_msgs/Image` / `CameraInfo` | `rgb_image`、`depth_image`、`camera_info` |

## 依赖

工作区内需已编译并可被 `source` 找到：

- `robot_interfaces`（`RobotObservation` 等）
- `service_interfaces`（`GetAngleAct1`）
- ROS 2 Humble 常见消息包：`sensor_msgs`、`geometry_msgs`、`std_msgs`、`builtin_interfaces`

## 构建

在工作区根目录（例如 `~/ws_franka_inspire`）：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select observation_aggregator
source install/setup.bash
```

若使用本工作区虚拟环境，可先 `source .venv/bin/activate` 再执行上述命令。

## 运行

```bash
ros2 run observation_aggregator observation_aggregator_node.py
```

典型组合：先启动 Franka 侧（例如 `franka_bringup` 的 `franka.launch.py`，仅拉起 `ros2_control` 与状态广播器），再启动 RealSense 与 Inspire 相关节点，最后启动本节点。

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/robot/observation`（默认，可改） | `robot_interfaces/msg/RobotObservation` | 统一观测 |

## 订阅话题（默认与说明）

### FR3

节点会按**候选列表顺序**同时订阅多路可能存在的 topic（避免 `~/` 私有名在不同节点名下展开不一致导致订阅失败）。默认候选形如：

- `/{franka_state_controller_name}/measured_joint_states`
- `/{ros2_control_node_name}/measured_joint_states`
- `/measured_joint_states`

以及 `current_pose`、`desired_end_effector_twist` 的对应候选（均会加上 `franka_namespace` 前缀，见下节参数）。

与 `franka_bringup` 中 `franka.launch.py` 仅 spawner `joint_state_broadcaster` 与 `franka_robot_state_broadcaster` 的场景一致时，通常由 **`franka_robot_state_broadcaster`** 提供上述测量状态话题。

### Inspire

- 默认：`/angle_data`（`GetAngleAct1`）

### RealSense（常见默认名）

- `/camera/color/image_raw`
- `/camera/depth/image_rect_raw`
- `/camera/color/camera_info`

若你的相机节点名或 remap 不同，请通过参数覆盖（见下）。

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `observation_topic` | string | `/robot/observation` | 发布统一观测的话题名 |
| `publish_rate_hz` | double | `30.0` | 发布频率（Hz） |
| `franka_namespace` | string | `""` | 若 `franka.launch.py` 使用了 `namespace:=xxx`，此处填 `xxx`（不要带首尾 `/`） |
| `franka_state_controller_name` | string | `franka_robot_state_broadcaster` | 状态广播控制器名，用于拼接 FR3 订阅候选 |
| `ros2_control_node_name` | string | `ros2_control_node` | `ros2_control_node` 节点名，用于备选订阅路径 |
| `arm_joint_name_order` | string[] | `[]` | 非空时按**关节名**重排 `JointState` 到固定顺序；空则沿用消息内顺序 |
| `hand_angle_topic` | string | `/angle_data` | Inspire 角度话题 |
| `hand_finger_id_order` | int[] | `[0,1,2,3,4,5]` | 将 `GetAngleAct1.finger_ids` 映射到 `hand_joint_position` 的固定顺序 |
| `rs_rgb_image_topic` | string | `/camera/color/image_raw` | RGB 图像 |
| `rs_depth_image_topic` | string | `/camera/depth/image_rect_raw` | 深度图像 |
| `rs_camera_info_topic` | string | `/camera/color/camera_info` | 相机标定（常与 RGB 对齐） |

命令行覆盖示例：

```bash
ros2 run observation_aggregator observation_aggregator_node.py --ros-args \
  -p franka_namespace:=my_ns \
  -p publish_rate_hz:=20.0 \
  -p observation_topic:=/robot/observation
```

## 设计说明与注意

- **控制权**：本节点仅订阅传感器/状态话题并发布观测，不向机械臂写入控制指令；是否与 Franky 等其它栈在**底层连接**上冲突，取决于你是否同时让两套栈连接同一台机器人。
- **手部速度**：`GetAngleAct1` 不含速度字段时，由相邻两次角度消息的时间差做数值微分，属于近似值。
- **末端 twist**：当前订阅的是 Franka 状态广播里名为 `desired_end_effector_twist` 的项，语义为期望末端速度相关量；若需“测量速度”类字段，需后续对接其它 topic 或扩展消息。
- **遗留 C++ 源码**：`src/observation_aggregator_node.cpp` 为早期实现，当前包通过 CMake 安装 **Python** 可执行脚本；构建时不再编译该 C++ 文件。

## 许可证

与包内 `package.xml` 声明一致（当前为 MIT）。
