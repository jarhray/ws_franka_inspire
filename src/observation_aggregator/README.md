# observation_aggregator

将 **FR3（Franka）**、**Inspire 灵巧手** 与 **RealSense** 的原始话题汇聚为一条统一的 `robot_interfaces/msg/RobotObservation` 消息，供策略层或记录模块消费。当前实现为 **“最新值汇总 + 固定频率发布”**，时间戳取各子源中较新的时间（为后续精确时间同步预留扩展空间）。

## 功能概览

| 来源 | 订阅内容 | 写入 `RobotObservation` 字段 |
|------|----------|------------------------------|
| FR3 | `sensor_msgs/JointState`（测量关节状态） | `arm_joint_position`、`arm_joint_velocity` |
| FR3 | `geometry_msgs/PoseStamped`（末端位姿） | `ee_pose`、`ee_pose_frame` |
| FR3 | `geometry_msgs/TwistStamped`（末端 twist，来自 Franka 状态广播中的期望项） | `ee_twist` |
| Inspire | `service_interfaces/msg/GetAngleAct1`（`angle_data` 等） | 节点内部仍按原样接收 **硬件整数 k**；仅在**发布** `/robot/observation` 时把 k 换成弧度 **r=f(k)** 写入 `hand_joint_position`，`hand_joint_velocity` 为 **rad/s**（对 r 的差分）。与 `inspire_executor` 成对。 |
| RealSense | `sensor_msgs/Image` / `CameraInfo` | `rgb_image`、`depth_image`、`camera_info`；多路时另写入 `camera_ids` / `rgb_images` / `depth_images` / `camera_infos`（并行数组） |

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
ros2 run observation_aggregator observation_aggregator_node
```

说明：`ros2 pkg executables observation_aggregator` 只会列出 **`observation_aggregator_node`**（无 `.py` 后缀）。若写 `observation_aggregator_node.py`，部分环境下会报 **No executable found**，请用上面这条。安装目录里通常仍有 `.../lib/observation_aggregator/observation_aggregator_node.py` 脚本，也可直接 `python3` 该路径运行。

典型组合：先启动 Franka 侧（例如 `franka_bringup` 的 `franka.launch.py`），再启动 RealSense 与 Inspire 相关节点，最后启动本节点。

**若曾用旧版 C++ 可执行文件编译过本包**，请先重新 `colcon build` 并 `source install/setup.bash`，避免 `install/` 里残留同名二进制覆盖 Python 入口。

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/robot/observation`（默认，可改） | `robot_interfaces/msg/RobotObservation` | 统一观测 |

## 订阅话题（默认与说明）

### FR3

节点会**同时订阅**多路候选 topic（任一路有数据即更新缓存）。默认包括：

- `/{franka_state_controller_name}/measured_joint_states` 等（来自 **`franka_robot_state_broadcaster`**，真机常用）
- **额外回退**（参数 `arm_joint_state_extra_topics`，默认 `[/franka/joint_states, /joint_states]`）：由 `joint_state_broadcaster` + `franka.launch.py` 里的 `joint_states` remap 提供

**重要**：`franka.launch.py` 在 `use_fake_hardware:=true` 时**不会** spawner `franka_robot_state_broadcaster`（见 launch 里 `UnlessCondition(use_fake_hardware)`），因此此时**不存在** `.../measured_joint_states`。仿真/假硬件下机械臂关节只会出现在 **`/franka/joint_states`** 或合并后的 **`/joint_states`**，必须使用上述回退（本包已默认订阅）。

末端位姿 `current_pose` / `desired_end_effector_twist` 同样依赖 `franka_robot_state_broadcaster`；在仅假硬件、未加载该控制器时，`ee_pose` / `ee_twist` 可能一直为空或为零，这是预期现象。

### Inspire

- 默认：`/angle_data`（`GetAngleAct1`）
- **接口未改**：订阅回调仍只处理 `angle_data` 的原始 **k**（与此前一致）。
- **发布时换算**：在组装 `RobotObservation` 时增加一步 **k→弧度 r=f(k)**（与 `inspire_executor` 中前向 f 一致）：
  - 四指（id 0–3）：\(r = -5\times10^{-10}k^3 + 9\times10^{-7}k^2 - 0.0018k + 1.4191\)
  - 拇指弯曲（id 4）：\(r = 8\times10^{-11}k^3 - 4\times10^{-8}k^2 - 0.0006k + 0.5869\)
  - 拇指侧摆（id 5）：\(r = -0.0012k + 1.1641\)

### RealSense（常见默认名）

- `/camera/color/image_raw`
- `/camera/depth/image_rect_raw`
- `/camera/color/camera_info`

若 launch 把相机放在嵌套命名空间下（例如同时存在 `/camera/camera/color/image_raw`），本节点默认**额外订阅**一组 alternates（见参数 `rs_*_topic_alternates`），也可手动改 `rs_rgb_image_topic` 等。

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `observation_topic` | string | `/robot/observation` | 发布统一观测的话题名 |
| `publish_rate_hz` | double | `30.0` | 发布频率（Hz） |
| `franka_namespace` | string | `""` | 若 `franka.launch.py` 使用了 `namespace:=xxx`，此处填 `xxx`（不要带首尾 `/`） |
| `franka_state_controller_name` | string | `franka_robot_state_broadcaster` | 状态广播控制器名，用于拼接 FR3 订阅候选 |
| `ros2_control_node_name` | string | `ros2_control_node` | `ros2_control_node` 节点名，用于备选订阅路径 |
| `arm_joint_state_extra_topics` | string[] | `[/franka/joint_states,/joint_states]` | 假硬件或未加载 `franka_robot_state_broadcaster` 时的关节回退话题 |
| `arm_joint_name_order` | string[] | `[]` | 非空时按**关节名**重排 `JointState` 到固定顺序；空则沿用消息内顺序 |
| `hand_angle_topic` | string | `/angle_data` | Inspire 角度话题 |
| `hand_finger_id_order` | int[] | `[0,1,2,3,4,5]` | 将 `GetAngleAct1.finger_ids` 映射到 `hand_joint_position` 的固定顺序 |
| `rs_rgb_image_topic` | string | `/camera/color/image_raw` | RGB 图像 |
| `rs_depth_image_topic` | string | `/camera/depth/image_rect_raw` | 深度图像 |
| `rs_camera_info_topic` | string | `/camera/color/camera_info` | 相机标定（常与 RGB 对齐） |
| `rs_rgb_image_topic_alternates` 等 | string[] | 见代码默认 | 与主 `rs_*` 并行订阅的备选 topic（嵌套相机名时常用） |
| `enabled_cameras` | string | `""` | **推荐**：逗号分隔逻辑名（与 `robot_bringup/hardware_bringup.launch.py` 的 `enabled_cameras` 一致）。非空时覆盖 `observation_camera_ids`，并按 `/id/id/color|depth|camera_info` 自动订阅 |
| `observation_camera_ids` | string[] | `[]` | **非空则启用多相机模式**（此时不再订阅 `rs_*`）。若 RGB/Depth topic 列表均为空，则按与 `enabled_cameras` 相同的 RealSense 布局自动生成 topic |
| `observation_camera_rgb_topics` | string[] | `[]` | 与 `observation_camera_ids` 等长；或留空以启用上条自动布局 |
| `observation_camera_depth_topics` | string[] | `[]` | 同上 |
| `observation_camera_info_topics` | string[] | `[]` | 与上等长；留空则自动订阅各路 `color/camera_info`；可填 `""` 跳过该路订阅 |
| `default_camera_id` | string | `cam1` | 写入 legacy `rgb_image`/`depth_image`/`camera_info` 时选用的逻辑相机名 |

多相机示例（与 `hardware_bringup` 对齐，一行即可）：

```yaml
enabled_cameras: "cam1,cam3"
default_camera_id: cam1
```

等价写法（手写 topic，适用于非标准命名空间）：

```yaml
observation_camera_ids: [cam1, cam3]
observation_camera_rgb_topics: [/cam1/cam1/color/image_raw, /cam3/cam3/color/image_raw]
observation_camera_depth_topics: [/cam1/cam1/depth/image_rect_raw, /cam3/cam3/depth/image_rect_raw]
observation_camera_info_topics: [/cam1/cam1/color/camera_info, /cam3/cam3/color/camera_info]
default_camera_id: cam1
```

命令行覆盖示例：

```bash
ros2 run observation_aggregator observation_aggregator_node.py --ros-args \
  -p franka_namespace:=my_ns \
  -p publish_rate_hz:=20.0 \
  -p observation_topic:=/robot/observation
```

## 设计说明与注意

### 排查：关节为空但 `/franka/joint_states` 有数据

几乎总是下面两类原因之一：

1. 使用了 `use_fake_hardware:=true`，未加载 `franka_robot_state_broadcaster`，只应依赖 `/franka/joint_states`（本包已默认额外订阅）。
2. 运行了错误的可执行文件（旧 C++ 安装残留）：请重新编译并确认 `ros2 run observation_aggregator observation_aggregator_node` 启动日志里出现 **`observation_aggregator (Python) started`**。

### 其它

- **控制权**：本节点仅订阅传感器/状态话题并发布观测，不向机械臂写入控制指令；是否与 Franky 等其它栈在**底层连接**上冲突，取决于你是否同时让两套栈连接同一台机器人。
- **手部速度**：`GetAngleAct1` 不含速度字段时，由相邻两次角度消息的时间差做数值微分，属于近似值。
- **末端 twist**：当前订阅的是 Franka 状态广播里名为 `desired_end_effector_twist` 的项，语义为期望末端速度相关量；若需“测量速度”类字段，需后续对接其它 topic 或扩展消息。
- **遗留 C++ 源码**：`src/observation_aggregator_node.cpp` 为早期实现，当前包通过 CMake 安装 **Python** 可执行脚本；构建时不再编译该 C++ 文件。

## 许可证

与包内 `package.xml` 声明一致（当前为 MIT）。
