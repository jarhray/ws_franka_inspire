# fr3_franky_executor

`fr3_franky_executor` 是机械臂执行层节点：接收标准化 `ArmAction`，按控制模式通过 **franky**（`franky.Robot`）执行运动，并提供 stop/home/reset fault 服务与状态/错误发布。

- `mock_mode=true`（默认）：不连接机器人，仅打印/回显意图，便于无硬件联调。
- `mock_mode=false`：连接 Franka FCI，使用 `franky` 的 `JointMotion` / `JointVelocityMotion` / `CartesianMotion` / `CartesianVelocityMotion` 调用 `Robot.move()`。

真实控制相关代码在 `real_franky_backend.py`；mock 在 `franky_backend_impl.py`。仅 mock 时**不需要**安装 `franky`；真实模式需在运行该节点的同一 Python 环境中能 `import franky`。

## 功能概览

- 支持 `ArmAction.control_mode`：
  - `0`: `joint_position` → `franky.JointMotion`
  - `1`: `joint_velocity` → `franky.JointVelocityMotion`（`duration_sec` 缺省或 ≤0 时用 `default_velocity_duration_sec`）
  - `2`: `cartesian_pose` → `franky.CartesianMotion`（`geometry_msgs/Pose` 转为 `franky.Affine`）
  - `3`: `cartesian_velocity` → `franky.CartesianVelocityMotion`（`Twist` → `franky.Twist`）
- `ArmAction.is_relative` → `franky.ReferenceType.Relative` / `Absolute`
- 服务：
  - `stop_arm` → `Robot.stop()`
  - `home_arm` → 关节空间运动到 `home_joint_position`（`JointMotion`）
  - `reset_arm_fault` → `Robot.recover_from_errors()`（`hard_reset` 时会尝试连续调用两次）
- 非 mock 时，所有 `move`/服务在**单线程线程池**中执行，避免阻塞 `rclpy` 回调；完成/错误通过队列由定时器侧Drain 后发布。

## 订阅 / 发布 / 服务

- 订阅：`arm_action_topic`（默认 `/robot/arm_action`），`robot_interfaces/msg/ArmAction`
- 发布：`status_topic`、`error_topic`，`std_msgs/msg/String`
- 服务：`/stop_arm`、`/home_arm`、`/reset_arm_fault`

## 主要参数

| 参数 | 说明 |
|------|------|
| `mock_mode` | `true` 为仿真；`false` 使用 franky 真机 |
| `fci_hostname` | Franka 控制接口地址（常见 `172.16.0.2`） |
| `franky_controller_mode` | `joint_impedance` 或 `cartesian_impedance`（笛卡尔位姿/速度为主时可改为后者） |
| `franky_relative_dynamics_factor` | 传给 franky 的相对动力学因子 |
| `default_velocity_duration_sec` | 速度模式在 `duration_sec` 无效时的默认持续时间（秒） |
| `velocity_async` | 速度类 `move` 是否 `asynchronous=true`（流式发速度时可开） |
| `home_joint_position` | 长度 7，`home_arm` 目标关节角（弧度） |
| `command_timeout_sec` | 过久未收到新 `ArmAction` 时发超时错误 |
| `default_reference_frame` | 仅填充空字符串的 `reference_frame` 字段，不传给 franky |

## 依赖

- ROS：`rclpy`、`std_msgs`、`robot_interfaces`
- 真实模式：PyPI 包 **`franky-control`**（`import` 名仍为 `franky`）、`numpy`

### Python 解释器与 `ModuleNotFoundError: No module named 'franky'`

`ros2 run` 生成的入口脚本通常使用系统 **`/usr/bin/python3`**，而你用 **`pip install franky-control`** 往往装在 **`.venv`** 里，两套环境不一致时就会找不到 `franky`。

任选其一即可：

1. **让「ros2 用的那个 Python」也能 import franky**（推荐简单）：
   ```bash
   /usr/bin/python3 -m pip install --user franky-control
   ```
2. **把 venv 的 site-packages 放进 `PYTHONPATH`**（不改系统包）：
   ```bash
   export PYTHONPATH="$HOME/ws_franka_inspire/.venv/lib/python3.10/site-packages:${PYTHONPATH}"
   ros2 run fr3_franky_executor fr3_franky_executor_node --ros-args -p mock_mode:=false ...
   ```
   （把 `python3.10` 换成你 `python3 -c "import sys; print(sys.version_info)"` 对应目录。）

节点在无法 import `franky` 时会在 status/error 中打印当前 `sys.executable` 与上述安装提示，而不会仅在 import 阶段崩溃。

## 构建

```bash
cd ~/ws_franka_inspire
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_franky_executor
source install/setup.bash
```

## 使用方法

### 1) Mock（默认）

```bash
ros2 run fr3_franky_executor fr3_franky_executor_node
```

### 2) 真实 franky / FCI

```bash
ros2 run fr3_franky_executor fr3_franky_executor_node --ros-args \
  -p mock_mode:=false \
  -p fci_hostname:=172.16.0.2 \
  -p franky_controller_mode:=joint_impedance
```

### 3) 服务示例

```bash
ros2 service call /stop_arm robot_interfaces/srv/StopArm "{immediate: true}"
ros2 service call /home_arm robot_interfaces/srv/HomeArm "{wait: false, timeout_sec: 5.0}"
ros2 service call /reset_arm_fault robot_interfaces/srv/ResetArmFault "{hard_reset: false, timeout_sec: 3.0}"
```

## 安全提示

- 真实模式会直接运动机器人；请先在低速、小范围验证 `ArmAction` 与 `franky_controller_mode` 是否匹配你的任务。
- 若与 `franka_ros2` / 其他控制器同时争用 FCI，需自行避免多主控冲突。

## 许可证

与包内 `package.xml` 声明一致（MIT）。
