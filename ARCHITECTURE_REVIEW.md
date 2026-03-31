# FR3 + Inspire 工作区代码结构与逻辑评审

本文基于当前 `~/ws_franka_inspire/src` 代码现状整理，目标是：
- 详细总结现有代码结构与运行链路；
- 画出当前实现对应的流程图；
- 检查并指出逻辑上不够合理或有风险的地方；
- 给出不改代码前提下的整改优先级建议。

## 1. 工作区整体结构（当前代码）

你现在的工作区是「上游基础包 + 自研业务包」混合布局：

- 上游/第三方：
  - `franka_ros2`
  - `realsense-ros-4.55.1`
  - `inspire_ros2`
- 自研核心（围绕统一观测/策略/路由/执行）：
  - `robot_interfaces`
  - `observation_aggregator`
  - `policy_manager`
  - `action_router`
  - `fr3_franky_executor`
  - `inspire_executor`
  - `robot_bringup`
  - `experiment_logger`

这和你计划中的分层基本一致，说明框架雏形已经跑起来了。

## 2. 当前数据流 / 控制流（按代码真实行为）

```mermaid
flowchart LR
  subgraph HW[硬件与上游]
    RS[RealSense topics]
    FR3[FR3 state topics]
    INS[Inspire angle_data]
  end

  subgraph OBS[观测层]
    OA[observation_aggregator]
    RO[/robot/observation]
  end

  subgraph POL[策略层]
    PM[policy_manager<br/>dummy/act]
    WBA[/robot/whole_body_action]
  end

  subgraph RT[动作路由层]
    AR[action_router]
    AA[/robot/arm_action]
    HA[/robot/hand_action]
  end

  subgraph EXE[执行层]
    FE[fr3_franky_executor]
    IE[inspire_executor]
    ST[/robot/arm_execution/status]
    ER[/robot/arm_execution/error]
  end

  subgraph LOG[记录层]
    EL[experiment_logger]
  end

  RS --> OA
  FR3 --> OA
  INS --> OA
  OA --> RO
  RO --> PM
  PM --> WBA
  WBA --> AR
  AR --> AA
  AR --> HA
  AA --> FE
  HA --> IE
  FE --> ST
  FE --> ER
  RO --> EL
  AA --> EL
  HA --> EL
  ST --> EL
  ER --> EL
```

## 3. 各包职责与实现现状

### 3.1 `robot_interfaces`
- 已定义核心消息：
  - `RobotObservation.msg`
  - `ArmAction.msg`
  - `HandAction.msg`
  - `WholeBodyAction.msg`
- 已定义执行服务：
  - `StopArm.srv`
  - `HomeArm.srv`
  - `ResetArmFault.srv`


### 3.2 `observation_aggregator`
- 输入：
  - RealSense 图像/深度/相机内参
  - FR3 joint/pose/twist（候选 topic 机制）
  - Inspire `GetAngleAct1`（硬件角值）
- 输出：
  - 统一 `RobotObservation`
- 特点：
  - 通过多候选 topic 兼容不同 bringup 方式；
  - 手部角度在此转换为弧度；
  - 时间戳采用多源最大值策略。


### 3.3 `policy_manager`
- 支持 `policy_type` 参数分发；
- 已实际接入：
  - `dummy`
  - `act`
- `bc` / `vla` 目前仅保留入口，不执行有效策略（no-op）。


### 3.4 `action_router`
- 输入：`WholeBodyAction` + `RobotObservation`（用于相对模式）
- 输出：标准化 `ArmAction` / `HandAction`
- 支持：
  - arm 4 种控制模式；
  - hand `joint_position`；
  - duration clamp、关节限幅、速度限幅、相对转绝对。


### 3.5 `fr3_franky_executor`
- 机械臂唯一执行节点（支持 mock/real backend）；
- 订阅 `ArmAction`，发布执行状态和错误；
- 对外暴露 `stop/home/reset_fault` 服务；
- 真实模式下带异步 worker、状态轮询、重连退避。


### 3.6 `inspire_executor`
- 订阅 `HandAction`；
- 转换到 `SetAngle1` 并发布到 `set_angle_data`；
- 支持保持上一次命令定频重发。


### 3.7 `robot_bringup`
- `hardware_bringup.launch.py`：拉起 RealSense + FR3 executor + Inspire 上游控制；
- `perception_debug.launch.py`：拉起观测与日志；
- `real_policy_execute.launch.py`：拉起观测、策略、路由、手执行、日志。


### 3.8 `experiment_logger`
- 记录 observation / arm_action / hand_action / arm_status / arm_error；
- JSONL 分流输出，按 session 组织。


## 4. 你当前框架的优点（客观）

- 已形成“观测-策略-路由-执行-日志”的闭环；
- 接口先行（`robot_interfaces`）避免强耦合；
- executor 与策略分离，支持 mock 到 real 平滑切换；
- launch + yaml 参数化程度高，便于调参与分场景启动；
- observation 端做了多 topic 兼容，落地经验明显。
