你现在要帮我在一个已有的 ROS2 Humble 工作区里，搭建一套用于 FR3 + Inspire 灵巧手 + RealSense + BC/VLA 策略部署的系统框架。
请不要只给概念设计，我需要你输出能落地到 ROS2 工作区中的实际包结构和代码骨架，并尽量保证可以 colcon build。
====================
一、已有环境与硬件
====================

1. ROS 版本
- ROS2 Humble

2. 工作区
- 工作区路径：~/ws_franka_inspire
- 当前结构如下：

src/
  franka_ros2/
    ...  # 官方 franka_ros2 仓库，已经可以使用
  inspire_ros2/
    inspire_hand_modbus_ros2/
    service_interfaces/
  realsense-ros-4.55.1/
    realsense2_camera/
    realsense2_camera_msgs/
    realsense2_description/

使用这样的命令来激活虚拟环境
```
cd ~/ws_franka_inspire
source .venv/bin/activate
```


3. 硬件
- 机械臂：Franka Research 3 (FR3)
- 灵巧手：Inspire hand
- 相机：Intel RealSense

4. 现状
- franka_ros2 已经安装并可用
- inspire 的 ROS2 部分已经完成，可以通过 service 或 topic 控制手部
- realsense ROS2 包已经在工作区中
- 我希望机械臂控制部分最终使用 franky，而不是 MoveIt
- franka_ros2 只保留状态发布、机器人描述、已有消息/配置等功能，不再负责真实机械臂控制
- 真实机械臂控制全部交给 franky
- 灵巧手控制保留为细粒度手指控制
- BC/VLA 模型未来可能输出不同形式的动作，因此系统需要支持“通过参数切换控制模式”

====================
二、总体目标
====================

请你在这个工作区里新增一批 ROS2 包，构建一个统一的机器人系统框架，要求：

1. 不要修改上游仓库的核心结构
- 不要把我的业务逻辑直接写进 src/franka_ros2 的已有包内部
- 不要把我的业务逻辑直接硬塞进 realsense-ros 的已有包内部
- inspire_ros2 中如果已有驱动功能，尽量复用，不要破坏现有结构

2. 所有新增业务代码都作为“新的独立 ROS2 包”放在 src/ 下
- 可以直接在 src/ 下新增多个包
- 或者新建一个单独文件夹来放这些新包，但这些新包最终要能被 colcon 正常识别和编译

3. 整体系统需要支持以下能力
- 读取 RealSense 图像/深度/相机信息
- 获取 FR3 状态
- 获取 Inspire 手状态
- 把所有观测整合成统一观测消息
- 接入 BC 或 VLA 模型推理节点
- 对策略输出动作做统一路由、模式解释、安全裁剪
- 把机械臂动作交给 franky 执行
- 把手部动作交给 Inspire executor 执行
- 提供统一 bringup/launch

4. 机械臂控制模式要支持参数切换
至少支持：
- joint_position
- joint_velocity
- cartesian_pose
- cartesian_velocity

5. 灵巧手控制
- 以细粒度 finger joint control 为主
- 至少支持 joint_position
- 最好预留 joint_velocity 扩展接口

====================
三、最重要的架构约束
====================

1. FR3 控制链路
- franka_ros2 只负责状态、描述、必要消息、已有状态广播
- 不允许 franka_ros2 再占用真实机械臂控制权
- 真正的 FR3 控制执行器是一个新的独立节点：fr3_franky_executor
- fr3_franky_executor 负责与 franky 对接

2. Inspire 控制链路
- Inspire 已经有 ROS2 控制接口（service/topic）
- 新系统中要增加 inspire_executor 节点，用于接收统一手部动作，再调用现有 inspire 接口
- 不要破坏现有 inspire_ros2 包

3. 策略层与执行层解耦
- BC/VLA 模型节点不能直接连真实机器人
- 策略输出必须先经过 action_router / adapter / safety 这一层
- 最终由 arm executor 和 hand executor 分别执行

4. 不要上来就写一个“巨大的单体节点”
- 我希望是模块化的 ROS2 包
- 每个包职责明确
- topic / msg / launch 结构清晰

====================
四、希望新增的包
====================

请你按下面思路创建包，包名可以沿用这些名字。如果你觉得有更合理的命名，可以调整，但要保持职责清晰。

1. robot_interfaces
职责：
- 定义系统内部统一使用的 msg / srv / action（如有必要）
- 不放业务逻辑
建议内容：
- msg/RobotObservation.msg
- msg/ArmAction.msg
- msg/HandAction.msg
- msg/WholeBodyAction.msg
- srv/StopArm.srv
- srv/HomeArm.srv
- srv/ResetArmFault.srv
如果你认为部分 srv 先不需要，也可以减少，但消息定义要先做好

2. observation_aggregator
职责：
- 订阅 RealSense、FR3 状态、Inspire 状态等
- 组装成统一 RobotObservation
- 对外发布统一观测 topic
要求：
- 订阅 color image / depth 或 aligned depth / camera_info
- 订阅 FR3 joint state / ee pose / ee twist（如果当前已有来源）
- 订阅 Inspire hand joint state
- 输出统一观测消息
注意：
- 先不要求复杂同步算法，但代码结构要便于后续加入 message_filters 或时间同步

3. policy_manager
职责：
- 管理策略类型（BC / VLA）
- 统一策略入口
- 先可以只做“占位版”或“简单 mock 版”，但结构要留好
要求：
- 通过参数决定 policy_type，例如 bc / vla / dummy
- 订阅 RobotObservation
- 输出策略原始动作
说明：
- 这里先不一定要把真实大模型写进去
- 可以先提供 dummy policy，输出固定动作或零动作，方便打通链路
- 但接口设计要适配未来真实 BC/VLA

4. action_router
职责：
- 接收策略动作
- 根据参数解释当前动作的控制模式
- 做安全裁剪、限幅、模式翻译
- 输出标准化的 arm action 和 hand action
要求：
- 支持 arm 控制模式切换：
  - joint_position
  - joint_velocity
  - cartesian_pose
  - cartesian_velocity
- hand 先支持：
  - joint_position
  - 可选 joint_velocity
- 支持 relative / absolute 的参数开关
- 支持限幅与简单安全保护
建议：
- 把 arm 和 hand 的转换逻辑拆清楚，必要时可分成 arm_action_adapter 和 hand_action_adapter 两个类/模块，但包层面可以先放在 action_router 里

5. fr3_franky_executor
职责：
- 这是 FR3 唯一真实执行节点
- 接收标准化 arm action
- 调用 franky 执行机械臂动作
要求：
- 长期持有 robot 连接，不要每条消息重新建连接
- 支持上面 4 种控制模式
- 提供 stop / home / reset fault 等 service
- 发布 arm state / executor status / error status
说明：
- 如果 franky 当前还没接入完成，可以先把接口和类结构搭好，用 mock 或 TODO 标记，但整体节点框架要正确
- 如果可以直接接入 franky，请优先按真实接口写

6. inspire_executor
职责：
- 接收标准化 hand action
- 转成已有 inspire_ros2 所需的 service/topic 调用
要求：
- 做 joint 顺序映射
- 做上下限裁剪
- 统一发送频率
- 尽量不要依赖硬编码，改为参数化 joint 名称和限制

7. robot_bringup
职责：
- 放统一 launch 文件和 yaml 参数文件
要求：
- 至少提供两个 launch：
  1) perception_debug.launch.py
  2) real_policy_execute.launch.py
- perception_debug.launch.py：
  - 启动 realsense
  - 启动状态节点
  - 启动 observation_aggregator
  - 可选 RViz
- real_policy_execute.launch.py：
  - 启动 realsense
  - 启动 observation_aggregator
  - 启动 policy_manager
  - 启动 action_router
  - 启动 fr3_franky_executor
  - 启动 inspire_executor

8. experiment_logger（可选但推荐）
职责：
- 统一记录观测、动作、状态、错误信息
- 方便后续做数据回放或调试
如果时间有限，可以先给基本框架

====================
五、推荐的目录安排
====================

请在当前工作区下按这种思路组织：

~/ws_franka_inspire/
└── src
    ├── franka_ros2/
    ├── inspire_ros2/
    ├── realsense-ros-4.55.1/
    ├── robot_interfaces/
    ├── observation_aggregator/
    ├── policy_manager/
    ├── action_router/
    ├── fr3_franky_executor/
    ├── inspire_executor/
    ├── robot_bringup/
    └── experiment_logger/   # 可选

重点：
- 新增的这些包都应是标准 ROS2 包
- 不要把这些包写到 franka_ros2 仓库内部
- 不要破坏已有 inspire_ros2 和 realsense-ros 结构

====================
六、消息与接口设计要求
====================

请你先设计一套清晰、够用、不过度复杂的内部消息。

推荐至少有：

1. RobotObservation.msg
建议包含：
- Header header
- sensor_msgs/Image color
- sensor_msgs/Image depth（如果暂时不方便，可先留接口）
- sensor_msgs/CameraInfo camera_info
- sensor_msgs/JointState arm_joint_state
- sensor_msgs/JointState hand_joint_state
- geometry_msgs/PoseStamped ee_pose
- geometry_msgs/TwistStamped ee_twist
- uint8 task_phase（可选）

2. ArmAction.msg
建议包含：
- Header header
- string control_mode
- float64[] values
- bool relative
- float64 speed_scale

3. HandAction.msg
建议包含：
- Header header
- string control_mode
- float64[] values
- bool blocking

4. WholeBodyAction.msg
建议包含：
- Header header
- ArmAction arm
- HandAction hand

如果你觉得某些字段应进一步细化，也可以调整，但要保持清晰和通用。

====================
七、参数化要求
====================

系统必须参数化，不要大量硬编码。

重点参数包括：

1. action_router 参数
- arm_control_mode
- hand_control_mode
- relative_default
- joint_dim
- hand_joint_dim
- action_scale_xxx
- workspace_xyz_min / max
- joint_limit_min / max
- command_timeout
- allow_policy_override_mode

2. fr3_franky_executor 参数
- robot_ip
- default_control_mode
- publish_rate
- command_timeout
- joint_limits
- cartesian_limits
- home_joint_positions
- ee_frame / base_frame

3. inspire_executor 参数
- command topic / service name
- hand_joint_names
- lower_limits
- upper_limits
- max_step_per_cycle
- publish_rate

4. observation_aggregator 参数
- realsense topics
- arm state topics
- hand state topics
- use_depth
- queue size

====================
八、编码风格要求
====================

1. 代码要易维护
- 每个节点一个清晰的 main 和核心类
- 尽量拆头文件 / 源文件
- 命名清晰
- 适当注释

2. C++ / Python 语言建议
- 最好优先使用python
- launch 文件用 Python
- 如果 policy_manager 更适合 Python，也可以用 Python
- 但请你自行判断，前提是整体结构清晰、易编译、易维护

3. 不要过度实现
- 先以“框架正确、接口清晰、能编译、能启动、能打通链路”为第一目标
- 不要求一开始就把所有高级能力写满
- dummy policy / mock executor / TODO 是允许的，但整体结构必须合理

====================
九、我希望你交付的内容
====================

请按下面顺序交付：

1. 先给出最终建议的包结构树
2. 再给出每个包的职责说明
3. 再开始生成代码
4. 生成代码时，优先保证：
- CMakeLists.txt / setup.py
- package.xml
- 节点源码
- msg/srv 定义
- launch 文件
- yaml 参数文件
- README 或使用说明

5. 代码生成完成后，请明确说明：
- 新增了哪些文件
- 每个节点订阅/发布哪些 topic
- 如何编译
- 如何启动 perception_debug.launch.py
- 如何启动 real_policy_execute.launch.py

====================
十、实现优先级
====================

请按这个优先级来：

第一优先级：
- robot_interfaces
- observation_aggregator
- action_router
- inspire_executor
- robot_bringup

第二优先级：
- fr3_franky_executor 的完整框架
- policy_manager 的 dummy 版

第三优先级：
- experiment_logger
- 更完整的 safety 与 task phase

====================
十一、特别注意
====================

1. 不要让我去手工大改现有 franka_ros2 上游包
2. 不要把新增逻辑埋在 franka_ros2 仓库内部
3. 不要假设 MoveIt 是主执行链
4. 机械臂主执行链是 franky
5. 灵巧手保留细粒度 joint 控制
6. 代码必须围绕“未来要接 BC/VLA 模型”来设计
7. 所有动作都必须经过统一 router / adapter，而不是模型直接控制机器人
8. 如果你对某些真实 topic 名称不确定，可以先把它们做成参数，不要写死

====================
十二、你现在就开始做的第一步
====================

请先不要直接写全部代码。
先输出：
1. 你理解后的系统架构图（文字形式即可）
2. 建议新增的 ROS2 包列表
3. 每个包的职责
4. 推荐的 topic / service / msg 设计
5. 推荐的 launch 组织方式

等这些结构确认后，再开始逐包生成代码。