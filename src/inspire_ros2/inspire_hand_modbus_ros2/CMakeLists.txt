cmake_minimum_required(VERSION 3.8)
project(inspire_hand_modbus_ros2)

# 添加警告选项以提高代码质量
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------
# 查找依赖项
# ---------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(service_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------------------------
# 添加 modbus 库
# ---------------------------
find_library(MODBUS_LIBRARIES NAMES modbus REQUIRED)
include_directories(/usr/include/modbus)

# ---------------------------
# 添加 include 目录
# ---------------------------
include_directories(include ${ament_INCLUDE_DIRS})

# ---------------------------
# 设置可执行文件输出目录
# ---------------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# ---------------------------
# 添加 C++ 节点
# ---------------------------
add_executable(hand_modbus_control_node 
  src/hand_control.cpp
)

add_executable(hand_control_client_modbus_node 
  src/hand_control_client_modbus.cpp
)

# ---------------------------
# 设置 C++ 节点的依赖库
# ---------------------------
ament_target_dependencies(hand_modbus_control_node
  rclcpp
  service_interfaces
  std_msgs
  ament_index_cpp
)

ament_target_dependencies(hand_control_client_modbus_node
  rclcpp
  service_interfaces
  std_msgs
)

# ---------------------------
# 链接 modbus 库到 C++ 节点
# ---------------------------
target_link_libraries(hand_modbus_control_node ${MODBUS_LIBRARIES})
target_link_libraries(hand_control_client_modbus_node ${MODBUS_LIBRARIES})

# ---------------------------
# 安装 C++ 节点
# ---------------------------
install(TARGETS
  hand_modbus_control_node
  hand_control_client_modbus_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------
# 安装 Python 节点
# ---------------------------
# 根据脚本路径调整安装路径
install(PROGRAMS
  scripts/handcontrol_topic_publisher_modbus.py
  scripts/handcontrol_topic_subscriber_modbus.py
  scripts/inspire_hand_modbus_topic.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------
# 安装 Launch 文件
# ---------------------------
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  src/config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# ---------------------------
# 测试和代码检查
# ---------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ---------------------------
# 最后调用 ament_package()
# ---------------------------
ament_package()

