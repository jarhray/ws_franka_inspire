cmake_minimum_required(VERSION 3.8)
project(observation_aggregator)

find_package(ament_cmake REQUIRED)

# Python entry points: keep both names so `ros2 run ... observation_aggregator_node`
# works (avoids accidentally picking up a stale C++ binary from an old install).
install(
  PROGRAMS
    scripts/observation_aggregator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME observation_aggregator_node
)

install(
  PROGRAMS
    scripts/observation_aggregator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
