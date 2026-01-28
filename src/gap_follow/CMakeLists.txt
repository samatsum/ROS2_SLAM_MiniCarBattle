cmake_minimum_required(VERSION 3.8)
project(gap_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # <--- 追加: geometry_msgs が必要

# --- Reactive Gap Follower Node ---
add_executable(reactive_node src/reactive_node.cpp)
ament_target_dependencies(reactive_node rclcpp sensor_msgs ackermann_msgs)

# --- Sim Reset Node (追加部分) ---
add_executable(sim_reset_node src/sim_reset_node.cpp)
ament_target_dependencies(sim_reset_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  reactive_node
  sim_reset_node # <--- ここにも追加
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/data_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()