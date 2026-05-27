cmake_minimum_required(VERSION 3.8)
project(task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_control_msgs REQUIRED)

# 包含头文件目录
include_directories(include)

# ================= 关键修改位置 =================
# 更新 utils/task_helpers.cpp 的相对路径
add_executable(task_manager 
  src/task_manager.cpp
  src/utils/task_helpers.cpp
)
add_executable(task_VIC 
  src/task_VIC.cpp
  src/utils/task_helpers.cpp
)
# ================================================

ament_target_dependencies(task_manager
  rclcpp
  rclcpp_action
  robot_control_msgs
)

ament_target_dependencies(task_VIC
  rclcpp
  rclcpp_action
  robot_control_msgs
)

install(TARGETS
  task_manager
  task_VIC
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()