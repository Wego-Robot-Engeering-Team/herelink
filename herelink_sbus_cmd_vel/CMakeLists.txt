cmake_minimum_required(VERSION 3.8)
project(herelink_sbus_cmd_vel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(sbus_cmd_vel_node src/sbus_cmd_vel_node.cpp)
target_compile_features(sbus_cmd_vel_node PUBLIC cxx_std_17)
ament_target_dependencies(sbus_cmd_vel_node
  geometry_msgs
  rclcpp
)

install(TARGETS sbus_cmd_vel_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
