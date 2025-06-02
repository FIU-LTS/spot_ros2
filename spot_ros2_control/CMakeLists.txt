# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)
project(spot_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED) # <<< ENSURE THIS IS PRESENT
find_package(spot_msgs REQUIRED)
find_package(bosdyn_api_msgs REQUIRED)
find_package(yaml-cpp REQUIRED) # Changed from yaml-cpp_vendor if you use system yaml-cpp
find_package(tf2_geometry_msgs REQUIRED)
find_package(proto2ros REQUIRED)

# Find Bosdyn SDK and Protobuf libraries
find_package(bosdyn_cmake_module REQUIRED)
find_package(bosdyn REQUIRED)
find_package(Protobuf REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  spot_hardware_interface
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(
  spot_ros2_control
  SHARED
  src/spot_joint_map.cpp
)
target_compile_features(spot_ros2_control PUBLIC cxx_std_20)
target_include_directories(spot_ros2_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  spot_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(noarm_squat src/noarm_squat.cpp)
target_link_libraries(noarm_squat spot_ros2_control)

add_executable(wiggle_arm src/wiggle_arm.cpp)
target_link_libraries(wiggle_arm spot_ros2_control)

add_executable(joint_command_passthrough src/joint_command_passthrough.cpp)
target_link_libraries(joint_command_passthrough spot_ros2_control)

# Joy Service Caller (Buttons)
add_executable(joy_service_caller src/joy_service_caller.cpp)
ament_target_dependencies(joy_service_caller
  rclcpp
  sensor_msgs
  yaml-cpp
  std_srvs
  spot_msgs
)
target_link_libraries(joy_service_caller
  ${YAML_CPP_LIBRARIES}
)

# Joy Body Pose Controller (Axes)
add_executable(joy_body_pose_controller src/joy_body_pose_controller.cpp)
ament_target_dependencies(joy_body_pose_controller
  rclcpp
  sensor_msgs
  std_srvs # <<< ADDED THIS DEPENDENCY
  yaml-cpp
  tf2_geometry_msgs
  geometry_msgs
)
target_link_libraries(joy_body_pose_controller
  ${YAML_CPP_LIBRARIES}
)

# New Pose to RobotCommand Translator Node
add_executable(pose_to_robot_command_translator src/pose_to_robot_command_node.cpp)
ament_target_dependencies(pose_to_robot_command_translator
  rclcpp
  rclcpp_action
  geometry_msgs
  spot_msgs
  bosdyn_api_msgs
  proto2ros
)
target_link_libraries(pose_to_robot_command_translator
  bosdyn::bosdyn_client # From spot_driver CMakeLists.txt
  ${Protobuf_LIBRARIES}
)

install(TARGETS
  noarm_squat
  wiggle_arm
  joint_command_passthrough
  joy_service_caller
  joy_body_pose_controller
  pose_to_robot_command_translator
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch config # Removed rviz
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS spot_ros2_control
  EXPORT export_spot_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_spot_ros2_control HAS_LIBRARY_TARGET)
ament_package()
