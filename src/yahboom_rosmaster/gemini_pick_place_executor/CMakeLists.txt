cmake_minimum_required(VERSION 3.8)
project(gemini_pick_place_executor)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(yahboom_rosmaster_msgs REQUIRED)

add_executable(pick_place_manipulation_server src/pick_place_manipulation_server.cpp)
ament_target_dependencies(pick_place_manipulation_server
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp
  rclcpp_action
  shape_msgs
  yahboom_rosmaster_msgs
)

install(
  PROGRAMS
    scripts/gemini_pick_place_executor.py
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS
    pick_place_manipulation_server
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
