cmake_minimum_required(VERSION 3.16)
project(x3plus_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(reachability_cloud_publisher src/reachability_cloud_publisher.cpp)
ament_target_dependencies(reachability_cloud_publisher
  moveit_ros_planning
  rclcpp
  sensor_msgs
  std_msgs
)

ament_package()

install(
  PROGRAMS
    scripts/filter_moveit_joint_states.py
    scripts/perception_bridge.py
    scripts/publish_tabletop_scene.py
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS reachability_cloud_publisher
  DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
