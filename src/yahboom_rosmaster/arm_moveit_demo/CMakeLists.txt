cmake_minimum_required(VERSION 3.8)
project(arm_moveit_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(DEMO_DEPENDENCIES
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  shape_msgs
  tf2
  trajectory_msgs
)

include_directories(include)

foreach(demo
  01_random_move
  02_set_pose_plan
  03_set_joint_plan
  04_cartesian
  05_attached_object
  06_multi_track_motion
)
  add_executable(${demo} src/${demo}.cpp)
  target_compile_features(${demo} PUBLIC cxx_std_17)
  ament_target_dependencies(${demo} ${DEMO_DEPENDENCIES})
endforeach()

install(TARGETS
  01_random_move
  02_set_pose_plan
  03_set_joint_plan
  04_cartesian
  05_attached_object
  06_multi_track_motion
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz scene
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
