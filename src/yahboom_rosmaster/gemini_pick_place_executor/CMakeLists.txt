cmake_minimum_required(VERSION 3.8)
project(gemini_pick_place_executor)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/gemini_pick_place_executor.py
    scripts/lidar_debug.py
    scripts/return_to_origin.py
    scripts/teach_pose.py
  DESTINATION lib/${PROJECT_NAME})

install(
  FILES
    scripts/pick_preflight.py
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
