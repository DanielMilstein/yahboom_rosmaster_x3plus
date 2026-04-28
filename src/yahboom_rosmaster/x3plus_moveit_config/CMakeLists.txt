cmake_minimum_required(VERSION 3.16)
project(x3plus_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(
  PROGRAMS
    scripts/filter_moveit_joint_states.py
    scripts/publish_tabletop_scene.py
  DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
