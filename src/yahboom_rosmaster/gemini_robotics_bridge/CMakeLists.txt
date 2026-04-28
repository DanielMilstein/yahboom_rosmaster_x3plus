cmake_minimum_required(VERSION 3.8)
project(gemini_robotics_bridge)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/gemini_robotics_bridge.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY prompts DESTINATION share/${PROJECT_NAME})
install(FILES requirements-gemini.txt DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
