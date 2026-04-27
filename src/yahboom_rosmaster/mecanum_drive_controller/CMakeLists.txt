cmake_minimum_required(VERSION 3.16)
project(mecanum_drive_controller LANGUAGES CXX)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-Werror=range-loop-construct" HAS_WERROR_RANGE_LOOP_CONSTRUCT)
  set(MECANUM_DRIVE_CONTROLLER_WARNING_FLAGS
    -Wall
    -Wextra
    -Werror=conversion
    -Werror=unused-but-set-variable
    -Werror=return-type
    -Werror=shadow
    -Werror=format
    -Werror=missing-braces
  )
  if(HAS_WERROR_RANGE_LOOP_CONSTRUCT)
    list(APPEND MECANUM_DRIVE_CONTROLLER_WARNING_FLAGS -Werror=range-loop-construct)
  endif()
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable
                     -Werror=return-type -Werror=shadow -Werror=format -Werror=missing-braces)
endif()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pal_statistics_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(fmt REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

# Generate parameter header without generate_parameter_library() because the
# Humble generator macro uses INTERFACE target sources, which requires newer
# CMake behavior than CMake 3.16 provides on this platform.
find_program(GENERATE_PARAMETER_LIBRARY_CPP generate_parameter_library_cpp REQUIRED)
set(GENERATED_PARAMETER_INCLUDE_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/include/mecanum_drive_controller)
set(GENERATED_PARAMETER_HEADER
  ${GENERATED_PARAMETER_INCLUDE_DIR}/mecanum_drive_controller_parameters.hpp)
file(MAKE_DIRECTORY ${GENERATED_PARAMETER_INCLUDE_DIR})
add_custom_command(
  OUTPUT ${GENERATED_PARAMETER_HEADER}
  COMMAND ${GENERATE_PARAMETER_LIBRARY_CPP}
    ${GENERATED_PARAMETER_HEADER}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mecanum_drive_controller_parameter.yaml
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/mecanum_drive_controller_parameter.yaml
  COMMENT "Generating mecanum drive controller parameter header"
  VERBATIM
)
add_custom_target(mecanum_drive_controller_parameters
  DEPENDS ${GENERATED_PARAMETER_HEADER}
)

# Create controller library
add_library(mecanum_drive_controller SHARED
  src/mecanum_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

target_compile_features(mecanum_drive_controller PUBLIC cxx_std_17)

target_include_directories(mecanum_drive_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include/mecanum_drive_controller>
)

add_dependencies(mecanum_drive_controller mecanum_drive_controller_parameters)

target_link_libraries(mecanum_drive_controller PUBLIC
  fmt::fmt
  parameter_traits::parameter_traits
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rsl::rsl
  tcb_span::tcb_span
  tl_expected::tl_expected
)

ament_target_dependencies(mecanum_drive_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Visibility macros
target_compile_definitions(mecanum_drive_controller PRIVATE "MECANUM_DRIVE_CONTROLLER_BUILDING_DLL")

# Export plugin
pluginlib_export_plugin_description_file(controller_interface mecanum_drive_plugin.xml)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Test file directory definition
  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")

  # Controller loading test
  #ament_add_gmock(test_load_mecanum_drive_controller
  #  test/test_load_mecanum_drive_controller.cpp)
  #ament_target_dependencies(test_load_mecanum_drive_controller
  #  controller_manager
  #  ros2_control_test_assets
  #)
endif()

# Install
install(
  DIRECTORY include/
  DESTINATION include/mecanum_drive_controller
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION include
)

install(TARGETS mecanum_drive_controller
  EXPORT export_mecanum_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Export
ament_export_targets(export_mecanum_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
