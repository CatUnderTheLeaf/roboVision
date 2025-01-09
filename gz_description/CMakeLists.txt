cmake_minimum_required(VERSION 3.16)
project(gz_description LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  gz_description
  SHARED
  src/gz_legobot_system.cpp
)
target_compile_features(gz_description PUBLIC cxx_std_17)
target_include_directories(gz_description PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/gz_description>
)
ament_target_dependencies(
  gz_description PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface gz_ros2_control.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/gz_description
)
install(
  DIRECTORY models launch
  DESTINATION share/gz_description
)

install(TARGETS gz_description
  EXPORT export_gz_description
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  #ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  #ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
  #ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_gz_description HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()