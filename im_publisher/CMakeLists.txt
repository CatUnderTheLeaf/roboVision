cmake_minimum_required(VERSION 3.5)
project(im_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(camera_info_manager REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio)
message(STATUS "opencv version ${OpenCV_VERSION}")

ament_auto_add_library(im_publisher SHARED src/im_publisher.cpp)
target_link_libraries(im_publisher ${OpenCV_LIBRARIES} camera_info_manager::camera_info_manager)
rclcpp_components_register_nodes(im_publisher "${PROJECT_NAME}::ImagePublisher")
set(node_plugins "${node_plugins}${PROJECT_NAME}::ImagePublisher;$<TARGET_FILE:ImagePublisher>\n")

ament_auto_add_executable(im_publisher_node src/im_publisher_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
