cmake_minimum_required(VERSION 3.16)
project(hello_world)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(hello_node_component SHARED
  src/hello_node.cpp
)

ament_target_dependencies(hello_node_component
  rclcpp
  rclcpp_components
)

rclcpp_components_register_node(hello_node_component
  PLUGIN "hello_world::hello_node"
  EXECUTABLE hello_node
)

target_include_directories(hello_node_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
  hello_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
