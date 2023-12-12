cmake_minimum_required(VERSION 3.14)
project(keyboard_ackermann_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(keyboard_ackermann_controller_node SHARED
  src/keyboard_ackermann_controller/keyboard_ackermann_controller_node.cpp
)

rclcpp_components_register_node(keyboard_ackermann_controller_node
  PLUGIN "keyboard_ackermann_controller::AutowareKBAckControllerNode"
  EXECUTABLE keyboard_ackermann_controller
)

ament_auto_package(
  INSTALL_TO_SHARE
)
