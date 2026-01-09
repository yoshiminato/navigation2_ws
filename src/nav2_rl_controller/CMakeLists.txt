cmake_minimum_required(VERSION 3.8)
project(nav2_rl_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(nav2_rl_controller SHARED
  src/rl_controller.cpp
)

ament_target_dependencies(nav2_rl_controller
  rclcpp rclcpp_lifecycle nav2_core nav2_util nav_msgs geometry_msgs nav2_costmap_2d tf2_ros pluginlib
)

# install targets
install(TARGETS nav2_rl_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES rl_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core rl_controller_plugin.xml)

ament_package()

