cmake_minimum_required(VERSION 3.8)
project(ar3_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ar3_hardware_drivers REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/ar3_hardware_interface.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_srvs
  ar3_hardware_drivers
)

pluginlib_export_plugin_description_file(hardware_interface ar3_hardware_interface.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_srvs
  ar3_hardware_drivers
)

ament_package()