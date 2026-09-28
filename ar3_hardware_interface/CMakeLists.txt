cmake_minimum_required(VERSION 3.8)
project(ar3_hardware_interface)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ar3_hardware_drivers REQUIRED)

# 包含头文件目录
include_directories(
  include
)

# 创建库
add_library(${PROJECT_NAME} SHARED
  src/ar3_hardware_interface.cpp
)

# 依赖项
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_srvs
  ar3_hardware_drivers
)

# 导出插件
pluginlib_export_plugin_description_file(hardware_interface ar3_hardware_interface.xml)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 安装库
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 导出依赖信息
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

# 打包
ament_package()