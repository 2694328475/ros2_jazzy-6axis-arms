cmake_minimum_required(VERSION 3.8)
project(ar3_hardware_drivers)

# 默认使用 C++14
set(CMAKE_CXX_STANDARD 14)

# 强制生成共享库
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)

# 设置输出库名
set(LIBRARY_NAME ar3_hardware_drivers)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 添加库 - 明确使用 SHARED
add_library(${LIBRARY_NAME} SHARED
  src/TeensyDriver.cpp
)

# 确保库名正确
set_target_properties(${LIBRARY_NAME} PROPERTIES
  OUTPUT_NAME ${LIBRARY_NAME}
  POSITION_INDEPENDENT_CODE ON
)

# 包含目录
target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 添加ament依赖
ament_target_dependencies(${LIBRARY_NAME}
  rclcpp
  std_msgs
)

# 安装目标 - 确保安装.so文件
install(TARGETS ${LIBRARY_NAME}
  EXPORT ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 导出
ament_export_libraries(${LIBRARY_NAME})
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs)

ament_package()