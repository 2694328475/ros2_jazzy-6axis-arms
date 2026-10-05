cmake_minimum_required(VERSION 3.8)
project(ar3_hardware_drivers)

set(CMAKE_CXX_STANDARD 14)

set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)

set(LIBRARY_NAME ar3_hardware_drivers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${LIBRARY_NAME} SHARED
  src/TeensyDriver.cpp
)

set_target_properties(${LIBRARY_NAME} PROPERTIES
  OUTPUT_NAME ${LIBRARY_NAME}
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${LIBRARY_NAME}
  rclcpp
  std_msgs
)

install(TARGETS ${LIBRARY_NAME}
  EXPORT ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(${LIBRARY_NAME})
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs)

ament_package()