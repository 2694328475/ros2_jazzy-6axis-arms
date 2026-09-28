cmake_minimum_required(VERSION 3.8)
project(ar3_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装 launch 文件夹
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 worlds 文件夹
install(DIRECTORY world
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()