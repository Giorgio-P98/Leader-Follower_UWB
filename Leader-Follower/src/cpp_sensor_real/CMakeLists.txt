cmake_minimum_required(VERSION 3.8)
project(cpp_sensor_real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mymsg_msgs REQUIRED)

add_executable(sensor_real_cpp src/sensor_real_cpp.cpp)
ament_target_dependencies(sensor_real_cpp rclcpp mymsg_msgs)

install(TARGETS
  sensor_real_cpp
  DESTINATION lib/${PROJECT_NAME})

ament_package()
