cmake_minimum_required(VERSION 3.5)
project(imu_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(imu_fusion_node src/imu_fusion.cpp)
ament_target_dependencies(imu_fusion_node rclcpp sensor_msgs message_filters)

target_include_directories(imu_fusion_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

install(TARGETS
  imu_fusion_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
