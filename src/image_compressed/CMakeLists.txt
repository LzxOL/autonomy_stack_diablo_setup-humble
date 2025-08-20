cmake_minimum_required(VERSION 3.22)
project(image_compressed)

# 必须包含这两行用于 ROS 2 和 C++
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_compressed_node src/image_compressed_node.cpp)

ament_target_dependencies(image_compressed_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(image_compressed_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS image_compressed_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
