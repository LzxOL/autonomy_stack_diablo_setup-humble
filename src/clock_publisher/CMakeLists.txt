cmake_minimum_required(VERSION 3.22)
project(clock_publisher)

# 必须包含这两行用于 ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# 安装 Python 脚本
install(PROGRAMS
  src/clock_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
