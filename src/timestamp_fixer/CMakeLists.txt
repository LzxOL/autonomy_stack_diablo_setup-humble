cmake_minimum_required(VERSION 3.22)
project(timestamp_fixer)

# 必须包含这两行用于 ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# 安装 Python 脚本
install(PROGRAMS
  src/timestamp_fixer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
