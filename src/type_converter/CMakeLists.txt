cmake_minimum_required(VERSION 3.22)
project(type_converter)

# 查找所需包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 添加可执行文件
add_executable(float64_to_float32_converter
  src/float64_to_float32_converter.cpp)

# 添加依赖
ament_target_dependencies(float64_to_float32_converter
  rclcpp
  std_msgs
)

# 安装可执行文件
install(TARGETS float64_to_float32_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 声明这是一个 ROS 2 包
ament_package()
