cmake_minimum_required(VERSION 3.22)
project(pointcloud_intensity_converter)

# 必须包含这两行用于 ROS 2 和 C++
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
# 添加TF2相关依赖
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
# 添加Eigen3依赖
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(pointcloud_intensity_node src/pointcloud_intensity_node.cpp)

ament_target_dependencies(pointcloud_intensity_node
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen
)

target_link_libraries(pointcloud_intensity_node
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

install(TARGETS pointcloud_intensity_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
