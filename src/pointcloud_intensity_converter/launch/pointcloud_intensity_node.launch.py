from launch import LaunchDescription
from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='pointcloud_intensity_converter',  # ← 替换为你的实际包名
#             executable='pointcloud_intensity_node',  # ← 和CMake中add_executable一致
#             name='pointcloud_intensity_node',
#             output='screen',
#             remappings=[
#                 ('/camera/depth_registered/points', '/camera/depth_registered/points'),  # 可根据需要重映射输入话题
#                 ('registered_scan', 'registered_scan')  # 可自定义输出话题名
#             ]
#         )
#     ])
