from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='type_converter',  # 替换为你的包名
            executable='float64_to_float32_converter',  # 替换为你的可执行文件名
            name='float_converter',
            output='screen',
            remappings=[
                ('/tita4264886/robot_vel', '/tita4264886/robot_vel'),  # 可选：自定义话题重映射
                ('/tita4264886/robot_vel_32', '/tita4264886/robot_vel_32')
            ]
        )
    ])
