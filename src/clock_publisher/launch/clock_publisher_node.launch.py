from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clock_publisher',
            executable='clock_publisher_node.py',
            name='clock_publisher',
            output='screen',
            parameters=[
                # 可以在这里添加参数，例如发布频率
                # {'publish_rate': 100.0}
            ]
        )
    ])
