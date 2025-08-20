from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_compressed',
            executable='image_compressed_node',
            name='image_compressed_node',
            output='screen',
            remappings=[
                ('/left/image_raw', '/left/image_raw'),
                ('/right/image_raw', '/right/image_raw'),
                ('/left/image_raw/compressed', '/left/image_raw/compressed'),
                ('/right/image_raw/compressed', '/right/image_raw/compressed')
            ]
        )
    ])