from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('timestamp_fixer')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'timestamp_fixer_params.yaml'),
        description='Full path to timestamp fixer config file'
    )
    
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='timestamp_fixer',
        description='Node name for parameter namespace'
    )
    
    # 时间戳修正节点
    timestamp_fixer_node = Node(
        package='timestamp_fixer',
        executable='timestamp_fixer_node.py',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    return LaunchDescription([
        config_file_arg,
        node_name_arg,
        timestamp_fixer_node
    ])
