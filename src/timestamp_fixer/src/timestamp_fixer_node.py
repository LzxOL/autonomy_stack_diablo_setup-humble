#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import importlib

class TimestampFixer(Node):
    def __init__(self):
        super().__init__('timestamp_fixer_node')
        
        # 声明参数以便动态配置话题名称列表
        self.declare_parameter('input_topics', ['/right/image_raw', '/left/image_raw'])
        self.declare_parameter('output_topics', ['/right/image_raw_fixed', '/left/image_raw_fixed'])
        self.declare_parameter('msg_types', ['sensor_msgs/msg/Image', 'sensor_msgs/msg/Image'])  # 新增参数

        # 获取参数值
        input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        output_topics = self.get_parameter('output_topics').get_parameter_value().string_array_value
        msg_types = self.get_parameter('msg_types').get_parameter_value().string_array_value

        # 处理空参数的情况
        if not input_topics:
            input_topics = ['/right/image_raw', '/left/image_raw']
            self.get_logger().warn('No input_topics specified, using default values')
        
        if not output_topics:
            output_topics = ['/right/image_raw_fixed', '/left/image_raw_fixed']
            self.get_logger().warn('No output_topics specified, using default values')
        
        if not msg_types or len(msg_types) != len(input_topics):
            msg_types = ['sensor_msgs/msg/Image'] * len(input_topics)
            self.get_logger().warn('No msg_types specified or count mismatch, using Image for all.')

        # 检查输入输出话题数量是否匹配
        if len(input_topics) != len(output_topics) or len(input_topics) != len(msg_types):
            self.get_logger().error(
                f'Input/output/msg_types count mismatch: {len(input_topics)}, {len(output_topics)}, {len(msg_types)}'
            )
            return

        # 存储订阅者和发布者 - 修改变量名避免冲突
        self.topic_subscriptions = []
        self.topic_publishers = {}
        self.topic_types = {}

        # 为每个话题对创建订阅者和发布者
        for i, (input_topic, output_topic, msg_type_str) in enumerate(zip(input_topics, output_topics, msg_types)):
            # 动态导入消息类型
            pkg, _, msg = msg_type_str.partition('/msg/')
            if not pkg or not msg:
                self.get_logger().error(f'Invalid msg_type: {msg_type_str}')
                continue
            msg_module = importlib.import_module(f'{pkg}.msg')
            msg_class = getattr(msg_module, msg)
            self.topic_types[input_topic] = msg_class

            # 创建发布者
            publisher = self.create_publisher(msg_class, output_topic, 10)
            self.topic_publishers[input_topic] = publisher
            
            # 创建订阅者，使用lambda捕获当前的input_topic
            subscription = self.create_subscription(
                msg_class,
                input_topic,
                lambda msg, topic=input_topic: self.listener_callback(msg, topic),
                10)
            self.topic_subscriptions.append(subscription)
            
            self.get_logger().info(f'Topic pair {i+1}: {input_topic} ({msg_type_str}) -> {output_topic}')
        
        self.get_logger().info(f'Timestamp Fixer node started with {len(input_topics)} topic pairs.')
        
        # 统计信息
        self.message_counts = {topic: 0 for topic in input_topics}

    def listener_callback(self, msg, input_topic):
        """
        时间戳修正回调函数
        将接收到的消息的时间戳替换为当前系统时间
        """
        # 兼容不同消息类型的 header
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            current_time = self.get_clock().now().to_msg()
            original_timestamp = msg.header.stamp
            msg.header.stamp = current_time
        else:
            self.get_logger().warn(f'Message on {input_topic} has no header.stamp, skipping')
            return

        # 发布修正后的消息
        if input_topic in self.topic_publishers:
            self.topic_publishers[input_topic].publish(msg)
        
        # 统计和调试信息
        self.message_counts[input_topic] += 1
        if self.message_counts[input_topic] % 100 == 0:  # 每100条消息打印一次
            self.get_logger().debug(f'Processed {self.message_counts[input_topic]} messages from {input_topic}')
            self.get_logger().debug(
                f'[{input_topic}] Original: {original_timestamp.sec}.{original_timestamp.nanosec:09d}, '
                f'Fixed: {current_time.sec}.{current_time.nanosec:09d}'
            )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        timestamp_fixer = TimestampFixer()
        rclpy.spin(timestamp_fixer)
    except KeyboardInterrupt:
        pass
    finally:
        if 'timestamp_fixer' in locals():
            timestamp_fixer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
