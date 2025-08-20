#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        
        # 创建时钟发布者
        self.publisher_ = self.create_publisher(Clock, '/clock', 10)
        
        # 创建定时器，100Hz频率发布时钟
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info('Clock publisher node started, publishing at 100Hz')

    def timer_callback(self):
        """定时器回调函数，发布当前时间"""
        msg = Clock()
        msg.clock = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        
        # 可选：定期打印调试信息
        # self.get_logger().debug(f'Published clock: {msg.clock.sec}.{msg.clock.nanosec}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        clock_publisher = ClockPublisher()
        rclpy.spin(clock_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'clock_publisher' in locals():
            clock_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
