import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import random
from std_msgs.msg import Header

class RandomPublisher(Node):
    def __init__(self):
        super().__init__('random_point_publisher')
        self.publisher_ = self.create_publisher(PointStamped, '/goal_point', 10)
        self.timer = self.create_timer(1.0, self.publish_random_point)

    def publish_random_point(self):
        msg = PointStamped()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.point.x = random.uniform(0.0, 20.0)  # 生成一个0到20之间的随机x
        msg.point.y = random.uniform(0.0, 20.0)  # 生成一个0到20之间的随机y
        msg.point.z = random.uniform(-5.0, 5.0)  # 生成一个-5到5之间的随机z

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}")

def main(args=None):
    rclpy.init(args=args)
    random_publisher = RandomPublisher()
    rclpy.spin(random_publisher)
    random_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
