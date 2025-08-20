import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from tita_locomotion_interfaces.msg import LocomotionCmd

class MultiTypeSubscriber(Node):
    def __init__(self):
        super().__init__('multi_type_subscriber')

        
        self.subscription_motion = self.create_subscription(
            MotionCtrl,  
            '/tita4264886/command/active/command', 
            self.listener_callback_motion,  
            10)  

        self.subscription_locomotion = self.create_subscription(
            LocomotionCmd,
            '/tita4264886/command/active/command',
            self.listener_callback_locomotion,
            10)

    def listener_callback_motion(self, msg):

        self.get_logger().info(f"Received MotionCtrl message: {msg}")

    def listener_callback_locomotion(self, msg):

        self.get_logger().info(f"Received LocomotionCmd message: {msg}")

def main(args=None):

    rclpy.init(args=args)


    multi_type_subscriber = MultiTypeSubscriber()

    
    rclpy.spin(multi_type_subscriber)

    
    multi_type_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
