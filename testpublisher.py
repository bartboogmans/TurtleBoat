import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(String, 'dsadsa', 10)
        timer_period = 1.0 / 5.0
        self.timer_ = self.create_timer(timer_period, self.publish_message)
        self.i = 0

    def publish_message(self):
        msg = String()
        msg.data = 'derp'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
