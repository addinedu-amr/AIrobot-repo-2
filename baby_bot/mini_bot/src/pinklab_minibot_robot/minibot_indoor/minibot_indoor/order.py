import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from minibot_msgs.msg import StartEnd

class Publisher(Node):
    def __init__(self):
        super().__init__('test_pub')
        self.publisher_ = self.create_publisher(StartEnd, 'test_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = StartEnd()
        msg.end_x = 11
        msg.end_y = 2
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
    
def main(args=None):
    rclpy.init(args=args)
    test_pub = Publisher()
    rclpy.spin(test_pub)
    test_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()