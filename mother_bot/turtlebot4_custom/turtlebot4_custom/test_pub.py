import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import Int16

class Rate(Node):
    def __init__(self):
        super().__init__('rate_node')
        self.count = 0

        self.publisher = self.create_publisher(Int16, '/rate', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int16()
        self.count += 1
        msg.data = self.count
        self.publisher.publish(msg)
        
def main(args=None):
    rp.init(args=args)

    node = Rate()
    rp.spin(node)

    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()