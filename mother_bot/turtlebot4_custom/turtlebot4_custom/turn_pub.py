import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Rate(Node):
    def __init__(self):
        super().__init__('rate_node')
        self.count = 0
        self.angle = 0
        self.forward = 0
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.5
        #self.timer_turn = self.create_timer(self.timer_period, self.timer_callback_turn)
        self.timer_forward = self.create_timer(self.timer_period, self.timer_callback_forward)

    # def timer_callback_turn(self):
    #     msg = Twist()
    #     if self.angle > 1.57:
    #         return
    #     msg.angular.z = 0.3
    #     self.angle += self.timer_period * msg.angular.z
    #     self.publisher.publish(msg)
        
    def timer_callback_forward(self):
        msg = Twist()
        if self.forward > 0.6:
            return
        msg.linear.x = 0.15
        self.forward += self.timer_period * msg.linear.x
        self.publisher.publish(msg)

def main(args=None):
    rp.init(args=args)

    node = Rate()
    rp.spin(node)

    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()