import rclpy as rp
from rclpy.node import Node

from sensor_msgs.msg import Image
from turtlebot4_custom_msgs.msg import LaneDetection

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.callback_color,
            10
        )
        self.publisher = self.create_publisher(LaneDetection, '/lane_det_result', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def callback_color(self, msg):
        print("TEST LINE")
        print("차선 판단하는 코드")

    def timer_callback(self):
        print("pub")


def main(args=None):
    rp.init(args=args)

    lane_detector = LaneDetector()
    rp.spin(lane_detector)

    lane_detector.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()