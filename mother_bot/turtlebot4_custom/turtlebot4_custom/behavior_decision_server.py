import rclpy as rp
from rclpy.action import ActionServer
from rclpy.node import Node

from turtlebot4_custom_msgs.action import Behavior
from turtlebot4_custom_msgs.msg import LaneDetection
from turtlebot4_custom_msgs.msg import ObjectDetection
from geometry_msgs.msg import Twist

class BehaviorServer(Node):
    def __init__(self):
        super().__init__('behavior_decision_server')
        self.action_server = ActionServer(
            self,
            Behavior,
            'behavior_decision',
            self.execute_callback
        )

        self.subscription = self.create_subscription(
            LaneDetection,
            '/lane_det_result',
            self.callback_lane,
            10
        )

        self.subscription = self.create_subscription(
            ObjectDetection,
            '/obj_det_result',
            self.callback_obj,
            10
        )

        self.publisher = self.create_publisher(Twist, '/turtlebot4/cmd_vel', 10)

    def execute_callback(self, goal_handle):
        goal_handle.succeed()
        result = Behavior.Result()
        return result

    def callback_lane(self, msg):
        print("TEST LINE10")

    def callback_obj(self, msg):
        print("TEST LINE20")
    
def main(args=None):
    rp.init(args=args)
    bd_server = BehaviorServer()
    rp.spin(bd_server)

if __name__ == '__main__':
    main()