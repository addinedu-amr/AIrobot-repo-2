import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from turtlebot4_custom_msgs.action import Behavior
from turtlebot4_custom_msgs.msg import ObjectDetection
from geometry_msgs.msg import Twist
import time

class LaneSubscriber(Node):
    def __init__(self):
        super().__init__('lane_ret_subscriber')
        self.linear_x = 0
        self.angular_z = 0
        self.subscription = self.create_subscription(
            Twist,
            '/lane_det_result',
            self.callback_lane,
            10
        )

    def callback_lane(self, msg):
        self.linear_x = msg.linear_x
        self.angular_z = msg.angular_z
        self.lane_num = msg.lane_num

class ObjectSubscriber(Node):
    def __init__(self):
        super().__init__('obj_ret_subscriber')
        self.pixel_x = -10
        self.pixel_y = -10
        self.distance = 0
        self.subscription = self.create_subscription(
            ObjectDetection,
            '/obj_det_result',
            self.callback_obj,
            10
        )

    def callback_obj(self, msg):
        self.pixel_x = msg.pixel_x
        self.pixel_y = msg.pixel_y
        self.distance = msg.distance

class BehaviorServer(Node):
    def __init__(self, sub1, sub2):
        super().__init__('behavior_decision_server')
        self.sb_obj = sub1
        self.sb_lane = sub2
        self.action_server = ActionServer(
            self,
            Behavior,
            'behavior_decision',
            self.execute_callback
        )

        self.publisher_lane = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_obs = self.create_publisher(Twist, '/cmd_vel', 10)

    def execute_callback(self, goal_handle):
        if not goal_handle.request.boot == 'start':
            self.get_logger().info("'start'를 입력해야합니다.")
            return 0
        
        feedback_msg = Behavior.Feedback()
        pub_msg = Twist()
        while True:
            # 장애물 회피
            if (self.sb_obj.pixel_x > 270 and self.sb_obj.pixel_x < 370) and (self.sb_obj.distance > 30 and self.sb_obj.distance < 70):
                #1차선
                print(self.lane_num)
                if True:
                    start_time = time.time()
                    while time.time() - start_time < 5:
                        pub_msg.angular.z = 0.2
                        pub_msg.linear.x = 0.0
                        self.publisher_obs.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        pub_msg.angular.z = 0.0
                        pub_msg.linear.x = 0.2
                        self.publisher_obs.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 5:
                        pub_msg.angular.z = -0.2
                        pub_msg.linear.x = 0.0
                        self.publisher_obs.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        pub_msg.angular.z = 0.0
                        pub_msg.linear.x = 0.2
                        self.publisher_obs.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        pub_msg.angular.z = -0.2
                        pub_msg.linear.x = 0.0
                        self.publisher_obs.publish(pub_msg)

                else:
                    # 2차선
                    start_time = time.time()
                    while time.time() - start_time < 5:
                        pub_msg.angular.z = -0.2
                        pub_msg.linear.x = 0.0
                        self.publisher_obs.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        pub_msg.angular.z = 0.0
                        pub_msg.linear.x = 0.2
                        self.publisher_obs.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 5:
                        pub_msg.angular.z = 0.2
                        pub_msg.linear.x = 0.0
                        self.publisher_obs.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        pub_msg.angular.z = 0.0
                        pub_msg.linear.x = 0.2
                        self.publisher_obs.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        pub_msg.angular.z = 0.2
                        pub_msg.linear.x = 0.0
                        self.publisher_obs.publish(pub_msg)
            else:
                # 일반주행
                pub_msg.linear.x = float(self.sb_lane.linear_x)
                pub_msg.angular.z = float(self.sb_lane.angular_z)
                self.publisher_lane.publish(pub_msg)

            # 도착 신호
            if False:
                break

            time.sleep(0.05)
        
        goal_handle.succeed()
        result = Behavior.Result()

        result.lane = str(self.lane_num)
        return result

def main(args=None):
    rp.init(args=args)

    obj_sub = ObjectSubscriber()
    lane_sub = LaneSubscriber()
    bd_server = BehaviorServer(obj_sub, lane_sub)
    

    executor = MultiThreadedExecutor()

    executor.add_node(bd_server)
    executor.add_node(obj_sub)
    executor.add_node(lane_sub)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        bd_server.destroy_node()
        obj_sub.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
