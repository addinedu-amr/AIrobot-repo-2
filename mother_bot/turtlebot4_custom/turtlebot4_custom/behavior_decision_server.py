import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from turtlebot4_custom_msgs.action import Behavior
from turtlebot4_custom_msgs.msg import ObjectDetection
from turtlebot4_custom_msgs.msg import LaneDetection
from turtlebot4_custom_msgs.msg import MarkerDetection
from geometry_msgs.msg import Twist
import time

class MarkerSubscriber(Node):
    def __init__(self):
        super().__init__('marker_ret_subscriber')
        self.marker_x = 0.0
        self.marker_y = 0.0
        self.marker_dist = 0.0
        self.subscription = self.create_subscription(
            MarkerDetection,
            '/marker_det_result',
            self.callback_marker,
            10
        )

    def callback_marker(self, msg):
        self.marker_x = msg.x
        self.marker_y = msg.y
        self.marker_dist = msg.distance

class LaneSubscriber(Node):
    def __init__(self):
        super().__init__('lane_ret_subscriber')
        self.linear_x = 0
        self.angular_z = 0
        self.lane_num = 0
        self.subscription = self.create_subscription(
            LaneDetection,
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
    def __init__(self, sub1, sub2, sub3):
        super().__init__('behavior_decision_server')
        self.sb_obj = sub1
        self.sb_lane = sub2
        self.sb_marker = sub3
        self.lane_num = 0
        self.is_lane_num_det = False
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
        
        if not self.is_lane_num_det:
            self.lane_num = self.sb_lane.lane_num
            self.is_lane_num_det = True

        feedback_msg = Behavior.Feedback()
        pub_msg = Twist()
        while True:
            # 장애물 회피
            if (self.sb_obj.pixel_x > 270 and self.sb_obj.pixel_x < 370) and (self.sb_obj.distance > 0 and self.sb_obj.distance < 180):
                feedback_msg.status = "장애물 회피중"
                #2차선
                #print(self.lane_num)
                if self.lane_num == 2:
                    start_time = time.time()
                    while time.time() - start_time < 1.5:
                        pub_msg.angular.z = 0.2
                        pub_msg.linear.x = 0.0
                        self.publisher_obs.publish(pub_msg)
                    self.lane_num = 1
                elif self.lane_num == 1:
                    #1차선
                    start_time = time.time()
                    while time.time() - start_time < 1.5:
                        pub_msg.angular.z = -0.2
                        pub_msg.linear.x = 0.0
                        self.publisher_obs.publish(pub_msg)
                else:
                    pass
            
            # Aruco Marker
            if self.sb_marker.marker_dist > 0:
                feedback_msg.status = "스테이션 도킹중"
                if self.sb_marker.marker_dist > 37:
                    pub_msg.linear.x = 0.03
                    pub_msg.angular.z = self.compensate_x(320, self.sb_marker.marker_x*10)
                    print(pub_msg.angular.z)
                    self.publisher_lane.publish(pub_msg)
                elif self.sb_marker.marker_dist <= 37 and self.sb_marker.marker_dist > 34:
                    pub_msg.linear.x = 0.0
                    pub_msg.angular.z = -self.compensate_x(320, self.sb_marker.marker_x*10)
                    self.publisher_lane.publish(pub_msg)
                    print(self.sb_marker.marker_x)
                    # if (320 + self.sb_marker.marker_x*10 < 390) and (320 + self.sb_marker.marker_x*10 > 345):
                    if (self.sb_marker.marker_x < 3) and (self.sb_marker.marker_x > 2):
                        print('---------------------------------------')
                        print('------------------END------------------')
                        print('---------------------------------------')
                        pub_msg.linear.x = 0.0
                        pub_msg.angular.z = 0.0
                        self.publisher_lane.publish(pub_msg)
                        print(self.sb_marker.marker_x)
                        break
                else:
                    pass

            else:
                # 일반주행
                pub_msg.angular.z = float(self.sb_lane.angular_z)
                pub_msg.linear.x = float(self.sb_lane.linear_x)
                self.publisher_lane.publish(pub_msg)
                feedback_msg.status = "현재 주행중"

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.01)
        
        goal_handle.succeed()
        result = Behavior.Result()
        #result.lane = str(self.lane_num)
        result.dock = "**********Docking Done************"
        return result
    
    def compensate_x(self, center_x, target_x):    
        p_x = 0.00008
        center_x = center_x
        target_x = target_x
        e_x = center_x - target_x
        control_x = p_x * e_x
        return control_x 

def main(args=None):
    rp.init(args=args)

    obj_sub = ObjectSubscriber()
    lane_sub = LaneSubscriber()
    marker_sub = MarkerSubscriber()
    bd_server = BehaviorServer(obj_sub, lane_sub, marker_sub)
    

    executor = MultiThreadedExecutor()

    executor.add_node(bd_server)
    executor.add_node(obj_sub)
    executor.add_node(lane_sub)
    executor.add_node(marker_sub)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        bd_server.destroy_node()
        obj_sub.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
