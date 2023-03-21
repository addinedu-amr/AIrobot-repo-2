import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from minibot_msgs.msg import IsObstacle
import numpy as np

class ObstacleDetect(Node):
    
    # /scan topic을 구독하고 장애물이 가까이 있는지 파악해서 알려줌
    def __init__(self):
        super().__init__("obstacle_detector")
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.callback_scan, qos_profile)
        self.timer_period = 0.5
        self.publisher = self.create_publisher(IsObstacle, "/obstacle_detect", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.is_osbstacle = IsObstacle()
        
    def callback_scan(self, msg):
        min_data =  msg.ranges[110:150]  # ladar의 앞 부분 인식
        try:
            min_data = np.average(min_data.remove(0.0))
        except:
            min_data = np.average(min_data)
        self.is_osbstacle.min_distance = float(min_data)
        
        if min_data < 0.3:                    # 0.35m 이하이면 주행 멈추라고 신호 줌
            self.is_osbstacle.check_obstacle = True
        else:
            self.is_osbstacle.check_obstacle = False

    def timer_callback(self):
        self.publisher.publish(self.is_osbstacle)

def main(args=None):
    rp.init(args=args)
    
    obstacle_detector = ObstacleDetect()
    rp.spin(obstacle_detector)
    
    obstacle_detector.destroy_node()
    rp.shutdown()
    
if __name__ == "__main__":
    main()