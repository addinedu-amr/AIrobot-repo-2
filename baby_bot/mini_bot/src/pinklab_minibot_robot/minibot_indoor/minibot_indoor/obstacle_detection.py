import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from minibot_msgs.msg import IsObstacle

class ObstacleDetect(Node):
    
    # /scan topic을 구독하고 장애물이 가까이 있는지 파악해서 알려줌
    def __init__(self):
        super().__init__("obstacle_detector")
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.callback_scan, 10)
        self.timer_period = 0.5
        self.publisher = self.create_publisher(IsObstacle, "/obstacle_detect", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.is_osbstacle = IsObstacle()
        
    def callback_scan(self, msg):
        min_data = min(msg.ranges[170:210])  # ladar의 앞 부분 인식
        self.is_osbstacle.min_distance = min_data
        
        if min_data < 0.2:                    # 0.2m 이하이면 주행 멈추라고 신호 줌
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