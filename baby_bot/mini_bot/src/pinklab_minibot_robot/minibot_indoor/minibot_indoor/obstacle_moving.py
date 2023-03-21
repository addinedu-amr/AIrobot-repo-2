# 사용하지 않는 코드, Path Planning.py코드가 모두 처리하고 있습니다.
import rclpy as rp
from rclpy.node import Node
from minibot_msgs.msg import IsObstacle
from nav2_simple_commander.robot_navigator import BasicNavigator

class ObstacleMoving(Node):
    
    def __init__(self):
        super().__init__("obstacle_moving")
        self.sub_obstacle = self.create_subscription(IsObstacle, "/obstacle_detect", self.callback_check_obstacle, 10)
        self.timer_period = 0.5
        self.publisher = self.create_publisher(IsObstacle, "/obstacle_detect", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.is_osbstacle = IsObstacle()
        self.is_obstacle_old = False
        self.count_obstacle_time = 0
        self.is_fixed_body = False
        self.original_map = [[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
                             [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                             [1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0]]
        
    # Subscribe obstacle_detect topic
    def callback_check_obstacle(self, msg):
        self.is_osbstacle = msg.check_obstacle
        
        # 장애물이 True로 뜨면 횟수 저장
        if self.is_obstacle_old == True:
            if self.is_osbstacle == self.is_obstacle_old :
                self.count_obstacle_time += 1
            elif self.is_osbstacle != self.is_obstacle_old:
                self.count_obstacle_time = 0
        
        self.is_obstacle_old = self.is_osbstacle
        
        # 장애물이 6초동안 움직이지 않으면 고정된 물체라고 판단함 (topic이 0.5초마다 발행)
        if self.count_obstacle_time >= 12:
            self.is_fixed_body = True
        else:
            self.is_fixed_body = False
        
        
        # 현재 위치를 받아와야함
        
        


def main(args=None):
    rp.init(args=args)
    nav = BasicNavigator()
    
    obstacle_moving = ObstacleMoving()
    rp.spin(obstacle_moving)
    
    obstacle_moving.destroy_node()
    rp.shutdown()
    
if __name__ == "__main__":
    main()
