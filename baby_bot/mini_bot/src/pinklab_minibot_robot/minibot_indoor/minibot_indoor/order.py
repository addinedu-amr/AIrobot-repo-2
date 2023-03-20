import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from minibot_msgs.msg import StartEnd
from minibot_msgs.msg import NextMission

class Publisher(Node):
    def __init__(self):
        super().__init__('test_pub')
        self.publisher_ = self.create_publisher(StartEnd, 'Robot_order', 10)
        self.subscriber = self.create_subscription(NextMission, 'Mission_complete', self.mission_callback, 10 )
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.end_point_x = [3, 9, 11]
        self.end_point_y = [2, 0, 2]

        self.end_point_count = 0
        self.now_status = False


    def timer_callback(self):
        if len(self.end_point_x) > self.end_point_count:
            msg = StartEnd()
            msg.end_x = self.end_point_x[self.end_point_count]
            msg.end_y = self.end_point_y[self.end_point_count]
            self.publisher_.publish(msg)

        elif len(self.end_point_x) == self.end_point_count:
            print("집(0,1)으로 이동을 시작합니다." )
            msg = StartEnd()
            msg.end_x = 0
            msg.end_y = 1
            self.publisher_.publish(msg)

    def mission_callback(self, msg):
        if msg.mission == True:
            self.now_status = True
        elif msg.mission == False and self.now_status == True:
            self.now_status = False
            if len(self.end_point_x) > self.end_point_count:
                print("명령을 완료했습니다. 다음 명령을 진행합니다.")
                self.end_point_count = self.end_point_count + 1
                print(str(self.end_point_x) + "," + str(self.end_point_y) + "로 이동을 시작합니다." )
            else:
                print("모든 명령을 완료했습니다. 집으로 복귀 시킵니다.")

    
def main(args=None):
    rclpy.init(args=args)
    test_pub = Publisher()
    rclpy.spin(test_pub)
    test_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()