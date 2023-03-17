import rclpy as rp
from rclpy.node import Node
from minibot_msgs.msg import IsObstacle

class Obstacle_Subscriber(Node):
    def __init__(self):
        super().__init__('turtlesim_subscriber') # Node 클래스에 다음 문자열 이름의 생성자를 거쳐온다.
        self.subscription = self.create_subscription(
            IsObstacle,
            '/obstacle_detect',
            self.callback,
            10
        ) # 이전 장에서 했던 이야기, create_subscription으로 만들었다.
        self.subscription

    def callback(self, msg):
        print("move : ", msg.check_obstacle, ", distance : ", msg.min_distance)
        # 데이터가 도착했을 때 출력하는 callback함수


def main(args=None):
    # args = None 아무런 의미가 없는말이다.
    rp.init(args=args)
    # init 에 뭔가를 넣을 수 있으니 만약에 하고 싶다면 이걸 수정해라 라고 해서 args = None이라고 한거다.

    turtlesim_subscriber = Obstacle_Subscriber()
    rp.spin(turtlesim_subscriber)
    # 객체를 생성해서 명령을 하라고 하고 꺼지면

    turtlesim_subscriber.destroy_node()
    rp.shutdown()
    # 종료해달라

if __name__ == '__main__':
    main()