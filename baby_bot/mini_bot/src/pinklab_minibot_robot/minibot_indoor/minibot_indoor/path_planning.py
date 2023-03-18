from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from minibot_indoor import image_processing
from minibot_indoor import Astar
import rclpy as rp
import threading
from rclpy.node import Node
from std_msgs.msg import String
from minibot_msgs.msg import IsObstacle
from minibot_msgs.msg import StartEnd

rp.init()
nav = BasicNavigator()

# 장애물을 탐지하는 부분..
class Subscriber(Node):
    def __init__(self):
        super().__init__('test_sub')
        self.callback_group = rp.callback_groups.ReentrantCallbackGroup()
        self.subscription1 = self.create_subscription(IsObstacle, '/obstacle_detect', self.obstacle_callback, 10, callback_group=self.callback_group)
        self.subscription2 = self.create_subscription(StartEnd, 'Robot_order', self.test_callback, 10)

        self.order_start = False
        self.path_count = 1
        self.path = None
        self.now_location = (1, 0)

        self.pre_order_x = None
        self.pre_order_y = None
        
        self.result = None
        self.check_obstacle = None

    def obstacle_callback(self, msg):
        threading.Thread(target=self.process_message_b, args=(msg,)).start()

    def process_message_b(self, msg):
        self.check_obstacle = msg.check_obstacle
        print("move : ", msg.check_obstacle, ", distance : ", msg.min_distance)

    def test_callback(self, msg):
        print("함수를 새로시작합니다.")
        if self.order_start == False:
            if self.pre_order_y == msg.end_y and self.pre_order_x == msg.end_x:
                pass
            else:
                self.result, self.path = my_robot_path("/home/du/mini_bot/baby_map.pgm", "/home/du/mini_bot/baby_map.yaml", self.now_location, (msg.end_y, msg.end_x) )
                self.pre_order_y = msg.end_y
                self.pre_order_x = msg.end_x
                self.order_start = True

        # 그래 여기까지는 괜찮아 이 밑에가 lifecycle이 길어 
        if self.path != None:
            if len(self.path) > self.path_count:
                self.now_location = go_my_robot(get_my_map_coordinate(), self.now_location, self.path[self.path_count])
                self.path_count = self.path_count + 1
            else:
                self.order_start = False
                self.path = None
                self.path_count = 1
        else:
            if self.order_start == True:
                print("이동할 수 없는 지점으로 명령하고 있습니다.")
                self.order_start = False

    def spin(self):
        rp.spin(self)



def my_robot_path(pgm_path, yaml_path, start, end):
    image = image_processing.pgm_to_matrix(pgm_path, yaml_path ,3 , 12, -1.1, 2.2)
    image.run()
    my_map = image.matrix
    make_route = Astar.Astar()
    result, path = make_route.run(my_map , start, end)
    return result, path

def go_my_robot(my_map_coordinate, start, end):
    print("명령을 이수합니다." + str(start) + "좌표로 이동합니다." )
    #북쪽
    if end[1] - start[1] == 1:
        my_map_coordinate[end[0]][end[1]].pose.orientation.x = 0.0
        my_map_coordinate[end[0]][end[1]].pose.orientation.y = 0.0
        my_map_coordinate[end[0]][end[1]].pose.orientation.z = 0.0
        my_map_coordinate[end[0]][end[1]].pose.orientation.w = 1.0
    
    #남쪽
    elif end[1] - start[1] == -1:
        my_map_coordinate[end[0]][end[1]].pose.orientation.x = 0.0
        my_map_coordinate[end[0]][end[1]].pose.orientation.y = 0.0
        my_map_coordinate[end[0]][end[1]].pose.orientation.z = 0.0
        my_map_coordinate[end[0]][end[1]].pose.orientation.w = -1.0
    
    # 서쪽
    elif end[0] - start[0] == -1:
        my_map_coordinate[end[0]][end[1]].pose.orientation.x = 0.0
        my_map_coordinate[end[0]][end[1]].pose.orientation.y = 0.0
        my_map_coordinate[end[0]][end[1]].pose.orientation.z = 0.7
        my_map_coordinate[end[0]][end[1]].pose.orientation.w = 0.7
        
    # 동쪽
    elif end[0] - start[0] == 1:
        my_map_coordinate[end[0]][end[1]].pose.orientation.x = 0.0
        my_map_coordinate[end[0]][end[1]].pose.orientation.y = 0.0
        my_map_coordinate[end[0]][end[1]].pose.orientation.z = -0.7
        my_map_coordinate[end[0]][end[1]].pose.orientation.w = 0.7
        
    nav.goToPose(my_map_coordinate[end[0]][end[1]])

    i=0
    while not nav.isTaskComplete():
        i = i+1
        feedback = nav.getFeedback()
        if feedback and i % 5 == 0:
            print("Distance remaining : " + str(feedback.distance_remaining) + " m")
            if feedback.distance_remaining > 0.1:
                pass
            else:
                print("Distance remaining : " + str(feedback.distance_remaining) + " m")
                nav.cancelTask()
                print("명령을 이수했습니다.")

    print(str(end) + "좌표로 이동했습니다." )
    print()
    
    return end

def get_my_map_coordinate():
    image = image_processing.pgm_to_matrix("/home/du/mini_bot/baby_map.pgm", "/home/du/mini_bot/baby_map.yaml" ,3 , 12, -1.1, 2.2)
    image.run()
    my_map_coordinate = []
    my_map_layer = []
    cor_x = image.origin_x  #1.1씩 더하세요
    cor_y = image.origin_y  #1.1씩 빼세요.

    for i in range(len(image.matrix)):
        cor_x = image.origin_x
        cor_y = cor_y - image.sero_m
        for j in range(len(image.matrix[0])):
            cor_x = cor_x + image.garo_m
            if image.matrix[i][j] == 1:
                my_map_layer.append(None)
            elif image.matrix[i][j] == 0:
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = nav.get_clock().now().to_msg()
                goal.pose.position.x = cor_x
                goal.pose.position.y = cor_y
                goal.pose.position.z = 0.006378173828125
                goal.pose.orientation.x = 0.0
                goal.pose.orientation.y = 0.0
                goal.pose.orientation.z = 0.0
                goal.pose.orientation.w = 1.0
                my_map_layer.append(goal)
        my_map_coordinate.append(my_map_layer)
        my_map_layer = []
    return my_map_coordinate


def main(args=None):
    node = Subscriber()
    node.spin()
    node.destroy_node()
    rp.shutdown()

    # args = None 아무런 의미가 없는말이다.
    # init 에 뭔가를 넣을 수 있으니 만약에 하고 싶다면 이걸 수정해라 라고 해서 args = None이라고 한거다.
    #test_sub = Subscriber()
    #rp.spin(test_sub)
    #test_sub.destroy_node()
    # 종료해달라
if __name__ == '__main__':
    main()
    

    


