import cv2
import numpy as np
import sys
import Astar
import time
import threading
import time
from queue import Queue


class make_route:
    start = (0, 0)    
    
    pre_station = 0
    robot_moving_list = []

    robot_x = 100
    robot_y = 100
    moving_status = 0
    count = 0
    direction = ""

    robot_status = "Not Working"
    robot_station = "0"

    def __init__(self, parent=None, position=None):
        self.reset_route_color()
        self.reset_station_color()
        

    def reset_route_color(self):
        self.row_line_color_list = []
        row_line_color_list_temp = [(255, 255, 255), (255, 255, 255)]
        self.row_line_color_list.append(row_line_color_list_temp)
        row_line_color_list_temp = []
        for i in range(4):
            row_line_color_list_temp = [(255, 255, 255), (255, 255, 255), (255, 255, 255), (255, 255, 255)]
            self.row_line_color_list.append(row_line_color_list_temp)
        self.col_line_color_list = [(255, 255, 255), (255, 255, 255), (255, 255, 255), (255, 255, 255)]

    def reset_station_color(self):
        self.circle_color_list = []
        for i in range(9):
            self.circle_color_list.append((255, 255, 255))

    
    def change_station_color(self, station:int):
        self.reset_station_color()
        self.circle_color_list[self.pre_station] = (0, 0, 255)
        self.circle_color_list[station] = (255, 0, 0)
        self.pre_station = station



    def robot_order(self, end):
        self.reset_route_color()
        maze = [[0, 0, 0, 1, 1], # 1층 
                [1, 1, 0, 1, 1], # 2층 
                [0, 0, 0, 0, 0], # 3층
                [1, 1, 0, 1, 1], # 4층
                [0, 0, 0, 0, 0], # 5층
                [1, 1, 0, 1, 1], # 6층
                [0, 0, 0, 0, 0], # 7층
                [1, 1, 0, 1, 1], # 8층
                [0, 0, 0, 0, 0]] # 9층

        make_route = Astar.Node()
        _, path = make_route.run(maze, self.start, end)

        self.row_line_color_list = []
        row_line_color_list_temp = [(255, 255, 255), (255, 255, 255)]
        self.row_line_color_list.append(row_line_color_list_temp)
        row_line_color_list_temp = []
        for i in range(4):
            row_line_color_list_temp = [(255, 255, 255), (255, 255, 255), (255, 255, 255), (255, 255, 255)]
            self.row_line_color_list.append(row_line_color_list_temp)

        self.col_line_color_list = [(255, 255, 255), (255, 255, 255), (255, 255, 255), (255, 255, 255)]
        try:
            pre_path = path[0]
            for now_path in path:
                if pre_path[1] != now_path[1]:
                    if pre_path[1] > now_path[1]:
                        self.row_line_color_list[int(now_path[0]/2)][pre_path[1]-1] = (0, 0, 255)
                        self.robot_moving_list.append("d")
                    elif pre_path[1] < now_path[1]:
                        self.row_line_color_list[int(now_path[0]/2)][pre_path[1]] = (0, 0, 255)
                        self.robot_moving_list.append("a")
                
                elif pre_path[0] != now_path[0] and now_path[0] % 2 == 0: 
                    if pre_path[0] > now_path[0]:
                        self.col_line_color_list[int(now_path[0]/2)] = (0, 0, 255)
                        self.robot_moving_list.append("s")
                    elif pre_path[0] < now_path[0]:
                        self.col_line_color_list[int(now_path[0]/2)-1] = (0, 0, 255)
                        self.robot_moving_list.append("w")
                pre_path = now_path
        except:
            print("같은 곳으로 이동하려고 합니다.")

        

        self.start = end
        
    def robot_moving(self):
        self.moving_status = 1

        if self.count == 0:
            try:
                self.direction = self.robot_moving_list.pop(0)

            except:
                self.moving_status = 0
                return

        if self.count < 10:
            if self.direction == "w":
                self.robot_y += 10
            elif self.direction == "s":
                self.robot_y -= 10
            elif self.direction == "a":
                self.robot_x += 13
            elif self.direction == "d":
                self.robot_x -= 13

            self.count += 1
            time.sleep(0.1)
        
        else:
            self.count = 0




        






def operation_opencv(q):
    # OpenCV 윈도우 생성
    cv2.namedWindow('image')
    first_order = make_route()
    second_order = make_route()

    order = first_order
    order_num = 1

    robot_status = "Not Working"
    robot_station = "0"

    while True:
        if order_num == 1:
            order = first_order
        elif order_num == 2:
            order = second_order


        # 검정색 배경으로 초기화된 이미지 생성
        img = np.zeros((740, 800 , 3), np.uint8)
        # 현재 사각형 그리기
        cv2.circle(img, center = (100, 100), radius = 20, color = order.circle_color_list[0], thickness = -1)
        cv2.putText(img,"station 0",(50, 70),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,0))
        cv2.circle(img, center = (100, 200), radius = 20, color = order.circle_color_list[1], thickness = -1)
        cv2.putText(img,"station 1",(50, 170),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,0))
        cv2.circle(img, center = (100, 300), radius = 20, color = order.circle_color_list[3], thickness = -1)
        cv2.putText(img,"station 3",(50, 270),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,0))
        cv2.circle(img, center = (100, 400), radius = 20, color = order.circle_color_list[5], thickness = -1)
        cv2.putText(img,"station 5",(50, 370),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,0))
        cv2.circle(img, center = (100, 500), radius = 20, color = order.circle_color_list[7], thickness = -1)
        cv2.putText(img,"station 7",(50, 470),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,0))

        cv2.circle(img, center = (620, 200), radius = 20, color = order.circle_color_list[2], thickness = -1)
        cv2.putText(img,"station 2",(600, 170),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,0))
        cv2.circle(img, center = (620, 300), radius = 20, color = order.circle_color_list[4], thickness = -1)
        cv2.putText(img,"station 4",(600, 270),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,0))
        cv2.circle(img, center = (620, 400), radius = 20, color = order.circle_color_list[6], thickness = -1)
        cv2.putText(img,"station 6",(600, 370),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,0))
        cv2.circle(img, center = (620, 500), radius = 20, color = order.circle_color_list[8], thickness = -1)
        cv2.putText(img,"station 8",(600, 470),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,0))
        # 가로줄
        cv2.line(img, pt1 = (100,100), pt2=(230, 100), color=order.row_line_color_list[0][0], thickness=2)
        cv2.line(img, pt1 = (230,100), pt2=(360, 100), color=order.row_line_color_list[0][1], thickness=2)

        cv2.line(img, pt1 = (100,200), pt2=(230, 200), color=order.row_line_color_list[1][0], thickness=2)
        cv2.line(img, pt1 = (230,200), pt2=(360, 200), color=order.row_line_color_list[1][1], thickness=2)
        cv2.line(img, pt1 = (360,200), pt2=(490, 200), color=order.row_line_color_list[1][2], thickness=2)
        cv2.line(img, pt1 = (490,200), pt2=(620, 200), color=order.row_line_color_list[1][3], thickness=2)

        cv2.line(img, pt1 = (100, 300), pt2=(230, 300), color=order.row_line_color_list[2][0], thickness=2)
        cv2.line(img, pt1 = (230, 300), pt2=(360, 300), color=order.row_line_color_list[2][1], thickness=2)
        cv2.line(img, pt1 = (360, 300), pt2=(490, 300), color=order.row_line_color_list[2][2], thickness=2)
        cv2.line(img, pt1 = (490, 300), pt2=(620,300), color=order.row_line_color_list[2][3], thickness=2)

        cv2.line(img, pt1 = (100,400), pt2=(230, 400), color=order.row_line_color_list[3][0], thickness=2)
        cv2.line(img, pt1 = (230,400), pt2=(360, 400), color=order.row_line_color_list[3][1], thickness=2)
        cv2.line(img, pt1 = (360,400), pt2=(490, 400), color=order.row_line_color_list[3][2], thickness=2)
        cv2.line(img, pt1 = (490,400), pt2=(620, 400), color=order.row_line_color_list[3][3], thickness=2)

        cv2.line(img, pt1 = (100,500), pt2=(230, 500), color=order.row_line_color_list[4][0], thickness=2)
        cv2.line(img, pt1 = (230,500), pt2=(360, 500), color=order.row_line_color_list[4][1], thickness=2)
        cv2.line(img, pt1 = (360,500), pt2=(490, 500), color=order.row_line_color_list[4][2], thickness=2)
        cv2.line(img, pt1 = (490,500), pt2=(620, 500), color=order.row_line_color_list[4][3], thickness=2)



        cv2.line(img, pt1 = (360,100), pt2=(360, 200), color= order.col_line_color_list[0], thickness=2)
        cv2.line(img, pt1 = (360,200), pt2=(360, 300), color= order.col_line_color_list[1], thickness=2)
        cv2.line(img, pt1 = (360,300), pt2=(360, 400), color= order.col_line_color_list[2], thickness=2)
        cv2.line(img, pt1 = (360,400), pt2=(360, 500), color= order.col_line_color_list[3], thickness=2)

        cv2.rectangle(img, pt1=(first_order.robot_x-25, first_order.robot_y - 25), pt2 =(first_order.robot_x + 25, first_order.robot_y + 25), color=(0, 255, 0), thickness=2, lineType=None, shift = None)
        cv2.putText(img,"ROBOT1 : " + first_order.robot_status + " " + str(first_order.robot_station),(100, 600),cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0))

        cv2.rectangle(img, pt1=(second_order.robot_x-20, second_order.robot_y - 20), pt2 =(second_order.robot_x + 25, second_order.robot_y + 25), color=(0, 255, 255), thickness=2, lineType=None, shift = None)
        cv2.putText(img,"ROBOT2 : " + second_order.robot_status + " " + str(second_order.robot_station),(100, 700),cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0))

        # 이미지 표시
        cv2.imshow('image', img)
        # q 키를 누르면 종료


        if order.moving_status == 0:
            order.robot_status = "not working"

            data = q.get()
            if data is None:
                pass

            else:
                data = str(data)
                robot_name, station =data.split(",")
                robot_name = int(robot_name)
                station = int(station)

                if station == 0:
                    end_x = 0
                    end_y = 0
                elif station % 2 == 1 :
                    end_x = station + 1
                    end_y = 0
                elif station % 2 == 0:
                    end_x = station
                    end_y = 4

                
                if robot_name == 1:
                    order_num = 1
                    order = first_order
                elif robot_name == 2:
                    order_num = 2
                    order = second_order
                
                end = (end_x, end_y)
                print(end)
                order.change_station_color(station)
                order.robot_order(end)
                order.robot_moving()
                order.robot_station = station
                
        else:
            order.robot_status = "moving"
            order.robot_moving()

        if cv2.waitKey(1) == 27:
            break

    # 리소스 해제
    cv2.destroyAllWindows()


def robot_operation(q):
    while(True):
        input_order= input("입력하세요. (로봇,스테이션) 형식으로 >> ")
        q.put(input_order)

def q_put_Node(q):
    while(True):
        time.sleep(0.1)
        q.put(None)



if __name__ == '__main__':
    q = Queue()
    t1 = threading.Thread(target=operation_opencv, args=(q,  ))
    t2 = threading.Thread(target=robot_operation, args=(q,  ))
    t3 = threading.Thread(target=q_put_Node, args=(q,  ))
    t3.start()
    t2.start()
    t1.start()
    