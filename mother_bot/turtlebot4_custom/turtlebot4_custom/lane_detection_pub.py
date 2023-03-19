import rclpy as rp
import numpy as np
from rclpy.node import Node

import cv_bridge
import cv2
from sensor_msgs.msg import Image
from turtlebot4_custom_msgs.msg import LaneDetection
#from geometry_msgs.msg import Twist
from ultralytics import YOLO
from .yolo_segmentation import YOLOSegmentation
from ament_index_python.packages import get_package_share_directory

import time

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.turn_direction = ""
        self.center_pt_x_blue = 0
        self.center_pt_x_orange = 0
        self.pt_y_blue = 0
        self.pt_y_orange = 0
        self.lane_num = 0
        self.is_lane_num_det = False
        self.is_turning = False
        self.target_point_x = 320
        self.target_point_y = 0
        self.pt_path = get_package_share_directory('turtlebot4_custom') + '/pt_files/lane_L.pt'
        self.model = YOLOSegmentation(self.pt_path)
        self.cv_bridge_color = cv_bridge.CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.callback_color,
            10
        )
        self.publisher = self.create_publisher(LaneDetection, '/lane_det_result', 10)

    def callback_color(self, msg):
        self.cv_image = self.cv_bridge_color.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.rgb_cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        self.height, self.width = self.rgb_cv_image.shape[:2]
        self.roi_area = 5*self.height//7
        self.roi = self.rgb_cv_image[self.roi_area:self.height, 0:self.width]
        
        # seg detection
        try:
            bboxes, classes, segmentations, scores = self.model.detect(self.roi)
            for bbox, class_id, seg, score in zip(bboxes, classes, segmentations, scores):
                if class_id == 0:

                    cv2.polylines(self.roi, [seg], True, (0, 255, 0), 4)
                    condition = (seg[:, 1] >= 0) & (seg[:, 1] < 300)
                    filtered_arr_x = seg[condition, 0]
                    
                    try:
                        max_pt_x_orange = np.max(filtered_arr_x)
                        min_pt_x_orange = np.min(filtered_arr_x)
                        self.center_pt_x_orange = (max_pt_x_orange + min_pt_x_orange) // 2
                    except:
                        pass

                    self.pt_y_orange = np.min(seg[:, 1])

                if class_id == 1:

                    cv2.polylines(self.roi, [seg], True, (0, 255, 0), 4)

                    condition = (seg[:, 1] >= 0) & (seg[:, 1] < 300)
                    
                    filtered_arr_x = seg[condition, 0]
                    
                    try:
                        max_pt_x_blue = np.max(filtered_arr_x)
                        min_pt_x_blue = np.min(filtered_arr_x)
                        self.center_pt_x_blue = (max_pt_x_blue + min_pt_x_blue) // 2
                    except:
                        pass

                    self.pt_y_blue = np.min(seg[:, 1])
            
            pub_msg = LaneDetection()
            
            # 주행시작시 현재 차선 
            if (not self.is_lane_num_det) and (self.center_pt_x_orange < self.center_pt_x_blue):
                self.lane_num = 1
                pub_msg.lane_num = self.lane_num
                self.is_lane_num_det = True
            elif (not self.is_lane_num_det) and (self.center_pt_x_orange > self.center_pt_x_blue):
                self.lane_num = 2
                pub_msg.lane_num = self.lane_num
                self.is_lane_num_det = True
            else:
                pass
            
            self.target_point_y = min(self.pt_y_orange, self.pt_y_blue)
            self.target_point_x = (self.center_pt_x_orange + self.center_pt_x_blue) // 2

            pub_msg.linear_x = self.compensate_y(self.roi_area, self.target_point_y)
            pub_msg.angular_z = self.compensate_x(self.width//2, self.target_point_x)
            
            print("pt_y_blue ; ", str(self.pt_y_blue))
            print("pt_y_ornage ; ", str(self.pt_y_orange))
            
            # 1차선 회전
            if (self.lane_num == 1) and ((self.pt_y_blue > 40) or (self.pt_y_orange > 40)) :
                self.turn_direction = "STRAIGHT"
                self.is_turning = False
                self.old_target_point_x = self.target_point_x
                pub_msg.angular_z = 0.0
                if (self.pt_y_orange > 100) and (self.pt_y_blue > 100):
                    pub_msg.linear_x = 0.0
                    pub_msg.angular_z = self.compensate_x(self.width//2, self.old_target_point_x)
                    self.turn_direction = "TURN"
                    self.is_turning = True
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        pub_msg.linear_x = 0.1
                        pub_msg.angular_z = 0.0
                        self.publisher.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 5:
                        pub_msg.linear_x = 0.0
                        pub_msg.angular_z = 0.2
                        self.publisher.publish(pub_msg)

            elif (self.pt_y_blue < 5) and (self.pt_y_orange < 5):
                self.is_turning = False
                self.turn_direction = "STRAIGHT"

            # 2차선 회전
            if (self.lane_num == 2) and ((self.pt_y_blue > 40) or (self.pt_y_orange > 40)) :
                self.turn_direction = "STRAIGHT"
                self.is_turning = False
                self.old_target_point_x = self.target_point_x
                pub_msg.angular_z = 0.0
                if (self.pt_y_orange > 80) and (self.pt_y_blue > 80):
                    # pub_msg.angular.z = self.compensate_x(self.width//2, self.old_target_point_x)
                    self.turn_direction = "TURN"
                    self.is_turning = True
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        pub_msg.linear_x = 0.1
                        pub_msg.angular_z = 0.0
                        self.publisher.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 5:
                        pub_msg.angular_z = 0.2
                        pub_msg.linear_x = 0.0
                        self.publisher.publish(pub_msg)
                        
            elif (self.pt_y_blue < 5) and (self.pt_y_orange < 5):
                self.is_turning = False
                self.turn_direction = "STRAIGHT"

            # 가로선
            # cyan
            cv2.line(self.rgb_cv_image, (0, self.roi_area), (self.width, self.roi_area), (255, 255, 0), 5)
            # blue
            cv2.line(self.roi, (0, self.target_point_y), (self.width, self.target_point_y), (255, 0, 0), 5)
            # lane num
            cv2.putText(self.rgb_cv_image, "LN : " + str(self.lane_num), (self.width//2 - 100, 140), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(self.rgb_cv_image, "linear_x : {:.2f} m/s".format(self.compensate_y(self.roi_area, self.target_point_y)), (self.width//2 - 100, 170), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(self.rgb_cv_image, "angular_z : {:.2f} rad/s".format(self.compensate_x(self.width//2, self.target_point_x)), (self.width//2 - 100, 200), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(self.rgb_cv_image, str(self.turn_direction), (self.width//2 - 100, 230), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(self.rgb_cv_image, 'blue :' + str(self.pt_y_blue), (self.width//2 - 100, 250), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(self.rgb_cv_image, 'ornage :' + str(self.pt_y_orange), (self.width//2 - 100, 270), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            # 앵글 타겟 포인트
            cv2.circle(self.rgb_cv_image, (self.target_point_x, self.roi_area), 10, (0, 255, 0), 3)
            cv2.circle(self.rgb_cv_image, (self.width//2, self.roi_area), 10, (50, 50, 50), -1)
            cv2.imshow('Image window', self.rgb_cv_image)
            cv2.waitKey(1)
            
            self.publisher.publish(pub_msg)

        except:
            cv2.line(self.rgb_cv_image, (0, self.roi_area), (self.width, self.roi_area), (255, 255, 0), 5)
            cv2.line(self.roi, (0, self.target_point_y), (self.width, self.target_point_y), (255, 0, 0), 5)
            cv2.circle(self.rgb_cv_image, (self.width//2, self.roi_area), 10, (50, 50, 50), -1)
            cv2.putText(self.rgb_cv_image, "LN : " + str(self.lane_num), (self.width//2 - 100, 140), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(self.rgb_cv_image, "linear_x : {:.2f} m/s".format(self.compensate_y(self.roi_area, self.target_point_y)), (self.width//2 - 100, 170), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(self.rgb_cv_image, "angular_z : {:.2f} rad/s".format(self.compensate_x(self.width//2, self.target_point_x)), (self.width//2 - 100, 200), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(self.rgb_cv_image, str(self.turn_direction), (self.width//2 - 100, 230), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(self.rgb_cv_image, 'blue :' + str(self.pt_y_blue), (self.width//2 - 100, 250), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(self.rgb_cv_image, 'ornage :' + str(self.pt_y_orange), (self.width//2 - 100, 270), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.circle(self.rgb_cv_image, (self.target_point_x, self.roi_area), 10, (0, 255, 0), 3)
            cv2.imshow('Image window', self.rgb_cv_image)
            cv2.waitKey(1)

            pub_msg = LaneDetection()
            pub_msg.linear_x = self.compensate_y(self.roi_area, self.target_point_y)
            pub_msg.angular_z = self.compensate_x(self.width//2, self.target_point_x)

            print("pt_y_blue ; ", str(self.pt_y_blue))
            print("pt_y_ornage ; ", str(self.pt_y_orange))

            # 1차선 회전
            if (self.lane_num == 1) and ((self.pt_y_blue > 40) or (self.pt_y_orange > 40)) :
                self.turn_direction = "STRAIGHT"
                self.is_turning = False
                self.old_target_point_x = self.target_point_x
                pub_msg.angular_z = 0.0
                if (self.pt_y_orange > 80) and (self.pt_y_blue > 80):
                    pub_msg.linear_x = 0.0
                    pub_msg.angular_z = self.compensate_x(self.width//2, self.old_target_point_x)
                    self.turn_direction = "TURN"
                    self.is_turning = True
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        pub_msg.linear_x = 0.1
                        pub_msg.angular_z = 0.0
                        self.publisher.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 5:
                        pub_msg.angular_z = 0.2
                        pub_msg.linear_x = 0.0
                        self.publisher.publish(pub_msg)
            elif (self.pt_y_blue < 5) and (self.pt_y_orange < 5):
                self.is_turning = False
                self.turn_direction = "STRAIGHT"

            # 2차선 회전
            if (self.lane_num == 2) and ((self.pt_y_blue > 40) or (self.pt_y_orange > 40)) :
                self.turn_direction = "STRAIGHT"
                self.is_turning = False
                self.old_target_point_x = self.target_point_x
                pub_msg.angular_z = 0.0
                if (self.pt_y_orange > 80) and (self.pt_y_blue > 80):
                    pub_msg.linear_x = 0.0
                    pub_msg.angular_z = 0.0
                    # pub_msg.angular.z = self.compensate_x(self.width//2, self.old_target_point_x)
                    self.turn_direction = "TURN"
                    self.is_turning = True
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        pub_msg.linear_x = 0.1
                        pub_msg.angular_z = 0.0
                        self.publisher.publish(pub_msg)
                    start_time = time.time()
                    while time.time() - start_time < 5:
                        pub_msg.angular_z = 0.2
                        pub_msg.linear_x = 0.0
                        self.publisher.publish(pub_msg)
            elif (self.pt_y_blue < 5) and (self.pt_y_orange < 5):
                self.is_turning = False
                self.turn_direction = "STRAIGHT"
            
            self.publisher.publish(pub_msg)

    def compensate_x(self, center_x, target_x):    
        p_x = 0.001
        center_x = center_x
        target_x = target_x
        e_x = center_x - target_x
        control_x = p_x * e_x
        return control_x 

    def compensate_y(self, position_y, target_y):
        p_y = 0.0003
        position_y = position_y
        target_y = target_y
        e_y = position_y - target_y
        control_y = p_y * e_y
        return control_y


def main(args=None):
    rp.init(args=args)

    lane_detector = LaneDetector()
    rp.spin(lane_detector)

    lane_detector.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()