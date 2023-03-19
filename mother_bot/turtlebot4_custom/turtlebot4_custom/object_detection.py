import cv_bridge
import cv2
import rclpy as rp

from turtlebot4_custom.obstacle_model import Inference

from rclpy.node import Node
from sensor_msgs.msg import Image
from turtlebot4_custom_msgs.msg import ObjectDetection

import os
import numpy as np
import torch

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.center_x, self.center_y = -1, -1
        
        self.inf = Inference()
        self.cv_bridge_color = cv_bridge.CvBridge()
        self.cv_bridge_depth = cv_bridge.CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.callback_color,
            10)

        # 거리값 subscriber 
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.callback_depth,
            10
        )

        self.publisher = self.create_publisher(ObjectDetection, '/obj_det_result', 10)

    def callback_depth(self, msg):
        cv_depth = self.cv_bridge_depth.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        depth_scale = 0.1
        self.depth_image = cv_depth.astype('float32') * depth_scale

        if not (self.center_x < 0 and self.center_y < 0):
            self.get_logger().info(f'The distance of the detected obstacle is {self.depth_image[int(self.center_y), int(self.center_x)]} cm.')
            self.pub_msg.distance = int(self.depth_image[int(self.center_y), int(self.center_x)])
            self.publisher.publish(self.pub_msg)

    def callback_color(self, msg):
        cv_image = self.cv_bridge_color.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        rgb_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        result = self.inf.inference(rgb_cv_image)
        try:    
            result_tolist = result[0].tolist()
        except:
            result_tolist = []

        if result_tolist:
            self.center_x, self.center_y = self.inf.get_center_point(result_tolist[0], result_tolist[1], result_tolist[2], result_tolist[3])
            cv2.circle(rgb_cv_image, (int(self.center_x), int(self.center_y)), 10, (0, 255, 0), -1)
            print(str(self.center_x), str(self.center_y), str(result_tolist[-1]))
        else:
            print('not detected', result_tolist)
            self.center_x, self.center_y = -1, -1

        self.pub_msg = ObjectDetection()
        try:
            if result_tolist[-1] == 0:
                self.pub_msg.class_name = 'box'
        except:
            self.pub_msg.class_name = 'nothing'
            self.pub_msg.distance = 0
        #self.publisher.publish(self.pub_msg)
        cv2.imshow('Image window', rgb_cv_image)
        cv2.waitKey(1)

def main(args=None):
    rp.init(args=args)
    node = ObjectDetector()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
