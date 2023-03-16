import cv_bridge
import cv2
import rclpy as rp

from .obstacle_model import Inference

from rclpy.node import Node
from sensor_msgs.msg import Image
from turtlebot4_custom_msgs.msg import ObjectDetection

import os
import numpy as np
import pyrealsense2
import torch

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
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

        depth_scale = 0.001
        depth_image = cv_depth.astype('float32') * depth_scale
        
        self.get_logger().info(f'The distance of the detected obstacle is {depth_image[center_y, center_x]} m.')
        pass

    def callback_color(self, msg):
        cv_image = self.cv_bridge_color.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.rgb_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        result = self.inf.inference(self.rgb_cv_image)
        try:
            result_tolist = result[0].tolist()
        except:
            pass
        cv2.imshow('Image window', self.rgb_cv_image)
        cv2.waitKey(1)
        pub_msg = ObjectDetection()
        try:
            pub_msg.class_num = result_tolist
        except:
            pub_msg.class_num = [100.0,]
        pub_msg.distance = [100,]
        self.publisher.publish(pub_msg)

def main(args=None):
    rp.init(args=args)
    node = ObjectDetector()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
