import cv_bridge
import cv2
import rclpy as rp
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from turtlebot4_custom_msgs.msg import ObjectDetection

from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.center_x, self.center_y = -10, -10
        self.center_x, self.center_y = -1, -1
        self.pt_path = get_package_share_directory('turtlebot4_custom') + '/pt_files/box.pt'
        self.model = YOLO(self.pt_path)
        self.cv_bridge_color = cv_bridge.CvBridge()
        self.cv_bridge_depth = cv_bridge.CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.callback_color,
            10)

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.callback_depth,
            10
        )

        self.publisher = self.create_publisher(ObjectDetection, '/obj_det_result', 10)

    def callback_color(self, msg):
        cv_image = self.cv_bridge_color.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        rgb_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.model(rgb_cv_image, save=False, save_txt=False, conf=0.8, max_det=2)

        try:
            boxes = results[0].boxes.cpu()
            boxes = np.array(boxes.xyxy)
            self.center_x = (boxes[0][0] + boxes[0][2]) // 2 
            self.center_y = (boxes[0][1] + boxes[0][3]) // 2
        except:
            boxes = []
            self.center_x, self.center_y = -10, -10
            
        #print("boxes : " + str(boxes))
        #print("c_x, c_y : " + str(self.center_x), str(self.center_y))
        cv2.circle(rgb_cv_image, (int(self.center_x), int(self.center_y)), 10, (0, 255, 0), -1)
        cv2.imshow('Image window', rgb_cv_image)
        cv2.waitKey(1)

    def callback_depth(self, msg):
        cv_depth = self.cv_bridge_depth.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_scale = 0.1
        depth_image = cv_depth.astype('float32') * depth_scale

        if not (self.center_x < 0 and self.center_y < 0):
            self.pub_msg = ObjectDetection()
            self.pub_msg.distance = int(depth_image[int(self.center_y), int(self.center_x)])
            self.pub_msg.pixel_x = int(self.center_x)
            self.pub_msg.pixel_y = int(self.center_y)
            self.get_logger().info(f'The distance of the detected obstacle is {depth_image[int(self.center_y), int(self.center_x)]} cm.')
            self.publisher.publish(self.pub_msg)

def get_center_point(x1, y1, x2, y2):
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    return center_x, center_y

def main(args=None):
    rp.init(args=args)
    node = ObjectDetector()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
