import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealsenseListener(Node):

    def __init__(self):
        super().__init__('realsense_listener')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert the ROS image to OpenCV format using cv_bridge
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Extract depth values from the image
        depth_scale = 0.001  # The depth scale of Realsense cameras
        depth_image = cv_image.astype('float32') * depth_scale

        # Do something with the depth image here...
        self.get_logger().info('Received depth image with shape {}'.format(depth_image.shape))


def main(args=None):
    rclpy.init(args=args)

    realsense_listener = RealsenseListener()

    rclpy.spin(realsense_listener)

    realsense_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
