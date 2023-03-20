import cv_bridge
import cv2
from cv2 import aruco
import rclpy as rp
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from turtlebot4_custom_msgs.msg import MarkerDetection

from ament_index_python.packages import get_package_share_directory

class ArucoMarker(Node):
    def __init__(self):
        super().__init__('marker_detector')
        self.distance = 0.0
        self.x = 0.0
        self.y = 0.0
        self.calibration_path = get_package_share_directory('turtlebot4_custom') + '/calibration_files/MultiMatrix.npz'
        self.calib_data = np.load(self.calibration_path)
        self.cam_mat = self.calib_data["camMatrix"]
        self.dist_coef = self.calib_data["distCoef"]
        self.r_vectors = self.calib_data["rVector"]
        self.t_vectors = self.calib_data["tVector"]

        self.MARKER_SIZE = 3  # centimeters
        self.marker_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
        self.param_markers = aruco.DetectorParameters_create()

        self.cv_bridge_color = cv_bridge.CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.callback_marker,
            10)

        self.publisher = self.create_publisher(MarkerDetection, '/marker_det_result', 10)

    def callback_marker(self, msg):
        cv_image = self.cv_bridge_color.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, self.marker_dict, parameters=self.param_markers
        )
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
            )
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv2.polylines(
                    cv_image, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
                # so I have rectified that mistake, I have test that out it increase the accuracy overall.
                # Calculating the distance
                self.distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )
                # Draw the pose of the marker
                point = cv2.drawFrameAxes(cv_image, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 4)
                cv2.putText(
                    cv_image,
                    f"id: {ids[0]} Dist: {round(self.distance, 2)}",
                    top_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    cv_image,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                    bottom_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                self.x, self.y = round(tVec[i][0][0],1), round(tVec[i][0][1],1)
                # print(ids, "  ", corners)
        rgb_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv2.imshow("frame", rgb_cv_image)
        cv2.waitKey(1)
        #self.get_logger().info(f'The distance of the detected Marker is {self.distance} cm.')
        #self.get_logger().info(f'The X, Y  of the detected Marker is {self.x}, {self.y} cm.')
        key = cv2.waitKey(1)
        pub_msg = MarkerDetection()
        pub_msg.x = self.x
        pub_msg.y = self.y
        pub_msg.distance = self.distance
        self.publisher.publish(pub_msg)

def main(args=None):
    rp.init(args=args)
    node = ArucoMarker()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
