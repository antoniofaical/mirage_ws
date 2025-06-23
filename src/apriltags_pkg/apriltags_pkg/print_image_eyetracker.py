import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

APRILTAG_36h11_HALF_LENGTH = 28
APRILTAG_TAH16H5_HALF_LENGTH = 55
print_tags_positions = False

class PoseEstimationEyeTracker(Node):
    def __init__(self):
        super().__init__('pose_estimation_eyetracker')
        self.image_sub = self.create_subscription(
            Image,
            '/tobii_glasses/front_camera',
            self.image_callback,
            10)
        self.frame = None
        cv2.namedWindow("Eye-Tracker", 1)

    def image_callback(self, data):
        self.bridge = CvBridge()
        try:
            if data.encoding == '8UC3':
                self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            else:
                self.frame = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return 

    # def process_frame(self):
        if self.frame is not None:
        ##########################################

            undistorted_frame = self.frame

            cv2.imshow("Eye-tracker", undistorted_frame)
            if cv2.waitKey(1) == ord('q'):
                return

# stream.release()

def main():
    rclpy.init()
    pose_estimation_eyetracker = PoseEstimationEyeTracker()

    while rclpy.ok():
        rclpy.spin_once(pose_estimation_eyetracker)

    rclpy.shutdown()
    cv2.destroyAllWindows()
