import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import os
import random
import pyautogui
import numpy as np
from time import time


class VisualInterfaceNode(Node):
    def __init__(self):
        super().__init__('visual_interface_node')
        path_36h11 = os.path.join(os.getcwd(), 'src/apriltags_pkg/apriltags_pkg/apriltags','apriltag36h11.png')
        self.image_36H11 = cv2.imread(path_36h11)  # Update with actual path
        
        path_16h5 = os.path.join(os.getcwd(), 'src/apriltags_pkg/apriltags_pkg/apriltags','apriltag16h5.png')
        self.image_16H5 = cv2.imread(path_16h5)    # Update with actual path

        self.image_16H5_h, self.image_16H5_w = 176, 178
        self.image_16H5 = cv2.resize(self.image_16H5, (self.image_16H5_w, self.image_16H5_h))
        
        self.white_background_h, self.white_background_w = 1200, 600
        self.white_background = np.full((self.white_background_w, self.white_background_h, 3), 255, dtype=np.uint8)  # Update dimensions as needed
        
        self.image_36H11_h, self.image_36H11_w = 448, 489
        self.white_background[0:self.image_36H11_h, 0:self.image_36H11_w] = self.image_36H11

        self.timer = self.create_timer(0.05, self.timer_callback)
        # self.show_initial_images()
        cv2.namedWindow("Interface", 1)
        cv2.imshow("Interface", self.white_background)
        self.t0 = 0
        self.delta_t = 3
        self.h_position_rand_16h5 = 250 # TODO: it should be half of its corresponding dimension of the white background
        self.w_position_rand_16h5 = 250 # TODO: it should be half of its corresponding dimension of the white background

        ### GAZE
        self.x_gaze_on_eyetracker_frame = 0.0
        self.y_gaze_on_eyetracker_frame = 0.0
        self.gaze_buffer_size = 5
        self.gaze_x_buffer = []
        self.gaze_y_buffer = []
        self.subscription = self.create_subscription(
            String,
            '/tobii_glasses/gaze_position',
            self.gaze_position_callback,
            10
        )

    def show_initial_images(self):
        cv2.imshow("tag36h11", self.image_36H11)
        cv2.imshow("tag16h5", self.image_16H5)

    def timer_callback(self):

        # Reset the white background
        self.white_background = np.full((600, 1200, 3), 255, dtype=np.uint8)  # Update dimensions as needed
        self.white_background[0:448, 0:489] = self.image_36H11
        self.white_background[self.h_position_rand_16h5:(self.h_position_rand_16h5+self.image_16H5_h), self.w_position_rand_16h5:(self.w_position_rand_16h5+self.image_16H5_w)] = self.image_16H5

        # just update the target apriltag it the time is up
        if time() - self.t0 > self.delta_t:
            self.h_position_rand_16h5 = random.randint(100, 400)
            self.w_position_rand_16h5 = random.randint(500, 900)
            self.t0 = time()
            self.white_background[self.h_position_rand_16h5:(self.h_position_rand_16h5+self.image_16H5_h), self.w_position_rand_16h5:(self.w_position_rand_16h5+self.image_16H5_w)] = np.full((self.image_16H5_h, self.image_16H5_w, 3), 255, dtype=np.uint8)

        # Draw the red ring at the gaze position
        gaze_x = int(self.x_gaze_on_eyetracker_frame*self.white_background_w)
        gaze_y = int(self.y_gaze_on_eyetracker_frame*self.white_background_h)
        # print(gaze_x, gaze_y)
        cv2.circle(self.white_background, (gaze_x, gaze_y), 20, (0, 0, 255), 2)

        cv2.imshow("Interface", self.white_background)
        if cv2.waitKey(1) == ord('q'):
            rclpy.shutdown()

    def gaze_position_callback(self, msg):
        try:
            x_str, y_str = msg.data.split(',')
            x = float(x_str)
            y = float(y_str)

            self.gaze_x_buffer.append(x)
            self.gaze_y_buffer.append(y)

            if len(self.gaze_x_buffer) > self.gaze_buffer_size:
                self.gaze_x_buffer.pop(0)
                self.gaze_y_buffer.pop(0)

            self.x_gaze_on_eyetracker_frame = sum(self.gaze_x_buffer) / len(self.gaze_x_buffer)
            self.y_gaze_on_eyetracker_frame = sum(self.gaze_y_buffer) / len(self.gaze_y_buffer)

            # self.get_logger().info(f'Received gaze position: x={self.x_gaze_on_eyetracker_frame}, y={self.y_gaze_on_eyetracker_frame}')
        except ValueError:
            self.get_logger().error('Failed to parse gaze position from message.')



def main(args=None):
    rclpy.init(args=args)
    node = VisualInterfaceNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()