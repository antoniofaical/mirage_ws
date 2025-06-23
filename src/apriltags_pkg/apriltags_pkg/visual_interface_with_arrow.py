import rclpy
from rclpy.node import Node
import cv2
import os
import random
import numpy as np
from time import time

class VisualInterfaceNode(Node):
    def __init__(self):
        super().__init__('visual_interface_node')
        path_36h11_id0 = os.path.join(os.getcwd(), 'src/apriltags_pkg/apriltags_pkg/apriltags','apriltag36h11_0.png')
        path_36h11_id1 = os.path.join(os.getcwd(), 'src/apriltags_pkg/apriltags_pkg/apriltags','apriltag36h11_1.png')
        path_36h11_id2 = os.path.join(os.getcwd(), 'src/apriltags_pkg/apriltags_pkg/apriltags','apriltag36h11_2.png')
        path_36h11_id3 = os.path.join(os.getcwd(), 'src/apriltags_pkg/apriltags_pkg/apriltags','apriltag36h11_3.png')
        self.image_36H11_id0 = cv2.imread(path_36h11_id0)  # Update with actual path
        self.image_36H11_id1 = cv2.imread(path_36h11_id1)
        self.image_36H11_id2 = cv2.imread(path_36h11_id2)
        self.image_36H11_id3 = cv2.imread(path_36h11_id3)
        
        path_16h5 = os.path.join(os.getcwd(), 'src/apriltags_pkg/apriltags_pkg/apriltags','apriltag16h5_0.png')
        self.image_16H5 = cv2.imread(path_16h5)    # Update with actual path

        self.image_16H5_h, self.image_16H5_w = 108, 106
        self.image_16H5 = cv2.resize(self.image_16H5, (self.image_16H5_w, self.image_16H5_h))
        
        self.white_background_h, self.white_background_w = 600,1200
        self.white_background = np.full((self.white_background_h, self.white_background_w, 3), 255, dtype=np.uint8)  # Update dimensions as needed
        
        self.image_36H11_all_h, self.image_36H11_all_w = 179, 196
        self.image_36H11_id0 = cv2.resize(self.image_36H11_id0, (self.image_36H11_all_w, self.image_36H11_all_h))
        self.image_36H11_id1 = cv2.resize(self.image_36H11_id1, (self.image_36H11_all_w, self.image_36H11_all_h))
        self.image_36H11_id2 = cv2.resize(self.image_36H11_id2, (self.image_36H11_all_w, self.image_36H11_all_h))
        self.image_36H11_id3 = cv2.resize(self.image_36H11_id3, (self.image_36H11_all_w, self.image_36H11_all_h))


        self.white_background[0:self.image_36H11_all_h, 0:self.image_36H11_all_w] = self.image_36H11_id0
        self.white_background[(self.white_background_h-self.image_36H11_all_h):self.white_background_h, 0:self.image_36H11_all_w] = self.image_36H11_id1
        self.white_background[(self.white_background_h-self.image_36H11_all_h):self.white_background_h, (self.white_background_w-self.image_36H11_all_w):self.white_background_w] = self.image_36H11_id2
        self.white_background[0:self.image_36H11_all_h, (self.white_background_w-self.image_36H11_all_w):self.white_background_w] = self.image_36H11_id3

        #Drawing Square
        cv2.line(self.white_background,(196,10),(1004,10),(255,0,0),5)
        cv2.line(self.white_background,(196,590),(1004,590),(255,0,0),5)
        cv2.line(self.white_background,(196,10),(196,590),(255,0,0),5)
        cv2.line(self.white_background,(196,10),(196,590),(255,0,0),5)
        cv2.line(self.white_background,(1004,10),(1004,590),(255,0,0),5)

        #Inserting logo
        path_logo = os.path.join(os.getcwd(), 'src/apriltags_pkg/apriltags_pkg/apriltags','logo_einstein.png')
        self.image_logo = cv2.imread(path_logo)

        self.image_logo_h, self.image_logo_w = 200, 190
        self.image_logo = cv2.resize(self.image_logo, (self.image_logo_w, self.image_logo_h))

        self.white_background[(self.image_36H11_all_h+20):(self.image_36H11_all_h+self.image_logo_h+20), 0:self.image_logo_w]=self.image_logo
        self.white_background[(self.image_36H11_all_h+20):(self.image_36H11_all_h+self.image_logo_h+20), (self.white_background_w-self.image_logo_w):self.white_background_w]=self.image_logo

        self.h_position=20
        self.w_position=210
        
        self.timer = self.create_timer(0.2, self.timer_callback)
        # self.show_initial_images()
        cv2.namedWindow("Interface", 1)
        cv2.imshow("Interface", self.white_background)
        self.t0 = 0
        self.delta_t = 3

    def show_initial_images(self):
        cv2.imshow("tag36h11", self.image_36H11)
        cv2.imshow("tag16h5", self.image_16H5)

    def timer_callback(self):

        
        if time() - self.t0 > self.delta_t:


            self.white_background[self.h_position:(self.h_position+self.image_16H5_h), self.w_position:(self.w_position+self.image_16H5_w)] = self.image_16H5

            h_position_new = random.randint(40, 482)
            w_position_new = random.randint(230, 898)

            cv2.arrowedLine(self.white_background,(self.w_position,self.h_position),(w_position_new,h_position_new),(0,0,0),2)

            cv2.imshow("Interface", self.white_background)

            self.white_background[self.h_position:(self.h_position+self.image_16H5_h), self.w_position:(self.w_position+self.image_16H5_w)] = np.full((self.image_16H5_h, self.image_16H5_w, 3), 255, dtype=np.uint8)

            cv2.arrowedLine(self.white_background,(self.w_position,self.h_position),(w_position_new,h_position_new),(255,255,255),2)

            self.h_position=h_position_new
            self.w_position=w_position_new


            self.t0 = time()

        if cv2.waitKey(1) == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = VisualInterfaceNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()