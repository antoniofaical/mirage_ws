import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

import cv2
import os
import random
import numpy as np
from time import time
from enum import Enum
import spatialmath as spa

OVERSHOOT_X=0 #For demonstration
OVERSHOOT_Y=0
ROBOT_POSITION_X=196+30 #For demonstration
ROBOT_POSITION_Y=90
ROBOT_STEP=10
CC_X_R=0.88 # Distance between two apriltags
CC_Y_R=0.35 #
CC_X_I=1004
CC_Y_I=421

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

        self.declare_parameter('use_external_apriltag_as_target', False)
        self.use_virtual_apriltag_as_target = self.get_parameter('use_external_apriltag_as_target').value
        print("AQUI: ", self.use_virtual_apriltag_as_target)
        
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

        #Loading robot gripper
        path_gripper=os.path.join(os.getcwd(), 'src/apriltags_pkg/apriltags_pkg/image_gripper','images.png')
        self.image_gripper=cv2.imread(path_gripper)
        self.image_gripper_h, self.image_gripper_w = 100, 100
        self.image_gripper = cv2.resize(self.image_gripper, (self.image_gripper_w, self.image_gripper_h))
        self.gripper_init_x=ROBOT_POSITION_X
        self.gripper_init_y=ROBOT_POSITION_Y
        self.gripper_final_x=0
        self.gripper_final_y=0
        self.cont_gripper=0

        self.timer = self.create_timer(0.05, self.timer_callback)
        # self.show_initial_images()
        cv2.namedWindow("Interface", 1)
        cv2.imshow("Interface", self.white_background)
        self.t0 = 0
        self.delta_t = 3
        self.h_position_rand_16h5 = 250 # TODO: it should be half of its corresponding dimension of the white background
        self.w_position_rand_16h5 = 250 # TODO: it should be half of its corresponding dimension of the white background
        self.old_target_x=0
        self.old_target_y=0
        self.total_time=5
        self.start_time=time()
        self.current_state=State.OBSERVATION    

        ### GAZE
        self.target_point = PointStamped()
        self.subscriber = self.create_subscription(
            PointStamped,
            '/target_info/position',
            self.target_position_callback,
            10
        )

        # Publisher robot_point
        self.target_pose_publisher = self.create_publisher(
            PoseStamped,
            '/target_pose_in_apriltag_frame',
            1
        )

        self.target_pose_msg = PoseStamped()

        self.cartesian_pose = PoseStamped()
        self.subscriber_cartesian_pose = self.create_subscription(
            PoseStamped,
            '/cartesian_pose',
            self.cartesian_pose_callback,
            10
        )

    def show_initial_images(self):
        cv2.imshow("tag36h11", self.image_36H11)
        cv2.imshow("tag16h5", self.image_16H5)

    def timer_callback(self):

        # Reset the white background
        self.white_background = np.full((600, 1200, 3), 255, dtype=np.uint8)  # Update dimensions as needed

        self.white_background[0:self.image_36H11_all_h, 0:self.image_36H11_all_w] = self.image_36H11_id0
        self.white_background[(self.white_background_h-self.image_36H11_all_h):self.white_background_h, 0:self.image_36H11_all_w] = self.image_36H11_id1
        self.white_background[(self.white_background_h-self.image_36H11_all_h):self.white_background_h, (self.white_background_w-self.image_36H11_all_w):self.white_background_w] = self.image_36H11_id2
        self.white_background[0:self.image_36H11_all_h, (self.white_background_w-self.image_36H11_all_w):self.white_background_w] = self.image_36H11_id3

        if self.use_virtual_apriltag_as_target:
            self.white_background[self.h_position_rand_16h5:(self.h_position_rand_16h5+self.image_16H5_h), self.w_position_rand_16h5:(self.w_position_rand_16h5+self.image_16H5_w)] = self.image_16H5

        #Drawing Square
        cv2.line(self.white_background,(196,10),(1004,10),(255,0,0),5)
        cv2.line(self.white_background,(196,590),(1004,590),(255,0,0),5)
        cv2.line(self.white_background,(196,10),(196,590),(255,0,0),5)
        cv2.line(self.white_background,(196,10),(196,590),(255,0,0),5)
        cv2.line(self.white_background,(1004,10),(1004,590),(255,0,0),5)

        #Inserting logo
        self.image_logo_h, self.image_logo_w = 200, 190
        self.image_logo = cv2.resize(self.image_logo, (self.image_logo_w, self.image_logo_h))

        self.white_background[(self.image_36H11_all_h+20):(self.image_36H11_all_h+self.image_logo_h+20), 0:self.image_logo_w]=self.image_logo
        self.white_background[(self.image_36H11_all_h+20):(self.image_36H11_all_h+self.image_logo_h+20), (self.white_background_w-self.image_logo_w):self.white_background_w]=self.image_logo

        #gripper
        ############## REMOVE THE CIRCLE ##############
        self.white_background[self.gripper_init_y:(self.image_gripper_h+self.gripper_init_y), self.gripper_init_x:(self.image_gripper_w+self.gripper_init_x)] = self.image_gripper
        cv2.circle(self.white_background, (self.gripper_init_x, self.gripper_init_y), 10, (0, 255, 0), -1) #FOR TEST AND MEASUREMENT 

        ###################################################
        # just update the target apriltag it the time is up
        if time() - self.t0 > self.delta_t and self.use_virtual_apriltag_as_target:
            self.h_position_rand_16h5 = random.randint(100, 400)
            self.w_position_rand_16h5 = random.randint(500, 900)
            self.t0 = time()
            self.white_background[self.h_position_rand_16h5:(self.h_position_rand_16h5+self.image_16H5_h), self.w_position_rand_16h5:(self.w_position_rand_16h5+self.image_16H5_w)] = np.full((self.image_16H5_h, self.image_16H5_w, 3), 255, dtype=np.uint8)

        # Draw the red ring at the gaze position
        target_x = int((self.target_point.point.x))
        target_y = int((self.target_point.point.y))
        # cv2.circle(self.white_background, (98, 90), 20, (0, 0, 255), 2)
        # cv2.circle(self.white_background, (98, 510), 20, (0, 0, 255), 2)
        # cv2.circle(self.white_background, (1102, 510), 20, (0, 0, 255), 2)
        # cv2.circle(self.white_background, (1102, 90), 20, (0, 0, 255), 2)

        #PUT THE TOLERANCE CONDITIONS INTO EACH STATE, GOING TO OBSERVATION.

        if ((not self.use_virtual_apriltag_as_target)):

            if (self.current_state==State.OBSERVATION):

                self.white_background[self.gripper_init_y:(self.image_gripper_h+self.gripper_init_y), self.gripper_init_x:(self.image_gripper_w+self.gripper_init_x)] = self.image_gripper
                cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), 25, (0, 0, 255), 15)
                self.start_time=time()
                self.current_state=State.TIMER

            elif(self.current_state==State.TIMER):

                elapsed_time = time() - self.start_time
                angle = int((elapsed_time / self.total_time) * 360)
                self.white_background[self.gripper_init_y:(self.image_gripper_h+self.gripper_init_y), self.gripper_init_x:(self.image_gripper_w+self.gripper_init_x)] = self.image_gripper
                cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), 25, (0, 0, 255), 15)
                cv2.ellipse(self.white_background, (target_x,target_y+OVERSHOOT_Y), (25, 25), -90, 0, angle, (0,255,0), 15)

                if (abs(self.old_target_x-target_x)>20 or target_x==0):
                    self.current_state=State.OBSERVATION

                self.old_target_x=target_x
                self.old_target_y=target_y

                if(elapsed_time>=self.total_time):
                    self.start_time=time()
                    self.current_state=State.TEXT

            elif(self.current_state==State.TEXT):

                self.white_background[self.gripper_init_y:(self.image_gripper_h+self.gripper_init_y), self.gripper_init_x:(self.image_gripper_w+self.gripper_init_x)] = self.image_gripper
                cv2.circle(self.white_background, (self.old_target_x, self.old_target_y+OVERSHOOT_Y), 25, (0, 0, 0), 15)
                cv2.putText(self.white_background,'YES',(self.old_target_x+100,self.old_target_y+100),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
                cv2.putText(self.white_background,'NO',(self.old_target_x-200,self.old_target_y+100),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)

                if(abs((self.old_target_x+100)-target_x)<50):

                    elapsed_time = time() - self.start_time
                    angle = int((elapsed_time / self.total_time) * 360)
                    cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), 25, (0, 0, 255), 15)
                    cv2.ellipse(self.white_background, (target_x,target_y+OVERSHOOT_Y), (25, 25), -90, 0, angle, (0,255,0), 15)

                    if(elapsed_time>=self.total_time):

                        self.gripper_final_x=self.old_target_x
                        self.gripper_final_y=self.old_target_y

                        #REAL WORLD POINT

                        point_target_x= ((self.gripper_final_x-98)*CC_X_R)/(CC_X_I)
                        point_target_y= ((self.gripper_final_y-89.5)*CC_Y_R)/(CC_Y_I)

                        RPY = spa.SE3.RPY(np.pi,  0.0, 0.0)
                        point_to_robot=(RPY*(spa.SE3.Trans(point_target_x,point_target_y,0)))

                        print(point_to_robot.t[0])


                        self.target_pose_msg.header.stamp = self.get_clock().now().to_msg()
                        self.target_pose_msg.header.frame_id = "apriltag_TAG36H11"
                        self.target_pose_msg.pose.position.x = point_to_robot.t[0]
                        self.target_pose_msg.pose.position.y = point_to_robot.t[1]
                        self.target_pose_msg.pose.position.z = 0.0

                        self.target_pose_publisher.publish(self.target_pose_msg)

                        self.cont_gripper=0
                        self.current_state=State.WAIT_ROBOTMOVING
                
                elif(abs((self.old_target_x-200)-target_x)<50):

                    elapsed_time = time() - self.start_time
                    angle = int((elapsed_time / self.total_time) * 360)
                    cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), 25, (0, 0, 255), 15)
                    cv2.ellipse(self.white_background, (target_x,target_y+OVERSHOOT_Y), (25, 25), -90, 0, angle, (0,255,0), 15)

                    if(elapsed_time>=self.total_time):
                        self.current_state=State.OBSERVATION
                
                else:
                    cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), 25, (0, 0, 255), 15)
                    self.start_time=time()

            elif(self.current_state==State.WAIT_ROBOTMOVING):

                self.cont_gripper=self.cont_gripper+1

                for i in range(1,(ROBOT_STEP+1)):

                    self.white_background[(self.gripper_init_y+int(((self.gripper_final_y-self.gripper_init_y)/(ROBOT_STEP))*i)):(self.image_gripper_h+self.gripper_init_y+int(((self.gripper_final_y-self.gripper_init_y)/(ROBOT_STEP))*i)),  
                                        (self.gripper_init_x+int(((self.gripper_final_x-self.gripper_init_x)/(ROBOT_STEP))*i)):(self.image_gripper_w+self.gripper_init_x+int(((self.gripper_final_x-self.gripper_init_x)/(ROBOT_STEP))*i))] = self.image_gripper
            
        #Only modify the cont_gripper
        # Its more easy to upgrade the chungus position calculating the difference between 
        # the subscriber point and de ROBOT_STEP. Make a new if.  

                self.white_background[(self.gripper_init_y+int(((self.gripper_final_y-self.gripper_init_y)/(ROBOT_STEP))*self.cont_gripper)):(self.image_gripper_h+self.gripper_init_y+int(((self.gripper_final_y-self.gripper_init_y)/(ROBOT_STEP))*self.cont_gripper)),  
                                      (self.gripper_init_x+int(((self.gripper_final_x-self.gripper_init_x)/(ROBOT_STEP))*self.cont_gripper)):(self.image_gripper_w+self.gripper_init_x+int(((self.gripper_final_x-self.gripper_init_x)/(ROBOT_STEP))*self.cont_gripper))] = self.image_gripper
                

                cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), 25, (0, 0, 255), 15)

                if(self.cont_gripper==ROBOT_STEP):
                    self.gripper_init_x=self.gripper_final_x
                    self.gripper_init_y=self.gripper_final_y
                    self.current_state=State.OBSERVATION



        cv2.imshow("Interface", self.white_background)

        if cv2.waitKey(1) == ord('q'):
            rclpy.shutdown()

    def target_position_callback(self, msg):
        try:
            self.target_point.header.frame_id = msg.header.frame_id
            self.target_point.header.stamp = msg.header.stamp
            self.target_point.point.x = msg.point.x
            self.target_point.point.y = msg.point.y
            self.target_point.point.z = msg.point.z
                        
        except ValueError:
            self.get_logger().error('Failed to parse target pose from message.')

    def cartesian_pose_callback(self, msg):

        self.cartesian_pose.header.frame_id=msg.header.frame_id
        self.cartesian_pose.header.stamp=msg.header.stamp
        self.cartesian_pose.pose.position.x=msg.pose.position.x
        self.cartesian_pose.pose.position.y=msg.pose.position.y


class State(Enum):
    OBSERVATION=1
    TIMER=2
    TEXT=3
    WAIT_ROBOTMOVING=4

def main(args=None):
    rclpy.init(args=args)
    node = VisualInterfaceNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()