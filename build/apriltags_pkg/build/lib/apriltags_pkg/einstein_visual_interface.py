import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from rclpy.exceptions import ParameterNotDeclaredException


import cv2
import os
import random
import numpy as np
from time import time
from time import sleep
from enum import Enum
import spatialmath as spa
import math

OVERSHOOT_X=0 #For demonstration
OVERSHOOT_Y=0
ROBOT_STEP=10
MOVE_OPTION_X=0
MOVE_OPTION_Y=20

#Will be parameters in the future
CC_X_R=0.70 # Distance between two apriltags (Real world)
CC_Y_R=0.385 #

class VisualInterfaceNode(Node):
    def __init__(self):
        super().__init__('visual_interface_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('interface_height',rclpy.Parameter.Type.INTEGER),
                ('interface_width',rclpy.Parameter.Type.INTEGER),
                ('apriltag_proportion',rclpy.Parameter.Type.DOUBLE),
                ('distance_proportion',rclpy.Parameter.Type.DOUBLE),
                ('gripper_proportion',rclpy.Parameter.Type.DOUBLE),
                ('gaze_circle_proportion',rclpy.Parameter.Type.DOUBLE)
            ]
        )
        try:
            # Get parameters
            self.interface_height = self.get_parameter('interface_height').get_parameter_value().integer_value
            self.interface_width = self.get_parameter('interface_width').get_parameter_value().integer_value
            self.apriltag_proportion = self.get_parameter('apriltag_proportion').get_parameter_value().double_value
            self.distance_proportion = self.get_parameter('distance_proportion').get_parameter_value().double_value
            self.gripper_proportion = self.get_parameter('gripper_proportion').get_parameter_value().double_value
            self.gaze_circle_proportion = self.get_parameter('gaze_circle_proportion').get_parameter_value().double_value
            
            self.get_logger().info(f'Interface Height: {self.interface_height}')
            self.get_logger().info(f'Interface Width: {self.interface_width}')
            self.get_logger().info(f'Apriltag Proportion: {self.apriltag_proportion}')
            self.get_logger().info(f'Distance Proportion: {self.distance_proportion}')
            self.get_logger().info(f'Gripper Proportion: {self.gripper_proportion}')
            self.get_logger().info(f'Gaze Proportion: {self.gaze_circle_proportion}')

        except ParameterNotDeclaredException as e:
            self.get_logger().error(f'Error: {e}')
            self.interface_height = 600
            self.interface_width = 1200
            self.apriltag_proportion=0.3
            self.distance_proportion=0.05
            self.gripper_proportion=0.05
            self.gaze_circle_proportion=0.05


        ############################DEMONSTRATION##################
        self.ROBOT_POSITION_X=int(0.5*self.interface_width)
        self.ROBOT_POSITION_Y=int(0.5*self.interface_height)
        ###########################################################



        self.declaring_apriltags_and_variables()
        self.generating_default_interface()

        self.timer = self.create_timer(0.05, self.timer_callback)

        cv2.namedWindow("Interface", 1)
        cv2.imshow("Interface", self.white_background)


        #Subscriber GAZE
        self.target_point = PointStamped()
        self.subscriber = self.create_subscription(
            PointStamped,
            '/target_info/position',
            self.target_position_callback,
            10
        )  

        #Subscriber Robot
        self.robot_pose = PoseStamped()
        self.subscriber = self.create_subscription(
            PoseStamped,
            '/cartesian_pose',
            self.robot_position_callback,
            10
        )

        #Subscriber Desired_Position
        self.robot_pose_desired = PoseStamped()
        self.subscriber = self.create_subscription(
            PoseStamped,
            '/desired_cartesian_trajectory',
            self.robot_position_desired_callback,
            10
        )

        #Publisher robot_point
        self.target_pose_publisher = self.create_publisher(
            PoseStamped,
            '/target_pose_in_apriltag_frame',
            1
        )

        #Robot_point 
        self.target_pose_msg = PoseStamped()

    def timer_callback(self):

        # Reset the white background
        self.generating_default_interface()

        if self.use_virtual_apriltag_as_target:
            self.white_background[self.h_position_rand_16h5:(self.h_position_rand_16h5+self.image_16H5_h), self.w_position_rand_16h5:(self.w_position_rand_16h5+self.image_16H5_w)] = self.image_16H5


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

        if ((not self.use_virtual_apriltag_as_target)):

            if (self.current_state==State.OBSERVATION):

                self.white_background[self.gripper_init_y:(self.image_gripper_h+self.gripper_init_y), self.gripper_init_x:(self.image_gripper_w+self.gripper_init_x)] = self.image_gripper
                #self.white_background[(self.gripper_init_y-int(self.image_gripper_h/2)):(int(self.image_gripper_h/2)+self.gripper_init_y), (self.gripper_init_x-int(self.image_gripper_w/2)):(int(self.image_gripper_w/2)+self.gripper_init_x)] = self.image_gripper
                cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)
                self.start_time=time()
                self.current_state=State.TIMER

            elif(self.current_state==State.TIMER):

                elapsed_time = time() - self.start_time
                self.white_background[self.gripper_init_y:(self.image_gripper_h+self.gripper_init_y), self.gripper_init_x:(self.image_gripper_w+self.gripper_init_x)] = self.image_gripper
                #self.white_background[(self.gripper_init_y-int(self.image_gripper_h/2)):(int(self.image_gripper_h/2)+self.gripper_init_y), (self.gripper_init_x-int(self.image_gripper_w/2)):(int(self.image_gripper_w/2)+self.gripper_init_x)] = self.image_gripper
                cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)
                if(elapsed_time>=self.count_time):
                    angle = int(((elapsed_time-self.count_time) / self.update_time) * 360)
                    cv2.ellipse(self.white_background, (target_x,target_y+OVERSHOOT_Y), (int(self.gaze_circle_proportion*self.interface_height), int(self.gaze_circle_proportion*self.interface_height)), -90, 0, angle, (0,255,0), 15)

                if (abs(self.old_target_x-target_x)>20 or target_x==0):
                    self.current_state=State.OBSERVATION

                self.old_target_x=target_x
                self.old_target_y=target_y

                if(elapsed_time>=self.total_time):
                    self.start_time=time()
                    self.current_state=State.MENU

            elif(self.current_state==State.MENU):

                self.white_background[self.gripper_init_y:(self.image_gripper_h+self.gripper_init_y), self.gripper_init_x:(self.image_gripper_w+self.gripper_init_x)] = self.image_gripper
                #self.white_background[(self.gripper_init_y-int(self.image_gripper_h/2)):(int(self.image_gripper_h/2)+self.gripper_init_y), (self.gripper_init_x-int(self.image_gripper_w/2)):(int(self.image_gripper_w/2)+self.gripper_init_x)] = self.image_gripper
                cv2.circle(self.white_background, (self.old_target_x, self.old_target_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 0), 15)
                cv2.line(self.white_background,(self.old_target_x-30,self.old_target_y),(self.old_target_x+30,self.old_target_y),(255,255,255),1) # linha de baixo
                cv2.line(self.white_background,(self.old_target_x,self.old_target_y-30),(self.old_target_x,self.old_target_y+30),(255,255,255),1)
                cv2.putText(self.white_background,'MOVE',(self.old_target_x-40,self.old_target_y-100),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),3)
                cv2.line(self.white_background,(self.old_target_x-50,self.old_target_y-80),(self.old_target_x+50,self.old_target_y-80),(0,0,0),3) # linha de baixo
                cv2.line(self.white_background,(self.old_target_x-50,self.old_target_y-140),(self.old_target_x+50,self.old_target_y-140),(0,0,0),3) # linha de cima
                cv2.line(self.white_background,(self.old_target_x-50,self.old_target_y-80),(self.old_target_x-50,self.old_target_y-140),(0,0,0),3) # linha esquerda
                cv2.line(self.white_background,(self.old_target_x+50,self.old_target_y-80),(self.old_target_x+50,self.old_target_y-140),(0,0,0),3) # linha direita
                cv2.putText(self.white_background,'CANCEL',(self.old_target_x+150,self.old_target_y+OVERSHOOT_Y),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,155),3)
                cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y-40),(self.old_target_x+280,self.old_target_y-40),(0,0,155),3) # linha de baixo
                cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y+20),(self.old_target_x+280,self.old_target_y+20),(0,0,155),3) # linha de cima
                cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y-40),(self.old_target_x+140,self.old_target_y+20),(0,0,155),3) # linha esquerda
                cv2.line(self.white_background,(self.old_target_x+280,self.old_target_y-40),(self.old_target_x+280,self.old_target_y+20),(0,0,155),3) # linha direita

                if(self.choice==State.PLACE):
                    cv2.putText(self.white_background,'PICK',(self.old_target_x-35,self.old_target_y+100+MOVE_OPTION_Y),cv2.FONT_HERSHEY_SIMPLEX,1,(143,10,18),3)
                    cv2.line(self.white_background,(self.old_target_x-45,self.old_target_y+120+MOVE_OPTION_Y),(self.old_target_x+45,self.old_target_y+120+MOVE_OPTION_Y),(143,10,18),3) # linha de baixo
                    cv2.line(self.white_background,(self.old_target_x-45,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x+45,self.old_target_y+60+MOVE_OPTION_Y),(143,10,18),3) # linha de cima
                    cv2.line(self.white_background,(self.old_target_x-45,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x-45,self.old_target_y+120+MOVE_OPTION_Y),(143,10,18),3) # linha de esquerda
                    cv2.line(self.white_background,(self.old_target_x+45,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x+45,self.old_target_y+120+MOVE_OPTION_Y),(143,10,18),3) # linha de esquerda
                if(self.choice==State.PICK):
                    cv2.putText(self.white_background,'PLACE',(self.old_target_x-47,self.old_target_y+100+MOVE_OPTION_Y),cv2.FONT_HERSHEY_SIMPLEX,1,(159,184,0),3)
                    cv2.line(self.white_background,(self.old_target_x-55,self.old_target_y+120+MOVE_OPTION_Y),(self.old_target_x+55,self.old_target_y+120+MOVE_OPTION_Y),(159,184,0),3) # linha de baixo
                    cv2.line(self.white_background,(self.old_target_x-55,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x+55,self.old_target_y+60+MOVE_OPTION_Y),(159,184,0),3) # linha de cima
                    cv2.line(self.white_background,(self.old_target_x-55,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x-55,self.old_target_y+120+MOVE_OPTION_Y),(159,184,0),3) # linha de esquerda
                    cv2.line(self.white_background,(self.old_target_x+55,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x+55,self.old_target_y+120+MOVE_OPTION_Y),(159,184,0),3) # linha de esquerda

                if((self.old_target_x-80)<target_x<(self.old_target_x+80) and (self.old_target_y-170)<target_y<(self.old_target_y-50)):
                    cv2.putText(self.white_background,'MOVE',(self.old_target_x-40,self.old_target_y-100),cv2.FONT_HERSHEY_SIMPLEX,1,(100,100,100),3)
                    cv2.line(self.white_background,(self.old_target_x-50,self.old_target_y-80),(self.old_target_x+50,self.old_target_y-80),(100,100,100),3) # linha de baixo
                    cv2.line(self.white_background,(self.old_target_x-50,self.old_target_y-140),(self.old_target_x+50,self.old_target_y-140),(100,100,100),3) # linha de cima
                    cv2.line(self.white_background,(self.old_target_x-50,self.old_target_y-80),(self.old_target_x-50,self.old_target_y-140),(100,100,100),3) # linha esquerda
                    cv2.line(self.white_background,(self.old_target_x+50,self.old_target_y-80),(self.old_target_x+50,self.old_target_y-140),(100,100,100),3) # linha direita
                    elapsed_time = time() - self.start_time
                    angle = int((elapsed_time / self.total_time) * 360)
                    cv2.circle(self.white_background, (self.target_choice_x, self.target_choice_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)
                    cv2.ellipse(self.white_background, (self.target_choice_x,self.target_choice_y+OVERSHOOT_Y), (int(self.gaze_circle_proportion*self.interface_height), int(self.gaze_circle_proportion*self.interface_height)), -90, 0, angle, (0,255,0), 15)
                    
                    if(elapsed_time>=self.total_time):
                        self.start_time=time()
                        self.current_state=State.TEXT_MOVE

                elif((self.old_target_x-75)<target_x<(self.old_target_x+75) and (self.old_target_y+30+MOVE_OPTION_Y)<target_y<(self.old_target_y+150+MOVE_OPTION_Y) and self.choice==State.PLACE  ):

                    cv2.putText(self.white_background,'PICK',(self.old_target_x-35,self.old_target_y+100+MOVE_OPTION_Y),cv2.FONT_HERSHEY_SIMPLEX,1,(243,110,118),3)
                    cv2.line(self.white_background,(self.old_target_x-45,self.old_target_y+120+MOVE_OPTION_Y),(self.old_target_x+45,self.old_target_y+120+MOVE_OPTION_Y),(243,110,118),3) # linha de baixo
                    cv2.line(self.white_background,(self.old_target_x-45,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x+45,self.old_target_y+60+MOVE_OPTION_Y),(243,110,118),3) # linha de cima
                    cv2.line(self.white_background,(self.old_target_x-45,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x-45,self.old_target_y+120+MOVE_OPTION_Y),(243,110,118),3) # linha de esquerda
                    cv2.line(self.white_background,(self.old_target_x+45,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x+45,self.old_target_y+120+MOVE_OPTION_Y),(243,110,118),3) # linha de esquerda
                    
                    elapsed_time = time() - self.start_time
                    angle = int((elapsed_time / self.total_time) * 360)
                    cv2.circle(self.white_background, (self.target_choice_x, self.target_choice_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)
                    cv2.ellipse(self.white_background, (self.target_choice_x,self.target_choice_y+OVERSHOOT_Y), (int(self.gaze_circle_proportion*self.interface_height), int(self.gaze_circle_proportion*self.interface_height)), -90, 0, angle, (0,255,0), 15)

                    if(elapsed_time>=self.total_time):
                        self.start_time=time()
                        print("PICK")
                        self.current_state=State.PICK
                    
                elif((self.old_target_x-55)<target_x<(self.old_target_x+55) and (self.old_target_y+30+MOVE_OPTION_Y)<target_y<(self.old_target_y+150+MOVE_OPTION_Y) and self.choice==State.PICK ):

                    cv2.putText(self.white_background,'PLACE',(self.old_target_x-47,self.old_target_y+100+MOVE_OPTION_Y),cv2.FONT_HERSHEY_SIMPLEX,1,(259,284,100),3)                   
                    cv2.line(self.white_background,(self.old_target_x-55,self.old_target_y+120+MOVE_OPTION_Y),(self.old_target_x+55,self.old_target_y+120+MOVE_OPTION_Y),(259,284,100),3) # linha de baixo
                    cv2.line(self.white_background,(self.old_target_x-55,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x+55,self.old_target_y+60+MOVE_OPTION_Y),(259,284,100),3) # linha de cima
                    cv2.line(self.white_background,(self.old_target_x-55,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x-55,self.old_target_y+120+MOVE_OPTION_Y),(259,284,100),3) # linha de esquerda
                    cv2.line(self.white_background,(self.old_target_x+55,self.old_target_y+60+MOVE_OPTION_Y),(self.old_target_x+55,self.old_target_y+120+MOVE_OPTION_Y),(259,284,100),3) # linha de esquerda

                    elapsed_time = time() - self.start_time
                    angle = int((elapsed_time / self.total_time) * 360)
                    cv2.circle(self.white_background, (self.target_choice_x, self.target_choice_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)
                    cv2.ellipse(self.white_background, (self.target_choice_x,self.target_choice_y+OVERSHOOT_Y), (int(self.gaze_circle_proportion*self.interface_height), int(self.gaze_circle_proportion*self.interface_height)), -90, 0, angle, (0,255,0), 15)

                    if(elapsed_time>=self.total_time):

                        self.start_time=time()
                        print("PLACE")
                        self.current_state=State.PLACE

                elif((self.old_target_x+110)<target_x<(self.old_target_x+350) and (self.old_target_y-80)<target_y<(self.old_target_y+70)):

                    cv2.putText(self.white_background,'CANCEL',(self.old_target_x+150,self.old_target_y+OVERSHOOT_Y),cv2.FONT_HERSHEY_SIMPLEX,1,(100,100,255),3)
                    cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y-40),(self.old_target_x+280,self.old_target_y-40),(100,100,255),3) # linha de baixo
                    cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y+20),(self.old_target_x+280,self.old_target_y+20),(100,100,255),3) # linha de cima
                    cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y-40),(self.old_target_x+140,self.old_target_y+20),(100,100,255),3) # linha esquerda
                    cv2.line(self.white_background,(self.old_target_x+280,self.old_target_y-40),(self.old_target_x+280,self.old_target_y+20),(100,100,255),3) # linha direita

                    elapsed_time = time() - self.start_time
                    angle = int((elapsed_time / self.total_time) * 360)
                    cv2.circle(self.white_background, (self.target_choice_x, self.target_choice_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)
                    cv2.ellipse(self.white_background, (self.target_choice_x,self.target_choice_y+OVERSHOOT_Y), (int(self.gaze_circle_proportion*self.interface_height), int(self.gaze_circle_proportion*self.interface_height)), -90, 0, angle, (0,255,0), 15)

                    if(elapsed_time>=self.total_time):
                        self.current_state=State.OBSERVATION

                else:
                    cv2.circle(self.white_background, (self.target_choice_x, self.target_choice_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)
                    self.start_time=time()

            elif(self.current_state==State.TEXT_MOVE):
                cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)
                self.white_background[self.gripper_init_y:(self.image_gripper_h+self.gripper_init_y), self.gripper_init_x:(self.image_gripper_w+self.gripper_init_x)] = self.image_gripper
                #self.white_background[(self.gripper_init_y-int(self.image_gripper_h/2)):(int(self.image_gripper_h/2)+self.gripper_init_y), (self.gripper_init_x-int(self.image_gripper_w/2)):(int(self.image_gripper_w/2)+self.gripper_init_x)] = self.image_gripper
                # cv2.circle(self.white_background, (self.old_target_x, self.old_target_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 0), 15)
                # cv2.putText(self.white_background,'MOVE HERE?',(self.old_target_x-100,self.old_target_y-100),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
                # cv2.putText(self.white_background,'YES',(self.old_target_x+150,self.old_target_y+OVERSHOOT_Y),cv2.FONT_HERSHEY_SIMPLEX,1,(0,100,0),3)
                # cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y-40),(self.old_target_x+220,self.old_target_y-40),(0,100,0),3) # linha de baixo
                # cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y+20),(self.old_target_x+220,self.old_target_y+20),(0,100,0),3) # linha de cima
                # cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y-40),(self.old_target_x+140,self.old_target_y+20),(0,100,0),3) # linha esquerda
                # cv2.line(self.white_background,(self.old_target_x+220,self.old_target_y-40),(self.old_target_x+220,self.old_target_y+20),(0,100,0),3) # linha direita
                # cv2.putText(self.white_background,'NO',(self.old_target_x-200,self.old_target_y+OVERSHOOT_Y),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),3)
                # cv2.line(self.white_background,(self.old_target_x-220,self.old_target_y-40),(self.old_target_x-140,self.old_target_y-40),(0,0,255),3) # linha de baixo
                # cv2.line(self.white_background,(self.old_target_x-220,self.old_target_y+20),(self.old_target_x-140,self.old_target_y+20),(0,0,255),3) # linha de cima
                # cv2.line(self.white_background,(self.old_target_x-220,self.old_target_y-40),(self.old_target_x-220,self.old_target_y+20),(0,0,255),3) # linha esquerda
                # cv2.line(self.white_background,(self.old_target_x-140,self.old_target_y-40),(self.old_target_x-140,self.old_target_y+20),(0,0,255),3) # linha direita

                # if((self.old_target_x+140)<target_x<(self.old_target_x+220) and (self.old_target_y-40)<target_y<(self.old_target_y+20)):

                #     cv2.putText(self.white_background,'YES',(self.old_target_x+150,self.old_target_y+OVERSHOOT_Y),cv2.FONT_HERSHEY_SIMPLEX,1,(100,200,100),3)
                #     cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y-40),(self.old_target_x+220,self.old_target_y-40),(100,200,100),3) # linha de baixo
                #     cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y+20),(self.old_target_x+220,self.old_target_y+20),(100,200,100),3) # linha de cima
                #     cv2.line(self.white_background,(self.old_target_x+140,self.old_target_y-40),(self.old_target_x+140,self.old_target_y+20),(100,200,100),3) # linha esquerda
                #     cv2.line(self.white_background,(self.old_target_x+220,self.old_target_y-40),(self.old_target_x+220,self.old_target_y+20),(100,200,100),3) # linha direita

                #     elapsed_time = time() - self.start_time
                #     angle = int((elapsed_time / self.total_time) * 360)
                #     cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)
                #     cv2.ellipse(self.white_background, (target_x,target_y+OVERSHOOT_Y), (int(self.gaze_circle_proportion*self.interface_height), int(self.gaze_circle_proportion*self.interface_height)), -90, 0, angle, (0,255,0), 15)

                #     if(elapsed_time>=self.total_time):

                self.gripper_final_x=self.old_target_x
                self.gripper_final_y=self.old_target_y
                #REAL WORLD POINT
                #MUDAR 98 E 89.5
                point_target_x= ((self.gripper_final_x-(self.w_distance+self.image_36H11_all_w/2))*CC_X_R)/(self.CC_X_I)
                point_target_y= ((self.gripper_final_y-(self.h_distance+self.image_36H11_all_h/2))*CC_Y_R)/(self.CC_Y_I)

                RPY = spa.SE3.RPY(np.pi,  0.0, 0.0)
                point_to_robot=(RPY*(spa.SE3.Trans(point_target_x,point_target_y,0)))
                self.target_pose_msg.header.stamp = self.get_clock().now().to_msg()
                self.target_pose_msg.header.frame_id = "apriltag_TAG36H11"
                self.target_pose_msg.pose.position.x = point_to_robot.t[0]
                self.target_pose_msg.pose.position.y = point_to_robot.t[1]
                self.target_pose_msg.pose.position.z = 0.0

                self.target_pose_publisher.publish(self.target_pose_msg)

                self.cont_gripper=0
                self.real_world_init_gripper_x=0#cartesian pose
                self.real_world_init_gripper_y=0#cartesian pose
                self.distance_robot_x=0#abs(self.real_world_init_gripper_x-point_target_x)
                self.distance_robot_y=0#abs(self.real_world_init_gripper_y-point_target_y)
                self.start_time=time()
                self.old_state=State.TEXT_MOVE
                self.current_state=State.WAIT_ROBOTMOVING
                
                # elif((self.old_target_x-220)<target_x<(self.old_target_x-140) and (self.old_target_y-40)<target_y<(self.old_target_y+20)):

                #     cv2.putText(self.white_background,'NO',(self.old_target_x-200,self.old_target_y+OVERSHOOT_Y),cv2.FONT_HERSHEY_SIMPLEX,1,(100,100,255),3)
                #     cv2.line(self.white_background,(self.old_target_x-220,self.old_target_y-40),(self.old_target_x-140,self.old_target_y-40),(100,100,255),3) # linha de baixo
                #     cv2.line(self.white_background,(self.old_target_x-220,self.old_target_y+20),(self.old_target_x-140,self.old_target_y+20),(100,100,255),3) # linha de cima
                #     cv2.line(self.white_background,(self.old_target_x-220,self.old_target_y-40),(self.old_target_x-220,self.old_target_y+20),(100,100,255),3) # linha esquerda
                #     cv2.line(self.white_background,(self.old_target_x-140,self.old_target_y-40),(self.old_target_x-140,self.old_target_y+20),(100,100,255),3) # linha direita

                #     elapsed_time = time() - self.start_time
                #     angle = int((elapsed_time / self.total_time) * 360)
                #     cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)
                #     cv2.ellipse(self.white_background, (target_x,target_y+OVERSHOOT_Y), (int(self.gaze_circle_proportion*self.interface_height), int(self.gaze_circle_proportion*self.interface_height)), -90, 0, angle, (0,255,0), 15)

                #     if(elapsed_time>=self.total_time):
                #         self.current_state=State.OBSERVATION
                
                # else:
                #     cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)
                #     self.start_time=time()

            elif(self.current_state==State.PICK):

                self.gripper_final_x=self.old_target_x
                self.gripper_final_y=self.old_target_y
                #REAL WORLD POINT
                #MUDAR 98 E 89.5
                point_target_x= ((self.gripper_final_x-(self.w_distance+self.image_36H11_all_w/2))*CC_X_R)/(self.CC_X_I)
                point_target_y= ((self.gripper_final_y-(self.h_distance+self.image_36H11_all_h/2))*CC_Y_R)/(self.CC_Y_I)

                RPY = spa.SE3.RPY(np.pi,  0.0, 0.0)
                point_to_robot=(RPY*(spa.SE3.Trans(point_target_x,point_target_y,0)))
                self.target_pose_msg.header.stamp = self.get_clock().now().to_msg()
                self.target_pose_msg.header.frame_id = "apriltag_TAG36H11"
                self.target_pose_msg.pose.position.x = point_to_robot.t[0]
                self.target_pose_msg.pose.position.y = point_to_robot.t[1]
                self.target_pose_msg.pose.position.z = 2.0

                self.target_pose_publisher.publish(self.target_pose_msg)

                self.cont_gripper=0
                self.real_world_init_gripper_x=0#cartesian pose
                self.real_world_init_gripper_y=0#cartesian pose
                self.distance_robot_x=0#abs(self.real_world_init_gripper_x-point_target_x)
                self.distance_robot_y=0#abs(self.real_world_init_gripper_y-point_target_y)
                self.choice=State.PICK
                self.old_state=State.PICK
                self.start_time=time()
                self.current_state=State.WAIT_ROBOTMOVING

            elif(self.current_state==State.PLACE):

                self.gripper_final_x=self.old_target_x
                self.gripper_final_y=self.old_target_y
                #REAL WORLD POINT
                #MUDAR 98 E 89.5
                point_target_x= ((self.gripper_final_x-(self.w_distance+self.image_36H11_all_w/2))*CC_X_R)/(self.CC_X_I)
                point_target_y= ((self.gripper_final_y-(self.h_distance+self.image_36H11_all_h/2))*CC_Y_R)/(self.CC_Y_I)

                RPY = spa.SE3.RPY(np.pi,  0.0, 0.0)
                point_to_robot=(RPY*(spa.SE3.Trans(point_target_x,point_target_y,0)))
                self.target_pose_msg.header.stamp = self.get_clock().now().to_msg()
                self.target_pose_msg.header.frame_id = "apriltag_TAG36H11"
                self.target_pose_msg.pose.position.x = point_to_robot.t[0]
                self.target_pose_msg.pose.position.y = point_to_robot.t[1]
                self.target_pose_msg.pose.position.z = 3.0

                self.target_pose_publisher.publish(self.target_pose_msg)

                self.cont_gripper=0
                self.real_world_init_gripper_x=0#cartesian pose
                self.real_world_init_gripper_y=0#cartesian pose
                self.distance_robot_x=0#abs(self.real_world_init_gripper_x-point_target_x)
                self.distance_robot_y=0#abs(self.real_world_init_gripper_y-point_target_y)
                self.choice=State.PLACE
                self.old_state=State.PLACE
                self.start_time=time()
                self.current_state=State.WAIT_ROBOTMOVING

            elif(self.current_state==State.WAIT_ROBOTMOVING):

                elapsed_time = time() - self.start_time

                self.white_background[(self.gripper_init_y+int(((self.gripper_final_y-self.gripper_init_y)/(1)))):(self.image_gripper_h+self.gripper_init_y+int(((self.gripper_final_y-self.gripper_init_y)/(1)))),  
                    (self.gripper_init_x+int(((self.gripper_final_x-self.gripper_init_x)/(1)))):(self.image_gripper_w+self.gripper_init_x+int(((self.gripper_final_x-self.gripper_init_x)/(1))))] = self.image_gripper
                
                if(self.old_state==State.PICK):
                    self.time_wait=18
                    cv2.circle(self.white_background, (self.old_target_x, self.old_target_y+OVERSHOOT_Y), int(0.08*self.interface_height), (255, 0, 0), 15)
                if(self.old_state==State.PLACE):
                    self.time_wait=18
                    cv2.circle(self.white_background, (self.old_target_x, self.old_target_y+OVERSHOOT_Y), int(0.08*self.interface_height), (0, 0, 255), 15)
                if(self.old_state==State.TEXT_MOVE):
                    self.time_wait=5

                for i in range(1,ROBOT_STEP+1):
                    if(i%2!=0):
                        cv2.arrowedLine(self.white_background,(self.gripper_init_x+int(((self.gripper_final_x-self.gripper_init_x)/(ROBOT_STEP))*i)+int(self.image_36H11_all_w/2),self.gripper_init_y+int(((self.gripper_final_y-self.gripper_init_y)/(ROBOT_STEP))*i)+int(self.image_36H11_all_h/2)),(self.gripper_init_x+int(((self.gripper_final_x-self.gripper_init_x)/(ROBOT_STEP))*(i+1))+int(self.image_36H11_all_w/2),self.gripper_init_y+int(((self.gripper_final_y-self.gripper_init_y)/(ROBOT_STEP))*(i+1))+int(self.image_36H11_all_h/2)),(255,0,0),2)
                
                cv2.circle(self.white_background, (target_x, target_y+OVERSHOOT_Y), int(self.gaze_circle_proportion*self.interface_height), (0, 0, 255), 15)

                if( abs(self.robot_pose.pose.position.x-self.robot_pose_desired.pose.position.x)<=0.01 and abs(self.robot_pose.pose.position.y-self.robot_pose_desired.pose.position.y)<=0.01 and elapsed_time>=self.time_wait) :

                    self.gripper_init_x=self.gripper_final_x
                    self.gripper_init_y=self.gripper_final_y

                    #TO PREVENT FROM BREAKING
                    if(self.gripper_init_x>=(self.interface_width-int(self.gripper_proportion*self.interface_width))):
                        self.gripper_init_x=(self.interface_width-int(self.gripper_proportion*self.interface_width))

                    if(self.gripper_init_y>=(self.interface_height-int(self.gripper_proportion*self.interface_height))):
                        self.gripper_init_y=(self.interface_width-int(self.gripper_proportion*self.interface_height))
                    
                    self.current_state=State.OBSERVATION
                    
        cv2.imshow("Interface", self.white_background)

        if cv2.waitKey(1) == ord('q'):
            #rclpy.shutdown()
            self.target_point.point.x=0
            self.target_point.point.y=0
            self.current_state=State.OBSERVATION
            self.choice=State.PLACE

    def declaring_apriltags_and_variables(self):

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

        self.image_16H5_h, self.image_16H5_w = 108, 106 #Didn't update
        self.image_16H5 = cv2.resize(self.image_16H5, (self.image_16H5_w, self.image_16H5_h))
               
        self.image_36H11_all_h, self.image_36H11_all_w = int(self.apriltag_proportion*self.interface_height), int(self.apriltag_proportion*self.interface_height)
        self.image_36H11_id0 = cv2.resize(self.image_36H11_id0, (self.image_36H11_all_w, self.image_36H11_all_h))
        self.image_36H11_id1 = cv2.resize(self.image_36H11_id1, (self.image_36H11_all_w, self.image_36H11_all_h))
        self.image_36H11_id2 = cv2.resize(self.image_36H11_id2, (self.image_36H11_all_w, self.image_36H11_all_h))
        self.image_36H11_id3 = cv2.resize(self.image_36H11_id3, (self.image_36H11_all_w, self.image_36H11_all_h))

        #Loading robot gripper
        path_gripper=os.path.join(os.getcwd(), 'src/apriltags_pkg/apriltags_pkg/image_gripper','icone_UR20.jpg')
        self.image_gripper=cv2.imread(path_gripper)
        self.image_gripper_h, self.image_gripper_w = int(self.gripper_proportion*self.interface_width), int(self.gripper_proportion*self.interface_width)
        self.image_gripper = cv2.resize(self.image_gripper, (self.image_gripper_w, self.image_gripper_h))
        self.gripper_init_x=self.ROBOT_POSITION_X
        self.gripper_init_y=self.ROBOT_POSITION_Y
        self.gripper_final_x=0
        self.gripper_final_y=0
        self.cont_gripper=0

        #Logo
        path_logo = os.path.join(os.getcwd(), 'src/apriltags_pkg/apriltags_pkg/apriltags','logo_einstein.png')
        self.image_logo = cv2.imread(path_logo)
        self.image_logo_h, self.image_logo_w = int(self.apriltag_proportion*self.interface_height), int(self.apriltag_proportion*self.interface_height)
        self.image_logo = cv2.resize(self.image_logo, (self.image_logo_w, self.image_logo_h))

        #Some variables

        self.t0 = 0
        self.delta_t = 3
        self.h_position_rand_16h5 = 250 # TODO: it should be half of its corresponding dimension of the white background
        self.w_position_rand_16h5 = 250 # TODO: it should be half of its corresponding dimension of the white background
        self.old_target_x=0
        self.old_target_y=0
        self.total_time=2.0
        self.count_time=1.5
        self.update_time=0.5
        self.start_time=time()
        self.current_state=State.OBSERVATION 
        self.choice=State.PLACE
        self.old_state=State.OBSERVATION
        self.time_wait=0
        self.target_choice_x=0
        self.target_choice_y=0
        self.target_choice_buffer_x=[]
        self.target_choice_buffer_y=[]
        self.target_choice_buffer_size=5

    def generating_default_interface(self):

        self.white_background_h, self.white_background_w = self.interface_height,self.interface_width
        self.white_background = np.full((self.white_background_h, self.white_background_w, 3), 255, dtype=np.uint8)  # Update dimensions as needed

        self.h_distance=int(self.distance_proportion*self.interface_height)
        self.w_distance=int(self.distance_proportion*self.interface_width)

        self.white_background[0+self.h_distance:self.image_36H11_all_h+self.h_distance, 0+self.w_distance:self.image_36H11_all_w+self.w_distance] = self.image_36H11_id0
        self.white_background[(self.white_background_h-self.image_36H11_all_h-self.h_distance):self.white_background_h-self.h_distance, 0+self.w_distance:self.image_36H11_all_w+self.w_distance] = self.image_36H11_id1
        self.white_background[(self.white_background_h-self.image_36H11_all_h-self.h_distance):self.white_background_h-self.h_distance, (self.white_background_w-self.image_36H11_all_w-self.w_distance):self.white_background_w-self.w_distance] = self.image_36H11_id2
        self.white_background[0+self.h_distance:self.image_36H11_all_h+self.h_distance, (self.white_background_w-self.image_36H11_all_w-self.w_distance):self.white_background_w-self.w_distance] = self.image_36H11_id3

        #Inserting logo
        self.white_background[(int(self.interface_height/2-self.image_logo_h/2)):((int(self.interface_height/2+self.image_logo_h/2))), self.w_distance:self.image_logo_w+self.w_distance]=self.image_logo
        self.white_background[(int(self.interface_height/2-self.image_logo_h/2)):((int(self.interface_height/2+self.image_logo_h/2))), (self.white_background_w-self.image_logo_w-self.w_distance):(self.white_background_w-self.w_distance)]=self.image_logo

        #Inserting robot image
        self.white_background[self.gripper_init_y:(self.image_gripper_h+self.gripper_init_y), self.gripper_init_x:(self.image_gripper_w+self.gripper_init_x)] = self.image_gripper
        #self.white_background[(self.gripper_init_y-int(self.image_gripper_h/2)):(int(self.image_gripper_h/2)+self.gripper_init_y), (self.gripper_init_x-int(self.image_gripper_w/2)):(int(self.image_gripper_w/2)+self.gripper_init_x)] = self.image_gripper

    
        #Conversion to real world 
        self.CC_X_I=self.white_background_w-2*self.w_distance-self.image_36H11_all_w
        self.CC_Y_I=self.white_background_h-2*self.h_distance-self.image_36H11_all_h

    def target_position_callback(self, msg):
        try:
            self.target_point.header.frame_id = msg.header.frame_id
            self.target_point.header.stamp = msg.header.stamp
            self.target_point.point.x = msg.point.x
            self.target_point.point.y = msg.point.y
            self.target_point.point.z = msg.point.z

            self.target_choice_buffer_x.append(self.target_point.point.x)
            self.target_choice_buffer_y.append(self.target_point.point.y)

            if len(self.target_choice_buffer_x)>=self.target_choice_buffer_size:
                self.target_choice_buffer_x.pop(0)
                self.target_choice_buffer_y.pop(0)

            self.target_choice_x=int(sum(self.target_choice_buffer_x)/len(self.target_choice_buffer_x))
            self.target_choice_y=int(sum(self.target_choice_buffer_y)/len(self.target_choice_buffer_y))
                        
        except ValueError:
            self.get_logger().error('Failed to parse target pose from message.')

    def robot_position_callback(self, msg_r):
        try:
            self.robot_pose.pose.position.x=msg_r.pose.position.x
            self.robot_pose.pose.position.y=msg_r.pose.position.y
        except ValueError:
            self.get_logger().error('Failed to parse target pose from message.')

    def robot_position_desired_callback(self, msg_d):
        try:
            self.robot_pose_desired.pose.position.x=msg_d.pose.position.x
            self.robot_pose_desired.pose.position.y=msg_d.pose.position.y
        except ValueError:
            self.get_logger().error('Failed to parse target pose from message.')

class State(Enum):
    OBSERVATION=1
    TIMER=2
    MENU=3
    TEXT_MOVE=4
    PICK=5
    PLACE=6
    WAIT_ROBOTMOVING=7

def main(args=None):
    rclpy.init(args=args)
    node = VisualInterfaceNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()