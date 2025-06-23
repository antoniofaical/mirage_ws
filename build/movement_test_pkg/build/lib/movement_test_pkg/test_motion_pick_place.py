import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy.timer
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rclpy.parameter
import roboticstoolbox as rbt
from math import pi
import spatialmath as spa
from builtin_interfaces.msg import Duration
import numpy as np
from enum import Enum
import math
from time import time
from movement_test_pkg.robotiq_gripper_serial import RobotiqGripper


FREQ_TF = 80 # Hz
Z_TOOL = 0.244
dt_CONTROLLER = 1/FREQ_TF


# ALTURA PARA PEGAR OBJETO: 933 mm

pos_robot_relative_to_apriltag = PoseStamped()
pos_robot_relative_to_apriltag.header.frame_id = 'base'
pos_robot_relative_to_apriltag.pose.position.x = -0.335
pos_robot_relative_to_apriltag.pose.position.y = -0.685
pos_robot_relative_to_apriltag.pose.position.z = 1.05 # deve ir para 997 mm 0.997
RPY_init = spa.SE3.RPY(np.pi,0.0,np.pi/2) #0 pi 0
quat_init = RPY_init.UnitQuaternion().vec
pos_robot_relative_to_apriltag.pose.orientation.x = quat_init[1]
pos_robot_relative_to_apriltag.pose.orientation.y = quat_init[2]
pos_robot_relative_to_apriltag.pose.orientation.z = quat_init[3]
pos_robot_relative_to_apriltag.pose.orientation.w = quat_init[0]

class MoveToTargetNode(Node):
    def __init__(self):
        super().__init__('square_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        #Parâmetros DH do robô
        self.UR20= rbt.DHRobot([
            rbt.RevoluteDH(d=0.2363,alpha=pi/2),
            rbt.RevoluteDH(a=-0.8620),
            rbt.RevoluteDH(a=-0.7287),
            rbt.RevoluteDH(d=0.2010,alpha=pi/2),
            rbt.RevoluteDH(d=0.1593,alpha=-pi/2),
            rbt.RevoluteDH(d=0.1543)
        ],name="ur20")

        #Inicialização de variáveis necessárias
        self.T=None
        self.T_tool=None
        self.sol=None
        self.dt=0.0
        self.q0=np.array([72.55, -69.92, 61.48, -81.51, -86.28, -37.05])*np.pi/180
        self.controle_choice=0.0
        self.controle_garra=0.0
        self.moving=False
        self.current_state=State.WAIT_USER_INPUT
        self.desired_cartesian_trajectory_msg = PoseStamped()
        self.target_pose_in_base_frame_msg = PoseStamped()
        self.target_pose_in_apriltag_frame_msg = PoseStamped()
        self.target_vel_in_apriltag_frame_msg = TwistStamped()
        self.old_target_pose=PoseStamped()

        self.current_pose_in_base_frame_msg = PoseStamped()
        self.joint_trajectory_msg = JointTrajectory()

        #gripper
        ip = "192.168.1.102"
        print("Creating gripper...")
        self.gripper = RobotiqGripper()
        print("Connecting to gripper...")
        self.gripper.connect(ip, 63352)
        print("Activating gripper...")
        self.gripper.activate()
        ##########

        self.timer = self.create_timer(0.05, self.pos_controller_callback)

        # Publisher to /cartesian_position topic
        self.desired_cartesian_trajectory_publisher = self.create_publisher(
            PoseStamped,
            '/desired_cartesian_trajectory',
            1
        )

        self.joint_pos = {'shoulder_lift_joint': 0.0,
                           'elbow_joint': 0.0,
                           'wrist_1_joint': 0.0,
                           'wrist_2_joint': 0.0,
                           'wrist_3_joint': 0.0,
                           'shoulder_pan_joint': 0.0}

        # Subscriber to /joint_states topic
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            1
        )

        # Subscriber to /joint_states topic
        self.target_pose_in_apriltag_frame_subscriber = self.create_subscription(
            PoseStamped,
            '/target_pose_in_apriltag_frame',
            self.target_pose_in_apriltag_frame_callback,
            1
        )

        # Subscriber to /cartesian_pose topic
        self.cartesian_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/cartesian_pose',
            self.cartesian_pose_callback,
            1
        )


    def cartesian_pose_callback(self, msg):
        self.current_pose_in_base_frame_msg.pose.position.x = msg.pose.position.x
        self.current_pose_in_base_frame_msg.pose.position.y = msg.pose.position.y
        self.current_pose_in_base_frame_msg.pose.position.z = msg.pose.position.z
        self.current_pose_in_base_frame_msg.pose.orientation.x = msg.pose.orientation.x
        self.current_pose_in_base_frame_msg.pose.orientation.y = msg.pose.orientation.y
        self.current_pose_in_base_frame_msg.pose.orientation.z = msg.pose.orientation.z
        self.current_pose_in_base_frame_msg.pose.orientation.w = msg.pose.orientation.w
    
    def target_pose_in_apriltag_frame_callback(self, msg):
        self.target_pose_in_apriltag_frame_msg = msg
    
    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_pos.keys():
                self.joint_pos[name] = position

    def pos_controller_callback(self):

        if(self.current_state==State.WAIT_USER_INPUT):

            if(self.target_pose_in_apriltag_frame_msg.pose.position.x!=self.old_target_pose.pose.position.x
               and self.target_pose_in_apriltag_frame_msg is not None):
                
                self.old_target_pose=self.target_pose_in_apriltag_frame_msg
                self.current_state=State.MOVE

            else:
                self.current_state=State.WAIT_USER_INPUT

        elif(self.current_state==State.MOVE):

            self.q0 = [self.joint_pos[joint_name] for joint_name in ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']]

            self.target_pose_in_base_frame_msg.pose.position.x = self.target_pose_in_apriltag_frame_msg.pose.position.x + pos_robot_relative_to_apriltag.pose.position.x
            self.target_pose_in_base_frame_msg.pose.position.y = self.target_pose_in_apriltag_frame_msg.pose.position.y + pos_robot_relative_to_apriltag.pose.position.y
            self.target_pose_in_base_frame_msg.pose.position.z = self.target_pose_in_apriltag_frame_msg.pose.position.z*0 + pos_robot_relative_to_apriltag.pose.position.z + Z_TOOL # TODO: multiply by zero?
            self.target_pose_in_base_frame_msg.pose.orientation.x=pos_robot_relative_to_apriltag.pose.orientation.x
            self.target_pose_in_base_frame_msg.pose.orientation.y=pos_robot_relative_to_apriltag.pose.orientation.y
            self.target_pose_in_base_frame_msg.pose.orientation.z=pos_robot_relative_to_apriltag.pose.orientation.z
            self.target_pose_in_base_frame_msg.pose.orientation.w=pos_robot_relative_to_apriltag.pose.orientation.w

            print(self.target_pose_in_apriltag_frame_msg.pose.position.x)
            self.controle_choice=self.target_pose_in_apriltag_frame_msg.pose.position.z

            self.T_tool = (spa.SE3(self.target_pose_in_base_frame_msg.pose.position.x,
                                self.target_pose_in_base_frame_msg.pose.position.y,
                                self.target_pose_in_base_frame_msg.pose.position.z)*RPY_init) #Matriz de transformação homogênea
            
            self.sol=self.UR20.ikine_LM(self.T_tool,self.q0) #Posição das juntas
            #print(self.sol)
            
            self.joint_trajectory_msg.joint_names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']

            point=JointTrajectoryPoint()
            point.positions=[self.sol.q[0], self.sol.q[1], self.sol.q[2], self.sol.q[3], self.sol.q[4], self.sol.q[5]]
            duration=Duration()
            duration.sec = 5
            duration.nanosec = 0 #int(dt_CONTROLLER*1e9)
            point.time_from_start=duration
            self.joint_trajectory_msg.points=[point]

            self.publisher_.publish(self.joint_trajectory_msg)
            self.get_logger().info('Publicando Move')
            self.start_time=time()

            self.desired_cartesian_trajectory_publisher.publish(self.target_pose_in_base_frame_msg)
            print("Calculo antes da IK=",self.target_pose_in_base_frame_msg)
            print("Calculo depois da IK=",self.UR20.fkine(self.sol.q))
            self.current_state=State.CHOISE

        elif(self.current_state==State.CHOISE):

            elapsed_time=0

            while (elapsed_time < 5): 
                 
                 elapsed_time= time() - self.start_time
        
            
            if(self.controle_choice==0):
                self.current_state=State.WAIT_USER_INPUT

            if(self.controle_choice==2):
                print("PICK")
                self.start_time=time()
                self.current_state=State.PICK
            
            if(self.controle_choice==3):
                print("PLACE")
                self.start_time=time()
                self.current_state=State.PLACE

        elif(self.current_state==State.PICK):

            elapsed_time=0

            self.T_tool = (spa.SE3(self.target_pose_in_base_frame_msg.pose.position.x,
                                self.target_pose_in_base_frame_msg.pose.position.y,
                                self.target_pose_in_base_frame_msg.pose.position.z-0.117)*RPY_init) #Matriz de transformação homogênea
            
            self.sol=self.UR20.ikine_LM(self.T_tool,self.q0) #Posição das juntas
            #print(self.sol)
            
            self.joint_trajectory_msg.joint_names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']

            point=JointTrajectoryPoint()
            point.positions=[self.sol.q[0], self.sol.q[1], self.sol.q[2], self.sol.q[3], self.sol.q[4], self.sol.q[5]]
            duration=Duration()
            duration.sec = 5
            duration.nanosec = 0 #int(dt_CONTROLLER*1e9)
            point.time_from_start=duration
            self.joint_trajectory_msg.points=[point]

            self.publisher_.publish(self.joint_trajectory_msg)
            self.get_logger().info('Publicando Pick - DESCENDO')

            self.desired_cartesian_trajectory_publisher.publish(self.target_pose_in_base_frame_msg)
            print("Calculo antes da IK=",self.target_pose_in_base_frame_msg)
            print("Calculo depois da IK=",self.UR20.fkine(self.sol.q))

            while (elapsed_time < 5): 
                 
                 elapsed_time= time() - self.start_time
        
            self.gripper.move_and_wait_for_pos(255, 255, 255)

            elapsed_time=0
            self.start_time=time()

            while (elapsed_time < 3): 
                 
                 elapsed_time= time() - self.start_time

            self.T_tool = (spa.SE3(self.target_pose_in_base_frame_msg.pose.position.x,
                                self.target_pose_in_base_frame_msg.pose.position.y,
                                self.target_pose_in_base_frame_msg.pose.position.z)*RPY_init) #Matriz de transformação homogênea
            
            self.sol=self.UR20.ikine_LM(self.T_tool,self.q0) #Posição das juntas
            #print(self.sol)
            
            self.joint_trajectory_msg.joint_names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']

            point=JointTrajectoryPoint()
            point.positions=[self.sol.q[0], self.sol.q[1], self.sol.q[2], self.sol.q[3], self.sol.q[4], self.sol.q[5]]
            duration=Duration()
            duration.sec = 5
            duration.nanosec = 0 #int(dt_CONTROLLER*1e9)
            point.time_from_start=duration
            self.joint_trajectory_msg.points=[point]

            self.publisher_.publish(self.joint_trajectory_msg)
            self.get_logger().info('Publicando Pick - SUBINDO')

            self.desired_cartesian_trajectory_publisher.publish(self.target_pose_in_base_frame_msg)
            print("Calculo antes da IK=",self.target_pose_in_base_frame_msg)
            print("Calculo depois da IK=",self.UR20.fkine(self.sol.q))

            elapsed_time=0
            self.start_time=time()

            while (elapsed_time < 5): 
                 
                 elapsed_time= time() - self.start_time

            self.current_state=State.WAIT_USER_INPUT
            print("VOLTANDO PRO WAIT")

        elif(self.current_state==State.PLACE):

            elapsed_time=0

            self.T_tool = (spa.SE3(self.target_pose_in_base_frame_msg.pose.position.x,
                                self.target_pose_in_base_frame_msg.pose.position.y,
                                self.target_pose_in_base_frame_msg.pose.position.z-0.107)*RPY_init) #Matriz de transformação homogênea
            
            self.sol=self.UR20.ikine_LM(self.T_tool,self.q0) #Posição das juntas
            #print(self.sol)
            
            self.joint_trajectory_msg.joint_names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']

            point=JointTrajectoryPoint()
            point.positions=[self.sol.q[0], self.sol.q[1], self.sol.q[2], self.sol.q[3], self.sol.q[4], self.sol.q[5]]
            duration=Duration()
            duration.sec = 5
            duration.nanosec = 0 #int(dt_CONTROLLER*1e9)
            point.time_from_start=duration
            self.joint_trajectory_msg.points=[point]

            self.publisher_.publish(self.joint_trajectory_msg)
            self.get_logger().info('Publicando Place - DESCENDO')

            self.desired_cartesian_trajectory_publisher.publish(self.target_pose_in_base_frame_msg)
            print("Calculo antes da IK=",self.target_pose_in_base_frame_msg)
            print("Calculo depois da IK=",self.UR20.fkine(self.sol.q))

            while (elapsed_time < 5): 
                 
                 elapsed_time= time() - self.start_time
        
            self.gripper.move_and_wait_for_pos(0, 255, 255)

            elapsed_time=0
            self.start_time=time()

            while (elapsed_time < 3): 
                 
                 elapsed_time= time() - self.start_time

            self.T_tool = (spa.SE3(self.target_pose_in_base_frame_msg.pose.position.x,
                                self.target_pose_in_base_frame_msg.pose.position.y,
                                self.target_pose_in_base_frame_msg.pose.position.z)*RPY_init) #Matriz de transformação homogênea
            
            self.sol=self.UR20.ikine_LM(self.T_tool,self.q0) #Posição das juntas
            #print(self.sol)
            
            self.joint_trajectory_msg.joint_names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']

            point=JointTrajectoryPoint()
            point.positions=[self.sol.q[0], self.sol.q[1], self.sol.q[2], self.sol.q[3], self.sol.q[4], self.sol.q[5]]
            duration=Duration()
            duration.sec = 5
            duration.nanosec = 0 #int(dt_CONTROLLER*1e9)
            point.time_from_start=duration
            self.joint_trajectory_msg.points=[point]

            self.publisher_.publish(self.joint_trajectory_msg)
            self.get_logger().info('Publicando Place - SUBINDO')

            self.desired_cartesian_trajectory_publisher.publish(self.target_pose_in_base_frame_msg)
            print("Calculo antes da IK=",self.target_pose_in_base_frame_msg)
            print("Calculo depois da IK=",self.UR20.fkine(self.sol.q))

            elapsed_time=0
            self.start_time=time()

            while (elapsed_time < 5): 
                 
                 elapsed_time= time() - self.start_time

            self.current_state=State.WAIT_USER_INPUT
            print("VOLTANDO PRO WAIT")


class State(Enum):
    WAIT_USER_INPUT=1
    MOVE=2
    CHOISE=3
    PICK=4
    PLACE=5

def main(args=None):
    rclpy.init()
    node = MoveToTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()