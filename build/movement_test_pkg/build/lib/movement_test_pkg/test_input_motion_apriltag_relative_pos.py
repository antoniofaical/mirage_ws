import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy.timer
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rclpy.parameter
import roboticstoolbox as rbt
from math import pi
import spatialmath as spa
from builtin_interfaces.msg import Duration
import numpy as np

FREQ_TF = 80 # Hz
Z_TOOL = 0.244
dt_CONTROLLER = 1/FREQ_TF

pos_robot_relative_to_apriltag = PoseStamped()
pos_robot_relative_to_apriltag.header.frame_id = 'base'
pos_robot_relative_to_apriltag.pose.position.x = -0.384
pos_robot_relative_to_apriltag.pose.position.y = -0.679
pos_robot_relative_to_apriltag.pose.position.z = 0.961
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
        self.controle=0.0
        self.desired_cartesian_trajectory_msg = PoseStamped()
        self.target_pose_in_base_frame_msg = PoseStamped()
        self.target_pose_in_apriltag_frame_msg = PoseStamped()
        self.target_vel_in_apriltag_frame_msg = TwistStamped()

        self.current_pose_in_base_frame_msg = PoseStamped()
        self.joint_trajectory_msg = JointTrajectory()

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

    def pos_controller_callback(self, T):
        # # print(self.coeff_x)
        # self.x_val=self.coeff_x[0]+self.coeff_x[1]*self.dt+self.coeff_x[2]*(self.dt**2)+self.coeff_x[3]*(self.dt**3)
        # self.x_val=float(self.x_val)

        # self.y_val=self.coeff_y[0]+self.coeff_y[1]*self.dt+self.coeff_y[2]*(self.dt**2)+self.coeff_y[3]*(self.dt**3)
        # self.y_val=float(self.y_val)

        # self.z_val=self.coeff_z[0]+self.coeff_z[1]*self.dt+self.coeff_z[2]*(self.dt**2)+self.coeff_z[3]*(self.dt**3)
        # self.z_val=float(self.z_val)

        # self.desired_cartesian_trajectory_msg.pose.position.x = self.x_val
        # self.desired_cartesian_trajectory_msg.pose.position.y = self.y_val
        # self.desired_cartesian_trajectory_msg.pose.position.z = self.z_val

        # self.T_tool=(spa.SE3(self.x_val,self.y_val,self.z_val)*RPY_init) #Matriz de transformação homogênea
        # # print(self.current_pose_in_base_frame_msg)
        # print(self.T_tool)

        self.q0 = [self.joint_pos[joint_name] for joint_name in ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']]

        self.target_pose_in_base_frame_msg.pose.position.x = self.target_pose_in_apriltag_frame_msg.pose.position.x + pos_robot_relative_to_apriltag.pose.position.x
        self.target_pose_in_base_frame_msg.pose.position.y = self.target_pose_in_apriltag_frame_msg.pose.position.y + pos_robot_relative_to_apriltag.pose.position.y
        self.target_pose_in_base_frame_msg.pose.position.z = self.target_pose_in_apriltag_frame_msg.pose.position.z*0 + pos_robot_relative_to_apriltag.pose.position.z + Z_TOOL # TODO: multiply by zero?
        self.target_pose_in_base_frame_msg.pose.orientation.x=pos_robot_relative_to_apriltag.pose.orientation.x
        self.target_pose_in_base_frame_msg.pose.orientation.y=pos_robot_relative_to_apriltag.pose.orientation.y
        self.target_pose_in_base_frame_msg.pose.orientation.z=pos_robot_relative_to_apriltag.pose.orientation.z
        self.target_pose_in_base_frame_msg.pose.orientation.w=pos_robot_relative_to_apriltag.pose.orientation.w

        # self.desired_cartesian_trajectory_msg.pose.position.x = self.target_pose_in_base_frame_msg.pose.position.x
        # self.desired_cartesian_trajectory_msg.pose.position.y = self.target_pose_in_base_frame_msg.pose.position.y
        # self.desired_cartesian_trajectory_msg.pose.position.z = self.target_pose_in_base_frame_msg.pose.position.z

        self.T_tool = (spa.SE3(self.target_pose_in_base_frame_msg.pose.position.x,
                               self.target_pose_in_base_frame_msg.pose.position.y,
                               self.target_pose_in_base_frame_msg.pose.position.z)*RPY_init) #Matriz de transformação homogênea
        
        self.sol=self.UR20.ikine_LM(self.T_tool,self.q0) #Posição das juntas
        #print(self.sol)
        
        self.joint_trajectory_msg.joint_names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']

        point=JointTrajectoryPoint()
        point.positions=[self.sol.q[0], self.sol.q[1], self.sol.q[2], self.sol.q[3], self.sol.q[4], self.sol.q[5]]
        duration=Duration()
        duration.sec = T
        duration.nanosec = 0 #int(dt_CONTROLLER*1e9)
        point.time_from_start=duration
        self.joint_trajectory_msg.points=[point]

        # self.publisher_.publish(msg)
        self.get_logger().info('Publicando')

        # self.desired_cartesian_trajectory_publisher.publish(self.desired_cartesian_trajectory_msg)
        self.desired_cartesian_trajectory_publisher.publish(self.target_pose_in_base_frame_msg)
        print("Calculo antes da IK=",self.target_pose_in_base_frame_msg)
        print("Calculo depois da IK=",self.UR20.fkine(self.sol.q))
        # self.q0 = [self.joint_pos[joint_name] for joint_name in ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']]

        # if(self.dt>=self.T):
        #     self.dt=0
        #     return True

        # self.dt += dt_CONTROLLER
        # return False
             
    def motion_planner(self, T=15.0):
        '''This function is not actually used anymore. We're keeping it for reference.'''
        self.T = T
        self.target_pose_in_base_frame_msg.pose.position.x = self.target_pose_in_apriltag_frame_msg.pose.position.x + pos_robot_relative_to_apriltag.pose.position.x
        self.target_pose_in_base_frame_msg.pose.position.y = self.target_pose_in_apriltag_frame_msg.pose.position.y + pos_robot_relative_to_apriltag.pose.position.y
        self.target_pose_in_base_frame_msg.pose.position.z = self.target_pose_in_apriltag_frame_msg.pose.position.z*0 + pos_robot_relative_to_apriltag.pose.position.z # TODO: multiply by zero?
        
        self.q0 = [self.joint_pos[joint_name] for joint_name in ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']]

        print("CURRENT ROBOT'S POSE IN BASE FRAME = \n", self.current_pose_in_base_frame_msg)
        print("\nAPRILTAG POSE IN BASE FRAME = \n", pos_robot_relative_to_apriltag)
        print("\nTARGET POSE IN APRILTAG FRAME = \n", self.target_pose_in_apriltag_frame_msg)
        print("\nTARGET POSE IN BASE FRAME = \n", self.target_pose_in_base_frame_msg)

        A = np.array([[1,0, 0, 0],
                      [0,1,0,0],
                      [1,self.T,(self.T**2),(self.T**3)],
                      [0,1,2*self.T,3*(self.T**2)]])
        
        bx = np.array([[self.current_pose_in_base_frame_msg.pose.position.x],
                       [0.0],
                       [self.target_pose_in_base_frame_msg.pose.position.x],
                       [self.target_vel_in_apriltag_frame_msg.twist.linear.x]])
        
        by = np.array([[self.current_pose_in_base_frame_msg.pose.position.y],
                        [0.0],
                        [self.target_pose_in_base_frame_msg.pose.position.y],
                        [self.target_vel_in_apriltag_frame_msg.twist.linear.y]])
        
        bz = np.array([[self.current_pose_in_base_frame_msg.pose.position.z],
                       [0.0],
                       [self.target_pose_in_base_frame_msg.pose.position.z],
                       [self.target_vel_in_apriltag_frame_msg.twist.linear.z]])
        
        self.coeff_x = np.linalg.solve(A,bx)
        self.coeff_y = np.linalg.solve(A,by)
        self.coeff_z = np.linalg.solve(A,bz)
        
        '''
        #Solving to 3°: x
        # self.a_x=np.array([[1,0, 0, 0],
        #                    [0,1,0,0],
        #                    [1,self.T,(self.T**2),(self.T**3)],
        #                    [0,1,2*self.T,3*(self.T**2)]])
        # self.b_x=np.array([[self.x_o],
        #                    [self.xd_o],
        #                    [self.x_f],
        #                    [self.xd_f]])
        # self.coe_x=np.linalg.solve(self.a_x, self.b_x)


        # #Solving to 3°: y
        # self.a_y=np.array([[1,0, 0, 0],
        #                    [0,1,0,0],
        #                    [1,self.T,(self.T**2),(self.T**3)],
        #                    [0,1,2*self.T,3*(self.T**2)]])
        # self.b_y=np.array([[self.y_o],
        #                    [self.yd_o],
        #                    [self.y_f],
        #                    [self.yd_f]])
        # self.coe_y=np.linalg.solve(self.a_y,self.b_y)

        # #Solving to 3°: z
        # self.a_z=np.array([[1,0, 0, 0],
        #                    [0,1,0,0],
        #                    [1,self.T,(self.T**2),(self.T**3)],
        #                    [0,1,2*self.T,3*(self.T**2)]])
        # self.b_z=np.array([[self.z_o],
        #                    [self.zd_o],
        #                    [self.z_f],
        #                    [self.zd_f]])
        # self.coe_z=np.linalg.solve(self.a_z,self.b_z)
        '''


def main(args=None):
    rclpy.init(args=args)
    position_publisher = MoveToTargetNode()
    
    while position_publisher.current_pose_in_base_frame_msg.pose.position.x == 0.0:
        rclpy.spin_once(position_publisher)
    
    while position_publisher.target_pose_in_apriltag_frame_msg.pose.position.x == 0.0:
        rclpy.spin_once(position_publisher)

    while rclpy.ok():

        # print("Pressione qualquer tecla para executar")
        # input()

        T=5
        position_publisher.pos_controller_callback(T=T)
        # print("GOING TO \n", position_publisher.joint_trajectory_msg)
        position_publisher.publisher_.publish(position_publisher.joint_trajectory_msg)
        time_now = position_publisher.get_clock().now().to_msg().sec
        while np.linalg.norm(np.array([position_publisher.current_pose_in_base_frame_msg.pose.position.x,
                                       position_publisher.current_pose_in_base_frame_msg.pose.position.y,
                                       position_publisher.current_pose_in_base_frame_msg.pose.position.z]) - np.array([position_publisher.target_pose_in_base_frame_msg.pose.position.x,
                                       position_publisher.target_pose_in_base_frame_msg.pose.position.y,
                                       position_publisher.target_pose_in_base_frame_msg.pose.position.z])) > 0.01 or position_publisher.get_clock().now().to_msg().sec - time_now < T+2:
            rclpy.spin_once(position_publisher)
            
        rclpy.spin_once(position_publisher)

if __name__ == '__main__':
    main()