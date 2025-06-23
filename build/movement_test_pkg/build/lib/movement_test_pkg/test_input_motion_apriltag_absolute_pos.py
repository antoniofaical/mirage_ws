import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import rclpy.parameter
import roboticstoolbox as rbt
from math import pi
import spatialmath as spa
from builtin_interfaces.msg import Duration

from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

####ANOTACOES PARA SUBSTITUICAO COM AS APRILTAGS########
## RETIRAR A PARTE INICIAL DE COLOCAR O PONTO.
## COMO AS TF JA SERAO FORMADAS NO RVIZ2, SERA NECESSARIO,APENAS, REALIZAR A TRANSFORMADA E PEGAR O PONTO.
#PROXIMAS ATIVIDADES ENT:
# IMPRIMIR A TAG16H5
# ESCREVER UM CODIGO ROS2 DE RECONHECIMENTO DAS DUAS E REALIZAR A TRANSFORMADA DE UMA PRA OUTRA
# SE A TRANSFORMADA FUNCIONAR BEM E TUDO FUNCIONAR, ESCREVER UM RASCUNHO PARA BANCADA DE SEXTA (04/09)


class PublisherNode(Node):

    def __init__(self):
        super().__init__('square_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.timer= self.create_timer(1,self.timer_callback)

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
        self.T_ool=None
        self.sol=None
        self.coe_x=None
        self.coe_y=None
        self.i=0.0
        self.x_val=0.0
        self.y_val=0.0
        self.z_val=0.0
        self.q0=[-1.600, -1.7277, -2.20295, -0.8079, 1.5950, -0.03106]
        self.controle=0.0
        #Variáveis para o planejador de trajerória e input:
        #Posição: x:-0.221 mm , y:-0.796 mm , z: 0.412 mm
        self.x_o_o=-0.221
        self.y_o_o=-0.796
        self.z_o_o=0.412
        self.x_f_f=None
        self.y_f_f=None
        self.z_f_f=None

    def timer_callback(self):
        ###########################CONFIGURAÇÃO INICIAL PADRÃO(Pose)#############################
        #Configuração inicial para uma boa trajetória
        #Posição: x:-0.221 mm , y:-0.796 mm , z: 0.412 mm
        #Ângulo de junta: [-1.600, -1.7277, -2.20295, -0.8079, 1.5950, -0.03106]
        #Rotação: Roll: 3.117 rad, Pitch:0.0025 rad, Yaw:-3.141 rad (ferramenta para baixo)
        ################REALIZAÇÃO DA TRAJETORIA#################################################

        self.x_val=self.coe_x[0]+self.coe_x[1]*self.i+self.coe_x[2]*(self.i**2)+self.coe_x[3]*(self.i**3)
        self.x_val=float(self.x_val)

        self.y_val=self.coe_y[0]+self.coe_y[1]*self.i+self.coe_y[2]*(self.i**2)+self.coe_y[3]*(self.i**3)
        self.y_val=float(self.y_val)

        self.z_val=self.coe_z[0]+self.coe_z[1]*self.i+self.coe_z[2]*(self.i**2)+self.coe_z[3]*(self.i**3)
        self.z_val=float(self.z_val)


        #Tool points para construção de um quadrado com planejador de trajetórioa de terceira ordem
        #Posição: Definidas acima
        #Rotação: Roll: 3.117 rad, Pitch:0.0025 rad, Yaw:-3.141 rad (ferramenta para baixo)
        self.T_ool=(spa.SE3(self.x_val,self.y_val,self.z_val)*spa.SE3.RPY(3.117,0.025,-3.141)) #Matriz de transformação homogênea
        print(self.T_ool)
        #############Cinemática inversa#################
        self.sol=self.UR20.ikine_LM(self.T_ool,self.q0) #Posição das juntas
        print(self.sol)
        ######################CALCULO CINEMATICO#######################################
        ######################CONSTRUCAO DA MENSAGEM###################################
        msg=JointTrajectory()
        msg.joint_names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        point=JointTrajectoryPoint()
        point.positions=[self.sol.q[0], self.sol.q[1], self.sol.q[2], self.sol.q[3], self.sol.q[4], self.sol.q[5]]
        duration=Duration()
        duration.sec=1
        duration.nanosec=0
        point.time_from_start=duration
        msg.points=[point]

        ########################PUBLICANDO###############################################

        self.publisher_.publish(msg)
        self.get_logger().info('Publicando')

        self.q0=[self.sol.q[0], self.sol.q[1], self.sol.q[2], self.sol.q[3], self.sol.q[4], self.sol.q[5]]

        if(self.i==self.T):
            self.i=0
            self.controle=0.0

        self.i=1+self.i
        
    def motion_planner(self):
        print(self.x_o_o)
        print(self.x_f_f)
        #Planejador de trajetório de 3° ordem
        #Defining initial and final positions (x):
        self.x_o=self.x_o_o
        self.x_f=self.x_f_f
        self.xd_o=0.0
        self.xd_f=0.0
        #Defining initial and final positions (y):
        self.y_o=self.y_o_o
        self.y_f=self.y_f_f
        self.yd_o=0.0
        self.yd_f=0.0
        #Defining initial and final positions (z):
        self.z_o=self.z_o_o
        self.z_f=self.z_f_f
        self.zd_o=0.0
        self.zd_f=0.0
        #Defining the time lapse:
        self.T=15.0 #s
        #Solving to 3°: x
        self.a_x=np.array([[1,0, 0, 0],[0,1,0,0],[1,self.T,(self.T**2),(self.T**3)],[0,1,2*self.T,3*(self.T**2)]])
        self.b_x=np.array([[self.x_o],[self.xd_o],[self.x_f],[self.xd_f]])
        self.coe_x=np.linalg.solve(self.a_x,self.b_x)
        #Solving to 3°: y
        self.a_y=np.array([[1,0, 0, 0],[0,1,0,0],[1,self.T,(self.T**2),(self.T**3)],[0,1,2*self.T,3*(self.T**2)]])
        self.b_y=np.array([[self.y_o],[self.yd_o],[self.y_f],[self.yd_f]])
        self.coe_y=np.linalg.solve(self.a_y,self.b_y)
        #Solving to 3°: z
        self.a_z=np.array([[1,0, 0, 0],[0,1,0,0],[1,self.T,(self.T**2),(self.T**3)],[0,1,2*self.T,3*(self.T**2)]])
        self.b_z=np.array([[self.z_o],[self.zd_o],[self.z_f],[self.zd_f]])
        self.coe_z=np.linalg.solve(self.a_z,self.b_z)
        #Defining controll
        self.controle=1.0
        #New initial positions
        self.x_o_o=self.x_f_f
        self.y_o_o=self.y_f_f
        self.z_o_o=self.z_f_f


   
        
class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_frame')
        #Variáveis para transformada e input
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.transform_point)

        self.trans_controll=0
        self.x_trans=None

    def transform_point(self):
        #Transformada
        from_frame_rel = 'apriltag_TAG16H5'
        to_frame_rel = 'base' #world
    
        trans = None

        timeout = rclpy.duration.Duration(seconds=0.1)

        available_frames = self.tf_buffer.all_frames_as_string()
        self.get_logger().info(f'Frames disponíveis após inicialização: {available_frames}')
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now,timeout=timeout)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        print(trans.transform.translation.x*(-1))
        self.x_trans = trans.transform.translation.x #*(-1)
        self.y_trans = trans.transform.translation.y #*(-1)
        self.z_trans = trans.transform.translation.z
        self.trans_controll=1+self.trans_controll



def main(args=None):
    rclpy.init(args=args)
    square_publisher = PublisherNode()
    transform_frame = TransformNode()
    #Node da transformada
    while rclpy.ok():
        if(square_publisher.controle==0.0):
            #Esperando input para controle
            print("Pressione qualquer tecla")
            input()
            print("Continuando...")

            print("************Recolhendo os frames e transformando********************")
            while (transform_frame.trans_controll!=3):
              rclpy.spin_once(transform_frame)
            transform_frame.trans_controll=0

            print("************Planejando trajetoria********************")
            square_publisher.x_f_f=transform_frame.x_trans
            square_publisher.y_f_f=transform_frame.y_trans
            square_publisher.z_f_f=transform_frame.z_trans
            square_publisher.motion_planner() 

        if(square_publisher.controle==1.0):
            print("***************Executando Trajetoria*******************")
            rclpy.spin_once(square_publisher)


if __name__ == '__main__':
    main()