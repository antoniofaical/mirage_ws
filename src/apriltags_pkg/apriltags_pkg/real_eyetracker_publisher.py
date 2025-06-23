from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster,TransformListener
from rclpy.exceptions import ParameterNotDeclaredException
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String


import cv2
import numpy as np
from queue import Queue


APRILTAG_36h11_HALF_LENGTH = 81.0 #27.5
APRILTAG_TAH16H5_HALF_LENGTH = 76.5 #outro

CENTER_X_16H5=53 #
CENTER_Y_16H5=54 #

MY_BASE = 'base' # 'world'

class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('apriltag_reference',rclpy.Parameter.Type.STRING),
                ('apriltag_target',rclpy.Parameter.Type.STRING),
                ('interface_height',rclpy.Parameter.Type.INTEGER),
                ('interface_width',rclpy.Parameter.Type.INTEGER),
                ('apriltag_proportion',rclpy.Parameter.Type.DOUBLE),
                ('distance_proportion',rclpy.Parameter.Type.DOUBLE)
            ]
        )
        try:
            # Get parameters
            self.apriltag_reference = self.get_parameter('apriltag_reference').get_parameter_value().string_value
            self.apriltag_target = self.get_parameter('apriltag_target').get_parameter_value().string_value
            self.interface_height = self.get_parameter('interface_height').get_parameter_value().integer_value
            self.interface_width = self.get_parameter('interface_width').get_parameter_value().integer_value
            self.apriltag_proportion = self.get_parameter('apriltag_proportion').get_parameter_value().double_value
            self.distance_proportion = self.get_parameter('distance_proportion').get_parameter_value().double_value
            
            self.get_logger().info(f'AprilTag Reference: {self.apriltag_reference}')
            self.get_logger().info(f'AprilTag Target: {self.apriltag_target}')
            self.get_logger().info(f'Interface Height: {self.interface_height}')
            self.get_logger().info(f'Interface Width: {self.interface_width}')
            self.get_logger().info(f'Apriltag Proportion: {self.apriltag_proportion}')
            self.get_logger().info(f'Distance Proportion: {self.distance_proportion}')

        except ParameterNotDeclaredException as e:
            self.get_logger().error(f'Error: {e}')
            self.apriltag_reference = 'apriltag_TAG36H11'
            self.apriltag_target = 'apriltag_TAG16H5'
            self.interface_height = 600
            self.interface_width = 1200
            self.apriltag_proportion=0.3
            self.distance_proportion=0.05


        self.interface_variables()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher_point=self.create_publisher(PointStamped,'/target_info/position',10)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

        self.image_sub = self.create_subscription(
            Image,
            '/tobii_glasses/front_camera',
            self.image_callback,
            10)
        self.frame_sub = None


        qos_profile = rclpy.qos.QoSProfile(depth=10)
        qos_profile.durability = rclpy.qos.DurabilityPolicy.VOLATILE  
        self.gaze_subscription = self.create_subscription(
            String,
            '/tobii_glasses/gaze_position',
            self.gaze_callback,
            qos_profile)
        self.gaze_subscription  # Prevent unused variable warning
        self.gaze_position = (0, 0)  # Initialize gaze position
        self.gaze_buffer_size = 10
        self.gaze_x_buffer = []
        self.gaze_y_buffer = []
        self.array_point_x_old=[0,0,0,0]
        self.array_point_y_old=[0,0,0,0]



        self.apriltag_buffer_x= Queue(maxsize=5)
        self.apriltag_buffer_y= Queue(maxsize=5)
        self.apriltag_buffer_x= []
        self.apriltag_buffer_y= []
        self.apriltag_buffer_size=5




    def broadcast_timer_callback(self):
        ##############################################
        # 1° Parte: Abrindo câmera
        stream=self.frame_sub
        #alpha=1.5
        #beta=50.0

        #stream = cv2.convertScaleAbs(stream, alpha=alpha, beta=beta)

        if stream is not None:
            
            frame=stream
        ##########################################
        #2° Parte: Detecção da Apriltag
        #Parâmetros para o 3D:

            # camera_matrix = np.array([[963.38, 0, 670.202], 
            #                   [0, 960.79, 326.59], 
            #                   [0, 0, 1]], np.float32) 
            # dist_coeffs = np.array([[0.051362, -0.20001669, -0.005869, 0.00217872, 0.2262359]], np.float32)

            camera_matrix = np.array([[771.33, 0, 627.400], 
                              [0, 771.326, 335.60], 
                              [0, 0, 1]], np.float32) 
            dist_coeffs = np.array([[0.10335281, -0.19184085, 0.00443415, 0.01376285, 0.11262108]], np.float32)

            undistorted_frame = frame
            undistorted_april = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)

            #Primiera Apriltag: Tag36h11
            april_detec_TAG36H11 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            parameters = cv2.aruco.DetectorParameters_create()
            corners_TAG36H11, ids_TAG36H11, _ = cv2.aruco.detectMarkers(undistorted_april, april_detec_TAG36H11, parameters=parameters)

            height_gaze, width_gaze, _ = undistorted_frame.shape
            x_gaze = int(self.x_gaze_on_eyetracker_frame * width_gaze)
            y_gaze = int(self.y_gaze_on_eyetracker_frame * height_gaze)
            cv2.circle(undistorted_frame, (x_gaze, y_gaze), 10, (0, 255, 0), -1)


        #############################################################################
        #############################################################################
        #############################PRIMEIRA APRILTAG###############################

            if ids_TAG36H11 is not None:

                array_point_x=[] ###########
                array_point_y=[] ###########

                cv2.aruco.drawDetectedMarkers(undistorted_frame, corners_TAG36H11, ids_TAG36H11)

                for i in range(len(ids_TAG36H11)):
  
                        tag_corners = corners_TAG36H11[i][0]  # Obtém os 4 cantos do marcador
                        center_x_TAG36H11 = int(np.mean(tag_corners[:, 0]))  # Média das coordenadas x
                        center_y_TAG36H11 = int(np.mean(tag_corners[:, 1]))  # Média das coordenadas y
                        center_TAG36H11 = (center_x_TAG36H11, center_y_TAG36H11)
                        cv2.circle(undistorted_frame, center_TAG36H11, 5, (0, 0, 255), -1)

                        if (ids_TAG36H11[i][0]==0):
                        #CALCULO DO PONTO
                            #Dimensões da apriltag encontrada:

                            top_left,top_right,bottom_right,bottom_left=corners_TAG36H11[i][0]

                            width_top=np.linalg.norm(top_right-top_left)
                            width_bottom=np.linalg.norm(bottom_right-bottom_left)
                            width=(width_top+width_bottom)/2


                            height_left=np.linalg.norm(bottom_left-top_left)
                            height_right=np.linalg.norm(bottom_right-top_right)
                            height=(height_left+height_right)/2

                            Con_x=int(((self.SIZE_APRILTAG36H11_INTERFACE*(center_x_TAG36H11-x_gaze))/(width)))
                            Con_y=int(((self.SIZE_APRILTAG36H11_INTERFACE*(center_y_TAG36H11-y_gaze))/(height)))
                            #Circulo
                            P_x=self.CENTER_X_36H11_0-Con_x
                            P_y=self.CENTER_Y_36H11_0-Con_y

                            array_point_x.append(P_x)
                            array_point_y.append(P_y)


                        if (ids_TAG36H11[i][0]==1):
                        #CALCULO DO PONTO
                            #Dimensões da apriltag encontrada:
                            top_left,top_right,bottom_right,bottom_left=corners_TAG36H11[i][0]

                            width_top=np.linalg.norm(top_right-top_left)
                            width_bottom=np.linalg.norm(bottom_right-bottom_left)
                            width=(width_top+width_bottom)/2


                            height_left=np.linalg.norm(bottom_left-top_left)
                            height_right=np.linalg.norm(bottom_right-top_right)
                            height=(height_left+height_right)/2

                            #Conversão de resolução
                            #196 and 179 are the apriltag's dimensions (in pixels) on the visual interface
                            Con_x=int(((self.SIZE_APRILTAG36H11_INTERFACE*(center_x_TAG36H11-x_gaze))/(width)))
                            Con_y=int(((self.SIZE_APRILTAG36H11_INTERFACE*(center_y_TAG36H11-y_gaze))/(height)))
                            #Circulo
                            P_x=self.CENTER_X_36H11_1-Con_x
                            P_y=self.CENTER_Y_36H11_1-Con_y

                            array_point_x.append(P_x)
                            array_point_y.append(P_y)


                        if (ids_TAG36H11[i][0]==2):
                        #CALCULO DO PONTO
                            #Dimensões da apriltag encontrada:
                            top_left,top_right,bottom_right,bottom_left=corners_TAG36H11[i][0]

                            width_top=np.linalg.norm(top_right-top_left)
                            width_bottom=np.linalg.norm(bottom_right-bottom_left)
                            width=(width_top+width_bottom)/2


                            height_left=np.linalg.norm(bottom_left-top_left)
                            height_right=np.linalg.norm(bottom_right-top_right)
                            height=(height_left+height_right)/2

                            Con_x=int(((self.SIZE_APRILTAG36H11_INTERFACE*(center_x_TAG36H11-x_gaze))/(width)))
                            Con_y=int(((self.SIZE_APRILTAG36H11_INTERFACE*(center_y_TAG36H11-y_gaze))/(height)))
                            #Circulo
                            P_x=self.CENTER_X_36H11_2-Con_x
                            P_y=self.CENTER_Y_36H11_2-Con_y

                            array_point_x.append(P_x)
                            array_point_y.append(P_y)


                        if (ids_TAG36H11[i][0]==3):
                        #CALCULO DO PONTO
                            #Dimensões da apriltag encontrada:
                            top_left,top_right,bottom_right,bottom_left=corners_TAG36H11[i][0]

                            width_top=np.linalg.norm(top_right-top_left)
                            width_bottom=np.linalg.norm(bottom_right-bottom_left)
                            width=(width_top+width_bottom)/2


                            height_left=np.linalg.norm(bottom_left-top_left)
                            height_right=np.linalg.norm(bottom_right-top_right)
                            height=(height_left+height_right)/2

                            #Conversão de resolução
                            #196 and 179 are the apriltag's dimensions (in pixels) on the visual interface
                            Con_x=int(((self.SIZE_APRILTAG36H11_INTERFACE*(center_x_TAG36H11-x_gaze))/(width)))
                            Con_y=int(((self.SIZE_APRILTAG36H11_INTERFACE*(center_y_TAG36H11-y_gaze))/(height)))
                            #Circulo
                            P_x=self.CENTER_X_36H11_3-Con_x
                            P_y=self.CENTER_Y_36H11_3-Con_y

                            array_point_x.append(P_x)
                            array_point_y.append(P_y)
                        

            ###########################################
            #Pose estimation

                        objectPoints_TAG36H11 = np.array([
                            [-APRILTAG_36h11_HALF_LENGTH, -APRILTAG_36h11_HALF_LENGTH, 0],  # Canto superior esquerdo (em mm) (NEGATIVO,NEGATIVO)
                            [ APRILTAG_36h11_HALF_LENGTH, -APRILTAG_36h11_HALF_LENGTH, 0],  # Canto superior direito ()
                            [ APRILTAG_36h11_HALF_LENGTH,  APRILTAG_36h11_HALF_LENGTH, 0],  # Canto inferior direito ()
                            [-APRILTAG_36h11_HALF_LENGTH,  APRILTAG_36h11_HALF_LENGTH, 0]   # Canto inferior esquerdo ()
                        ], dtype=np.float32)

                        corners2_TAG36H11=corners_TAG36H11[i][0]

                        ret_TAG36H11,rvecs_TAG36H11, tvecs_TAG36H11 = cv2.solvePnP(objectPoints_TAG36H11*-1, corners2_TAG36H11, camera_matrix, dist_coeffs)
            ##############################################
            #Desenhando frame
                        axis_TAG36H11 = np.float32([[APRILTAG_36h11_HALF_LENGTH,0,0], [0, APRILTAG_36h11_HALF_LENGTH,0], [0,0,APRILTAG_36h11_HALF_LENGTH]]).reshape(-1,3) #Tamanho do eixo que será plotado
                        td_points_36H11,_=cv2.projectPoints(axis_TAG36H11, rvecs_TAG36H11, tvecs_TAG36H11, camera_matrix, dist_coeffs) #Projetando os pontos
                        point_xf_TAG36H11=(int(td_points_36H11[0][0][0]), int(td_points_36H11[0][0][1])) #Definição do eixo x
                        cv2.line(undistorted_frame,center_TAG36H11, point_xf_TAG36H11, (0, 0, 255), 2) #Desenhando eixo x
                        point_yf_TAG36H11=(int(td_points_36H11[1][0][0]), int(td_points_36H11[1][0][1])) #Definição do eixo y
                        cv2.line(undistorted_frame,center_TAG36H11, point_yf_TAG36H11, (0, 255, 0), 2) #Desenhando eixo y
                        point_zf_TAG36H11=(int(td_points_36H11[2][0][0]),int(td_points_36H11[2][0][1])) #Definição do eixo z
                        cv2.line(undistorted_frame,center_TAG36H11, point_zf_TAG36H11, (255, 0, 0), 2) #Desenhando eixo z
                        rotation_matrix_TAG36H11,_=cv2.Rodrigues(rvecs_TAG36H11)
                        rotation_TAG36H11=R.from_matrix(rotation_matrix_TAG36H11)
                        rvec_quaternion_TAG36H11=rotation_TAG36H11.as_quat()

##############################################
#####################################################################################
#####################################################################################
############################PLOT FRAME###############################################
            #7° Parte: Enviando as transformadas

                        if(ids_TAG36H11[i][0]==0):
                                t_TAG36H11_0 = TransformStamped()
                                t_TAG36H11_0.header.stamp = self.get_clock().now().to_msg()
                                t_TAG36H11_0.header.frame_id = MY_BASE
                                t_TAG36H11_0.child_frame_id = self.apriltag_reference
                                t_TAG36H11_0.transform.translation.x = float(tvecs_TAG36H11[0]/1000)
                                t_TAG36H11_0.transform.translation.y = float(tvecs_TAG36H11[1]/1000)
                                t_TAG36H11_0.transform.translation.z = float(tvecs_TAG36H11[2]/1000)
                                t_TAG36H11_0.transform.rotation.x = rvec_quaternion_TAG36H11[0]
                                t_TAG36H11_0.transform.rotation.y = rvec_quaternion_TAG36H11[1]
                                t_TAG36H11_0.transform.rotation.z = rvec_quaternion_TAG36H11[2]
                                t_TAG36H11_0.transform.rotation.w = rvec_quaternion_TAG36H11[3]


                                self.tf_broadcaster.sendTransform(t_TAG36H11_0) #April_tag frame
                    
                        if(ids_TAG36H11[i][0]==1):
                                t_TAG36H11_1 = TransformStamped()
                                t_TAG36H11_1.header.stamp = self.get_clock().now().to_msg()
                                t_TAG36H11_1.header.frame_id = MY_BASE
                                t_TAG36H11_1.child_frame_id = 'apriltag_TAG36H11_1'
                                t_TAG36H11_1.transform.translation.x = float(tvecs_TAG36H11[0]/1000)
                                t_TAG36H11_1.transform.translation.y = float(tvecs_TAG36H11[1]/1000)
                                t_TAG36H11_1.transform.translation.z = float(tvecs_TAG36H11[2]/1000)
                                t_TAG36H11_1.transform.rotation.x = rvec_quaternion_TAG36H11[0]
                                t_TAG36H11_1.transform.rotation.y = rvec_quaternion_TAG36H11[1]
                                t_TAG36H11_1.transform.rotation.z = rvec_quaternion_TAG36H11[2]
                                t_TAG36H11_1.transform.rotation.w = rvec_quaternion_TAG36H11[3]


                                self.tf_broadcaster.sendTransform(t_TAG36H11_1) #April_tag frame

                        if(ids_TAG36H11[i][0]==2):
                                t_TAG36H11_2 = TransformStamped()
                                t_TAG36H11_2.header.stamp = self.get_clock().now().to_msg()
                                t_TAG36H11_2.header.frame_id = MY_BASE
                                t_TAG36H11_2.child_frame_id = 'apriltag_TAG36H11_2'
                                t_TAG36H11_2.transform.translation.x = float(tvecs_TAG36H11[0]/1000)
                                t_TAG36H11_2.transform.translation.y = float(tvecs_TAG36H11[1]/1000)
                                t_TAG36H11_2.transform.translation.z = float(tvecs_TAG36H11[2]/1000)
                                t_TAG36H11_2.transform.rotation.x = rvec_quaternion_TAG36H11[0]
                                t_TAG36H11_2.transform.rotation.y = rvec_quaternion_TAG36H11[1]
                                t_TAG36H11_2.transform.rotation.z = rvec_quaternion_TAG36H11[2]
                                t_TAG36H11_2.transform.rotation.w = rvec_quaternion_TAG36H11[3]


                                self.tf_broadcaster.sendTransform(t_TAG36H11_2) #April_tag frame

                        if(ids_TAG36H11[i][0]==3):
                                t_TAG36H11_3 = TransformStamped()
                                t_TAG36H11_3.header.stamp = self.get_clock().now().to_msg()
                                t_TAG36H11_3.header.frame_id = MY_BASE
                                t_TAG36H11_3.child_frame_id = 'apriltag_TAG36H11_3'
                                t_TAG36H11_3.transform.translation.x = float(tvecs_TAG36H11[0]/1000)
                                t_TAG36H11_3.transform.translation.y = float(tvecs_TAG36H11[1]/1000)
                                t_TAG36H11_3.transform.translation.z = float(tvecs_TAG36H11[2]/1000)
                                t_TAG36H11_3.transform.rotation.x = rvec_quaternion_TAG36H11[0]
                                t_TAG36H11_3.transform.rotation.y = rvec_quaternion_TAG36H11[1]
                                t_TAG36H11_3.transform.rotation.z = rvec_quaternion_TAG36H11[2]
                                t_TAG36H11_3.transform.rotation.w = rvec_quaternion_TAG36H11[3]


                                self.tf_broadcaster.sendTransform(t_TAG36H11_3) #April_tag frame
               
               
                med_point_x_raw=(np.mean(array_point_x))
                med_point_y_raw=(np.mean(array_point_y))

                self.apriltag_buffer_x.append(med_point_x_raw)
                self.apriltag_buffer_y.append(med_point_y_raw)

                if len(self.apriltag_buffer_x) > self.apriltag_buffer_size:

                    self.apriltag_buffer_x.pop(0)
                    self.apriltag_buffer_y.pop(0)

                #med_point_x=np.mean(self.apriltag_buffer_x)
                #med_point_y=np.mean(self.apriltag_buffer_y)

                med_point_x= sum(self.apriltag_buffer_x)/len(self.apriltag_buffer_x)
                med_point_y= sum(self.apriltag_buffer_y)/len(self.apriltag_buffer_y)

                # MELHOR COM PROJETOR (Manter estabilidade). Ruim com caso real
                # if(len(array_point_x)!=4):
                #      med_point_x_old=int(np.mean(self.array_point_x_old))
                #      med_point_x=int((med_point_x+med_point_x_old*3)/4)
                # else:
                #      self.array_point_x_old=array_point_x

                # if(len(array_point_y)!=4):
                #      med_point_y_old=int(np.mean(self.array_point_y_old))
                #      med_point_y=int((med_point_y+med_point_y_old*3)/4)
                # else:
                #      self.array_point_y_old=array_point_y

                msg=PointStamped()
                msg.header.stamp=self.get_clock().now().to_msg()
                msg.header.frame_id=MY_BASE
                msg.point.x=float(med_point_x)
                msg.point.y=float(med_point_y)
                msg.point.z=0.0

                self.publisher_point.publish(msg)
        
        #############################################################################
        #############################################################################
        #############################SEGUNDA APRILTAG###############################
         
            
            # Naming a window 
            cv2.namedWindow("Webcam", cv2.WINDOW_NORMAL) 
            # Using resizeWindow() 
            cv2.resizeWindow("Webcam", 900, 900) 
            cv2.imshow("Webcam",undistorted_frame)
            if cv2.waitKey(1) == ord('q'):
                return
    
    def image_callback(self, data):

        self.bridge = CvBridge()
        try:
            if data.encoding == '8UC3':
                self.frame_sub = self.bridge.imgmsg_to_cv2(data, "bgr8")
            else:
                self.frame_sub = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return 

    def gaze_callback(self, msg):
        # Parse the gaze position data
        try:
            x_str, y_str = msg.data.split(',')
            self.gaze_position = (float(x_str), float(y_str))

            self.gaze_x_buffer.append(self.gaze_position[0])
            self.gaze_y_buffer.append(self.gaze_position[1])

            if len(self.gaze_x_buffer) > self.gaze_buffer_size:

                self.gaze_x_buffer.pop(0)
                self.gaze_y_buffer.pop(0)

            self.x_gaze_on_eyetracker_frame = sum(self.gaze_x_buffer) / len(self.gaze_x_buffer)
            self.y_gaze_on_eyetracker_frame = sum(self.gaze_y_buffer) / len(self.gaze_y_buffer)

        except Exception as e:
            self.get_logger().error(f"Failed to parse gaze position: {e}")     

    def interface_variables(self):
        self.SIZE_APRILTAG36H11_INTERFACE= int(self.apriltag_proportion*self.interface_height) #apriltagproportion*interface_height
        self.CENTER_X_36H11_0=int((self.distance_proportion*self.interface_width)+self.SIZE_APRILTAG36H11_INTERFACE/2) #DISTANCEPROPORTION*interface_width+SIZE_APRILTAG36H11_INTERFACE/2
        self.CENTER_Y_36H11_0=int((self.distance_proportion*self.interface_height)+self.SIZE_APRILTAG36H11_INTERFACE/2)  #DISTANCEPROPORTION*interface_height+SIZE_APRILTAG36H11_INTERFACE/2
        self.CENTER_X_36H11_1=int((self.distance_proportion*self.interface_width)+self.SIZE_APRILTAG36H11_INTERFACE/2) #DISTANCEPROPORTION*interface_width+SIZE_APRILTAG36H11_INTERFACE/2
        self.CENTER_Y_36H11_1=int(self.interface_height-(self.distance_proportion*self.interface_height)-self.SIZE_APRILTAG36H11_INTERFACE/2) #DISTANCEPROPORTION*interface_height+SIZE_APRILTAG36H11_INTERFACE/2
        self.CENTER_X_36H11_2=int(self.interface_width-(self.distance_proportion*self.interface_width)-self.SIZE_APRILTAG36H11_INTERFACE/2) #
        self.CENTER_Y_36H11_2=int(self.interface_height-(self.distance_proportion*self.interface_height)-self.SIZE_APRILTAG36H11_INTERFACE/2) #
        self.CENTER_X_36H11_3=int(self.interface_width-(self.distance_proportion*self.interface_width)-self.SIZE_APRILTAG36H11_INTERFACE/2) #
        self.CENTER_Y_36H11_3=int((self.distance_proportion*self.interface_height)+self.SIZE_APRILTAG36H11_INTERFACE/2) #

         
def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()