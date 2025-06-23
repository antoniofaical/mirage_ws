from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster,TransformListener
from rclpy.exceptions import ParameterNotDeclaredException
from scipy.spatial.transform import Rotation as R

import cv2
import numpy as np

APRILTAG_36h11_HALF_LENGTH = 50.0 #27.5
APRILTAG_TAH16H5_HALF_LENGTH = 25.0 #outro
MY_BASE = 'base' # 'world'

class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

        try:
            # Get parameters
            self.apriltag_reference = self.get_parameter('apriltag_reference').get_parameter_value().string_value
            self.apriltag_target = self.get_parameter('apriltag_target').get_parameter_value().string_value
            
            self.get_logger().info(f'AprilTag Reference: {self.apriltag_reference}')
            self.get_logger().info(f'AprilTag Target: {self.apriltag_target}')
        except ParameterNotDeclaredException as e:
            self.get_logger().error(f'Error: {e}')
            self.apriltag_reference = 'apriltag_TAG36H11'
            self.apriltag_target = 'apriltag_TAG16H5'


    def broadcast_timer_callback(self):
        ##############################################
        # 1° Parte: Abrindo câmera
        stream = cv2.VideoCapture(2)
        stream.set(cv2.CAP_PROP_FRAME_WIDTH,1280) 
        stream.set(cv2.CAP_PROP_FRAME_HEIGHT,720)

        if not stream.isOpened():
            print("No stream")
            exit()

        while(True):
            ret, frame = stream.read()
            if not ret:
                print("No more stream")
                break
        ##########################################
        #2° Parte: Detecção da Apriltag
        #Parâmetros para o 3D:

            camera_matrix = np.array([[963.38, 0, 670.202], 
                              [0, 960.79, 326.59], 
                              [0, 0, 1]], np.float32) 
            dist_coeffs = np.array([[0.051362, -0.20001669, -0.005869, 0.00217872, 0.2262359]], np.float32)

            undistorted_frame = frame
            undistorted_april = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)

            #Primiera Apriltag: Tag36h11
            april_detec_TAG36H11 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            parameters = cv2.aruco.DetectorParameters_create()
            corners_TAG36H11, ids_TAG36H11, _ = cv2.aruco.detectMarkers(undistorted_april, april_detec_TAG36H11, parameters=parameters)

            #Segunda Apriltag: Tag16h5
            april_detec_TAG16H5=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
            parameters = cv2.aruco.DetectorParameters_create()
            corners_TAG16H5, ids_TAH16H5, _ = cv2.aruco.detectMarkers(undistorted_april, april_detec_TAG16H5, parameters=parameters)

        #############################################################################
        #############################################################################
        #############################PRIMEIRA APRILTAG###############################


            if ids_TAG36H11 is not None:
                cv2.aruco.drawDetectedMarkers(undistorted_frame, corners_TAG36H11, ids_TAG36H11)

                for i in range(len(ids_TAG36H11)):
                        # print(f"ID detectado: {ids_TAG36H11[i][0]}")
                        tag_corners = corners_TAG36H11[i][0]  # Obtém os 4 cantos do marcador
                        center_x_TAG36H11 = int(np.mean(tag_corners[:, 0]))  # Média das coordenadas x
                        center_y_TAG36H11 = int(np.mean(tag_corners[:, 1]))  # Média das coordenadas y
                        center_TAG36H11 = (center_x_TAG36H11, center_y_TAG36H11)
                        cv2.circle(undistorted_frame, center_TAG36H11, 5, (0, 0, 255), -1)

            ###########################################
            #Pose estimation

                        objectPoints_TAG36H11 = np.array([
                            [-APRILTAG_36h11_HALF_LENGTH, -APRILTAG_36h11_HALF_LENGTH, 0],  # Canto superior esquerdo (em mm)
                            [ APRILTAG_36h11_HALF_LENGTH, -APRILTAG_36h11_HALF_LENGTH, 0],  # Canto superior direito
                            [ APRILTAG_36h11_HALF_LENGTH,  APRILTAG_36h11_HALF_LENGTH, 0],  # Canto inferior direito
                            [-APRILTAG_36h11_HALF_LENGTH,  APRILTAG_36h11_HALF_LENGTH, 0]   # Canto inferior esquerdo
                        ], dtype=np.float32)

                        corners2_TAG36H11=corners_TAG36H11[i][0]

                        ret_TAG36H11,rvecs_TAG36H11, tvecs_TAG36H11 = cv2.solvePnP(objectPoints_TAG36H11, corners2_TAG36H11, camera_matrix, dist_coeffs)
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
        #############################################################################
        #############################################################################
        #############################SEGUNDA APRILTAG###############################


            if ids_TAH16H5 is not None:
            
                cv2.aruco.drawDetectedMarkers(undistorted_frame, corners_TAG16H5, ids_TAH16H5)

                for i in range(len(ids_TAH16H5)):
                        # print(f"ID detectado: {ids_TAH16H5[i][0]}")

                        tag_corners_TAG16H5 = corners_TAG16H5[i][0]  # Obtém os 4 cantos do marcador
                        center_x_TAG16H5 = int(np.mean(tag_corners_TAG16H5[:, 0]))  # Média das coordenadas x
                        center_y_TAG16H5 = int(np.mean(tag_corners_TAG16H5[:, 1]))  # Média das coordenadas y
                        center_TAG16H5 = (center_x_TAG16H5, center_y_TAG16H5)
                        cv2.circle(undistorted_frame, center_TAG16H5, 5, (0, 0, 255), -1)

            ################################################################
            #Pose estimation

                        objectPoints_TAG16H5 = np.array([
                            [-APRILTAG_TAH16H5_HALF_LENGTH, -APRILTAG_TAH16H5_HALF_LENGTH, 0],  # Canto superior esquerdo (em mm)
                            [ APRILTAG_TAH16H5_HALF_LENGTH, -APRILTAG_TAH16H5_HALF_LENGTH, 0],  # Canto superior direito
                            [ APRILTAG_TAH16H5_HALF_LENGTH,  APRILTAG_TAH16H5_HALF_LENGTH, 0],  # Canto inferior direito
                            [-APRILTAG_TAH16H5_HALF_LENGTH,  APRILTAG_TAH16H5_HALF_LENGTH, 0]   # Canto inferior esquerdo
                        ], dtype=np.float32)

                        corners2_TAG16H5=corners_TAG16H5[i][0]

                        ret_TAG16H5,rvecs_TAG16H5, tvecs_TAG16H5 = cv2.solvePnP(objectPoints_TAG16H5, corners2_TAG16H5, camera_matrix, dist_coeffs)
            ##############################################
            #Desenhando frame

                        axis_TAG16H5 = np.float32([[APRILTAG_TAH16H5_HALF_LENGTH,0,0], [0,APRILTAG_TAH16H5_HALF_LENGTH,0], [0,0,APRILTAG_TAH16H5_HALF_LENGTH]]).reshape(-1,3) #Tamanho do eixo que será plotado
                        td_points_16H5,_=cv2.projectPoints(axis_TAG16H5, rvecs_TAG16H5, tvecs_TAG16H5, camera_matrix, dist_coeffs) #Projetando os pontos
                        point_xf_TAG16H5=(int(td_points_16H5[0][0][0]), int(td_points_16H5[0][0][1])) #Definição do eixo x
                        cv2.line(undistorted_frame,center_TAG16H5, point_xf_TAG16H5, (0, 0, 255), 2) #Desenhando eixo x
                        point_yf_TAG16H5=(int(td_points_16H5[1][0][0]), int(td_points_16H5[1][0][1])) #Definição do eixo y
                        cv2.line(undistorted_frame,center_TAG16H5, point_yf_TAG16H5, (0, 255, 0), 2) #Desenhando eixo y
                        point_zf_TAG16H5=(int(td_points_16H5[2][0][0]),int(td_points_16H5[2][0][1])) #Definição do eixo z
                        cv2.line(undistorted_frame,center_TAG16H5, point_zf_TAG16H5, (255, 0, 0), 2) #Desenhando eixo z

                        rotation_matrix_TAG16H5,_=cv2.Rodrigues(rvecs_TAG16H5)
                        rotation_TAG16H5=R.from_matrix(rotation_matrix_TAG16H5)
                        rvec_quaternion_TAG16H5=rotation_TAG16H5.as_quat()
##############################################
#####################################################################################
#####################################################################################
############################PLOT FRAME###############################################
            #7° Parte: Enviando as transformadas

                        t_16H5 = TransformStamped()
                        t_16H5.header.stamp = self.get_clock().now().to_msg()
                        t_16H5.header.frame_id = MY_BASE
                        t_16H5.child_frame_id = self.apriltag_target
                        t_16H5.transform.translation.x = float(tvecs_TAG16H5[0]/1000)
                        t_16H5.transform.translation.y = float(tvecs_TAG16H5[1]/1000)
                        t_16H5.transform.translation.z = float(tvecs_TAG16H5[2]/1000)
                        t_16H5.transform.rotation.x = rvec_quaternion_TAG16H5[0]
                        t_16H5.transform.rotation.y = rvec_quaternion_TAG16H5[1]
                        t_16H5.transform.rotation.z = rvec_quaternion_TAG16H5[2]
                        t_16H5.transform.rotation.w = rvec_quaternion_TAG16H5[3]


                        self.tf_broadcaster.sendTransform(t_16H5) #April_tag frame
            
             #7° Parte: Enviando as transformadas

                        # t_16H5_relative_to_36H11 = TransformStamped()
                        # t_16H5_relative_to_36H11.header.stamp = self.get_clock().now().to_msg()
                        # t_16H5_relative_to_36H11.header.frame_id = MY_BASE
                        # t_16H5_relative_to_36H11.child_frame_id = 'apriltag_16H5_relative_to_36H11'
                        # t_16H5_relative_to_36H11.transform.translation.x = float(tvecs_TAG16H5[0]/1000) - float(tvecs_TAG36H11[0]/1000)
                        # t_16H5_relative_to_36H11.transform.translation.y = float(tvecs_TAG16H5[1]/1000) - float(tvecs_TAG36H11[1]/1000)
                        # t_16H5_relative_to_36H11.transform.translation.z = float(tvecs_TAG16H5[2]/1000) - float(tvecs_TAG36H11[2]/1000)
                        # t_16H5_relative_to_36H11.transform.rotation.x = 0.0
                        # t_16H5_relative_to_36H11.transform.rotation.y = 0.0
                        # t_16H5_relative_to_36H11.transform.rotation.z = 0.0
                        # t_16H5_relative_to_36H11.transform.rotation.w = 1.0


                        # self.tf_broadcaster.sendTransform(t_16H5_relative_to_36H11) #April_tag frame


            # Naming a window 
            cv2.namedWindow("Webcam", cv2.WINDOW_NORMAL) 
            # Using resizeWindow() 
            cv2.resizeWindow("Webcam", 900, 900) 
            cv2.imshow("Webcam",undistorted_frame)
            if cv2.waitKey(1) == ord('q'):
                break
    


def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()