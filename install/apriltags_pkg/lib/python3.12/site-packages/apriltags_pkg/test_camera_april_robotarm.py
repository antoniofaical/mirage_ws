from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster,TransformListener

import cv2
import numpy as np


class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)


    def broadcast_timer_callback(self):
        ##############################################
        # 1° Parte: Abrindo câmera
        stream = cv2.VideoCapture(0)

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

            camera_matrix = np.array([[1002.72, 0, 618.73], 
                                    [0, 1008.33, 288.62], 
                                    [0, 0, 1]], np.float32) 
            dist_coeffs = np.array([[-0.105508, 0.40147, -0.024952, 0.00550041, -0.81168]], np.float32)

            undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
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
                        print(f"ID detectado: {ids_TAG36H11[i][0]}")
                        tag_corners = corners_TAG36H11[i][0]  # Obtém os 4 cantos do marcador
                        center_x_TAG36H11 = int(np.mean(tag_corners[:, 0]))  # Média das coordenadas x
                        center_y_TAG36H11 = int(np.mean(tag_corners[:, 1]))  # Média das coordenadas y
                        center_TAG36H11 = (center_x_TAG36H11, center_y_TAG36H11)
                        cv2.circle(undistorted_frame, center_TAG36H11, 5, (0, 0, 255), -1)

            ###########################################
            #Pose estimation

                        objectPoints_TAG36H11 = np.array([
                            [-28, -28, 0],  # Canto superior esquerdo (em mm)
                            [ 28, -28, 0],  # Canto superior direito
                            [ 28,  28, 0],  # Canto inferior direito
                            [-28,  28, 0]   # Canto inferior esquerdo
                        ], dtype=np.float32)

                        corners2_TAG36H11=corners_TAG36H11[i][0]

                        ret_TAG36H11,rvecs_TAG36H11, tvecs_TAG36H11 = cv2.solvePnP(objectPoints_TAG36H11, corners2_TAG36H11, camera_matrix, dist_coeffs)
            ##############################################
            #Desenhando frame
                        axis_TAG36H11 = np.float32([[-28,0,0], [0,-28,0], [0,0,-28]]).reshape(-1,3) #Tamanho do eixo que será plotado
                        td_points_36H11,_=cv2.projectPoints(axis_TAG36H11, rvecs_TAG36H11, tvecs_TAG36H11, camera_matrix, dist_coeffs) #Projetando os pontos
                        point_xf_TAG36H11=(int(td_points_36H11[0][0][0]), int(td_points_36H11[0][0][1])) #Definição do eixo x
                        cv2.line(undistorted_frame,center_TAG36H11, point_xf_TAG36H11, (0, 0, 255), 2) #Desenhando eixo x
                        point_yf_TAG36H11=(int(td_points_36H11[1][0][0]), int(td_points_36H11[1][0][1])) #Definição do eixo y
                        cv2.line(undistorted_frame,center_TAG36H11, point_yf_TAG36H11, (0, 255, 0), 2) #Desenhando eixo y
                        point_zf_TAG36H11=(int(td_points_36H11[2][0][0]),int(td_points_36H11[2][0][1])) #Definição do eixo z
                        cv2.line(undistorted_frame,center_TAG36H11, point_zf_TAG36H11, (255, 0, 0), 2) #Desenhando eixo z

##############################################
#####################################################################################
#####################################################################################
############################PLOT FRAME###############################################
            #7° Parte: Enviando o frame da APRILTAG já conhecida (TAGH36H11)

                        t_TAG36H11 = TransformStamped()
                        t_TAG36H11.header.stamp = self.get_clock().now().to_msg()
                        t_TAG36H11.header.frame_id = 'world'
                        t_TAG36H11.child_frame_id = 'apriltag_TAG36H11'
                        t_TAG36H11.transform.translation.x = 1.5 #MUDAR CONFORME BANCADA
                        t_TAG36H11.transform.translation.y = 1.5 #MUDAR CONFORME BANCADA
                        t_TAG36H11.transform.translation.z = 1.0 #MUDAR CONFORME BANCADA
                        t_TAG36H11.transform.rotation.x = 0.0
                        t_TAG36H11.transform.rotation.y = 0.0
                        t_TAG36H11.transform.rotation.z = 0.0
                        t_TAG36H11.transform.rotation.w = 1.0


                        self.tf_broadcaster.sendTransform(t_TAG36H11) #April_tag frame

            #8° Determinando a posição da Câmera: MUDAR CONFORME A CÂMERA (MUDAR DEPOIS OS EIXOS por causa da rotação)
            # O eixo da câmera corresponde a:
            # Zrviz2=-Ycamera ; Yrviz2= -Zcamera; Xrviz2= Xcamera
                        t_camera = TransformStamped ()
                        t_camera.header.stamp= self.get_clock().now().to_msg()
                        t_camera.header.frame_id= 'apriltag_TAG36H11'
                        t_camera.child_frame_id= 'camera'
                        # Multiplicar por menos para determinar a posição da câmera
                        # Não propriamente da Apriltag, já conhecida.
                        t_camera.transform.translation.x=float((-1)*(tvecs_TAG36H11[0]/1000))
                        t_camera.transform.translation.y=float((tvecs_TAG36H11[2]/1000))
                        t_camera.transform.translation.z=float((tvecs_TAG36H11[1]/1000))
                        t_camera.transform.rotation.x= 0.0
                        t_camera.transform.rotation.y= 0.0
                        t_camera.transform.rotation.z= 0.0
                        t_camera.transform.rotation.w= 1.0

                        self.tf_broadcaster.sendTransform(t_camera)

        #############################################################################
        #############################################################################
        #############################SEGUNDA APRILTAG###############################


            if ids_TAH16H5 is not None:
            
                cv2.aruco.drawDetectedMarkers(undistorted_frame, corners_TAG16H5, ids_TAH16H5)

                for i in range(len(ids_TAH16H5)):
                        print(f"ID detectado: {ids_TAH16H5[i][0]}")

                        tag_corners_TAG16H5 = corners_TAG16H5[i][0]  # Obtém os 4 cantos do marcador
                        center_x_TAG16H5 = int(np.mean(tag_corners_TAG16H5[:, 0]))  # Média das coordenadas x
                        center_y_TAG16H5 = int(np.mean(tag_corners_TAG16H5[:, 1]))  # Média das coordenadas y
                        center_TAG16H5 = (center_x_TAG16H5, center_y_TAG16H5)
                        cv2.circle(undistorted_frame, center_TAG16H5, 5, (0, 0, 255), -1)

            ################################################################
            #Pose estimation

                        objectPoints_TAG16H5 = np.array([
                            [-55, -55, 0],  # Canto superior esquerdo (em mm)
                            [ 55, -55, 0],  # Canto superior direito
                            [ 55,  55, 0],  # Canto inferior direito
                            [-55,  55, 0]   # Canto inferior esquerdo
                        ], dtype=np.float32)

                        corners2_TAG16H5=corners_TAG16H5[i][0]

                        ret_TAG16H5,rvecs_TAG16H5, tvecs_TAG16H5 = cv2.solvePnP(objectPoints_TAG16H5, corners2_TAG16H5, camera_matrix, dist_coeffs)
            ##############################################
            #Desenhando frame

                        axis_TAG16H5 = np.float32([[-28,0,0], [0,-28,0], [0,0,-28]]).reshape(-1,3) #Tamanho do eixo que será plotado
                        td_points_16H5,_=cv2.projectPoints(axis_TAG16H5, rvecs_TAG16H5, tvecs_TAG16H5, camera_matrix, dist_coeffs) #Projetando os pontos
                        point_xf_TAG16H5=(int(td_points_16H5[0][0][0]), int(td_points_16H5[0][0][1])) #Definição do eixo x
                        cv2.line(undistorted_frame,center_TAG16H5, point_xf_TAG16H5, (0, 0, 255), 2) #Desenhando eixo x
                        point_yf_TAG16H5=(int(td_points_16H5[1][0][0]), int(td_points_16H5[1][0][1])) #Definição do eixo y
                        cv2.line(undistorted_frame,center_TAG16H5, point_yf_TAG16H5, (0, 255, 0), 2) #Desenhando eixo y
                        point_zf_TAG16H5=(int(td_points_16H5[2][0][0]),int(td_points_16H5[2][0][1])) #Definição do eixo z
                        cv2.line(undistorted_frame,center_TAG16H5, point_zf_TAG16H5, (255, 0, 0), 2) #Desenhando eixo z
##############################################
#####################################################################################
#####################################################################################
############################PLOT FRAME###############################################
            #7° Parte: Enviando as transformadas: MUDAR CONFORME A CÂMERA
            #REALIZAR AS MESMAS MODIFICAÇÃOES DA TAG36H11

                        t_16H5 = TransformStamped()
                        t_16H5.header.stamp = self.get_clock().now().to_msg()
                        t_16H5.header.frame_id = 'camera'
                        t_16H5.child_frame_id = 'apriltag_TAG16H5'
                        t_16H5.transform.translation.x = float(tvecs_TAG16H5[0]/1000)
                        t_16H5.transform.translation.y = float((-1)*tvecs_TAG16H5[2]/1000)
                        t_16H5.transform.translation.z = float((-1)*tvecs_TAG16H5[1]/1000)
                        t_16H5.transform.rotation.x = 0.0
                        t_16H5.transform.rotation.y = 0.0
                        t_16H5.transform.rotation.z = 0.0
                        t_16H5.transform.rotation.w = 1.0


                        self.tf_broadcaster.sendTransform(t_16H5) #April_tag frame



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