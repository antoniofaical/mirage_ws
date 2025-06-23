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

#####################################################################################
#####################################################################################
############################POSE ESTIMATION##########################################
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

            april_detec = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            parameters = cv2.aruco.DetectorParameters()
            corners, ids, _ = cv2.aruco.detectMarkers(undistorted_april, april_detec, parameters=parameters)
        ############################################
        #3° Parte: Desenhando a bounding box e centro

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(undistorted_frame, corners, ids)

                for i in range(len(ids)):
                        print(f"ID detectado: {ids[i][0]}")
                        tag_corners = corners[i][0]  # Obtém os 4 cantos do marcador
                        center_x = int(np.mean(tag_corners[:, 0]))  # Média das coordenadas x
                        center_y = int(np.mean(tag_corners[:, 1]))  # Média das coordenadas y
                        center = (center_x, center_y)
                        cv2.circle(undistorted_frame, center, 5, (0, 0, 255), -1)
                        print(corners)
            ###########################################
            #4° Parte: Pose estimation

                        objectPoints = np.array([
                            [-28, -28, 0],  # Canto superior esquerdo (em mm)
                            [ 28, -28, 0],  # Canto superior direito
                            [ 28,  28, 0],  # Canto inferior direito
                            [-28,  28, 0]   # Canto inferior esquerdo
                        ], dtype=np.float32)

                        corners2=corners[i][0]

                        ret,rvecs, tvecs = cv2.solvePnP(objectPoints, corners2, camera_matrix, dist_coeffs)
            ##############################################
            #5° Parte: Desenhando frame
                        axis = np.float32([[-28,0,0], [0,-28,0], [0,0,-28]]).reshape(-1,3) #Tamanho do eixo que será plotado
                        td_points,_=cv2.projectPoints(axis, rvecs, tvecs, camera_matrix, dist_coeffs) #Projetando os pontos
                        point_xf=(int(td_points[0][0][0]), int(td_points[0][0][1])) #Definição do eixo x
                        cv2.line(undistorted_frame,center, point_xf, (0, 0, 255), 2) #Desenhando eixo x
                        point_yf=(int(td_points[1][0][0]), int(td_points[1][0][1])) #Definição do eixo y
                        cv2.line(undistorted_frame,center, point_yf, (0, 255, 0), 2) #Desenhando eixo y
                        point_zf=(int(td_points[2][0][0]),int(td_points[2][0][1])) #Definição do eixo z
                        cv2.line(undistorted_frame,center, point_zf, (255, 0, 0), 2) #Desenhando eixo z
                        
##############################################
#####################################################################################
#####################################################################################
############################PLOT FRAME###############################################
            #7° Parte: Enviando as transformadas

                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = 'world'
                        t.child_frame_id = 'apriltag_TAG36H11'
                        t.transform.translation.x = float(tvecs[0]/1000)
                        t.transform.translation.y = float(tvecs[1]/1000)
                        t.transform.translation.z = float(tvecs[2]/1000)
                        t.transform.rotation.x = 0.0
                        t.transform.rotation.y = 0.0
                        t.transform.rotation.z = 0.0
                        t.transform.rotation.w = 1.0


                        self.tf_broadcaster.sendTransform(t) #April_tag frame



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