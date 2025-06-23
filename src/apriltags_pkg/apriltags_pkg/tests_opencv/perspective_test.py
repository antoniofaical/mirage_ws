import cv2
import os
import random
import numpy as np
from scipy.spatial.transform import Rotation as R

APRILTAG_36h11_HALF_LENGTH=80

def perspective_transformation():

    ##############################################
    # 1° Parte: Abrindo câmera
    stream = cv2.VideoCapture(2) #2
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
        


        imgtrans=undistorted_frame
        corners_g=np.array([[100,100]])
        points=np.array([[100,100]])
        
    #############################################################################
    #############################################################################
    #############################PRIMEIRA APRILTAG###############################


        if ids_TAG36H11 is not None:

            cv2.aruco.drawDetectedMarkers(undistorted_frame, corners_TAG36H11, ids_TAG36H11)

            for i in range(len(ids_TAG36H11)):

                tag_corners = corners_TAG36H11[i][0]  # Obtém os 4 cantos do marcador
                center_x_TAG36H11 = int(np.mean(tag_corners[:, 0]))  # Média das coordenadas x
                center_y_TAG36H11 = int(np.mean(tag_corners[:, 1]))  # Média das coordenadas y
                center_TAG36H11 = (center_x_TAG36H11, center_y_TAG36H11)
                cv2.circle(undistorted_frame, center_TAG36H11, 5, (0, 0, 255), -1)
                corners_matrix = np.array(tag_corners, dtype=np.float32)
                center_matrix=np.array([[center_x_TAG36H11,center_y_TAG36H11]], dtype=np.float32)  # Criação da matriz (4x2)

                if (ids_TAG36H11[i][0]==0):
                    

                    p2=np.array([[270,270],
                                [0,270],
                                [0,0],
                                [270,0]],dtype=np.float32)
                    
                    corners_g=np.concatenate((corners_g,corners_matrix),axis=0)
                    points=np.concatenate((points,p2),axis=0)

                    #T=cv2.getPerspectiveTransform(corners_matrix,p2)
                    #imgtrans=cv2.warpPerspective(imgtrans,T,(900,900))

                if (ids_TAG36H11[i][0]==1):
                    
                    p2=np.array([[270,720],
                                [0,720],
                                [0,450],
                                [270,450]],dtype=np.float32)
                    
                    corners_g=np.concatenate((corners_g,corners_matrix),axis=0)
                    points=np.concatenate((points,p2),axis=0)

                if (ids_TAG36H11[i][0]==2):
                    
                    p2=np.array([[1280,720],
                                [1010,720],
                                [1010,450],
                                [1280,450]],dtype=np.float32)
                    
                    corners_g=np.concatenate((corners_g,corners_matrix),axis=0)
                    points=np.concatenate((points,p2),axis=0)

                if (ids_TAG36H11[i][0]==3):
                    
                    p2=np.array([[1280,270],
                                [1010,270],
                                [1010,0],
                                [1280,0]],dtype=np.float32)
                    
                    corners_g=np.concatenate((corners_g,corners_matrix),axis=0)
                    points=np.concatenate((points,p2),axis=0)


    
                ###########################################
                #Pose estimation
                #COPIAR CODIGO DO DUMMY SE NECESSÁRIO
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

                ###########################################

                ########################################################3
                #MUDANÇA DE PERSPECTIVA

            corners_g=np.delete(corners_g,0,axis=0)
            print(corners_g)
            points=np.delete(points,0,axis=0)    
            print()
            print(points)
            print('AAAAAAA')
            T,_= cv2.findHomography(corners_g, points,cv2.BORDER_CONSTANT)
            h,w=undistorted_frame.shape[:2]
            imgtrans=cv2.warpPerspective(imgtrans,T,(1280,720))


        # Naming a window 
        cv2.namedWindow("Webcam", cv2.WINDOW_NORMAL) 
        # Using resizeWindow() 
        cv2.resizeWindow("Webcam", 900, 900) 
        cv2.namedWindow("AAA", cv2.WINDOW_NORMAL) 
        # Using resizeWindow() 
        cv2.resizeWindow("AAA", 1280, 720) 
        cv2.imshow("Webcam",undistorted_frame)
        cv2.imshow("AAA",imgtrans)
        if cv2.waitKey(1) == ord('q'):
            return




def main():
    perspective_transformation()

if __name__ == '__main__':
    main()