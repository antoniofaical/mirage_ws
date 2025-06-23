import cv2
import numpy as np

##############################################
# 1° Parte: Abrindo câmera
stream = cv2.VideoCapture("/dev/video2")

APRILTAG_36h11_HALF_LENGTH = 28
APRILTAG_TAH16H5_HALF_LENGTH = 55

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
                [-APRILTAG_36h11_HALF_LENGTH, -APRILTAG_36h11_HALF_LENGTH, 0],  # Canto superior esquerdo (em mm)
                [ APRILTAG_36h11_HALF_LENGTH, -APRILTAG_36h11_HALF_LENGTH, 0],  # Canto superior direito
                [ APRILTAG_36h11_HALF_LENGTH,  APRILTAG_36h11_HALF_LENGTH, 0],  # Canto inferior direito
                [-APRILTAG_36h11_HALF_LENGTH,  APRILTAG_36h11_HALF_LENGTH, 0]   # Canto inferior esquerdo
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
#Plotando a distância (Medidas em milímetros)
            print("\n tvecs_TAG36H11:")
            print(tvecs_TAG36H11[2])
            distance=str(tvecs_TAG36H11[2])
            cv2.putText(undistorted_frame, distance, (0, 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            #Coordenadas X e Y
            #X
            print(tvecs_TAG36H11[0])
            Coord_x_TAG36H11=str(tvecs_TAG36H11[0])
            cv2.putText(undistorted_frame, Coord_x_TAG36H11, (0, 80),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            #Y
            print(tvecs_TAG36H11[1])
            Coord_y_TAG36H11=str(tvecs_TAG36H11[1])
            cv2.putText(undistorted_frame, Coord_y_TAG36H11, (0, 60 ),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


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

            axis_TAG16H5 = np.float32([[-APRILTAG_TAH16H5_HALF_LENGTH,0,0], [0,-APRILTAG_TAH16H5_HALF_LENGTH,0], [0,0,-APRILTAG_TAH16H5_HALF_LENGTH]]).reshape(-1,3) #Tamanho do eixo que será plotado
            td_points_16H5,_=cv2.projectPoints(axis_TAG16H5, rvecs_TAG16H5, tvecs_TAG16H5, camera_matrix, dist_coeffs) #Projetando os pontos
            point_xf_TAG16H5=(int(td_points_16H5[0][0][0]), int(td_points_16H5[0][0][1])) #Definição do eixo x
            cv2.line(undistorted_frame,center_TAG16H5, point_xf_TAG16H5, (0, 0, 255), 2) #Desenhando eixo x
            point_yf_TAG16H5=(int(td_points_16H5[1][0][0]), int(td_points_16H5[1][0][1])) #Definição do eixo y
            cv2.line(undistorted_frame,center_TAG16H5, point_yf_TAG16H5, (0, 255, 0), 2) #Desenhando eixo y
            point_zf_TAG16H5=(int(td_points_16H5[2][0][0]),int(td_points_16H5[2][0][1])) #Definição do eixo z
            cv2.line(undistorted_frame,center_TAG16H5, point_zf_TAG16H5, (255, 0, 0), 2) #Desenhando eixo z

#Plotando a distância (Medidas em milímetros)
            print("\ntvecs_TAG16H5:")
            print(tvecs_TAG16H5[2])
            distance=str(tvecs_TAG16H5[2])
            cv2.putText(undistorted_frame, distance, (0, 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            #Coordenadas X e Y
            #X
            print(tvecs_TAG16H5[0])
            Coord_x_TAG36H11=str(tvecs_TAG16H5[0])
            cv2.putText(undistorted_frame, Coord_x_TAG36H11, (0, 80),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            #Y
            print(tvecs_TAG16H5[1])
            Coord_y_TAG36H11=str(tvecs_TAG16H5[1])
            cv2.putText(undistorted_frame, Coord_y_TAG36H11, (0, 60 ),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


            print("delta_z = ", tvecs_TAG16H5[2] - tvecs_TAG36H11[2])
            print("delta_x = ", tvecs_TAG16H5[0] - tvecs_TAG36H11[0])
            print("delta_y = ", tvecs_TAG16H5[1] - tvecs_TAG36H11[1])


    cv2.imshow("Webcam",undistorted_frame)
    if cv2.waitKey(1) == ord('q'):
        break

stream.release()
cv2.destroyAllWindows()