import cv2
import numpy as np

##############################################
# 1° Parte: Abrindo câmera
stream = cv2.VideoCapture("/dev/video2")

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

    april_detec = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters_create()
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

            ret,rvecs, tvecs = cv2.solvePnP(objectPoints, corners2, camera_matrix, dist_coeffs, cv2.SOLVEPNP_IPPE_SQUARE)
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
#6° Parte: Plotando a distância (Medidas em milímetros)
            print(tvecs[2])
            distance=str(tvecs[2])
            cv2.putText(undistorted_frame, distance, (0, 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            #Coordenadas X e Y
            #X
            print(tvecs[0])
            Coord_x=str(tvecs[0])
            cv2.putText(undistorted_frame, Coord_x, (0, 80),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            #Y
            print(tvecs[1])
            Coord_y=str(tvecs[1])
            cv2.putText(undistorted_frame, Coord_y, (0, 60 ),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)



    cv2.imshow("Webcam",undistorted_frame)
    if cv2.waitKey(1) == ord('q'):
        break

stream.release()
cv2.destroyAllWindows()